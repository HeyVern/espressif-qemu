/* Strict receive: a frame is received only if the receiver was listening for
 * it, from its preamble to its end.
 *
 * A firmware that re-arms its receiver part-way through a frame, or runs a CAD
 * scan while one is arriving, loses that frame on silicon. Without strict
 * receive the inbox grace hands it over anyway, which hides exactly the
 * firmware faults this mode exists to show.
 */
#include "harness.h"

static const uint8_t kFrame[] = {0x11, 0x00, 0x33, 0x0a, 0xc2, 0x1f, 0x44, 0x55,
                                 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd};

static inline void set_rx(vsx_chip* c) {
  const uint8_t rx[] = {0x82, 0xFF, 0xFF, 0xFF};
  vsx_spi_transaction(c, rx, nullptr, sizeof(rx));
}

/* The air carries one frame for its airtime. `rearmAt` is the millisecond
 * into it at which the firmware goes to standby and straight back to receive,
 * or -1 for never; `rxAt` is when the receiver first enters receive, 0 for
 * already listening. Returns the flags the chip ends with. */
static uint16_t one_frame(vsx_chip* c, uint64_t* now, int rearmAt, int rxAt) {
  const uint32_t air = vsx_est_airtime_ms(c, (int)sizeof(kFrame));
  if (rxAt > 0) {
    set_standby(c);
  }
  vsx_set_channel_busy(c, 1);
  for (uint32_t ms = 1; ms <= air; ++ms) {
    vsx_tick(c, ++*now);
    if ((int)ms == rearmAt) {
      set_standby(c);
      set_rx(c);
    }
    if ((int)ms == rxAt) {
      set_rx(c);
    }
  }
  vsx_deliver_frame(c, kFrame, sizeof(kFrame));
  vsx_set_channel_busy(c, 0);
  vsx_tick(c, ++*now);
  return irq_flags(c);
}

int main() {
  std::printf("virtual-sx1262");

  /* ---------------------------------------------------------------- */
  CASE("without strict receive, a re-arm mid-frame still delivers it");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint16_t f = one_frame(c, &now, 150, -1);
    check((f & IRQ_RX_DONE) != 0, "the inbox grace hands the frame over");
    check(vsx_frames_aborted(c) == 0, "and nothing is counted as aborted");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("with strict receive, a receiver that listened throughout gets the frame");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    vsx_set_strict_receive(c, 1);
    const uint16_t f = one_frame(c, &now, -1, 0);
    check((f & IRQ_RX_DONE) != 0, "RxDone");
    check(vsx_frames_aborted(c) == 0, "nothing aborted");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a re-arm part-way through the payload loses the frame");
  /* SetStandby aborts a reception. SetRx afterwards searches for a new
   * preamble, and the one this frame had is long gone. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    vsx_set_strict_receive(c, 1);
    const uint16_t f = one_frame(c, &now, 150, -1);
    check((f & IRQ_RX_DONE) == 0, "no RxDone");
    check(vsx_frames_aborted(c) == 1, "one frame aborted");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a re-arm early in the preamble still locks on");
  /* Sixteen preamble symbols of 4.1 ms each, and the detector needs four of
   * them: a receiver back in receive 10 ms in has plenty left. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    vsx_set_strict_receive(c, 1);
    const uint16_t f = one_frame(c, &now, 10, -1);
    check((f & IRQ_RX_DONE) != 0, "RxDone");
    check(vsx_frames_aborted(c) == 0, "nothing aborted");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a receiver that starts listening after the preamble hears no preamble");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    vsx_set_strict_receive(c, 1);
    const uint32_t air = vsx_est_airtime_ms(c, (int)sizeof(kFrame));
    set_standby(c);
    vsx_set_channel_busy(c, 1);
    for (uint32_t ms = 1; ms <= air; ++ms) {
      vsx_tick(c, ++now);
      if (ms == 120) {
        set_rx(c);
      }
    }
    const uint16_t during = irq_flags(c);
    check((during & IRQ_PREAMBLE) == 0, "PreambleDetected stays clear");
    check((during & IRQ_HEADER) == 0, "and so does HeaderValid");
    vsx_deliver_frame(c, kFrame, sizeof(kFrame));
    vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_RX_DONE) == 0, "and the frame is not received");
    check(vsx_frames_aborted(c) == 1, "one frame aborted");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("strict detection flags outlive the carrier, as on silicon");
  /* The part has no interrupt for a preamble that came to nothing, so a flag
   * raised for a frame that never finished stays until the firmware clears it.
   * That stale flag is what a driver's own timeouts exist to handle. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    vsx_set_strict_receive(c, 1);
    vsx_set_channel_busy(c, 1);
    for (int i = 0; i < 200; ++i) {
      vsx_tick(c, ++now);
    }
    vsx_set_channel_busy(c, 0);
    vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_PREAMBLE) != 0, "PreambleDetected is still set");
    check((irq_flags(c) & IRQ_HEADER) != 0, "and so is HeaderValid");

    vsx_deliver_frame_failed(c, kFrame, sizeof(kFrame), 3);
    check((irq_flags(c) & IRQ_HEADER) == 0,
          "a frame never locked onto withdraws HeaderValid");
    check((irq_flags(c) & IRQ_PREAMBLE) != 0, "and leaves PreambleDetected latched");
    check((irq_flags(c) & IRQ_RX_DONE) == 0, "with no RxDone");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("without strict receive, a frame never locked onto changes nothing");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    vsx_set_channel_busy(c, 1);
    for (int i = 0; i < 200; ++i) {
      vsx_tick(c, ++now);
    }
    vsx_deliver_frame_failed(c, kFrame, sizeof(kFrame), 3);
    check((irq_flags(c) & IRQ_HEADER) != 0, "HeaderValid is untouched");
    check((irq_flags(c) & IRQ_CRC_ERR) == 0, "and it is not read as a CRC error");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("clearing the flags mid-frame does not detect the same preamble again");
  /* A firmware that times a flag out and clears it while the frame is still on
   * the air must see the flag stay down: the preamble has gone by, and the
   * part has nothing to detect until the next one. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    vsx_set_strict_receive(c, 1);
    vsx_set_channel_busy(c, 1);
    for (int i = 0; i < 150; ++i) {
      vsx_tick(c, ++now);
    }
    check((irq_flags(c) & (IRQ_PREAMBLE | IRQ_HEADER)) == (IRQ_PREAMBLE | IRQ_HEADER),
          "both flags are up part-way through the frame");
    const uint8_t clr[] = {0x02, 0xFF, 0xFF};
    vsx_spi_transaction(c, clr, nullptr, sizeof(clr));
    for (int i = 0; i < 100; ++i) {
      vsx_tick(c, ++now);
    }
    check((irq_flags(c) & (IRQ_PREAMBLE | IRQ_HEADER)) == 0,
          "and stay down for the rest of the same carrier");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a header error withdraws the HeaderValid its carrier raised");
  /* Silicon never reports both: a header that failed was never valid. Left
   * up, RadioLib reads HeaderErr beside it as a good packet and returns
   * whatever the buffer last held. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    vsx_set_strict_receive(c, 1);
    vsx_set_channel_busy(c, 1);
    for (int i = 0; i < 200; i++) vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_HEADER) != 0, "the carrier raised HeaderValid");
    vsx_deliver_frame_failed(c, kFrame, sizeof(kFrame), 2);
    const uint16_t f = irq_flags(c);
    check((f & IRQ_HEADER_ERR) != 0 && (f & IRQ_HEADER) == 0,
          "HeaderErr is raised and HeaderValid is withdrawn");
    vsx_destroy(c);
  }

  return report("");
}
