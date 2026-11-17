/* What the chip does with a signal: when a carrier counts as present, when a
 * frame is receivable and when it is simply too late, and how long a
 * transmission occupies the air.
 *
 * Every case here is a bug that reached a four-minute emulator run before this
 * library had a bench of its own. The names say what went wrong, not what the
 * function is called.
 */
#include "harness.h"

int main() {
  std::printf("virtual-sx1262");

  /* ---------------------------------------------------------------- */
  CASE("a carrier that has ended is not still 'present'");
  /* The bug: deliverPending raised PreambleDetected|HeaderValid|SyncWord at
   * RxDone - asserting that a signal was arriving at the instant it stopped.
   * MeshCore's CustomSX1262::isReceiving reads HEADER_VALID as "the channel is
   * busy", so a repeater with a packet to forward never saw a clear channel
   * and relayed nothing. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);

    /* A transmission arrives and runs for a while, then ends and is
     * delivered - which is the order the simulator uses. */
    vsx_set_channel_busy(c, 1);
    for (int i = 0; i < 200; ++i) {
      vsx_tick(c, ++now);
    }
    check((irq_flags(c) & IRQ_HEADER) != 0,
          "while the carrier is on the air, HeaderValid is set");
    check((irq_flags(c) & IRQ_SYNC_WORD) == 0,
          "and SyncWordValid is not: that is the FSK modem's, and this is LoRa");

    const uint8_t frame[] = {0x11, 0x00, 0x33, 0x0a, 0xc2, 0x1f};
    vsx_deliver_frame(c, frame, sizeof(frame));
    vsx_set_channel_busy(c, 0);
    vsx_tick(c, ++now);

    const uint16_t f = irq_flags(c);
    check((f & IRQ_RX_DONE) != 0, "the delivered packet raises RxDone");
    check((f & IRQ_HEADER) == 0, "HeaderValid is clear once the carrier has gone");
    check((f & IRQ_PREAMBLE) == 0, "PreambleDetected is clear too");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a packet handed to a deaf chip does not turn up later");
  /* The bug: a frame delivered while the chip was not listening sat in the
   * inbox until it started listening - however long that took - so a packet
   * could arrive seconds after the air that carried it went quiet. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);

    set_standby(c);
    vsx_tick(c, ++now);
    check(mode_of(c) == 0, "the chip is in standby, not listening");

    const uint8_t frame[] = {0x11, 0x00, 0xde, 0xad};
    vsx_deliver_frame(c, frame, sizeof(frame));

    /* Far longer than any re-arm, well short of the seconds this used to
     * survive. */
    for (int i = 0; i < 500; ++i) {
      vsx_tick(c, ++now);
    }

    /* Now it starts listening again. The stale frame must not appear. */
    const uint8_t rx[] = {0x82, 0xFF, 0xFF, 0xFF};
    vsx_spi_transaction(c, rx, nullptr, sizeof(rx));
    for (int i = 0; i < 10; ++i) {
      vsx_tick(c, ++now);
    }

    vsx_counters k;
    vsx_get_counters(c, &k);
    check((irq_flags(c) & IRQ_RX_DONE) == 0,
          "the stale packet is not delivered when the chip returns to receive");
    check(k.frames_dropped == 1, "it is counted as dropped, not silently lost");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a brief re-arm gap still receives");
  /* The other half of that fix: the driver's own startReceive is
   * standby -> configure -> SetRx, so mode is briefly 0 in a perfectly
   * healthy receiver. A frame landing in that gap is receivable, and an
   * immediate drop would have lost it. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);

    set_standby(c);
    const uint8_t frame[] = {0x11, 0x00, 0xbe, 0xef};
    vsx_deliver_frame(c, frame, sizeof(frame));

    vsx_tick(c, ++now); /* one millisecond of re-arm */
    const uint8_t rx[] = {0x82, 0xFF, 0xFF, 0xFF};
    vsx_spi_transaction(c, rx, nullptr, sizeof(rx));
    vsx_tick(c, ++now);

    vsx_counters k;
    vsx_get_counters(c, &k);
    check((irq_flags(c) & IRQ_RX_DONE) != 0,
          "a frame that arrived during the gap is delivered");
    check(k.frames_dropped == 0, "and nothing was dropped");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("the wait a frame gets is the time its signal was on the air");
  /* The bound on that wait started life as a fixed 10 ms and that was the wrong
   * shape: the hosts advance this clock at their own rate, so a constant that
   * is generous against one is tight against another and drops a frame that was
   * genuinely receivable. It is derived from the frame's own airtime now. This
   * pins that, because a regression to a small constant passes both tests above
   * and quietly loses packets in the host that matters. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);

    /* A full-length advert: hundreds of milliseconds on the air at SF8. */
    uint8_t frame[119];
    std::memset(frame, 0xA5, sizeof(frame));
    const uint32_t air = vsx_est_airtime_ms(c, (int)sizeof(frame));
    check(air > 100, "a 119-byte frame at SF8 is hundreds of ms on the air");

    set_standby(c);
    vsx_deliver_frame(c, frame, sizeof(frame));

    /* Well past any fixed small constant, still inside the airtime. */
    for (uint32_t i = 0; i < air / 2; ++i) {
      vsx_tick(c, ++now);
    }
    vsx_counters k;
    vsx_get_counters(c, &k);
    check(k.frames_dropped == 0, "half an airtime into a re-arm it is still held");

    const uint8_t rx[] = {0x82, 0xFF, 0xFF, 0xFF};
    vsx_spi_transaction(c, rx, nullptr, sizeof(rx));
    vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_RX_DONE) != 0,
          "and it is delivered when listening resumes");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("arming the receiver delivers, without waiting to be ticked");
  /* Delivery used to happen only on a tick, which made every reception depend
   * on a tick landing after the firmware armed the receiver. A host that ticks
   * in its own process nearly always got one; a host whose ticks arrive from a
   * simulator over a socket did not, and the frame went in the bin. Same model,
   * same calls, different answer depending on the plumbing. No tick here on
   * purpose: the SetRx alone has to be enough. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);

    set_standby(c);
    const uint8_t frame[] = {0x11, 0x00, 0x5A, 0x5A};
    vsx_deliver_frame(c, frame, sizeof(frame));
    vsx_tick(c, ++now); /* the engine's last tick, while the chip is still deaf */
    check((irq_flags(c) & IRQ_RX_DONE) == 0, "nothing arrives while it is deaf");

    const uint8_t rx[] = {0x82, 0xFF, 0xFF, 0xFF};
    vsx_spi_transaction(c, rx, nullptr, sizeof(rx));
    check((irq_flags(c) & IRQ_RX_DONE) != 0, "the SetRx alone hands the frame over");
    check(vsx_dio1_asserted(c) == 1, "and DIO1 is up in that same transaction");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("airtime is the chip's answer, not the host's");
  /* MeshCore's listen-before-talk is built on the firmware's own
   * getEstAirtimeFor, so a host must be able to ask rather than reimplement
   * Semtech's formula beside it and drift. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint32_t a = vsx_est_airtime_ms(c, 119);
    const uint32_t b = vsx_est_airtime_ms(c, 238);
    check(a > 0, "a 119-byte frame has an airtime");
    check(b > a, "a longer frame takes longer");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a transmitted frame comes back out once");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);

    const uint8_t payload[] = {0x11, 0x00, 0xaa, 0xbb, 0xcc};
    uint8_t wr[3 + sizeof(payload)] = {0x0E, 0x00, 0};
    std::memcpy(wr + 2, payload, sizeof(payload));
    vsx_spi_transaction(c, wr, nullptr, 2 + sizeof(payload));

    const uint8_t pp[] = {0x8C, 0x00, 0x08, 0x00, (uint8_t)sizeof(payload),
                          0x01, 0x00, 0x00, 0x00};
    vsx_spi_transaction(c, pp, nullptr, sizeof(pp));
    const uint8_t tx[] = {0x83, 0, 0, 0};
    vsx_spi_transaction(c, tx, nullptr, sizeof(tx));

    uint8_t got[64] = {0};
    const size_t n = vsx_take_tx(c, got, sizeof(got));
    check(n == sizeof(payload), "the frame the firmware wrote is the frame that leaves");
    check(std::memcmp(got, payload, sizeof(payload)) == 0, "and its bytes are unchanged");
    check(vsx_take_tx(c, got, sizeof(got)) == 0, "it is not handed out twice");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("HeaderValid waits for the preamble, the SFD and the header block");
  /* It was raised twelve symbols into the carrier whatever the preamble, so for
   * the chip's default sixteen-symbol preamble it came 16.25 symbols early. A
   * firmware that sees preambles and one that does not answer differently only
   * between PreambleDetected and HeaderValid, and that stretch was a third of
   * its real width. At SF8 and 62.5 kHz a symbol is 4.096 ms, so the header ends
   * (16 + 4.25 + 8) symbols, about 115.7 ms, into the frame. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    vsx_set_channel_busy(c, 1);
    for (int i = 0; i < 100; ++i) {
      vsx_tick(c, ++now);
    }
    check((irq_flags(c) & IRQ_PREAMBLE) != 0, "the preamble is detected early on");
    check((irq_flags(c) & IRQ_HEADER) == 0,
          "but 100 ms in, the header has not ended yet");
    for (int i = 0; i < 20; ++i) {
      vsx_tick(c, ++now);
    }
    check((irq_flags(c) & IRQ_HEADER) != 0, "and 120 ms in, it has");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("an implicit-header receiver never raises HeaderValid");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    /* SetPacketParams: preamble 16, implicit header, length 32, CRC on. */
    const uint8_t pp[] = {0x8C, 0x00, 0x10, 0x01, 0x20, 0x01, 0x00, 0x00, 0x00};
    vsx_spi_transaction(c, pp, nullptr, sizeof(pp));
    vsx_set_channel_busy(c, 1);
    for (int i = 0; i < 300; ++i) {
      vsx_tick(c, ++now);
    }
    check((irq_flags(c) & IRQ_PREAMBLE) != 0, "the preamble is still detected");
    check((irq_flags(c) & IRQ_HEADER) == 0, "but there is no header to validate");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("the four-byte GetIrqStatus RadioLib sends gets both flag bytes");
  /* The low byte answered only a five-byte read, so RadioLib's readData never
   * saw CrcErr and a firmware forwarded a frame that failed its CRC. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t frame[] = {0x15, 0x00, 0x34, 0xb8, 0x2d, 0xb7};
    vsx_deliver_frame_failed(c, frame, sizeof(frame), 1);
    vsx_tick(c, ++now);
    const uint8_t tx[] = {0x12, 0, 0, 0};
    uint8_t rx[sizeof(tx)] = {0};
    vsx_spi_transaction(c, tx, rx, sizeof(tx));
    const uint16_t flags = (uint16_t)((rx[2] << 8) | rx[3]);
    check((flags & IRQ_CRC_ERR) != 0, "CrcErr is in the low byte of a four-byte read");
    check((flags & IRQ_RX_DONE) != 0, "and so is RxDone");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("the received header's coding rate and CRC flag can be read back");
  /* RadioLib's getLoRaRxHeaderInfo() and Semtech's own driver read these two
   * registers to learn what the transmitter's header said. Left at zero, every
   * frame read as sent without a CRC, and a driver that checks would have
   * dropped them all. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t frame[] = {0x15, 0x00, 0x34, 0xb8, 0x2d, 0xb7};
    vsx_deliver_frame(c, frame, sizeof(frame));
    vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_RX_DONE) != 0, "the frame is received");
    check((read_reg(c, 0x076B) & 0x10) != 0,
          "0x076B bit 4 says the header carried a CRC");
    check(((read_reg(c, 0x0749) >> 4) & 0x07) == 4,
          "0x0749 bits 6:4 give coding rate 4/8");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a header that says no CRC delivers the frame unchecked, and says so");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t frame[] = {0x15, 0x00, 0x34, 0xb8, 0x2d, 0xb7};
    vsx_deliver_frame_failed(c, frame, sizeof(frame), 4);
    vsx_tick(c, ++now);
    const uint16_t f = irq_flags(c);
    check((f & IRQ_RX_DONE) != 0, "RxDone is raised");
    check((f & IRQ_CRC_ERR) == 0, "and CrcErr is not: there was nothing to check");
    check((read_reg(c, 0x076B) & 0x10) == 0, "0x076B bit 4 is clear");
    uint8_t len = 0, start = 0, got[sizeof(frame)] = {0};
    rx_buffer_status(c, &len, &start);
    read_buffer(c, start, got, sizeof(got));
    check(len == sizeof(frame) && std::memcmp(got, frame, sizeof(frame)) == 0,
          "the bytes are in the buffer as they arrived");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("single-shot receive goes back to standby after a packet");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t single[] = {0x82, 0x00, 0x00, 0x00};
    vsx_spi_transaction(c, single, nullptr, sizeof(single));
    vsx_tick(c, ++now);
    const uint8_t frame[] = {0x15, 0x00, 0x34, 0xb8};
    vsx_deliver_frame(c, frame, sizeof(frame));
    vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_RX_DONE) != 0, "the packet is received");
    check(mode_of(c) == 0, "and the chip is in standby, not still listening");
    vsx_destroy(c);
  }

  CASE("continuous receive keeps listening after a packet");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t frame[] = {0x15, 0x00, 0x34, 0xb8};
    vsx_deliver_frame(c, frame, sizeof(frame));
    vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_RX_DONE) != 0, "the packet is received");
    check(mode_of(c) == 1, "and the chip is still receiving");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("the next frame's header before the read: its length, the last frame's start");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t a[] = {0x01, 0x00, 0xAA, 0xBB, 0xCC};
    const uint8_t b[] = {0x01, 0x01, 0x91, 0xAA, 0xBB, 0xCC};
    vsx_deliver_frame(c, a, sizeof(a));
    vsx_deliver_frame_failed(c, b, sizeof(b), 5);
    vsx_tick(c, ++now);
    uint8_t len = 0, start = 0, got[sizeof(b)] = {0};
    rx_buffer_status(c, &len, &start);
    read_buffer(c, start, got, len);
    check(len == sizeof(b), "the length is the next frame's");
    check(std::memcmp(got, a, sizeof(a)) == 0 && got[sizeof(a)] == b[0],
          "and reading it from the start gives this frame and the next one's first byte");
    vsx_destroy(c);
  }

  CASE(
      "the next frame's header during the read: the length changes after the first "
      "status read");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t a[] = {0x01, 0x00, 0xAA, 0xBB, 0xCC};
    const uint8_t b[] = {0x01, 0x01, 0x91, 0xAA, 0xBB, 0xCC};
    vsx_deliver_frame(c, a, sizeof(a));
    vsx_deliver_frame_failed(c, b, sizeof(b), 6);
    vsx_tick(c, ++now);
    uint8_t len1 = 0, len2 = 0, start = 0;
    rx_buffer_status(c, &len1, &start);
    rx_buffer_status(c, &len2, &start);
    check(len1 == sizeof(a), "the first status read gives this frame's length");
    check(len2 == sizeof(b), "the second gives the next frame's");
    vsx_destroy(c);
  }

  CASE("single-shot receive never hears the next frame");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t single[] = {0x82, 0x00, 0x00, 0x00};
    vsx_spi_transaction(c, single, nullptr, sizeof(single));
    vsx_tick(c, ++now);
    const uint8_t a[] = {0x01, 0x00, 0xAA, 0xBB, 0xCC};
    const uint8_t b[] = {0x01, 0x01, 0x91, 0xAA, 0xBB, 0xCC};
    vsx_deliver_frame(c, a, sizeof(a));
    vsx_deliver_frame_failed(c, b, sizeof(b), 5);
    vsx_tick(c, ++now);
    uint8_t len = 0, start = 0;
    rx_buffer_status(c, &len, &start);
    check(len == sizeof(a), "the length stays this frame's");
    vsx_destroy(c);
  }

  return report("");
}
