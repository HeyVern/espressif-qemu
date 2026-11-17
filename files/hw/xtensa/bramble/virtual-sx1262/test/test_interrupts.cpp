/* DIO1: what raises it, when it is pushed, and the one case where the chip is
 * asked to misbehave the way real parts do.
 *
 * The pin is the whole reception path on an nRF52 - MeshCore reads a received
 * packet only from the interrupt it raises - so a chip that receives perfectly
 * and cannot say so is a node that forwards nothing.
 */
#include "harness.h"

/* Two frames heard under one IrqMask, the way a receive loop takes them: a
 * carrier long enough to carry its header, the frame, the carrier gone, and the
 * status cleared before the next. Answers what the chip counted as suppressed. */
static uint32_t suppressed_over_two_frames(uint16_t mask) {
  vsx_chip* c = vsx_create();
  uint64_t now = 0;
  bring_up(c, &now);
  /* SetDioIrqParams: the mask under test, and RxDone alone routed to DIO1. */
  const uint8_t dio[] = {0x08, (uint8_t)(mask >> 8), (uint8_t)mask, 0, 0x02, 0, 0, 0, 0};
  vsx_spi_transaction(c, dio, nullptr, sizeof(dio));
  const uint8_t frame[] = {1, 2, 3, 4};
  for (int n = 0; n < 2; ++n) {
    vsx_set_channel_busy(c, 1);
    for (int i = 0; i < 150; ++i) {
      vsx_tick(c, ++now);
    }
    vsx_deliver_frame(c, frame, sizeof(frame));
    vsx_set_channel_busy(c, 0);
    vsx_tick(c, ++now);
    const uint8_t clear[] = {0x02, 0xFF, 0xFF};
    vsx_spi_transaction(c, clear, nullptr, sizeof(clear));
  }
  vsx_counters k;
  vsx_get_counters(c, &k);
  vsx_destroy(c);
  return k.irq_suppressed;
}

int main() {
  std::printf("virtual-sx1262");

  /* ---------------------------------------------------------------- */
  CASE("only what is routed to DIO1 raises DIO1");
  /* The bug this repository was extracted to fix, and the reason every other
   * case here missed it: bring_up unmasks everything onto DIO1, which is more
   * generous than any real driver, so the model could confuse the two masks and
   * still pass.
   *
   * SetDioIrqParams carries IrqMask and then Dio1Mask. RadioLib's startReceive
   * enables RxDone, Timeout, CrcErr, HeaderValid and HeaderErr in the status
   * register but routes ONLY RxDone to the pin. Gate the pin on the enable mask
   * and HeaderValid raises DIO1 twelve symbols into the carrier, so DIO1 is
   * already high when RxDone lands, there is no rising edge, and a driver that
   * attached the pin with GpioInterruptRising - which RadioLib does - never
   * learns the packet exists. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    const uint8_t mod[] = {0x8B, 8, 0x1A, 4, 0};
    vsx_spi_transaction(c, mod, nullptr, sizeof(mod));

    /* RadioLib's own two masks, not a convenient pair. 0x0272 is
     * RxDone|HeaderValid|HeaderErr|CrcErr|Timeout; 0x0002 is RxDone alone. */
    const uint8_t dio[] = {0x08, 0x02, 0x72, 0x00, 0x02, 0, 0, 0, 0};
    vsx_spi_transaction(c, dio, nullptr, sizeof(dio));
    const uint8_t rx[] = {0x82, 0xFF, 0xFF, 0xFF};
    vsx_spi_transaction(c, rx, nullptr, sizeof(rx));
    vsx_tick(c, ++now);

    vsx_state st;
    vsx_get_state(c, &st);
    check(st.irq_mask == 0x0272, "the enable mask is what the driver sent");
    check(st.dio1_mask == 0x0002, "and the DIO1 mask is the narrower one");

    /* A carrier arrives and is detected. */
    vsx_set_channel_busy(c, 1);
    for (int i = 0; i < 200; ++i) {
      vsx_tick(c, ++now);
    }
    check((irq_flags(c) & IRQ_HEADER) != 0,
          "HeaderValid is set, so the channel check can still read it");
    check(vsx_dio1_asserted(c) == 0, "but HeaderValid does not raise DIO1");

    /* The carrier ends and the frame is handed over. */
    vsx_set_channel_busy(c, 0);
    const uint8_t frame[] = {0x11, 0x00, 0xAB, 0xCD};
    vsx_deliver_frame(c, frame, sizeof(frame));
    vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_RX_DONE) != 0, "RxDone is set on delivery");
    check(vsx_dio1_asserted(c) == 1, "and RxDone does raise DIO1");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("the chip raises DIO1 instead of waiting to be asked");
  /* The socket this ABI replaces was request-response, so hosts polled the
   * line on a 1 ms timer - a millisecond of latency on every packet, for a
   * pin a real part asserts. */
  {
    struct Seen {
      int edges = 0;
      int level = -1;
    } seen;
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    vsx_set_dio1_callback(
        c,
        [](void* u, int asserted) {
          auto* s = static_cast<Seen*>(u);
          ++s->edges;
          s->level = asserted;
        },
        &seen);

    check(seen.edges == 0, "a quiet chip raises nothing");

    const uint8_t frame[] = {0x11, 0x00, 0x01, 0x02};
    vsx_deliver_frame(c, frame, sizeof(frame));
    check(seen.edges == 0, "handing over a frame alone does not assert it");

    vsx_tick(c, ++now);
    check(seen.edges == 1 && seen.level == 1, "delivering the packet asserts DIO1, once");

    vsx_tick(c, ++now);
    vsx_tick(c, ++now);
    check(seen.edges == 1, "and it does not re-fire while the level holds");

    /* ClearIrqStatus of everything - the driver acknowledging. */
    const uint8_t clr[] = {0x02, 0xFF, 0xFF};
    vsx_spi_transaction(c, clr, nullptr, sizeof(clr));
    check(seen.edges == 2 && seen.level == 0, "acknowledging it drops the line, once");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("the chip only lies when it is asked to");
  /* Latching a detection flag past the signal is what real SX1262s do
   * sometimes, and what MeshCore 1.17's recovery exists to survive. It is a
   * deliberate variant, not the default - a normal chip must not lie, or
   * every repeater in a run believes the channel is busy for ever. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    vsx_set_stuck_irq_ms(c, 50);

    for (int i = 0; i < 200; ++i) {
      vsx_tick(c, ++now);
    }
    vsx_counters k;
    vsx_get_counters(c, &k);
    check(k.spurious_raises > 0,
          "the stuck variant raises a flag with nothing on the air");
    vsx_destroy(c);

    c = vsx_create();
    now = 0;
    bring_up(c, &now);
    for (int i = 0; i < 200; ++i) {
      vsx_tick(c, ++now);
    }
    vsx_get_counters(c, &k);
    check(k.spurious_raises == 0, "the default chip does not");
    check((irq_flags(c) & (IRQ_PREAMBLE | IRQ_HEADER)) == 0,
          "and reports a quiet channel when the air is quiet");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("an interrupt the firmware masked off is not in the status register");
  /* IrqMask was parsed and then never used, so the register answered every
   * event whatever the firmware had enabled. That is forgiving in the direction
   * that hides a fault: a driver waiting on an interrupt it forgot to enable
   * waits for ever on silicon and works perfectly here, and the difference only
   * shows up on a board.
   *
   * The pair sent here is RadioLib's own for a receive that wants only RxDone:
   * enabled in the register, routed to the pin, and nothing else in either. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    const uint8_t mod[] = {0x8B, 8, 0x1A, 4, 0};
    vsx_spi_transaction(c, mod, nullptr, sizeof(mod));
    const uint8_t dio[] = {0x08, 0x00, 0x02, 0x00, 0x02, 0, 0, 0, 0};
    vsx_spi_transaction(c, dio, nullptr, sizeof(dio));
    const uint8_t rx[] = {0x82, 0xFF, 0xFF, 0xFF};
    vsx_spi_transaction(c, rx, nullptr, sizeof(rx));

    vsx_set_channel_busy(c, 1);
    for (int i = 0; i < 100; ++i) {
      vsx_tick(c, ++now);
    }
    check((irq_flags(c) & (IRQ_PREAMBLE | IRQ_HEADER)) == 0,
          "the detection flags are masked off, so the register does not carry them");

    vsx_counters k;
    vsx_get_counters(c, &k);
    check(k.preamble_raises > 0, "the carrier was detected all the same");
    check(k.irq_suppressed > 0, "and the chip counts what it did not report");

    const uint8_t frame[] = {1, 2, 3, 4};
    vsx_deliver_frame(c, frame, sizeof(frame));
    vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_RX_DONE) != 0, "the one interrupt it did enable arrives");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a masked interrupt cannot raise DIO1 either");
  /* DIO1 selects from what the register carries. Routing a bit to the pin that
   * the mask keeps out of the register is a driver asking for something the
   * part cannot do, and answering it would put a level on the pin with nothing
   * behind it to read. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    const uint8_t mod[] = {0x8B, 8, 0x1A, 4, 0};
    vsx_spi_transaction(c, mod, nullptr, sizeof(mod));
    /* Nothing enabled, everything routed. */
    const uint8_t dio[] = {0x08, 0x00, 0x00, 0xFF, 0xFF, 0, 0, 0, 0};
    vsx_spi_transaction(c, dio, nullptr, sizeof(dio));
    const uint8_t rx[] = {0x82, 0xFF, 0xFF, 0xFF};
    vsx_spi_transaction(c, rx, nullptr, sizeof(rx));

    vsx_set_channel_busy(c, 1);
    for (int i = 0; i < 100; ++i) {
      vsx_tick(c, ++now);
    }
    check(vsx_dio1_asserted(c) == 0, "the pin stays low");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a receive timeout expires into Timeout and standby");
  /* SetRx carried a timeout that was discarded, so the bit could never be set
   * and a bounded receive never ended. Continuous receive is all ones and is
   * the case that must keep working, because it is the one MeshCore uses. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    /* 6400 counts of 15.625 us is 100 ms. */
    const uint8_t rx[] = {0x82, 0x00, 0x19, 0x00};
    vsx_spi_transaction(c, rx, nullptr, sizeof(rx));

    vsx_tick(c, now + 99);
    check(mode_of(c) == 1, "before the deadline it is still listening");
    check((irq_flags(c) & IRQ_TIMEOUT) == 0, "and has not timed out");

    vsx_tick(c, now + 100);
    check((irq_flags(c) & IRQ_TIMEOUT) != 0, "at the deadline Timeout is raised");
    check(mode_of(c) == 0, "and the receiver has stopped");
    vsx_destroy(c);

    c = vsx_create();
    now = 0;
    bring_up(c, &now);
    for (int i = 0; i < 500; ++i) {
      vsx_tick(c, ++now);
    }
    check(mode_of(c) == 1, "continuous receive has no deadline to reach");
    check((irq_flags(c) & IRQ_TIMEOUT) == 0, "and never times out");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a transmit timeout expires into Timeout and standby");
  /* SetTx carries the same three bytes SetRx does, and they were discarded the
   * same way. Unlike receive there is no continuous case: a transmission ends,
   * so a deadline reached means the waveform never finished. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    /* 6400 counts of 15.625 us is 100 ms. */
    const uint8_t tx[] = {0x83, 0x00, 0x19, 0x00};
    vsx_spi_transaction(c, tx, nullptr, sizeof(tx));
    check(mode_of(c) == 2, "the chip is transmitting");

    vsx_tick(c, now + 99);
    check((irq_flags(c) & IRQ_TIMEOUT) == 0, "before the deadline, nothing");
    vsx_tick(c, now + 100);
    check((irq_flags(c) & IRQ_TIMEOUT) != 0, "at it, Timeout");
    check(mode_of(c) == 0, "and the transmitter has stopped");
    check((irq_flags(c) & IRQ_TX_DONE) == 0, "a transmission that timed out is not done");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a transmission that finishes in time does not also time out");
  /* The deadline has to be disarmed by the thing it was waiting for, or a chip
   * that transmitted perfectly reports a fault a hundred milliseconds later. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t tx[] = {0x83, 0x00, 0x19, 0x00};
    vsx_spi_transaction(c, tx, nullptr, sizeof(tx));

    vsx_tick(c, now + 10);
    vsx_transmit_finished(c);
    check((irq_flags(c) & IRQ_TX_DONE) != 0, "TxDone arrives");

    vsx_tick(c, now + 500);
    check((irq_flags(c) & IRQ_TIMEOUT) == 0, "and the deadline it beat never fires");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a transmit timeout of zero is a driver that will wait");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t tx[] = {0x83, 0x00, 0x00, 0x00};
    vsx_spi_transaction(c, tx, nullptr, sizeof(tx));

    vsx_tick(c, now + 5000);
    check(mode_of(c) == 2, "still transmitting five seconds later");
    check((irq_flags(c) & IRQ_TIMEOUT) == 0, "with no deadline to reach");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a LoRa header is not counted against a mask that leaves FSK out");
  /* The header was raised as HeaderValid and SyncWordValid together, and
   * SyncWordValid is the GFSK modem's. No LoRa firmware enables it, so every
   * header counted as suppressed and a correctly configured firmware looked as
   * though it were still missing interrupts. The two masks are MeshCore's own:
   * 1.17 enables PreambleDetected for receive and 1.16 does not. */
  {
    check(suppressed_over_two_frames(0x0276) == 0,
          "1.17's mask misses nothing on a frame it receives");
    check(suppressed_over_two_frames(0x0272) == 2,
          "1.16's misses PreambleDetected, once per frame");
  }

  return report("");
}
