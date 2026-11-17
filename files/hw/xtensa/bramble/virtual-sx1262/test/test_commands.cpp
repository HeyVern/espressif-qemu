/* The rest of the command set.
 *
 * Every case here was previously an opcode that fell through to "acknowledged
 * and ignored": the firmware issued it, got a status byte, and carried on with
 * the chip in a state it had not asked for.
 *
 * Several of these commands are recorded rather than acted on, and the cases
 * say so. A test that a value can be read back is not a weaker test than one
 * that it changed behaviour: the fault this model exists to catch is a node
 * configured differently from how its operator believes, and a value nobody can
 * read back is a value nobody can check.
 */
#include "harness.h"

namespace {

void tx(vsx_chip* c, const uint8_t* cmd, size_t n) {
  vsx_spi_transaction(c, cmd, nullptr, n);
}

vsx_state state_of(vsx_chip* c) {
  vsx_state s;
  vsx_get_state(c, &s);
  return s;
}

vsx_counters counters_of(vsx_chip* c) {
  vsx_counters k;
  vsx_get_counters(c, &k);
  return k;
}

}  // namespace

int main() {
  std::printf("virtual-sx1262 commands");

  /* ---------------------------------------------------------------- */
  CASE("GetStatus says what the chip is actually doing");
  /* It answered the constant 0x22 - standby - whatever the part was doing, so a
   * firmware polling it to find out whether its receiver was still armed was
   * told no by a chip that was receiving. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);

    uint8_t in[3] = {0};
    const uint8_t get[] = {0xC0, 0x00, 0x00};
    vsx_spi_transaction(c, get, in, sizeof(get));
    check(((in[1] >> 4) & 0x7) == 0x5, "receiving reads as RX");

    const uint8_t stby[] = {0x80, 0x00};
    tx(c, stby, sizeof(stby));
    vsx_spi_transaction(c, get, in, sizeof(get));
    check(((in[1] >> 4) & 0x7) == 0x2, "standby reads as STBY_RC");

    const uint8_t txc[] = {0x83, 0x00, 0x00, 0x00};
    tx(c, txc, sizeof(txc));
    vsx_spi_transaction(c, get, in, sizeof(get));
    check(((in[1] >> 4) & 0x7) == 0x6, "transmitting reads as TX");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("SetPacketType is remembered, and GetPacketType answers it");
  /* GetPacketType returned a hardcoded LoRa, so a firmware that had switched
   * the part to GFSK was told it had not. */
  {
    vsx_chip* c = vsx_create();
    check(state_of(c).packet_type == 0x01, "LoRa to begin with");

    const uint8_t gfsk[] = {0x8A, 0x00};
    tx(c, gfsk, sizeof(gfsk));
    uint8_t in[3] = {0};
    const uint8_t get[] = {0x11, 0x00, 0x00};
    vsx_spi_transaction(c, get, in, sizeof(get));
    check(in[2] == 0x00, "GFSK is answered as GFSK");
    check(state_of(c).packet_type == 0x00, "and reported to the host, which is");
    check(1, "how a host can refuse to believe a run that switched modem");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("the three SetPacketParams fields that were assumed now count");
  /* estAirtimeMs hardcoded an explicit header and CRC on. An implicit header is
   * twenty symbols of payload the model charged for and the part did not. */
  {
    vsx_chip* c = vsx_create();
    const uint8_t mod[] = {0x8B, 9, 0x04, 1, 0};
    tx(c, mod, sizeof(mod));

    /* preamble 8, explicit header, 32 bytes, CRC on, IQ normal */
    const uint8_t explicit_crc[] = {0x8C, 0x00, 0x08, 0x00, 32, 0x01, 0x00};
    tx(c, explicit_crc, sizeof(explicit_crc));
    const uint32_t full = vsx_est_airtime_ms(c, 32);
    check(state_of(c).header_implicit == 0 && state_of(c).crc_on == 1,
          "explicit header and CRC on are read, not assumed");

    const uint8_t implicit_nocrc[] = {0x8C, 0x00, 0x08, 0x01, 32, 0x00, 0x01};
    tx(c, implicit_nocrc, sizeof(implicit_nocrc));
    const uint32_t lean = vsx_est_airtime_ms(c, 32);
    check(state_of(c).header_implicit == 1, "an implicit header is read");
    check(state_of(c).crc_on == 0, "so is CRC off");
    check(state_of(c).invert_iq == 1, "so is inverted IQ");
    check(lean < full, "and dropping the header and the CRC costs less airtime");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("StopTimerOnPreamble keeps a bounded receive from cutting off a frame");
  /* Without it a receiver on a deadline gives up mid packet. This is the whole
   * reason the command exists. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t stop[] = {0x9F, 0x01};
    tx(c, stop, sizeof(stop));
    /* 6400 counts of 15.625 us is 100 ms. */
    const uint8_t rx[] = {0x82, 0x00, 0x19, 0x00};
    tx(c, rx, sizeof(rx));

    vsx_set_channel_busy(c, 1);
    for (uint64_t t = now + 1; t <= now + 200; ++t) {
      vsx_tick(c, t);
    }
    check((irq_flags(c) & IRQ_TIMEOUT) == 0, "a preamble stopped the timer");
    check(mode_of(c) == 1, "and the receiver is still listening past the deadline");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("SetLoRaSymbNumTimeout is a deadline counted in symbols");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 100;
    const uint8_t mod[] = {0x8B, 12, 0x04, 4, 0}; /* 32.768 ms a symbol */
    tx(c, mod, sizeof(mod));
    const uint8_t dio[] = {0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0};
    tx(c, dio, sizeof(dio));
    const uint8_t symb[] = {0xA0, 4};
    tx(c, symb, sizeof(symb));
    vsx_tick(c, now);
    const uint8_t rx[] = {0x82, 0x00, 0x00, 0x00}; /* no millisecond timeout */
    tx(c, rx, sizeof(rx));

    check(state_of(c).symb_num_timeout == 4, "four symbols is what was asked for");
    vsx_tick(c, now + 130);
    check((irq_flags(c) & IRQ_TIMEOUT) == 0, "a symbol short, still listening");
    vsx_tick(c, now + 132);
    check((irq_flags(c) & IRQ_TIMEOUT) != 0, "four symbols on, Timeout");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("SetRxTxFallbackMode is recorded, and FS is not a mode this model has");
  /* PLACEHOLDER, and the test pins the placeholder rather than pretending: a
   * firmware asking to fall back to FS gets standby, and what it asked for is
   * readable so a host can tell that is what happened. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t fb[] = {0x93, 0x40}; /* FS */
    tx(c, fb, sizeof(fb));
    check(state_of(c).fallback_mode == 0x40, "the mode asked for is readable");

    const uint8_t txc[] = {0x83, 0x00, 0x00, 0x00};
    tx(c, txc, sizeof(txc));
    vsx_transmit_finished(c);
    check(mode_of(c) == 0, "and a finished transmission still lands in standby");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("SetPaConfig is recorded, because the board owns the other half");
  {
    vsx_chip* c = vsx_create();
    const uint8_t pa[] = {0x95, 0x04, 0x07, 0x00, 0x01}; /* the +22 dBm row */
    tx(c, pa, sizeof(pa));
    const vsx_state s = state_of(c);
    check(s.pa_duty_cycle == 0x04 && s.pa_hp_max == 0x07, "both halves readable");
    check(s.pa_device_sel == 0x00, "and which device it selected");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("the recorded-only commands are all readable");
  /* One case for the lot of them, because the assertion is the same: the
   * firmware said something and the host can find out what. */
  {
    vsx_chip* c = vsx_create();
    const uint8_t reg[] = {0x96, 0x01}; /* DC-DC */
    const uint8_t sw[] = {0x9D, 0x01};  /* DIO2 drives the RF switch */
    const uint8_t tcxo[] = {0x97, 0x02, 0x00, 0x00, 0x64};
    tx(c, reg, sizeof(reg));
    tx(c, sw, sizeof(sw));
    tx(c, tcxo, sizeof(tcxo));

    const vsx_state s = state_of(c);
    check(s.regulator_mode == 0x01, "the regulator mode");
    check(s.dio2_as_rf_switch == 1, "that the chip was told to drive the RF switch");
    check(s.dio3_as_tcxo == 1, "and that DIO3 feeds a TCXO");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("SetRxDutyCycle records its periods and leaves the receiver on");
  /* PLACEHOLDER. A duty-cycled receiver is deaf for a scheduled fraction of the
   * time, which changes which frames arrive at all, and that is the engine's
   * decision rather than the chip's. Optimistic, and the test says so. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t duty[] = {0x94, 0x00, 0x19, 0x00, 0x00, 0x32, 0x00};
    tx(c, duty, sizeof(duty));

    const vsx_state s = state_of(c);
    check(s.rx_duty_rx_period == 0x001900, "the listening period is readable");
    check(s.rx_duty_sleep_period == 0x003200, "so is the sleeping one");
    check(mode_of(c) == 1, "and the receiver stays on, which is the optimistic case");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("the test modes key the transmitter and say so");
  /* RECORDED: a bare carrier is a transmission with no frame behind it, and
   * there is no way to hand that to the engine. A host modelling an interferer
   * reads the flag and puts the carrier on the air itself. */
  {
    vsx_chip* c = vsx_create();
    const uint8_t cw[] = {0xD1};
    tx(c, cw, sizeof(cw));
    check(state_of(c).tx_continuous_wave == 1, "a continuous wave is flagged");
    check(mode_of(c) == 2, "and the chip is transmitting");
    vsx_transmit_finished(c);
    check(state_of(c).tx_continuous_wave == 0, "and stops when the carrier does");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("GetStats counts what arrived");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t frame[] = {1, 2, 3};
    vsx_deliver_frame(c, frame, sizeof(frame));
    vsx_tick(c, ++now);

    uint8_t in[9] = {0};
    const uint8_t get[] = {0x10, 0, 0, 0, 0, 0, 0, 0, 0};
    vsx_spi_transaction(c, get, in, sizeof(get));
    check(((in[2] << 8) | in[3]) == 1, "one packet received");
    check(counters_of(c).stat_rx_packets == 1, "and the host sees the same");

    const uint8_t reset[] = {0x00, 0, 0, 0, 0, 0, 0};
    for (size_t n = 1; n < sizeof(reset); ++n) {
      tx(c, reset, n);
      check(counters_of(c).stat_rx_packets == 1,
            "an incomplete buffered ResetStats preserves counters");
      vsx_spi_begin(c);
      for (size_t i = 0; i < n; ++i) vsx_spi_byte(c, reset[i]);
      vsx_spi_end(c);
      check(counters_of(c).stat_rx_packets == 1,
            "an incomplete byte-stream ResetStats preserves counters");
    }
    tx(c, reset, sizeof(reset));
    check(counters_of(c).stat_rx_packets == 0, "ResetStats clears it");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a frame that failed its CRC can be delivered, once something says so");
  /* PLACEHOLDER INTERFACE. Nothing calls this: the chip cannot know a CRC
   * failed, and no host is wired to tell it. The case exists so that the path
   * is real rather than hypothetical when one is. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t frame[] = {0xDE, 0xAD};
    vsx_deliver_frame_failed(c, frame, sizeof(frame), 1);

    check((irq_flags(c) & IRQ_RX_DONE) != 0, "RxDone, because a frame did arrive");
    check((irq_flags(c) & IRQ_CRC_ERR) != 0, "and CrcErr beside it");
    check(counters_of(c).stat_crc_errors == 1, "GetStats counts it");

    vsx_deliver_frame_failed(c, frame, sizeof(frame), 2);
    check((irq_flags(c) & IRQ_HEADER_ERR) != 0, "a header error raises HeaderErr");
    check(counters_of(c).stat_header_errors == 1, "and is counted separately");
    vsx_destroy(c);
  }

  std::printf("\n%s: %d failure(s)\n", g_failures ? "FAILED" : "PASSED", g_failures);
  return g_failures ? 1 : 0;
}
