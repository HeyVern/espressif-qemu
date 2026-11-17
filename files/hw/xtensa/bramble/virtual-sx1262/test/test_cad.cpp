/* Channel activity detection: what a scan costs, what it finds, and where it
 * leaves the chip.
 *
 * Every case here failed before CAD was modelled, and most of them failed the
 * same way: the old scan answered inside the SPI command, so it cost nothing,
 * held the chip for no time and always came back to standby. A firmware doing
 * listen before talk with it paid none of the dwell a real part charges, which
 * is the entire price of the technique.
 *
 * Timed at SF12 and 125 kHz throughout, where one symbol is 32.768 ms. That is
 * slow enough that a dwell cannot be confused with a rounding error, which at
 * SF7 it can.
 */
#include "harness.h"

namespace {

/* SF12, 125 kHz, CR 4/8, every interrupt unmasked onto DIO1. Not bring_up():
 * that one picks its own modulation, and every assertion below is a time in
 * symbols. */
void bring_up_slow(vsx_chip* c) {
  const uint8_t mod[] = {0x8B, 12, 0x04, 4, 0};
  vsx_spi_transaction(c, mod, nullptr, sizeof(mod));
  const uint8_t dio[] = {0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0};
  vsx_spi_transaction(c, dio, nullptr, sizeof(dio));
}

/* SetCadParams: symbolNum, detPeak, detMin, exitMode, timeout 23:16..7:0. */
void set_cad_params(vsx_chip* c, uint8_t symbols, uint8_t exit_mode) {
  const uint8_t p[] = {0x88, symbols, 22, 10, exit_mode, 0, 0, 0};
  vsx_spi_transaction(c, p, nullptr, sizeof(p));
}

void set_cad(vsx_chip* c) {
  const uint8_t cad[] = {0xC5};
  vsx_spi_transaction(c, cad, nullptr, sizeof(cad));
}

uint32_t cad_runs(vsx_chip* c) {
  vsx_counters k;
  vsx_get_counters(c, &k);
  return k.cad_runs;
}

}  // namespace

int main() {
  std::printf("virtual-sx1262");

  /* ---------------------------------------------------------------- */
  CASE("a scan holds the chip for the dwell its symbol count asks for");
  /* The old model set CadDone inside the command and returned to standby
   * before it came back, so a firmware could scan and transmit in the same
   * millisecond. Four symbols at SF12/125 kHz is 131 ms, and the answer is not
   * available until they have passed. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 100;
    bring_up_slow(c);
    set_cad_params(c, 2, 0x00); /* 1 << 2 symbols */
    vsx_tick(c, now);
    set_cad(c);

    check(mode_of(c) == 3, "the chip is in CAD as soon as the command lands");
    check((irq_flags(c) & (IRQ_CAD_DONE | IRQ_CAD_DETECTED)) == 0,
          "and has answered nothing yet");

    vsx_tick(c, now + 130);
    check(mode_of(c) == 3, "one millisecond short of the dwell it is still scanning");
    check((irq_flags(c) & IRQ_CAD_DONE) == 0, "and still silent");

    vsx_tick(c, now + 131);
    check((irq_flags(c) & IRQ_CAD_DONE) != 0, "at the end of the dwell it reports");
    check(cad_runs(c) == 1, "and the scan is counted");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a longer symbol count is a longer scan");
  /* The parameter was discarded entirely, so every scan cost the same nothing
   * whatever the firmware asked for. */
  {
    vsx_chip* c = vsx_create();
    bring_up_slow(c);
    set_cad_params(c, 4, 0x00); /* 16 symbols, 524 ms */
    vsx_tick(c, 100);
    set_cad(c);

    vsx_tick(c, 100 + 131);
    check(mode_of(c) == 3, "four symbols in, a sixteen symbol scan is not finished");
    vsx_tick(c, 100 + 524);
    check((irq_flags(c) & IRQ_CAD_DONE) != 0, "sixteen symbols in, it is");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a carrier that comes and goes inside the dwell is still found");
  /* Sampling at either end of the scan instead of across it would miss exactly
   * the transmission CAD exists to catch: a short one, starting after the scan
   * began and ending before it finished. */
  {
    vsx_chip* c = vsx_create();
    bring_up_slow(c);
    set_cad_params(c, 2, 0x00);
    vsx_tick(c, 100);
    set_cad(c);

    vsx_set_channel_busy(c, 1);
    vsx_tick(c, 150);
    vsx_set_channel_busy(c, 0);
    vsx_tick(c, 180);
    vsx_tick(c, 231);

    check((irq_flags(c) & IRQ_CAD_DONE) != 0, "the scan finished");
    check((irq_flags(c) & IRQ_CAD_DETECTED) != 0,
          "and it found the carrier it overlapped");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a quiet channel is scanned and reported quiet");
  {
    vsx_chip* c = vsx_create();
    bring_up_slow(c);
    set_cad_params(c, 2, 0x00);
    vsx_tick(c, 100);
    set_cad(c);
    vsx_tick(c, 231);

    check((irq_flags(c) & IRQ_CAD_DONE) != 0, "CadDone is raised either way");
    check((irq_flags(c) & IRQ_CAD_DETECTED) == 0, "CadDetected is not");
    check(mode_of(c) == 0, "and CAD_ONLY leaves the chip in standby");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("the thresholds the firmware programmed are reported, not applied");
  /* detPeak and detMin threshold a correlator peak, and this model is told
   * whether a carrier is present rather than how strong it is. Recording them
   * is what lets a host that does know the level act on them, and what makes a
   * firmware setting an implausible threshold visible at all. */
  {
    vsx_chip* c = vsx_create();
    bring_up_slow(c);
    set_cad_params(c, 3, 0x01);

    vsx_state s;
    vsx_get_state(c, &s);
    check(s.cad_symbol_num == 3, "the symbol count is readable");
    check(s.cad_det_peak == 22 && s.cad_det_min == 10, "so are both thresholds");
    check(s.cad_exit_mode == 0x01, "and the exit mode");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a symbol count the part does not define is clamped, not shifted with");
  /* 1 << 9 symbols is not a scan, it is undefined behaviour in the dwell
   * arithmetic and a dwell no firmware asked for. */
  {
    vsx_chip* c = vsx_create();
    bring_up_slow(c);
    set_cad_params(c, 9, 0x00);

    vsx_state s;
    vsx_get_state(c, &s);
    check(s.cad_symbol_num == 4, "clamped to the longest scan the part defines");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a scan the firmware masked off runs, and is not reported");
  /* The mask governs the status register, not the radio. The scan still costs
   * its dwell and the host still sees it happened; the firmware does not. */
  {
    vsx_chip* c = vsx_create();
    const uint8_t mod[] = {0x8B, 12, 0x04, 4, 0};
    vsx_spi_transaction(c, mod, nullptr, sizeof(mod));
    /* Everything except the two CAD bits. */
    const uint8_t dio[] = {0x08, 0xFE, 0x7F, 0xFF, 0xFF, 0, 0, 0, 0};
    vsx_spi_transaction(c, dio, nullptr, sizeof(dio));
    set_cad_params(c, 2, 0x00);
    vsx_tick(c, 100);
    vsx_set_channel_busy(c, 1);
    set_cad(c);
    vsx_tick(c, 231);

    check((irq_flags(c) & (IRQ_CAD_DONE | IRQ_CAD_DETECTED)) == 0,
          "neither CAD flag reaches the status register");
    check(cad_runs(c) == 1, "but the scan ran");
    vsx_destroy(c);
  }

  std::printf("\n%s: %d failure(s)\n", g_failures ? "FAILED" : "PASSED", g_failures);
  return g_failures ? 1 : 0;
}
