/* What SetModulationParams programmed, and who the chip will listen to.
 *
 * Two kinds of fault share this file because they present the same way: a node
 * that is configured wrong and reports nothing wrong. A bandwidth code the part
 * does not define used to leave the modem on its previous setting in silence,
 * and a sync word the firmware set was written into a register that nothing
 * ever read back, so two meshes on one frequency heard each other perfectly.
 */
#include "harness.h"

namespace {

void set_modulation(vsx_chip* c, uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro) {
  const uint8_t mod[] = {0x8B, sf, bw, cr, ldro};
  vsx_spi_transaction(c, mod, nullptr, sizeof(mod));
}

void write_sync_word(vsx_chip* c, uint16_t word) {
  const uint8_t w[] = {0x0D, 0x07, 0x40, (uint8_t)(word >> 8), (uint8_t)(word & 0xFF)};
  vsx_spi_transaction(c, w, nullptr, sizeof(w));
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

void arm_receiver(vsx_chip* c) {
  const uint8_t dio[] = {0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0};
  vsx_spi_transaction(c, dio, nullptr, sizeof(dio));
  const uint8_t rx[] = {0x82, 0xFF, 0xFF, 0xFF};
  vsx_spi_transaction(c, rx, nullptr, sizeof(rx));
}

}  // namespace

int main() {
  std::printf("virtual-sx1262 modem");

  /* ---------------------------------------------------------------- */
  CASE("every bandwidth in the table is a bandwidth");
  {
    vsx_chip* c = vsx_create();
    const uint8_t codes[] = {0x00, 0x08, 0x01, 0x09, 0x02, 0x0A, 0x03, 0x04, 0x05, 0x06};
    const uint32_t hz[] = {7810,  10420, 15630,  20830,  31250,
                           41670, 62500, 125000, 250000, 500000};
    bool all = true;
    for (unsigned i = 0; i < sizeof(codes); ++i) {
      set_modulation(c, 9, codes[i], 1, 0);
      if (state_of(c).bandwidth_hz != hz[i]) {
        all = false;
      }
    }
    check(all, "all ten codes decode to the datasheet's bandwidths");
    check(counters_of(c).params_rejected == 0, "and none of them is refused");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a bandwidth code the part does not define is refused and counted");
  /* This file's own harness had been sending 0x1A, which is not a code. The
   * modem kept its previous setting, so every timing assertion ran at a
   * bandwidth nobody had chosen and nothing said so. */
  {
    vsx_chip* c = vsx_create();
    set_modulation(c, 9, 0x04, 1, 0);
    check(state_of(c).bandwidth_hz == 125000, "125 kHz to begin with");

    set_modulation(c, 9, 0x1A, 1, 0);
    check(state_of(c).bandwidth_hz == 125000,
          "a code that is not a code changes nothing");
    check(counters_of(c).params_rejected == 1, "and is counted as refused");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a spreading factor or coding rate outside the part is refused too");
  {
    vsx_chip* c = vsx_create();
    set_modulation(c, 9, 0x04, 1, 0);

    set_modulation(c, 20, 0x04, 1, 0);
    check(state_of(c).spreading_factor == 9, "SF20 does not become the setting");
    set_modulation(c, 9, 0x04, 7, 0);
    check(state_of(c).coding_rate == 5, "nor does a coding rate of 4/11");
    check(counters_of(c).params_rejected == 2, "both are counted");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("low data rate optimisation is the field the firmware programmed");
  /* It used to be inferred from the spreading factor alone, which is the one
   * thing it does not depend on: the threshold is a symbol duration, so SF11
   * crosses it at 125 kHz and does not at 250. The airtime this chip quotes is
   * the figure the firmware times its own CSMA on, so inferring it wrong
   * desynchronised the chip from the air it was supposed to be describing. */
  {
    vsx_chip* c = vsx_create();

    set_modulation(c, 11, 0x05, 1, 0); /* SF11, 250 kHz, 4/5, LDRO off */
    check(state_of(c).low_data_rate_optimize == 0, "off is reported off");
    const uint32_t without = vsx_est_airtime_ms(c, 50);
    check(without >= 640 && without <= 642, "a 50 byte frame is about 641 ms");

    set_modulation(c, 11, 0x05, 1, 1); /* the same, LDRO on */
    check(state_of(c).low_data_rate_optimize == 1, "on is reported on");
    const uint32_t with = vsx_est_airtime_ms(c, 50);
    check(with >= 722 && with <= 724, "with it on the same frame is about 723 ms");
    check(with > without, "two bits a symbol is not free");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a firmware that never programmed it falls back to the symbol duration");
  /* Not to the spreading factor. SF11 is over the threshold at 125 kHz and
   * under it at 250, and the fallback has to say so. */
  {
    vsx_chip* c = vsx_create();
    const uint8_t narrow[] = {0x8B, 11, 0x04, 1}; /* no LDRO byte at all */
    vsx_spi_transaction(c, narrow, nullptr, sizeof(narrow));
    check(state_of(c).low_data_rate_optimize == 1,
          "SF11 at 125 kHz is over the threshold");

    vsx_chip* d = vsx_create();
    const uint8_t wide[] = {0x8B, 11, 0x05, 1};
    vsx_spi_transaction(d, wide, nullptr, sizeof(wide));
    check(state_of(d).low_data_rate_optimize == 0, "SF11 at 250 kHz is under it");
    vsx_destroy(c);
    vsx_destroy(d);
  }

  /* ---------------------------------------------------------------- */
  CASE("the chip comes up on the private sync word");
  /* Zero would have every node agreeing with every other node about a value
   * none of them chose, which is the same bug as the shared keypair: a default
   * that is not the part's default is a default that hides a missing setting. */
  {
    vsx_chip* c = vsx_create();
    check(state_of(c).sync_word == 0x1424, "0x1424, as the part resets to");
    write_sync_word(c, 0x3444);
    check(state_of(c).sync_word == 0x3444, "and the firmware can set the public one");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a frame sent under another sync word is not this node's to hear");
  /* The word is written through two registers rather than a command, so it
   * landed in the register array and nothing read it back: two meshes sharing a
   * frequency heard each other perfectly, which is the opposite of what the
   * word is for. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    write_sync_word(c, 0x1424);

    const uint8_t frame[] = {1, 2, 3, 4};
    vsx_deliver_frame_from(c, frame, sizeof(frame), 0x3444);
    vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_RX_DONE) == 0, "the other network's frame is refused");
    check(counters_of(c).sync_mismatches == 1, "and counted, not silently lost");

    vsx_deliver_frame_from(c, frame, sizeof(frame), 0x1424);
    vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_RX_DONE) != 0, "its own network's frame arrives");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a host that does not model sync words is unaffected");
  /* vsx_deliver_frame makes no claim about the transmitter, and no claim is not
   * a claim that it matched. Refusing on it would break every host that has not
   * been taught about the word yet. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    write_sync_word(c, 0x3444);

    const uint8_t frame[] = {9, 9};
    vsx_deliver_frame(c, frame, sizeof(frame));
    vsx_tick(c, ++now);
    check((irq_flags(c) & IRQ_RX_DONE) != 0, "the older entry point still delivers");
    check(counters_of(c).sync_mismatches == 0, "and refuses nothing");
    vsx_destroy(c);
  }

  /* ---------------------------------------------------------------- */
  CASE("a refused frame does not occupy the receiver either");
  /* Dropping it into the inbox and then refusing it later would make it a frame
   * the chip was deaf to, which is a different counter and a different fault. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    arm_receiver(c);
    const uint8_t frame[] = {1};
    vsx_deliver_frame_from(c, frame, sizeof(frame), 0x3444);
    vsx_tick(c, ++now);
    check(counters_of(c).frames_dropped == 0, "it was never handed over to be dropped");
    check(counters_of(c).sync_mismatches == 1, "it was refused at the door");
    vsx_destroy(c);
  }

  std::printf("\n%s: %d failure(s)\n", g_failures ? "FAILED" : "PASSED", g_failures);
  return g_failures ? 1 : 0;
}
