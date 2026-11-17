/* The two ways into the command decoder, and the noise the receiver answers
 * with.
 *
 * A host with the command in a buffer calls vsx_spi_transaction; an emulator
 * has no buffer and clocks it in a byte at a time. Both run the same decoder
 * and must give the same answers, or an emulated node is a different radio
 * from a native one.
 */
#include "harness.h"

int main() {
  std::printf("virtual-sx1262");

  /* ---------------------------------------------------------------- */
  CASE("both ways of framing a transaction give the same answer");
  /* There are two ways into this chip and they must not disagree.
   *
   * A host with a whole buffer calls spiTransfer. A host with a real chip
   * select shifts bytes one at a time, and that path consults returnsData() to
   * decide whether a command answers with anything. An opcode missing from that
   * list reads as zeroes down the byte path and correctly down the buffer path,
   * so the same chip behaves differently depending on how the caller is
   * plumbed - and both emulators use the byte path, so it is the one that
   * matters and the one nothing was checking.
   *
   * GetRssiInst was missing. Every emulated board read its receiver as 0 dBm:
   * no noise floor, no signal strength, and no entropy in the register RadioLib
   * samples to seed the firmware's random number generator. Rather than assert
   * that one opcode, walk every reader. */
  {
    const uint8_t readers[] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x17, 0x1D, 0x1E, 0xC0};
    for (uint8_t op : readers) {
      VirtualSX1262 chip;
      chip.setNoiseSeed(7);
      chip.setLastSignal(-73.5f, 4.25f);

      /* Six bytes is past the data of every command in the list. */
      uint8_t cmd[6] = {op, 0, 0, 0, 0, 0};
      uint8_t whole[6] = {0, 0, 0, 0, 0, 0};
      chip.spiTransfer(cmd, sizeof(cmd), whole);

      VirtualSX1262 twin;
      twin.setNoiseSeed(7);
      twin.setLastSignal(-73.5f, 4.25f);
      uint8_t shifted[6] = {0, 0, 0, 0, 0, 0};
      twin.beginTransaction();
      for (size_t i = 0; i < sizeof(cmd); ++i) {
        shifted[i] = twin.transferByte(cmd[i]);
      }
      twin.endTransaction();

      char what[96];
      std::snprintf(what, sizeof(what),
                    "opcode 0x%02X answers the same shifted as it does whole", op);
      check(std::memcmp(whole, shifted, sizeof(whole)) == 0, what);
    }
  }

  /* ---------------------------------------------------------------- */
  CASE("the receiver has noise, and two chips do not share it");
  /* What this is really testing is an identity. RadioLib's randomByte() reads
   * GetRssiInst eight times and keeps the low bit of each; MeshCore seeds its
   * PRNG with that and derives its keypair from the PRNG. This model returned
   * -rssi*2, which is always even, so the bit was always 0, the seed was always
   * 0, and every emulated board came up with the same identity. Worse, both
   * Arduino cores treat a zero seed as "ignore me", so nothing even looked
   * wrong until two boards were put side by side. */
  {
    auto sample_byte = [](vsx_chip* c) {
      /* Exactly what RadioLib's randomByte() does: ReadRegister of
       * REG_RANDOM_NUMBER_0 (0x0819), eight times, keeping the low bit of each.
       * [op][addr hi][addr lo][nop][value]. */
      uint8_t b = 0;
      for (int i = 0; i < 8; ++i) {
        const uint8_t cmd[] = {0x1D, 0x08, 0x19, 0x00, 0x00};
        uint8_t in[5] = {0, 0, 0, 0, 0};
        vsx_spi_transaction(c, cmd, in, sizeof(cmd));
        b = (uint8_t)((b << 1) | (in[4] & 1));
      }
      return b;
    };

    vsx_chip* a = vsx_create();
    uint64_t now = 0;
    bring_up(a, &now);
    vsx_set_noise_seed(a, 1);
    uint8_t first = sample_byte(a);
    uint8_t second = sample_byte(a);
    check(!(first == 0x00 && second == 0x00), "the low bit is not always zero");
    check(first != second, "successive reads differ, so eight of them are eight bits");

    vsx_chip* b = vsx_create();
    uint64_t bnow = 0;
    bring_up(b, &bnow);
    vsx_set_noise_seed(b, 2);
    check(sample_byte(b) != first, "a chip with another seed reads differently");

    /* And still reproducible: same seed, same stream. Determinism is the reason
     * this is seeded by the host rather than sampled from the machine. */
    vsx_chip* again = vsx_create();
    uint64_t anow = 0;
    bring_up(again, &anow);
    vsx_set_noise_seed(again, 1);
    check(sample_byte(again) == first, "the same seed gives the same noise");

    /* The dither must not move the reading itself more than a receiver's own
     * noise would: this register is read for signal strength as well. */
    vsx_state st;
    vsx_get_state(a, &st);
    const uint8_t cmd[] = {0x15, 0x00, 0x00};
    uint8_t in[3] = {0, 0, 0};
    vsx_spi_transaction(a, cmd, in, sizeof(cmd));
    const int reported = -(int)in[2] / 2;
    check(reported < -95 && reported > -105, "and it still reads about -100 dBm");

    vsx_destroy(a);
    vsx_destroy(b);
    vsx_destroy(again);
  }

  /* ---------------------------------------------------------------- */
  CASE("the ABI says which one it is");
  {
    int major = -1, minor = -1;
    vsx_abi_version(&major, &minor);
    check(major == 1 && minor >= 0, "vsx_abi_version reports a version");
  }
  return report("");
}
