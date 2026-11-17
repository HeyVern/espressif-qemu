/* The header has to be valid C, not just C++.
 *
 * Two of the four hosts are not C++: QEMU's device is C, and Renode reaches the
 * library through a P/Invoke declaration written from this header by hand. A
 * header that only compiles under a C++ compiler passes every C++ test in this
 * repository and then fails in the one place it matters, so this file is
 * compiled by the C compiler on purpose and calls every entry point.
 *
 * It asserts almost nothing about behaviour. That is test_model.cpp's job. What
 * this proves is that the ABI is reachable from C: no C++ types leak through the
 * header, no default arguments, no references, and every symbol links.
 */
#include "virtual_sx1262.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check(int ok, const char* what) {
  printf("    %-4s %s\n", ok ? "ok" : "FAIL", what);
  if (!ok) {
    ++failures;
  }
}

static int dio1_edges = 0;

static void on_dio1(void* user, int asserted) {
  (void)asserted;
  ++*(int*)user;
}

/* An emulator has no buffer to hand the chip: its SPI controller clocks one
 * byte and wants the answering byte back before it clocks the next. That is a
 * second way into the same command decoder, and the two must not drift - an
 * emulated node and a native one being different radios is what this whole
 * arrangement exists to avoid, and it would show up as a board that configures
 * its radio and then behaves slightly differently for reasons nobody can see.
 *
 * Driven from C rather than from test_model.cpp because this is the path QEMU
 * and Renode take, and they take it through this header.
 */
static void byte_path_matches_buffer_path(void) {
  /* Commands that answer with data, which is where the two paths could differ:
   * a write is only run at the release and has nothing to say meanwhile.
   * GetRssiInst is here deliberately - it reads the receiver's noise, which is
   * drawn once per transaction, so it also proves the two paths draw alike. */
  static const uint8_t cmds[][9] = {
      {0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0}, /* SetDioIrqParams */
      {0x82, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0, 0},    /* SetRx */
      {0xC0, 0x00, 0, 0, 0, 0, 0, 0, 0},          /* GetStatus */
      {0x12, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0},    /* GetIrqStatus */
      {0x15, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0}, /* GetRssiInst */
      {0x14, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0}, /* GetPacketStatus */
      {0x13, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0},    /* GetRxBufferStatus */
      {0x17, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0},    /* GetDeviceErrors */
  };
  static const size_t lens[] = {9, 4, 2, 4, 5, 5, 4, 4};
  /* Two chips rather than one, run through the identical sequence: same
   * default noise seed, so any difference is the path and not the draw. */
  vsx_chip* buffered = vsx_create();
  vsx_chip* clocked = vsx_create();
  size_t c;
  int mismatches = 0;

  for (c = 0; c < sizeof(lens) / sizeof(lens[0]); c++) {
    uint8_t a[16];
    uint8_t b[16];
    size_t i;

    memset(a, 0, sizeof(a));
    memset(b, 0, sizeof(b));
    vsx_spi_transaction(buffered, cmds[c], a, lens[c]);

    vsx_spi_begin(clocked);
    for (i = 0; i < lens[c]; i++) {
      b[i] = vsx_spi_byte(clocked, cmds[c][i]);
    }
    vsx_spi_end(clocked);

    if (memcmp(a, b, lens[c]) != 0) {
      printf("      0x%02x: buffered", cmds[c][0]);
      for (i = 0; i < lens[c]; i++) {
        printf(" %02x", a[i]);
      }
      printf(", clocked");
      for (i = 0; i < lens[c]; i++) {
        printf(" %02x", b[i]);
      }
      printf("\n");
      ++mismatches;
    }
  }
  check(mismatches == 0, "every command answers the same clocked as buffered");

  /* And a write actually takes effect on the byte path, which is the half a
   * comparison of replies cannot see: a chip that answered identically and ran
   * nothing would pass everything above. */
  {
    vsx_state buffered_state;
    vsx_state clocked_state;
    vsx_get_state(buffered, &buffered_state);
    vsx_get_state(clocked, &clocked_state);
    check(clocked_state.mode == 1, "a write clocked in byte by byte took effect");
    check(clocked_state.dio1_mask == buffered_state.dio1_mask,
          "and left the chip in the same state as the buffered path");
  }

  vsx_destroy(buffered);
  vsx_destroy(clocked);
}

int main(void) {
  vsx_chip* chip;
  vsx_state state;
  vsx_counters counters;
  /* Big enough for the longest transaction below: SetDioIrqParams is 9. A
   * short reply buffer here overflowed the stack and the DIO1 callback
   * silently stopped arriving, which is how this was found. */
  uint8_t in[16];
  uint8_t frame[4];
  uint8_t tx[64];
  int major = -1, minor = -1;
  uint64_t now = 0;

  /* SetRx, so the chip is listening and a delivered frame has somewhere to go. */
  const uint8_t set_rx[4] = {0x82, 0xFF, 0xFF, 0xFF};
  /* SetDioIrqParams, everything unmasked onto DIO1. */
  const uint8_t set_dio[9] = {0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0};

  printf("virtual-sx1262 C ABI\n\n  every entry point, from C\n");

  vsx_abi_version(&major, &minor);
  check(major >= 1, "vsx_abi_version");

  chip = vsx_create();
  check(chip != NULL, "vsx_create");

  vsx_set_dio1_callback(chip, on_dio1, &dio1_edges);
  check(1, "vsx_set_dio1_callback");

  memset(in, 0, sizeof(in));
  vsx_spi_transaction(chip, set_dio, in, sizeof(set_dio));
  vsx_spi_transaction(chip, set_rx, in, sizeof(set_rx));
  check(1, "vsx_spi_transaction");

  check(vsx_busy(chip) == 0, "vsx_busy");
  check(vsx_dio1_asserted(chip) == 0, "vsx_dio1_asserted");

  vsx_tick(chip, ++now);
  check(1, "vsx_tick");

  vsx_set_channel_busy(chip, 1);
  vsx_set_channel_busy(chip, 0);
  check(1, "vsx_set_channel_busy");

  frame[0] = 0x11;
  frame[1] = 0x00;
  frame[2] = 0xAB;
  frame[3] = 0xCD;
  vsx_deliver_frame(chip, frame, sizeof(frame));
  vsx_tick(chip, ++now);
  check(dio1_edges == 1, "vsx_deliver_frame reaches the callback through C");

  vsx_transmit_finished(chip);
  check(1, "vsx_transmit_finished");

  /* The sync word, and a frame from a network this chip is not on. Checked
   * from C as well as C++ because the refusal is the whole point of the entry
   * point, and a host that gets an uint16_t argument wrong here would deliver
   * every frame instead of none. */
  {
    vsx_state s;
    vsx_get_state(chip, &s);
    check(s.sync_word == 0x1424, "vsx_get_state reports the sync word");
  }
  dio1_edges = 0;
  vsx_deliver_frame_from(chip, frame, sizeof(frame), 0x3444);
  {
    vsx_counters k;
    vsx_get_counters(chip, &k);
    check(k.sync_mismatches == 1, "vsx_deliver_frame_from refuses another network");
  }

  check(vsx_take_tx(chip, tx, sizeof(tx)) == 0, "vsx_take_tx with nothing to send");

  vsx_set_fem_enabled(chip, 1);
  vsx_set_last_signal(chip, -92.5f, -7.25f);
  check(1, "vsx_set_fem_enabled, vsx_set_last_signal");

  vsx_get_state(chip, &state);
  check(state.mode == 1, "vsx_get_state reports receive mode");
  /* The field appended in ABI 1.1. Checked from C because a struct that grew is
   * exactly where a host built against the old header and a library built
   * against the new one disagree, and two of the four hosts are not C++. */
  check(state.dio1_mask == 0xFFFF, "vsx_get_state reports the DIO1 mask");

  vsx_get_counters(chip, &counters);
  check(counters.frames_dropped == 0, "vsx_get_counters");

  check(vsx_est_airtime_ms(chip, 119) > 0, "vsx_est_airtime_ms");

  vsx_set_stuck_irq_ms(chip, 0);
  check(1, "vsx_set_stuck_irq_ms");

  vsx_set_strict_receive(chip, 0);
  check(vsx_frames_aborted(chip) == 0, "vsx_set_strict_receive and vsx_frames_aborted");

  /* Every call must tolerate a null handle, because a host that failed to
   * create one should get a quiet no-op rather than a crash inside the
   * library it just loaded. */
  vsx_set_dio1_callback(NULL, on_dio1, &dio1_edges);
  vsx_spi_transaction(NULL, set_rx, in, sizeof(set_rx));
  vsx_tick(NULL, 1);
  vsx_set_channel_busy(NULL, 1);
  vsx_deliver_frame(NULL, frame, sizeof(frame));
  vsx_deliver_frame_from(NULL, frame, sizeof(frame), 0x1424);
  vsx_deliver_frame_failed(NULL, frame, sizeof(frame), 1);
  vsx_frame_started(NULL, frame, sizeof(frame));
  vsx_transmit_finished(NULL);
  vsx_set_fem_enabled(NULL, 1);
  vsx_set_last_signal(NULL, 0.0f, 0.0f);
  vsx_get_state(NULL, &state);
  vsx_get_counters(NULL, &counters);
  check(vsx_busy(NULL) == 0, "a null handle is a no-op, not a crash");
  check(vsx_dio1_asserted(NULL) == 0, "vsx_dio1_asserted(NULL)");
  check(vsx_take_tx(NULL, tx, sizeof(tx)) == 0, "vsx_take_tx(NULL)");
  check(vsx_est_airtime_ms(NULL, 119) == 0, "vsx_est_airtime_ms(NULL)");
  vsx_set_strict_receive(NULL, 1);
  check(vsx_frames_aborted(NULL) == 0, "vsx_frames_aborted(NULL)");

  /* A transaction with no reply buffer is legal: most commands are writes. */
  vsx_spi_transaction(chip, set_rx, NULL, sizeof(set_rx));
  check(1, "vsx_spi_transaction with no reply buffer");

  vsx_spi_begin(NULL);
  vsx_spi_end(NULL);
  check(vsx_spi_byte(NULL, 0xC0) == 0, "the byte path tolerates a null handle");

  vsx_destroy(chip);
  vsx_destroy(NULL);
  check(1, "vsx_destroy, including of nothing");

  printf("\n  the byte path answers what the buffer path answers\n");
  byte_path_matches_buffer_path();

  printf("\n%s: %d failure(s)\n\n", failures ? "FAILED" : "PASSED", failures);
  return failures ? 1 : 0;
}
