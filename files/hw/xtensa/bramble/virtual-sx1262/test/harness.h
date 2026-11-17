/* The bench these tests are written on.
 *
 * A header rather than a library because there is no test framework here, per
 * the one rule: check() prints a line and counts a failure, which is all a
 * library this size needs. Each test binary gets its own copy of the counter,
 * which is what makes them independent.
 */
#ifndef VIRTUAL_SX1262_TEST_HARNESS_H
#define VIRTUAL_SX1262_TEST_HARNESS_H

#include "virtual_sx1262.h"

/* The class as well as the ABI: the byte-at-a-time path below is how the
 * emulators drive this chip and it is not reachable through the C surface. */
#include "VirtualSX1262.h"

#include <cstdio>
#include <cstring>
#include <vector>

static inline int g_failures = 0;
static const char* g_case = "";

#define CASE(name)                 \
  do {                             \
    g_case = name;                 \
    std::printf("\n  %s\n", name); \
  } while (0)

static inline void check(bool ok, const char* what) {
  std::printf("    %-4s %s\n", ok ? "ok" : "FAIL", what);
  if (!ok) {
    ++g_failures;
  }
}

/* SX1262 IRQ bits, as the datasheet numbers them. */
enum {
  IRQ_TX_DONE = 1u << 0,
  IRQ_RX_DONE = 1u << 1,
  IRQ_PREAMBLE = 1u << 2,
  IRQ_SYNC_WORD = 1u << 3,
  IRQ_HEADER = 1u << 4,
  IRQ_HEADER_ERR = 1u << 5,
  IRQ_CRC_ERR = 1u << 6,
  IRQ_CAD_DONE = 1u << 7,
  IRQ_CAD_DETECTED = 1u << 8,
  IRQ_TIMEOUT = 1u << 9,
};

/* Put the chip in a state a driver would recognise: LoRa at SF8/62.5 kHz, every
 * IRQ unmasked onto DIO1, and receiving.
 *
 * Deliberately more generous than RadioLib, which routes only RxDone to the
 * pin, so the cases below can talk about one flag at a time without restating a
 * mask each time. That generosity is also how a confusion between the two masks
 * in SetDioIrqParams passed every test here for months, so the case named "only
 * what is routed to DIO1 raises DIO1" sends the real pair instead. */
static inline void bring_up(vsx_chip* c, uint64_t* now) {
  /* SetModulationParams: sf, bw, cr, ldro. 0x03 is the 62.5 kHz code.
   *
   * It said 0x1A for a long time, which is not a code at all: the model kept
   * whatever bandwidth it already had, so every case here that talks about
   * symbols was silently timed at the 250 kHz default while the comment said
   * 62.5. Nothing failed, because nothing asserted a number that only holds at
   * one of the two. The model now counts a parameter it had to refuse, which is
   * what makes the next one of these visible. */
  const uint8_t mod[] = {0x8B, 8, 0x03, 4, 0};
  vsx_spi_transaction(c, mod, nullptr, sizeof(mod));

  /* SetDioIrqParams: mask, dio1, dio2, dio3 - unmask everything onto DIO1. */
  const uint8_t dio[] = {0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0};
  vsx_spi_transaction(c, dio, nullptr, sizeof(dio));

  /* SetRx, timeout infinite. */
  const uint8_t rx[] = {0x82, 0xFF, 0xFF, 0xFF};
  vsx_spi_transaction(c, rx, nullptr, sizeof(rx));

  vsx_tick(c, ++*now);
}

static inline uint16_t irq_flags(vsx_chip* c) {
  vsx_state s;
  vsx_get_state(c, &s);
  return s.irq_flags;
}

static inline uint8_t mode_of(vsx_chip* c) {
  vsx_state s;
  vsx_get_state(c, &s);
  return s.mode;
}

static inline void set_standby(vsx_chip* c) {
  const uint8_t sb[] = {0x80, 0};
  vsx_spi_transaction(c, sb, nullptr, sizeof(sb));
}

/* ReadRegister: [op][addr 15:8][addr 7:0][status][data]. */
static inline uint8_t read_reg(vsx_chip* c, uint16_t addr) {
  const uint8_t tx[] = {0x1D, (uint8_t)(addr >> 8), (uint8_t)addr, 0, 0};
  uint8_t rx[sizeof(tx)] = {0};
  vsx_spi_transaction(c, tx, rx, sizeof(tx));
  return rx[4];
}

/* GetRxBufferStatus: [op][status][payload length][start pointer]. */
static inline void rx_buffer_status(vsx_chip* c, uint8_t* len, uint8_t* start) {
  const uint8_t tx[] = {0x13, 0, 0, 0};
  uint8_t rx[sizeof(tx)] = {0};
  vsx_spi_transaction(c, tx, rx, sizeof(tx));
  *len = rx[2];
  *start = rx[3];
}

/* ReadBuffer: [op][offset][status][data...]. */
static inline void read_buffer(vsx_chip* c, uint8_t offset, uint8_t* out, size_t n) {
  std::vector<uint8_t> tx(3 + n, 0), rx(3 + n, 0);
  tx[0] = 0x1E;
  tx[1] = offset;
  vsx_spi_transaction(c, tx.data(), rx.data(), tx.size());
  std::memcpy(out, rx.data() + 3, n);
}

/* Every binary ends the same way: say what happened, and answer the shell. */
static inline int report(const char* what) {
  std::printf("\n%s: %d failure(s)\n\n", g_failures ? "FAILED" : "PASSED", g_failures);
  (void)what;
  return g_failures ? 1 : 0;
}

#endif /* VIRTUAL_SX1262_TEST_HARNESS_H */
