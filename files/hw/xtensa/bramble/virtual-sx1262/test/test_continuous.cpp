/* Continuous receive: where each frame lands in the data buffer, and what a
 * driver that reads late finds there. */
#include "harness.h"

static std::vector<uint8_t> frame_of(size_t n, uint8_t seed) {
  std::vector<uint8_t> f(n);
  for (size_t i = 0; i < n; i++) f[i] = (uint8_t)(seed + i);
  return f;
}

static void clear_irq(vsx_chip* c) {
  const uint8_t clr[] = {0x02, 0xFF, 0xFF};
  vsx_spi_transaction(c, clr, nullptr, sizeof(clr));
}

/* Tick a millisecond at a time, as the hosts do. */
static void run_to(vsx_chip* c, uint64_t* now, uint64_t until) {
  while (*now < until) vsx_tick(c, ++*now);
}

/* SF8 at 62.5 kHz with the default 16-symbol preamble: HeaderValid 28.25
 * symbols of 4.096 ms in. */
static const uint64_t kHeaderMs = 116;

int main() {
  CASE("continuous receive puts each frame straight after the last");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const auto a = frame_of(20, 0x10), b = frame_of(30, 0x80);
    vsx_deliver_frame(c, a.data(), a.size());
    vsx_tick(c, ++now);
    clear_irq(c);
    vsx_deliver_frame(c, b.data(), b.size());
    vsx_tick(c, ++now);
    uint8_t len = 0, start = 0;
    rx_buffer_status(c, &len, &start);
    check(start == a.size() && len == b.size(),
          "the second starts where the first ended");
    std::vector<uint8_t> got(a.size());
    read_buffer(c, 0, got.data(), got.size());
    check(got == a, "and the first is still there to read");
    vsx_destroy(c);
  }

  CASE("entering receive fills the buffer from its base again");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const auto a = frame_of(20, 0x10);
    vsx_deliver_frame(c, a.data(), a.size());
    vsx_tick(c, ++now);
    set_standby(c);
    const uint8_t rx[] = {0x82, 0xFF, 0xFF, 0xFF};
    vsx_spi_transaction(c, rx, nullptr, sizeof(rx));
    vsx_deliver_frame(c, a.data(), a.size());
    vsx_tick(c, ++now);
    uint8_t len = 0, start = 0;
    rx_buffer_status(c, &len, &start);
    check(start == 0, "the next frame starts at the base");
    vsx_destroy(c);
  }

  CASE("a frame that runs past the end wraps, and ReadBuffer follows it");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const auto a = frame_of(200, 0x01), b = frame_of(100, 0x40);
    vsx_deliver_frame(c, a.data(), a.size());
    vsx_deliver_frame(c, b.data(), b.size());
    vsx_tick(c, ++now);
    vsx_tick(c, ++now);
    uint8_t len = 0, start = 0;
    rx_buffer_status(c, &len, &start);
    std::vector<uint8_t> got(len);
    read_buffer(c, start, got.data(), got.size());
    check(start == 200 && got == b, "the second frame reads back whole across the end");
    read_buffer(c, 0, got.data(), 44);
    check(std::memcmp(got.data(), b.data() + 56, 44) == 0,
          "and it has overwritten the first frame's opening 44 bytes");
    vsx_destroy(c);
  }

  CASE("a late read gets the next frame's length once its header has arrived");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const auto a = frame_of(20, 0x10), b = frame_of(21, 0x80);
    vsx_deliver_frame(c, a.data(), a.size());
    vsx_tick(c, ++now);
    const uint64_t began = now;
    vsx_frame_started(c, b.data(), b.size());
    run_to(c, &now, began + kHeaderMs - 2);
    uint8_t len = 0, start = 0;
    rx_buffer_status(c, &len, &start);
    check(len == a.size(), "before the next header, the length is still this frame's");
    run_to(c, &now, began + kHeaderMs + 1);
    rx_buffer_status(c, &len, &start);
    check(start == 0 && len == b.size(), "after it, the length is the next frame's");
    vsx_destroy(c);
  }

  CASE("the next frame's payload lands as its symbols do, after the last frame");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const auto a = frame_of(20, 0x10), b = frame_of(40, 0x80);
    vsx_deliver_frame(c, a.data(), a.size());
    vsx_tick(c, ++now);
    const uint64_t began = now;
    const uint64_t air = vsx_est_airtime_ms(c, (int)b.size());
    vsx_frame_started(c, b.data(), b.size());
    run_to(c, &now, began + (kHeaderMs + air) / 2);
    std::vector<uint8_t> got(b.size());
    read_buffer(c, (uint8_t)a.size(), got.data(), got.size());
    const bool started = got[0] == b[0];
    const bool unfinished = got[b.size() - 1] != b[b.size() - 1];
    check(started && unfinished,
          "half way through its payload, part of it is in the buffer");
    run_to(c, &now, began + air);
    vsx_deliver_frame(c, b.data(), b.size());
    vsx_tick(c, ++now);
    uint8_t len = 0, start = 0;
    rx_buffer_status(c, &len, &start);
    read_buffer(c, start, got.data(), got.size());
    check(start == a.size() && got == b,
          "and once handed over it is all there, after the last");
    vsx_destroy(c);
  }

  CASE("a frame sent at a slower coding rate is still there when it is handed over");
  /* The transmitter's coding rate need not be this receiver's: at 4/8 a frame
   * lasts about a quarter longer than this chip's own 4/5 airtime says. */
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const uint8_t cr45[] = {0x8B, 8, 0x03, 1, 0};
    vsx_spi_transaction(c, cr45, nullptr, sizeof(cr45));
    const auto a = frame_of(20, 0x10), b = frame_of(40, 0x80);
    vsx_deliver_frame(c, a.data(), a.size());
    vsx_tick(c, ++now);
    const uint64_t began = now;
    vsx_frame_started(c, b.data(), b.size());
    run_to(c, &now, began + vsx_est_airtime_ms(c, (int)b.size()) * 5 / 4);
    uint8_t len = 0, start = 0;
    rx_buffer_status(c, &len, &start);
    check(len == b.size(), "the length is still the arriving frame's");
    vsx_destroy(c);
  }

  CASE("an announced frame that is never handed over is taken back out");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const auto a = frame_of(20, 0x10), b = frame_of(40, 0x80);
    vsx_deliver_frame(c, a.data(), a.size());
    vsx_tick(c, ++now);
    const uint64_t began = now;
    vsx_frame_started(c, b.data(), b.size());
    run_to(c, &now, began + vsx_est_airtime_ms(c, (int)b.size()) * 2 + 100);
    uint8_t len = 0, start = 0, got[40] = {0};
    rx_buffer_status(c, &len, &start);
    read_buffer(c, (uint8_t)a.size(), got, sizeof(got));
    check(len == a.size(), "the length is the last frame's again");
    check(got[0] != b[0], "and what its payload overwrote is back");
    vsx_destroy(c);
  }

  CASE("an announced frame whose header fails is taken back out");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const auto a = frame_of(20, 0x10), b = frame_of(40, 0x80);
    vsx_deliver_frame(c, a.data(), a.size());
    vsx_tick(c, ++now);
    const uint64_t began = now;
    vsx_frame_started(c, b.data(), b.size());
    run_to(c, &now, began + kHeaderMs + 20);
    vsx_deliver_frame_failed(c, b.data(), b.size(), 2);
    uint8_t len = 0, start = 0;
    rx_buffer_status(c, &len, &start);
    check(len == a.size(), "the length is the last frame's again");
    vsx_destroy(c);
  }

  CASE("a receiver not in receive does not follow a frame");
  {
    vsx_chip* c = vsx_create();
    uint64_t now = 0;
    bring_up(c, &now);
    const auto a = frame_of(20, 0x10), b = frame_of(21, 0x80);
    vsx_deliver_frame(c, a.data(), a.size());
    vsx_tick(c, ++now);
    set_standby(c);
    const uint64_t began = now;
    vsx_frame_started(c, b.data(), b.size());
    run_to(c, &now, began + kHeaderMs + 20);
    uint8_t len = 0, start = 0;
    rx_buffer_status(c, &len, &start);
    check(len == a.size(), "the length stays the last frame's");
    vsx_destroy(c);
  }

  return report("");
}
