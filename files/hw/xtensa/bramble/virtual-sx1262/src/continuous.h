#pragma once

// Where receive puts each frame in the data buffer, and the frame arriving now.
//
// Continuous receive writes each frame straight after the last (datasheet 7.2),
// and the start pointer GetRxBufferStatus reports moves to a frame when it
// completes. What the part reports while the next frame arrives is not
// documented. This models the case the corruption seen on air needs: the
// length follows the newest header from the moment it is received, and the
// payload lands as its symbols do, while the start pointer stays on the last
// completed frame.

#include <stdint.h>

#include <vector>

struct ContinuousRx {
  uint8_t write = 0;  // where the next frame's payload starts
  uint8_t start = 0;  // the last completed frame

  // The frame the receiver is following, from the start of its carrier.
  std::vector<uint8_t> frame;
  uint64_t sinceMs = 0;
  bool header = false;  // received: the length and header registers are this frame's
  size_t written = 0;   // payload bytes in the buffer so far

  // What its header and payload replaced, put back if it comes to nothing.
  uint8_t saved[256] = {0};
  uint8_t savedLen = 0, savedCr = 0, savedCrc = 0;
};
