/* virtual-sx1262 - where received frames land in the data buffer.
 *
 * Each frame goes straight after the last, so a firmware that reads late finds
 * the next one behind it, or over it once 256 bytes have arrived since. A
 * frame the host announces as its carrier begins also lands as it arrives: its
 * length from its header, its payload as its symbols do. A driver that trusts
 * the length it reads after the next header has been received gets the next
 * frame's, which is what the corruption seen on air looks like. See continuous.h
 * for what is modelled rather than documented.
 */
#include "VirtualSX1262.h"
#include "registers.h"

// A carrier this receiver's demodulator has started to follow. Only one at a
// time, as the part has one demodulator, and only in explicit header mode: an
// implicit-header frame has no length on the air to report early.
void VirtualSX1262::frameStarted(const uint8_t* frame, size_t len) {
  if (mode_ != kModeRx || !cont_.frame.empty() || len == 0 || headerImplicit_) {
    return;
  }
  cont_.frame.assign(frame, frame + (len > 255 ? 255 : len));
  cont_.sinceMs = nowMs_;
  cont_.header = false;
  cont_.written = 0;
}

// The followed frame's header and payload, at the instants they arrive. The
// header is where HeaderValid is; the payload is spread evenly over the rest of
// its airtime, which is close enough at a symbol's resolution and needs no
// second copy of the coding chain.
void VirtualSX1262::tickFollowing() {
  if (cont_.frame.empty()) return;
  if (mode_ != kModeRx) {
    dropFollowing();
    return;
  }
  const size_t len = cont_.frame.size();
  const double sinceMs = (double)(nowMs_ - cont_.sinceMs);
  const double headerMs =
      (preambleSyms_ + kSyncAndSfdSymbols + kHeaderBlockSymbols) * symbolMs();
  const double endMs = (double)estAirtimeMs((int)len);
  // Never handed over, so never locked onto: the host decided it was not
  // received, and the part would not have shown its header either. Judged at
  // the slowest coding rate, as the transmitter's need not be this receiver's.
  if (sinceMs > endMs * 8 / cr_ + (double)kInboxGraceFloorMs) {
    dropFollowing();
    return;
  }
  if (sinceMs < headerMs) return;
  if (!cont_.header) {
    memcpy(cont_.saved, buffer_, sizeof(buffer_));
    cont_.savedLen = rxLen_;
    cont_.savedCr = regs_[kRegLoRaRxCodingRate];
    cont_.savedCrc = regs_[kRegLoRaRxCrc];
    rxLen_ = (uint8_t)len;
    latchRxHeader(crcOn_);
    cont_.header = true;
  }
  size_t due = len;
  if (sinceMs < endMs && endMs > headerMs) {
    due = (size_t)((double)len * (sinceMs - headerMs) / (endMs - headerMs));
  }
  for (; cont_.written < due; cont_.written++) {
    buffer_[(uint8_t)(cont_.write + cont_.written)] = cont_.frame[cont_.written];
  }
}

// The followed frame came to nothing: what its header and payload replaced goes
// back, as the part never received them.
void VirtualSX1262::dropFollowing() {
  if (cont_.header) {
    for (size_t i = 0; i < cont_.written; i++) {
      const uint8_t at = (uint8_t)(cont_.write + i);
      buffer_[at] = cont_.saved[at];
    }
    rxLen_ = cont_.savedLen;
    regs_[kRegLoRaRxCodingRate] = cont_.savedCr;
    regs_[kRegLoRaRxCrc] = cont_.savedCrc;
  }
  cont_.frame.clear();
  cont_.header = false;
  cont_.written = 0;
}

// A frame received whole, good or not: it goes after the last one, and the
// start pointer moves to it. One the receiver was not following replaces
// whatever it was, as a stronger signal captures the demodulator.
void VirtualSX1262::writeReceived(const uint8_t* frame, size_t len) {
  const size_t n = len > 255 ? 255 : len;
  const bool followed =
      cont_.frame.size() == n && memcmp(cont_.frame.data(), frame, n) == 0;
  if (!followed) dropFollowing();
  for (size_t i = 0; i < n; i++) buffer_[(uint8_t)(cont_.write + i)] = frame[i];
  rxLen_ = (uint8_t)n;
  cont_.start = cont_.write;
  cont_.write = (uint8_t)(cont_.write + n);
  cont_.frame.clear();
  cont_.header = false;
  cont_.written = 0;
}
