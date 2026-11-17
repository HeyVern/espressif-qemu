/* virtual-sx1262 - whether the receiver was listening for a frame.
 *
 * The part locks onto a frame during its preamble and then has to stay in
 * receive until the frame ends. A receiver that was in standby, in CAD or
 * transmitting while the preamble went by never locked, and one that left
 * receive after locking lost the frame there: SetStandby aborts a reception
 * rather than pausing it, and SetRx afterwards searches for a new preamble.
 *
 * Off by default. The emulated hosts advance this chip's clock at their own
 * rate, so a frame's start as this clock sees it is only approximately where
 * the engine put it, and those hosts keep the inbox grace instead. A host
 * whose clock is the engine's own switches this on.
 */
#include "VirtualSX1262.h"
#include "registers.h"

// When the receiver last went into receive, or the carrier began, whichever is
// later: detection can only count preamble symbols it was listening for.
uint64_t VirtualSX1262::listeningSinceMs() const {
  if (strictReceive_ && rxSinceMs_ > busySinceMs_) {
    return rxSinceMs_;
  }
  return busySinceMs_;
}

// Whether enough preamble was left, once the receiver was listening, for the
// detector to find it. Past that point what is on the air is sync word, header
// and payload, none of which looks like a preamble to the part.
bool VirtualSX1262::preambleWasHeard() const {
  if (!strictReceive_) {
    return true;
  }
  const double lateMs = (double)(listeningSinceMs() - busySinceMs_);
  const double spare = (double)preambleSyms_ - kPreambleSymbols;
  return spare > 0 && lateMs <= spare * symbolMs();
}

// Whether a frame ending now, len bytes long, was received from its preamble to
// its end. The engine hands a frame over as it ends, so its start is now less
// its airtime, and the receiver must have entered receive with a detectable
// preamble still to come and not left since. One millisecond of slack covers a
// host that steps its clock in whole milliseconds.
bool VirtualSX1262::caughtFrameOf(size_t len) const {
  if (mode_ != kModeRx) {
    return false;
  }
  const uint64_t air = estAirtimeMs((int)len);
  const double startMs = nowMs_ > air ? (double)(nowMs_ - air) : 0;
  const double spare = (double)preambleSyms_ - kPreambleSymbols;
  const double lockByMs = startMs + (spare > 0 ? spare : 0) * symbolMs() + 1;
  return (double)rxSinceMs_ <= lockByMs;
}

// Strict receive's answer to the inbox: a frame the receiver was not listening
// for goes in the bin as it arrives, with no grace, because the grace exists
// for a host whose clock is not the engine's and a strict host's is.
void VirtualSX1262::settleStrict() {
  while (!inbox.empty() && !caughtFrameOf(inbox.front().size())) {
    inbox.pop_front();
    framesAborted_++;
  }
  deliverPending();
}

// A frame the receiver detected and never locked onto: too weak to sync, or
// lost before its header. HeaderValid here was raised from how long the carrier
// had lasted, which is this model's stand-in for a header it cannot decode, and
// for this frame the stand-in was wrong, so it goes. PreambleDetected stays
// latched, as it does on silicon, which has no interrupt for a preamble that
// came to nothing: clearing it is the firmware's business. Strict receive only,
// because the lenient model clears both flags with the carrier anyway.
void VirtualSX1262::forgetHeader() {
  if (strictReceive_) {
    irq_ &= (uint16_t)~kIrqHeaderValid;
  }
}
