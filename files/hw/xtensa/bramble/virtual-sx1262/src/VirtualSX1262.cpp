/* The radio: what the part does between commands.
 *
 * Modes, the clock, carrier detection, reception, transmission and Semtech's
 * airtime formula. Nothing here knows how a command reached it; that is spi.cpp
 * next door. The split is not cosmetic - the wire protocol is a transcription
 * of a table and the radio is a model of a physical thing, and the two get
 * changed for different reasons.
 */
#include "VirtualSX1262.h"

#include "registers.h"

#include <vector>

#include <math.h>

VirtualSX1262::VirtualSX1262() {
  // The chip has to introduce itself. RadioLib reads sixteen bytes at
  // REG_VERSION_STRING and compares the first six against the chip type; a chip
  // that answers zeroes is retried ten times and then reported as not present,
  // which is exactly how this first failed to boot.
  //
  // The expected string for an SX1262 is "SX1261". That is not a mistake here
  // or in RadioLib: the part genuinely reports its family that way, and a
  // virtual chip that answered "SX1262" would be the one telling the lie.
  static const char kVersion[] = "SX1261";
  memcpy(&regs_[0x0320], kVersion, 6);

  // Receiver gain, which MeshCore reads back to report whether boosted mode is
  // on. Power-on default is the non-boosted value.
  regs_[0x08AC] = 0x94;

  // The sync word the part comes up holding. A firmware that never writes one
  // is on the private network word, and a model that came up on zero would
  // have every node agreeing with every other node about a value none of them
  // chose.
  regs_[kRegSyncWordMsb] = (uint8_t)(kSyncWordPrivate >> 8);
  regs_[kRegSyncWordLsb] = (uint8_t)(kSyncWordPrivate & 0xFF);

#ifdef VIRTUAL_SX1262_STUCK_IRQ_MS
  // A deliberately misbehaving chip, built as its own firmware variant.
  //
  // Real SX1262s sometimes latch the detection flags and refuse to clear them -
  // the "4 second lock-up" MeshCore's release notes describe. A driver that
  // trusts those flags then believes the channel is busy for ever and stops
  // transmitting; 1.17 exists to time them out and recover. On a chip that
  // behaves, 1.16 and 1.17 are indistinguishable, which is exactly what a
  // twelve-run sweep found. This variant makes the fault reproducible, so the
  // difference between the two versions can be measured rather than assumed.
  stuckIrqMs_ = VIRTUAL_SX1262_STUCK_IRQ_MS;
#endif
}

void VirtualSX1262::raiseIrq(uint16_t bits) {
  const uint16_t allowed = (uint16_t)(bits & irqMask_);
  if (allowed != bits) {
    irqSuppressed_++;
  }
  irq_ |= allowed;
}

// One symbol, in milliseconds, at whatever the firmware has programmed.
//
// Shared rather than recomputed because three things now time themselves in
// symbols - preamble detection, header detection and a CAD scan - and a second
// copy of this is a second place for the bandwidth to be forgotten.
double VirtualSX1262::symbolMs() const {
  return (double)(1u << sf_) / (bwKHz_ > 0 ? bwKHz_ : 250.0);
}

void VirtualSX1262::tick(uint64_t nowMs) {
  // Time with the busy flags up, which is what the firmware is reacting to.
  if ((irq_ & (kIrqPreambleDetected | kIrqHeaderValid)) && nowMs > lastBusyTickMs_) {
    busyMs_ += (uint32_t)(nowMs - lastBusyTickMs_);
  }
  lastBusyTickMs_ = nowMs;
  nowMs_ = nowMs;

  // When the queue went from empty to occupied, so the grace below can be
  // measured. The hosts push straight into `inbox`, so this is where a new
  // arrival is first seen.
  if (inbox.empty()) {
    inboxSinceMs_ = nowMs_;
  } else if (!inboxWasOccupied_) {
    inboxSinceMs_ = nowMs_;
  }
  inboxWasOccupied_ = !inbox.empty();

  // The fault, on builds compiled to have it: a detection interrupt that fires
  // with nothing on the air and then stays up.
  //
  // Clearing it works perfectly well - which is the whole point. A driver that
  // clears the flag when it has been set implausibly long recovers; one that
  // trusts it believes the channel is busy for ever and stops transmitting.
  // That is the difference between MeshCore 1.16 and 1.17, and it cannot be
  // seen on a chip that never lies.
  if (stuckIrqMs_ > 0 && mode_ == kModeRx && nowMs_ >= nextSpuriousMs_) {
    raiseIrq(kIrqPreambleDetected);
    spuriousRaises_++;
    nextSpuriousMs_ = nowMs_ + stuckIrqMs_;
  }
  settleInbox();

  // A scan in progress is the chip's own business and runs before the receive
  // path below bows out, because CAD is not receive mode and would otherwise
  // never be looked at again once it started.
  tickCad();

  // The receive deadline, which exists only for a firmware that asked for one.
  // Reaching it is a Timeout and a return to standby, exactly as a completed
  // reception would be, because a receiver that stops listening has stopped
  // listening whichever way it got there.
  if (mode_ == kModeRx && rxDeadlineArmed_ && nowMs_ >= rxDeadlineMs_) {
    rxDeadlineArmed_ = false;
    mode_ = kModeStandby;
    raiseIrq(kIrqTimeout);
  }

  // The transmit deadline, which the part enforces against a waveform that
  // never finished. The frame stays queued: the chip gave up on it, and whether
  // any of it reached the air is the simulator's to say, not ours.
  // A timeout goes to STDBY_RC whatever SetRxTxFallbackMode asked for: the
  // fallback is where a transmission that finished goes, and this one did not.
  if (mode_ == kModeTx && txDeadlineArmed_ && nowMs_ >= txDeadlineMs_) {
    txDeadlineArmed_ = false;
    mode_ = kModeStandby;
    raiseIrq(kIrqTimeout);
  }

  // Preamble and header detection, in receive mode only. A node cannot hear
  // anything while its own transmitter is keyed - that is half duplex, and the
  // engine already reports the channel as clear to a node that is transmitting.
  tickFollowing();
  if (mode_ != kModeRx) return;

  const double symMs = symbolMs();
  if (channelBusy_) {
    const double sinceMs = (double)(nowMs_ - busySinceMs_);
    const double heardMs = (double)(nowMs_ - listeningSinceMs());
    if (!preambleRaised_ && preambleWasHeard() && heardMs >= kPreambleSymbols * symMs) {
      raiseIrq(kIrqPreambleDetected);
      preambleRaised_ = true;
      preambleRaises_++;
      // StopTimerOnPreamble: a receiver that has heard something is no longer
      // on a deadline, so a bounded receive cannot cut off the frame it is in
      // the middle of.
      if (stopTimerOnPreamble_) {
        rxDeadlineArmed_ = false;
      }
    }
    // The receiver's own preamble length stands in for the transmitter's: a
    // mesh agrees on one, and the chip is told nothing about who is sending.
    // An implicit-header receiver has no header to find.
    //
    // HeaderValid alone: SyncWordValid is the GFSK modem's interrupt, and a LoRa
    // receiver reports its header through HeaderValid (DS_SX1261-2 table 13-29).
    // Raising both made every header count as suppressed on a firmware that
    // correctly leaves the FSK bit out of its mask.
    const double headerEnd = preambleSyms_ + kSyncAndSfdSymbols + kHeaderBlockSymbols;
    if (!headerImplicit_ && !headerRaised_ && (preambleRaised_ || !strictReceive_) &&
        sinceMs >= headerEnd * symMs) {
      raiseIrq(kIrqHeaderValid);
      headerRaised_ = true;
    }
  }
}

uint16_t VirtualSX1262::syncWord() const {
  return (uint16_t)(((uint16_t)regs_[kRegSyncWordMsb] << 8) | regs_[kRegSyncWordLsb]);
}

// A frame arriving from a transmitter whose sync word the caller knows.
//
// The sync word is what a LoRa receiver uses to decide a frame is not meant for
// it, and it was the one piece of the header this model could not act on: the
// firmware writes it through two registers rather than a command, so it landed
// in the register array and nothing ever read it back. Two meshes on one
// frequency with different words therefore heard each other perfectly, which is
// the opposite of the property the word exists to provide.
//
// Kept separate from deliverFrame() rather than folded into it, because a host
// that does not model sync words at all must go on working: no claim about the
// transmitter means no grounds to refuse, and the old entry point makes no
// claim. A host that does know says so here.
void VirtualSX1262::deliverFrameFrom(const uint8_t* frame, size_t len,
                                     uint16_t syncWordSent) {
  if (syncWordSent != syncWord()) {
    syncMismatches_++;
    return;
  }
  deliverFrame(frame, len);
}

void VirtualSX1262::setChannelBusy(bool busy) {
  if (busy && !channelBusy_) busySinceMs_ = nowMs_;
  if (!busy) {
    // The air went quiet: the signal that raised the preamble/header detection
    // flags is gone, so a well-behaved chip's detection state clears with it.
    // RxDone stays - that is a completed packet the firmware still has to read -
    // but the "a carrier is present" flags must not outlive the carrier, or the
    // driver's own channel-activity check (CustomSX1262::isReceiving reads
    // HEADER_VALID) reports the air busy for the ~4 s it takes that check to time
    // the stale flag out. On a mesh whose adverts arrive every few seconds that
    // window never closes, and a repeater that has a packet to forward never
    // sees a clear channel to send it on - which is why an emulated board
    // received every advert and relayed none.
    //
    // The deliberately-misbehaving STUCK_IRQ variant is exempt: leaving its
    // flags latched past the signal is the whole point of it, because that is
    // the real "4 second lock-up" MeshCore 1.17's driver exists to recover from.
    preambleRaised_ = false;
    headerRaised_ = false;
    if (stuckIrqMs_ == 0 && !strictReceive_) {
      irq_ &= (uint16_t) ~(kIrqPreambleDetected | kIrqHeaderValid);
    }
  }
  channelBusy_ = busy;
}

void VirtualSX1262::transmitFinished() {
  if (mode_ == kModeTx) {
    txDeadlineArmed_ = false;
    txContinuousWave_ = false;
    txInfinitePreamble_ = false;
    raiseIrq(kIrqTxDone);
    applyFallback();
  }
}

// How long a frame the chip could not take may wait for it to start listening.
//
// Derived from the signal rather than picked: the packet occupied the air for
// its airtime, so a receiver that starts listening inside that window was
// plausibly listening for part of the transmission and can be given it. Past
// it the carrier is definitively gone, and anything delivered later would be a
// packet arriving after the air that carried it went quiet.
//
// A fixed number of milliseconds was the first attempt and it was the wrong
// shape. The hosts advance this clock at their own rate - an emulated board
// runs on its own time, not the simulator's - so a constant that looks generous
// against one host is tight against another, and a frame that was genuinely
// receivable gets dropped.
uint64_t VirtualSX1262::inboxGraceMs() const {
  const uint64_t air = inbox.empty() ? 0 : estAirtimeMs((int)inbox.front().size());
  return air > kInboxGraceFloorMs ? air : kInboxGraceFloorMs;
}

// Whether the frame at the front of the inbox goes to the firmware or goes in
// the bin. Both of its callers ask the same question, so it is asked once here:
// tick(), as the clock moves, and startRx(), as the firmware arms the receiver.
//
// A frame the engine handed over while the chip was not listening used to sit in
// the inbox until the chip returned to receive, however long that took, so a
// packet could arrive seconds after the air that carried it went quiet. The
// engine has already ruled on half duplex before it delivers anything - it
// withholds what a node transmitted over - so this is the chip's own deafness,
// and it needs a bound rather than an open queue.
//
// The bound is a grace rather than an immediate drop on purpose: a driver's
// re-arm is standby, configure, SetRx, so mode_ is briefly 0 in the middle of a
// perfectly healthy receiver, and a frame landing in that gap is receivable.
//
// Only the deaf case has a deadline. Once the chip is listening the frame is
// handed over however long it waited, because the wait was the chip's own
// deafness and the grace exists to cover exactly that.
void VirtualSX1262::settleInbox() {
  if (inbox.empty()) return;
  if (strictReceive_) {
    settleStrict();
    return;
  }
  if (mode_ == kModeRx) {
    deliverPending();
    return;
  }
  if (nowMs_ - inboxSinceMs_ > inboxGraceMs()) {
    inbox.clear();
    framesDropped_++;
  }
}

void VirtualSX1262::deliverPending() {
  if (inbox.empty()) return;
  writeReceived(inbox.front().data(), inbox.front().size());
  inbox.pop_front();
  statRxPackets_++;
  latchRxHeader(crcOn_);
  raiseIrq(kIrqRxDone);
  endSingleShot();
  if (!nextFrame_.empty()) startNextFrame();
  // Deliberately only RxDone. The detection flags say "a carrier is present",
  // and by the time a frame is delivered the carrier has ended - raising them
  // here asserted the presence of a signal at the moment it stopped, which is
  // backwards. tick() raises them as the signal arrives, which is when they are
  // true, and setChannelBusy(false) clears them when it goes. Nothing needs
  // them at delivery: RadioLib reads the payload on RxDone alone, and the
  // driver's channel check reads them to ask "is something arriving now".
}

// Receiver noise, as a counter-based stream.
//
// Counter-based rather than a running generator so it is reproducible from the
// seed alone and shares no state with anything: the same node in the same run
// sees the same noise, and two nodes with different seeds see different noise.
// That second half is the point - firmware derives its identity from this, so a
// stream every node shares is a keypair every node shares.
//
// splitmix64, which is small, has no bad seeds, and passes the only test that
// matters here: consecutive counters give uncorrelated output, so the low bit
// of eight successive reads is eight usable bits rather than a pattern.
void VirtualSX1262::refreshNoise() {
  uint64_t z = noiseSeed_ + 0x9E3779B97F4A7C15ULL * (++noiseCounter_);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  z ^= z >> 31;
  noiseNow_ = (uint32_t)z;
}

void VirtualSX1262::startRx(uint32_t timeoutRaw) {
  // Entering receive starts filling the buffer from its base again.
  if (mode_ != kModeRx) {
    rxSinceMs_ = nowMs_;
    cont_.write = rxBase_;
  }
  mode_ = 1;

  // SetRx carries a 24-bit timeout, and two of its values are not durations.
  // All ones is continuous receive and zero is single-shot with no deadline, so
  // only what lies between them arms anything. A driver asking for continuous
  // and a driver asking for the longest finite timeout write different things
  // and mean different things, and collapsing the two is how a receiver that
  // should never stop acquires a deadline.
  rxDeadlineArmed_ = timeoutRaw != 0 && timeoutRaw != kRxContinuous;
  rxContinuous_ = timeoutRaw == kRxContinuous;
  if (rxDeadlineArmed_) {
    rxDeadlineMs_ = nowMs_ + (uint64_t)((double)timeoutRaw * kTimeoutStepMs + 0.5);
  } else if (timeoutRaw == 0 && symbNumTimeout_ > 0) {
    // SetLoRaSymbNumTimeout, which is the same deadline counted in symbols. It
    // only gets a look in when SetRx did not bring one of its own: the part
    // runs a single timer, and which of the two wins is the placeholder noted
    // in commands.cpp.
    rxDeadlineArmed_ = true;
    rxDeadlineMs_ = nowMs_ + (uint64_t)((double)symbNumTimeout_ * symbolMs() + 0.5);
  }

  // Arming the receiver is itself a delivery point, not just a change of mode.
  //
  // Delivery used to happen only on a tick, which quietly made every reception
  // depend on a tick landing after the firmware armed the receiver. A host that
  // ticks the model in its own process gets one almost immediately; a host whose
  // ticks arrive from a simulator over a socket may not, and the frame sat in
  // the inbox until the grace ran out and then went in the bin. Same model, same
  // seed, different answer depending on how the caller was plumbed, which is not
  // a property a chip is allowed to have.
  settleInbox();
}

void VirtualSX1262::startTx(uint32_t timeoutRaw) {
  mode_ = 2;

  // SetTx's timeout, which unlike SetRx's has no continuous case: a
  // transmission ends. Zero disables it and is what a driver content to wait
  // for TxDone sends; anything else is the part promising to give up. Reaching
  // it means the waveform never finished, which from inside the chip is
  // indistinguishable from a host that forgot to say it had.
  txDeadlineArmed_ = timeoutRaw != 0;
  if (txDeadlineArmed_) {
    txDeadlineMs_ = nowMs_ + (uint64_t)((double)timeoutRaw * kTimeoutStepMs + 0.5);
  }

  // Latched here because this is the instant it decides anything. RadioLib
  // raises the RF switch into transmit before issuing SetTx, so by now the line
  // carries the answer to "does this transmission reach the antenna".
  femAtTx_ = femEnabled_;
  hasTransmitted_ = true;
  pendingTx.assign(&buffer_[txBase_], &buffer_[txBase_] + txLenForSend_);
  hasPendingTx = true;
}

uint32_t VirtualSX1262::estAirtimeMs(int lenBytes) const {
  // Semtech's own airtime formula, from the parameters the firmware programmed.
  const double bwHz = bwKHz_ * 1000.0;
  const double tSym = (double)(1u << sf_) / bwHz;
  const int de = lowDataRateOptimize() ? 1 : 0;
  // IH is 1 when the header is implicit, which is the header being absent, so
  // it subtracts its twenty symbols. Both of these were constants standing in
  // for fields SetPacketParams carries.
  const int crc = crcOn_ ? 1 : 0, header = headerImplicit_ ? 1 : 0, crDen = cr_;
  double num = 8.0 * lenBytes - 4.0 * sf_ + 28 + 16 * crc - 20 * header;
  double den = 4.0 * (sf_ - 2 * de);
  double payloadSyms = 8 + fmax(ceil(num / den) * crDen, 0.0);
  double t = (preambleSyms_ + 4.25 + payloadSyms) * tSym;
  return (uint32_t)(t * 1000.0 + 0.5);
}

// The datasheet's LoRa bandwidth table, whole. Returns 0 for a code the part
// does not define, which is not the same as a bandwidth of zero and must not be
// adopted as one.
float VirtualSX1262::bandwidthForCode(uint8_t code) {
  switch (code) {
    case 0x00:
      return 7.81f;
    case 0x08:
      return 10.42f;
    case 0x01:
      return 15.63f;
    case 0x09:
      return 20.83f;
    case 0x02:
      return 31.25f;
    case 0x0A:
      return 41.67f;
    case 0x03:
      return 62.5f;
    case 0x04:
      return 125.0f;
    case 0x05:
      return 250.0f;
    case 0x06:
      return 500.0f;
    default:
      return 0.0f;
  }
}

// SetModulationParams: [sf][bw][cr][ldro].
//
// Every field is checked, and a field the part does not define is counted and
// left alone rather than adopted. Keeping the old value is the safe half of
// that and was already here; the counter is the half that was missing, because
// a rejected parameter used to be indistinguishable from one that was never
// sent. This model's own test harness had been programming bandwidth 0x1A,
// which is not a code, and quietly running every timing assertion at the
// default 250 kHz while its comment said 62.5.
void VirtualSX1262::applyModulation(const uint8_t* p, size_t n) {
  if (p[0] >= kSfMin && p[0] <= kSfMax) {
    sf_ = p[0];
  } else {
    paramsRejected_++;
  }

  const float bw = bandwidthForCode(p[1]);
  if (bw > 0.0f) {
    bwKHz_ = bw;
  } else {
    paramsRejected_++;
  }

  if (p[2] >= kCrMin && p[2] <= kCrMax) {
    cr_ = 4 + p[2];
  } else {
    paramsRejected_++;
  }

  // Low data rate optimisation is a field the firmware programs, not something
  // to infer. Inferring it was wrong at every bandwidth but 125 kHz, and wrong
  // in the figure the firmware asks this chip for to time its own CSMA: at
  // SF11 and 250 kHz it added two bits per symbol the part would not have
  // added, and quoted an airtime about an eighth too long.
  if (n >= 4) {
    ldroProgrammed_ = true;
    ldro_ = p[3] != 0;
  }
}

// Whether the modem is spending two bits a symbol on low data rate
// optimisation.
//
// The programmed field wins, because that is the one the part obeys. Only a
// firmware that never sent the field falls back to the symbol duration rule,
// which is the rule the chip would have applied to itself.
bool VirtualSX1262::lowDataRateOptimize() const {
  return ldroProgrammed_ ? ldro_ : (symbolMs() >= kLowDataRateSymbolMs);
}

// SetPacketParams: [preamble 15:8][preamble 7:0][headerType][payloadLen][crcOn]
// [invertIq].
//
// Three of the six used to be discarded, and estAirtimeMs assumed values for
// two of them. That is the same shape as the low data rate optimisation bug:
// a field the firmware programs, replaced by an assumption, in the figure the
// firmware times its own CSMA on. An implicit header is twenty symbols of
// payload the model was charging for and the part was not, and CRC off is
// sixteen bits it was charging for too.
void VirtualSX1262::applyPacketParams(const uint8_t* p) {
  preambleSyms_ = ((uint32_t)p[0] << 8) | p[1];
  headerImplicit_ = p[2] != 0;
  txLenForSend_ = p[3];
  crcOn_ = p[4] != 0;
  invertIq_ = p[5] != 0;
}
