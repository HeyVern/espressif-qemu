/* The rest of the command set: the commands MeshCore never issues.
 *
 * Its own file because the reason all of these were missing is the same, and it
 * is not a good reason: the model grew to the shape of one firmware's driver,
 * so a command nothing in that path sends was a command that silently did
 * nothing. A host would issue it, get a status byte back, and carry on with the
 * chip in a state it had not asked for.
 *
 * Three kinds of thing live here, and each one says which it is:
 *
 *   ACTED ON    the command changes what the model does.
 *   RECORDED    the command's parameters are stored and reported, and nothing
 *               here acts on them, because acting would mean inventing physics
 *               this model is not given. Recording is still the point: the
 *               fault worth catching is a firmware configured differently from
 *               how its operator believes, and a value nobody can read back is
 *               a value nobody can check.
 *   PLACEHOLDER the command is decoded but its effect is not modelled, and the
 *               comment says what would have to exist first.
 *
 * Nothing here is a stub that pretends. A command that is RECORDED does not
 * report success at something it did not do.
 */
#include "VirtualSX1262.h"

#include "registers.h"

// SetPaConfig. RECORDED.
//
// This is half of what sets output power on silicon, the other half being
// SetTxParams, and the two are a pair: the datasheet gives four configurations
// for +14, +17, +20 and +22 dBm and the duty cycle must not be raised beyond
// the one for the power asked for. Turning the pair into an actual figure at
// the antenna is a board question as much as a chip one, so it belongs to
// whatever knows the board. Reported so that it can be.
void VirtualSX1262::applyPaConfig(const uint8_t* p) {
  paDutyCycle_ = p[0];
  paHpMax_ = p[1];
  paDeviceSel_ = p[2];
  paLut_ = p[3];
}

// SetRxDutyCycle. PLACEHOLDER.
//
// The part alternates receive and sleep on its own timers, waking to listen for
// rxPeriod and sleeping for sleepPeriod, and a preamble seen during a listening
// window holds it awake. Modelling it means a receiver that is deaf for a
// scheduled fraction of the time, which changes which frames arrive at all, and
// that is a decision about what a node hears: the engine already owns that, and
// two things deciding it would be worse than one thing deciding it badly.
//
// So the periods are recorded and the chip stays in continuous receive. A
// firmware using duty-cycled receive will therefore appear to hear everything,
// which is the optimistic direction and must not be read as a result about
// power saving.
void VirtualSX1262::applyRxDutyCycle(const uint8_t* p) {
  rxDutyRxPeriod_ = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
  rxDutySleepPeriod_ = ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 8) | p[5];
  startRx(0);
}

// Where the part goes when a transmission or reception finishes. ACTED ON, as
// far as this model has modes to go to.
//
// PLACEHOLDER for FS. The part distinguishes STDBY_RC, STDBY_XOSC and FS, and
// this model has one standby: the difference between them is start-up latency
// and current draw, neither of which it represents. A firmware that asks to
// fall back to FS gets standby, and the mode it asked for is readable through
// fallbackMode() so a host can tell that is what happened.
void VirtualSX1262::applyFallback() {
  mode_ = kModeStandby;
}

// GetStatus' byte. ACTED ON, where it used to be the constant 0x22.
//
// Bits 6:4 are the chip mode and 3:1 the command status. It answered STBY_RC
// whatever the part was doing, so a firmware polling this to find out whether
// its receiver was still armed was told "standby" by a chip in receive. The
// command status is the weaker half: this model has no failing commands, so it
// reports data available when there is a frame waiting and completed otherwise,
// which is true but narrower than the five values the part can return.
uint8_t VirtualSX1262::statusByte() const {
  uint8_t chipMode = kStatusModeStandbyRc;
  switch (mode_) {
    case kModeRx:
      chipMode = kStatusModeRx;
      break;
    case kModeTx:
      chipMode = kStatusModeTx;
      break;
    case kModeCad:
      // CAD is not one of the modes the status byte names. The part is
      // receiving while it scans, and that is the honest answer of the ones
      // available.
      chipMode = kStatusModeRx;
      break;
    default:
      chipMode = kStatusModeStandbyRc;
      break;
  }
  const uint8_t cmd = rxLen_ > 0 ? kStatusCmdDataAvailable : kStatusCmdCompleted;
  return (uint8_t)((chipMode << 4) | (cmd << 1));
}

// The header of a frame that was received carries the transmitter's coding rate
// and whether a CRC follows the payload, and the part keeps both for the driver
// to read. Every frame the simulator hands over says the same as this receiver
// is set to, because a mismatch would not have been received at all; only
// failure 4 says otherwise.
void VirtualSX1262::latchRxHeader(bool crcOn) {
  regs_[kRegLoRaRxCodingRate] = (uint8_t)(((cr_ - 4) & 0x07) << 4);
  regs_[kRegLoRaRxCrc] = crcOn ? 0x10 : 0x00;
}

// Single mode, with or without a timeout, returns to STDBY_RC after a packet
// (datasheet 13.1.5, table 13-9). Only continuous receive keeps listening, and a driver
// that asked for one packet must not be handed a second it never re-armed for.
void VirtualSX1262::endSingleShot() {
  if (!rxContinuous_) {
    mode_ = kModeStandby;
    rxDeadlineArmed_ = false;
  }
}

// Continuous receive writes each packet straight after the last one in the
// buffer (datasheet 7.2). What the part reports while the next one arrives is
// not documented; this models the case the corruption seen on air needs: the
// payload length follows the newest header while the start pointer stays on the
// last completed packet. A host uses failures 5 and 6 to test a driver against
// that, not as a statement of how the part behaves.
void VirtualSX1262::startNextFrame() {
  if (mode_ != kModeRx || !rxContinuous_) {
    nextFrame_.clear();  // a receiver that stopped never heard it
    framesAborted_++;
    return;
  }
  if (nextWhen_ == kReceiveNextBeforeRead) {
    applyNextFrame();
  } else {
    nextArmed_ = true;
  }
}

void VirtualSX1262::applyNextFrame() {
  const uint8_t at = cont_.write;
  const size_t n = nextFrame_.size() > 255 ? 255 : nextFrame_.size();
  for (size_t i = 0; i < n; i++) buffer_[(uint8_t)(at + i)] = nextFrame_[i];
  rxLen_ = (uint8_t)n;
  nextFrame_.clear();
  nextArmed_ = false;
}

// A frame the receiver got and could not trust. See the header for why the
// interface is a placeholder: the chip cannot decide this, and nothing calls it
// yet.
void VirtualSX1262::deliverFrameFailed(const uint8_t* frame, size_t len,
                                       uint8_t failure) {
  if (failure == kReceiveNextBeforeRead || failure == kReceiveNextDuringRead) {
    nextFrame_.assign(frame, frame + len);
    nextWhen_ = failure;
    if (inbox.empty() && (irq_ & kIrqRxDone)) startNextFrame();
    return;
  }
  if (failure == kReceiveNoLock) {
    forgetHeader();
    dropFollowing();
    return;
  }
  if (mode_ != kModeRx) {
    return;
  }
  if (strictReceive_ && !caughtFrameOf(len)) {
    framesAborted_++;
    return;
  }
  if (failure == kReceiveHeaderError) {
    // A header that failed was never valid: withdraw the HeaderValid the
    // carrier raised, or a driver reads HeaderErr beside it as a good packet.
    forgetHeader();
    dropFollowing();
    statHeaderErrors_++;
    raiseIrq(kIrqHeaderErr);
    return;
  }
  writeReceived(frame, len);
  if (failure == kReceiveNoCrc) {
    // The header said there is no CRC, so the part checks nothing and the
    // frame is as good as any other to the firmware. Only the header register
    // tells the two apart.
    statRxPackets_++;
    latchRxHeader(false);
    raiseIrq(kIrqRxDone);
    endSingleShot();
    return;
  }
  // A CRC failure is a frame that arrived: the payload is in the buffer and
  // RxDone is raised beside CrcErr, because that is what the part does and a
  // driver that reads the buffer on RxDone must find the corrupt bytes there
  // rather than the last good frame's.
  statCrcErrors_++;
  latchRxHeader(crcOn_);
  raiseIrq(kIrqRxDone | kIrqCrcErr);
  endSingleShot();
}

void VirtualSX1262::deliverFrame(const uint8_t* frame, size_t len) {
  inbox.emplace_back(frame, frame + len);
}

// The decoder for everything above, returning false for an opcode that is still
// nobody's. Split from runCommand()'s switch so that the commands a real driver
// issues stay readable as one table.
bool VirtualSX1262::runExtraCommand(uint8_t op, const uint8_t* out, size_t len,
                                    uint8_t* in) {
  switch (op) {
    // ACTED ON. The chip is one packet type at a time, and GetPacketType now
    // answers what was set rather than always LoRa. Only LoRa has a data path
    // here: a firmware that switches to GFSK or LR-FHSS will find the type
    // reported back correctly and the modem still behaving as LoRa, which is
    // recorded in packetType() so a host can refuse to believe the run.
    case kSetPacketType:
      if (len >= 2) packetType_ = out[1];
      return true;

    // RECORDED.
    case kSetPaConfig:
      if (len >= 5) applyPaConfig(&out[1]);
      return true;

    // ACTED ON at the end of a transmission or reception, with FS collapsed
    // into standby. See applyFallback().
    case kSetRxTxFallbackMode:
      if (len >= 2) fallbackMode_ = out[1];
      return true;

    // ACTED ON. A symbol-count timeout is the same deadline SetRx's millisecond
    // one is, measured in the unit the firmware chose to think in. Zero
    // disables it, which is the reset value.
    //
    // PLACEHOLDER for the interaction with SetRx's own timeout: the part runs
    // one timer, and which of the two wins when a firmware sets both is not
    // something this model has been able to check against silicon. Here the
    // millisecond timeout wins if it is armed, because that is the one
    // RadioLib actually sends.
    case kSetLoRaSymbNumTimeout:
      if (len >= 2) symbNumTimeout_ = out[1];
      return true;

    // ACTED ON. With the timer stopped on preamble, a receiver that has heard
    // something is no longer on a deadline: it waits for the frame. This is the
    // setting that stops a bounded receive from cutting off a packet it is in
    // the middle of.
    case kStopTimerOnPreamble:
      if (len >= 2) stopTimerOnPreamble_ = out[1] != 0;
      return true;

    // PLACEHOLDER. See applyRxDutyCycle().
    case kSetRxDutyCycle:
      if (len >= 7) applyRxDutyCycle(&out[1]);
      return true;

    // ACTED ON, as far as there is anything to act on: frequency synthesis is
    // not receiving and not transmitting, so it is this model's standby. The
    // part distinguishes them by start-up latency, which is not represented.
    case kSetFs:
      mode_ = kModeStandby;
      return true;

    // RECORDED, and deliberately not acted on.
    //
    // Both of these key the transmitter with no frame behind it: a bare carrier
    // and an endless preamble, which exist for regulatory measurement. They are
    // a transmission this model has no way to hand to the engine, because
    // pendingTx carries frames and there is no frame. A host wanting to model an
    // interferer reads txContinuousWave() and puts a carrier on the air itself.
    case kSetTxContinuousWave:
      txContinuousWave_ = true;
      mode_ = kModeTx;
      return true;
    case kSetTxInfinitePreamble:
      txInfinitePreamble_ = true;
      mode_ = kModeTx;
      return true;

    // RECORDED. DC-DC against LDO is a current-draw decision, and this model
    // has no current.
    case kSetRegulatorMode:
      if (len >= 2) regulatorMode_ = out[1];
      return true;

    // RECORDED, and the first one matters more than it looks.
    //
    // With DIO2 driving the RF switch, the chip raises the transmit line itself
    // rather than the firmware doing it as a GPIO. This model has the line the
    // other way round, through setFemEnabled(), because that is how MeshCore
    // drives it. A firmware that hands the job to DIO2 would therefore look like
    // one that never raises the line at all, and femAtTx() would dock it for a
    // fault it does not have.
    //
    // PLACEHOLDER: nothing reconciles the two. dio2AsRfSwitch() is reported so a
    // host can notice the case, and the honest fix is for the chip to drive
    // femAtTx_ itself when this is set, which needs a host that does not also
    // drive it.
    case kSetDio2AsRfSwitchCtrl:
      if (len >= 2) dio2AsRfSwitch_ = out[1] != 0;
      return true;
    case kSetDio3AsTcxoCtrl:
      dio3AsTcxo_ = true;
      return true;

    // RECORDED as a no-op. Image calibration is an analogue procedure with no
    // register this model keeps, unlike Calibrate, which has one.
    case kCalibrateImage:
      return true;

    // ACTED ON. [op][nop][status][pktRx 15:8][pktRx 7:0][crcErr..][hdrErr..]
    //
    // Two of the three can only move through deliverFrameFailed(), which
    // nothing calls, so a firmware reading this will see received packets climb
    // and both error counts sit at zero however bad the channel is.
    case kGetStats:
      if (len >= 4) in[2] = (uint8_t)(statRxPackets_ >> 8);
      if (len >= 5) in[3] = (uint8_t)(statRxPackets_ & 0xFF);
      if (len >= 6) in[4] = (uint8_t)(statCrcErrors_ >> 8);
      if (len >= 7) in[5] = (uint8_t)(statCrcErrors_ & 0xFF);
      if (len >= 8) in[6] = (uint8_t)(statHeaderErrors_ >> 8);
      if (len >= 9) in[7] = (uint8_t)(statHeaderErrors_ & 0xFF);
      return true;

    case kResetStats:
      // The opcode and six trailing bytes form the complete reset command.
      if (len < 7) return true;
      statRxPackets_ = 0;
      statCrcErrors_ = 0;
      statHeaderErrors_ = 0;
      return true;

    default:
      return false;
  }
}
