/* The wire: how a command gets from a chip-select to the radio.
 *
 * Transaction framing, the byte-at-a-time shift a real SPI slave performs, and
 * the opcode decode. The awkward part, and the reason it is its own file: the
 * SX1262's protocol carries no length, so a slave cannot know a command has
 * ended until chip-select rises. This model therefore evaluates a growing
 * prefix on every byte and commits on the transaction boundary, which is what
 * lets a host that only has whole transactions and a host that has real
 * chip-select edges both drive the same code.
 */
#include "VirtualSX1262.h"

#include "registers.h"

#include <math.h>

// Commands that answer with data rather than just a status byte.
//
// All of them are reads, and none of them changes any state, which is what
// makes it safe to evaluate a growing prefix on every byte.
//
// An opcode missing from this list is not a small mistake. A host that shifts
// bytes one at a time - which is both emulators, since a real chip select is
// how they frame a transaction - gets 0x00 back for every data byte of that
// command, while a host that hands over a whole buffer gets the right answer.
// The same chip then behaves differently depending on how the caller is
// plumbed. GetRssiInst was missing, so every emulated board read its receiver
// as 0: no noise floor, no signal strength, and no entropy in a register the
// firmware reads eight times to seed its random number generator. Every board
// came up with the same identity because of this line.
static bool returnsData(uint8_t op) {
  switch (op) {
    case kReadBuffer:
    case kReadRegister:
    case kGetIrqStatus:
    case kGetRxBufferStatus:
    case kGetPacketStatus:
    case kGetRssiInst:
    case kGetStatus:
    case kGetDeviceErrors:
    case kGetPacketType:
    case kGetStats:
      return true;
    default:
      return false;
  }
}

void VirtualSX1262::beginTransaction() {
  txn_.clear();
  inTxn_ = true;
  // One draw per transaction, so a command reads the same whichever way the
  // host frames it, and eight successive reads are still eight fresh samples.
  refreshNoise();
}

uint8_t VirtualSX1262::transferByte(uint8_t out) {
  if (!inTxn_) beginTransaction();
  txn_.push_back(out);

  const size_t i = txn_.size() - 1;
  if (returnsData(txn_[0])) {
    // Evaluated against a padded command, not against the bytes received so
    // far. Several getters fill their reply only once the buffer is long
    // enough - GetPacketStatus writes in[2..4] under a length guard - so a
    // prefix would answer zero for a byte the buffer path answers properly.
    // Which byte carries which value does not depend on the length, only the
    // guard does, so padding gives the same answer the full command would.
    uint8_t scratch[260] = {0};
    uint8_t padded[260] = {0};
    const size_t have = txn_.size() < sizeof(padded) ? txn_.size() : sizeof(padded);
    memcpy(padded, txn_.data(), have);
    const size_t n = have > 16 ? have : 16;
    runCommand(padded, n, scratch);
    return i < sizeof(scratch) ? scratch[i] : 0;
  }
  // Everything else answers with the status byte in position 1 and nothing
  // else, exactly as the buffer path does.
  return i == 1 ? statusByte() : 0x00;
}

void VirtualSX1262::endTransaction() {
  inTxn_ = false;
  if (txn_.empty()) return;    // a chip select with no command in it
  if (returnsData(txn_[0])) {  // already evaluated, and it changed nothing
    if (txn_[0] == kGetRxBufferStatus && nextArmed_) applyNextFrame();
    txn_.clear();
    return;
  }
  uint8_t scratch[260] = {0};
  const size_t n = txn_.size() < sizeof(scratch) ? txn_.size() : sizeof(scratch);
  runCommand(txn_.data(), n, scratch);
  txn_.clear();
}

void VirtualSX1262::spiTransfer(const uint8_t* out, size_t len, uint8_t* in) {
  if (len == 0) return;
  refreshNoise();
  memset(in, 0, len);
  runCommand(out, len, in);
  // After the reply, so the read that arms failure 6 still sees the old length.
  if (out[0] == kGetRxBufferStatus && nextArmed_) applyNextFrame();
}

void VirtualSX1262::runCommand(const uint8_t* out, size_t len, uint8_t* in) {
  const uint8_t op = out[0];
  // Byte 1 of every reply is the status byte. RadioLib reads data from
  // buffIn[cmdLen + 1], so a one-byte command puts its first data byte at
  // in[2] - which is where the datasheet puts it too.
  auto status = [&](uint8_t v) {
    if (len > 1) in[1] = v;
  };
  status(statusByte());

  switch (op) {
    case kSetStandby:
      mode_ = 0;
      break;
    case kSetSleep:
      mode_ = 0;
      break;
    case kSetRx:
      // The timeout is the three bytes after the opcode. A command that arrives
      // without them is a firmware asking for single-shot receive, which is what
      // all-zero means, so a short command and an explicit zero agree.
      startRx(len >= 4 ? (((uint32_t)out[1] << 16) | ((uint32_t)out[2] << 8) | out[3])
                       : 0);
      break;
    case kSetTx:
      // The same three timeout bytes SetRx carries, on the same unit. All-zero
      // is a driver that will wait for TxDone however long it takes, which is
      // also what a command too short to carry the field means.
      startTx(len >= 4 ? (((uint32_t)out[1] << 16) | ((uint32_t)out[2] << 8) | out[3])
                       : 0);
      break;
    case kSetCad:
      startCad();
      break;

    case kSetRfFrequency:
      if (len >= 5) {
        uint32_t raw = ((uint32_t)out[1] << 24) | ((uint32_t)out[2] << 16) |
                       ((uint32_t)out[3] << 8) | out[4];
        // The datasheet's PLL step: freq = raw * 32e6 / 2^25.
        freqHz_ = (uint32_t)((double)raw * 32000000.0 / 33554432.0);
      }
      break;

    // The PA drive level, in dBm as a signed byte. Stored where it was
    // discarded before: the engine used to take transmit power from the board
    // profile alone, which is a datasheet figure and not a claim about what
    // this firmware asked for.
    case kSetTxParams:
      if (len >= 2) txPowerDbm_ = (int8_t)out[1];
      break;

    // Calibration returns the receive gain register to its reset default.
    //
    // Modelled rather than ignored because it is the mechanism behind the fault
    // MeshCore 1.17.1 fixed. sx126xResetAGC() runs a full CALIBRATE_ALL and then
    // re-applies the compile-time SX126X_RX_BOOSTED_GAIN macro - so a variant
    // that does not define the macro, generic-e22 among them, re-applies nothing
    // and boosted gain is gone until the node reboots. The firmware's own prefs
    // and its CLI go on reporting the setting the operator chose, so there is no
    // symptom anywhere except sensitivity.
    //
    // "May" is doing work in MeshCore's own comment for this - SX126xReset.h
    // calls it "RX settings that calibration may reset" - so what this models is
    // the chip behaving the way the firmware's authors assumed. Section 9.6 of
    // the SX126x datasheet is the authority, and it should be reconciled against
    // this before any sensitivity figure derived from it is published.
    case kCalibrate:
      regs_[kRegRxGain] = kRxGainPowerSaving;
      break;

    case kSetModulationParams:
      if (len >= 4) applyModulation(&out[1], len - 1);
      break;
    case kSetPacketParams:
      if (len >= 7) applyPacketParams(&out[1]);
      break;

    // The scan's parameters, which decide what a later SetCad costs and where
    // it leaves the chip. Ignored until now, so every scan ran with whatever
    // the reset defaults were however the firmware had configured it.
    case kSetCadParams:
      if (len >= 8) applyCadParams(&out[1]);
      break;
    case kSetBufferBase:
      if (len >= 3) {
        txBase_ = out[1];
        rxBase_ = out[2];
      }
      break;

    case kWriteBuffer:
      if (len >= 2) {
        // The address is eight bits, like every pointer the part reports, so
        // it wraps: continuous receive puts frames across the end of the buffer.
        uint8_t offset = out[1];
        for (size_t i = 2; i < len; i++) buffer_[(uint8_t)(offset + (i - 2))] = out[i];
      }
      break;

    case kReadBuffer:
      if (len >= 3) {
        uint8_t offset = out[1];
        for (size_t i = 3; i < len; i++) in[i] = buffer_[(uint8_t)(offset + (i - 3))];
      }
      break;

    case kWriteRegister:
      if (len >= 3) {
        uint16_t addr = ((uint16_t)out[1] << 8) | out[2];
        for (size_t i = 3; i < len && (size_t)addr + (i - 3) < sizeof(regs_); i++) {
          regs_[addr + (i - 3)] = out[i];
        }
      }
      break;

    case kReadRegister:
      if (len >= 4) {
        uint16_t addr = ((uint16_t)out[1] << 8) | out[2];
        for (size_t i = 4; i < len; i++) {
          size_t idx = (size_t)addr + (i - 4);
          if (idx >= kRegRandomNumber && idx < kRegRandomNumber + 4) {
            // The random number registers, which are noise and not storage.
            //
            // This is where firmware actually gets its entropy: RadioLib's
            // randomByte() reads this address eight times and keeps the low bit
            // of each, and MeshCore seeds its PRNG from that and derives its
            // identity from the PRNG. Backed by the register array like every
            // other address, it read zero every time, so the seed was zero and
            // every emulated node came up with the same keypair. Two nRF52
            // boards reporting one public key is what that looks like from
            // outside.
            in[i] = (uint8_t)(noiseBits(8) ^ (uint8_t)(idx - kRegRandomNumber));
          } else {
            in[i] = idx < sizeof(regs_) ? regs_[idx] : 0;
          }
        }
      }
      break;

    case kSetDioIrqParams:
      // [op][IrqMask][Dio1Mask][Dio2Mask][Dio3Mask], two bytes each, MSB first.
      //
      // Two masks, and they do different jobs: IrqMask says which events are
      // recorded in the IRQ status register at all, Dio1Mask says which of those
      // are wired out to the DIO1 pin. Reading the first and using it as the
      // second is the whole of a bug that made a board deaf - see the note on
      // irqAsserted().
      if (len >= 9) {
        irqMask_ = ((uint16_t)out[1] << 8) | out[2];
        dio1Mask_ = ((uint16_t)out[3] << 8) | out[4];
      }
      break;

    case kGetIrqStatus:
      // [op][status][irq hi][irq lo], table 13-30: four bytes, which is what
      // RadioLib clocks. The low byte answered only a fifth, so a driver
      // reading the flags never saw RxDone, CrcErr, HeaderValid or
      // PreambleDetected, and a CRC failure read as a good packet.
      irqReads_++;
      if (irq_ & (kIrqPreambleDetected | kIrqHeaderValid)) busyReads_++;
      if (len >= 3) in[2] = (uint8_t)(irq_ >> 8);
      if (len >= 4) in[3] = (uint8_t)(irq_ & 0xFF);
      break;

    case kClearIrqStatus:
      if (len >= 3) {
        uint16_t clear = ((uint16_t)out[1] << 8) | out[2];
        irq_ &= (uint16_t)~clear;
        // Under strict receive a carrier's preamble and header are detected
        // once: clearing the flag mid-frame does not put the preamble back on
        // the air, and re-raising it held a receiver busy for whole frames.
        if (!strictReceive_ && (clear & kIrqPreambleDetected)) preambleRaised_ = false;
        if (!strictReceive_ && (clear & kIrqHeaderValid)) headerRaised_ = false;
      }
      break;

    case kGetRxBufferStatus:
      // [op][status][payload len][start ptr], table 13-78
      if (len >= 3) in[2] = rxLen_;
      if (len >= 4) in[3] = cont_.start;
      break;

    case kGetPacketStatus:
      // RSSI and SNR as the datasheet encodes them, from what the engine
      // measured - the one place a virtual chip can be exactly right.
      // A byte is answerable as soon as the master clocks it: to receive
      // in[2] it sends three bytes, not four. The guards were each one too
      // strict, so signalRssiPkt came back as zero for the five-byte command
      // RadioLib actually issues - which the streaming path exposed, because
      // there the chip answers whatever is clocked and cannot consult a length
      // it has not been told.
      if (len >= 3) in[2] = (uint8_t)(-rssi_ * 2);
      if (len >= 4) in[3] = (uint8_t)(int8_t)(snr_ * 4);
      if (len >= 5) in[4] = (uint8_t)(-rssi_ * 2);
      break;

    case kGetRssiInst:
      // Half-dBm, with the receiver's own noise on the bottom of it.
      //
      // Without the dither this byte is -rssi*2, which is always even, so the
      // low bit RadioLib samples for entropy is always zero and every node
      // seeds its PRNG with 0. Two bits is about 1.5 dB of wander, which is the
      // right order for a receiver sitting on its noise floor and is what makes
      // eight reads of this register worth eight bits.
      if (len >= 3) {
        const int base = (int)(-rssi_ * 2);
        const int dithered = base + (int)noiseBits(2) - 1;
        in[2] = (uint8_t)(dithered < 0 ? 0 : (dithered > 255 ? 255 : dithered));
      }
      break;

    case kGetStatus:
      // The status byte is the reply, and status() above has already put it in
      // in[1]; a bare GetStatus is the one command whose whole answer it is.
      break;
    case kGetPacketType:
      if (len >= 3) in[2] = packetType_;
      break;
    case kGetDeviceErrors:
      // [op][status][errors hi][errors lo], table 13-84.
      if (len >= 3) in[2] = (uint8_t)(deviceErrors_ >> 8);
      if (len >= 4) in[3] = (uint8_t)(deviceErrors_ & 0xFF);
      break;
    case kClearDeviceErrors:
      break;
    default:
      // Everything the driver this model grew around never sends. A command
      // that is still nobody's lands here and is acknowledged, which is what
      // silicon does with an opcode it does not implement.
      runExtraCommand(op, out, len, in);
      break;
  }
}
