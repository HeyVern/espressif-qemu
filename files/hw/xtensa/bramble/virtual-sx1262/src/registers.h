/* The SX1262's own tables, transcribed.
 *
 * Separate from both the radio behaviour and the wire protocol because it is
 * neither: it is the datasheet, and the two implementation files below each
 * need about half of it. Kept complete rather than trimmed to what is used
 * today, because a register map with holes in it is harder to check against
 * the part than a full one, and the next opcode this model learns is already
 * named here. That is what [[maybe_unused]] says: these are documentation as
 * much as code, so an unused one is not dead code to be deleted.
 *
 * Source: Semtech SX1261/2 datasheet rev 2.1, tables 11-1 (opcodes) and 13-29
 * (IRQ flags).
 */
#ifndef VIRTUAL_SX1262_REGISTERS_H
#define VIRTUAL_SX1262_REGISTERS_H

#include <stdint.h>

namespace {
[[maybe_unused]] constexpr uint8_t kSetSleep = 0x84;
[[maybe_unused]] constexpr uint8_t kSetStandby = 0x80;
[[maybe_unused]] constexpr uint8_t kSetTx = 0x83;
[[maybe_unused]] constexpr uint8_t kSetRx = 0x82;
[[maybe_unused]] constexpr uint8_t kSetCad = 0xC5;
[[maybe_unused]] constexpr uint8_t kSetRfFrequency = 0x86;
[[maybe_unused]] constexpr uint8_t kSetPacketType = 0x8A;
[[maybe_unused]] constexpr uint8_t kGetPacketType = 0x11;
[[maybe_unused]] constexpr uint8_t kSetTxParams = 0x8E;
[[maybe_unused]] constexpr uint8_t kSetModulationParams = 0x8B;
[[maybe_unused]] constexpr uint8_t kSetPacketParams = 0x8C;
[[maybe_unused]] constexpr uint8_t kSetCadParams = 0x88;
[[maybe_unused]] constexpr uint8_t kCalibrate = 0x89;
[[maybe_unused]] constexpr uint8_t kSetBufferBase = 0x8F;
[[maybe_unused]] constexpr uint8_t kWriteBuffer = 0x0E;
[[maybe_unused]] constexpr uint8_t kReadBuffer = 0x1E;
[[maybe_unused]] constexpr uint8_t kWriteRegister = 0x0D;
[[maybe_unused]] constexpr uint8_t kReadRegister = 0x1D;
[[maybe_unused]] constexpr uint8_t kSetDioIrqParams = 0x08;
[[maybe_unused]] constexpr uint8_t kGetIrqStatus = 0x12;
[[maybe_unused]] constexpr uint8_t kClearIrqStatus = 0x02;
[[maybe_unused]] constexpr uint8_t kGetRxBufferStatus = 0x13;
[[maybe_unused]] constexpr uint8_t kGetPacketStatus = 0x14;
[[maybe_unused]] constexpr uint8_t kGetStatus = 0xC0;
[[maybe_unused]] constexpr uint8_t kGetDeviceErrors = 0x17;
[[maybe_unused]] constexpr uint8_t kClearDeviceErrors = 0x07;
[[maybe_unused]] constexpr uint8_t kGetRssiInst = 0x15;

// IRQ bits.
[[maybe_unused]] constexpr uint16_t kIrqTxDone = 1 << 0;
[[maybe_unused]] constexpr uint16_t kIrqRxDone = 1 << 1;
[[maybe_unused]] constexpr uint16_t kIrqPreambleDetected = 1 << 2;
[[maybe_unused]] constexpr uint16_t kIrqSyncWordValid = 1 << 3;
[[maybe_unused]] constexpr uint16_t kIrqHeaderValid = 1 << 4;
[[maybe_unused]] constexpr uint16_t kIrqHeaderErr = 1 << 5;
[[maybe_unused]] constexpr uint16_t kIrqCrcErr = 1 << 6;
[[maybe_unused]] constexpr uint16_t kIrqCadDone = 1 << 7;
[[maybe_unused]] constexpr uint16_t kIrqCadDetected = 1 << 8;
[[maybe_unused]] constexpr uint16_t kIrqTimeout = 1 << 9;

// The rest of the command set. Declared together because the reason each one
// was missing is the same: nothing in MeshCore's path issues it, so the model
// grew only what it was asked for. What each one now does is in commands.cpp,
// and several of them record rather than act. That is marked there.
[[maybe_unused]] constexpr uint8_t kSetFs = 0xC1;
[[maybe_unused]] constexpr uint8_t kSetTxContinuousWave = 0xD1;
[[maybe_unused]] constexpr uint8_t kSetTxInfinitePreamble = 0xD2;
[[maybe_unused]] constexpr uint8_t kSetRegulatorMode = 0x96;
[[maybe_unused]] constexpr uint8_t kCalibrateImage = 0x98;
[[maybe_unused]] constexpr uint8_t kSetPaConfig = 0x95;
[[maybe_unused]] constexpr uint8_t kSetRxTxFallbackMode = 0x93;
[[maybe_unused]] constexpr uint8_t kSetRxDutyCycle = 0x94;
[[maybe_unused]] constexpr uint8_t kStopTimerOnPreamble = 0x9F;
[[maybe_unused]] constexpr uint8_t kSetLoRaSymbNumTimeout = 0xA0;
[[maybe_unused]] constexpr uint8_t kSetDio2AsRfSwitchCtrl = 0x9D;
[[maybe_unused]] constexpr uint8_t kSetDio3AsTcxoCtrl = 0x97;
[[maybe_unused]] constexpr uint8_t kGetStats = 0x10;
[[maybe_unused]] constexpr uint8_t kResetStats = 0x00;

// Packet types, from SetPacketType. Only LoRa has a data path in this model.
[[maybe_unused]] constexpr uint8_t kPacketTypeGfsk = 0x00;
[[maybe_unused]] constexpr uint8_t kPacketTypeLora = 0x01;
[[maybe_unused]] constexpr uint8_t kPacketTypeLrFhss = 0x03;

// Where the part goes when a transmission or reception ends, from
// SetRxTxFallbackMode.
[[maybe_unused]] constexpr uint8_t kFallbackFs = 0x40;
[[maybe_unused]] constexpr uint8_t kFallbackStandbyXosc = 0x30;
[[maybe_unused]] constexpr uint8_t kFallbackStandbyRc = 0x20;

// The chip modes this model distinguishes, which are fewer than the part has.
// See the note on kModeFs in VirtualSX1262.h.
[[maybe_unused]] constexpr uint8_t kModeStandby = 0;
[[maybe_unused]] constexpr uint8_t kModeRx = 1;
[[maybe_unused]] constexpr uint8_t kModeTx = 2;
[[maybe_unused]] constexpr uint8_t kModeCad = 3;

// GetStatus packs the chip mode into bits 6:4 and the command status into 3:1.
[[maybe_unused]] constexpr uint8_t kStatusModeStandbyRc = 0x2;
[[maybe_unused]] constexpr uint8_t kStatusModeStandbyXosc = 0x3;
[[maybe_unused]] constexpr uint8_t kStatusModeFs = 0x4;
[[maybe_unused]] constexpr uint8_t kStatusModeRx = 0x5;
[[maybe_unused]] constexpr uint8_t kStatusModeTx = 0x6;
[[maybe_unused]] constexpr uint8_t kStatusCmdDataAvailable = 0x2;
[[maybe_unused]] constexpr uint8_t kStatusCmdCompleted = 0x6;

// How a reception failed, for the deliberately corrupt delivery path.
[[maybe_unused]] constexpr uint8_t kReceiveOk = 0;
[[maybe_unused]] constexpr uint8_t kReceiveCrcError = 1;
[[maybe_unused]] constexpr uint8_t kReceiveHeaderError = 2;
[[maybe_unused]] constexpr uint8_t kReceiveNoLock = 3;
// A frame whose explicit header said the payload carries no CRC. The receiver
// has nothing to check, so it is delivered like a good one: RxDone and nothing
// else, whatever the bytes are.
[[maybe_unused]] constexpr uint8_t kReceiveNoCrc = 4;
// The next frame's header, arriving in continuous receive after the last frame's
// RxDone and before the firmware read it (5), or between its buffer status read
// and the rest of the read (6). See VirtualSX1262::applyNextFrame.
[[maybe_unused]] constexpr uint8_t kReceiveNextBeforeRead = 5;
[[maybe_unused]] constexpr uint8_t kReceiveNextDuringRead = 6;

// What the last explicit header said, as Semtech's own driver reads it back
// (sx126x_get_lora_params_from_header): the coding rate in bits 6:4 of 0x0749
// and the CRC flag in bit 4 of 0x076B. RadioLib's getLoRaRxHeaderInfo() reads
// the same two.
[[maybe_unused]] constexpr uint16_t kRegLoRaRxCodingRate = 0x0749;
[[maybe_unused]] constexpr uint16_t kRegLoRaRxCrc = 0x076B;

// The LoRa sync word, which lives in two registers rather than a command.
//
// RadioLib writes it as a pair through WriteRegister, so nothing in the opcode
// table names it and a model that only watches commands never sees it change.
// The reset value is the private network word, 0x1424, which is what the part
// comes up holding and what a firmware that never sets one is using.
[[maybe_unused]] constexpr uint16_t kRegSyncWordMsb = 0x0740;
[[maybe_unused]] constexpr uint16_t kRegSyncWordLsb = 0x0741;
[[maybe_unused]] constexpr uint16_t kSyncWordPrivate = 0x1424;
[[maybe_unused]] constexpr uint16_t kSyncWordPublic = 0x3444;

// What SetModulationParams is allowed to say. Outside these the firmware has
// programmed something the part does not define, which is a fault to report
// rather than a value to adopt.
[[maybe_unused]] constexpr uint8_t kSfMin = 5;
[[maybe_unused]] constexpr uint8_t kSfMax = 12;
[[maybe_unused]] constexpr uint8_t kCrMin = 1;
[[maybe_unused]] constexpr uint8_t kCrMax = 4;

// The symbol duration past which the part wants low data rate optimisation, in
// milliseconds. It is a property of the symbol and not of the spreading factor:
// SF11 crosses it at 125 kHz and does not at 250 kHz, so a rule written in SF
// alone is wrong at every bandwidth but one.
[[maybe_unused]] constexpr double kLowDataRateSymbolMs = 16.0;

// Untested.
// // CAD exit modes, from SetCadParams. CAD_ONLY drops back to standby whatever it
// // found; CAD_RX goes straight on into receive when it found something, which is
// // the mode a driver uses to avoid re-arming between the scan and the packet it
// // scanned for.
// [[maybe_unused]] constexpr uint8_t kCadExitOnly = 0x00;
// [[maybe_unused]] constexpr uint8_t kCadExitRx = 0x01;

// The symbol counts SetCadParams selects between, as 1 << cadSymbolNum: the
// register holds 0 to 4 and the scan lasts 1, 2, 4, 8 or 16 symbols. Anything
// above 4 is not a longer scan, it is a value the part does not define, so it is
// clamped rather than shifted with.
[[maybe_unused]] constexpr uint8_t kCadSymbolNumMax = 4;

// The timeout register's unit, shared by SetRx, SetTx and SetCadParams: one
// count is 15.625 us, which is 1/64 of a millisecond.
[[maybe_unused]] constexpr double kTimeoutStepMs = 1.0 / 64.0;

// A 24-bit timeout of all ones is not a very long timeout, it is continuous
// receive, and a driver that means "listen until I say otherwise" writes this.
[[maybe_unused]] constexpr uint32_t kRxContinuous = 0xFFFFFF;

// How far into a transmission a receiver locks onto the preamble, and where the
// header ends. Both are in symbols and become milliseconds through the current
// modem settings, because that is what makes them behave like a radio rather
// than like a constant: at SF12 a preamble takes an age and at SF7 it is gone in
// a blink, and MeshCore's listen-before-talk times exactly that.
//
// The header cannot be read until the whole preamble, the two sync symbols and
// the 2.25-symbol SFD have gone by, and it is the first interleaver block after
// them: eight symbols, always at CR 4/8. A fixed twelve symbols raised
// HeaderValid 32.25 symbols early for MeshCore's 32-symbol preamble at SF8 and
// below (16 at SF9 and up). That shrank the stretch where a firmware that can
// see preambles answers differently from one that cannot, from PreambleDetected
// to HeaderValid, to eight symbols of its real 40.25: a fifth.
[[maybe_unused]] constexpr double kPreambleSymbols = 4.0;
[[maybe_unused]] constexpr double kSyncAndSfdSymbols = 4.25;
[[maybe_unused]] constexpr double kHeaderBlockSymbols = 8.0;
}  // namespace

#endif /* VIRTUAL_SX1262_REGISTERS_H */
