/* Channel activity detection: the other half of listen before talk.
 *
 * Its own file rather than another few functions in VirtualSX1262.cpp, for the
 * reason that file and spi.cpp are already apart: a scan is a thing the part
 * does over time, with its own programmed parameters and its own way out, and
 * it is changed for different reasons from the receive path it sits beside.
 *
 * What makes CAD worth modelling properly is that it costs time. A driver runs
 * it to find out whether to transmit, and what it pays for the answer is the
 * dwell: 1 to 16 symbols, which at SF12 is most of a second and at SF7 is
 * nothing. A scan that answers instantly is not a cheaper CAD, it is a
 * different radio, and a comparison between a firmware that scans and one that
 * does not then measures nothing at all.
 */
#include "VirtualSX1262.h"

#include "registers.h"

// SetCadParams: [symbolNum][detPeak][detMin][exitMode][timeout 23:16..7:0].
//
// exitMode and the timeout are recorded or ignored, not acted on: every scan
// ends in standby.
//
// detPeak and detMin are recorded and not acted on, which is a deliberate stop
// rather than an unfinished one. They are thresholds against a correlator peak,
// and this model is not handed a correlator peak: the simulator tells it
// whether a carrier is present, full stop. Thresholding a boolean would be the
// chip inventing a signal level and then deciding reception from it, which is
// the simulator's job and not ours. Reported instead, so a host that does know
// the signal level can apply them, and so a firmware setting a threshold no
// real part would detect at is visible rather than silently fine.
void VirtualSX1262::applyCadParams(const uint8_t* p) {
  cadSymbolNum_ = p[0] > kCadSymbolNumMax ? kCadSymbolNumMax : p[0];
  cadDetPeak_ = p[1];
  cadDetMin_ = p[2];
  cadExitMode_ = p[3];
}

// SetCad. The scan starts here and finishes on a later tick.
//
// Nothing is reported at this point, because at this point nothing is known.
// The old model answered inside the command, which meant a firmware could scan
// the channel, get its answer and transmit without a millisecond having passed
// - so listen before talk cost exactly nothing and every arm that used it was
// flattered by however long the scan should have taken.
void VirtualSX1262::startCad() {
  mode_ = 3;
  cadRuns_++;
  // The carrier is sampled across the whole dwell rather than at either end of
  // it, so a transmission that starts or stops mid scan is still found. Seeded
  // with the state at the instant the scan begins, which is the one sample a
  // zero-length dwell would get.
  cadSawCarrier_ = channelBusy_;
  cadEndMs_ = nowMs_ + (uint64_t)((double)(1u << cadSymbolNum_) * symbolMs() + 0.5);
}

void VirtualSX1262::tickCad() {
  if (mode_ != 3) return;
  if (channelBusy_) cadSawCarrier_ = true;
  if (nowMs_ < cadEndMs_) return;
  finishCad();
}

// The end of a scan, and where it leaves the chip.
//
// CadDone always; CadDetected only if a carrier was there to find. Then standby,
// whatever the answer and whatever exit mode was programmed.
void VirtualSX1262::finishCad() {
  raiseIrq(kIrqCadDone);
  if (cadSawCarrier_) {
    cadDetections_++;
    raiseIrq(kIrqCadDetected);
  }
  mode_ = kModeStandby;
}
