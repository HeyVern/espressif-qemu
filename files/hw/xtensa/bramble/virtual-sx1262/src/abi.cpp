/* The C ABI over VirtualSX1262.
 *
 * The model stays a plain C++ class that knows nothing about its hosts. Two
 * things live here rather than in it:
 *
 *   The DIO1 edge. The model exposes the level (irqAsserted); turning that into
 *   "the line just changed, tell somebody" is a host concern, and doing it here
 *   means every host gets a pushed interrupt without the model growing a
 *   callback it would have to invoke at the right moments.
 *
 *   The handle. Hosts get an opaque pointer, so the C++ layout is not part of
 *   the ABI and the model can be rearranged without breaking a DllImport.
 */
#include "virtual_sx1262.h"

#include "VirtualSX1262.h"

#include <cstring>
#include <new>

#define VSX_ABI_MAJOR 1
#define VSX_ABI_MINOR 9

struct vsx_chip {
  VirtualSX1262 chip;
  vsx_dio1_fn dio1_fn = nullptr;
  void* dio1_user = nullptr;
  bool dio1_last = false;
};

namespace {

/* Every entry point that can move an IRQ bit ends here. The callback fires only
 * on a change, so a host can wire it straight to a level-driven interrupt line
 * without filtering, and a chip nobody is listening to costs one comparison. */
void settle_dio1(vsx_chip* c) {
  const bool now = c->chip.irqAsserted();
  if (now == c->dio1_last) {
    return;
  }
  c->dio1_last = now;
  if (c->dio1_fn) {
    c->dio1_fn(c->dio1_user, now ? 1 : 0);
  }
}

}  // namespace

extern "C" {

vsx_chip* vsx_create(void) {
  return new (std::nothrow) vsx_chip();
}

void vsx_destroy(vsx_chip* chip) {
  delete chip;
}

void vsx_set_dio1_callback(vsx_chip* chip, vsx_dio1_fn fn, void* user) {
  if (!chip) {
    return;
  }
  chip->dio1_fn = fn;
  chip->dio1_user = user;
  /* Report the line as it stands, so a host that registers late is not left
   * waiting for an edge that already happened. */
  if (fn && chip->dio1_last) {
    fn(user, 1);
  }
}

void vsx_spi_transaction(vsx_chip* chip, const uint8_t* out, uint8_t* in, size_t len) {
  if (!chip || !out || len == 0) {
    return;
  }
  uint8_t scratch[260];
  uint8_t* dst = in;
  if (!dst) {
    dst = scratch;
    if (len > sizeof(scratch)) {
      len = sizeof(scratch);
    }
  }
  chip->chip.spiTransfer(out, len, dst);
  settle_dio1(chip);
}

void vsx_spi_begin(vsx_chip* chip) {
  if (chip) {
    chip->chip.beginTransaction();
  }
}

uint8_t vsx_spi_byte(vsx_chip* chip, uint8_t out) {
  if (!chip) {
    return 0;
  }
  const uint8_t in = chip->chip.transferByte(out);
  /* Settled here as well as at the release, because a command that returns data
   * is run as its bytes arrive rather than at the end, and GetIrqStatus is read
   * inside the same transaction that RadioLib then acts on. */
  settle_dio1(chip);
  return in;
}

void vsx_spi_end(vsx_chip* chip) {
  if (!chip) {
    return;
  }
  chip->chip.endTransaction();
  settle_dio1(chip);
}

int vsx_busy(const vsx_chip* chip) {
  (void)chip;
  /* Held low, which is what the native path does too: the model does not
   * represent the time a real part spends digesting a command, and answering
   * differently here would give an emulated node a different radio from a
   * native one. */
  return 0;
}

int vsx_dio1_asserted(const vsx_chip* chip) {
  return chip && chip->chip.irqAsserted() ? 1 : 0;
}

void vsx_tick(vsx_chip* chip, uint64_t now_ms) {
  if (!chip) {
    return;
  }
  chip->chip.tick(now_ms);
  settle_dio1(chip);
}

void vsx_set_channel_busy(vsx_chip* chip, int busy) {
  if (!chip) {
    return;
  }
  chip->chip.setChannelBusy(busy != 0);
  settle_dio1(chip);
}

void vsx_deliver_frame(vsx_chip* chip, const uint8_t* frame, size_t len) {
  if (!chip || !frame || len == 0) {
    return;
  }
  chip->chip.inbox.emplace_back(frame, frame + len);
  /* No settle: nothing is asserted until a tick delivers it, and delivering
   * on arrival would let a packet in while the chip was transmitting. */
}

void vsx_deliver_frame_from(vsx_chip* chip, const uint8_t* frame, size_t len,
                            uint16_t sync_word) {
  if (chip && frame) {
    chip->chip.deliverFrameFrom(frame, len, sync_word);
    settle_dio1(chip);
  }
}

void vsx_deliver_frame_failed(vsx_chip* chip, const uint8_t* frame, size_t len,
                              uint8_t failure) {
  if (chip && frame) {
    chip->chip.deliverFrameFailed(frame, len, failure);
    settle_dio1(chip);
  }
}

void vsx_frame_started(vsx_chip* chip, const uint8_t* frame, size_t len) {
  if (chip && frame) {
    chip->chip.frameStarted(frame, len);
  }
}

void vsx_transmit_finished(vsx_chip* chip) {
  if (!chip) {
    return;
  }
  chip->chip.transmitFinished();
  settle_dio1(chip);
}

size_t vsx_take_tx(vsx_chip* chip, uint8_t* dst, size_t cap) {
  if (!chip || !chip->chip.hasPendingTx) {
    return 0;
  }
  const size_t n = chip->chip.pendingTx.size();
  if (dst && cap) {
    std::memcpy(dst, chip->chip.pendingTx.data(), n < cap ? n : cap);
  }
  chip->chip.hasPendingTx = false;
  return n;
}

void vsx_set_fem_enabled(vsx_chip* chip, int enabled) {
  if (chip) {
    chip->chip.setFemEnabled(enabled != 0);
  }
}

void vsx_set_last_signal(vsx_chip* chip, float rssi_dbm, float snr_db) {
  if (chip) {
    chip->chip.setLastSignal(rssi_dbm, snr_db);
  }
}

void vsx_get_state(const vsx_chip* chip, vsx_state* out) {
  if (!chip || !out) {
    return;
  }
  const VirtualSX1262& c = chip->chip;
  std::memset(out, 0, sizeof(*out));
  out->freq_hz = c.freqHz();
  out->bandwidth_hz = (uint32_t)(c.bwKHz() * 1000.0f + 0.5f);
  out->preamble_syms = (uint16_t)c.preambleSyms();
  out->irq_mask = c.irqMask();
  out->dio1_mask = c.dio1Mask();
  out->irq_flags = c.irqFlags();
  out->spreading_factor = (uint8_t)c.sf();
  out->coding_rate = (uint8_t)c.cr();
  out->mode = c.mode();
  out->tx_power_dbm = c.txPowerDbm();
  out->rx_gain_reg = c.rxGainReg();
  /* Three states, because "has not transmitted" is not "transmitted with the
   * module out". */
  out->fem_at_tx = !c.hasTransmitted() ? 0 : (c.femAtTx() ? 2 : 1);
  out->cad_symbol_num = c.cadSymbolNum();
  out->cad_det_peak = c.cadDetPeak();
  out->cad_det_min = c.cadDetMin();
  out->cad_exit_mode = c.cadExitMode();
  out->sync_word = c.syncWord();
  out->low_data_rate_optimize = c.lowDataRateOptimize() ? 1 : 0;
  out->packet_type = c.packetType();
  out->fallback_mode = c.fallbackMode();
  out->regulator_mode = c.regulatorMode();
  out->dio2_as_rf_switch = c.dio2AsRfSwitch() ? 1 : 0;
  out->dio3_as_tcxo = c.dio3AsTcxo() ? 1 : 0;
  out->pa_duty_cycle = c.paDutyCycle();
  out->pa_hp_max = c.paHpMax();
  out->pa_device_sel = c.paDeviceSel();
  out->symb_num_timeout = c.symbNumTimeout();
  out->stop_timer_on_preamble = c.stopTimerOnPreamble() ? 1 : 0;
  out->tx_continuous_wave = c.txContinuousWave() ? 1 : 0;
  out->tx_infinite_preamble = c.txInfinitePreamble() ? 1 : 0;
  out->header_implicit = c.headerImplicit() ? 1 : 0;
  out->crc_on = c.crcOn() ? 1 : 0;
  out->invert_iq = c.invertIq() ? 1 : 0;
  out->rx_duty_rx_period = c.rxDutyRxPeriod();
  out->rx_duty_sleep_period = c.rxDutySleepPeriod();
}

void vsx_get_counters(const vsx_chip* chip, vsx_counters* out) {
  if (!chip || !out) {
    return;
  }
  const VirtualSX1262& c = chip->chip;
  out->irq_reads = c.irqReads();
  out->busy_reads = c.busyReads();
  out->busy_ms = c.busyMs();
  out->spurious_raises = c.spuriousRaises();
  out->preamble_raises = c.preambleRaises();
  out->frames_dropped = c.framesDropped();
  out->irq_suppressed = c.irqSuppressed();
  out->cad_runs = c.cadRuns();
  out->cad_detections = c.cadDetections();
  out->sync_mismatches = c.syncMismatches();
  out->params_rejected = c.paramsRejected();
  out->stat_rx_packets = c.statRxPackets();
  out->stat_crc_errors = c.statCrcErrors();
  out->stat_header_errors = c.statHeaderErrors();
}

uint32_t vsx_est_airtime_ms(const vsx_chip* chip, int len_bytes) {
  return chip ? chip->chip.estAirtimeMs(len_bytes) : 0;
}

void vsx_set_stuck_irq_ms(vsx_chip* chip, uint32_t ms) {
  if (chip) {
    chip->chip.setStuckIrqMs(ms);
  }
}

void vsx_set_strict_receive(vsx_chip* chip, int on) {
  if (chip) {
    chip->chip.setStrictReceive(on != 0);
  }
}

uint32_t vsx_frames_aborted(const vsx_chip* chip) {
  return chip ? chip->chip.framesAborted() : 0;
}

void vsx_set_noise_seed(vsx_chip* chip, uint64_t seed) {
  if (chip) {
    chip->chip.setNoiseSeed(seed);
  }
}

void vsx_abi_version(int* major, int* minor) {
  if (major) {
    *major = VSX_ABI_MAJOR;
  }
  if (minor) {
    *minor = VSX_ABI_MINOR;
  }
}

}  // extern "C"
