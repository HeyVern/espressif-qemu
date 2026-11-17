/* virtual-sx1262 - a virtual Semtech SX1262, as a C ABI.
 *
 * The model itself is C++ and knows nothing of its hosts. This is the surface
 * every host links against instead of talking to a server over a socket:
 *
 *   the native MeshCore build   links the C++ class directly
 *   QEMU's sx1262 device        links this, extern "C"
 *   Renode's C# peripheral      DllImports this
 *   MeshBench                   cgo, where it wants the model in Go tests
 *
 * Two deliberate shapes, both learned from the socket this replaces:
 *
 *   SPI both ways round, because the hosts are not alike. vsx_spi_transaction
 *   takes a whole chip-select framed exchange, which is what a host with the
 *   command already in a buffer has. An emulator does not: its SPI controller
 *   clocks one byte and wants the answering byte back before it clocks the
 *   next, so QEMU and Renode drive vsx_spi_begin / vsx_spi_byte / vsx_spi_end
 *   instead. The model runs the same command either way and the two are held to
 *   identical answers by a test, because an emulated node and a native one
 *   being different radios would make every comparison between them a
 *   comparison of our own code.
 *
 *   The chip raises DIO1; nobody asks it. The socket was request-response, so
 *   the model had no way to call back and its hosts polled the line on a timer
 *   instead - up to a millisecond of latency on every received packet, for a
 *   pin a real part asserts. Register a callback and it is pushed.
 *
 * Not thread-safe. A host that drives SPI and the clock from different threads
 * serialises them itself, which is what the radioserver's lock did.
 */
#ifndef VIRTUAL_SX1262_H
#define VIRTUAL_SX1262_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct vsx_chip vsx_chip;

/* DIO1 changed level. Called from inside whichever vsx_ call caused it, on that
 * caller's thread - never from a thread of the model's own - so a host may
 * drive its own interrupt line straight from here. */
typedef void (*vsx_dio1_fn)(void* user, int asserted);

/* ---- lifecycle ---- */
vsx_chip* vsx_create(void);
void vsx_destroy(vsx_chip* chip);

/* NULL fn goes back to polling with vsx_dio1_asserted. */
void vsx_set_dio1_callback(vsx_chip* chip, vsx_dio1_fn fn, void* user);

/* ---- the firmware side ---- */

/* One chip-select framed SPI exchange. `out` is MOSI, `in` receives MISO, both
 * `len` bytes. `in` may be NULL for a command whose reply is not wanted. */
void vsx_spi_transaction(vsx_chip* chip, const uint8_t* out, uint8_t* in, size_t len);

/* The same exchange a byte at a time, for a host whose SPI controller has no
 * buffer to hand: chip select down, one call per clocked byte with the answering
 * byte returned there and then, chip select up.
 *
 * The framing is not decoration. There is no length anywhere in an SX1262
 * command, so the rising edge of chip select is the only thing that says a
 * command is complete and may take effect. A host that clocks bytes and never
 * releases has issued nothing.
 *
 * A byte clocked without a begin starts a transaction, because that is what the
 * silicon does with a chip select somebody else drove low. */
void vsx_spi_begin(vsx_chip* chip);
uint8_t vsx_spi_byte(vsx_chip* chip, uint8_t out);
void vsx_spi_end(vsx_chip* chip);

/* BUSY, which RadioLib spins on. */
int vsx_busy(const vsx_chip* chip);

/* DIO1 now, for a host that would rather ask than be told. */
int vsx_dio1_asserted(const vsx_chip* chip);

/* ---- the simulator side ---- */

/* Advance the chip's clock. Monotonic, in milliseconds of simulated time. */
void vsx_tick(vsx_chip* chip, uint64_t now_ms);

/* Whether another station is on the air here. Drives preamble and header
 * detection, and the answer CAD gives. */
void vsx_set_channel_busy(vsx_chip* chip, int busy);

/* Hand the chip a received frame. It is delivered to the firmware on a tick
 * where the chip is listening; if it is not listening it is dropped, because a
 * packet cannot arrive after the signal that carried it has gone. */
void vsx_deliver_frame(vsx_chip* chip, const uint8_t* frame, size_t len);

/* The same, from a transmitter whose sync word the caller knows.
 *
 * A LoRa receiver uses the sync word to decide a frame was not meant for it,
 * and a frame whose word does not match this chip's is refused and counted
 * rather than delivered. The firmware sets its own word through two registers
 * rather than a command, so nothing above the chip can see it except through
 * vsx_state.sync_word.
 *
 * Separate from vsx_deliver_frame rather than an extra argument to it, because
 * a host that does not model sync words must go on working unchanged: making no
 * claim about the transmitter is not the same as claiming it matched, and the
 * older entry point makes no claim. */
void vsx_deliver_frame_from(vsx_chip* chip, const uint8_t* frame, size_t len,
                            uint16_t sync_word);

/* A frame the receiver got and could not trust. `failure` is 1 for a CRC error
 * and 2 for a header error. 3, since ABI 1.7, is a frame the receiver detected
 * and never locked onto: under strict receive it withdraws the HeaderValid the
 * model raised for it and leaves PreambleDetected latched; otherwise it does
 * nothing. A library older than 1.7 reads 3 as a CRC error, so a host sends it
 * only to a library that reports 1.7 or later.
 *
 * 4, since ABI 1.8, is a frame whose explicit header said its payload carries
 * no CRC. The part has nothing to check, so it is delivered like a good frame,
 * RxDone alone, and only the header CRC flag (bit 4 of register 0x076B) says
 * otherwise. Every other reception sets that flag.
 *
 * 5 and 6, since ABI 1.8, are the next frame's header arriving in continuous
 * receive after the last frame's RxDone and before the firmware read it (5),
 * or after its first buffer status read (6). The bytes go straight after the
 * last frame in the buffer and the reported length becomes the next frame's,
 * while the start pointer stays where it was. That timing is not documented by
 * the part: it is a model for testing a driver, not a claim. A receiver not in
 * continuous receive never hears it.
 *
 * PLACEHOLDER INTERFACE, and the only way CrcErr or HeaderErr can be raised at
 * all. The chip cannot decide this: whether a frame's CRC checks depends on
 * what arrived at the antenna, which the simulator owns. Nothing calls this
 * yet, so both flags stay unreachable in practice and the firmware's error
 * paths stay unexercised: under calculated RF a frame arrives whole or not at
 * all and nothing ever will, and under waveform RF the demodulator genuinely
 * can fail a CRC but nothing is wired to say so. A CRC failure delivers the
 * corrupt payload and raises RxDone beside CrcErr, as the part does. */
void vsx_deliver_frame_failed(vsx_chip* chip, const uint8_t* frame, size_t len,
                              uint8_t failure);

/* A frame whose carrier has just begun here, from a host that knows what it
 * carries before it ends. Since ABI 1.9.
 *
 * Optional. A frame announced here lands in the data buffer as it arrives: its
 * length and header registers at the instant HeaderValid is raised, and its
 * payload as its symbols do, straight after the last frame received. It still
 * has to be handed over with vsx_deliver_frame or vsx_deliver_frame_failed when
 * it ends; one that never is, or that fails before its payload, is taken back
 * out. Without this a frame lands whole when it is handed over, and a driver
 * that reads late can never see the next frame's header. */
void vsx_frame_started(vsx_chip* chip, const uint8_t* frame, size_t len);

/* The waveform the chip started has finished on the air. The chip cannot know
 * this: how long a transmission occupied the channel is a property of the
 * samples the simulator generated. */
void vsx_transmit_finished(vsx_chip* chip);

/* Take a frame the firmware transmitted, if there is one. Returns its length,
 * or 0 when there is nothing to send. Copies at most `cap` bytes; a return
 * greater than `cap` means the frame was truncated. */
size_t vsx_take_tx(vsx_chip* chip, uint8_t* dst, size_t cap);

/* The front-end module's enable line, which decides how much power leaves the
 * board. */
void vsx_set_fem_enabled(vsx_chip* chip, int enabled);

/* What the last reception measured, for the registers the firmware reads back. */
void vsx_set_last_signal(vsx_chip* chip, float rssi_dbm, float snr_db);

/* ---- what the chip has been programmed to, and what it has seen ---- */

typedef struct {
  uint32_t freq_hz;
  uint32_t bandwidth_hz;
  uint16_t preamble_syms;
  uint16_t irq_mask;
  uint16_t irq_flags;
  uint8_t spreading_factor;
  uint8_t coding_rate;
  uint8_t mode; /* 0 standby, 1 rx, 2 tx, 3 cad */
  int8_t tx_power_dbm;
  uint8_t rx_gain_reg;
  uint8_t fem_at_tx; /* 0 never transmitted, 1 module out, 2 module in */
  /* Which interrupts are wired out to the DIO1 pin, which is a narrower set
   * than irq_mask: that one says what reaches the status register. Appended at
   * the end, per the ABI rule, so a host built against 1.0 is unaffected. */
  uint16_t dio1_mask;
  /* What SetCadParams programmed, reported because a scan's cost and its answer
   * both come from here and neither is visible any other way. cad_symbol_num is
   * the register value, so the dwell is 1 << it symbols; cad_exit_mode is the
   * programmed byte, recorded only: every scan ends in standby. The two
   * thresholds are recorded and not applied:
   * they threshold a correlator peak, and this model is told whether a carrier
   * is present rather than how strong it is, so a host that does know the level
   * is the only thing that can honestly act on them. */
  uint8_t cad_symbol_num;
  uint8_t cad_det_peak;
  uint8_t cad_det_min;
  uint8_t cad_exit_mode;
  /* The LoRa sync word the firmware programmed, as the two registers hold it:
   * 0x1424 is the private network default this chip comes up on and 0x3444 is
   * the public one. Two meshes sharing a frequency are told apart by this and
   * by nothing else the chip can see. */
  uint16_t sync_word;
  /* Whether the modem is running low data rate optimisation, from the field
   * SetModulationParams programmed. Reported because it costs two bits a symbol
   * and so changes every airtime this chip quotes, and because a firmware that
   * never programs it is a different case from one that programs it off. */
  uint8_t low_data_rate_optimize;
  /* The rest of the command set, as the firmware programmed it. Most of these
   * are recorded rather than acted on; src/commands.cpp says which is which for
   * each one, and a host reading a value here is reading what was asked for,
   * not a claim that the model obeyed it. */
  uint8_t packet_type;       /* 0 GFSK, 1 LoRa, 3 LR-FHSS; only LoRa runs */
  uint8_t fallback_mode;     /* 0x20 STDBY_RC, 0x30 STDBY_XOSC, 0x40 FS */
  uint8_t regulator_mode;    /* recorded only; this model has no current */
  uint8_t dio2_as_rf_switch; /* recorded; the FEM line is host-driven here */
  uint8_t dio3_as_tcxo;      /* recorded only */
  uint8_t pa_duty_cycle;     /* recorded; with pa_hp_max this is half of */
  uint8_t pa_hp_max;         /* what sets output power on silicon, and the */
  uint8_t pa_device_sel;     /* board owns the other half */
  uint8_t symb_num_timeout;  /* a receive deadline counted in symbols */
  uint8_t stop_timer_on_preamble;
  uint8_t tx_continuous_wave; /* a bare carrier; the host must put it on air */
  uint8_t tx_infinite_preamble;
  uint8_t header_implicit; /* the three SetPacketParams fields that were */
  uint8_t crc_on;          /* discarded, all of which change airtime */
  uint8_t invert_iq;
  uint32_t rx_duty_rx_period; /* recorded; the receiver does not sleep */
  uint32_t rx_duty_sleep_period;
} vsx_state;

void vsx_get_state(const vsx_chip* chip, vsx_state* out);

typedef struct {
  uint32_t irq_reads;
  uint32_t busy_reads;
  uint32_t busy_ms;
  uint32_t spurious_raises;
  uint32_t preamble_raises;
  uint32_t frames_dropped; /* handed over while deaf, past the grace */
  /* Events the chip had and the firmware never saw, because IrqMask had them
   * masked off. A firmware waiting on an interrupt it did not enable looks
   * exactly like a quiet mesh, and this is the only thing that tells the two
   * apart. One per event, however many of its bits were masked: a header is one
   * event whether one interrupt or three would have announced it. */
  uint32_t irq_suppressed;
  /* Scans run, and scans that found a carrier. A host comparing a firmware that
   * scans against one that does not needs to know the scans happened and what
   * they cost, which is cad_runs multiplied by the dwell in vsx_state. */
  uint32_t cad_runs;
  uint32_t cad_detections;
  /* Frames refused because they were sent under another sync word, and
   * modulation fields refused because the part does not define them. Both are
   * faults that otherwise present as a quiet network: a node on the wrong sync
   * word receives nothing and reports nothing wrong. */
  uint32_t sync_mismatches;
  uint32_t params_rejected;
  /* GetStats' three counters, as the firmware reads them. rx_packets is real;
   * the other two can only move through vsx_deliver_frame_failed, which nothing
   * calls, so a firmware polling them sees zero however bad the channel is. */
  uint16_t stat_rx_packets;
  uint16_t stat_crc_errors;
  uint16_t stat_header_errors;
} vsx_counters;

void vsx_get_counters(const vsx_chip* chip, vsx_counters* out);

/* MeshCore's listen-before-talk is built on the airtime the firmware itself
 * computes, so a host that needs the figure asks the chip rather than
 * reimplementing Semtech's formula beside it. */
uint32_t vsx_est_airtime_ms(const vsx_chip* chip, int len_bytes);

/* Make the chip misbehave the way real SX1262s do: latch a detection flag and
 * refuse to drop it. 0 disables. This is what MeshCore 1.17's stale-flag
 * recovery exists to survive, and the only place the model lies on purpose. */
void vsx_set_stuck_irq_ms(vsx_chip* chip, uint32_t ms);

/* Receive a frame only if the receiver was in receive from its preamble to its
 * end, as the part does: SetStandby, CAD or a transmission part-way through
 * loses it, and preamble detection only counts symbols heard in receive. The
 * detection flags also stay latched when the carrier ends, as on silicon,
 * until the firmware clears them or failure 3 withdraws a header. Off by
 * default, for a host whose clock is the engine's own; the emulated hosts drift
 * against the engine and keep the inbox grace. Since ABI 1.7. */
void vsx_set_strict_receive(vsx_chip* chip, int on);

/* Frames refused under strict receive, because the receiver was not listening
 * for all of them. A separate call rather than a vsx_counters field, so a host
 * built against an older header is not handed a larger struct. Since ABI 1.7. */
uint32_t vsx_frames_aborted(const vsx_chip* chip);

/* Seed this chip's receiver noise.
 *
 * Firmware helps itself to a radio's noise for entropy - RadioLib reads the
 * instantaneous RSSI eight times and keeps the low bit of each, and MeshCore
 * seeds its PRNG from that and derives its identity from the PRNG. Unseeded,
 * every chip answers alike and every node ends up with the same keypair.
 *
 * Give each node a different seed, derived from the run's own seed so the
 * result is still reproducible. */
void vsx_set_noise_seed(vsx_chip* chip, uint64_t seed);

/* Semantic version of this ABI, for a host that loads the library at runtime. */
void vsx_abi_version(int* major, int* minor);

#ifdef __cplusplus
}
#endif
#endif /* VIRTUAL_SX1262_H */
