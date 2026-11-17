/* Optional in-process chip backend. All entry points run under the BQL. */
#include <gmodule.h>
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "qemu/bswap.h"
#include "sysemu/runstate.h"
#include "virtual-sx1262/include/virtual_sx1262.h"

#define VSX_FUNCTIONS(X) \
    X(vsx_abi_version) X(vsx_create) X(vsx_destroy) \
    X(vsx_set_dio1_callback) X(vsx_spi_begin) X(vsx_spi_byte) \
    X(vsx_spi_end) X(vsx_tick) X(vsx_set_channel_busy) \
    X(vsx_deliver_frame) X(vsx_deliver_frame_from) \
    X(vsx_deliver_frame_failed) X(vsx_transmit_finished) X(vsx_take_tx) \
    X(vsx_set_last_signal) X(vsx_get_state) X(vsx_get_counters) \
    X(vsx_est_airtime_ms) X(vsx_set_strict_receive) X(vsx_set_noise_seed) \
    X(vsx_set_stuck_irq_ms) X(vsx_frames_aborted)

typedef struct {
    GModule *module;
    vsx_chip *chip;
#define VSX_DECLARE(name) __typeof__(name) *name;
    VSX_FUNCTIONS(VSX_DECLARE)
#undef VSX_DECLARE
    QEMUTimer *tick_timer;
    QEMUTimer *irq_timer;
    uint32_t dio1_gpio;
    uint64_t seed;
    int64_t carrier_until;
    int64_t spur_until;
    bool spi_active;
    bool irq_level;
    bool strict;
    uint32_t stuck_ms;
    CharBackend engine;
    bool engine_attached;
    bool engine_open;
    bool engine_busy;
    uint32_t engine_ms;
    bool lockstep;
    uint32_t engine_target;
    int64_t engine_vm_start;
    uint32_t engine_step_start;
    QEMUTimer *engine_barrier;
    GByteArray *engine_buffer;
} BrambleVirtualSx1262;

static void virtual_sx1262_irq_deferred(void *opaque)
{
    BrambleVirtualSx1262 *s = opaque;
    bramble_gpio_set_input(s->dio1_gpio, s->irq_level);
}

static void virtual_sx1262_irq(void *opaque, int asserted)
{
    BrambleVirtualSx1262 *s = opaque;
    s->irq_level = asserted != 0;
    if (asserted && s->spi_active) {
        /* RadioLib finishes arming after SetRx; an inline IRQ loses its flag. */
        timer_mod(s->irq_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
    } else {
        timer_del(s->irq_timer);
        bramble_gpio_set_input(s->dio1_gpio, s->irq_level);
    }
}

static void virtual_sx1262_tick(void *opaque)
{
    BrambleVirtualSx1262 *s = opaque;
    if (s->engine_attached) {
        if (s->lockstep) {
            int64_t now = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
            uint32_t elapsed = MAX((int64_t)0, now - s->engine_vm_start);
            uint32_t at = MIN(s->engine_target, s->engine_step_start + elapsed);
            s->vsx_tick(s->chip, at);
            timer_mod(s->tick_timer, now + 1);
        }
        return;
    }
    int64_t now = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
    vsx_state state = {0};
    s->vsx_get_state(s->chip, &state);
    s->vsx_set_channel_busy(s->chip, now < s->carrier_until ||
                            (state.mode == 3 && now < s->spur_until));
    s->vsx_tick(s->chip, now);
    timer_mod(s->tick_timer, now + 1);
}

static void virtual_sx1262_txdone(QDict *msg, void *opaque)
{
    BrambleVirtualSx1262 *s = opaque;
    s->vsx_transmit_finished(s->chip);
}

static void virtual_sx1262_receive(QDict *msg, void *opaque)
{
    BrambleVirtualSx1262 *s = opaque;
    vsx_state state = {0};
    size_t n = 0;
    const char *payload = qdict_get_try_str(msg, "payload");
    if (!payload) {
        return;
    }
    s->vsx_get_state(s->chip, &state);
    if (qdict_haskey(msg, "freq_khz") && state.freq_hz &&
        qdict_get_int(msg, "freq_khz") != (state.freq_hz + 500) / 1000) {
        return;
    }
    if ((qdict_haskey(msg, "sf") &&
         qdict_get_int(msg, "sf") != state.spreading_factor) ||
        (qdict_haskey(msg, "bw") &&
         qdict_get_int(msg, "bw") != state.bandwidth_hz) ||
        (qdict_haskey(msg, "invert_iq") &&
         qdict_get_int(msg, "invert_iq") != state.invert_iq)) {
        return;
    }
    g_autofree uint8_t *bytes = qbase64_decode(payload, -1, &n, NULL);
    if (!bytes || n == 0 || n > 255) {
        return;
    }
    s->vsx_set_last_signal(s->chip, qdict_get_try_int(msg, "rssi", -100),
                           qdict_get_try_int(msg, "snr", 0));
    if (qdict_haskey(msg, "failure")) {
        int64_t failure = qdict_get_int(msg, "failure");
        if (failure < 1 || failure > 6) {
            return;
        }
        s->vsx_deliver_frame_failed(s->chip, bytes, n, failure);
    } else if (qdict_haskey(msg, "sync_word")) {
        s->vsx_deliver_frame_from(s->chip, bytes, n,
                                 qdict_get_int(msg, "sync_word"));
    } else {
        s->vsx_deliver_frame(s->chip, bytes, n);
    }
}

static void virtual_sx1262_carrier(QDict *msg, void *opaque)
{
    BrambleVirtualSx1262 *s = opaque;
    vsx_state state = {0};
    s->vsx_get_state(s->chip, &state);
    if (qdict_haskey(msg, "freq_khz") && state.freq_hz &&
        qdict_get_int(msg, "freq_khz") != (state.freq_hz + 500) / 1000) {
        return;
    }
    int64_t duration = qdict_get_try_int(msg, "ms", 0);
    if (duration > 0 && duration <= 3600000) {
        int64_t *until = qdict_get_try_int(msg, "spur", 0) ?
                         &s->spur_until : &s->carrier_until;
        *until = MAX(*until, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + duration);
        if (until == &s->carrier_until || state.mode == 3) {
            s->vsx_set_channel_busy(s->chip, 1);
        }
    }
}

static void virtual_sx1262_reset(BrambleVirtualSx1262 *s)
{
    timer_del(s->irq_timer);
    s->vsx_destroy(s->chip);
    s->chip = s->vsx_create();
    if (!s->chip) {
        error_report("virtual-sx1262: cannot allocate chip on reset");
        abort();
    }
    s->carrier_until = 0;
    s->spur_until = 0;
    s->spi_active = false;
    s->irq_level = false;
    bramble_gpio_set_input(s->dio1_gpio, false);
    s->vsx_set_noise_seed(s->chip, s->seed);
    s->vsx_set_strict_receive(s->chip, s->strict);
    s->vsx_set_stuck_irq_ms(s->chip, s->stuck_ms);
    s->vsx_set_dio1_callback(s->chip, virtual_sx1262_irq, s);
    virtual_sx1262_tick(s);
}

static void virtual_sx1262_engine_send(BrambleVirtualSx1262 *s, uint8_t kind,
                                        const uint8_t *bytes, uint16_t size)
{
    if (!s->engine_open) {
        return;
    }
    uint8_t packet[258] = {kind, size >> 8, size & 255};
    assert(size <= 255);
    if (size) {
        memcpy(packet + 3, bytes, size);
    }
    qemu_chr_fe_write_all(&s->engine, packet, size + 3);
}

static void virtual_sx1262_engine_stats(BrambleVirtualSx1262 *s)
{
    vsx_state st = {0};
    vsx_counters ct = {0};
    uint8_t data[48] = {0};
    s->vsx_get_state(s->chip, &st);
    s->vsx_get_counters(s->chip, &ct);
    stl_be_p(data, ct.irq_reads);
    stl_be_p(data + 4, ct.busy_reads);
    stl_be_p(data + 8, ct.busy_ms);
    stl_be_p(data + 12, ct.spurious_raises);
    data[16] = st.rx_gain_reg;
    data[17] = st.tx_power_dbm;
    data[19] = st.mode;
    data[20] = st.spreading_factor;
    data[21] = st.coding_rate;
    stl_be_p(data + 22, st.freq_hz);
    stl_be_p(data + 26, st.bandwidth_hz);
    stw_be_p(data + 30, st.preamble_syms);
    stw_be_p(data + 32, st.irq_mask);
    stw_be_p(data + 34, st.irq_flags);
    data[36] = st.fem_at_tx;
    stw_be_p(data + 37, st.dio1_mask);
    stl_be_p(data + 43, ct.irq_suppressed);
    data[47] = 7;
    virtual_sx1262_engine_send(s, 9, data, sizeof(data));
}

static void virtual_sx1262_engine_barrier(void *opaque)
{
    BrambleVirtualSx1262 *s = opaque;
    vm_stop(RUN_STATE_PAUSED);
    s->engine_ms = s->engine_target;
    s->vsx_tick(s->chip, s->engine_ms);
    virtual_sx1262_engine_stats(s);
    uint8_t ack[4];
    stl_be_p(ack, s->engine_ms);
    virtual_sx1262_engine_send(s, 3, ack, sizeof(ack));
}

static int virtual_sx1262_engine_can_read(void *opaque)
{
    return 4096;
}

static void virtual_sx1262_engine_read(void *opaque, const uint8_t *bytes,
                                        int size)
{
    BrambleVirtualSx1262 *s = opaque;
    g_byte_array_append(s->engine_buffer, bytes, size);
    while (s->engine_buffer->len >= 3) {
        uint8_t *data = s->engine_buffer->data;
        size_t n = lduw_be_p(data + 1);
        if (s->engine_buffer->len < n + 3) {
            return;
        }
        const uint8_t *payload = data + 3;
        switch (data[0]) {
        case 1:
            if (n > 0 && n <= 255) {
                s->vsx_deliver_frame(s->chip, payload, n);
            }
            break;
        case 2:
            if (n == 4) {
                uint32_t target = ldl_be_p(payload);
                if (s->lockstep) {
                    s->engine_step_start = s->engine_ms;
                    s->engine_vm_start = qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL);
                    s->engine_target = target;
                    uint32_t delta = target > s->engine_ms ? target - s->engine_ms : 0;
                    if (delta == 0) {
                        virtual_sx1262_engine_stats(s);
                        virtual_sx1262_engine_send(s, 3, payload, 4);
                    } else {
                        timer_mod(s->engine_barrier, s->engine_vm_start + delta);
                        timer_mod(s->tick_timer, s->engine_vm_start + 1);
                        vm_start();
                    }
                    break;
                }
                while (s->engine_ms < target) {
                    s->vsx_tick(s->chip, ++s->engine_ms);
                }
                virtual_sx1262_engine_stats(s);
                virtual_sx1262_engine_send(s, 3, payload, 4);
            }
            break;
        case 4:
            s->vsx_transmit_finished(s->chip);
            break;
        case 8:
            if (n) {
                s->engine_busy = payload[0] != 0;
                s->vsx_set_channel_busy(s->chip, s->engine_busy);
            }
            break;
        case 10:
            if (n > 1 && n <= 256 && payload[0] >= 1 && payload[0] <= 6) {
                s->vsx_deliver_frame_failed(s->chip, payload + 1, n - 1, payload[0]);
            }
            break;
        default:
            break;
        }
        g_byte_array_remove_range(s->engine_buffer, 0, n + 3);
    }
}

static void virtual_sx1262_engine_event(void *opaque, QEMUChrEvent event)
{
    BrambleVirtualSx1262 *s = opaque;
    s->engine_open = event == CHR_EVENT_OPENED;
    if (event == CHR_EVENT_CLOSED) {
        g_byte_array_set_size(s->engine_buffer, 0);
        warn_report("SX1262 RF engine disconnected; radio traffic is unavailable");
    }
}

static bool virtual_sx1262_engine_attach(BrambleVirtualSx1262 *s, Error **errp)
{
    Chardev *chr = qemu_chr_find("meshbench-radio");
    if (!chr) {
        return true;
    }
    if (!qemu_chr_fe_init(&s->engine, chr, errp)) {
        return false;
    }
    s->engine_attached = true;
    s->engine_buffer = g_byte_array_new();
    timer_del(s->tick_timer);
    if (s->lockstep) {
        s->engine_barrier = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                        virtual_sx1262_engine_barrier, s);
    }
    qemu_chr_fe_set_handlers(&s->engine, virtual_sx1262_engine_can_read,
        virtual_sx1262_engine_read, virtual_sx1262_engine_event, NULL, s, NULL, true);
    fprintf(stderr, "bramble-sx1262: MeshBench RF engine connected\n");
    return true;
}

static void virtual_sx1262_free(BrambleVirtualSx1262 *s)
{
    if (!s) {
        return;
    }
    if (s->engine_barrier) {
        timer_free(s->engine_barrier);
    }
    if (s->engine_attached) {
        qemu_chr_fe_deinit(&s->engine, false);
    }
    if (s->engine_buffer) {
        g_byte_array_unref(s->engine_buffer);
    }
    if (s->tick_timer) {
        timer_free(s->tick_timer);
    }
    if (s->irq_timer) {
        timer_free(s->irq_timer);
    }
    if (s->chip) {
        s->vsx_destroy(s->chip);
    }
    if (s->module) {
        g_module_close(s->module);
    }
    g_free(s);
}

static BrambleVirtualSx1262 *virtual_sx1262_new(const char *path,
        uint32_t gpio, uint64_t seed, bool strict, uint32_t stuck, bool lockstep, Error **errp)
{
    BrambleVirtualSx1262 *s = g_new0(BrambleVirtualSx1262, 1);
    s->module = g_module_open(path, G_MODULE_BIND_LOCAL);
    if (!s->module) {
        error_setg(errp, "virtual-sx1262: cannot load %s: %s", path,
                   g_module_error());
        goto fail;
    }
#define VSX_LOAD(name) \
    if (!g_module_symbol(s->module, #name, (gpointer *)&s->name)) { \
        error_setg(errp, "virtual-sx1262: missing %s in %s", #name, path); \
        goto fail; \
    }
    VSX_FUNCTIONS(VSX_LOAD)
#undef VSX_LOAD
    int major = 0, minor = 0;
    s->vsx_abi_version(&major, &minor);
    /* State and counter structs have no size argument; pin their ABI exactly. */
    if (major != 1 || minor != 9) {
        error_setg(errp, "virtual-sx1262: requires ABI 1.9, got %d.%d",
                   major, minor);
        goto fail;
    }
    s->lockstep = lockstep;
    s->dio1_gpio = gpio;
    s->seed = seed;
    s->strict = strict;
    s->stuck_ms = stuck;
    s->tick_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, virtual_sx1262_tick, s);
    s->irq_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                               virtual_sx1262_irq_deferred, s);
    virtual_sx1262_reset(s);
    if (!virtual_sx1262_engine_attach(s, errp)) {
        goto fail;
    }
    emulink_on("txdone", virtual_sx1262_txdone, s);
    emulink_on("rx", virtual_sx1262_receive, s);
    emulink_on("carrier", virtual_sx1262_carrier, s);
    fprintf(stderr, "bramble-sx1262: virtual backend ABI 1.9, strict=%d\n",
            strict);
    return s;
fail:
    virtual_sx1262_free(s);
    return NULL;
}

static void virtual_sx1262_cs(BrambleVirtualSx1262 *s, bool level)
{
    if (!level) {
        s->spi_active = true;
        s->vsx_spi_begin(s->chip);
        return;
    }
    s->vsx_spi_end(s->chip);
    s->spi_active = false;
    uint8_t bytes[255];
    size_t n = s->vsx_take_tx(s->chip, bytes, sizeof(bytes));
    if (n > 0 && n <= sizeof(bytes)) {
        if (s->engine_attached) {
            virtual_sx1262_engine_stats(s);
            virtual_sx1262_engine_send(s, 1, bytes, n);
            return;
        }
        vsx_state state = {0};
        s->vsx_get_state(s->chip, &state);
        g_autofree char *payload = g_base64_encode(bytes, n);
        g_autofree char *line = g_strdup_printf(
            "{\"t\":\"tx\",\"payload\":\"%s\",\"freq\":%u,"
            "\"freq_khz\":%u,\"sf\":%u,\"bw\":%u,\"cr\":%u,\"power\":%d,"
            "\"preamble\":%u,\"explicit_header\":%s,\"crc\":%s,"
            "\"ldro\":%u,\"invert_iq\":%u,\"sync_word\":%u,\"chip_toa_ms\":%u}\n",
            payload, (state.freq_hz + 500000) / 1000000,
            (state.freq_hz + 500) / 1000, state.spreading_factor,
            state.bandwidth_hz, state.coding_rate, state.tx_power_dbm,
            state.preamble_syms, state.header_implicit ? "false" : "true",
            state.crc_on ? "true" : "false", state.low_data_rate_optimize,
            state.invert_iq, state.sync_word,
            s->vsx_est_airtime_ms(s->chip, n));
        emulink_write(line, strlen(line));
    }
}
