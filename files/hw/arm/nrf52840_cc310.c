// Replaces the nrf_cc310 library's entry points with host crypto, because the CryptoCell engines are not modelled.
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "exec/cpu-common.h"
#include "qemu/guest-random.h"
#include "crypto/aes.h"
#include "nrf52840_cc310.h"
#include "nrf52840_cc310_sigs.h"

#define CC_OK 0
#define CC_FAIL 1
#define MODE_SHA256 2
#define AES_DECRYPT_MODE 1
#define CC_MAX_INPUT (64 * 1024)
#define SHA256_LEN 32

typedef struct {
    GChecksum *hash;
    GHmac *hmac;
    AES_KEY aes;
    bool decrypt;
} CcContext;

static GHashTable *contexts;

static void context_free(gpointer data)
{
    CcContext *c = data;

    if (c->hash) {
        g_checksum_free(c->hash);
    }
    if (c->hmac) {
        g_hmac_unref(c->hmac);
    }
    g_free(c);
}

static CcContext *context_new(uint32_t guest)
{
    CcContext *c = g_new0(CcContext, 1);

    g_hash_table_replace(contexts, GUINT_TO_POINTER(guest), c);
    return c;
}

static CcContext *context_get(uint32_t guest)
{
    return g_hash_table_lookup(contexts, GUINT_TO_POINTER(guest));
}

static uint8_t *guest_copy(uint32_t addr, uint32_t len)
{
    uint8_t *buf = g_malloc(len + 1);

    cpu_physical_memory_read(addr, buf, len);
    return buf;
}

// Ed25519 verification; field and group arithmetic follow the public-domain TweetNaCl.
typedef int64_t gf[16];

static const gf gf0;
static const gf gf1 = { 1 };
static const gf D2 = { 0xf159, 0x26b2, 0x9b94, 0xebd6, 0xb156, 0x8283, 0x149a, 0x00e0,
                       0xd130, 0xeef3, 0x80f2, 0x198e, 0xfce7, 0x56df, 0xd9dc, 0x2406 };
static const gf DD = { 0x78a3, 0x1359, 0x4dca, 0x75eb, 0xd8ab, 0x4141, 0x0a4d, 0x0070,
                       0xe898, 0x7779, 0x4079, 0x8cc7, 0xfe73, 0x2b6f, 0x6cee, 0x5203 };
static const gf XX = { 0xd51a, 0x8f25, 0x2d60, 0xc956, 0xa7b2, 0x9525, 0xc760, 0x692c,
                       0xdc5c, 0xfdd6, 0xe231, 0xc0a4, 0x53fe, 0xcd6e, 0x36d3, 0x2169 };
static const gf YY = { 0x6658, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666,
                       0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666 };
static const gf II = { 0xa0b0, 0x4a0e, 0x1b27, 0xc4ee, 0xe478, 0xad2f, 0x1806, 0x2f43,
                       0xd7a7, 0x3dfb, 0x0099, 0x2b4d, 0xdf0b, 0x4fc1, 0x2480, 0x2b83 };
static const uint64_t LL[32] = { 0xed, 0xd3, 0xf5, 0x5c, 0x1a, 0x63, 0x12, 0x58, 0xd6, 0x9c, 0xf7,
                                 0xa2, 0xde, 0xf9, 0xde, 0x14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0x10 };

static void set25519(gf r, const gf a)
{
    memcpy(r, a, sizeof(gf));
}

static void car25519(gf o)
{
    for (int i = 0; i < 16; i++) {
        o[i] += (1LL << 16);
        int64_t c = o[i] >> 16;

        o[(i + 1) * (i < 15)] += c - 1 + 37 * (c - 1) * (i == 15);
        o[i] -= (uint64_t)c << 16;
    }
}

static void sel25519(gf p, gf q, int b)
{
    int64_t c = ~(b - 1);

    for (int i = 0; i < 16; i++) {
        int64_t t = c & (p[i] ^ q[i]);

        p[i] ^= t;
        q[i] ^= t;
    }
}

static void pack25519(uint8_t *o, const gf n)
{
    gf m, t;

    set25519(t, n);
    car25519(t);
    car25519(t);
    car25519(t);
    for (int j = 0; j < 2; j++) {
        m[0] = t[0] - 0xffed;
        for (int i = 1; i < 15; i++) {
            m[i] = t[i] - 0xffff - ((m[i - 1] >> 16) & 1);
            m[i - 1] &= 0xffff;
        }
        m[15] = t[15] - 0x7fff - ((m[14] >> 16) & 1);
        int b = (m[15] >> 16) & 1;

        m[14] &= 0xffff;
        sel25519(t, m, 1 - b);
    }
    for (int i = 0; i < 16; i++) {
        o[2 * i] = t[i] & 0xff;
        o[2 * i + 1] = t[i] >> 8;
    }
}

static bool neq25519(const gf a, const gf b)
{
    uint8_t c[32], d[32];

    pack25519(c, a);
    pack25519(d, b);
    return memcmp(c, d, 32) != 0;
}

static uint8_t par25519(const gf a)
{
    uint8_t d[32];

    pack25519(d, a);
    return d[0] & 1;
}

static void unpack25519(gf o, const uint8_t *n)
{
    for (int i = 0; i < 16; i++) {
        o[i] = n[2 * i] + ((int64_t)n[2 * i + 1] << 8);
    }
    o[15] &= 0x7fff;
}

static void fadd(gf o, const gf a, const gf b)
{
    for (int i = 0; i < 16; i++) {
        o[i] = a[i] + b[i];
    }
}

static void fsub(gf o, const gf a, const gf b)
{
    for (int i = 0; i < 16; i++) {
        o[i] = a[i] - b[i];
    }
}

static void fmul(gf o, const gf a, const gf b)
{
    int64_t t[31] = { 0 };

    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            t[i + j] += a[i] * b[j];
        }
    }
    for (int i = 0; i < 15; i++) {
        t[i] += 38 * t[i + 16];
    }
    memcpy(o, t, sizeof(gf));
    car25519(o);
    car25519(o);
}

static void fsqr(gf o, const gf a)
{
    fmul(o, a, a);
}

static void inv25519(gf o, const gf i)
{
    gf c;

    set25519(c, i);
    for (int a = 253; a >= 0; a--) {
        fsqr(c, c);
        if (a != 2 && a != 4) {
            fmul(c, c, i);
        }
    }
    set25519(o, c);
}

static void pow2523(gf o, const gf i)
{
    gf c;

    set25519(c, i);
    for (int a = 250; a >= 0; a--) {
        fsqr(c, c);
        if (a != 1) {
            fmul(c, c, i);
        }
    }
    set25519(o, c);
}

static void point_add(gf p[4], gf q[4])
{
    gf a, b, c, d, t, e, f, g, h;

    fsub(a, p[1], p[0]);
    fsub(t, q[1], q[0]);
    fmul(a, a, t);
    fadd(b, p[0], p[1]);
    fadd(t, q[0], q[1]);
    fmul(b, b, t);
    fmul(c, p[3], q[3]);
    fmul(c, c, D2);
    fmul(d, p[2], q[2]);
    fadd(d, d, d);
    fsub(e, b, a);
    fsub(f, d, c);
    fadd(g, d, c);
    fadd(h, b, a);
    fmul(p[0], e, f);
    fmul(p[1], h, g);
    fmul(p[2], g, f);
    fmul(p[3], e, h);
}

static void point_swap(gf p[4], gf q[4], int b)
{
    for (int i = 0; i < 4; i++) {
        sel25519(p[i], q[i], b);
    }
}

static void point_pack(uint8_t *r, gf p[4])
{
    gf tx, ty, zi;

    inv25519(zi, p[2]);
    fmul(tx, p[0], zi);
    fmul(ty, p[1], zi);
    pack25519(r, ty);
    r[31] ^= par25519(tx) << 7;
}

static void scalarmult(gf p[4], gf q[4], const uint8_t *s)
{
    set25519(p[0], gf0);
    set25519(p[1], gf1);
    set25519(p[2], gf1);
    set25519(p[3], gf0);
    for (int i = 255; i >= 0; i--) {
        int b = (s[i / 8] >> (i & 7)) & 1;

        point_swap(p, q, b);
        point_add(q, p);
        point_add(p, p);
        point_swap(p, q, b);
    }
}

static void scalarbase(gf p[4], const uint8_t *s)
{
    gf q[4];

    set25519(q[0], XX);
    set25519(q[1], YY);
    set25519(q[2], gf1);
    fmul(q[3], XX, YY);
    scalarmult(p, q, s);
}

static void reduce_scalar(uint8_t *r)
{
    int64_t x[64], carry;
    int i, j;

    for (i = 0; i < 64; i++) {
        x[i] = r[i];
    }
    memset(r, 0, 64);
    for (i = 63; i >= 32; --i) {
        carry = 0;
        for (j = i - 32; j < i - 12; ++j) {
            x[j] += carry - 16 * x[i] * LL[j - (i - 32)];
            carry = (x[j] + 128) >> 8;
            x[j] -= (uint64_t)carry << 8;
        }
        x[j] += carry;
        x[i] = 0;
    }
    carry = 0;
    for (j = 0; j < 32; j++) {
        x[j] += carry - (x[31] >> 4) * LL[j];
        carry = x[j] >> 8;
        x[j] &= 255;
    }
    for (j = 0; j < 32; j++) {
        x[j] -= carry * LL[j];
    }
    for (i = 0; i < 32; i++) {
        x[i + 1] += x[i] >> 8;
        r[i] = x[i] & 255;
    }
}

static bool unpack_neg(gf r[4], const uint8_t *p)
{
    gf t, chk, num, den, den2, den4, den6;

    set25519(r[2], gf1);
    unpack25519(r[1], p);
    fsqr(num, r[1]);
    fmul(den, num, DD);
    fsub(num, num, r[2]);
    fadd(den, r[2], den);
    fsqr(den2, den);
    fsqr(den4, den2);
    fmul(den6, den4, den2);
    fmul(t, den6, num);
    fmul(t, t, den);
    pow2523(t, t);
    fmul(t, t, num);
    fmul(t, t, den);
    fmul(t, t, den);
    fmul(r[0], t, den);
    fsqr(chk, r[0]);
    fmul(chk, chk, den);
    if (neq25519(chk, num)) {
        fmul(r[0], r[0], II);
    }
    fsqr(chk, r[0]);
    fmul(chk, chk, den);
    if (neq25519(chk, num)) {
        return false;
    }
    if (par25519(r[0]) == (p[31] >> 7)) {
        fsub(r[0], gf0, r[0]);
    }
    fmul(r[3], r[0], r[1]);
    return true;
}

static bool ed25519_verify(const uint8_t *sig, const uint8_t *pub, const uint8_t *msg, uint32_t len)
{
    gf p[4], q[4];
    uint8_t h[64], t[32];
    gsize hlen = sizeof(h);
    GChecksum *sha = g_checksum_new(G_CHECKSUM_SHA512);

    if (!unpack_neg(q, pub)) {
        g_checksum_free(sha);
        return false;
    }
    g_checksum_update(sha, sig, 32);
    g_checksum_update(sha, pub, 32);
    g_checksum_update(sha, msg, len);
    g_checksum_get_digest(sha, h, &hlen);
    g_checksum_free(sha);
    reduce_scalar(h);
    scalarmult(p, q, h);
    scalarbase(q, sig + 32);
    point_add(p, q);
    point_pack(t, p);
    return memcmp(sig, t, 32) == 0;
}

static uint32_t call_verify(uint32_t sig_addr, uint32_t sig_len, uint32_t pub_addr, uint32_t pub_len,
                            uint32_t msg_addr, uint32_t msg_len)
{
    uint8_t sig[64], pub[32];

    if (sig_len != sizeof(sig) || pub_len != sizeof(pub) || msg_len > CC_MAX_INPUT) {
        return CC_FAIL;
    }
    cpu_physical_memory_read(sig_addr, sig, sizeof(sig));
    cpu_physical_memory_read(pub_addr, pub, sizeof(pub));
    g_autofree uint8_t *msg = guest_copy(msg_addr, msg_len);

    return ed25519_verify(sig, pub, msg, msg_len) ? CC_OK : CC_FAIL;
}

static void write_digest(GChecksum *hash, GHmac *hmac, uint32_t out)
{
    uint8_t digest[SHA256_LEN];
    gsize len = sizeof(digest);

    if (hash) {
        g_checksum_get_digest(hash, digest, &len);
    } else {
        g_hmac_get_digest(hmac, digest, &len);
    }
    cpu_physical_memory_write(out, digest, sizeof(digest));
}

static uint32_t call_hash(uint32_t mode, uint32_t in, uint32_t len, uint32_t out)
{
    if (mode != MODE_SHA256 || len > CC_MAX_INPUT) {
        return CC_FAIL;
    }
    g_autofree uint8_t *data = guest_copy(in, len);
    GChecksum *hash = g_checksum_new(G_CHECKSUM_SHA256);

    g_checksum_update(hash, data, len);
    write_digest(hash, NULL, out);
    g_checksum_free(hash);
    return CC_OK;
}

static uint32_t call_update(CcContext *c, uint32_t in, uint32_t len)
{
    if (!c || len > CC_MAX_INPUT) {
        return CC_FAIL;
    }
    g_autofree uint8_t *data = guest_copy(in, len);

    if (c->hash) {
        g_checksum_update(c->hash, data, len);
    } else if (c->hmac) {
        g_hmac_update(c->hmac, data, len);
    }
    return CC_OK;
}

static uint32_t call_hmac_init(uint32_t ctx, uint32_t mode, uint32_t key_addr, uint32_t key_len)
{
    if (mode != MODE_SHA256 || key_len > CC_MAX_INPUT) {
        return CC_FAIL;
    }
    g_autofree uint8_t *key = guest_copy(key_addr, key_len);

    context_new(ctx)->hmac = g_hmac_new(G_CHECKSUM_SHA256, key, key_len);
    return CC_OK;
}

static uint32_t call_finish(CcContext *c, uint32_t ctx, uint32_t out)
{
    if (!c) {
        return CC_FAIL;
    }
    write_digest(c->hash, c->hmac, out);
    g_hash_table_remove(contexts, GUINT_TO_POINTER(ctx));
    return CC_OK;
}

static uint32_t call_aes_key(CcContext *c, uint32_t key_data)
{
    uint32_t words[2];
    uint8_t key[32];

    if (!c) {
        return CC_FAIL;
    }
    cpu_physical_memory_read(key_data, words, sizeof(words));
    if (words[1] != 16 && words[1] != 24 && words[1] != 32) {
        return CC_FAIL;
    }
    cpu_physical_memory_read(words[0], key, words[1]);
    if (c->decrypt) {
        AES_set_decrypt_key(key, words[1] * 8, &c->aes);
    } else {
        AES_set_encrypt_key(key, words[1] * 8, &c->aes);
    }
    return CC_OK;
}

static uint32_t call_aes_block(CcContext *c, uint32_t in, uint32_t len, uint32_t out)
{
    if (!c || (len % AES_BLOCK_SIZE) || len > CC_MAX_INPUT) {
        return CC_FAIL;
    }
    g_autofree uint8_t *data = guest_copy(in, len);

    for (uint32_t i = 0; i < len; i += AES_BLOCK_SIZE) {
        if (c->decrypt) {
            AES_decrypt(data + i, data + i, &c->aes);
        } else {
            AES_encrypt(data + i, data + i, &c->aes);
        }
    }
    cpu_physical_memory_write(out, data, len);
    return CC_OK;
}

static uint32_t call_random(uint32_t len, uint32_t out)
{
    g_autofree uint8_t *data = g_malloc(len + 1);

    qemu_guest_getrandom_nofail(data, len);
    cpu_physical_memory_write(out, data, len);
    return CC_OK;
}

static uint32_t read32(uint32_t addr)
{
    uint32_t value = 0;

    cpu_physical_memory_read(addr, &value, 4);
    return le32_to_cpu(value);
}

bool nrf52840_cc310_call(uint32_t frame, uint32_t number, uint32_t exc_return, uint32_t *result)
{
    uint32_t r0 = read32(frame), r1 = read32(frame + 4);
    uint32_t r2 = read32(frame + 8), r3 = read32(frame + 12);
    uint32_t xpsr = read32(frame + 28);
    uint32_t stack = frame + ((exc_return & 0x10) ? 0x20 : 0x68) + ((xpsr >> 9) & 1) * 4;
    CcContext *c = context_get(r0);

    switch (number) {
    case 0xe0:
        *result = call_verify(r0, r1, r2, r3, read32(stack), read32(stack + 4));
        return true;
    case 0xe1:
        *result = call_hash(r0, r1, r2, r3);
        return true;
    case 0xe2:
        if (r1 != MODE_SHA256) {
            *result = CC_FAIL;
            return true;
        }
        context_new(r0)->hash = g_checksum_new(G_CHECKSUM_SHA256);
        *result = CC_OK;
        return true;
    case 0xe3:
    case 0xe6:
        *result = call_update(c, r1, r2);
        return true;
    case 0xe4:
    case 0xe7:
        *result = call_finish(c, r0, r1);
        return true;
    case 0xe5:
        *result = call_hmac_init(r0, r1, r2, r3 & 0xffff);
        return true;
    case 0xe8:
        context_new(r0)->decrypt = (r1 == AES_DECRYPT_MODE);
        *result = CC_OK;
        return true;
    case 0xe9:
        *result = call_aes_key(c, r2);
        return true;
    case 0xea:
        *result = call_aes_block(c, r1, r2, r3);
        return true;
    case 0xeb:
        *result = CC_OK;
        return true;
    case 0xec:
        g_hash_table_remove(contexts, GUINT_TO_POINTER(r0));
        *result = CC_OK;
        return true;
    case 0xed:
        *result = call_random(r1 & 0xffff, r2);
        return true;
    default:
        return false;
    }
}

static int find_unique(const uint8_t *image, uint32_t len, const Cc310Sig *sig)
{
    int found = -1;

    for (uint32_t off = 0; off + sig->len <= len; off += 2) {
        bool match = true;

        for (uint32_t i = 0; i < sig->len && match; i++) {
            match = !sig->mask[i] || image[off + i] == sig->bytes[i];
        }
        if (match) {
            if (found >= 0) {
                return -2;
            }
            found = off;
        }
    }
    return found;
}

void nrf52840_cc310_patch(uint8_t *image, uint32_t len)
{
    int patched = 0;

    if (!contexts) {
        contexts = g_hash_table_new_full(NULL, NULL, NULL, context_free);
    }
    for (size_t n = 0; n < ARRAY_SIZE(cc310_sigs); n++) {
        const Cc310Sig *sig = &cc310_sigs[n];
        int off = find_unique(image, len, sig);

        if (off < 0) {
            warn_report("nrf52840-mobmesh: %s %s; its crypto will not work", sig->name,
                        off == -2 ? "matched more than once" : "not found");
            continue;
        }
        stw_le_p(image + off, 0xdf00 | sig->svc);
        stw_le_p(image + off + 2, 0x4770);
        patched++;
    }
    fprintf(stderr, "nrf52840-mobmesh: %d of %zu crypto functions replaced\n", patched,
            ARRAY_SIZE(cc310_sigs));
}
