#include "kauha_cmac.h"

#include <string.h>

static uint8_t sbox[256];
static int sbox_ready;

static uint8_t rotl8(uint8_t x, int n) { return (uint8_t)((x << n) | (x >> (8 - n))); }
static uint8_t xt(uint8_t a) { return (uint8_t)((a << 1) ^ ((a & 0x80) ? 0x1B : 0)); }

static void init_sbox(void) {
    uint8_t p = 1, q = 1;
    do {
        p = (uint8_t)(p ^ (p << 1) ^ ((p & 0x80) ? 0x1B : 0));
        q ^= (uint8_t)(q << 1);
        q ^= (uint8_t)(q << 2);
        q ^= (uint8_t)(q << 4);
        if (q & 0x80) q ^= 0x09;
        sbox[p] = (uint8_t)(q ^ rotl8(q, 1) ^ rotl8(q, 2) ^ rotl8(q, 3) ^ rotl8(q, 4) ^ 0x63);
    } while (p != 1);
    sbox[0] = 0x63;
    sbox_ready = 1;
}

static void expand(const uint8_t key[16], uint8_t rk[11][16]) {
    static const uint8_t rcon[10] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36};
    uint8_t w[44][4];
    memcpy(w, key, 16);
    for (int i = 4; i < 44; i++) {
        uint8_t t[4];
        memcpy(t, w[i - 1], 4);
        if (i % 4 == 0) {
            uint8_t t0 = t[0];
            t[0] = (uint8_t)(sbox[t[1]] ^ rcon[i / 4 - 1]);
            t[1] = sbox[t[2]];
            t[2] = sbox[t[3]];
            t[3] = sbox[t0];
        }
        for (int j = 0; j < 4; j++) w[i][j] = (uint8_t)(w[i - 4][j] ^ t[j]);
    }
    memcpy(rk, w, 176);
}

static void encrypt(const uint8_t rk[11][16], const uint8_t in[16], uint8_t out[16]) {
    uint8_t s[16], m[16];
    for (int i = 0; i < 16; i++) s[i] = (uint8_t)(in[i] ^ rk[0][i]);
    for (int rnd = 1; rnd <= 10; rnd++) {
        for (int i = 0; i < 16; i++) m[i] = sbox[s[(i + 4 * (i % 4)) % 16]];    /* sub bytes + shift rows */
        if (rnd < 10) {
            for (int c = 0; c < 16; c += 4) {
                uint8_t a0 = m[c], a1 = m[c + 1], a2 = m[c + 2], a3 = m[c + 3];
                uint8_t t = (uint8_t)(a0 ^ a1 ^ a2 ^ a3);
                s[c] = (uint8_t)(a0 ^ t ^ xt((uint8_t)(a0 ^ a1)));
                s[c + 1] = (uint8_t)(a1 ^ t ^ xt((uint8_t)(a1 ^ a2)));
                s[c + 2] = (uint8_t)(a2 ^ t ^ xt((uint8_t)(a2 ^ a3)));
                s[c + 3] = (uint8_t)(a3 ^ t ^ xt((uint8_t)(a3 ^ a0)));
            }
        } else {
            memcpy(s, m, 16);
        }
        for (int i = 0; i < 16; i++) s[i] ^= rk[rnd][i];
    }
    memcpy(out, s, 16);
}

static void dbl(const uint8_t in[16], uint8_t out[16]) {
    uint8_t carry = (uint8_t)(in[0] >> 7);
    for (int i = 0; i < 15; i++) out[i] = (uint8_t)((in[i] << 1) | (in[i + 1] >> 7));
    out[15] = (uint8_t)((in[15] << 1) ^ (carry ? 0x87 : 0));
}

void kauha_cmac_init(cmac_key_t *k, const uint8_t key[16]) {
    uint8_t zero[16] = {0}, l[16];
    if (!sbox_ready) init_sbox();
    expand(key, k->rk);
    encrypt(k->rk, zero, l);
    dbl(l, k->k1);
    dbl(k->k1, k->k2);
}

void kauha_cmac(const cmac_key_t *k, const uint8_t *msg, size_t len, uint8_t tag[16]) {
    size_t n = len ? (len + 15) / 16 : 1;
    size_t tail = len - (n - 1) * 16;
    uint8_t x[16] = {0}, last[16] = {0};
    if (len && tail == 16) {
        for (int i = 0; i < 16; i++) last[i] = (uint8_t)(msg[(n - 1) * 16 + i] ^ k->k1[i]);
    } else {
        if (len) memcpy(last, msg + (n - 1) * 16, tail);
        else tail = 0;
        last[tail] = 0x80;
        for (int i = 0; i < 16; i++) last[i] ^= k->k2[i];
    }
    for (size_t b = 0; b + 1 < n; b++) {
        for (int i = 0; i < 16; i++) x[i] ^= msg[b * 16 + i];
        encrypt(k->rk, x, x);
    }
    for (int i = 0; i < 16; i++) x[i] ^= last[i];
    encrypt(k->rk, x, tag);
}

void kauha_session_key(const uint8_t sender_key[16], const uint8_t challenge[8], cmac_key_t *out) {
    cmac_key_t base;
    uint8_t in[16], sk[16];
    memcpy(in, "KAUHA-S1", 8);
    memcpy(in + 8, challenge, 8);
    kauha_cmac_init(&base, sender_key);
    kauha_cmac(&base, in, 16, sk);
    kauha_cmac_init(out, sk);
}

void kauha_auth_payload(const cmac_key_t *skey, uint8_t data_id, uint32_t frame_id, const uint8_t payload[8],
                        uint32_t freshness, uint8_t out[8]) {
    uint8_t in[16], tag[16];
    frame_id &= 0x1FFFFFFFu;
    in[0] = data_id;
    in[1] = (uint8_t)(frame_id >> 24); in[2] = (uint8_t)(frame_id >> 16);
    in[3] = (uint8_t)(frame_id >> 8);  in[4] = (uint8_t)frame_id;
    memcpy(in + 5, payload, 8);
    in[13] = (uint8_t)freshness; in[14] = (uint8_t)(freshness >> 8); in[15] = (uint8_t)(freshness >> 16);
    kauha_cmac(skey, in, 16, tag);
    out[0] = data_id;
    out[1] = in[13]; out[2] = in[14]; out[3] = in[15];
    memcpy(out + 4, tag, 4);
}
