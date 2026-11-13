/* AES-128 CMAC (RFC 4493) for frame authentication. Port of kauha/can/auth.py; the host
 * tests compare the two bit for bit, including the RFC test vectors.
 *
 * Small table-free AES (S-box only), encrypt direction only. Not a certified module.
 * Keys are loaded at commissioning; none are compiled into the firmware. */
#ifndef KAUHA_CMAC_H
#define KAUHA_CMAC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t rk[11][16];
    uint8_t k1[16], k2[16];
} cmac_key_t;

void kauha_cmac_init(cmac_key_t *k, const uint8_t key[16]);
void kauha_cmac(const cmac_key_t *k, const uint8_t *msg, size_t len, uint8_t tag[16]);
/* Session key for one power cycle of the guard: CMAC(sender key, "KAUHA-S1" | challenge). */
void kauha_session_key(const uint8_t sender_key[16], const uint8_t challenge[8], cmac_key_t *out);
/* The 8 data bytes of a FrameAuth companion frame. */
void kauha_auth_payload(const cmac_key_t *skey, uint8_t data_id, uint32_t frame_id, const uint8_t payload[8],
                        uint32_t freshness, uint8_t out[8]);

#ifdef __cplusplus
}
#endif
#endif
