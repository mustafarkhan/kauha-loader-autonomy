/* Segment guard between the open bus segment and the safety segment.
 * Port of kauha/can/guard.py (SegmentGuard); see that file for the rules and what they
 * do and do not protect against. tests/firmware compares the two on random and attack
 * frame streams. */
#ifndef KAUHA_GUARD_CORE_H
#define KAUHA_GUARD_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "kauha_cmac.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GUARD_PAIR_WINDOW_S 0.02
#define GUARD_CHALLENGE_PERIOD_S 1.0
#define GUARD_SENDERS 3            /* autonomy 0x80, remote gateway 0x82, zone source 0x86 */
#define GUARD_SLOTS 4              /* allowed frames: DriveCmd, ImplementCmd, RemoteHeartbeat, ZoneStatus */
#define GUARD_AUTH_PGN 0xFF60u
#define GUARD_CHALLENGE_ID (0x18000000u | (0xFF61u << 8) | 0x83u)

typedef struct {
    uint32_t forwarded, dropped_not_allowed, dropped_no_auth, dropped_bad_mac, dropped_stale, intrusions;
} guard_stats_t;

typedef struct {
    uint32_t pgn; uint8_t sa;      /* the allowed frame */
    bool pending; uint32_t frame_id; uint8_t data[8]; double t;
    uint32_t last;                 /* last accepted freshness */
} guard_slot_t;

typedef struct {
    uint8_t sender_sa[GUARD_SENDERS];
    cmac_key_t skey[GUARD_SENDERS];
    guard_slot_t slot[GUARD_SLOTS];
    uint8_t challenge[8];
    double next_challenge;
    guard_stats_t stats;
} guard_t;

/* keys[i] is the 128-bit key of sender_sa {0x80, 0x82, 0x86}[i]; challenge is 8 random bytes
 * drawn at power-up from a hardware random source. */
void guard_init(guard_t *g, const uint8_t keys[GUARD_SENDERS][16], const uint8_t challenge[8]);
/* One frame from the open segment. Returns true and fills out_id/out_data when a frame is to
 * be forwarded to the safety segment. dlc is the received data length. */
bool guard_from_open(guard_t *g, double t, uint32_t frame_id, const uint8_t *data, uint8_t dlc,
                     uint32_t *out_id, uint8_t out_data[8]);
/* True once per period: send GUARD_CHALLENGE_ID with g->challenge on the open segment. */
bool guard_challenge_due(guard_t *g, double t);

#ifdef __cplusplus
}
#endif
#endif
