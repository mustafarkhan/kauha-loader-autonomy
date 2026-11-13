#include "guard_core.h"

#include <string.h>

static const struct { uint32_t pgn; uint8_t sa; } ALLOWED[GUARD_SLOTS] = {
    {0xFF10u, 0x80}, {0xFF11u, 0x80}, {0xFF40u, 0x82}, {0xFF52u, 0x86}};
static const uint8_t SENDERS[GUARD_SENDERS] = {0x80, 0x82, 0x86};
static const uint8_t SAFETY_SOURCES[4] = {0x81, 0x83, 0x84, 0x85};

void guard_init(guard_t *g, const uint8_t keys[GUARD_SENDERS][16], const uint8_t challenge[8]) {
    memset(g, 0, sizeof *g);
    memcpy(g->challenge, challenge, 8);
    for (int i = 0; i < GUARD_SENDERS; i++) {
        g->sender_sa[i] = SENDERS[i];
        kauha_session_key(keys[i], challenge, &g->skey[i]);
    }
    for (int i = 0; i < GUARD_SLOTS; i++) { g->slot[i].pgn = ALLOWED[i].pgn; g->slot[i].sa = ALLOWED[i].sa; }
}

bool guard_challenge_due(guard_t *g, double t) {
    if (t < g->next_challenge) return false;
    g->next_challenge = t + GUARD_CHALLENGE_PERIOD_S;
    return true;
}

static guard_slot_t *find_slot(guard_t *g, uint8_t sa, uint8_t data_id) {
    for (int i = 0; i < GUARD_SLOTS; i++)
        if (g->slot[i].sa == sa && (g->slot[i].pgn & 0xFFu) == data_id) return &g->slot[i];
    return NULL;
}

static bool on_auth(guard_t *g, double t, uint8_t sa, const uint8_t *d, uint8_t dlc,
                    uint32_t *out_id, uint8_t out_data[8]) {
    int si = -1;
    for (int i = 0; i < GUARD_SENDERS; i++) if (g->sender_sa[i] == sa) si = i;
    if (si < 0 || dlc != 8) { g->stats.dropped_not_allowed++; return false; }
    guard_slot_t *s = find_slot(g, sa, d[0]);
    bool had = s && s->pending;
    if (s) s->pending = false;
    if (!had || t - s->t > GUARD_PAIR_WINDOW_S) { g->stats.dropped_no_auth++; return false; }
    uint32_t fresh = (uint32_t)d[1] | ((uint32_t)d[2] << 8) | ((uint32_t)d[3] << 16);
    if (fresh <= s->last) { g->stats.dropped_stale++; return false; }
    uint8_t want[8], diff = 0;
    kauha_auth_payload(&g->skey[si], d[0], s->frame_id, s->data, fresh, want);
    for (int i = 4; i < 8; i++) diff |= (uint8_t)(want[i] ^ d[i]);       /* no early exit */
    if (diff) { g->stats.dropped_bad_mac++; return false; }
    s->last = fresh;
    g->stats.forwarded++;
    *out_id = s->frame_id;
    memcpy(out_data, s->data, 8);
    return true;
}

bool guard_from_open(guard_t *g, double t, uint32_t frame_id, const uint8_t *data, uint8_t dlc,
                     uint32_t *out_id, uint8_t out_data[8]) {
    uint32_t pgn = (frame_id >> 8) & 0x3FFFFu;
    uint8_t sa = (uint8_t)frame_id;
    if (pgn == GUARD_AUTH_PGN) return on_auth(g, t, sa, data, dlc, out_id, out_data);
    for (int i = 0; i < 4; i++) if (sa == SAFETY_SOURCES[i]) { g->stats.intrusions++; return false; }
    guard_slot_t *s = NULL;
    for (int i = 0; i < GUARD_SLOTS; i++) if (g->slot[i].pgn == pgn && g->slot[i].sa == sa) s = &g->slot[i];
    if (!s || dlc != 8) { g->stats.dropped_not_allowed++; return false; }
    if (s->pending) g->stats.dropped_no_auth++;
    s->pending = true;
    s->frame_id = frame_id;
    memcpy(s->data, data, 8);
    s->t = t;
    return false;
}
