/* Host harness for the shared firmware code (tests/firmware/test_firmware.py).
 * Modes: crc | e2e | gw | wheels | cmac | guard; text protocol, doubles as C99 hex floats. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "drive_ctrl.h"
#include "gateway_core.h"
#include "guard_core.h"
#include "kauha_e2e.h"

static double rd(void) { double x; if (scanf("%la", &x) != 1) exit(0); return x; }
static long ri(void) { long x; if (scanf("%ld", &x) != 1) exit(0); return x; }
static void bytes8(uint8_t d[8]) { for (int i = 0; i < 8; i++) d[i] = (uint8_t)ri(); }

int main(int argc, char **argv) {
    if (argc < 2) return 2;
    if (!strcmp(argv[1], "crc")) {
        for (;;) { uint8_t id = (uint8_t)ri(), d[8]; bytes8(d); printf("%u\n", kauha_crc8_j1850(id, d)); }
    } else if (!strcmp(argv[1], "e2e")) {
        e2e_rx_t rx; uint8_t id = (uint8_t)ri(); double to = rd();
        kauha_e2e_rx_init(&rx, id, to);
        for (;;) {
            double t = rd(); long nf = ri(); uint8_t d[8]; bytes8(d);
            int st = nf ? (int)kauha_e2e_check(&rx, d, t) : -1;
            printf("%d %d\n", st, kauha_e2e_fresh(&rx, t));
        }
    } else if (!strcmp(argv[1], "gw")) {
        /* per line: t n (id b0..b7)*n  ->  speed articulation boom bucket brake */
        gw_t gw; gw_init(&gw);
        for (;;) {
            double t = rd(); long n = ri();
            for (long k = 0; k < n; k++) { uint32_t id = (uint32_t)ri(); uint8_t d[8]; bytes8(d); gw_receive(&gw, t, id, d); }
            gw_command_t c = gw_command(&gw, t);
            printf("%a %a %a %a %d %u\n", c.speed, c.articulation, c.boom, c.bucket, c.brake, gw.clamped);
        }
    } else if (!strcmp(argv[1], "wheels")) {
        drive_geom_t g = {rd(), rd(), rd(), 1.2, 2.0};
        for (;;) {
            double v = rd(), gam = rd(), gd = rd(), w[4];
            drive_wheel_speeds(&g, v, gam, gd, w);
            printf("%a %a %a %a %a\n", w[0], w[1], w[2], w[3], drive_speed_estimate(&g, w, gam));
        }
    } else if (!strcmp(argv[1], "cmac")) {
        /* per line: key(16) len msg(len)  ->  tag as 16 numbers */
        for (;;) {
            uint8_t key[16], msg[80], tag[16]; cmac_key_t k;
            for (int i = 0; i < 16; i++) key[i] = (uint8_t)ri();
            long n = ri();
            for (long i = 0; i < n; i++) msg[i] = (uint8_t)ri();
            kauha_cmac_init(&k, key); kauha_cmac(&k, msg, (size_t)n, tag);
            for (int i = 0; i < 16; i++) printf("%u%c", tag[i], i == 15 ? '\n' : ' ');
        }
    } else if (!strcmp(argv[1], "guard")) {
        /* first line: 3 keys (16 each) and the challenge (8); then per line: t id dlc b0..b7
         *  ->  forwarded id (or -1) and the six counters */
        uint8_t keys[GUARD_SENDERS][16], ch[8]; guard_t g;
        for (int k = 0; k < GUARD_SENDERS; k++) for (int i = 0; i < 16; i++) keys[k][i] = (uint8_t)ri();
        bytes8(ch);
        guard_init(&g, (const uint8_t (*)[16])keys, ch);
        for (;;) {
            double t = rd(); uint32_t id = (uint32_t)ri(); uint8_t dlc = (uint8_t)ri(), d[8], od[8]; uint32_t oid = 0;
            bytes8(d);
            bool f = guard_from_open(&g, t, id, d, dlc, &oid, od);
            printf("%ld %u %u %u %u %u %u\n", f ? (long)oid : -1L, g.stats.forwarded, g.stats.dropped_not_allowed,
                   g.stats.dropped_no_auth, g.stats.dropped_bad_mac, g.stats.dropped_stale, g.stats.intrusions);
        }
    }
    return 0;
}
