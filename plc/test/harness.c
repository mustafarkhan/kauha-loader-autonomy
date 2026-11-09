/* Test harness around the matiec translation of the KAUHA safety PLC sources.
 *
 * Built by tests/plc/test_st_equivalence.py:
 *   iec2c the ST sources -> POUS.c/POUS.h, then gcc this file (it includes them).
 * Doubles cross the boundary as C99 hex floats (%a), so comparisons are exact.
 *
 * Modes (argv[1]):
 *   core    first line: configuration; then one line of inputs per cycle;
 *           prints outputs and FB memory after every cycle.
 *   crc     lines "data_id b0 .. b7" -> CRC8_J1850
 *   e2e     first line "data_id timeout max_delta"; then "t new b0 .. b7" ->
 *           "status accepted fresh age"
 *   node    configuration line, then per cycle: t estop_ok use_io io[7] n (id b0..b7)*n
 *           -> state cause speed_limit field_set drive impl code tx_id faults tx[8]
 *   status  lines "state cause speed_limit field_set reset_pending dock_blocked counter data_id"
 *           -> 8 payload bytes of ENCODE_SAFETY_STATUS
 */
#include "iec_std_lib.h"
#include "accessor.h"
#include "POUS.h"
#include "POUS.c"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

TIME __CURRENT_TIME;
BOOL __DEBUG;
double iec_lib_fmod(double x, double y) { return fmod(x, y); }

static double rd(void) { double x; if (scanf("%la", &x) != 1) exit(0); return x; }
static long ri(void) { long x; if (scanf("%ld", &x) != 1) exit(0); return x; }

#define IN(name) d.INP.value.name.value
#define OUT(name) d.OUT.value.name.value
#define CF(name) c->name.value

static void read_cfg(ST_SAFETYCONFIG *c) {
    /* configuration, in the order of kauha/verify/st_equivalence.py config_values() */
    CF(REMOTE_TIMEOUT_S) = rd(); CF(WATCHDOG_TIMEOUT_S) = rd(); CF(OVERSPEED_TOL) = rd();
    CF(DECEL_RAMP) = rd(); CF(RAMP_REACTION_S) = rd(); CF(DISCREPANCY_TOL) = rd();
    CF(DISCREPANCY_TIME_S) = rd(); CF(POSITION_INVALID_S) = rd(); CF(MAX_MUTE_S) = rd();
    CF(AUTO_RESTART_CLEAR_S) = rd(); CF(WARNING_HOLD_S) = rd(); CF(AUTO_SPEED) = rd();
    CF(TELEOP_SPEED) = rd(); CF(WARNING_SPEED) = rd(); CF(STANDSTILL) = rd();
    CF(RECOVERY_SPEED) = rd(); CF(SIDE_RECOVERY_SPEED) = rd(); CF(T_STANDSTILL) = rd();
    CF(DIG_SPEED) = rd(); CF(DOCK_SPEED) = rd(); CF(SPEED_CAP) = rd();
    CF(AUTO_RESTART) = ri();
    CF(BAND0) = rd(); CF(BAND1) = rd(); CF(BAND2) = rd(); CF(BAND3) = rd();
    CF(SECTOR0) = rd(); CF(SECTOR1) = rd(); CF(SECTOR2) = rd(); CF(SECTOR3) = rd();
    CF(VPROT0) = rd(); CF(VPROT1) = rd(); CF(VPROT2) = rd(); CF(VPROT3) = rd();
    CF(N_SECTORS) = ri(); CF(N_BANDS) = ri();
}

static void core(void) {
    static FB_SAFETYCORE_data__ d;               /* zeroed, like PLC static memory */
    FB_SAFETYCORE_init__(&d, 0);
    read_cfg(&d.CFG.value);
    for (;;) {
        IN(T) = rd(); IN(MODE) = ri(); IN(SPEED_A) = rd(); IN(SPEED_B) = rd(); IN(ARTICULATION) = rd();
        IN(FRONT_PROT) = ri(); IN(FRONT_WARN) = ri(); IN(REAR_PROT) = ri(); IN(REAR_WARN) = ri();
        IN(SIDE_PROT) = ri(); IN(SIDE_CONTACT) = ri(); IN(SCANNERS_OK) = ri();
        IN(REMOTE_ESTOP) = ri(); IN(RESET_REQ) = ri(); IN(MACHINE_ESTOP) = ri();
        IN(DIG_REQ) = ri(); IN(DOCK_REQ) = ri(); IN(IN_OPERATING_AREA) = ri(); IN(IN_DIG_ZONE) = ri();
        IN(IN_DOCK_ZONE) = ri(); IN(TRUCK_DOOR_OPEN) = ri(); IN(BOOM_RAISED) = ri(); IN(FRONT_SIDE) = ri();
        IN(UNDER_NEAR) = ri(); IN(UNDER_FAR) = ri(); IN(UNDER_MID) = ri(); IN(POSITION_VALID) = ri();
        IN(REMOTE_HB_AGE) = rd(); IN(AUTONOMY_CMD_AGE) = rd();
        FB_SAFETYCORE_body__(&d);
        printf("%d %u %a %d %d %d %d %d %d %a %u %d %d %d",
               OUT(STATE), OUT(CAUSE), OUT(SPEED_LIMIT), OUT(FIELD_SET), OUT(BRAKE_RELEASE),
               OUT(IMPLEMENT_ENABLE), OUT(RESET_PENDING), OUT(DOCK_BLOCKED), OUT(FS_DIRECTION),
               OUT(FS_V_PROTECTED), OUT(RECOVERY), OUT(EV_STOP), OUT(EV_RECOVERY), OUT(EV_RESET));
        printf(" %d %u %d %u %d %d %a %d %a %d %a %d %a %d %a %a %a %a %a %a\n",
               d.M_STATE.value, d.M_LATCHED.value, d.M_FIELD_SET.value, d.M_RECOVERY.value,
               d.M_PREV_RESET.value, d.M_OVER_ON.value, d.M_OVER_T0.value, d.M_DISC_ON.value,
               d.M_DISC_T0.value, d.M_POSINV_ON.value, d.M_POSINV_T0.value, d.M_MUTE_ON.value,
               d.M_MUTE_T0.value, d.M_CLEAR_ON.value, d.M_CLEAR_T0.value, d.M_WARN_UNTIL.value,
               d.M_ALLOWED.value, d.M_PREV_LIMIT.value, d.M_ENV_T0.value, d.M_ENV_V0.value);
    }
}

static void read_payload(ST_PAYLOAD *p) {
    for (int i = 0; i < 8; i++) p->B.table[i].value = (BYTE)ri();
}

static void crc(void) {
    for (;;) {
        BYTE id = (BYTE)ri();
        ST_PAYLOAD p; read_payload(&p);
        BOOL eno;
        printf("%u\n", ___CRC8_J1850(1, &eno, id, p));
    }
}

static void e2e(void) {
    static FB_E2ERX_data__ d;               /* zeroed, like PLC static memory */
    FB_E2ERX_init__(&d, 0);
    d.DATA_ID.value = (BYTE)ri(); d.TIMEOUT_S.value = rd(); d.MAX_DELTA.value = ri();
    for (;;) {
        d.T.value = rd(); d.NEW_FRAME.value = ri();
        for (int i = 0; i < 8; i++) d.DATA.value.B.table[i].value = (BYTE)ri();
        FB_E2ERX_body__(&d);
        printf("%d %d %d %a\n", d.STATUS.value, d.ACCEPTED.value, d.FRESH.value, d.AGE.value);
    }
}

static void status(void) {
    for (;;) {
        ST_SAFETYOUTPUTS o;
        memset(&o, 0, sizeof o);
        o.STATE.value = ri(); o.CAUSE.value = ri(); o.SPEED_LIMIT.value = rd(); o.FIELD_SET.value = ri();
        o.RESET_PENDING.value = ri(); o.DOCK_BLOCKED.value = ri();
        INT ctr = ri(); BYTE id = (BYTE)ri();
        ST_PAYLOAD p; memset(&p, 0, sizeof p);
        BOOL eno;
        ___ENCODE_SAFETY_STATUS(1, &eno, o, ctr, id, &p);
        for (int i = 0; i < 8; i++) printf(i ? " %u" : "%u", p.B.table[i].value);
        printf("\n");
    }
}


static void node(void) {
    static FB_SAFETYNODE_data__ d;               /* zeroed, like PLC static memory */
    FB_SAFETYNODE_init__(&d, 0);
    read_cfg(&d.CFG.value);
    for (;;) {
        d.T.value = rd(); d.ESTOP_CHAIN_OK.value = ri(); d.USE_SCANNER_IO.value = ri();
        ST_SCANNERIO *io = &d.SCANNER_IO.value;
        io->FRONT_OSSD_CLEAR.value = ri(); io->REAR_OSSD_CLEAR.value = ri(); io->FRONT_WARN_CLEAR.value = ri();
        io->REAR_WARN_CLEAR.value = ri(); io->SIDE_CLEAR.value = ri(); io->UNDER_CLEAR.value = ri();
        io->SCANNERS_READY.value = ri();
        long n = ri();
        d.RX.value.N.value = (INT)n;
        for (long k = 0; k < n; k++) {
            d.RX.value.F.table[k].ID.value = (DWORD)ri();
            for (int i = 0; i < 8; i++) d.RX.value.F.table[k].DATA.B.table[i].value = (BYTE)ri();
        }
        FB_SAFETYNODE_body__(&d);
        ST_SAFETYOUTPUTS *o = &d.OUT.value;
        printf("%d %u %a %d %d %d %d %u %d %d", o->STATE.value, o->CAUSE.value, o->SPEED_LIMIT.value,
               o->FIELD_SET.value, d.DRIVE_ENABLE.value, d.IMPLEMENT_ENABLE.value, d.FIELD_SET_CODE.value,
               d.TX_ID.value, d.SENSOR_FAULT.value, d.SCANNER_FAULT.value);
        for (int i = 0; i < 8; i++) printf(" %u", d.TX.value.B.table[i].value);
        printf("\n");
        fflush(stdout);   /* closed loop: one line out per line in */
    }
}

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IOFBF, 1 << 16);
    if (argc < 2) return 2;
    if (!strcmp(argv[1], "core")) core();
    else if (!strcmp(argv[1], "crc")) crc();
    else if (!strcmp(argv[1], "e2e")) e2e();
    else if (!strcmp(argv[1], "status")) status();
    else if (!strcmp(argv[1], "node")) node();
    return 0;
}
