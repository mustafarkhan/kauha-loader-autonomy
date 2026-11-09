(* KAUHA safety PLC: one 25 ms cycle of the safety logic.

   This is a line-by-line port of kauha/safety/core.py step(), the reference that
   the z3 proofs (120 properties) and the adversarial searches run on. The test
   tests/plc/test_st_equivalence.py compiles this file with matiec and checks it
   against the reference on long random and scripted input sequences: every
   output and every memory value must match bit for bit.

   Keep it that way: change core.py first, prove it, then port the change here
   and rerun the equivalence test. Requirement tags (SR-xx) follow core.py. *)

FUNCTION_BLOCK FB_SafetyCore
VAR_INPUT
    cfg : ST_SafetyConfig;
    inp : ST_SafetyInputs;
END_VAR
VAR_OUTPUT
    out : ST_SafetyOutputs;
END_VAR
VAR
    (* memory between cycles (core.Memory) *)
    m_state      : INT := 0;
    m_latched    : WORD := 0;
    m_field_set  : INT := 0;
    m_recovery   : WORD := 0;
    m_prev_reset : BOOL := FALSE;
    m_over_on    : BOOL := FALSE;
    m_over_t0    : LREAL := 0.0;
    m_disc_on    : BOOL := FALSE;
    m_disc_t0    : LREAL := 0.0;
    m_posinv_on  : BOOL := FALSE;
    m_posinv_t0  : LREAL := 0.0;
    m_mute_on    : BOOL := FALSE;
    m_mute_t0    : LREAL := 0.0;
    m_clear_on   : BOOL := FALSE;
    m_clear_t0   : LREAL := 0.0;
    m_warn_until : LREAL := -1.0;
    m_allowed    : LREAL := 0.0;
    m_prev_limit : LREAL := 0.0;
    m_env_t0     : LREAL := -1.0E9;
    m_env_v0     : LREAL := 0.0;
    (* cycle temporaries *)
    t, v, a_v       : LREAL;
    st              : INT;
    lat, rec        : WORD;
    c               : BOOL;
    ev_stop         : BOOL;
    rising_reset    : BOOL;
    dig_ok, dock_ok : BOOL;
    muting          : BOOL;
    mute_t0, posinv_t0, disc_t0, over_t0, clear_t0 : LREAL;
    mute_timeout    : BOOL;
    dig_sel, dock_sel, still : BOOL;
    sector, band, base, moving_id : INT;
    fs_id, fs_dir   : INT;
    v_band, fs_vprot : LREAL;
    at_rest         : BOOL;
    exempt, under   : BOOL;
    active_mode, is_auto, posinv, posinv_on, disc : BOOL;
    in_zone, warn_trig, warning : BOOL;
    warn_until      : LREAL;
    spd_lim, creep, env_t0, env_v0, env_ramp, allowed : LREAL;
    dec, over, travel_set, standstill : BOOL;
    active, remaining : WORD;
    stopped_now, enter_rec, clear_on, auto_ok, do_reset, stopped : BOOL;
END_VAR
VAR CONSTANT
    RUN_ST : INT := 0;
    WARNING_ST : INT := 1;
    PROTECTIVE_STOP : INT := 2;
    EMERGENCY_STOP : INT := 3;
    MODE_IDLE : INT := 0;
    MODE_AUTO : INT := 1;
    MODE_TELEOP : INT := 2;
    FIELD_FRONT : WORD := 16#0001;
    FIELD_REAR : WORD := 16#0002;
    FIELD_SIDE : WORD := 16#0004;
    OVERSPEED : WORD := 16#0008;
    REMOTE_LINK : WORD := 16#0010;
    WATCHDOG : WORD := 16#0020;
    GEOFENCE : WORD := 16#0040;
    SPEED_DISCREPANCY : WORD := 16#0080;
    SCANNER_FAULT : WORD := 16#0100;
    MUTING_TIMEOUT : WORD := 16#0200;
    ESTOP_MACHINE : WORD := 16#0400;
    ESTOP_REMOTE : WORD := 16#0800;
    POSITION_INVALID : WORD := 16#1000;
    TRUCK_DOOR : WORD := 16#2000;
    UNDER_BUCKET : WORD := 16#4000;
    FIELD_CAUSES : WORD := 16#4007;      (* FRONT | REAR | SIDE | UNDER_BUCKET *)
    CLEARING_CAUSES : WORD := 16#6007;   (* FIELD_CAUSES | TRUCK_DOOR *)
    CYCLE_S : LREAL := 0.025;
    DIG_SET : INT := 100;
    DOCK_SET : INT := 101;
    STANDSTILL_SET : INT := 0;
END_VAR

t := inp.t;
v := inp.speed_b;
st := m_state;
lat := m_latched;
ev_stop := FALSE;
out.ev_recovery := FALSE;
out.ev_reset := FALSE;

rising_reset := inp.reset_req AND NOT m_prev_reset;

(* SR-01 emergency stops: latch. *)
c := inp.machine_estop;
IF c THEN
    IF (EMERGENCY_STOP > st) OR ((ESTOP_MACHINE AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR ESTOP_MACHINE;
    IF EMERGENCY_STOP > st THEN st := EMERGENCY_STOP; END_IF;
END_IF;
c := inp.remote_estop;
IF c THEN
    IF (EMERGENCY_STOP > st) OR ((ESTOP_REMOTE AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR ESTOP_REMOTE;
    IF EMERGENCY_STOP > st THEN st := EMERGENCY_STOP; END_IF;
END_IF;

(* Field-set selection (SR-02, SR-10, SR-14). *)
dig_ok := inp.dig_req AND inp.in_dig_zone AND (ABS(v) <= cfg.dig_speed + cfg.overspeed_tol) AND inp.position_valid;
dock_ok := inp.dock_req AND inp.in_dock_zone AND (ABS(v) <= cfg.dock_speed + cfg.overspeed_tol)
           AND inp.position_valid AND NOT inp.truck_door_open;
(* SR-14: the driver leaving the cab while the loader is docking stops it. *)
c := inp.truck_door_open AND (m_field_set = DOCK_SET) AND inp.in_dock_zone;
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((TRUCK_DOOR AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR TRUCK_DOOR;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
muting := dig_ok AND (v > -cfg.t_standstill);
mute_t0 := SEL(muting AND m_mute_on, t, m_mute_t0);
mute_timeout := muting AND (t - mute_t0 > cfg.max_mute_s);
c := mute_timeout;
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((MUTING_TIMEOUT AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR MUTING_TIMEOUT;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
dig_ok := dig_ok AND NOT mute_timeout;

(* select(): set id, direction, protected speed *)
dig_sel := dig_ok AND (v > -cfg.t_standstill);
dock_sel := NOT dig_sel AND dock_ok AND (v > -cfg.t_standstill);
still := ABS(v) < cfg.t_standstill;
a_v := ABS(v);
IF inp.articulation < cfg.sector0 THEN sector := 0;
ELSIF inp.articulation < cfg.sector1 THEN sector := 1;
ELSIF inp.articulation < cfg.sector2 THEN sector := 2;
ELSIF inp.articulation < cfg.sector3 THEN sector := 3;
ELSE sector := 4;
END_IF;
IF a_v <= cfg.band0 THEN band := 0;
ELSIF a_v <= cfg.band1 THEN band := 1;
ELSIF a_v <= cfg.band2 THEN band := 2;
ELSE band := 3;
END_IF;
base := SEL(v > 0.0, 1 + cfg.n_bands * cfg.n_sectors, 1);
moving_id := base + band * cfg.n_sectors + sector;
IF dig_sel THEN fs_id := DIG_SET;
ELSIF dock_sel THEN fs_id := DOCK_SET;
ELSIF still THEN fs_id := STANDSTILL_SET;
ELSE fs_id := moving_id;
END_IF;
IF dig_sel OR dock_sel THEN fs_dir := 1;
ELSIF still THEN fs_dir := 0;
ELSIF v > 0.0 THEN fs_dir := 1;
ELSE fs_dir := -1;
END_IF;
IF band = 0 THEN v_band := cfg.vprot0;
ELSIF band = 1 THEN v_band := cfg.vprot1;
ELSIF band = 2 THEN v_band := cfg.vprot2;
ELSE v_band := cfg.vprot3;
END_IF;
IF dig_sel THEN fs_vprot := cfg.dig_speed;
ELSIF dock_sel THEN fs_vprot := cfg.dock_speed;
ELSIF still THEN fs_vprot := cfg.t_standstill;
ELSE fs_vprot := v_band;
END_IF;

(* SR-12 recovery: after a supervisor reset in TELEOP, motion away from an occupied
   field is allowed at creep speed; motion toward it trips again. A field counts as
   clear only at standstill. *)
rec := SEL(inp.mode = MODE_TELEOP, WORD#0, m_recovery);
at_rest := ABS(v) < cfg.standstill;
IF at_rest AND NOT inp.front_prot THEN rec := rec AND NOT FIELD_FRONT; END_IF;
IF at_rest AND NOT inp.rear_prot THEN rec := rec AND NOT FIELD_REAR; END_IF;
IF at_rest AND NOT inp.side_prot THEN rec := rec AND NOT FIELD_SIDE; END_IF;
(* SR-16: someone between the front tyres and the bucket, or under the raised bucket. *)
exempt := inp.in_dig_zone OR (inp.in_dock_zone AND NOT inp.truck_door_open);
under := NOT inp.in_dig_zone AND (inp.under_near OR (NOT exempt AND (inp.under_mid OR (inp.boom_raised AND inp.under_far))));
IF at_rest AND NOT under THEN rec := rec AND NOT UNDER_BUCKET; END_IF;

c := HAS(rec, FIELD_SIDE) AND inp.side_contact AND NOT exempt;
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((FIELD_SIDE AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR FIELD_SIDE;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
c := HAS(rec, UNDER_BUCKET) AND (v > cfg.standstill);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((UNDER_BUCKET AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR UNDER_BUCKET;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
c := HAS(rec, FIELD_FRONT) AND (v > cfg.standstill);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((FIELD_FRONT AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR FIELD_FRONT;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
c := HAS(rec, FIELD_REAR) AND (v < -cfg.standstill);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((FIELD_REAR AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR FIELD_REAR;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;

(* SR-02 protective fields. The scanners evaluate the selected set. *)
c := inp.front_prot AND (fs_dir >= 0) AND NOT HAS(rec, FIELD_FRONT);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((FIELD_FRONT AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR FIELD_FRONT;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
c := inp.rear_prot AND ((fs_dir <= 0) OR (ABS(v) < cfg.standstill)) AND NOT HAS(rec, FIELD_REAR);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((FIELD_REAR AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR FIELD_REAR;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
c := inp.side_prot AND (ABS(v) > cfg.standstill) AND NOT HAS(rec, FIELD_SIDE);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((FIELD_SIDE AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR FIELD_SIDE;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
c := NOT inp.scanners_ok;
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((SCANNER_FAULT AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR SCANNER_FAULT;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
(* SR-16: a person under the raised bucket stops travel and holds the boom. *)
c := under AND NOT HAS(rec, UNDER_BUCKET);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((UNDER_BUCKET AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR UNDER_BUCKET;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
(* Reversing: someone stepping in beside the front frame heads for the gap between
   the front tyres and the bucket's back edge. *)
c := inp.front_side AND (v < -cfg.standstill) AND NOT exempt AND NOT HAS(rec, FIELD_SIDE);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((FIELD_SIDE AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR FIELD_SIDE;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;

(* SR-05 remote supervision link, SR-06 autonomy watchdog. *)
active_mode := (inp.mode = MODE_AUTO) OR (inp.mode = MODE_TELEOP);
c := active_mode AND (inp.remote_hb_age > cfg.remote_timeout_s);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((REMOTE_LINK AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR REMOTE_LINK;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
c := active_mode AND (inp.autonomy_cmd_age > cfg.watchdog_timeout_s);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((WATCHDOG AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR WATCHDOG;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;

(* SR-07 geofence; an invalid position is tolerated briefly. *)
is_auto := inp.mode = MODE_AUTO;
posinv := is_auto AND NOT inp.position_valid;
posinv_t0 := SEL(posinv AND NOT m_posinv_on, m_posinv_t0, t);
posinv_on := SEL(is_auto, m_posinv_on, NOT inp.position_valid);
c := posinv AND (t - posinv_t0 > cfg.position_invalid_s);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((POSITION_INVALID AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR POSITION_INVALID;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
c := is_auto AND inp.position_valid AND NOT inp.in_operating_area;
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((GEOFENCE AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR GEOFENCE;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;

(* SR-08 two-channel speed plausibility. *)
disc := ABS(inp.speed_a - inp.speed_b) > cfg.discrepancy_tol;
disc_t0 := SEL(disc AND NOT m_disc_on, m_disc_t0, t);
c := disc AND (t - disc_t0 > cfg.discrepancy_time_s);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((SPEED_DISCREPANCY AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR SPEED_DISCREPANCY;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;

(* SR-03 warning field -> speed limit (non-latching), not inside the dig and dock zones. *)
in_zone := inp.in_dig_zone OR inp.in_dock_zone;
warn_trig := NOT in_zone AND ((inp.front_warn AND (fs_dir >= 0)) OR (inp.rear_warn AND (fs_dir <= 0)));
warn_until := SEL(warn_trig, m_warn_until, t + cfg.warning_hold_s);
warning := t < warn_until;

(* SR-04 speed limit and overspeed monitoring. *)
IF is_auto THEN
    spd_lim := FMIN(cfg.auto_speed, cfg.speed_cap);
ELSIF inp.mode = MODE_TELEOP THEN
    spd_lim := FMIN(cfg.teleop_speed, cfg.speed_cap);
ELSE
    spd_lim := 0.0;
END_IF;
IF warning THEN spd_lim := FMIN(spd_lim, cfg.warning_speed); END_IF;
creep := SEL(HAS(rec, FIELD_SIDE), cfg.recovery_speed, cfg.side_recovery_speed);
IF rec <> 0 THEN spd_lim := FMIN(spd_lim, creep); END_IF;
IF fs_id = DIG_SET THEN
    spd_lim := FMIN(spd_lim, cfg.dig_speed);
ELSIF fs_id = DOCK_SET THEN
    spd_lim := FMIN(spd_lim, cfg.dock_speed);
END_IF;
(* The monitored envelope follows a limit increase at once; after a decrease it holds
   the current speed for the reaction time, then ramps down (SS1-r style). *)
dec := spd_lim < m_prev_limit - 1.0E-9;
env_t0 := SEL(dec, m_env_t0, t);
env_v0 := SEL(dec, m_env_v0, FMIN(m_allowed, ABS(v)));
env_ramp := env_v0 - cfg.decel_ramp * FMAX(0.0, t - env_t0 - cfg.ramp_reaction_s);
allowed := FMAX(spd_lim, env_ramp);
over := ABS(v) > allowed + cfg.overspeed_tol;
over_t0 := SEL(over AND NOT m_over_on, m_over_t0, t);
c := over AND (t - over_t0 >= CYCLE_S);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((OVERSPEED AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR OVERSPEED;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;
(* Never travel faster than the selected protective field can stop from. *)
travel_set := (fs_dir <> 0) AND (fs_id <> DIG_SET) AND (fs_id <> DOCK_SET);
c := travel_set AND (ABS(v) > fs_vprot);
IF c THEN
    IF (PROTECTIVE_STOP > st) OR ((OVERSPEED AND NOT lat) <> 0) THEN ev_stop := TRUE; END_IF;
    lat := lat OR OVERSPEED;
    IF PROTECTIVE_STOP > st THEN st := PROTECTIVE_STOP; END_IF;
END_IF;

(* Reset handling (SR-09, SR-11, SR-12). *)
standstill := ABS(v) < cfg.standstill;
active := 0;
IF inp.machine_estop THEN active := active OR ESTOP_MACHINE; END_IF;
IF inp.remote_estop THEN active := active OR ESTOP_REMOTE; END_IF;
IF inp.front_prot THEN active := active OR FIELD_FRONT; END_IF;
IF inp.rear_prot THEN active := active OR FIELD_REAR; END_IF;
IF inp.side_prot THEN active := active OR FIELD_SIDE; END_IF;
IF NOT inp.scanners_ok THEN active := active OR SCANNER_FAULT; END_IF;
IF (inp.mode <> MODE_IDLE) AND (inp.remote_hb_age > cfg.remote_timeout_s) THEN active := active OR REMOTE_LINK; END_IF;
IF (inp.mode <> MODE_IDLE) AND (inp.autonomy_cmd_age > cfg.watchdog_timeout_s) THEN active := active OR WATCHDOG; END_IF;
IF is_auto AND inp.position_valid AND NOT inp.in_operating_area THEN active := active OR GEOFENCE; END_IF;
IF ABS(inp.speed_a - inp.speed_b) > cfg.discrepancy_tol THEN active := active OR SPEED_DISCREPANCY; END_IF;
IF is_auto AND NOT inp.position_valid THEN active := active OR POSITION_INVALID; END_IF;
IF inp.truck_door_open AND inp.in_dock_zone THEN active := active OR TRUCK_DOOR; END_IF;
IF under THEN active := active OR UNDER_BUCKET; END_IF;
(* Muting past its limit blocks reset while the dig request continues. *)
IF mute_timeout THEN active := active OR MUTING_TIMEOUT; END_IF;
(* A cause latched in this very cycle cannot be reset in the same cycle. *)
stopped_now := (st >= PROTECTIVE_STOP) AND NOT ((lat AND NOT m_latched) <> 0);
remaining := lat AND active;
enter_rec := stopped_now AND rising_reset AND (inp.mode = MODE_TELEOP) AND (st = PROTECTIVE_STOP)
             AND (remaining <> 0) AND ((remaining AND NOT FIELD_CAUSES) = 0)
             AND NOT (HAS(remaining, FIELD_SIDE) AND inp.side_contact AND NOT exempt);
(* Fields already in recovery stay in it. *)
IF enter_rec THEN rec := rec OR remaining; END_IF;
out.ev_recovery := enter_rec;
IF enter_rec THEN remaining := 0; END_IF;
clear_t0 := SEL(stopped_now AND (remaining = 0) AND NOT m_clear_on, m_clear_t0, t);
clear_on := SEL(stopped_now, m_clear_on, remaining = 0);
IF cfg.auto_restart THEN
    auto_ok := (st = PROTECTIVE_STOP) AND ((lat AND NOT CLEARING_CAUSES) = 0) AND clear_on
               AND (t - clear_t0 >= cfg.auto_restart_clear_s) AND NOT warning;
ELSE
    auto_ok := FALSE;
END_IF;
do_reset := stopped_now AND standstill AND (remaining = 0) AND (rising_reset OR auto_ok);
out.ev_reset := do_reset;
IF do_reset THEN
    st := RUN_ST;
    lat := 0;
    clear_on := FALSE;
END_IF;
IF st < PROTECTIVE_STOP THEN
    st := SEL(warning, RUN_ST, WARNING_ST);
END_IF;

stopped := st >= PROTECTIVE_STOP;
m_state := st;
m_latched := lat;
m_field_set := fs_id;
m_recovery := rec;
m_prev_reset := inp.reset_req;
m_over_on := over;
m_over_t0 := over_t0;
m_disc_on := disc;
m_disc_t0 := disc_t0;
m_posinv_on := posinv_on;
m_posinv_t0 := posinv_t0;
m_mute_on := muting;
m_mute_t0 := mute_t0;
m_clear_on := clear_on;
m_clear_t0 := clear_t0;
m_warn_until := warn_until;
m_allowed := allowed;
m_prev_limit := spd_lim;
m_env_t0 := env_t0;
m_env_v0 := env_v0;

(* The recovery creep limit applies from the cycle recovery starts. *)
creep := SEL(HAS(rec, FIELD_SIDE), cfg.recovery_speed, cfg.side_recovery_speed);
IF rec <> 0 THEN spd_lim := FMIN(spd_lim, creep); END_IF;
out.state := st;
out.cause := lat;
out.speed_limit := SEL(stopped, spd_lim, 0.0);
out.field_set := fs_id;
out.brake_release := NOT stopped;
(* SR-12: no boom/bucket motion while recovering from an occupied field. *)
out.implement_enable := NOT stopped AND (rec = 0);
out.reset_pending := stopped AND ((lat AND active) = 0);
out.dock_blocked := inp.truck_door_open;
out.fs_direction := fs_dir;
out.fs_v_protected := fs_vprot;
out.recovery := rec;
out.ev_stop := ev_stop;
END_FUNCTION_BLOCK
