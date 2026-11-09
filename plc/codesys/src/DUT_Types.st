(* KAUHA safety PLC: data types.
   IEC 61131-3 Structured Text, 2nd edition subset (no methods, no references),
   so the same source compiles in CODESYS and in the matiec compiler the tests use. *)

TYPE ST_SafetyConfig :
STRUCT
    (* SafetyParams *)
    remote_timeout_s     : LREAL;
    watchdog_timeout_s   : LREAL;
    overspeed_tol        : LREAL;
    decel_ramp           : LREAL;
    ramp_reaction_s      : LREAL;
    discrepancy_tol      : LREAL;
    discrepancy_time_s   : LREAL;
    position_invalid_s   : LREAL;
    max_mute_s           : LREAL;
    auto_restart_clear_s : LREAL;
    warning_hold_s       : LREAL;
    auto_speed           : LREAL;
    teleop_speed         : LREAL;
    warning_speed        : LREAL;
    standstill           : LREAL;
    recovery_speed       : LREAL;
    side_recovery_speed  : LREAL;
    (* FieldTable: facts about the field sets loaded into the scanners *)
    t_standstill         : LREAL;
    dig_speed            : LREAL;
    dock_speed           : LREAL;
    speed_cap            : LREAL;
    auto_restart         : BOOL;     (* SR-11 automatic restart after a clearing stop *)
    band0                : LREAL;    (* speed band tops, m/s *)
    band1                : LREAL;
    band2                : LREAL;
    band3                : LREAL;
    sector0              : LREAL;    (* articulation sector boundaries, rad *)
    sector1              : LREAL;
    sector2              : LREAL;
    sector3              : LREAL;
    vprot0               : LREAL;    (* highest speed each band's protective field stops from *)
    vprot1               : LREAL;
    vprot2               : LREAL;
    vprot3               : LREAL;
    n_sectors            : INT;
    n_bands              : INT;
END_STRUCT;
END_TYPE

TYPE ST_SafetyInputs :
STRUCT
    t                : LREAL;   (* monotonic time, s *)
    mode             : INT;     (* 0 IDLE, 1 AUTO, 2 TELEOP *)
    speed_a          : LREAL;   (* drive-by-wire gateway, non-safety source, m/s *)
    speed_b          : LREAL;   (* PLC's own speed sensor, m/s *)
    articulation     : LREAL;   (* PLC's own articulation sensor, rad *)
    front_prot       : BOOL;
    front_warn       : BOOL;
    rear_prot        : BOOL;
    rear_warn        : BOOL;
    side_prot        : BOOL;
    side_contact     : BOOL;
    scanners_ok      : BOOL;
    remote_hb_age    : LREAL;
    remote_estop     : BOOL;
    reset_req        : BOOL;
    machine_estop    : BOOL;
    autonomy_cmd_age : LREAL;
    dig_req          : BOOL;
    dock_req         : BOOL;
    in_operating_area : BOOL;
    in_dig_zone      : BOOL;
    in_dock_zone     : BOOL;
    truck_door_open  : BOOL;
    boom_raised      : BOOL;
    front_side       : BOOL;
    under_near       : BOOL;
    under_far        : BOOL;
    under_mid        : BOOL;
    position_valid   : BOOL;
END_STRUCT;
END_TYPE

TYPE ST_SafetyOutputs :
STRUCT
    state            : INT;     (* 0 RUN, 1 WARNING, 2 PROTECTIVE_STOP, 3 EMERGENCY_STOP *)
    cause            : WORD;    (* latched causes, bit mask as in kauha/safety/core.py *)
    speed_limit      : LREAL;
    field_set        : INT;
    brake_release    : BOOL;    (* FALSE: drive power cut, motors short-braked (hardwired) *)
    implement_enable : BOOL;    (* FALSE: actuator power cut (hardwired) *)
    reset_pending    : BOOL;
    dock_blocked     : BOOL;
    fs_direction     : INT;
    fs_v_protected   : LREAL;
    recovery         : WORD;    (* fields in SR-12 recovery *)
    ev_stop          : BOOL;    (* a stop event was logged this cycle *)
    ev_recovery      : BOOL;
    ev_reset         : BOOL;
END_STRUCT;
END_TYPE

(* Eight payload bytes. A struct, not a bare array type, so it can be passed to
   functions by value in every IEC compiler (matiec mis-handles bare array types). *)
TYPE ST_Payload :
STRUCT
    b : ARRAY [0..7] OF BYTE;
END_STRUCT;
END_TYPE

TYPE ST_Frame :
STRUCT
    id   : DWORD;                    (* 29-bit J1939 identifier *)
    data : ST_Payload;
END_STRUCT;
END_TYPE
