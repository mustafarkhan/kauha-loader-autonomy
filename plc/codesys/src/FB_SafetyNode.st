(* KAUHA safety PLC, testbed node: everything between the CAN driver and the
   safety core, in plain IEC 61131-3 so it is tested with the core.

   Each 25 ms cycle PRG_Main (CODESYS-specific CAN driver calls) hands this block
   the frames received since the last cycle and the GPIO inputs. It checks every
   frame end to end (CRC, alive counter, freshness), builds the core's inputs,
   runs FB_SafetyCore, and returns the SafetyStatus frame and the hardwired outputs.

   Fail-safe defaults: a frame that is missing, stale or corrupt never makes the
   machine run; see the comment at each input. *)

TYPE ST_RxBatch :
STRUCT
    n : INT;                               (* frames valid in f[0..n-1] *)
    f : ARRAY [0..31] OF ST_Frame;
END_STRUCT;
END_TYPE

TYPE ST_ScannerIO :
STRUCT
    (* Rated scanner option (SICK nanoScan3 I/O or similar), wired to PLC inputs.
       OSSD-type signals are TRUE when the field is CLEAR. *)
    front_ossd_clear : BOOL;
    rear_ossd_clear  : BOOL;
    front_warn_clear : BOOL;
    rear_warn_clear  : BOOL;
    side_clear       : BOOL;
    under_clear      : BOOL;    (* under_near and under_mid share one output on the testbed *)
    scanners_ready   : BOOL;    (* scanner "device ready / no fault" outputs, both scanners *)
END_STRUCT;
END_TYPE

FUNCTION_BLOCK FB_SafetyNode
VAR_INPUT
    cfg            : ST_SafetyConfig;
    t              : LREAL;          (* s, monotonic *)
    rx             : ST_RxBatch;
    estop_chain_ok : BOOL;           (* machine e-stop chain closed: safety relay K1 output state *)
    use_scanner_io : BOOL;           (* TRUE: rated scanner outputs on GPIO; FALSE: ScannerStatus frames *)
    scanner_io     : ST_ScannerIO;
END_VAR
VAR_OUTPUT
    out              : ST_SafetyOutputs;
    drive_enable     : BOOL;         (* relay K-DRIVE: motor power and dynamic-brake changeover *)
    implement_enable : BOOL;         (* relay K-IMPL: actuator power *)
    field_set_code   : BYTE;         (* to the rated scanners' static control inputs *)
    tx_id            : DWORD;
    tx               : ST_Payload;   (* SafetyStatus, protected *)
    inputs           : ST_SafetyInputs;   (* what the core saw this cycle, for diagnostics *)
    sensor_fault     : BOOL;
    scanner_fault    : BOOL;
END_VAR
VAR
    core : FB_SafetyCore;
    rxDrive, rxRemote, rxMs1, rxSensors, rxScanner, rxZone : FB_E2ERx;
    k : INT;
    id : DWORD;
    pgn : DWORD;
    sa : DWORD;
    d : ST_Payload;
    (* last accepted values; start in the safe state *)
    mode_req : INT := 0;
    remote_estop : BOOL := TRUE;
    reset_req : BOOL := FALSE;
    dig_req, dock_req : BOOL := FALSE;
    speed_a : LREAL := 0.0;
    speed_b : LREAL := 0.0;
    articulation : LREAL := 0.0;
    boom_raised : BOOL := TRUE;
    sensor_ok : BOOL := FALSE;
    sc_bits : ST_Payload;
    sc_set : INT := -1;
    zone : ST_Payload;
    hist : ARRAY [0..3] OF INT := [-2, -2, -2, -2];
    set_ok : BOOL;
    counter : INT := 0;
    ok : BOOL;
END_VAR

FOR k := 0 TO rx.n - 1 DO
    id := rx.f[k].id;
    d := rx.f[k].data;
    pgn := SHR(id, 8) AND 16#3FFFF;
    sa := id AND 16#FF;
    IF (pgn = 16#FF40) AND (sa = 16#82) THEN                 (* RemoteHeartbeat *)
        rxRemote(data_id := 16#40, timeout_s := cfg.remote_timeout_s, new_frame := TRUE, data := d, t := t);
        IF rxRemote.accepted THEN
            remote_estop := GET_BIT(d, 0);
            reset_req := GET_BIT(d, 1);
            mode_req := GET_U4(d, 4);
        END_IF;
    ELSIF (pgn = 16#FF10) AND (sa = 16#80) THEN              (* DriveCmd *)
        rxDrive(data_id := 16#10, timeout_s := cfg.watchdog_timeout_s, new_frame := TRUE, data := d, t := t);
        IF rxDrive.accepted THEN
            dig_req := GET_BIT(d, 37);
            dock_req := GET_BIT(d, 38);
        END_IF;
    ELSIF (pgn = 16#FF30) AND (sa = 16#83) THEN              (* MachineState1 *)
        rxMs1(data_id := 16#30, timeout_s := 0.1, new_frame := TRUE, data := d, t := t);
        IF rxMs1.accepted THEN speed_a := GET_S16(d, 0, 0.001); END_IF;
    ELSIF (pgn = 16#FF50) AND (sa = 16#84) THEN              (* SafeSensors *)
        rxSensors(data_id := 16#50, timeout_s := 0.1, new_frame := TRUE, data := d, t := t);
        IF rxSensors.accepted THEN
            speed_b := GET_S16(d, 0, 0.001);
            articulation := GET_S16(d, 2, 0.0001);
            boom_raised := GET_BIT(d, 52);
            sensor_ok := GET_BIT(d, 53);
        END_IF;
    ELSIF (pgn = 16#FF51) AND (sa = 16#85) THEN              (* ScannerStatus *)
        rxScanner(data_id := 16#51, timeout_s := 0.1, new_frame := TRUE, data := d, t := t);
        IF rxScanner.accepted THEN
            sc_bits := d;
            sc_set := BYTE_TO_INT(d.b[2]);
        END_IF;
    ELSIF (pgn = 16#FF52) AND (sa = 16#86) THEN              (* ZoneStatus *)
        rxZone(data_id := 16#52, timeout_s := 0.5, new_frame := TRUE, data := d, t := t);
        IF rxZone.accepted THEN zone := d; END_IF;
    END_IF;
END_FOR;
(* freshness and age at this cycle's time *)
rxRemote(new_frame := FALSE, t := t);
rxDrive(new_frame := FALSE, t := t);
rxMs1(new_frame := FALSE, t := t);
rxSensors(new_frame := FALSE, t := t);
rxScanner(new_frame := FALSE, t := t);
rxZone(new_frame := FALSE, t := t);

inputs.t := t;
inputs.mode := mode_req;
(* SR-05/SR-06 run on the ages; before the first frame the age is 1E9 s. *)
inputs.remote_hb_age := rxRemote.age;
inputs.autonomy_cmd_age := rxDrive.age;
(* Stale remote: keep the e-stop state last received; the link timeout stops the machine. *)
inputs.remote_estop := remote_estop;
inputs.reset_req := reset_req AND rxRemote.fresh;
inputs.dig_req := dig_req AND rxDrive.fresh;
inputs.dock_req := dock_req AND rxDrive.fresh;
inputs.machine_estop := NOT estop_chain_ok;
(* SR-08: a silent gateway reads as a large speed, so the discrepancy check stops the machine. *)
IF rxMs1.fresh THEN inputs.speed_a := speed_a; ELSE inputs.speed_a := 1000.0; END_IF;
(* The PLC's own sensors: if stale or reporting a fault, the core sees a sensor/scanner
   fault (cause SCANNER_FAULT) and the last speed is held. *)
sensor_fault := NOT (rxSensors.fresh AND sensor_ok);
inputs.speed_b := speed_b;
inputs.articulation := articulation;
inputs.boom_raised := boom_raised OR sensor_fault;
(* Field inputs *)
IF use_scanner_io THEN
    inputs.front_prot := NOT scanner_io.front_ossd_clear;
    inputs.rear_prot := NOT scanner_io.rear_ossd_clear;
    inputs.front_warn := NOT scanner_io.front_warn_clear;
    inputs.rear_warn := NOT scanner_io.rear_warn_clear;
    inputs.side_prot := NOT scanner_io.side_clear;
    inputs.side_contact := NOT scanner_io.side_clear;
    inputs.front_side := NOT scanner_io.side_clear;
    inputs.under_near := NOT scanner_io.under_clear;
    inputs.under_mid := NOT scanner_io.under_clear;
    inputs.under_far := FALSE;
    scanner_fault := NOT scanner_io.scanners_ready;
ELSE
    (* The evaluator must be evaluating a set the PLC selected in the last four cycles. *)
    set_ok := (sc_set = hist[0]) OR (sc_set = hist[1]) OR (sc_set = hist[2]) OR (sc_set = hist[3]);
    scanner_fault := NOT (rxScanner.fresh AND GET_BIT(sc_bits, 10) AND set_ok);
    IF scanner_fault THEN
        (* every field reads occupied *)
        inputs.front_prot := TRUE; inputs.rear_prot := TRUE; inputs.front_warn := TRUE; inputs.rear_warn := TRUE;
        inputs.side_prot := TRUE; inputs.side_contact := TRUE; inputs.front_side := TRUE;
        inputs.under_near := TRUE; inputs.under_mid := TRUE; inputs.under_far := TRUE;
    ELSE
        inputs.front_prot := GET_BIT(sc_bits, 0);
        inputs.front_warn := GET_BIT(sc_bits, 1);
        inputs.rear_prot := GET_BIT(sc_bits, 2);
        inputs.rear_warn := GET_BIT(sc_bits, 3);
        inputs.side_prot := GET_BIT(sc_bits, 4);
        inputs.side_contact := GET_BIT(sc_bits, 5);
        inputs.front_side := GET_BIT(sc_bits, 6);
        inputs.under_near := GET_BIT(sc_bits, 7);
        inputs.under_mid := GET_BIT(sc_bits, 8);
        inputs.under_far := GET_BIT(sc_bits, 9);
    END_IF;
END_IF;
inputs.scanners_ok := NOT scanner_fault AND NOT sensor_fault;
(* Zones: stale means position invalid and outside every zone. *)
IF rxZone.fresh THEN
    inputs.in_operating_area := GET_BIT(zone, 0);
    inputs.in_dig_zone := GET_BIT(zone, 1);
    inputs.in_dock_zone := GET_BIT(zone, 2);
    inputs.position_valid := GET_BIT(zone, 3);
    inputs.truck_door_open := GET_BIT(zone, 4);
ELSE
    inputs.in_operating_area := FALSE;
    inputs.in_dig_zone := FALSE;
    inputs.in_dock_zone := FALSE;
    inputs.position_valid := FALSE;
    inputs.truck_door_open := TRUE;
END_IF;

core(cfg := cfg, inp := inputs);
out := core.out;

hist[3] := hist[2];
hist[2] := hist[1];
hist[1] := hist[0];
hist[0] := out.field_set;

drive_enable := out.brake_release;
implement_enable := out.implement_enable;
field_set_code := INT_TO_BYTE(out.field_set);
counter := (counter + 1) MOD 16;
tx_id := 16#08FF2081;
ok := ENCODE_SAFETY_STATUS(out, counter, 16#20, tx);
END_FUNCTION_BLOCK
