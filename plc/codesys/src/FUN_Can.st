(* KAUHA machine bus in the safety PLC: CRC-8 (SAE J1850), end-to-end receiver,
   signal packing. Port of kauha/can/e2e.py and the frame layouts in
   hardware/can/kauha_machine.dbc (little-endian, Intel byte order).
   tests/plc/test_st_equivalence.py checks these against the Python codec. *)

FUNCTION CRC8_J1850 : BYTE
VAR_INPUT
    data_id : BYTE;
    data    : ST_Payload;   (* CRC over data_id then bytes 0..6 *)
END_VAR
VAR
    crc : BYTE;
    i, k : INT;
    nb : BYTE;
END_VAR
crc := 16#FF;
FOR i := -1 TO 6 DO
    IF i < 0 THEN nb := data_id; ELSE nb := data.b[i]; END_IF;
    crc := crc XOR nb;
    FOR k := 1 TO 8 DO
        IF (crc AND 16#80) <> 0 THEN
            crc := SHL(crc, 1) XOR 16#1D;
        ELSE
            crc := SHL(crc, 1);
        END_IF;
    END_FOR;
END_FOR;
CRC8_J1850 := crc XOR 16#FF;
END_FUNCTION

FUNCTION E2E_PROTECT : BOOL   (* writes counter and CRC into data; always TRUE *)
VAR_INPUT
    data_id : BYTE;
    counter : INT;
END_VAR
VAR_IN_OUT
    data : ST_Payload;
END_VAR
data.b[6] := (data.b[6] AND 16#F0) OR (INT_TO_BYTE(counter) AND 16#0F);
data.b[7] := CRC8_J1850(data_id, data);
E2E_PROTECT := TRUE;
END_FUNCTION

(* E2E receiver. status: 0 OK, 1 OK_SOME_LOST, 2 REPEATED, 3 WRONG_SEQUENCE, 4 CRC_ERROR.
   Call with new_frame := TRUE once per received frame; fresh is evaluated every call. *)
FUNCTION_BLOCK FB_E2ERx
VAR_INPUT
    data_id   : BYTE;
    timeout_s : LREAL;
    max_delta : INT := 2;
    new_frame : BOOL;
    data      : ST_Payload;
    t         : LREAL;
END_VAR
VAR_OUTPUT
    status : INT;
    accepted : BOOL;    (* this frame passed: OK or OK_SOME_LOST *)
    fresh  : BOOL;
    age    : LREAL;     (* seconds since the last accepted frame (1E9 before the first) *)
END_VAR
VAR
    last_counter : INT := -1;
    last_ok_time : LREAL := -1.0E9;
    have_ok : BOOL := FALSE;
    ctr, delta : INT;
END_VAR
accepted := FALSE;
IF new_frame THEN
    IF CRC8_J1850(data_id, data) <> data.b[7] THEN
        status := 4;
    ELSE
        ctr := BYTE_TO_INT(data.b[6] AND 16#0F);
        IF last_counter < 0 THEN
            status := 0;
            last_counter := ctr;
        ELSE
            delta := (ctr - last_counter + 16) MOD 16;
            IF delta = 0 THEN
                status := 2;
            ELSE
                IF delta > max_delta THEN status := 3;
                ELSIF delta = 1 THEN status := 0;
                ELSE status := 1;
                END_IF;
                last_counter := ctr;
            END_IF;
        END_IF;
        IF status <= 1 THEN
            accepted := TRUE;
            last_ok_time := t;
            have_ok := TRUE;
        END_IF;
    END_IF;
END_IF;
fresh := have_ok AND (t - last_ok_time <= timeout_s);
IF have_ok THEN age := t - last_ok_time; ELSE age := 1.0E9; END_IF;
END_FUNCTION_BLOCK

(* Signal access, Intel byte order. start_byte is the signal's start bit / 8. *)
FUNCTION GET_S16 : LREAL   (* signed 16 bit times factor *)
VAR_INPUT
    data : ST_Payload;
    start_byte : INT;
    factor : LREAL;
END_VAR
VAR
    raw : DINT;
END_VAR
raw := BYTE_TO_DINT(data.b[start_byte]) + 256 * BYTE_TO_DINT(data.b[start_byte + 1]);
IF raw >= 32768 THEN raw := raw - 65536; END_IF;
GET_S16 := DINT_TO_LREAL(raw) * factor;
END_FUNCTION

FUNCTION GET_U16 : DINT
VAR_INPUT
    data : ST_Payload;
    start_byte : INT;
END_VAR
GET_U16 := BYTE_TO_DINT(data.b[start_byte]) + 256 * BYTE_TO_DINT(data.b[start_byte + 1]);
END_FUNCTION

FUNCTION GET_BIT : BOOL
VAR_INPUT
    data : ST_Payload;
    bit : INT;            (* DBC start bit *)
END_VAR
GET_BIT := (SHR(data.b[bit / 8], bit MOD 8) AND 16#01) <> 0;
END_FUNCTION

FUNCTION GET_U4 : INT
VAR_INPUT
    data : ST_Payload;
    bit : INT;            (* DBC start bit, nibble aligned *)
END_VAR
GET_U4 := BYTE_TO_INT(SHR(data.b[bit / 8], bit MOD 8) AND 16#0F);
END_FUNCTION

FUNCTION PUT_U16 : BOOL
VAR_INPUT
    value : DINT;         (* 0..65535 *)
    start_byte : INT;
END_VAR
VAR_IN_OUT
    data : ST_Payload;
END_VAR
data.b[start_byte] := DINT_TO_BYTE(value MOD 256);
data.b[start_byte + 1] := DINT_TO_BYTE(value / 256);
PUT_U16 := TRUE;
END_FUNCTION

(* SafetyStatus frame (PLC -> gateway, autonomy, remote). Payload as the DBC:
   State 0|4, ResetPending 4|1, DockBlocked 5|1, ActiveFieldSet 8|8, Cause 16|16,
   SpeedLimit 32|16 (0.001 m/s), AliveCounter 48|4, Crc 56|8. *)
FUNCTION ENCODE_SAFETY_STATUS : BOOL
VAR_INPUT
    o : ST_SafetyOutputs;
    counter : INT;
    data_id : BYTE;
END_VAR
VAR_IN_OUT
    data : ST_Payload;
END_VAR
VAR
    i : INT;
    lim : DINT;
    ok : BOOL;
END_VAR
FOR i := 0 TO 7 DO data.b[i] := 0; END_FOR;
data.b[0] := INT_TO_BYTE(o.state) AND 16#0F;
IF o.reset_pending THEN data.b[0] := data.b[0] OR 16#10; END_IF;
IF o.dock_blocked THEN data.b[0] := data.b[0] OR 16#20; END_IF;
data.b[1] := INT_TO_BYTE(o.field_set);
ok := PUT_U16(WORD_TO_DINT(o.cause), 2, data);
lim := LREAL_TO_DINT(o.speed_limit * 1000.0);
IF lim < 0 THEN lim := 0; END_IF;
IF lim > 6000 THEN lim := 6000; END_IF;
ok := PUT_U16(lim, 4, data);
ok := E2E_PROTECT(data_id, counter, data);
ENCODE_SAFETY_STATUS := TRUE;
END_FUNCTION
