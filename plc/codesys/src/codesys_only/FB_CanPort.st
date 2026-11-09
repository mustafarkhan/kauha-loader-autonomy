(* CAN port for CODESYS Control (Raspberry Pi SL with SocketCAN, or any controller
   with a CAN interface), using the "CAA CAN Low Level" library (namespace CL2).

   Not compiled by the matiec tests: this is the only code that calls a vendor
   library. It moves raw frames between the driver and FB_SafetyNode; every
   check on their content happens in the tested code.

   Library signatures follow the CODESYS online help for CAA CAN Low Level
   3.5.16/3.5.17 (DriverOpenH, CreateMaskReceiver, Read, GetMessageId,
   GetMessageDataPointer, FreeMessage, CreateMessage, Write). If the compiler
   reports a parameter mismatch on another library version, adjust the call here;
   nothing else depends on it. *)

FUNCTION_BLOCK FB_CanPort
VAR_INPUT
    usiNetId   : USINT := 0;       (* CAN interface number: can0 = 0 *)
    uiBaudrate : UINT := 500;      (* kbit/s, as configured on the bus *)
    xReceive   : BOOL;             (* collect the frames waiting in the driver this call *)
    xSend      : BOOL;             (* send tx_id/tx this call *)
    tx_id      : DWORD;
    tx         : ST_Payload;
END_VAR
VAR_OUTPUT
    rx         : ST_RxBatch;       (* frames received since the last call, oldest first *)
    xOpen      : BOOL;
    udiRx      : UDINT;
    udiTx      : UDINT;
    udiErrors  : UDINT;
    udiDropped : UDINT;            (* more than 32 frames waiting in one cycle *)
END_VAR
VAR
    hDriver   : CAA.HANDLE := CAA.gc_hINVALID;
    hReceiver : CAA.HANDLE := CAA.gc_hINVALID;
    hMsg      : CAA.HANDLE;
    eError    : CL2.ERROR;
    ctLeft    : CAA.COUNT;
    pByte     : POINTER TO BYTE;
    i         : INT;
END_VAR

IF hDriver = CAA.gc_hINVALID THEN
    hDriver := CL2.DriverOpenH(usiNetId := usiNetId, uiBaudrate := uiBaudrate, xSupport29Bits := TRUE,
                               ctMessages := 64, peError := ADR(eError));
    IF hDriver <> CAA.gc_hINVALID THEN
        (* every extended-ID data frame that this node did not send itself *)
        hReceiver := CL2.CreateMaskReceiver(hDriver := hDriver,
                                            cobIdValue := 0, cobIdMask := 0,
                                            xRTRValue := FALSE, xRTRMask := TRUE,
                                            x29BitIdValue := TRUE, x29BitIdMask := TRUE,
                                            xTransmitValue := FALSE, xTransmitMask := TRUE,
                                            xAlwaysNewest := FALSE, eEvent := CB.EVENT.NO_EVENT,
                                            xEnableSyncWindow := FALSE, peError := ADR(eError));
    END_IF
END_IF
xOpen := (hDriver <> CAA.gc_hINVALID) AND (hReceiver <> CAA.gc_hINVALID);

IF xReceive THEN
    rx.n := 0;
END_IF
WHILE xOpen AND xReceive DO
    hMsg := CL2.Read(hReceiverId := hReceiver, pctMsgLeft := ADR(ctLeft), peError := ADR(eError));
    IF hMsg = CAA.gc_hINVALID THEN
        EXIT;
    END_IF
    IF rx.n < 32 THEN
        rx.f[rx.n].id := CL2.GetMessageId(hMessage := hMsg, peError := ADR(eError));
        pByte := CL2.GetMessageDataPointer(hMessage := hMsg, peError := ADR(eError));
        FOR i := 0 TO 7 DO
            rx.f[rx.n].data.b[i] := (pByte + i)^;
        END_FOR
        rx.n := rx.n + 1;
        udiRx := udiRx + 1;
    ELSE
        udiDropped := udiDropped + 1;   (* dropped frames read as missing: the E2E checks see them as lost *)
    END_IF
    CL2.FreeMessage(hMessage := hMsg, peError := ADR(eError));
END_WHILE

IF xSend AND xOpen THEN
    hMsg := CL2.CreateMessage(hDriver := hDriver, cobID := tx_id, usiLength := 8, xRTR := FALSE,
                              x29BitID := TRUE, peError := ADR(eError));
    IF hMsg <> CAA.gc_hINVALID THEN
        pByte := CL2.GetMessageDataPointer(hMessage := hMsg, peError := ADR(eError));
        FOR i := 0 TO 7 DO
            (pByte + i)^ := tx.b[i];
        END_FOR
        eError := CL2.Write(hDriver := hDriver, hMessage := hMsg, usiPriority := 0, xEnableSyncWindow := FALSE);
        IF eError = CL2.ERROR.NO_ERROR THEN
            udiTx := udiTx + 1;
        ELSE
            udiErrors := udiErrors + 1;
        END_IF
    ELSE
        udiErrors := udiErrors + 1;
    END_IF
END_IF
END_FUNCTION_BLOCK
