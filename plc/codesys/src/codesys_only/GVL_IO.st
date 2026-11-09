(* Hardware I/O of the 1:8 testbed safety PLC. Map each variable to its channel in
   the device's "I/O Mapping" tab (Raspberry Pi GPIOs device, or the I/O modules of
   your controller). Pin numbers are the testbed wiring in docs/scale/wiring.md.
   Inputs arrive through the 24 V opto-isolated input board (high = 24 V present). *)
VAR_GLOBAL
    (* inputs *)
    xEstopChainOk    : BOOL;   (* IN1  GPIO5:  safety relay K1 contact 33-34, TRUE = chain closed and K1 reset *)
    xRelayFeedbackOk : BOOL;   (* IN2  GPIO6:  K-DRIVE and K-IMPL NC mirror contacts in series, TRUE = both dropped *)
    stScannerIO      : ST_ScannerIO;
                               (* IN3  GPIO12: front_ossd_clear   IN4  GPIO13: rear_ossd_clear
                                  IN5  GPIO19: front_warn_clear   IN6  GPIO20: rear_warn_clear
                                  IN7  GPIO21: side_clear         IN8  GPIO26: under_clear
                                  scanners_ready: IN3..IN8 board "power good" (GPIO4) *)
    (* outputs *)
    xDriveEnable     : BOOL;   (* OUT1 GPIO23: coil of K-DRIVE (motor power and dynamic-brake changeover) *)
    xImplementEnable : BOOL;   (* OUT2 GPIO24: coil of K-IMPL (actuator power) *)
    xFs0, xFs1, xFs2, xFs3, xFs4, xFs5 : BOOL;
                               (* OUT3..OUT8 GPIO16, 17, 22, 27, 18, 14: field-set code bits 0..5 to the
                                  rated scanners' static control inputs (only with USE_SCANNER_IO) *)
    xRunLamp         : BOOL;   (* OUT9 GPIO15: green beacon, machine may move *)
END_VAR

VAR_GLOBAL CONSTANT
    (* FALSE: field bits from the ScannerStatus frame (software evaluator on a hobby lidar, not safety rated).
       TRUE: field bits from rated scanner outputs on stScannerIO. *)
    USE_SCANNER_IO : BOOL := FALSE;
END_VAR
