(* Small helpers shared by the safety POUs. FMIN/FMAX reproduce the reference
   model's Min/Max exactly (first argument on ties), so results are bit-identical. *)

FUNCTION FMIN : LREAL
VAR_INPUT
    a : LREAL;
    b : LREAL;
END_VAR
FMIN := SEL(a <= b, b, a);
END_FUNCTION

FUNCTION FMAX : LREAL
VAR_INPUT
    a : LREAL;
    b : LREAL;
END_VAR
FMAX := SEL(a >= b, b, a);
END_FUNCTION

FUNCTION HAS : BOOL
VAR_INPUT
    mask : WORD;
    bits : WORD;
END_VAR
HAS := (mask AND bits) <> 0;
END_FUNCTION
