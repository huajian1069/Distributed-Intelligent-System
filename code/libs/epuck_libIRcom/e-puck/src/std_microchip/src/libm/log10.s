;-----------------------------------------------------------------------;
; log10.s: Floating-point common logarithm elementary function.
;
; This file is part of the compact math library for the dsPIC30.
; (c) Microchip Technology. 2003.
;-----------------------------------------------------------------------;
        .include "libm.inc"
	.section .libm,code
;-----------------------------------------------------------------------;
;
; log10
;
;       Common logarithm elementary function.
;
; Input:
;
;       (w3:w0) Floating-point number x
;
; Output:
;
;       (w3:w0) Floating-point number log10(x)
;
; Description:
;
;       Computes the common logarithm of the argument x.
;       If the argiment is not in the function domain [0,+infinity],
;       a quiet NaN is returned, and errno is set to EDOMAIN.
;       
; Notes:
;
;       The logarithm is computed using the identity:
;
;       ln(x) = log(x)/log(e)
;       log(x) = ln(x) * log(e)
;
;-----------------------------------------------------------------------;

        .global _log10
        .global _log10l

_log10:
_log10l:
        rcall   _log            ; y = ln(x)
        mov     #DP_LOGe3,w7    ; z = log(e)
        mov     #DP_LOGe2,w6    ; *
        mov     #DP_LOGe1,w5    ; *
        mov     #DP_LOGe0,w4    ; *
        bra     ___muldf3       ; return ln(x) * log(e)

;-----------------------------------------------------------------------;
        .end


