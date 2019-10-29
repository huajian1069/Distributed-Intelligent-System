;-----------------------------------------------------------------------;
; fabs.s: Floating-point absolute value function.
;
; This file is part of the compact math library for the dsPIC30.
; (c) Microchip Technology. 2003.
;
;-----------------------------------------------------------------------;
        .include "libm.inc"
	.section .libm,code
;-----------------------------------------------------------------------;

;-----------------------------------------------------------------------;
;
; fabs
;
;       Double-precision absolute value function.
;
; Input:
;
;       (w3:w0) Floating-point number x
;
; Output:
;
;       (w3:w0) Floating-point number |x|
;
; Description:
;
;       Computes the absolute value of the argument x.
;       
;-----------------------------------------------------------------------;

        .global _fabs
        .global _fabsl

_fabs:
_fabsl:
        bclr    w3,#15          ; |x|
        return                  ; Done
;-----------------------------------------------------------------------;
        .end
