;-----------------------------------------------------------------------;
; fabsf.s: Floating-point absolute value function.
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
; fabsf
;
;       Single-precision absolute value function.
;
; Input:
;
;       (w1:w0) Floating-point number x
;
; Output:
;
;       (w1:w0) Floating-point number |x|
;
; Description:
;
;       Computes the absolute value of the argument x.
;       
;-----------------------------------------------------------------------;

        .global _fabsf

_fabsf:
        bclr    w1,#15          ; |x|
        return                  ; Done
;-----------------------------------------------------------------------;
        .end
