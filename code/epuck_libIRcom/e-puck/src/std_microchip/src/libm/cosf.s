;-----------------------------------------------------------------------;
; cosf.s: Floating-point cosine elementary function.
;
; This file is part of the compact math library for the dsPIC30.
; (c) Microchip Technology. 2003.
;
;-----------------------------------------------------------------------;
        .include "libm.inc"
	.section .libm,code
;-----------------------------------------------------------------------;
;
; cosf
;
;       Single-precision cosine elementary function.
;
; Input:
;
;       (w1:w0) Floating-point number x
;
; Output:
;
;       (w1:w0) Floating-point number cosine(x)
;
; Description:
;
;       Computes the cosine of the argument x.
;       
;-----------------------------------------------------------------------;

        .global _cosf

_cosf:
        setm    w2              ; Indicate cosine required
        bra     __sincosf       ; Join common processing ...
;-----------------------------------------------------------------------;
        .end
