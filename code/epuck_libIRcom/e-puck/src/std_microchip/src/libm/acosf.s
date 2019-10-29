;-----------------------------------------------------------------------;
; acosf.s: Floating-point arccosine elementary function.
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
; acosf
;
;       Single-precision arccosine elementary function.
;
; Input:
;
;       (w1:w0) Floating-point number x
;
; Output:
;
;       (w1:w0) Floating-point number arccosine(x)
;
; Description:
;
;       Computes the arccosine of the argument x.
;       
;-----------------------------------------------------------------------;

        .global _acosf

_acosf:
        mov.w   #3,w2           ; Indicate arccosine required
        bra     __asinacosf     ; Join common processing ...
;-----------------------------------------------------------------------;
        .end
