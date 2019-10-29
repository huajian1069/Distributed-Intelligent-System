;-----------------------------------------------------------------------;
; acos.s: Floating-point arccosine elementary function.
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
; acos
;
;       Double-precision arccosine elementary function.
;
; Input:
;
;       (w3:w0) Floating-point number x
;
; Output:
;
;       (w3:w0) Floating-point number arccosine(x)
;
; Description:
;
;       Computes the arccosine of the argument x.
;       
;-----------------------------------------------------------------------;

        .global _acos
        .global _acosl

_acos:
_acosl:
        mov.w   #3,w4           ; Indicate arccosine required
        bra     __asinacos      ; Join common processing ...
;-----------------------------------------------------------------------;
        .end
