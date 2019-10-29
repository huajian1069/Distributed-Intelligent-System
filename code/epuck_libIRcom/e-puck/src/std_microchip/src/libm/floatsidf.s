;-----------------------------------------------------------------------;
; floatsidf.s: Convert 32-bit, signed integer to floating-point
;
; This file is part of the compact math library for the dsPIC30.
; (c) Microchip Technology. 2003.
;
;-----------------------------------------------------------------------;
        .include "libm.inc"
	.section .libm,code
;-----------------------------------------------------------------------;
; __floatsidf: Convert 32-bit, signed integer to floating-point
;
; Input:
;
;       (w1:w0) Integer to be converted
;
; Output:
;
;       (w3:w2:w1:w0) Floating-point representation of input value
;
; Description:
;
;       Convert a 32-bit, signed integer to floating-point.
;
;-----------------------------------------------------------------------;

        .global ___floatsidf

___floatsidf:
        asr     w1,#15,w2               ; Sign extend 32->64
        mov     w2,w3                   ; *
        bra     ___floatdidf            ; Use 64->64 conversion

;-----------------------------------------------------------------------;

        .end
