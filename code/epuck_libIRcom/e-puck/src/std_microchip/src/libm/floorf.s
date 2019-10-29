;-----------------------------------------------------------------------;
;
; floorf.s: Single-precision floating-point floor function.
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
; floorf: Single-precision floating-point floor function.
;
; Description:
;
;       Computes the largets integral value not greater than x.
;
; Input:
;
;       (w1:w0) x: argument to be floored
;
; Output:
;
;       (w1:w0) Floored argument
;
; Description:
;
;       Computes the largets integral value not greater than x.
;
;-----------------------------------------------------------------------;
        .global _floorf

_floorf:
        clr     w2              ; Chop below 2^0
        rcall   __fchop         ; Chop fractional bits
        bra     z,exit          ; Nothing chopped ...

;------ Bits were chopped

        mov     #ONELO,w2       ; (w3:w2) = 1.0f
        mov     #ONEHI,w3       ; *
        btsc    w1,#15          ; x < 0 ?
        rcall   ___subsf3       ; Yes ... result -= 1.0
exit:
        return                  ; Done

;-----------------------------------------------------------------------;

        .end


