;-----------------------------------------------------------------------;
; umodsi3.s: Integer division elementary operation.
;
; This file is part of the compact math library for the dsPIC30.
; (c) Microchip Technology. 2003.
;
;-----------------------------------------------------------------------;
        .include "libm.inc"
	.section .libm,code
;-----------------------------------------------------------------------;
;
; __umodsi3
;
;       Unsigned 32-bit integer remainder.
;
; Input:
;
;       (w1:w0) Dividend (Q)
;       (w3:w2) Divisor (D)
;
; Output:
;
;       (w1:w0) Remainder
;
;-----------------------------------------------------------------------;
        .global ___umodsi3

___umodsi3:
        rcall   ___udivsi3      ; Calculate quotient & remainder
        mov.d   w4,w0           ; Return the remainder
        return                  ; Done
;-----------------------------------------------------------------------;
        .end

