;-----------------------------------------------------------------------;
; floatsisf.s: Convert 32-bit, signed integer to floating-point
;
; This file is part of the compact math library for the dsPIC30.
; (c) Microchip Technology. 2003.
;
;-----------------------------------------------------------------------;
        .include "libm.inc"
	.section .libm,code
;-----------------------------------------------------------------------;
; __floatsisf: Convert 32-bit, signed integer to floating-point
;
; Description:
;
;       Convert a 32-bit, signed integer to floating-point.
;
; Input:
;
;       (w1:w0) Integer to be converted
;
; Output:
;
;       (w1:w0) Floating-point representation of input value
;
;-----------------------------------------------------------------------;

        .global ___floatsisf

___floatsisf:
        mov.d   w8,[w15++]              ; Preserve scratch
        mov.d   w10,[w15++]             ; Preserve scratch
        mov     w1,w10                  ; Preserve sign

;------ Check for special arguments: 0x00000000 and 0x80000000

        cp0     w1                      ; arg < 0
        bra     lt,negative             ; Yes ...
        bra     gt,notspecial           ; No ...
        cp0     w0                      ; arg == 0
        bra     z,return0               ; Yes ...
        bra     notspecial              ; arg is > 0
negative:
        subr    w0,#0,w0                ; (w1:w0) = |arg|
        subbr   w1,#0,w1                ; *
        bra     nn,notspecial           ; arg is in range

;------ Maximum negative integer: -1.0 x 2^31

        mov     #LONG_MINHI,w1          ; Convert to floating-point
        bra     return0                 ; Done

;-----------------------------------------------------------------------;
;       Argument is not special
;-----------------------------------------------------------------------;

notspecial:

;------ Form significand, exponent, sticky & round

        mov.d   w0,w8                   ; (w9:w8) = significand
        mul.uu  w2,#0,w2                ; (w2) = round
                                        ; (w3) = sticky
        mov     #23+FLT_BIAS,w11        ; (w11)= exponent

;------ Form the significand by aligning the MSB at position 23
 
;       Using the find-first instructions, the bit numbering is
;       such that bit 31 is numbered 1 and bit 0 is numbered 32.
;       We want to align the MSB of the argument at position 23,
;       i.e., bit number 9. So, subtracting 9 from the bit number
;       gives the shift count for aligning the MSB.
;       If the shift count is negative, it means that the MSB
;       is to the left of position 23, so a right shift is needed.

        ff1l    w9,w4                   ; (w4) = shift count
        bra     nc,fixshift             ; Found the leading '1' ...
        ff1l    w8,w4                   ; Scan lsw
        add.b   #16,w4                  ; Account for lsw
fixshift:
        sub.b   w4,#9,w4                ; (w4) = shift count
        bra     z,round                 ; No shift required ...
        bra     nn,shiftleft            ; Left shift required ...

;------ Shift right

shiftright:
        ior     w3,w2,w3                ; (w3) = (sticky |= round)
        and     w8,#1,w2                ; (w2) = (round = sig & 1)
        lsr     w9,w9                   ; (w9:w8) = (sig >>= 1)
        rrc     w8,w8                   ; *
        inc     w11,w11                 ; (w11) = exp++
        inc.b   w4,w4                   ; Count the bits
        bra     nz,shiftright           ; Not there yet ...
        bra     round                   ; Round and pack ...

;------ Shift left

shiftleft:
        add     w8,w8,w8                ; (w9:w8) = (sig <<= 1)
        addc    w9,w9,w9                ; *
        dec     w11,w11                 ; (w11) = exp--
        dec.b   w4,w4                   ; Count the bits
        bra     nz,shiftleft            ; Not there yet ...

;-----------------------------------------------------------------------;
;       Rounding
;-----------------------------------------------------------------------;

round:

;       (w3)   = sticky
;       (w2)   = round
;       (w9:w8)= s(significand)
;       (w11)  = s(exponent)

        rcall   __fpack                 ; Round and pack

        btsc    w10,#15                 ; Result negative ?
        bset    w1,#15                  ; Yes
return0:
        mov.d   [--w15],w10             ; Recover scratch
        mov.d   [--w15],w8              ; Recover scratch
        return                          ; Done

;-----------------------------------------------------------------------;

        .end
