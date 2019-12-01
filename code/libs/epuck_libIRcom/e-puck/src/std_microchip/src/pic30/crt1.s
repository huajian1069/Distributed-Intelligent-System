;; 
;; C Run-time startup module for dsPIC30 C compiler.
;; (c) Copyright 2002,2004 Microchip Technology, All rights reserved
;;
;; Alternate version, without data initialization support.
;; The linker loads this version when the --no-data-init
;; option is selected.
;; 
;; See file crt0.s for the primary version with
;; data initialization support.
;; 
;; Entry __reset takes control at device reset and
;; performs the following:
;;
;;  1. initialize stack and stack limit register
;;  2. initialize PSV window if __const_length > 0
;;  3. call the user's _main entry point
;;
;; Supports the zero-parameter form of main() by default.
;; If the symbol __ARGV is defined, supports the two-parameter
;; form of main().
;;
;; Un-comment the following line to define symbol __ARGV:
;;      .equiv __ARGV,1
;; 
        .equ __30F2010, 1   
        .include "p30f2010.inc"

        .section .libc,code

        .global __resetALT
        .ifdef __dsPIC30ELF
        .type   __resetALT,@function
        .endif
__resetALT:     
        .weak  __reset
        .ifdef __dsPIC30ELF
        .type   __reset,@function
        .endif
__reset:
;;
;; Initialize stack and PSV window
;; 
;; registers used:  w1
;;  (w0 is cleared by device reset, so ARGC = 0)
;;
;; Inputs (defined by user or linker):
;;  __SP_init         = initial value of stack pointer
;;  __SPLIM_init      = initial value of stack limit register

;; Inputs (defined by linker):
;;  __const_length    = length of section .const
;;  __const_psvpage   = PSVPAG setting for section .const
;; 
;; Outputs:
;;  (does not return - resets the processor)
;; 
;; Calls:
;;  _main or __main
;; 

        mov      #__SP_init,w15      ; initialize w15
        mov      #__SPLIM_init,w1    ; 
        mov      w1,_SPLIM           ; initialize SPLIM

        bclr     _CORCON,#PSV        ; disable PSV (default)
        mov      #__const_length,w1  ; 
        cp0      w1                  ; test length of constants
        bra      z,1f                ; br if zero

        mov      #__const_psvpage,w1 ; 
        mov      w1,_PSVPAG          ; PSVPAG = psvpage(constants)
        bset     _CORCON,#PSV        ; enable PSV
1:
        .ifdef __ARGV
          rcall  __main              ; call two-parameter main() setup
        .else
          call  _main                ; call user's main()
        .endif

        .pword 0xDA4000              ; halt the simulator
        reset                        ; reset the processor

        
        .ifdef __ARGV        
__main:
;; 
;; Initialize a stack frame to support
;;   the two-parameter form of main()
;; 
;; Registers used:  w0 w1
;;
;; Inputs:
;;  (none)
;;
;; Outputs:
;;  (none)
;;
;; Calls:
;;  _main
;; 
        lnk      #6             ; construct frame

        mov      #0,w0          ; store a null terminator
        mov.b    w0,[w15-6]     ;  for program name

        sub      w15,#6,w1
        mov      w1,[w15-4]     ; argv[0] = &pgmname             
        mov      w0,[w15-2]     ; argv[1] = 0

        mov      #1,w0          ; w0 = argc
        sub      w15,#4,w1      ; w1 = &argv
        call    _main           ; call the user's main()

        ulnk                    ; destroy frame
        return                  ;  and exit
        .endif

        .end

