/***************************************************************************************************************

Title:		UART2_RX_CHAR.s

Author:		Inspired Microchip library
			Davis Daidié
			Bonani Michael

History:
	11/07/05		Start day
	18/07/05		To be continued
	21/12/05		Optimisation and new function name 

****************************************************************************************************************/
;to be used with uart_txrx_char.h

.include "p30f6014a.inc"

.global _U2RXCnt
.global _U2RXBuf
.global _U2RXOK

.section  .data
_U2RXBuf:  .hword 0x0000     
_U2RXOK:   .hword 0x0000

.global __U2RXInterrupt
.section  .text

__U2RXInterrupt:
        push.d  w0                      ; save context - w0,w1, w2					
        clr     w0                      ;Clear w0

        mov.b   U2RXREG, WREG           ;Transfer received byte to w0
		mov		#_U2RXBuf, w1			; pointer in w2
		mov.b	w0,[w1]
		mov		#1,w0
		mov		w0,_U2RXOK

        pop.d   w0                      ;Restore w0, w1
        bclr    IFS1, #U2RXIF           ;Clear the interrupt flag.
        retfie                          ;Return from Interrupt


.global _e_getchar_uart2

_e_getchar_uart2:
		cp0		_U2RXOK					; check flag
		bra		Z,no_char_to_ret	;if flag not ok, no char to return
		clr		w1
		mov		w1,_U2RXOK			; clr flag
		mov		#_U2RXBuf,w2		; move buf to target buf		
		mov.b	[w2],w1
		mov.b	w1,[w0]
		mov		#1,w0				;return 1
		return

no_char_to_ret:
		clr		w0					;return 0 because no data
        return
.end                                    ;EOF



