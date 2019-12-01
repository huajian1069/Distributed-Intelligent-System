/***************************************************************************************************************

Title:		UART1_RX_CHAR.s

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

.global _U1RXCnt
.global _U1RXBuf
.global _U1RXOK

.section  .data
;_U1RXCnt:  .hword 0x0000
_U1RXBuf:  .hword 0x0000     
_U1RXOK: .hword 0x0000

.global __U1RXInterrupt
.section  .text

__U1RXInterrupt:
        push.d  w0                      ; save context - w0,w1, w2
        bclr    IFS0, #U1RXIF           ;Clear the interrupt flag. 
        clr     w0                      ;Clear w0

        mov.b   U1RXREG, WREG           ;Transfer received byte to w0
		mov		#_U1RXBuf, w1			; pointer in w2
		mov.b	w0,[w1]
		mov		#1,w0
		mov		w0,_U1RXOK

       ; pop     w2                      ;Restore context - w2
        pop.d   w0                      ;Restore w0, w1

        retfie                          ;Return from Interrupt


.global _e_getchar_uart1

_e_getchar_uart1:
		cp0		_U1RXOK					; check flag
		bra		Z,no_char_to_ret	;if flag not ok, no char to return
        bclr    IEC0, #U1RXIE
		clr		w1
		mov		w1,_U1RXOK			; clr flag
		mov		#_U1RXBuf,w2		; move buf to target buf		
		mov.b	[w2],w1
		mov.b	w1,[w0]
		mov		#1,w0				;return 1
        bset    IEC0, #U1RXIE
		return

no_char_to_ret:
		clr		w0					;return 0 because no data
        return


.end                                    ;EOF



