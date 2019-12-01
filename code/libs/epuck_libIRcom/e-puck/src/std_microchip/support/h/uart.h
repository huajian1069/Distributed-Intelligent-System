/********************************************************************
                Header for UART module library functions
********************************************************************/
#ifndef __UART_H
#define __UART_H

/* List of SFRs for UART */
/* This list contains the SFRs with default (POR) values to be used for configuring UART */
/* The user can modify this based on the requirement */
#define UxMODE_VALUE            0x0000
#define UxSTA_VALUE             0x0110
#define UxTXREG_VALUE           0x0000
#define UxRXREG_VALUE           0x0000
#define UxBRG_VALUE             0x0000

#define getcUART1               ReadUART1
#define putcUART1               WriteUART1

#define getcUART2               ReadUART2
#define putcUART2               WriteUART2

/* defines for UxMODE register */
#define UART_EN                 0xEFE7  /* Module enable */
#define UART_DIS                0x6FE7  /* Module disable */

#define UART_IDLE_CON           0xCFE7  /* Work in IDLE mode */
#define UART_IDLE_STOP          0xEFE7  /* Stop all functions in IDLE mode*/

/*ALTIO pin for UART1 is defined for following devices */
#if defined(__dsPIC30F2010__) || defined(__dsPIC30F3010__) || defined(__dsPIC30F3011__) || \
    defined(__dsPIC30F4011__) || defined(__dsPIC30F4012__) || defined(__dsPIC30F2011__) || \
    defined(__dsPIC30F3012__) || defined(__dsPIC30F3013__) || defined(__dsPIC30F2012__) || \
    defined(__dsPIC30F3014__) || defined(__dsPIC30F4013__) 
	
#define UART_ALTRX_ALTTX        0xEFE7  /*Communication through ALT pins*/
#define UART_RX_TX              0xEBE7  /*Communication through the normal pins*/

#endif

#define UART_EN_WAKE            0xEFE7  /*Enable Wake-up on START bit Detect during SLEEP Mode bit*/
#define UART_DIS_WAKE           0xEF67  /*Disable Wake-up on START bit Detect during SLEEP Mode bit*/

#define UART_EN_LOOPBACK        0xEFE7  /*Loop back enabled*/
#define UART_DIS_LOOPBACK       0xEFA7  /*Loop back disabled*/

#define UART_EN_ABAUD           0xEFE7  /*Input to Capture module from UxRX pin*/
#define UART_DIS_ABAUD          0xEFC7  /*Input to Capture module from ICx pin*/

#define UART_NO_PAR_9BIT        0xEFE7  /*No parity 9 bit*/
#define UART_ODD_PAR_8BIT       0xEFE5  /*odd parity 8 bit*/
#define UART_EVEN_PAR_8BIT      0xEFE3  /*even parity 8 bit*/
#define UART_NO_PAR_8BIT        0xEFE1  /*no parity 8 bit*/

#define UART_2STOPBITS          0xEFE7  /*2 stop bits*/
#define UART_1STOPBIT           0xEFE6  /*1 stop bit*/

/* defines for UART Status register */

#define UART_INT_TX_BUF_EMPTY   0xFFFF  /* Interrupt on TXBUF becoming empty */
#define UART_INT_TX             0x7FFF  /* Interrupt on transfer of every character to TSR */

#define UART_TX_PIN_NORMAL      0xF7FF  /* UART TX pin operates normally */
#define UART_TX_PIN_LOW         0xFFFF  /* UART TX pin driven low */

#define UART_TX_ENABLE          0xFFFF  /* Transmit enable */
#define UART_TX_DISABLE         0xFBFF  /* Transmit disable */

#define UART_INT_RX_BUF_FUL     0xFFFF  /* Interrupt on RXBUF full */
#define UART_INT_RX_3_4_FUL     0xFFBF  /* Interrupt on RXBUF 3/4 full */
#define UART_INT_RX_CHAR        0xFF7F  /* Interrupt on every char received */\

#define UART_ADR_DETECT_EN      0xFFFF  /* address detect enable */
#define UART_ADR_DETECT_DIS     0xFFDF  /* address detect disable */

#define UART_RX_OVERRUN_CLEAR   0xFFFD  /* Rx buffer Over run status bit clear */

/* defines for UART Interrupt configuartion */
#define UART_RX_INT_EN          0xFFFF  /*Receive interrupt enabled*/
#define UART_RX_INT_DIS         0xFFF7  /*Receive interrupt disabled*/

#define UART_RX_INT_PR0         0xFFF8  /*Priority RX interrupt 0*/
#define UART_RX_INT_PR1         0xFFF9  /*Priority RX interrupt 1*/
#define UART_RX_INT_PR2         0xFFFA  /*Priority RX interrupt 2*/
#define UART_RX_INT_PR3         0xFFFB  /*Priority RX interrupt 3*/
#define UART_RX_INT_PR4         0xFFFC  /*Priority RX interrupt 4*/
#define UART_RX_INT_PR5         0xFFFD  /*Priority RX interrupt 5*/
#define UART_RX_INT_PR6         0xFFFE  /*Priority RX interrupt 6*/
#define UART_RX_INT_PR7         0xFFFF  /*Priority RX interrupt 7*/

#define UART_TX_INT_EN          0xFFFF  /*transmit interrupt enabled*/
#define UART_TX_INT_DIS         0xFF7F  /*transmit interrupt disabled*/

#define UART_TX_INT_PR0         0xFF8F  /*Priority TX interrupt 0*/
#define UART_TX_INT_PR1         0xFF9F  /*Priority TX interrupt 1*/
#define UART_TX_INT_PR2         0xFFAF  /*Priority TX interrupt 2*/
#define UART_TX_INT_PR3         0xFFBF  /*Priority TX interrupt 3*/
#define UART_TX_INT_PR4         0xFFCF  /*Priority TX interrupt 4*/
#define UART_TX_INT_PR5         0xFFDF  /*Priority TX interrupt 5*/
#define UART_TX_INT_PR6         0xFFEF  /*Priority TX interrupt 6*/
#define UART_TX_INT_PR7         0xFFFF  /*Priority TX interrupt 7*/

/* Macros to  Enable/Disable interrupts and set Interrupt priority of UART1 */
#define EnableIntU1RX                    asm("BSET IEC0,#9")
#define EnableIntU1TX                    asm("BSET IEC0,#10")

#define DisableIntU1RX                   asm("BCLR IEC0,#9")
#define DisableIntU1TX                   asm("BCLR IEC0,#10")

#define SetPriorityIntU1RX(priority)     (IPC2bits.U1RXIP = priority)
#define SetPriorityIntU1TX(priority)     (IPC2bits.U1TXIP = priority)

/************************Function prototype**************************/
void putsUART1(unsigned int *buffer) __attribute__ ((section (".libperi")));

void WriteUART1(unsigned int data) __attribute__ ((section (".libperi")));

void CloseUART1(void) __attribute__ ((section (".libperi")));

void ConfigIntUART1(unsigned int config) __attribute__ ((section (".libperi")));

char DataRdyUART1(void) __attribute__ ((section (".libperi")));

unsigned int getsUART1(unsigned int length,unsigned int *buffer, 
                       unsigned int uart_data_wait) __attribute__ ((section (".libperi")));

void OpenUART1(unsigned int config1,unsigned int config2, unsigned int ubrg) __attribute__ ((section (".libperi")));

unsigned int ReadUART1(void) __attribute__ ((section (".libperi")));

char BusyUART1(void) __attribute__ ((section (".libperi")));

/*UART2 is defined in following devices */
#if defined(__dsPIC30F3011__) || defined(__dsPIC30F4011__) || defined(__dsPIC30F6010__) || \
    defined(__dsPIC30F3013__) || defined(__dsPIC30F3014__) || defined(__dsPIC30F5011__)	|| \
    defined(__dsPIC30F6011__) || defined(__dsPIC30F6012__) || defined(__dsPIC30F5013__) || \
    defined(__dsPIC30F6013__) || defined(__dsPIC30F6014__) || defined(__dsPIC30F4013__) || \
    defined(__dsPIC30F6010A__) || defined(__dsPIC30F6011A__) || defined(__dsPIC30F6012A__) || \
    defined(__dsPIC30F6013A__) || defined(__dsPIC30F6014A__) || defined(__dsPIC30F6015__)

/* Macros to  Enable/Disable interrupts and set Interrupt priority of UART2 */
#define EnableIntU2RX                    asm("BSET IEC1,#8")
#define EnableIntU2TX                    asm("BSET IEC1,#9")

#define DisableIntU2RX                   asm("BCLR IEC1,#8")
#define DisableIntU2TX                   asm("BCLR IEC1,#9")

#define SetPriorityIntU2RX(priority)     (IPC6bits.U2RXIP = priority)
#define SetPriorityIntU2TX(priority)     (IPC6bits.U2TXIP = priority)

void putsUART2(unsigned int *buffer) __attribute__ ((section (".libperi")));

void WriteUART2(unsigned int data) __attribute__ ((section (".libperi")));

void CloseUART2(void) __attribute__ ((section (".libperi")));

void ConfigIntUART2(unsigned int config) __attribute__ ((section (".libperi")));

char DataRdyUART2(void) __attribute__ ((section (".libperi")));

unsigned int getsUART2(unsigned int length,unsigned int *buffer, 
                       unsigned int uart_data_wait) __attribute__ ((section (".libperi")));

void OpenUART2(unsigned int config1,unsigned int config2, unsigned int ubrg) __attribute__ ((section (".libperi")));

unsigned int ReadUART2(void) __attribute__ ((section (".libperi")));

char BusyUART2(void) __attribute__ ((section (".libperi")));

#endif

#endif /*__UART_H */
