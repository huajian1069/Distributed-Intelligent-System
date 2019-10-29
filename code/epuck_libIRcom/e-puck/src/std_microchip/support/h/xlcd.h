/********************************************************************/
/*            Header for XLCD module library functions for          */
/*            P-tec PCOG1602B LCD controller                        */
/********************************************************************/

#ifndef _XLCD_H
#define _XLCD_H

/*External LCD functions are only defined for the following devices */
#if defined(__dsPIC30F5011__) || defined(__dsPIC30F5013__) || defined(__dsPIC30F6010__) || \
    defined(__dsPIC30F6011__) || defined(__dsPIC30F6012__) || defined(__dsPIC30F6013__) || \
    defined(__dsPIC30F6014__) || defined(__dsPIC30F6010A__) || defined(__dsPIC30F6011A__) || \
    defined(__dsPIC30F6012A__) || defined(__dsPIC30F6013A__) || defined(__dsPIC30F6014A__) || \
    defined(__dsPIC30F5016__) || defined(__dsPIC30F6015__)

/* uncomment the following line if 8 bit interface is being used */
#define EIGHT_BIT_INTERFACE

/* #defines of the data pins and the corresponding tris pins */
#define DATA_PIN_7            PORTDbits.RD7
#define DATA_PIN_6            PORTDbits.RD6
#define DATA_PIN_5            PORTDbits.RD5
#define DATA_PIN_4            PORTDbits.RD4

#ifdef EIGHT_BIT_INTERFACE
#define DATA_PIN_3            PORTDbits.RD3
#define DATA_PIN_2            PORTDbits.RD2
#define DATA_PIN_1            PORTDbits.RD1
#define DATA_PIN_0            PORTDbits.RD0
#endif

#define TRIS_DATA_PIN_7       TRISDbits.TRISD7
#define TRIS_DATA_PIN_6       TRISDbits.TRISD6
#define TRIS_DATA_PIN_5       TRISDbits.TRISD5
#define TRIS_DATA_PIN_4       TRISDbits.TRISD4

#ifdef EIGHT_BIT_INTERFACE
#define TRIS_DATA_PIN_3       TRISDbits.TRISD3
#define TRIS_DATA_PIN_2       TRISDbits.TRISD2
#define TRIS_DATA_PIN_1       TRISDbits.TRISD1
#define TRIS_DATA_PIN_0       TRISDbits.TRISD0
#endif

/* #defines of the control pins and the corresponding tris pins */
#define E_PIN                 PORTDbits.RD8       /* PORT for E */
#define RW_PIN                PORTDbits.RD9       /* PORT for RW */
#define RS_PIN                PORTDbits.RD10      /* PORT for RS */

#define TRIS_E                TRISDbits.TRISD8    /* TRIS for E */
#define TRIS_RW               TRISDbits.TRISD9    /* TRIS for RW */
#define TRIS_RS               TRISDbits.TRISD10   /* TRIS for RS */

/* Display ON/OFF Control defines */
#define DON                   0x0f  /* Display on      */
#define DOFF                  0x0b  /* Display off     */
#define CURSOR_ON             0x0f  /* Cursor on       */
#define CURSOR_OFF            0x0d  /* Cursor off      */
#define BLINK_ON              0x0f  /* Cursor Blink    */
#define BLINK_OFF             0x0e  /* Cursor No Blink */

/* Cursor or Display Shift defines */
#define SHIFT_CUR_LEFT        0x13  /* Cursor shifts to the left   */
#define SHIFT_CUR_RIGHT       0x17  /* Cursor shifts to the right  */
#define SHIFT_DISP_LEFT       0x1b  /* Display shifts to the left  */
#define SHIFT_DISP_RIGHT      0x1f  /* Display shifts to the right */

/* Function Set defines */
#define FOUR_BIT              0x2f  /* 4-bit Interface  */
#define EIGHT_BIT             0x3f  /* 8-bit Interface  */

/* defines for no of lines */
#define SINGLE_LINE           0x37  /* Single line display  */
#define TWO_LINE              0x3F  /* Two line display  */

/* defines for segment data transfer direction */
#define SEG1_50_SEG51_100     0x39  /* SEG1->SEG50 ,SEG51->SEG100 */
#define SEG1_50_SEG100_51     0x3d  /* SEG1->SEG50 ,SEG100->SEG51 */
#define SEG100_51_SEG50_1     0x3b  /* SEG100->SEG51,SEG50->SEG1 */
#define SEG100_51_SEG1_50     0x3f  /* SEG100->SEG51,SEG1->SEG50 */

/* defines for COM data transfer direction */

#define COM1_COM16            0x3e  /* Data transfer direction is COM1 to COM16 */
#define COM16_COM1            0x3f  /* Data transfer direction is COM16 to COM1 */

/* Function prototypes */

/* OpenXLCD
 * Configures I/O pins for external LCD
 */
void OpenXLCD(char) __attribute__ ((section (".libperi")));

/* SetCGRamAddr
 * Sets the character generator address
 */
void SetCGRamAddr(char) __attribute__ ((section (".libperi")));

/* SetDDRamAddr
 * Sets the display data address
 */
void SetDDRamAddr(char) __attribute__ ((section (".libperi")));

/* BusyXLCD
 * Returns the busy status of the LCD
 */
char BusyXLCD(void)    __attribute__ ((section (".libperi")));

/* ReadAddrXLCD
 * Reads the current address
 */
char ReadAddrXLCD(void) __attribute__ ((section (".libperi")));

/* ReadDataXLCD
 * Reads a byte of data
 */
char ReadDataXLCD(void) __attribute__ ((section (".libperi")));

/* WriteCmdXLCD
 * Writes a command to the LCD
 */
void WriteCmdXLCD(char) __attribute__ ((section (".libperi")));

/* WriteDataXLCD
 * Writes a data byte to the LCD
 */
void WriteDataXLCD(char) __attribute__ ((section (".libperi")));

/* putsXLCD
 * Writes a string of characters to the LCD
 */
void putsXLCD(char *) __attribute__ ((section (".libperi")));

#define putrsXLCD   putsXLCD
#define getcXLCD    ReadDataXLCD
#define putcXLCD    WriteDataXLCD

/* prototypes of three delay routines used in XLCD functions */
void Delay18TCY()   __attribute__ ((section (".libperi")));
void DelayPORXLCD() __attribute__ ((section (".libperi")));
void DelayXLCD()    __attribute__ ((section (".libperi")));

#endif
#endif
