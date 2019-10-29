/*********************************************************************/
/*              Header for Reset/Control library functions           */
/*********************************************************************/

#ifndef __RESET_H
#define __RESET_H

/* disable interrupts for the given number of cycles */
#define DisableInterrupts(cycles) asm ("DISI #" #cycles)

/* macros for POR and BOR resets */
#define PORStatReset               asm ("BSET RCON,#0")
#define BORStatReset               asm ("BSET RCON,#1")

/* macros to enable/ disable WDT in software */
#define WDTSWEnable                asm ("BSET RCON,#5")
#define WDTSWDisable               asm ("BCLR RCON,#5")

char isMCLR(void) __attribute__ ((section (".libperi")));    /* to check if RESET is due to MCLR */
char isPOR(void) __attribute__ ((section (".libperi")));     /* to check if RESET is due to POR */
char isBOR(void) __attribute__ ((section (".libperi")));     /* to check if RESET is due to BOR */

#if defined(__dsPIC30F2011__) || defined(__dsPIC30F2012__) || defined(__dsPIC30F3012__) || \
    defined(__dsPIC30F3013__) || defined(__dsPIC30F3014__) || defined(__dsPIC30F4013__) || \
    defined(__dsPIC30F5011__) || defined (__dsPIC30F5013__) || defined (__dsPIC30F6011__) || \
    defined (__dsPIC30F6012__) || defined (__dsPIC30F6013__) || defined (__dsPIC30F6014__) || \
    defined (__dsPIC30F6011A__) || defined (__dsPIC30F6012A__) || defined (__dsPIC30F6013A__) || \
    defined (__dsPIC30F6014A__)

char isLVD(void) __attribute__ ((section (".libperi")));     /* to check if LVD occured */

#endif

char isWU(void) __attribute__ ((section (".libperi")));      /* to check for the cause of Wakeup from sleep */
char isWDTTO(void) __attribute__ ((section (".libperi")));   /* to check if WDT timeout occured during normal operation */
char isWDTWU(void) __attribute__ ((section (".libperi")));   /* to check if WDT timeout occured during sleep */

#endif /* __RESET_H */
