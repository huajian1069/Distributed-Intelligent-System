/*********************************************************************/
/*              Header for IO ports module library functions         */
/*********************************************************************/

#ifndef __PORTS_H
#define __PORTS_H

/* List of SFRs for IO ports */
/* This list contains the SFRs with default (POR) values to be used for configuring IOports */
/* The user can modify this based on the requirement */
#define CNEN1_VALUE             0x0000
#define CNPU1_VALUE             0x0000

#define CNEN2_VALUE             0x0000
#define CNPU2_VALUE             0x0000

#define IEC0_VALUE              0x0000
#define IEC1_VALUE              0x0000
#define IEC2_VALUE              0x0000

#define IFS0_VALUE              0x0000
#define IFS1_VALUE              0x0000
#define IFS2_VALUE              0x0000

#define IPC0_VALUE              0x4444
#define IPC1_VALUE              0x4444
#define IPC2_VALUE              0x4444
#define IPC3_VALUE              0x4444
#define IPC4_VALUE              0x4444
#define IPC5_VALUE              0x4444
#define IPC6_VALUE              0x4444
#define IPC7_VALUE              0x4444
#define IPC8_VALUE              0x4444
#define IPC9_VALUE              0x4444
#define IPC10_VALUE             0x4444
#define IPC11_VALUE             0x4444

/* Config Bit Defines to be used for CN interrupt configuration */
#define CHANGE_INT_ON           0xFFFFFFFF    /* interrupts on  */
#define CHANGE_INT_OFF          0xF7FFFFFF    /* interrupts off */

/* Setting the priority of CN interrupt */
#define CHANGE_INT_PRI_0        0xF8FFFFFF
#define CHANGE_INT_PRI_1        0xF9FFFFFF
#define CHANGE_INT_PRI_2        0xFAFFFFFF
#define CHANGE_INT_PRI_3        0xFBFFFFFF
#define CHANGE_INT_PRI_4        0xFCFFFFFF
#define CHANGE_INT_PRI_5        0xFDFFFFFF
#define CHANGE_INT_PRI_6        0xFEFFFFFF
#define CHANGE_INT_PRI_7        0xFFFFFFFF


/* Setting the priority of External interrupt */
#define EXT_INT_PRI_0           0xFFF8
#define EXT_INT_PRI_1           0xFFF9
#define EXT_INT_PRI_2           0xFFFA
#define EXT_INT_PRI_3           0xFFFB
#define EXT_INT_PRI_4           0xFFFC
#define EXT_INT_PRI_5           0xFFFD
#define EXT_INT_PRI_6           0xFFFE
#define EXT_INT_PRI_7           0xFFFF

/* enable / disable External interrupt */
#define EXT_INT_ENABLE          0xFFFF
#define EXT_INT_DISABLE         0xFFF7

/* External interrupts edge selection defines */
#define RISING_EDGE_INT         0xFFEF    /*Interrupt is set by a 
                                                    rising edge  */
#define FALLING_EDGE_INT        0xFFFF    /*Interrupt is set by a
                                                     falling edge */

/* Macros to Enable CN interrupts */
#define EnableCN0               asm ("BSET CNEN1,#0")
#define EnableCN1               asm ("BSET CNEN1,#1")
#define EnableCN2               asm ("BSET CNEN1,#2")
#define EnableCN3               asm ("BSET CNEN1,#3")
#define EnableCN4               asm ("BSET CNEN1,#4")
#define EnableCN5               asm ("BSET CNEN1,#5")
#define EnableCN6               asm ("BSET CNEN1,#6")
#define EnableCN7               asm ("BSET CNEN1,#7")

#if defined(__dsPIC30F5011__) || defined(__dsPIC30F6011__) || defined(__dsPIC30F6012__) || \
    defined(__dsPIC30F6010__) || defined(__dsPIC30F5013__) || defined(__dsPIC30F6013__) || \
    defined(__dsPIC30F6014__) || defined(__dsPIC30F5015__) || defined(__dsPIC30F6010A__) || \
    defined(__dsPIC30F6011A__) || defined(__dsPIC30F6012A__) || defined(__dsPIC30F6013A__) || \
    defined(__dsPIC30F6014A__) || defined(__dsPIC30F5016__) || defined(__dsPIC30F6015__)
#define EnableCN8               asm ("BSET CNEN1,#8")
#define EnableCN9               asm ("BSET CNEN1,#9")
#define EnableCN10              asm ("BSET CNEN1,#10")
#define EnableCN11              asm ("BSET CNEN1,#11")
#define EnableCN12              asm ("BSET CNEN1,#12")
#define EnableCN13              asm ("BSET CNEN1,#13")
#define EnableCN14              asm ("BSET CNEN1,#14")
#define EnableCN15              asm ("BSET CNEN1,#15")
#define EnableCN16              asm ("BSET CNEN2,#0")
#define EnableCN17              asm ("BSET CNEN2,#1")
#define EnableCN18              asm ("BSET CNEN2,#2")
#endif

#if defined(__dsPIC30F6010__) || defined(__dsPIC30F5013__) || defined(__dsPIC30F6013__) || \
    defined(__dsPIC30F6014__) || defined(__dsPIC30F6010A__) || defined(__dsPIC30F6013A__) || \
    defined(__dsPIC30F6014A__) || defined(__dsPIC30F5016__)
#define EnableCN19              asm ("BSET CNEN2,#3")
#define EnableCN20              asm ("BSET CNEN2,#4")
#define EnableCN21              asm ("BSET CNEN2,#5")
#endif

#if defined(__dsPIC30F5013__) || defined(__dsPIC30F6013__) || defined(__dsPIC30F6014__) || \
    defined(__dsPIC30F6013A__) || defined(__dsPIC30F6014A__)
#define EnableCN22              asm ("BSET CNEN2,#6")
#define EnableCN23              asm ("BSET CNEN2,#7")
#endif

/* Macros to Disable CN interrupts */
#define DisableCN0              asm ("BCLR CNEN1,#0")
#define DisableCN1              asm ("BCLR CNEN1,#1")
#define DisableCN2              asm ("BCLR CNEN1,#2")
#define DisableCN3              asm ("BCLR CNEN1,#3")
#define DisableCN4              asm ("BCLR CNEN1,#4")
#define DisableCN5              asm ("BCLR CNEN1,#5")
#define DisableCN6              asm ("BCLR CNEN1,#6")
#define DisableCN7              asm ("BCLR CNEN1,#7")

#if defined(__dsPIC30F5011__) || defined(__dsPIC30F6011__) || defined(__dsPIC30F6012__) || \
    defined(__dsPIC30F6010__) || defined(__dsPIC30F5013__) || defined(__dsPIC30F6013__) || \
    defined(__dsPIC30F6014__) || defined(__dsPIC30F5015__) || defined(__dsPIC30F6010A__) || \
    defined(__dsPIC30F6011A__) || defined(__dsPIC30F6012A__) || defined(__dsPIC30F6013A__) || \
    defined(__dsPIC30F6014A__) || defined(__dsPIC30F5016__) || defined(__dsPIC30F6015__)
#define DisableCN8              asm ("BCLR CNEN1,#8")
#define DisableCN9              asm ("BCLR CNEN1,#9")
#define DisableCN10             asm ("BCLR CNEN1,#10")
#define DisableCN11             asm ("BCLR CNEN1,#11")
#define DisableCN12             asm ("BCLR CNEN1,#12")
#define DisableCN13             asm ("BCLR CNEN1,#13")
#define DisableCN14             asm ("BCLR CNEN1,#14")
#define DisableCN15             asm ("BCLR CNEN1,#15")
#define DisableCN16             asm ("BCLR CNEN2,#0")
#define DisableCN17             asm ("BCLR CNEN2,#1")
#define DisableCN18             asm ("BCLR CNEN2,#2")
#endif

#if defined(__dsPIC30F6010__) || defined(__dsPIC30F5013__) || defined(__dsPIC30F6013__) || \
    defined(__dsPIC30F6014__) || defined(__dsPIC30F6010A__) || defined(__dsPIC30F6013A__) || \
    defined(__dsPIC30F6014A__) || defined(__dsPIC30F5016__)
#define DisableCN19             asm ("BCLR CNEN2,#3")
#define DisableCN20             asm ("BCLR CNEN2,#4")
#define DisableCN21             asm ("BCLR CNEN2,#5")
#endif

#if defined(__dsPIC30F5013__) || defined(__dsPIC30F6013__) || defined(__dsPIC30F6014__) || \
    defined(__dsPIC30F6013A__) || defined(__dsPIC30F6014A__)
#define DisableCN22             asm ("BCLR CNEN2,#6")
#define DisableCN23             asm ("BCLR CNEN2,#7")
#endif

/* Macros to Enable External interrupts */
#define EnableINT0              asm ("BSET IEC0, #0")
#define EnableINT1              asm ("BSET IEC1, #0")
#define EnableINT2              asm ("BSET IEC1, #7")
#define EnableINT3              asm ("BSET IEC2, #4")
#define EnableINT4              asm ("BSET IEC2, #5")

/* Macros to Disable External interrupts */
#define DisableINT0             asm ("BCLR IEC0, #0")
#define DisableINT1             asm ("BCLR IEC1, #0")
#define DisableINT2             asm ("BCLR IEC1, #7")
#define DisableINT3             asm ("BCLR IEC2, #4")
#define DisableINT4             asm ("BCLR IEC2, #5")

/* Macros to set priority for External interrupts */
#define SetPriorityInt0(priority)     (IPC0bits.INT0IP = priority)
#define SetPriorityInt1(priority)     (IPC4bits.INT1IP = priority)
#define SetPriorityInt2(priority)     (IPC5bits.INT2IP = priority)
#define SetPriorityInt3(priority)     (IPC9bits.INT3IP = priority)
#define SetPriorityInt4(priority)     (IPC9bits.INT4IP = priority)


/* Function Prototypes */

/* ConfigCNPullups
 * Enable/ disable pull up registers
 */
void ConfigCNPullups(long int) __attribute__ ((section (".libperi")));

/* ConfigIntCN
 * Enable/Disable CN interrupt and set priority
 */
void ConfigIntCN(long int) __attribute__ ((section (".libperi")));

/* configINT0
 * Enable/disable INT0 interrupts,set priority and falling edge/rising edge
 */
void configINT0(unsigned int) __attribute__ ((section (".libperi")));

/* CloseINT0
 * Disable INT0 interrupts, registers
 */
void CloseINT0() __attribute__ ((section (".libperi")));

/* configINT1
 * Enable/disable INT1 interrupt,set priority and falling edge/rising edge
 */
void configINT1(unsigned int) __attribute__ ((section (".libperi")));

/* CloseINT1
 * Disable INT1 interrupts, registers
 */
void CloseINT1() __attribute__ ((section (".libperi")));

/* configINT2
 * Enable/disable INT2 interrupt,set priority and falling edge/rising edge
 */
void configINT2(unsigned int) __attribute__ ((section (".libperi")));

/* CloseINT2
 * Disable INT2 interrupts, registers
 */
void CloseINT2() __attribute__ ((section (".libperi")));

#if defined(__dsPIC30F6010__) || defined(__dsPIC30F5011__) || defined(__dsPIC30F6011__)|| \
    defined(__dsPIC30F6012__) || defined(__dsPIC30F5013__) || defined(__dsPIC30F6013__)|| \
    defined(__dsPIC30F6014__) || defined(__dsPIC30F6010A__) || defined(__dsPIC30F6011A__) || \
    defined(__dsPIC30F6012A__) || defined(__dsPIC30F6013A__) || defined(__dsPIC30F6014A__) || \
    defined(__dsPIC30F5016__) || defined(__dsPIC30F6015__)

/* configINT3
 * Enable/disable INT3 interrupt,set priority and falling edge/rising edge
 */
void configINT3(unsigned int) __attribute__ ((section (".libperi")));

/* CloseINT3
 * Disable INT3 interrupts, registers
 */
void CloseINT3() __attribute__ ((section (".libperi")));

/* configINT4
 * Enable/disable INT4 interrupt,set priority and falling edge/rising edge
 */
void configINT4(unsigned int) __attribute__ ((section (".libperi")));

/* CloseINT4
 * Disable INT4 interrupts, registers
 */
void CloseINT4() __attribute__ ((section (".libperi")));

#endif

#endif
