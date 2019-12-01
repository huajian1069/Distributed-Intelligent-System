/********************************************************************/
//              Header for OutCompare module library functions      */
/********************************************************************/
#ifndef __OutCompare_H
#define __OutCompare_H

/* List of SFRs for Output compare modules */

/* This list contains the SFRs with default (POR) values to be used 
   for configuring Output compare modules 
   The user can modify this based on the requirement */
#define OC1RS_VALUE                0x0000
#define OC1R_VALUE                 0x0000
#define OC1CON_VALUE               0x0000
#define OC2RS_VALUE                0x0000
#define OC2R_VALUE                 0x0000
#define OC2CON_VALUE               0x0000

/* These devices support OutputCompare 3 and OutputCompare 4 */
#if defined (__dsPIC30F3011__) || defined (__dsPIC30F4011__) || defined (__dsPIC30F6010__) || \
    defined (__dsPIC30F5011__) || defined (__dsPIC30F6011__) || defined (__dsPIC30F6012__) || \
    defined (__dsPIC30F5013__) || defined (__dsPIC30F6013__) || defined (__dsPIC30F6014__) || \
    defined (__dsPIC30F4013__) || defined (__dsPIC30F5015__) || defined (__dsPIC30F6010A__) || \
    defined (__dsPIC30F6011A__) || defined (__dsPIC30F6012A__) || defined (__dsPIC30F6013A__) || \
    defined (__dsPIC30F6014A__) || defined (__dsPIC30F5016__) || defined (__dsPIC30F6015__)
#define OC3RS_VALUE                0x0000
#define OC3R_VALUE                 0x0000
#define OC3CON_VALUE               0x0000
#define OC4RS_VALUE                0x0000
#define OC4R_VALUE                 0x0000
#define OC4CON_VALUE               0x0000
#endif

/* This devices support OutputCompare 5 thru OutputCompare 8 */
#if defined (__dsPIC30F6010__) || defined (__dsPIC30F5011__) || defined (__dsPIC30F6011__) || \
    defined (__dsPIC30F6012__) || defined (__dsPIC30F5013__) || defined (__dsPIC30F6013__) || \
    defined (__dsPIC30F6014__) || defined (__dsPIC30F6010A__) || defined (__dsPIC30F6011A__) || \
    defined (__dsPIC30F6012A__) || defined (__dsPIC30F6013A__) || defined (__dsPIC30F6014A__) || \
    defined (__dsPIC30F6015__) 
#define OC5RS_VALUE                0x0000
#define OC5R_VALUE                 0x0000
#define OC5CON_VALUE               0x0000
#define OC6RS_VALUE                0x0000
#define OC6R_VALUE                 0x0000
#define OC6CON_VALUE               0x0000
#define OC7RS_VALUE                0x0000
#define OC7R_VALUE                 0x0000
#define OC7CON_VALUE               0x0000
#define OC8RS_VALUE                0x0000
#define OC8R_VALUE                 0x0000
#define OC8CON_VALUE               0x0000
#endif

/* Output Compare Stop in Idle mode Bit defines */
#define OC_IDLE_STOP               0xffff   /* stop in idle mode */
#define OC_IDLE_CON                0xdfff   /* continue operation in idle mode */

/* Output Compare timer select Bit Defines */
#define OC_TIMER2_SRC               0xfff7  /* Timer2 is the clock source 
                                                    for OutputCompare1 */
#define OC_TIMER3_SRC               0xffff  /* Timer3 is the clock source
                                                     for OutputCompare1 */

/* PWM Mode on OCx, fault pin enabled, (TxIF bit is set for PWM, OCxIF is set for fault)*/
#define OC_PWM_FAULT_PIN_ENABLE     0xffff

/* PWM Mode on OCx, fault pin disabled */
#define OC_PWM_FAULT_PIN_DISABLE    0xfffe
   
/* Generates Continuous Output pulse on OCx Pin */
#define OC_CONTINUE_PULSE           0xfffd
  
/* Generates Single Output pulse on OCx Pin */
#define OC_SINGLE_PULSE             0xfffc  
#define OC_TOGGLE_PULSE             0xfffb  /* Compare1 toggels  OCx pin*/
#define OC_HIGH_LOW                 0xfffa  /* Compare1 forces   OCx pin Low*/
#define OC_LOW_HIGH                 0xfff9  /* Compare1 forces  OCx pin High*/
#define OC_OFF                      0xfff8  /* OutputCompare x Off*/


/* Interrupt bit definitions */
#define OC_INT_ON                   0xffff  /* OutputCompare Enable  */
#define OC_INT_OFF                  0xfff7  /* OutputCompare Disable */
    
#define OC_INT_PRIOR_0              0xfff8  /* OutputCompare PriorityLevel 0 */
#define OC_INT_PRIOR_1              0xfff9  /* OutputCompare PriorityLevel 1 */
#define OC_INT_PRIOR_2              0xfffa  /* OutputCompare PriorityLevel 2 */
#define OC_INT_PRIOR_3              0xfffb  /* OutputCompare PriorityLevel 3 */
#define OC_INT_PRIOR_4              0xfffc  /* OutputCompare PriorityLevel 4 */
#define OC_INT_PRIOR_5              0xfffd  /* OutputCompare PriorityLevel 5 */
#define OC_INT_PRIOR_6              0xfffe  /* OutputCompare PriorityLevel 6 */
#define OC_INT_PRIOR_7              0xffff  /* OutputCompare PriorityLevel 7 */

#define Start                       0x01
#define Stop                        0x00

/* Function Prototypes for OutputCompare1 to OutPutCompare2 */

/* Macros to  Enable/Disable interrupts and set Interrupt priority of Output compare 1 thru 2 */
#define EnableIntOC1                    asm("BSET IEC0,#2")
#define EnableIntOC2                    asm("BSET IEC0,#5")

#define DisableIntOC1                   asm("BCLR IEC0,#2")
#define DisableIntOC2                   asm("BCLR IEC0,#5")

#define SetPriorityIntOC1(priority)     (IPC0bits.OC1IP = priority)
#define SetPriorityIntOC2(priority)     (IPC1bits.OC2IP = priority)

/*  CloseOC 1    */
void CloseOC1() __attribute__((section (".libperi")));

/*  CloseOC 2    */
void CloseOC2() __attribute__((section (".libperi")));

/* ConfigIntOC1  */
void ConfigIntOC1(unsigned int config) __attribute__ ((section (".libperi")));

/* ConfigIntOC2  */
void ConfigIntOC2(unsigned int config) __attribute__ ((section (".libperi")));

/*  ReadDCOC1PWM    */
unsigned int ReadDCOC1PWM() __attribute__((section (".libperi")));

/*  ReadDCOC2PWM    */
unsigned int ReadDCOC2PWM() __attribute__((section (".libperi")));
/*  ReadRegOC if Reg is Start reads Secondary Register and if Reg
    is  Stop read  Main Register    */
unsigned int ReadRegOC1(char reg) __attribute__((section (".libperi")));

/*  ReadRegOC if Reg is Start reads Secondary Register and if Reg
    is  Stop read  Main Register    */
unsigned int ReadRegOC2(char reg) __attribute__((section (".libperi")));


/*  SetDCOC1PWM  */
void SetDCOC1PWM(unsigned int dutycycle) __attribute__((section (".libperi")));

    /*  SetDCOC2PWM  */
void SetDCOC2PWM(unsigned int dutycycle) __attribute__((section (".libperi")));

    /*  SetPulseOC1    */
void SetPulseOC1(unsigned int pulse_start,unsigned int pulse_stop) __attribute__((section (".libperi")));

    /*  SetPulseOC2    */
void SetPulseOC2(unsigned int pulse_start,unsigned int pulse_stop) __attribute__((section (".libperi")));


    /*  OpenOC1   */
void OpenOC1(unsigned int config,unsigned int value1,
                 unsigned int value2) __attribute__ ((section (".libperi")));
    
    /*  OpenOC2   */
void OpenOC2(unsigned int config,unsigned int value1,
                 unsigned int value2) __attribute__ ((section (".libperi")));
    

/* These devices support OutputCompare 3 and OutputCompare 4 */
#if defined (__dsPIC30F3011__) || defined (__dsPIC30F4011__) || defined (__dsPIC30F6010__) || \
    defined (__dsPIC30F5011__) || defined (__dsPIC30F6011__) || defined (__dsPIC30F6012__) || \
    defined (__dsPIC30F5013__) || defined (__dsPIC30F6013__) || defined (__dsPIC30F6014__) || \
    defined (__dsPIC30F4013__) || defined (__dsPIC30F5015__) || defined (__dsPIC30F6010A__) || \
    defined (__dsPIC30F6011A__) || defined (__dsPIC30F6012A__) || defined (__dsPIC30F6013A__) || \
    defined (__dsPIC30F6014A__) || defined (__dsPIC30F5016__) || defined (__dsPIC30F6015__)

/* Macros to  Enable/Disable interrupts and set Interrupt priority of Output compare 3 thru 4 */
#define EnableIntOC3                    asm("BSET IEC1,#3")
#define EnableIntOC4                    asm("BSET IEC1,#4")

#define DisableIntOC3                   asm("BCLR IEC1,#3")
#define DisableIntOC4                   asm("BCLR IEC1,#4")

#define SetPriorityIntOC3(priority)     (IPC4bits.OC3IP = priority)
#define SetPriorityIntOC4(priority)     (IPC5bits.OC4IP = priority)

/*  CloseOC3    */
void CloseOC3() __attribute__ ((section (".libperi")));

/*  CloseOC4    */
void CloseOC4() __attribute__ ((section (".libperi")));

/* ConfigIntOC3  */
void ConfigIntOC3(unsigned int config) __attribute__((section(".libperi")));

/* ConfigIntOC4  */
void ConfigIntOC4(unsigned int config) __attribute__((section(".libperi")));

/*  ReadDCOC3    */
unsigned int ReadDCOC3PWM() __attribute__ ((section (".libperi")));

/*  ReadDCOC4    */
unsigned int ReadDCOC4PWM() __attribute__ ((section (".libperi")));

/*  ReadRegOC if Reg is Start reads Secondary Register and if Reg
    is  Stop read  Main Register    */
unsigned int ReadRegOC3(char reg) __attribute__ ((section(".libperi")));

/*  ReadRegOC if Reg is Start reads Secondary Register and if Reg
    is  Stop read  Main Register    */
unsigned int ReadRegOC4(char reg) __attribute__ ((section(".libperi")));

/*  OpenOC3   */
void OpenOC3(unsigned int config,unsigned int value1,
             unsigned int value2) __attribute__ ((section(".libperi")));

/*  OpenOC4   */
void OpenOC4(unsigned int config,unsigned int value1,
                 unsigned int value2) __attribute__ ((section(".libperi")));

/*  SetDCOC3PWM  */
void SetDCOC3PWM(unsigned int dutycycle) __attribute__ ((section(".libperi")));

/*  SetDCOC4PWM  */
void SetDCOC4PWM(unsigned int dutycycle) __attribute__ ((section(".libperi")));

/*  SetPulseOC3    */
void SetPulseOC3(unsigned int pulse_start,unsigned int pulse_stop) __attribute__ 
((section (".libperi")));

/*  SetPulseOC4    */
void SetPulseOC4(unsigned int pulse_start,unsigned int pulse_stop) __attribute__ 
((section (".libperi")));

#endif 

/* This devices support OutputCompare 5 thru OutputCompare 8 */
#if defined (__dsPIC30F6010__) || defined (__dsPIC30F5011__) || defined (__dsPIC30F6011__) || \
    defined (__dsPIC30F6012__) || defined (__dsPIC30F5013__) || defined (__dsPIC30F6013__) || \
    defined (__dsPIC30F6014__) || defined (__dsPIC30F6010A__) || defined (__dsPIC30F6011A__) || \
    defined (__dsPIC30F6012A__) || defined (__dsPIC30F6013A__) || defined (__dsPIC30F6014A__) || \
    defined (__dsPIC30F6015__)

/* Macros to  Enable/Disable interrupts and set Interrupt priority of output compare 5 thru 8 */
#define EnableIntOC5                    asm("BSET IEC2,#0")
#define EnableIntOC6                    asm("BSET IEC2,#1")
#define EnableIntOC7                    asm("BSET IEC2,#2")
#define EnableIntOC8                    asm("BSET IEC2,#3")

#define DisableIntOC5                   asm("BCLR IEC2,#0")
#define DisableIntOC6                   asm("BCLR IEC2,#1")
#define DisableIntOC7                   asm("BCLR IEC2,#2")
#define DisableIntOC8                   asm("BCLR IEC2,#3")

#define SetPriorityIntOC5(priority)     (IPC8bits.OC5IP = priority)
#define SetPriorityIntOC6(priority)     (IPC8bits.OC6IP = priority)
#define SetPriorityIntOC7(priority)     (IPC8bits.OC7IP = priority)
#define SetPriorityIntOC8(priority)     (IPC8bits.OC8IP = priority)

    /*  CloseOC 5    */
void CloseOC5() __attribute__ ((section (".libperi")));

    /*  CloseOC 6    */
void CloseOC6() __attribute__ ((section (".libperi")));

    /*  CloseOC 7    */
void CloseOC7() __attribute__ ((section (".libperi")));

    /*  CloseOC 8    */
void CloseOC8() __attribute__ ((section (".libperi")));

    /* ConfigIntOC5  */
void ConfigIntOC5(unsigned int config) __attribute__ ((section
(".libperi")));

    /* ConfigIntOC6  */
void ConfigIntOC6(unsigned int config) __attribute__ ((section
(".libperi")));

    /* ConfigIntOC7  */
void ConfigIntOC7(unsigned int config) __attribute__ ((section
(".libperi")));

    /* ConfigIntOC8  */
void ConfigIntOC8(unsigned int config) __attribute__ ((section
(".libperi")));

    /*  ReadDCOC5PWM    */
unsigned int ReadDCOC5PWM() __attribute__ ((section (".libperi")));

    /*  ReadDCOC6PWM    */
unsigned int ReadDCOC6PWM() __attribute__ ((section (".libperi")));

    /*  ReadDCOC7PWM    */
unsigned int ReadDCOC7PWM() __attribute__ ((section (".libperi")));

    /*  ReadDCOC8PWM    */
unsigned int ReadDCOC8PWM() __attribute__ ((section (".libperi")));

    /*  ReadRegOC if Reg is Start reads Secondary Register and if Reg
    is  Stop read  Main Register    */
unsigned int ReadRegOC5(char reg) __attribute__ ((section
(".libperi")));

    /*  ReadRegOC if Reg is Start reads Secondary Register and if Reg
    is  Stop read  Main Register    */
unsigned int ReadRegOC6(char reg) __attribute__ ((section
(".libperi")));

    /*  ReadRegOC if Reg is Start reads Secondary Register and if Reg
    is  Stop read  Main Register    */
unsigned int ReadRegOC7(char reg) __attribute__ ((section
(".libperi")));

    /*  ReadRegOC if Reg is Start reads Secondary Register and if Reg
    is  Stop read  Main Register    */
unsigned int ReadRegOC8(char reg) __attribute__ ((section
(".libperi")));

    /*  OpenOC5   */
void OpenOC5(unsigned int config,unsigned int value1,
                  unsigned int value2) __attribute__ ((section(".libperi")));
    /*  OpenOC6   */
void OpenOC6(unsigned int config,unsigned int value1,
                  unsigned int value2) __attribute__ ((section(".libperi")));
    /*  OpenOC7   */
void OpenOC7(unsigned int config,unsigned int value1,
                 unsigned int value2) __attribute__ ((section
(".libperi")));
    /*  OpenOC8   */
void OpenOC8(unsigned int config,unsigned int value1,
                 unsigned int value2) __attribute__ ((section
(".libperi")));

    /*  SetDCOC5PWM  */
void SetDCOC5PWM(unsigned int dutycycle) __attribute__ ((section
(".libperi")));

    /*  SetDCOC6PWM  */
void SetDCOC6PWM(unsigned int dutycycle) __attribute__ ((section
(".libperi")));

    /*  SetDCOC7PWM  */
void SetDCOC7PWM(unsigned int dutycycle) __attribute__ ((section
(".libperi")));

    /*  SetDCOC8PWM  */
void SetDCOC8PWM(unsigned int dutycycle) __attribute__ ((section
(".libperi")));

    /*  SetPulseOC5    */
void SetPulseOC5(unsigned int pulse_start,unsigned int pulse_stop) __attribute__ 
((section (".libperi")));

    /*  SetPulseOC6    */
void SetPulseOC6(unsigned int pulse_start,unsigned int pulse_stop) __attribute__ 
((section (".libperi")));
    /*  SetPulseOC7    */
void SetPulseOC7(unsigned int pulse_start,unsigned int pulse_stop) __attribute__ 
((section (".libperi")));
    /*  SetPulseOC8    */
void SetPulseOC8(unsigned int pulse_start,unsigned int pulse_stop) __attribute__ 
((section (".libperi")));
#endif 

#endif

