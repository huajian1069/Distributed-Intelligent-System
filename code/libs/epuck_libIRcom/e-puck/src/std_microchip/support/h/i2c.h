/*********************************************************************/
/*		Header File for I2C module Library routines          */
/*********************************************************************/

#ifndef I2C_H
#define I2C_H

/* List of SFRs for I2C module */
/* This list contains the SFRs with default (POR) values to be used for configuring I2C module */
/* The user can modify this based on the requirement */
#define I2CRCV_VALUE               0x0000
#define I2CTRN_VALUE               0x00FF
#define I2CBRG_VALUE               0x0000
#define I2CCON_VALUE               0x0000
#define I2CSTAT_VALUE              0x0000
#define I2CADD_VALUE               0x0000

/* I2CCON register Configuration bit definitions */

#define I2C_ON	                   0xFFFF	/*I2C module enabled */ 
#define I2C_OFF	                   0x7FFF	/*I2C module disabled */ 

#define I2C_IDLE_STOP              0xFFFF	/*stop I2C module in Idle mode */
#define I2C_IDLE_CON               0xDFFF	/*continue I2C module in Idle mode */

#define I2C_CLK_REL                0xFFFF	/*release clock */ 
#define I2C_CLK_HLD                0xEFFF	/*hold clock  */

#define I2C_IPMI_EN                0xFFFF	/*IPMI mode enabled */
#define I2C_IPMI_DIS               0xF7FF	/*IPMI mode not enabled */

#define I2C_10BIT_ADD              0xFFFF	/*I2CADD is 10-bit address */
#define I2C_7BIT_ADD               0xFBFF	/*I2CADD is 7-bit address */

#define I2C_SLW_DIS                0xFFFF	/*Disable Slew Rate Control for 100KHz */
#define I2C_SLW_EN                 0xFDFF	/*Enable Slew Rate Control for 400KHz */

#define I2C_SM_EN                  0xFFFF	/*Enable SM bus specification */
#define I2C_SM_DIS                 0xFEFF	/*Disable SM bus specification */

#define I2C_GCALL_EN               0xFFFF	/*Enable Interrupt when General call address is received. */
#define I2C_GCALL_DIS              0xFF7F	/*Disable General call address. */

#define I2C_STR_EN                 0xFFFF	/*Enable clock stretching */
#define I2C_STR_DIS                0xFFBF	/*disable clock stretching */

#define I2C_ACK                    0xFFDF	/*Transmit 0 to send ACK as acknowledge */
#define I2C_NACK                   0xFFFF	/*Transmit 1 to send NACK as acknowledge*/

#define I2C_ACK_EN                 0xFFFF	/*Initiate Acknowledge sequence */
#define I2C_ACK_DIS                0xFFEF	/*Acknowledge condition Idle */

#define I2C_RCV_EN                 0xFFFF	/*Enable receive mode */
#define I2C_RCV_DIS                0xFFF7	/*Receive sequence not in progress */

#define I2C_STOP_EN                0xFFFF	/*Initiate Stop sequence */
#define I2C_STOP_DIS               0xFFFB	/*Stop condition Idle */

#define I2C_RESTART_EN             0xFFFF	/*Initiate Restart sequence */
#define I2C_RESTART_DIS            0xFFFD	/*Start condition Idle */

#define I2C_START_EN               0xFFFF	/*Initiate Start sequence */
#define I2C_START_DIS              0xFFFE	/*Start condition Idle */


/* Priority for Slave I2C Interrupt */

#define SI2C_INT_PRI_7             0xFFFF
#define SI2C_INT_PRI_6             0xFFFE
#define SI2C_INT_PRI_5             0xFFFD
#define SI2C_INT_PRI_4             0xFFFC
#define SI2C_INT_PRI_3             0xFFFB
#define SI2C_INT_PRI_2             0xFFFA
#define SI2C_INT_PRI_1             0xFFF9
#define SI2C_INT_PRI_0             0xFFF8

/* Slave I2C Interrupt Enable/Disable */

#define SI2C_INT_ON                0xFFFF
#define SI2C_INT_OFF               0xFFF7

/* Priority for Master I2C Interrupt */

#define MI2C_INT_PRI_7             0xFFFF
#define MI2C_INT_PRI_6             0xFFEF
#define MI2C_INT_PRI_5             0xFFDF
#define MI2C_INT_PRI_4             0xFFCF
#define MI2C_INT_PRI_3             0xFFBF
#define MI2C_INT_PRI_2             0xFFAF
#define MI2C_INT_PRI_1             0xFF9F
#define MI2C_INT_PRI_0             0xFF8F

/* Master I2C Interrupt Enable/Disable */

#define MI2C_INT_ON                0xFFFF
#define MI2C_INT_OFF               0xFF7F

/* Macros to  Enable/Disable interrupts and set Interrupt priority of SI2C module*/
#define EnableIntSI2C                    asm("BSET IEC0,#13")
#define DisableIntSI2C                   asm("BCLR IEC0,#13")
#define SetPriorityIntSI2C(priority)     (IPC3bits.SI2CIP = priority)

/* Macros to  Enable/Disable interrupts and set Interrupt priority of MI2C module*/
#define EnableIntMI2C                    asm("BSET IEC0,#14")
#define DisableIntMI2C                   asm("BCLR IEC0,#14")
#define SetPriorityIntMI2C(priority)     (IPC3bits.MI2CIP = priority)


/* I2C module Converter Function Prototypes */

void AckI2C(void) __attribute__ ((section (".libperi")));

void CloseI2C(void) __attribute__ ((section (".libperi")));

void ConfigIntI2C(unsigned int) __attribute__ ((section (".libperi")));

char DataRdyI2C(void) __attribute__ ((section (".libperi")));

#define SlavegetcI2C SlaveReadI2C

#define MastergetcI2C MasterReadI2C

unsigned int SlavegetsI2C(unsigned char *, unsigned int) __attribute__ ((section (".libperi")));

unsigned int MastergetsI2C(unsigned int length, unsigned char * rdptr, unsigned int i2c_data_wait) __attribute__ ((section (".libperi")));

void IdleI2C(void) __attribute__ ((section (".libperi")));

void NotAckI2C(void) __attribute__ ((section (".libperi")));

void OpenI2C(unsigned int,unsigned int) __attribute__ ((section (".libperi")));

unsigned int MasterputsI2C(unsigned char *) __attribute__ ((section (".libperi")));

unsigned int SlaveputsI2C(unsigned char *) __attribute__ ((section (".libperi")));

#define SlaveputcI2C SlaveWriteI2C

#define MasterputcI2C MasterWriteI2C

unsigned char SlaveReadI2C(void) __attribute__ ((section (".libperi")));

unsigned char MasterReadI2C(void) __attribute__ ((section (".libperi")));

void SlaveWriteI2C(unsigned char) __attribute__ ((section (".libperi")));

char MasterWriteI2C(unsigned char) __attribute__ ((section (".libperi")));

void RestartI2C(void) __attribute__ ((section (".libperi")));

void StartI2C(void) __attribute__ ((section (".libperi")));

void StopI2C(void) __attribute__ ((section (".libperi")));

#endif	/*I2C_H  */
