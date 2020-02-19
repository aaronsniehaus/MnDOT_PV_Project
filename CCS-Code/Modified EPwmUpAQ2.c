//
//  MnDOT Research Project, SOPI Solar Panel Inverter
//
//  Overview of Functions:
//
//  EPWM 1..........................Boost Stage
//  EPWM 2 & 3......................Resonant Inverter Stage
//  EPWM 4..........................Half Wave Rectifier Stage
//  EPWM 5 & 6......................Grid Inverter Stage
//  TZ1  (2 Pin Header).............Input Sync
//  GPIO (2 Pin Header).............Output for Sync
//

//  Timestamp[x...] Descriptions
//---------------------------------------------------------------------------------------------------
//  0 - Sync Signal             1 - Begin Pulse             2 - End Pulse               3 - Undefined
//---------------------------------------------------------------------------------------------------
//  4 - Undefined               5 - Undefined               6 - Undefined               7 - Undefined
//---------------------------------------------------------------------------------------------------
//  8 - Undefined               9 - Undefined               10 - Undefined              11 - Undefined
//---------------------------------------------------------------------------------------------------
//  12 - Undefined              13 - Undefined              14 - Undefined              15 - Undefined


//
//  Note: Some of this code is residual from the original EPwmUpAQ Example.
//  If it's not clear why something is there, it might be unnecessary.
//

//  Contributors: Aaron Niehaus, MaoHang Qiu





//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Math.h"
#include <stdio.h>
//
// Typedefs
//
typedef struct
{
    volatile struct EPWM_REGS *EPwmRegHandle;
    Uint16 EPwm_CMPA_Direction;
    Uint16 EPwm_CMPB_Direction;
    Uint16 EPwmTimerIntCount;
    Uint16 EPwmMaxCMPA;
    Uint16 EPwmMinCMPA;
    Uint16 EPwmMaxCMPB;
    Uint16 EPwmMinCMPB;
} EPWM_INFO;

//
// Function Prototypes
//
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
void InitEPwm4Example(void);
void InitEPwm5Example(void);
void InitEPwm6Example(void);
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void epwm4_isr(void);
__interrupt void epwm5_isr(void);
__interrupt void epwm6_isr(void);
__interrupt void epwm1_tzint_isr(void);
__interrupt void adc_isr(void);
void Arbitrate(void);
void Pulse(void);
void Pulse2(void);
void DelayLoop(int k);

//
// Globals
//
EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm3_info;
EPWM_INFO epwm4_info;
EPWM_INFO epwm5_info;
EPWM_INFO epwm6_info;
Uint16 LoopCount;
Uint16 ConversionCount;
Uint16 Voltage1[10];
Uint16 Voltage2[10];

//
// Defines that configure the period for each timer
// All periods are set by a single period variable, EPWM_PERIOD
//

#define EPWM_PERIOD 150                                 //Set period to 150 interrupts ~ 250 kHz

#define EPWM1_TIMER_TBPRD  EPWM_PERIOD                  // Period register
#define EPWM1_MAX_CMPA     EPWM_PERIOD
#define EPWM1_MIN_CMPA     0
#define EPWM1_MAX_CMPB     EPWM_PERIOD
#define EPWM1_MIN_CMPB     0

#define EPWM2_TIMER_TBPRD  EPWM_PERIOD                  // Period register
#define EPWM2_MAX_CMPA     EPWM_PERIOD                  // Set Max to the total period
#define EPWM2_MIN_CMPA     0                            // Set Min to zero
#define EPWM2_MAX_CMPB     EPWM_PERIOD                  //
#define EPWM2_MIN_CMPB     0                            //

#define EPWM3_TIMER_TBPRD  EPWM_PERIOD                  // Period register
#define EPWM3_MAX_CMPA     EPWM_PERIOD                  // Set Max to the total period
#define EPWM3_MIN_CMPA     0                            // Set Min to zero
#define EPWM3_MAX_CMPB     EPWM_PERIOD                  //
#define EPWM3_MIN_CMPB     0                            //

#define EPWM4_TIMER_TBPRD  EPWM_PERIOD                  // Period register
#define EPWM4_MAX_CMPA     EPWM_PERIOD                  // Set Max to the total period
#define EPWM4_MIN_CMPA     0                            // Set Min to zero
#define EPWM4_MAX_CMPB     EPWM_PERIOD                  //
#define EPWM4_MIN_CMPB     0                            //

#define EPWM5_TIMER_TBPRD  EPWM_PERIOD                  // Period register
#define EPWM5_MAX_CMPA     EPWM_PERIOD                  // Set Max to the total period
#define EPWM5_MIN_CMPA     0                            // Set Min to zero
#define EPWM5_MAX_CMPB     EPWM_PERIOD                  //
#define EPWM5_MIN_CMPB     0                            //
#define EPWM5_HALF_CMPA    EPWM5_MAX_CMPA/2             // Division takes a long time in an interrupt. Calculate beforehand
#define EPWM5_HALF_CMPB    EPWM6_MAX_CMPB/2             //

#define EPWM6_TIMER_TBPRD  EPWM_PERIOD                  // Period register
#define EPWM6_MAX_CMPA     EPWM_PERIOD                  // Set Max to the total period
#define EPWM6_MIN_CMPA     0                            // Set Min to zero
#define EPWM6_MAX_CMPB     EPWM_PERIOD                  //
#define EPWM6_MIN_CMPB     0                            //
#define EPWM6_HALF_CMPA    EPWM6_MAX_CMPA/2             // Division takes a long time in an interrupt. Calculate beforehand
#define EPWM6_HALF_CMPB    EPWM5_MAX_CMPB/2             //

//
//Sine Function Variables
//
float PI = 3.14159265;
long int N = 0;
volatile long i;
float SineFactor = (1/(4.57*(EPWM_PERIOD)));            //Calculate the factor for the 60 Hz sine argument to avoid division ops in the interrupt

int TEST_VARIABLE;
int TEST_MATRIX[2000];



//
//Sync Variables
//
long int    EPwm1TZIntCount;
int         MasterSlave = 1;                            //Keeps track of whether the controller is in master or slave mode. 1 = Master, 0 = Slave
double long Nanoseconds = 0;                            //Global time variable, time in nanoseconds (approximate)
int         NanosecIncrement = 27.3*EPWM_PERIOD;        //Increment on an interrupt to give us accurate timing
double long TimeStamp[16] = {0,0,0,0};                      //Timestamps. A description can be found in the comments at the top of the document.


//
// Defines that keep track of which way the compare value is moving
//
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0


//
// Defines for the Maximum Dead Band values
//
#define EPWM1_MAX_DB   0x03FF
#define EPWM2_MAX_DB   0x03FF
#define EPWM3_MAX_DB   0x03FF
#define EPWM4_MAX_DB   0x03FF
#define EPWM5_MAX_DB   0x03FF
#define EPWM6_MAX_DB   0x03FF

#define EPWM1_MIN_DB   0
#define EPWM2_MIN_DB   0
#define EPWM3_MIN_DB   0
#define EPWM4_MIN_DB   0
#define EPWM5_MIN_DB   0
#define EPWM6_MIN_DB   0



//
// Main
//
void main(void)
{
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the DSP2833x_SysCtrl.c file.
    //
    InitSysCtrl();

    EALLOW;
            #if (CPU_FRQ_150MHZ)     // Default - 150 MHz SYSCLKOUT
                //
                // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
                //
                #define ADC_MODCLK 0x3
            #endif
            #if (CPU_FRQ_100MHZ)
                //
                // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2)   = 25.0 MHz
                //
                #define ADC_MODCLK 0x2
            #endif
            EDIS;

            EALLOW;
                        SysCtrlRegs.HISPCP.all = ADC_MODCLK;
                        PieVectTable.ADCINT = &adc_isr;
                        EDIS;

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the DSP2833x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example

    //
    // Init EPwm1,2,3,4,5,6 and Tz GPIO
    // These functions are in the DSP2833x_EPwm.c file
    //
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitTzGpio();
    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2833x_PieCtrl.c file.
    //
    InitPieCtrl();
    //
            // Disable CPU interrupts and clear all CPU interrupt flags:
            //
            IER = 0x0000;
            IFR = 0x0000;


    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in DSP2833x_DefaultIsr.c.
    // This function is found in DSP2833x_PieVect.c.
    //
    InitPieVectTable();



    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;  // This is needed to write to EALLOW protected registers

    //
    // Allow interrupts for all 6 EPWM
    //
    PieVectTable.EPWM1_INT = &epwm1_isr;
    PieVectTable.EPWM2_INT = &epwm2_isr;
    PieVectTable.EPWM3_INT = &epwm3_isr;
    PieVectTable.EPWM4_INT = &epwm4_isr;
    PieVectTable.EPWM5_INT = &epwm5_isr;
    PieVectTable.EPWM6_INT = &epwm6_isr;

    //  Enable TZ1 Interrupt (PHASE_SYNC_IN)
    PieVectTable.EPWM1_TZINT = &epwm1_tzint_isr;

    //  Enable GPC Multiplexer, containing GPIO84 (PHASE_SYNC_OUT)
    //  In the future, this pin may be changed, and the mux group will need to be changed
    //  Documentation on this MUX group can be found in "TMS320x, 2823x System Control and Interrupts PDF."
    GpioCtrlRegs.GPCMUX2.all    = 0x00000000;
    GpioCtrlRegs.GPCDIR.all     = 0xFFFFFFFF;
    GpioDataRegs.GPCDAT.bit.GPIO84    =1;       //Set PHASE_SYNC_OUT to high by default.

    EDIS;    // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in DSP2833x_InitPeripherals.c
    //
    // InitPeripherals();

    //
    // For this example, only initialize the ePWM
    //
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPwm1Example();
    InitEPwm2Example();
    InitEPwm3Example();
    InitEPwm4Example();
    InitEPwm5Example();
    InitEPwm6Example();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    //
    // Step 5. User specific code, enable interrupts
    //

    //
    // Enable CPU INT3 which is connected to EPWM1-3 INT
    //
    IER |= M_INT3;
    IER |= M_INT2;

    //
    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-6
    //
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx4 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx5 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx6 = 1;
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;

    EINT;               // Enable Global interrupt INTM
    ERTM;               // Enable Global realtime interrupt DBGM

//    for(;;)             // Loop forever
//        {
//
//}

    InitAdc();
    //
    // Enable ADCINT in PIE
    //
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    IER |= M_INT1;      // Enable CPU Interrupt 1
    EINT;               // Enable Global interrupt INTM
    ERTM;               // Enable Global realtime interrupt DBGM

    LoopCount = 0;
    ConversionCount = 0;

    //
    // Configure ADC
    //
    AdcRegs.ADCMAXCONV.all = 0x0001;       // Setup 2 conv's on SEQ1
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x3; // Setup ADCINA3 as 1st SEQ1 conv.
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x2; // Setup ADCINA2 as 2nd SEQ1 conv.

    //
    // Enable SOCA from ePWM to start SEQ1
    //
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;

    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // Enable SEQ1 interrupt (every EOS)

    //
    // Assumes ePWM1 clock is already enabled in InitSysCtrl();
    //
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;     // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;    // Select SOC from from CPMA on upcount
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event
    EPwm1Regs.CMPA.half.CMPA = 0x0080;  // Set compare A value
    EPwm1Regs.TBPRD = 0xFFFF;           // Set period for ePWM1
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;    // count up and start

    //
    // Wait for ADC interrupt
    //
    for(;;)
    {
    }
}

__interrupt void 
epwm1_isr(void)
{
    //
    // Clear INT flag for this timer
    //
    EPwm1Regs.ETCLR.bit.INT = 1;

    Nanoseconds = Nanoseconds+((27*EPWM_PERIOD)-(3*EPWM_PERIOD)/10);    //Keeps track of number of nanoseconds past.
    Arbitrate();                                        //Check Master or Slave.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


__interrupt void 
epwm2_isr(void)
{
    EPwm2Regs.CMPA.half.CMPA =  EPWM2_TIMER_TBPRD/2;;
    EPwm2Regs.CMPB =            EPWM2_TIMER_TBPRD/2;;

    //
    // Clear INT flag for this timer
    //
    EPwm2Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void 
epwm3_isr(void)
{
    //
    // Clear INT flag for this timer
    //
    EPwm3Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void
epwm4_isr(void)
{
    //
    // Clear INT flag for this timer
    //
    EPwm4Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void
epwm5_isr(void)
{
    // Clear INT flag for this timer
    EPwm5Regs.ETCLR.bit.INT = 1;

    //Generate 60 Hz Sine Wave with argument of N, increment N

    EPwm5Regs.CMPA.half.CMPA =  EPWM5_HALF_CMPA + EPWM5_HALF_CMPA*sin(PI*N*SineFactor);
    EPwm5Regs.CMPB =            EPWM5_HALF_CMPB + EPWM5_HALF_CMPB*sin(PI*N*SineFactor);
    EPwm6Regs.CMPA.half.CMPA =  EPWM6_HALF_CMPA + EPWM6_HALF_CMPA*sin(PI*N*SineFactor);
    EPwm6Regs.CMPB =            EPWM6_HALF_CMPB + EPWM6_HALF_CMPB*sin(PI*N*SineFactor);
    N++;

    //Reset N when the input argument is 2pi to prevent overflow
    if ((N*PI*SineFactor) >= 2*PI ){
        N = 0;
        Pulse2();
        TimeStamp[1] = Nanoseconds;
        }

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void
epwm6_isr(void)
{
    // Clear INT flag for this timer
    EPwm6Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


    //
    //epwm1_tzint_isr is used to reset the phase of the sine wave when the sync signal comes in.
    //

__interrupt void
epwm1_tzint_isr(void)
{
    N = 0;                          //Set phase of the sine wave to zero
    TimeStamp[0] = Nanoseconds;     //Update Timestamp for Master / Slave Mode
    MasterSlave = 0;

    //
    // Leave these flags set so we only take this
    // interrupt once
    //
    EALLOW;
    EPwm1Regs.TZCLR.bit.CBC = 1;
    EPwm1Regs.TZCLR.bit.INT = 1;
    EDIS;
    EPwm1TZIntCount++;

    if (Nanoseconds - TimeStamp[1] >= 500){
        TimeStamp[1] = Nanoseconds;
        Pulse2();
            }

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}

//
// adc_isr -
//
__interrupt void
adc_isr(void)
{
    Voltage1[ConversionCount] = AdcRegs.ADCRESULT0 >>4;
    Voltage2[ConversionCount] = AdcRegs.ADCRESULT1 >>4;

    //
    // If 40 conversions have been logged, start over
    //
    if(ConversionCount == 9)
    {
        ConversionCount = 0;
    }
    else
    {
        ConversionCount++;
    }

    //
    // Reinitialize for next ADC sequence
    //
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

    return;
}

//
// InitEPwm1Example -
//
void 
InitEPwm1Example(void)
{
    //
    // Enable TZ1 as cycle by cycle trip source
    //
    EALLOW;
    EPwm1Regs.TZSEL.bit.CBC1 = 1;

    // Enable TZ interrupt
    EPwm1Regs.TZEINT.bit.CBC = 1;
    EDIS;

    // Setup TBCLK
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;       // Set timer period
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    // Setup shadow register load on ZERO
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Set CMPA and CMPB to give a 50% Duty Cycle
    EPwm1Regs.CMPA.half.CMPA =  EPWM1_TIMER_TBPRD/2;
    EPwm1Regs.CMPB =            EPWM1_TIMER_TBPRD/2;


    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;      // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;    // Clear PWM1A on event A, up count

    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;      // Set PWM1B on Zero
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;    // Clear PWM1B on event B, up count


    //
    //  Complimentary Pair
    //
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;          //Turn on DB Module
    EPwm1Regs.DBRED = EPWM1_MIN_DB;                 //Set Minimum for Deadband
    EPwm1Regs.DBFED = EPWM1_MIN_DB;                 //Set Minimum for Deadband
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  //Enable All switches
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       //Active High, A&B are complimentary

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event
    
    //
    // Start by increasing CMPA & CMPB
    //
    epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;

    epwm1_info.EPwmTimerIntCount = 0;           // Zero the interrupt counter
    epwm1_info.EPwmRegHandle = &EPwm1Regs;      //Set the pointer to the ePWM module
    epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;    // Setup min/max CMPA/CMPB values
    epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
    epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
    epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
}

//
// InitEPwm2Example - 
//
void 
InitEPwm2Example(void)
{
    //
    // Setup TBCLK
    //
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD;       // Set timer period
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm2Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    //
    // Setup shadow register load on ZERO
    //
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Set CMPA and CMPB to give a 50% Duty Cycle
    EPwm2Regs.CMPA.half.CMPA =  EPWM2_TIMER_TBPRD/2;
    EPwm2Regs.CMPB =            EPWM2_TIMER_TBPRD/2;

    //
    // Set actions
    //
    EPwm2Regs.AQCTLA.bit.PRD = AQ_CLEAR;      // Clear PWM2A on Period
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;        // Set PWM2A on event A, up count

    EPwm2Regs.AQCTLB.bit.PRD = AQ_CLEAR;      // Clear PWM2B on Period
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;        // Set PWM2B on event B, up count

    //
    //  Complimentary Pair
    //
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;          //Turn on DB Module
    EPwm2Regs.DBRED = EPWM2_MIN_DB;                 //Set Minimum for Deadband
    EPwm2Regs.DBFED = EPWM2_MIN_DB;                 //Set Minimum for Deadband
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  //Enable All switches
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       //Active High, A&B are complimentary

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 1;            // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;       // Generate INT on 3rd event

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA
    epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;  // and decreasing CMPB
    epwm2_info.EPwmTimerIntCount = 0;         // Zero the interrupt counter
    epwm2_info.EPwmRegHandle = &EPwm2Regs; //Set the pointer to the ePWM module
    epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA;  // Setup min/max CMPA/CMPB values
    epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
    epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
    epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;
}

//
// InitEPwm3Example -
//
void 
InitEPwm3Example(void)
{
    //
    // Setup TBCLK
    //
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm3Regs.TBPRD = EPWM3_TIMER_TBPRD;       // Set timer period
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm3Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    //
    // Setup shadow register load on ZERO
    //
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Set CMPA and CMPB to give a 50% Duty Cycle
    EPwm3Regs.CMPA.half.CMPA =  EPWM3_TIMER_TBPRD/2;
    EPwm3Regs.CMPB =            EPWM3_TIMER_TBPRD/2;

    //
    // Set actions
    //
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;      // Set PWM3A on Zero
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;    // Clear PWM3A on event A, up count

    EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;      // Set PWM3B on Zero
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;    // Clear PWM3B on event B, up count


    //
    //  Complimentary Pair
    //
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;          //Turn on DB Module
    EPwm3Regs.DBRED = EPWM3_MIN_DB;                 //Set Minimum for Deadband
    EPwm3Regs.DBFED = EPWM3_MIN_DB;                 //Set Minimum for Deadband
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  //Enable All switches
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       //Active High, A&B are complimentary

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    //
    // Start by increasing CMPA & CMPB
    //
    epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;

    epwm3_info.EPwmTimerIntCount = 0;      // Zero the interrupt counter
    epwm3_info.EPwmRegHandle = &EPwm3Regs; //Set the pointer to the ePWM module
    epwm3_info.EPwmMaxCMPA = EPWM3_MAX_CMPA;  // Setup min/max CMPA/CMPB values
    epwm3_info.EPwmMinCMPA = EPWM3_MIN_CMPA;
    epwm3_info.EPwmMaxCMPB = EPWM3_MAX_CMPB;
    epwm3_info.EPwmMinCMPB = EPWM3_MIN_CMPB;
}

//
// InitEPwm4Example -
//
void
InitEPwm4Example(void)
{
    //
    // Setup TBCLK
    //
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm4Regs.TBPRD = EPWM4_TIMER_TBPRD;       // Set timer period
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm4Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm4Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    //
    // Setup shadow register load on ZERO
    //
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Set CMPA and CMPB to give a 50% Duty Cycle
    EPwm4Regs.CMPA.half.CMPA =  EPWM4_TIMER_TBPRD/2;
    EPwm4Regs.CMPB =            EPWM4_TIMER_TBPRD/2;

    //
    // Set actions
    //
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;      // Set PWM4A on Zero
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;    // Clear PWM4A on event A, up count

    EPwm4Regs.AQCTLB.bit.ZRO = AQ_SET;      // Set PWM4B on Zero
    EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;    // Clear PWM4B on event B, up count


    //
    //  Complimentary Pair
    //
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;          //Turn on DB Module
    EPwm4Regs.DBRED = EPWM4_MIN_DB;                 //Set Minimum for Deadband
    EPwm4Regs.DBFED = EPWM4_MIN_DB;                 //Set Minimum for Deadband
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  //Enable All switches
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;       //Active High, A&B are complimentary

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm4Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm4Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    //
    // Start by increasing CMPA & CMPB
    //
    epwm4_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    epwm4_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;

    epwm4_info.EPwmTimerIntCount = 0;      // Zero the interrupt counter
    epwm4_info.EPwmRegHandle = &EPwm4Regs; //Set the pointer to the ePWM module
    epwm4_info.EPwmMaxCMPA = EPWM4_MAX_CMPA;  // Setup min/max CMPA/CMPB values
    epwm4_info.EPwmMinCMPA = EPWM4_MIN_CMPA;
    epwm4_info.EPwmMaxCMPB = EPWM4_MAX_CMPB;
    epwm4_info.EPwmMinCMPB = EPWM4_MIN_CMPB;
}

void
InitEPwm5Example(void)
{
    //
    // Enable TZ1 as cycle by cycle trip source
    //
    EALLOW;
    EPwm5Regs.TZSEL.bit.CBC1 = 1;
    EPwm5Regs.TZCTL.bit.TZA = TZ_NO_CHANGE;
    EPwm5Regs.TZCTL.bit.TZB = TZ_NO_CHANGE;

    //
    // Enable TZ interrupt
    //
    EPwm5Regs.TZEINT.bit.CBC = 1;
    EDIS;

    //
    // Setup TBCLK
    //
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm5Regs.TBPRD = EPWM5_TIMER_TBPRD;       // Set timer period
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm5Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm5Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    //
    // Setup shadow register load on ZERO
    //
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Set CMPA and CMPB to give a 50% Duty Cycle
    EPwm5Regs.CMPA.half.CMPA =  EPWM5_TIMER_TBPRD/2;
    EPwm5Regs.CMPB =            EPWM5_TIMER_TBPRD/2;

    //
    // Set actions
    //
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;      // Set PWM5A on Zero
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;    // Clear PWM5A on event A, up count

    EPwm5Regs.AQCTLB.bit.ZRO = AQ_SET;      // Set PWM5B on Zero
    EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;    // Clear PWM5B on event B, up count


    //
    //  Complimentary Pair
    //
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;          //Turn on DB Module
    EPwm5Regs.DBRED = EPWM5_MIN_DB;                 //Set Minimum for Deadband
    EPwm5Regs.DBFED = EPWM5_MIN_DB;                 //Set Minimum for Deadband
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  //Enable All switches
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       //Active High, A&B are complimentary

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm5Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm5Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    epwm5_info.EPwmTimerIntCount = 0;           // Zero the interrupt counter
    epwm5_info.EPwmRegHandle = &EPwm5Regs;      //Set the pointer to the ePWM module
    epwm5_info.EPwmMaxCMPA = EPWM5_MAX_CMPA;    // Setup min/max CMPA/CMPB values
    epwm5_info.EPwmMinCMPA = EPWM5_MIN_CMPA;
    epwm5_info.EPwmMaxCMPB = EPWM5_MAX_CMPB;
    epwm5_info.EPwmMinCMPB = EPWM5_MIN_CMPB;
}

void
InitEPwm6Example(void)
{
    //
    // Enable TZ1 as cycle by cycle trip source
    //
    EALLOW;
    EPwm6Regs.TZSEL.bit.CBC1 = 1;

    //
    // Enable TZ interrupt
    //
    EPwm6Regs.TZEINT.bit.CBC = 1;
    EDIS;

    //
    // Setup TBCLK
    //
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm6Regs.TBPRD = EPWM6_TIMER_TBPRD;       // Set timer period
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm6Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    //
    // Setup shadow register load on ZERO
    //
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //Set CMPA and CMPB to give a 50% Duty Cycle
    EPwm6Regs.CMPA.half.CMPA =  EPWM6_TIMER_TBPRD/2;
    EPwm6Regs.CMPB =            EPWM6_TIMER_TBPRD/2;


    //
    // Set actions
    //
    EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;      // Set PWM6A on Zero
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;    // Clear PWM6A on event A, up count

    EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;      // Set PWM6B on Zero
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;    // Clear PWM6B on event B, up count


    //
    //  Complimentary Pair
    //
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;          //Turn on DB Module
    EPwm6Regs.DBRED = EPWM6_MIN_DB;                 //Set Minimum for Deadband
    EPwm6Regs.DBFED = EPWM6_MIN_DB;                 //Set Minimum for Deadband
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  //Enable All switches
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;       //Active High, A&B are complimentary

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm6Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm6Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    //
    // Start by increasing CMPA & CMPB
    //
    //epwm6_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    //epwm6_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;

    epwm6_info.EPwmTimerIntCount = 0;      // Zero the interrupt counter
    epwm6_info.EPwmRegHandle = &EPwm6Regs; //Set the pointer to the ePWM module
    epwm6_info.EPwmMaxCMPA = EPWM6_MAX_CMPA;  // Setup min/max CMPA/CMPB values
    epwm6_info.EPwmMinCMPA = EPWM6_MIN_CMPA;
    epwm6_info.EPwmMaxCMPB = EPWM6_MAX_CMPB;
    epwm6_info.EPwmMinCMPB = EPWM6_MIN_CMPB;
}

//  Arbitration keeps track of whether the module is a master or slave.
//  If the module does not receive a sync signal for 3 continuous powerline cycles (50 ms), turn on master.
//  Timestamp for the last synchronization signal is stored in TimeStamp[0].

void
Arbitrate(){
    if (Nanoseconds - TimeStamp[0] >= 50000000)
    {
        MasterSlave = 1;                    //If we haven't received a signal in the last 50 ms, become master.
        }
    else {
        MasterSlave = 0;                    //Assume slave mode.
    }
}

//
//Pulse() Generates a (roughly) 10 us pulse for synchronization. It must be looped to run correctly.
//
void
Pulse(){
    if (GpioDataRegs.GPCDAT.bit.GPIO84  == 1){    //If the GPIO was high, set it low. Log this time.
        TimeStamp[2] = Nanoseconds;
        GpioDataRegs.GPCDAT.bit.GPIO84  = 0;
    }

    else if (TimeStamp[1] - Nanoseconds >= 5000){    //If 10 microseconds passed, turn the GPIO off again.
            TimeStamp[1] = Nanoseconds;             //Log this time until GPIO
            GpioDataRegs.GPCDAT.bit.GPIO84  = 1;
    }
}

void
Pulse2(){
    GpioDataRegs.GPCDAT.bit.GPIO84  = 0;
    DelayLoop(1000);        //50
    GpioDataRegs.GPCDAT.bit.GPIO84  = 1;
    TimeStamp[1] = Nanoseconds;
}

void
DelayLoop(int k){
    for ( i = 0; i < k; i++){
        }
    i = 0;
}


//
// End of File
//
