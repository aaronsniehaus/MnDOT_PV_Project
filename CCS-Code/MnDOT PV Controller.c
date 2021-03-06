//
//  MnDOT Research Project, SOPI Solar Panel Inverter
//
//  OVERVIEW OF FUNCTIONS:
//
//  EPWM 1..........................Boost Stage.                M(D)  =  1/(EPWM1B)  =  1/(1-EPWM1A)
//  EPWM 2 & 3......................Resonant Inverter Stage
//  EPWM 4..........................Half Wave Rectifier Stage
//  EPWM 5 & 6......................Grid Inverter Stage
//  TZ1  (2 Pin Header).............Input Sync
//  GPIO84 (2 Pin Header)...........Output for Sync
//

//  TimeStamp[x...] Descriptions
//---------------------------------------------------------------------------------------------------
//  0 - Sync Signal             1 - Begin Pulse             2 - End Pulse               3 - Boost Soft Start Time Increment
//---------------------------------------------------------------------------------------------------
//  4 - Generic Fault Event     5 - Undefined               6 - Undefined               7 - Undefined
//---------------------------------------------------------------------------------------------------
//  8 - Undefined               9 - Undefined               10 - Undefined              11 - Undefined
//---------------------------------------------------------------------------------------------------
//  12 - Undefined              13 - Undefined              14 - Undefined              15 - Fault Event Clear


//  Corresponding ADC Values...
//----------------------------------------------------------------------------------------------------
//  ADC0 - PV Voltage           ADC1 - VBus 1               ADC2 - None                 ADC3 - VBus 2
//----------------------------------------------------------------------------------------------------
//  ADC4 - Vout+                ADC5 - Iout                 ADC6 - PV Current           ADC7 - None
//

//
//  Control System Parameters
//---------------------------------------------------------------------------------------------------------------------------------
//  |TargetDutyCycle[x]... Target Duty Cycle for parameter x. This allows to increment continuously in a linear soft start fashion.
//  |------------------------------------------------------------------------------------------------------------------------------
//  |TargetDutyCycle[0]... Boost, DC Bus 1
//  |    """        [1]...
//  |    """        [2]...
//  |    """        [3]...
//  |    """        [4]...
//  |    """        [5]...
//
//
//

//
//  Note: Some of this code is residual from the original EPwmUpAQ Example.
//  If it's not clear why something is there, it might be unnecessary.
//

//  Contributors: Aaron Niehaus, MaoHang Qiu, Garret Hoff
//
//
//  Disclaimer: This code is for use with a college research project and is not intended for modification.
//              This text file and accompanying documentation are provided as is, with no intent for sale or implied warranty.
//


//
//  Meat and Potatoes:
//

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
    Uint16 EPwmTimerIntCount;
} EPWM_INFO;

//
// Globals
//
#define EPWM_PERIOD 150                                 //Set period to 150 interrupts, 250 kHz switching frequency
EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm3_info;
EPWM_INFO epwm4_info;
EPWM_INFO epwm5_info;
EPWM_INFO epwm6_info;
Uint16 ConversionCount;


//
//  Sine Function Variables
//
double PI = 3.14159265;
long int N = 0;
volatile long i;
double SineFactor = (1/(4.57*(EPWM_PERIOD)));            //Calculate the factor for the 60 Hz sine argument to avoid division ops in the interrupts

//
//  Sync Variables
//
long int    EPwm1TZIntCount;
int         MasterSlave = 1;                            //Keeps track of whether the controller is in master or slave mode. 1 = Master, 0 = Slave
double long Nanoseconds = 0;                            //Global time variable, time in nanoseconds (approximate)
int         NanosecIncrement = 27.3*EPWM_PERIOD;        //Increment on an interrupt to give us accurate timing
double long TimeStamp[16] = {0,0,0,0};                  //Timestamps. A description can be found in the comments at the top of the document.

//
//  ADC Feedback Variables
//
const int SampleDepth = 50;                                         //Number of samples stored in ADC Matrices. Set this to however many samples you need.

const double SampleDepthQuotient = (double)1/SampleDepth;           //Calculate beforehand to avoid divisions in the interrupts
double ADCValues[2][8][SampleDepth];
double ADCVoltages[2][8][SampleDepth];
int CurrentSample[8] = {0,0,0,0,0,0,0,0};
long int ADCIntCounter = 0;
double RMSVoltage;
long int ConversionCounter = 0;
int FaultCounter = 0;


//double InputData1[SampleDepth];
double Square[SampleDepth];
int ErrorStatus = 0;

//
//  "Real" Values
//

const double Scalars[8] = {20,20,1,20,20,1,1,1};    //Scalars to convert input voltage to original value. Voltages = 20, Currents = 1(?)

double VpvRMS = 0;
double IpvRMS = 0;
double VpriRMS = 0;
double VsecRMS = 0;
double VoutRMS = 0;
double IoutRMS = 0;
double VBus1 = 0;
double VBus2 = 0;
double ADC1DC = 0;

//
//  Control System Parameters
//

float TargetDutyCycle[6] = {0.5,0,0,0,0,0};     //Assume 50% Boost Duty Cycle
int    Enable = 1;


//
// All PWM periods are defined by EPWM_PERIOD = 150, 250 kHz in this example.
//
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
// Function Prototypes
//
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void epwm4_isr(void);
__interrupt void epwm5_isr(void);
__interrupt void epwm6_isr(void);
__interrupt void epwm1_tzint_isr(void);
__interrupt void ADC_isr(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);

void InitEPwm1(void);
void InitEPwm2(void);
void InitEPwm3(void);
void InitEPwm4(void);
void InitEPwm5(void);
void InitEPwm6(void);
void CheckMaster(void);
void InitializeADC (void);
void Pulse(void);
void Pulse2(void);
void DelayLoop(int k);
double GetVoltage(double InputData);
double GetVoltageUnbiased(double InputData);
void UpdateADCMatrix();
double GetRMS(double InputData[SampleDepth]);
double GetACRMS (double InputData[SampleDepth]);
double GetAverage(double InputData[SampleDepth]);
void DisableAllSwitches(void);
void EnableAllSwitches(void);
void ShortOutput(void);
void Toggle(void);
void CheckFault(void);
void StartCpuTimers(void);
void SoftStart(void);





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

    #define CPU_FRQ_150MHZ 0x1

    #if (CPU_FRQ_150MHZ)     // Default - 150 MHz SYSCLKOUT
        //
        // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
        //
    #define ADC_MODCLK 0x3
    ConfigCpuTimer(&CpuTimer0, 150, 100000);
    ConfigCpuTimer(&CpuTimer1, 150, 100000);
    ConfigCpuTimer(&CpuTimer2, 150, 100000);
    #endif
        #if (CPU_FRQ_100MHZ)
        //
        // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2)   = 25.0 MHz
        //
    ConfigCpuTimer(&CpuTimer0, 100, 100);
    ConfigCpuTimer(&CpuTimer1, 100, 100);
    ConfigCpuTimer(&CpuTimer2, 100, 100);
        #define ADC_MODCLK 0x2
    #endif
    EDIS;

    StartCpuTimers();

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
    InitPieVectTable();
    InitAdc();


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
    PieVectTable.ADCINT = &ADC_isr;
    PieVectTable.EPWM1_TZINT = &epwm1_tzint_isr;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.XINT13 = &cpu_timer1_isr;
    PieVectTable.TINT2 = &cpu_timer2_isr;

    //  Enable GPC MUX, containing GPIO84 (PHASE_SYNC_OUT)
    //  In the future, this pin may be changed, and the MUX group will need to be changed
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
    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers (in DSP2833x_CpuTimers.h), the
    // below settings must also be updated.
    //


    //
    // For this example, only initialize the ePWM
    //
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;


    InitEPwm1();
    InitEPwm2();
    InitEPwm3();
    InitEPwm4();
    InitEPwm5();
    InitEPwm6();
    InitializeADC();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    //
    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-7
    //
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;      //EPwm
    //PieCtrlRegs.PIEIER3.bit.INTx2 = 1;      //ADC
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;      //EPwm
    //PieCtrlRegs.PIEIER3.bit.INTx4 = 1;      //
    //PieCtrlRegs.PIEIER3.bit.INTx5 = 1;      //
    //PieCtrlRegs.PIEIER3.bit.INTx6 = 1;      //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      //CpuTimers


    //
    // Enable Interrupts in PIE
    //

    IER |= M_INT1;      //ePWM
    IER |= M_INT3;      //ePWM
    IER |= M_INT2;      //ADC
    IER |= M_INT13;     //CpuTimer
    IER |= M_INT14;     //CpuTimer

    EINT;               // Enable Global interrupt INTM
    ERTM;               // Enable Global realtime interrupt DBGM

    ConversionCount = 0;

    EnableAllSwitches();
    while(1==1){

    }
}

//
//  EPWM1 ISR. Most operations are tied to the PWM1 interrupt. This can lead to glitches in the code when the instructions get too long.
//  In the future, critical timing functions should be moved to a CpuTimer0, 1, or 2 interrupt. These are more stable.
//

__interrupt void 
epwm1_isr(void)
{
    Toggle();
    //
    // Clear INT flag for this timer
    //
    EPwm1Regs.ETCLR.bit.INT = 1;

    Nanoseconds = Nanoseconds+12500;    //Nanoseconds is our global time. 12500 comes from the 12.5 us period of the wave measured on a scope.

    if (1==1){
        TargetDutyCycle[0] = 0;
    }

    UpdateADCMatrix();  //Update our ADC Values


    VpvRMS   = Scalars[0]*GetVoltage(GetAverage(ADCValues[0][0]));                   //DC Values are calculated taking the average of the bus.
    VpriRMS  = Scalars[1]*GetVoltage(GetAverage(ADCValues[0][1]));
    IpvRMS   = Scalars[2]*GetVoltage(GetAverage(ADCValues[0][2]));
    VsecRMS  = Scalars[3]*GetVoltage(GetAverage(ADCValues[0][3]));
    VoutRMS  = Scalars[4]*GetVoltageUnbiased(GetACRMS(ADCValues[0][4]));            //GetACRMS removes the DC component, so a different voltage conversion function is used.
    IoutRMS  = Scalars[5]*GetVoltageUnbiased(GetACRMS(ADCValues[0][5]));
    //IpvRMS   = Scalars[6]*GetVoltage(ADCValues[0][6][0]);


    //
    //  Disable the module if three faults happen in a row.
    //

    if (Enable == 1){
            if (Nanoseconds - TimeStamp[4] >= 50000000){    //Wait for 3 powerline cycles, 50 ms.
                if (ErrorStatus != 0){
                    InitCMP();                              //Set CMP's to their defaults.
                    EnableAllSwitches();                    //Try starting the switches again.

                    ErrorStatus = 0;                        //Clear the error status.
                    FaultCounter++;                         //Increment the fault counter.

                    if (FaultCounter >= 3){
                        Enable = 0;
                    }
                }
            }
        }

    CheckFault();


    //
    //  Generate 60 Hz sine wave on ePWM 5 & 6, use N to simulate time
    //  N should be tied to a timestamp in the future.
    //
    EPwm5Regs.CMPA.half.CMPA =  EPWM5_HALF_CMPA + EPWM5_HALF_CMPA*sin(PI*N*SineFactor);
    EPwm5Regs.CMPB =            EPWM5_HALF_CMPB + EPWM5_HALF_CMPB*sin(PI*N*SineFactor);
    EPwm6Regs.CMPA.half.CMPA =  EPWM6_HALF_CMPA + EPWM6_HALF_CMPA*sin(PI*N*SineFactor);
    EPwm6Regs.CMPB =            EPWM6_HALF_CMPB + EPWM6_HALF_CMPB*sin(PI*N*SineFactor);
    N++;


    if ((N*PI*SineFactor) >= 2*PI ){            // Prevent overflow by resetting N at zero phase of sine wave.
        N = 0;
        Pulse2();
        TimeStamp[1] = Nanoseconds;
        }


    CheckMaster();                              //Check if we're the master or slave

    EPwm1Regs.CMPA.half.CMPA = EPWM_PERIOD/2;

    SoftStart();

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    Toggle();
}


__interrupt void 
epwm2_isr(void)
{
    //Toggle();

    EPwm2Regs.ETFLG.bit.SOCA = 1;
    EPwm2Regs.ETFLG.bit.SOCB = 1;

    //
    // Clear INT flag for this timer
    //
    EPwm2Regs.ETCLR.bit.INT = 1;

    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;

    EPwm2Regs.ETCLR.bit.SOCA = 1;
    EPwm2Regs.ETCLR.bit.SOCB = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;


}

__interrupt void 
epwm3_isr(void)
{
    //Toggle();
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
    //Toggle();
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
    //Toggle();
    // Clear INT flag for this timer
    EPwm5Regs.ETCLR.bit.INT = 1;

    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void
epwm6_isr(void)
{
    //Toggle();
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
__interrupt void ADC_isr(void)
{
     //
     // If 10 conversions have been logged, start over
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
//  cpu_timer_0_isr - Use this to keep track of Nanoseconds passed
//

__interrupt void
cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    //Toggle();
    // Nanoseconds = Nanoseconds+12100;   //Keeps track of number of nanoseconds past.
    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// cpu_timer1_isr - Use this to capture real values and check for faults.

//
__interrupt void
cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;

    //
    // I can't get cpu timers to work quickly, so this will be used more in the future.
    // The commented lines can be found in the epwm1 interrupt.
    //

    //Toggle();

    //VpvRMS   = Scalars[0]*GetVoltage(GetRMS(ADCValues[0][0])); //Reference preset Scalars
    //VpriRMS  = Scalars[1]*GetVoltage(GetRMS(ADCValues[0][1]));
    //VpvRMS   = Scalars[2]*GetACRMS(ADCValues[0][2]);
    //VsecRMS  = Scalars[3]*GetVoltage(GetRMS(ADCValues[0][3]));
    //VoutRMS  = Scalars[4]*GetACRMS(ADCValues[0][4]);
    //IoutRMS  = Scalars[5]*GetACRMS(ADCValues[0][5]);
    //IpvRMS   = Scalars[6]*GetVoltage(GetRMS(ADCValues[0][6]));
    //CheckFault();

    //
    // The CPU acknowledges the interrupt.
    //
    EDIS;
}

//
// cpu_timer2_isr
//
__interrupt void
cpu_timer2_isr(void)
{
    EALLOW;
    CpuTimer2.InterruptCount++;

    //
    // The CPU acknowledges the interrupt.
    //
    EDIS;
}

void InitializeADC(void)
{
        //
        // Configure ADC
        //

        AdcRegs.ADCTRL3.bit.SMODE_SEL = 1;
        AdcRegs.ADCTRL1.bit.SEQ_CASC  = 1;

        AdcRegs.ADCMAXCONV.all = 0x0007;       // Setup 8 conv's
        AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; // Setup ADCINA3 as 1st SEQ1 conv.
        AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; // Setup ADCINA2 as 2nd SEQ1 conv
        AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; // Setup ADCINA2 as 2nd SEQ1 conv.
        AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3; // Setup ADCINA2 as 2nd SEQ1 conv.
        AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4; // Setup ADCINA2 as 2nd SEQ1 conv.
        AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5; // Setup ADCINA2 as 2nd SEQ1 conv.
        AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6;
        AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x7;
        //
        // Enable SOCA from ePWM to start SEQ1
        //
        AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;

        AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;   // Enable SEQ1 interrupt (every EOS)
        AdcRegs.ADCTRL2.bit.INT_ENA_SEQ2 = 1;   // Enable SEQ2 interrupt (every EOS)
}


void 
InitEPwm1(void)
{
    //
    // Enable TZ1 as cycle by cycle trip source
    //
    EALLOW;
    EPwm1Regs.TZSEL.bit.CBC1 = 1;

    // Enable TZ interrupt
    EPwm1Regs.TZEINT.bit.CBC = 1;
    EDIS;

    //
    // Setup TBCLK
    //
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;       // Set timer period
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    //
    // Setup shadow register load on ZERO
    //
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Setup compare
    //
    EPwm1Regs.CMPA.half.CMPA = EPWM_PERIOD*0;

    //
    // Set actions
    //
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;      // Set PWM1A on Zero
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;    // Clear PWM1A on event A, up count

    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;      // Set PWM1B on Zero
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;    // Clear PWM1B on event B, up count

    //
    // Interrupt where we will change the Compare Values
    //
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;            // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;       // Generate INT on 3rd event

    //
    //Set CMPA and CMPB to give a 50% Duty Cycle
    //
    EPwm1Regs.CMPA.half.CMPA =  EPWM1_MIN_CMPA;
    EPwm1Regs.CMPB =            EPWM1_MIN_CMPB;
}


void 
InitEPwm2(void)
{

    // Enable SOCA for ADC measurements
    EPwm2Regs.ETSEL.bit.SOCAEN = 1;             // Enable SOCA
    EPwm2Regs.ETSEL.bit.SOCASEL = 4;            // Generate SOCA pulse at 50% duty cycle
    EPwm2Regs.ETPS.bit.SOCAPRD = 1;             // Generate pulse on 1st event

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
    //Set CMPA and CMPB to give a 50% Duty Cycle
    //
    EPwm2Regs.CMPA.half.CMPA =  EPWM2_TIMER_TBPRD/2;
    EPwm2Regs.CMPB =            EPWM2_TIMER_TBPRD/2;

}


void 
InitEPwm3(void)
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
    //EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    //EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    //EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    //
    //  Set CMPA and CMPB to give a 50% Duty Cycle
    //
    EPwm3Regs.CMPA.half.CMPA =  EPWM3_TIMER_TBPRD/2;
    EPwm3Regs.CMPB =            EPWM3_TIMER_TBPRD/2;
}


void
InitEPwm4(void)
{
    //
    //   Setup TBCLK
    //
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm4Regs.TBPRD = EPWM4_TIMER_TBPRD;       // Set timer period
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm4Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm4Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    //
    //   Setup shadow register load on ZERO
    //
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    //   Set actions
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
    //   Interrupt where we will change the Compare Values
    //
    //EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    //EPwm4Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    //EPwm4Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    //
    //  Set CMPA and CMPB to give a 50% Duty Cycle
    //
    EPwm4Regs.CMPA.half.CMPA =  EPWM4_TIMER_TBPRD/2;
    EPwm4Regs.CMPB =            EPWM4_TIMER_TBPRD/2;


}

void
InitEPwm5(void)
{
    //
    //   Enable TZ1 as cycle by cycle trip source
    //
    EALLOW;
    EPwm5Regs.TZSEL.bit.CBC1 = 1;
    EPwm5Regs.TZCTL.bit.TZA = TZ_NO_CHANGE;
    EPwm5Regs.TZCTL.bit.TZB = TZ_NO_CHANGE;

    //
    //   Enable TZ interrupt
    //
    EPwm5Regs.TZEINT.bit.CBC = 1;
    EDIS;

    //
    //   Setup TBCLK
    //
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm5Regs.TBPRD = EPWM5_TIMER_TBPRD;       // Set timer period
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm5Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm5Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    //
    //   Setup shadow register load on ZERO
    //
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    //   Set actions
    //
    EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;      // Set PWM5A on Zero
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;    // Clear PWM5A on event A, up count

    EPwm5Regs.AQCTLB.bit.ZRO = AQ_SET;      // Set PWM5B on Zero
    EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;    // Clear PWM5B on event B, up count


    //
    //   Complimentary Pair
    //
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;          //Turn on DB Module
    EPwm5Regs.DBRED = EPWM5_MIN_DB;                 //Set Minimum for Deadband
    EPwm5Regs.DBFED = EPWM5_MIN_DB;                 //Set Minimum for Deadband
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  //Enable All switches
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       //Active High, A&B are complimentary

    //
    //   Interrupt where we will change the Compare Values
    //
    //EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    //EPwm5Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    //EPwm5Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    //
    //   Set CMPA and CMPB to give a 50% Duty Cycle
    //
    EPwm5Regs.CMPA.half.CMPA =  EPWM5_TIMER_TBPRD/2;
    EPwm5Regs.CMPB =            EPWM5_TIMER_TBPRD/2;
}

void
InitEPwm6(void)
{
    //
    //   Enable TZ1 as cycle by cycle trip source
    //
    EALLOW;
    EPwm6Regs.TZSEL.bit.CBC1 = 1;

    //
    //   Enable TZ interrupt
    //
    EPwm6Regs.TZEINT.bit.CBC = 1;
    EDIS;

    //
    //   Setup TBCLK
    //
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm6Regs.TBPRD = EPWM6_TIMER_TBPRD;       // Set timer period
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm6Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV2;

    //
    //   Setup shadow register load on ZERO
    //
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


    //
    //   Set actions
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
    //   Interrupt where we will change the Compare Values
    //
    //EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    //EPwm6Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    //EPwm6Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event


    //
    //   Set CMPA and CMPB to give a 50% Duty Cycle
    //
    EPwm6Regs.CMPA.half.CMPA =  EPWM6_TIMER_TBPRD/2;
    EPwm6Regs.CMPB =            EPWM6_TIMER_TBPRD/2;
}

//  CheckMaster keeps track of whether the module is a master or slave.
//  If the module does not receive a sync pulse for 3 continuous powerline cycles (50 ms), turn on master.
//  Timestamp for the last sync pulse is stored in TimeStamp[0].


void StartCpuTimers(void){
    InitCpuTimers();
    CpuTimer0Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0
    CpuTimer1Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0
    CpuTimer2Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0

    EALLOW;
    SysCtrlRegs.HISPCP.all = ADC_MODCLK;
    EDIS;
}

void InitCMP(void){
    EPwm1Regs.CMPA.half.CMPA = EPWM_PERIOD*0;
}


void
CheckMaster(void){
    if (Nanoseconds - TimeStamp[0] >= 50000000)
    {
        MasterSlave = 1;                    //If we haven't received a signal in the last 50 ms, become master.

    }
    else {
        MasterSlave = 0;                    //Assume slave mode.

    }
}

//
//  Pulse2 Generates a pulse of 50 DelayLoop cycles.
//
void
Pulse2(void){
    GpioDataRegs.GPCDAT.bit.GPIO84  = 0;
    DelayLoop(50);
    GpioDataRegs.GPCDAT.bit.GPIO84  = 1;
    TimeStamp[1] = Nanoseconds;
}

//
//  DelayLoop burns clocks for a specified duration.
//
void
DelayLoop(int k){
    for ( i = 0; i < k; i++){
        }
    i = 0;
}

//
//  Converts Numbers from ADC Data to Voltages, with biasing considered.
//
double GetVoltage(double InputData){
    double Test = InputData - 35500;
    double Voltage = ((InputData-35500)*0.00004538);
    return Voltage;

}

//
//  Converts Numbers from ADC Data to Voltages, without biasing considered.
//

double GetVoltageUnbiased(double InputData){
    double Test = InputData;
    double Voltage = ((InputData)*0.00004538);
    return Voltage;

}

//
//  Converts Numbers from ADC Data to Voltages, with no offset bias.
//  This is good for AC signals where RMS has already been calculated.
//

//
//  Updates All ADC Values
//
void
UpdateADCMatrix(void){

    for (i=0; i<=7; i++){                               //Reset CurrentSample when the sample reaches the end of the array
        if (CurrentSample[i] >= SampleDepth){
            CurrentSample[i] = 0;
        }
    }

    if (ADCIntCounter - ConversionCounter >= 16){       //Setting 16 gives us ~50 samples at 60 Hz.

        // Update ADC 0, ADCVoltages[] Respectively

        //GetVoltage: (InputData-36060)*0.00004538

        ADCValues[0][0][CurrentSample[0]]   = AdcRegs.ADCRESULT0;
        //ADCVoltages[0][0][CurrentSample[0]] = GetVoltage(ADCValues[0][0][CurrentSample[0]]);

        //ADCValues[1][0][CurrentSample[0]]   = AdcRegs.ADCRESULT1;
        //ADCVoltages[1][0][CurrentSample[0]] = GetVoltage(ADCValues[1][0][CurrentSample[0]]);
        CurrentSample[0]++;


        // Update ADC 1, ADCVoltages[] Respectively

        ADCValues[0][1][CurrentSample[1]]   = AdcRegs.ADCRESULT2;
        //ADCVoltages[0][1][CurrentSample[1]] = GetVoltage(ADCValues[0][1][CurrentSample[1]]);

        //ADCValues[1][1][CurrentSample[1]]   = AdcRegs.ADCRESULT3;
        //ADCVoltages[1][1][CurrentSample[1]] = GetVoltage(ADCValues[1][1][CurrentSample[1]]);
        CurrentSample[1]++;


        // Update ADC 2, ADCVoltages[] Respectively

        ADCValues[0][2][CurrentSample[2]]   = AdcRegs.ADCRESULT4;
        //ADCVoltages[0][2][CurrentSample[2]] = GetVoltage(ADCValues[0][2][CurrentSample[2]]);

        //ADCValues[1][2][CurrentSample[2]]   = AdcRegs.ADCRESULT5;
        //ADCVoltages[1][2][CurrentSample[2]] = GetVoltage(ADCValues[1][2][CurrentSample[2]]);
        CurrentSample[2]++;


        // Update ADC 3, ADCVoltages[] Respectively

        ADCValues[0][3][CurrentSample[3]]   = AdcRegs.ADCRESULT6;
        //ADCVoltages[0][3][CurrentSample[3]] = GetVoltage(ADCValues[0][3][CurrentSample[3]]);

        //ADCValues[1][3][CurrentSample[3]]   = AdcRegs.ADCRESULT7;
        //ADCVoltages[1][3][CurrentSample[3]] = GetVoltage(ADCValues[1][3][CurrentSample[3]]);
        CurrentSample[3]++;


        // Update ADC 4, ADCVoltages[] Respectively

        ADCValues[0][4][CurrentSample[4]]   = AdcRegs.ADCRESULT8;
        //ADCVoltages[0][4][CurrentSample[4]] = GetVoltage(ADCValues[0][4][CurrentSample[4]]);

        //ADCValues[1][4][CurrentSample[4]]   = AdcRegs.ADCRESULT9;
        //ADCVoltages[1][4][CurrentSample[4]] = GetVoltage(ADCValues[1][4][CurrentSample[4]]);
        CurrentSample[4]++;


        // Update ADC 5, ADCVoltages[] Respectively

        ADCValues[0][5][CurrentSample[5]]   = AdcRegs.ADCRESULT10;
        //ADCVoltages[0][5][CurrentSample[5]] = GetVoltage(ADCValues[0][5][CurrentSample[5]]);

        //ADCValues[1][5][CurrentSample[5]]   = AdcRegs.ADCRESULT11;
        //ADCVoltages[1][5][CurrentSample[5]] = GetVoltage(ADCValues[1][5][CurrentSample[5]]);
        CurrentSample[5]++;


        // Update ADC 6, ADCVoltages[] Respectively

        ADCValues[0][6][CurrentSample[6]]   = AdcRegs.ADCRESULT12;
        //ADCVoltages[0][6][CurrentSample[6]] = GetVoltage(ADCValues[0][6][CurrentSample[6]]);

        //ADCValues[1][0][CurrentSample[6]]   = AdcRegs.ADCRESULT13;
        //ADCVoltages[1][0][CurrentSample[6]] = GetVoltage(ADCValues[1][0][CurrentSample[6]]);
        CurrentSample[6]++;


        // Update ADC 7, ADCVoltages[] Respectively

        ADCValues[0][7][CurrentSample[7]]   = AdcRegs.ADCRESULT14;
        //ADCVoltages[0][7][CurrentSample[7]] = GetVoltage(ADCValues[0][7][CurrentSample[7]]);
        //
        //ADCValues[1][7][CurrentSample[7]]   = AdcRegs.ADCRESULT15;
        //ADCVoltages[1][7][CurrentSample[7]] = GetVoltage(ADCValues[1][7][CurrentSample[7]]);
        CurrentSample[7]++;


        ConversionCounter = ADCIntCounter;
    }
    ADCIntCounter++;
}

//
//  Calculate RMS value of given samples
//
double GetRMS(double InputData[SampleDepth]) {
    int Sum = 0;

    for (i=0; i<SampleDepth;i++) {
            Square[i]=0;
    }

    for (i=0; i < SampleDepth; i++){                        //Square the samples
        Square[i] = InputData[i]*InputData[i];
        int Sum = (int) (Sum + Square[i]);                              //Calculate the sum
    }

    double RMSOut = (double) sqrt(Sum * SampleDepthQuotient);           //SampleDepthQuotient = 1/SampleDepth.
                                                                    //Use this to compute the mean of Square[], and then take sqrt.
    return RMSOut;
}

//
//  Revisit this. Doesn't work correctly.
//

double
GetACRMS (double InputData[SampleDepth]){

    double Sum = 0;
    double Average = GetAverage(InputData);                //Calculate DC component

    for (i=0; i<SampleDepth;i++) {
            Square[i]=0;
    }

    for (i=0; i < SampleDepth; i++){                                    //Square value - DC component
        Square[i] = (InputData[i]-Average)*(InputData[i]-Average);
        Sum = Sum + Square[i];                                          //Calculate the sum
    }

    double ACRMSOut = sqrt(Sum * SampleDepthQuotient);           //SampleDepthQuotient = 1/SampleDepth.
                                                                    //Use this to compute the mean of Square[], and then take sqrt.
    return ACRMSOut;
}

double
GetAverage (double InputData[SampleDepth]){

    double Sum = 0;
    double AverageOut = 0;

    for (i=0; i < SampleDepth; i++){                    //Square the samples
            Sum = Sum + InputData[i];                   //Calculate the sum
        }
    AverageOut = Sum * SampleDepthQuotient;             //Compute average using 1/SampleDepth

    return AverageOut;
}

//
//  Force Mosfets to off for fault conditions.
//
void DisableAllSwitches(void) {
    EPwm1Regs.AQCSFRC.all = 1;      // 1 = Force low
    EPwm2Regs.AQCSFRC.all = 1;
    EPwm3Regs.AQCSFRC.all = 1;
    EPwm4Regs.AQCSFRC.all = 1;
    EPwm5Regs.AQCSFRC.all = 1;
    EPwm6Regs.AQCSFRC.all = 1;
    if (ErrorStatus == 0){
        ErrorStatus = 666;
    }
}


//
//  Force Mosfets on.
//
void EnableAllSwitches(void) {
    EPwm1Regs.AQCSFRC.all = 3;      // 3 = Enable switch, disable software forcing
    EPwm2Regs.AQCSFRC.all = 3;
    EPwm3Regs.AQCSFRC.all = 3;
    EPwm4Regs.AQCSFRC.all = 3;
    EPwm5Regs.AQCSFRC.all = 3;
    EPwm6Regs.AQCSFRC.all = 3;
}

//
//  Short the output stage.
//
void ShortOutput(void){
    EPwm1Regs.AQCSFRC.all = 1;      // 1 = Force low
    EPwm2Regs.AQCSFRC.all = 1;
    EPwm3Regs.AQCSFRC.all = 1;
    EPwm4Regs.AQCSFRC.all = 1;
    EPwm5Regs.AQCSFRC.bit.CSFA = 1;
    EPwm6Regs.AQCSFRC.bit.CSFA = 1;

    EPwm5Regs.AQCSFRC.bit.CSFB = 2; // 2 = Force high
    EPwm6Regs.AQCSFRC.bit.CSFB = 2;
}


//
//  Check for all fault conditions (Over voltage, Under voltage, Over current)
//

void CheckFault(void){

    if (VpvRMS >= 45){
        ErrorStatus = 1;
        DisableAllSwitches();
        TimeStamp[4] = Nanoseconds;
        ShortOutput();
    }
    if (VpvRMS <= 10){
        ErrorStatus = 2;
        DisableAllSwitches();
        TimeStamp[4] = Nanoseconds;
        ShortOutput();
    }
    if (IpvRMS >= 20){
        ErrorStatus = 3;
        DisableAllSwitches();
        TimeStamp[4] = Nanoseconds;
        ShortOutput();
    }


    if (VpriRMS >= 50){
        ErrorStatus = 4;
        DisableAllSwitches();
        TimeStamp[4] = Nanoseconds;
    }
    if (VpriRMS <= 0){
        //ErrorStatus = 5;
        //DisableAllSwitches();
        TimeStamp[4] = Nanoseconds;
    }


    if (VsecRMS >= 65){
        //ErrorStatus = 6;
        //DisableAllSwitches();
        TimeStamp[4] = Nanoseconds;
        }
    //if (VsecRMS <= 0){
    //    ErrorStatus = 7;
    //    DisableAllSwitches();
    //    TimeStamp[4] = Nanoseconds;
    //}


    if (VoutRMS >= 65){
        ErrorStatus = 8;
        DisableAllSwitches();
        TimeStamp[4] = Nanoseconds;
    }
    if (IoutRMS >= 20){
        ErrorStatus = 9;
        DisableAllSwitches();
        TimeStamp[4] = Nanoseconds;
    }

    if (FaultCounter >= 3){
        Enable = 0;
    }

    if (ErrorStatus == 0 && Nanoseconds - TimeStamp[4]>= 1000000000){   //If the fault is clear after one second, reset the fault counter and make a timestamp.
        TimeStamp[15] = Nanoseconds;
        FaultCounter = 0;

    }

    return;
}


//
//  Simple function to toggle GPIO84 on and off for debugging purposes.
//
void Toggle(void){
    GpioDataRegs.GPCTOGGLE.bit.GPIO84  = 1;
}



//
//  Soft start function
//  If TimeStamp[3] indicates 0.333 ms have passed, change the duty cycle. Full duty cycle is reached in 3 power cycles, 50 ms.
//

void SoftStart(void){

    if (Nanoseconds - TimeStamp[3] >= 333000){
        if (EPwm1Regs.CMPA.half.CMPA < (TargetDutyCycle[0]*EPWM_PERIOD)){     //TargetDutyCycle[0] is the duty cycle our boost should be at.
            EPwm1Regs.CMPA.half.CMPA++;
            TimeStamp[3] = Nanoseconds;
        }
        else if (EPwm1Regs.CMPA.half.CMPA > (TargetDutyCycle[0]*EPWM_PERIOD)){
            EPwm1Regs.CMPA.half.CMPA--;
            TimeStamp[3] = Nanoseconds;
        }

    }
}

//
// End of File
//
