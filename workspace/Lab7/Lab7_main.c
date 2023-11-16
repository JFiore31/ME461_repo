//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define RW          .19460
#define WR          .56759
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200
#define SIZEOFARRAY 32

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);

//here we are predefining the interrupts that will trigger when the ADC finishes converting a value for us from analog to discrete
__interrupt void ADCD_ISR (void);
__interrupt void ADCA_ISR (void);
//JMF predefinition for EX3 Function
void setupSpib(void);

//JMF Lab 6 predefinitions
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
float controlEffort(float Vk, float ek_1, float Ik_1);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
//JMF
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;
int16_t upDown = 1;
int16_t pwmvalue2 = 0;
int16_t pwmvalue3 = 0;
float scaledADC1 = 0.0;
float scaledADC2 = 0.0;
//JMF for EX3
int16_t address = 0;
int16_t accelxraw = 0;
int16_t accelyraw = 0;
int16_t accelzraw = 0;
int16_t temperature = 0;
int16_t gyroxraw = 0;
int16_t gyroyraw = 0;
int16_t gyrozraw = 0;
float accelx = 0.0;
float accely = 0.0;
float accelz = 0.0;
float gyrox = 0.0;
float gyroy = 0.0;
float gyroz = 0.0;

//Lab 6 Variables
//JMF Rad/s wheel values
float LeftWheel = 0.0;
float RightWheel = 0.0;

float leftFoot = 1.0/4.993; //JMF Feet/Radians
float rightFoot = 1.0/5.063;
//JMF commands sent to setting the EPWM for each motor
float uLeft = -5.0;
float uRight = -5.0;

//JMF position variable used in wheel control
float PosLeftPrev = 0.0;
float PosLeftCur = 0.0;
float PosRightPrev = 0.0;
float PosRightCur = 0.0;

float VLeftCur = 0.0;
float VRightCur = 0.0;

float refVelo = 0.0;
//JMF Current error
float ek_L = 0.0;
float ek_R = 0.0;

//JMF previous error
float ek_1 = 0.0;
float ek_1_R = 0.0;
float ek_1_L = 0.0;

float Ik_L = 0.0; //JMF current Integral
float Ik_R = 0.0;
float Ik_1 = 0.0; //JMF previous integral
float Ik_1_R = 0.0;
float Ik_1_L = 0.0;
uint16_t Kp = 3; //error gain
uint16_t Ki = 15; //integral gain
uint16_t Kturn = 3; //turn error gain
float et = 0.0; //turn error
float turn = 0.0; //turn reference that is sent (desired turn)

float printLV3 = 0;
float printLV4 = 0;
float printLV5 = 0;
float printLV6 = 0;
float printLV7 = 0;
float printLV8 = 0;
float x = 0;
float y = 0;
float bearing = 0;
extern uint16_t NewLVData;
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];
uint32_t numTimer2calls = 0;
float theta_L = 0.0;
float theta_R = 0.0;
float theta_avg = 0.0;
float theta_dot_avg = 0.0;
float x_dot = 0.0;
float y_dot = 0.0;
float x_dot_prev = 0.0;
float y_dot_prev = 0.0;
float theta_R_prev = 0.0;
float theta_L_prev = 0.0;

//Lab 7 added
//yk1 and yk2 are the mapped and filtered voltage values of the x and y directions given by the joystick
float yk1 = 0.0;
float yk2 = 0.0;
//these variables are used to hold the discrete value that adca's soc 0 and 1 converts. We need to pull these values from their bit fields because we need to map them to volts
int16_t adca0result = 0;
int16_t adca1result = 0;
//JMF scaled volt value array for storing the values of how much he joy-stick is in the X direction and Y direction
float dim1[SIZEOFARRAY] = {0,0,0,0,0};
float dim2[SIZEOFARRAY] = {0,0,0,0,0};

//JMF these count variables will be used to track how many times the interrupts are entered for each ACD. We record this to time how often we print values to terra term
int32_t ADCD1_count = 0;
int32_t ADCA1_count = 0;

int16_t spib_count = 0;

//JMF This array stores the filtering coefficients for a 31st order low pass filter with a cutoff frequency of 500Hz. The MATLAB function used was b=fir1(31,.25)
float b[32]={   -6.3046914864397922e-04,
                -1.8185681242784432e-03,
                -2.5619416124584822e-03,
                -1.5874939943956356e-03,
                2.3695126689747326e-03,
                8.3324969783531780e-03,
                1.1803612855040625e-02,
                6.7592967793297151e-03,
                -9.1745119977290398e-03,
                -2.9730906886035850e-02,
                -3.9816452266421651e-02,
                -2.2301647638687881e-02,
                3.1027965907247105e-02,
                1.1114350049251465e-01,
                1.9245540210070616e-01,
                2.4373020388648489e-01,
                2.4373020388648489e-01,
                1.9245540210070616e-01,
                1.1114350049251465e-01,
                3.1027965907247105e-02,
                -2.2301647638687881e-02,
                -3.9816452266421651e-02,
                -2.9730906886035850e-02,
                -9.1745119977290398e-03,
                6.7592967793297151e-03,
                1.1803612855040625e-02,
                8.3324969783531780e-03,
                2.3695126689747326e-03,
                -1.5874939943956356e-03,
                -2.5619416124584822e-03,
                -1.8185681242784432e-03,
                -6.3046914864397922e-04};

// Kalman Filter Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -.76;
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
//float LeftWheel = 0;
//float RightWheel = 0;
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;
float vel_Right = 0.0;
float vel_Left = 0.0;
float LeftWheelPrev = 0.0;
float RightWheelPrev = 0.0;
float tilt_valuePrev = 0.0;
float gyrorate_dot = 0.0;
float ubal = 0.0;
float K1 = -60.0;
float K2 = -4.5;
float K3 = -1.1;
float K4 = -0.1;

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    //JMF PWM for DC motors and H bridge PinMux setup. Set to 1 so that it is a PWM signal and not a GPIO. EPWM2A (EX 2)
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);

    //JMF PWM for DC motors and H bridge PinMux setup. Set to 1 so that it is a PWM signal and not a GPIO. EPWM2B (EX 2)
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);


    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    PieVectTable.ADCA1_INT = &ADCA_ISR;

    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    //JMF cputimer0 used for EX1
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
    //JMF cputimer1 used for EX2
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 4000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    setupSpib();
    init_eQEPs();

    //JMF instead of using the cpu timers to determine the period of how often the ADC will do its job, we want to use a PWM pin to do this. Here we are setting up EPWM 5 with a period value
    //that we determine is fitting for our ADCs to sample at. the input clock is set at 50 MHz, so we use the TBPRD counting bit field to determine the time span between samples
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    //for ex2,3
    //EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    //for ex4
    //EPwm5Regs.TBPRD = 12500; // Set Period to .25ms sample. (4000Hz) Input clock is 50MHz.
    //JMF for last section of ex4
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. (1000Hz) Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;


    //    set up ADCs
    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings

    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;


    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB:

    //sets up ADCs in the way we want. Not the channels are determined by where periferals are hooked up to the board while the SOC are always 0 and 1 since we want to use the first priorities in the list
    //trigsel is for determining how often they will run and we use EPWM5 which is pin 13
    //we must flag and clear to tell the interrupt that the ADC has a new value and is ready to send it
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //1.3 Set up DACs
    // Enable DACA and DACB outputs
    EALLOW;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.LOADMODE = 0; //load on next sysclk
    DacaRegs.DACCTL.bit.DACREFSEL = 1; //use ADC VREF as reference voltage
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0; //load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1; //use ADC VREF as reference voltage
    EDIS;


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6; //SPIB

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //JMF Enable SPIB in the PIE: Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    // ADCA
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    //init_serialSCIB(&SerialB,115200);
    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    //JMF Set EPwm2 register 2a right, 2b left for exercise 2 DC motors
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; //anything with 1x bit
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;  //JMF this sets to control up mode which tells the counter to go up every clock cycle
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; //JMF this value determines if the input frequency will be divided by a value. We chose 1 since we want to 50 MHz Freq.
    EPwm2Regs.TBCTL.bit.PHSEN = 0; //JMF count down after the synchronization event

    EPwm2Regs.TBCTR = 0; //JMF sets the counter bit field to 0 to start before counting

    EPwm2Regs.TBPRD = 2500; //JMF to set period that the counter will count to

    EPwm2Regs.CMPA.bit.CMPA = 0; //set duty cycle to 0
    EPwm2Regs.CMPB.bit.CMPB = 0; //JMF also set 0 duty cycle for compare to B

    EPwm2Regs.AQCTLA.bit.CAU = 1; //JMF Tells the processor what to do when TBCTR = CMPA. In this case we chose to clear and set the EPWM value to LOW.
    EPwm2Regs.AQCTLA.bit.ZRO = 2; //JMF tell the processor what to do when TBCTR - 0. In this case we chose to set the EPWM value to HIGH.
    EPwm2Regs.AQCTLB.bit.CBU = 1; //JMF Tells the processor what to do when TBCTR = CMPA. In this case we chose to clear and set the EPWM value to LOW.
    EPwm2Regs.AQCTLB.bit.ZRO = 2; //JMF tell the processor what to do when TBCTR - 0. In this case we chose to set the EPWM value to HIGH.

    EPwm2Regs.TBPHS.bit.TBPHS = 0; //JMF sets the signal phase to 0. not sure if necessary

    ////////////////////////////////////////////////////////////////////////////////  WHILE LOOP /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    while(1)
    {
        if (UARTPrint == 1 ) {
            //serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            //JMF to print ADC values to tera term that were received from the dan chip in EX2
            //serial_printf(&SerialA, "ADC1 Value:%.3f ADC2 Value:%.3f\r\n",scaledADC1,scaledADC2);
            //JMF to print X,Y,Z Acceleration and Gyro values received from MPU
//            serial_printf(&SerialA, "accel (g): %.3f %.3f %.3f, gyro (deg per sec): %.3f %.3f %.3f,  Voltages %.3f, %.3f   AngleL %.3f, AngleR %.3f\r\n\n",scldXAccel, scldYAccel, scldZAccel, scldXGyro, scldYGyro, scldZGyro,yk1,yk2,LeftWheel, RightWheel);
            //            serial_printf(&SerialA, "Wheel (L,R) %.3f %.3f\r\n",LeftWheel, RightWheel); //Lab6 Ex1
            //JMF Print current velocity of the two wheels
            //            serial_printf(&SerialA, "Wheel velocity (L,R) %.3f %.3f\r\n",VLeftCur, VRightCur); //Lab6 Ex2
            //            serial_printf(&SerialA, "X= %.3f Y= %.3f bearing= %.3f\r\n",x, y, bearing);
            //            serial_printf(&SerialA, "ADC1 Value:%.3f ADC2 Value:%.3f\r\n",scaledADC1,scaledADC2);
            //            serial_printf(&SerialA,"Channel 1 (X hat): %.3f, Channel 0 (Y hat): %.3f\r\n",yk1,yk2);
            serial_printf(&SerialA,"tilt_value %.3f, gyro_value %.3f, leftwheel %.3f, rightwheel %.3f\r\n",tilt_value, gyro_value, LeftWheel, RightWheel);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......
    vel_Right = .6*vel_Right + 100*(RightWheel - RightWheelPrev);
    vel_Left = .6*vel_Left + 100*(LeftWheel - LeftWheelPrev);
    LeftWheelPrev = LeftWheel;
    RightWheelPrev = RightWheel;
    gyrorate_dot = .6*gyro_value + 100*(tilt_value - tilt_valuePrev);
    tilt_valuePrev = tilt_value;


    ubal = -K1*tilt_value - K2*gyro_value -K3*(vel_Left + vel_Right)/2.0 -K4*gyrorate_dot;
    uLeft = ubal/2;
    uRight = ubal/2;

    setEPWM2A(uRight);
    setEPWM2B(-uLeft);


    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    //JMF Exercise 3
    // call every 1ms
    //    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    //
    //    SpibRegs.SPITXBUF = 0xBA00;
    //    SpibRegs.SPITXBUF = 0x0000;
    //    SpibRegs.SPITXBUF = 0x0000;
    //    SpibRegs.SPITXBUF = 0x0000;
    //    SpibRegs.SPITXBUF = 0x0000;
    //    SpibRegs.SPITXBUF = 0x0000;
    //    SpibRegs.SPITXBUF = 0x0000;
    //    SpibRegs.SPITXBUF = 0x0000;


    //Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    //JMF Clear to SS
    //    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 2; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    //    SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope


    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%25) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

//    if ((numTimer0calls%50) == 0) {
        // Blink LaunchPad Red LED
//        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
//    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    //    if ((CpuTimer0.InterruptCount % 200) == 0) {
    //        UARTPrint = 1;
    //    }
}

//JMF initialization of the Quadrature encoders that tell us how fast the wheels are spinning
void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}
float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    // 1200 counts per 1 wheel spin, ?count * (2pi (rad) / (400*30) (counts)) = ?rad
    return -1*(raw*(2*PI)/12000.0);
}
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(2*PI)/12000.0);
}

// cpu_timer1_isr - CPU Timer1 ISR
//JMF Used for EX2
__interrupt void cpu_timer1_isr(void)
{
    //Clear GPIO9 Low to act as a Slave Select. Right now, just to scope. Later to select DAN28027 chip
    //JMF Clear to SS
    //GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    //JMF send DA to set up SPI transmission as datasheet calls for
    //JMF writing to the FIFO happens in ~50 nano second but sending the data over takes something like 48 micro seconds.
    //SpibRegs.SPIFFRX.bit.RXFFIL = 3; //JMF Issue the SPIB_RX_INT when three values are in the RX FIFO. This is the command that sends the data out of the FIFO to the chip
    // JMF this being set to 3 means that the interrupt to recieve data back form the chip will also be called when 3 16 bit numbers are sent.

    //JMF first send the start command, then send our two values of PWM signal we want the DAN chip t use
    //    SpibRegs.SPITXBUF = 0x00DA;
    //    SpibRegs.SPITXBUF = pwmvalue2;
    //    SpibRegs.SPITXBUF = pwmvalue3;
    //JMF do not read the register instead just increment a variable and pass it to the send at the end of the if statement
    //JMF this is like setting a duty cycle
    if (upDown == 1){
        pwmvalue2 = pwmvalue2 + 10;
        pwmvalue3 = pwmvalue3 + 10;
        if (pwmvalue2 == 3000 || pwmvalue3 == 3000) {//JMF3000 to prevent overflow
            upDown = 0;
        }
    } else {
        pwmvalue2 = pwmvalue2 - 10;
        pwmvalue3 = pwmvalue3 - 10;
        if (pwmvalue2 == 0 || pwmvalue3 == 0) {
            upDown = 1;
        }
    }
    CpuTimer1.InterruptCount++;
    //JMF this is scaling to convert the ADC values we recieve from the DAN chip to float to be displayed as a voltage
    scaledADC1 = (3.3/4095)*spivalue2;
    scaledADC2 = (3.3/4095)*spivalue3;
    //JMF print statement set to print every 100 ms
    //    if ((CpuTimer1.InterruptCount % 5) == 0) {
    //        UARTPrint = 1;
    //    }
}

////////////////////////////////////////////////////////////////////////// CPU TIMER 2 /////////////////////////////////////////////////////////////////////////////////////////////////////////
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    //GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
    numTimer2calls++;

    //    //JMF store rad value of the wheels rotation
    //    LeftWheel = readEncLeft();
    //    RightWheel = readEncRight();
    //
    //    //Turn rad into ft
    //    PosLeftCur = LeftWheel*leftFoot;
    //    PosRightCur = RightWheel*rightFoot;
    //
    //    //JMF this interrupt is called every .004 s, so we are taking delta x over delta t which gives us velocity
    //    VLeftCur = (PosLeftCur - PosLeftPrev)/0.004;
    //    VRightCur = (PosRightCur - PosRightPrev)/0.004;
    //    //JMF pass back the velocity to the previous velocity
    //    PosLeftPrev = PosLeftCur;
    //    PosRightPrev = PosRightCur;
    //
    //    //JMF first, calculate what the turn error is based on the two wheels velo and the turn desired
    //    et = VLeftCur - VRightCur + turn;
    //
    //    // Controller Left
    //    //ek_L = refVelo - VLeftCur;
    //    //JMF this is the updated error to include turning.
    //    //Now we are comparing the reference to the actual velocity and if there is a turn desired we add it in
    //    ek_L = refVelo - VLeftCur - Kturn*et;
    //    //JMF solve for the integral error to improve motor function to actually get to vref
    //    Ik_L = Ik_1_L + 0.004*((ek_L+ek_1_L)/2.0);
    //
    //    //Set uLeft to the respective gains and errors. This s the value we actually set the motors to
    //    uLeft = Kp*ek_L + Ki*Ik_L;
    //    //ek_1_L = ek_L;
    //
    //    //JMF check if uLeft is saturated. This is a sign that we have integral windup which we want to prevent
    //    if(uLeft >= 10.0) {
    //        uLeft = 10.0;
    //        Ik_L = Ik_1_L;
    //    } else if(uLeft <= -10.0) {
    //        uLeft = -10.0;
    //        Ik_L = Ik_1_L;
    //    }
    //    // Controller Right
    //    //ek_R = refVelo - VRightCur;
    //    //JMF this is the updated error to include turning.
    //    //Now we are comparing the reference to the actual velocity and if there is a turn desired we add it in
    //    ek_R = refVelo - VRightCur + Kturn*et;
    //    //JMF solve for the integral error to improve motor function to actually get to vref
    //    Ik_R = Ik_1_R + 0.004*((ek_R+ek_1_R)/2.0);
    //
    //    //Set uRight to the respective gains and errors. This is the value we actually set the motors to
    //    uRight = Kp*ek_R + Ki*Ik_R;
    //    //ek_1_R = ek_R;
    //
    //    //JMF check if uRight is saturated. This is a sign that we have integral windup which we want to prevent
    //    if(uRight >= 10.0) {
    //        uRight = 10.0;
    //        Ik_R = Ik_1_R;
    //    } else if(uRight <= -10.0) {
    //        uRight = -10.0;
    //        Ik_R = Ik_1_R;
    //    }
    //
    //    //JMF need to pass -uLeft since the motors are wired the same, but are orientated 180 deg from each other so their positive directions are inverted.
    //    setEPWM2A(uRight);
    //    setEPWM2B(-uLeft);
    //    if ((CpuTimer2.InterruptCount % 10) == 0) {
    //        UARTPrint = 1;
    //    }

    //WIFI Commuications:
    if (NewLVData == 1) {
        NewLVData = 0;
        refVelo = fromLVvalues[0];
        turn = fromLVvalues[1];
        printLV3 = fromLVvalues[2];
        printLV4 = fromLVvalues[3];
        printLV5 = fromLVvalues[4];
        printLV6 = fromLVvalues[5];
        printLV7 = fromLVvalues[6];
        printLV8 = fromLVvalues[7];
    }

    theta_L = LeftWheel;
    theta_R = RightWheel;
    bearing = (RW/WR) * (theta_R - theta_L);
    theta_avg = 0.5* (theta_R + theta_L);
    theta_dot_avg = 0.5 * ((theta_L - theta_L_prev)/.004 + (theta_R - theta_R_prev)/.004);
    theta_L_prev = theta_L;
    theta_R_prev = theta_R;
    x_dot = RW*theta_dot_avg*cos(bearing);
    y_dot = RW*theta_dot_avg*sin(bearing);
    x = x + (x_dot + x_dot_prev)/2*.004;
    y = y + (y_dot + y_dot_prev)/2*.004;
    x_dot_prev = x_dot;
    y_dot_prev = y_dot;



    if((numTimer2calls%62) == 0) { // change to the counter variable of you selected 4ms. timer
        DataToLabView.floatData[0] = x;
        DataToLabView.floatData[1] = y;
        DataToLabView.floatData[2] = bearing;
        DataToLabView.floatData[3] = 2.0*((float)numTimer0calls)*.001;
        DataToLabView.floatData[4] = 3.0*((float)numTimer0calls)*.001;
        DataToLabView.floatData[5] = (float)numTimer0calls;
        DataToLabView.floatData[6] = (float)numTimer0calls*4.0;
        DataToLabView.floatData[7] = (float)numTimer0calls*5.0;
        LVsenddata[0] = '*'; // header for LVdata
        LVsenddata[1] = '$';
        for (int i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
            if (i%2==0) {
                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
            } else {
                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
            }
        }
        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    }

}



//JMF EX1 SPIB_isr interrupt
//__interrupt void SPIB_isr(void)
//{
//    spivalue1 = SpibRegs.SPIRXBUF; // Read first 16-bit value off RX FIFO. Probably is zero since no chip
//    spivalue2 = SpibRegs.SPIRXBUF; // Read second 16-bit value off RX FIFO. Again probably zero
//    GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO9 high to end Slave Select. Now Scope. Later to de-select DAN28027
//    // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.
//    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
//    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
//}

//JMF EX2 SPIB_isr interrupt
__interrupt void SPIB_isr(void)
{
    //EX3
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPI66 high to end Slave Select.
    address = SpibRegs.SPIRXBUF; // JMF Discard this value, it is nothing from address write byte and the INT_STATUS which we don't care about
    accelxraw = SpibRegs.SPIRXBUF; // JMF accelx
    accelyraw = SpibRegs.SPIRXBUF; // JMF accely
    accelzraw = SpibRegs.SPIRXBUF; // JMF accelz
    temperature = SpibRegs.SPIRXBUF; // JMF Discard this value, it is the temperature reading that we do not care about here. Just nice to take so we don't have to SS again.
    gyroxraw = SpibRegs.SPIRXBUF; // JMF gyrox
    gyroyraw = SpibRegs.SPIRXBUF; // JMF gyroy
    gyrozraw = SpibRegs.SPIRXBUF; // JMF gyroz
    //JMF X AND Y ACCEL SATURATED. GYRO SEEMS A LITLE OFF




    accelx = accelxraw*(4.0/32768.); //JMF accelx is 16 bit signed int. We want to map this range of +-32768 to the range of +-4
    accely = accelyraw*(4.0/32768.);
    accelz = accelzraw*(4.0/32768.);
    gyrox = gyroxraw*(250.0/32768.); //JMF gyrox is 16 bit signed int. We want to map this range of +-32768 to the range of +-250
    gyroy = gyroyraw*(250.0/32768.);
    gyroz = gyrozraw*(250.0/32768.);

    //EX2
    //    //JMF this interrupt is entered when we have sent over the amount of data values (in this case 3) to surpass the threshold to send to the DAN chip
    //    //JMF these are the values we read back from the Dan chip
    //    spivalue1 = SpibRegs.SPIRXBUF; // JMF Nothing signal from sending 0x00DA. Discard
    //    spivalue2 = SpibRegs.SPIRXBUF; // JMF ADC1 value from Dan Datasheet.
    //    spivalue3 = SpibRegs.SPIRXBUF; // JMF ADC2 value from Dan Datasheet.
    //
    //    GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO9 high to end Slave Select.
    //


    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){
        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;
        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2){
        accelx -=(accelx_offset);
        accely -=(accely_offset);
        accelz -=(accelz_offset-accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelz; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();
        if (SpibNumCalls >= 3) { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }
    }
    timecount++;
    if((timecount%200) == 0)
    {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        //UARTPrint = 1; // Tell While loop to print
    }



    //JMF need to pass -uLeft since the motors are wired the same, but are orientated 180 deg from each other so their positive directions are inverted.
//    setEPWM2A(-uRight);
//    setEPWM2B(uLeft);

    spib_count++;
    if(spib_count % 200 == 0){
        UARTPrint = 1;
    }


    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
}

void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in between each transfer to 0. Also don’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65, 66 which are also a part of the SPIB setup.
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected

    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;
    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16-bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    //JMF 50 MHz/1 MHz = SPIBRR+1. Here we are setting SPIBRR
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip.
    //JMF Be careful in counting in hex^ must go through letters
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; //Interrupt Level to 16 words or more received into FIFO causes
    //interrupt. This is just the initial setting for the register. Will be changed below
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; //JMF Same as above. This is setting to transmit and receive 16 bits each write to SPITXBUF
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00;
    //-----------------------------------------------------------------------------------------------------------------

    //Step 2.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16-bit transfers, so two registers at a time after the first 16-bit transfer.
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = 0x1300;
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = 0x0000;
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = 0x0000;
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = 0x0013;
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = 0x0200;
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = 0x0806;
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = 0x0000;

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    //JMF read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO. We have to do this 7 times since we sent 7 values!
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low

    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = 0x2300;
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = 0x408C;
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = 0x0288;
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = 0x0C0A;

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // JMF read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO. We have to do this 4 times since we sent 4 values!
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 4.
    // perform a single 16-bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = 0x2A81;

    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);

    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x0016); // 0x7700, offset for accelerometer. XA_OFFSET_H
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x001A); // 0x7800, XA_OFFSET_L
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00E1); // 0x7A00, YA_OFFSET_H
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x0022); // 0x7B00, YA_OFFSET_L
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0025); // 0x7D00,  ZA_OFFSET_H
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x002E); // 0x7E00,  ZA_OFFSET_L
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    //JMF NOTE FOR OFFSET CALIBRATION
    // if tera z offset = 2.1
    // 2.1 divide 0.00098 = 2143 (decimal) offset in negative needed
    // -2143 (decimal) = F7A1 (hex)
    // note for 16 bits, last bit is reserved. Need to leftshift value by 1: F7A1 <<1 = EF42
    // enter EF to Z high offset, enter 42 to Z low offset
    //We send a 16 bit number to the chip. to send these offset values we send the value 0x7DEF and 0x7E42. This is the register address and value we want to send.
    // refer to MPU-9250-Register-Map.pdf, ZA_OFFS[14:7]

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
// Exercise 2
//JMF this function is used to set the angle at which we want the RC servos to go to. They can travel form -90 to 90 degrees which we take as an input float
//We must map this float to the CMPA 16 bit integer to do this since CMPA is a bit field and doesn't accept a float
void setEPWM8A_RCServo(float angle) {
    if(angle > 90) {
        angle = 90;
    }
    if(angle < -90) {
        angle = -90;
    }
    //JMF linear mapping logic
    float slope = (0.08 * (float)(EPwm8Regs.TBPRD)) / 180;
    EPwm8Regs.CMPA.bit.CMPA = (int16_t)((slope * angle) + 0.08*(float)(EPwm8Regs.TBPRD));
}
//JMF this function is used to set the angle at which we want the RC servos to go to. They can travel form -90 to 90 degrees which we take as an input float
//We must map this float to the CMPA 16 bit integer to do this since CMPA is a bit field and doesn't accept a float
void setEPWM8B_RCServo(float angle){
    if(angle > 90) {
        angle = 90;
    }
    if(angle < -90) {
        angle = -90;
    }
    //JMF linear mapping logic
    float slope = (0.08 * (float)(EPwm8Regs.TBPRD)) / 180;
    EPwm8Regs.CMPB.bit.CMPB = (int16_t)((slope * angle) + 0.08*(float)(EPwm8Regs.TBPRD));
}
// JMF Exercise 2
//This function takes in a float value between 10 and -10 to set the percentage of the RC servo connected to the wheels speed and direction. Negative values are reverse
//We have to map this float value to the range of bits that can be passed to the EPWM CMPA value. TO do this, we had to cast the float to a 16 bit int correctly after mapping
void setEPWM2A(float controleffort){
    if(controleffort > 10) {
        controleffort = 10;
    }
    if(controleffort < -10) {
        controleffort = -10;
    }
    //JMF
    //-10 is CMPA = 0-full negative movement
    //10 is CMPA = TBPRD-full positive movement
    //0 is CMPA = .5*TBPRD-no movement
    EPwm2Regs.CMPA.bit.CMPA = (int16_t)((controleffort + 10) * ((float)(EPwm2Regs.TBPRD))/20);
}

//This function takes in a float value between 10 and -10 to set the percentage of the RC servo connected to the wheels speed and direction. Negative values are reverse
//We have to map this float value to the range of bits that can be passed to the EPWM CMPA value. TO do this, we had to cast the float to a 16 bit int correctly after mapping
void setEPWM2B(float controleffort) {
    if(controleffort > 10) {
        controleffort = 10;
    }
    if(controleffort < -10) {
        controleffort = -10;
    }
    //JMF
    //-10 is CMPA = 0-full negative movement
    //10 is CMPA = TBPRD-full positive movement
    //0 is CMPA = .5*TBPRD-no movement
    EPwm2Regs.CMPB.bit.CMPB = (int16_t)((controleffort + 10) * ((float)(EPwm2Regs.TBPRD))/20);
}



//This function sets DACA to the voltage between 0V and 3V passed to this function.
//If outside 0V to 3V the output is saturated at 0V to 3V
//Example code
//float myu = 2.25;
//setDACA(myu); // DACA will now output 2.25 Volts
//they are used to turn the digital signal we have recieved and filtered back into a analog one so that we can show the signal on the oscilloscope
void setDACA(float dacouta0) {
    int16_t DACOutInt = 0;
    DACOutInt = (4096.0/3.0)*dacouta0; // perform scaling of 0 – almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;

    DacaRegs.DACVALS.bit.DACVALS = DACOutInt;
}

void setDACB(float dacoutb0) {
    int16_t DACOutInt = 0;
    DACOutInt = (4096.0/3.0)*dacoutb0; // perform scaling of 0 – almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;

    DacbRegs.DACVALS.bit.DACVALS = DACOutInt;
}


__interrupt void ADCA_ISR (void) {
    yk1 = 0;
    yk2 = 0;
    adca0result = AdcaResultRegs.ADCRESULT0;
    adca1result = AdcaResultRegs.ADCRESULT1;
    // Here covert ADCIND0, ADCIND1 to volts
    dim1[0] = (3.0/4096.0)*adca0result;
    dim2[0] = (3.0/4096.0)*adca1result;

    for(int i = 0; i < SIZEOFARRAY; i++) {
        yk1 += b[i]*dim1[i];
        yk2 += b[i]*dim2[i];
    }

    for(int i = SIZEOFARRAY - 1; i >=1; i--){
        dim1[i] = dim1[i-1];
        dim2[i] = dim2[i-1];
    }


    // Here write yk1 to DACA channel and yk2 to DACB channel
    setDACA(yk1);
    setDACB(yk2);

    // Print ADCIND0 and ADCIND1’s voltage value to TeraTerm every 100ms
    //PWM period 1ms
    ADCA1_count++;
    //    if((ADCA1_count % 100) == 0){
    //        UARTPrint = 1;
    //    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;

    SpibRegs.SPITXBUF = 0xBA00;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
}
////adcd1 pie interrupt
//__interrupt void ADCD_ISR (void) {
//    yk = 0;
//    adcd0result = AdcdResultRegs.ADCRESULT0;
//    adcd1result = AdcdResultRegs.ADCRESULT1;
//    // Here covert ADCIND0, ADCIND1 to volts
//    xk_n[0] = (3.0/4096.0)*adcd0result;
//    ADCIND1Volts = (3.0/4096.0)*adcd1result;
//
//    //for 21st order in EX. 2
//    for(int i = 0; i < SIZEOFARRAY; i++) {
//        yk += b[i]*xk_n[i];
//    }
//    //yk = b[0]*xk[0] + b[1]*xk_1 + b[2]*xk_2 + b[3]*xk_3 + b[4]*xk_4;
//
//    //for 21st order
//    for(int i = SIZEOFARRAY - 1; i >=1; i--){
//        xk_n[i] = xk_n[i-1];
//    }
////Save past states before exiting from the function so that next sample they are the older state
////    xk_4 = xk_3;
////    xk_3 = xk_2;
////    xk_2 = xk_1;
////    xk_1 = xk;
//
//// Here write yk to DACA channel
//    setDACA(yk);
//// Print ADCIND0 and ADCIND1’s voltage value to TeraTerm every 100ms
//    //PWM period 1ms
//    ADCD1_count++;
//    if((ADCD1_count % 100) == 0){
//        UARTPrint = 1;
//        //Called every 100ms
//    }
//AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
//PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//}
