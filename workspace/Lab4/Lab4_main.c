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
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200
//JMF this defined value tells us how many coefficents we will be using to filter our signal in later steps. It is 22 terms in Ex2 and 31 in Ex4
#define SIZEOFARRAY 32

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

//JMF here we are predefining the interrupts that will trigger when the ADC finishes converting a value for us from analog to discrete
__interrupt void ADCD_ISR (void);
__interrupt void ADCA_ISR (void);
__interrupt void ADCB_ISR (void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
//ex1.4e
//JMF these variables are used to hold the discrete value that adcd's soc 0 and 1 converts. We need to pull these values from their bit fields because we need to map them to volts
int16_t adcd0result = 0;
int16_t adcd1result = 0;

//ex3
//JMF these variables are used to hold the discrete value that adca's soc 0 and 1 converts. We need to pull these values from their bit fields because we need to map them to volts
int16_t adca0result = 0;
int16_t adca1result = 0;

//ex4
int16_t adcb0result = 0; //JMF these variables are used to hold the discrete value that adcb's soc 0 converts. We need to pull this value from its bit field because we need to map them to volts

//JMF used to print ADCIND1 to tera term. Mapped variable from the bit held in the results bit field
float ADCIND1Volts = 0.0;
//JMF first variable used to map the bit number to volts for some input signal
float scaledadcd0result = 0.0;

//JMF these count variables will be used to track how many times the interrupts are entered for each ACD. We record this to time how often we print values to terra term
int32_t ADCD1_count = 0;
int32_t ADCA1_count = 0;
int32_t ADCB1_count = 0;

//ex2 predefinition of variables for print
float yk = 0;//JMF yk is the filtered value of our signal using a simple average filter

//ex3
//JMF yk1 and yk2 are the mapped and filtered voltage values of the x and y directions given by the joystick
float yk1 = 0.0;
float yk2 = 0.0;
//ex4
float yk3 = 0.0; //JMF yk3 is the filtered output of the voice signal given to the microphone

//JMF initial array used for 5 term filtering. all set to .2 since this gives us the average of the signal
//float b[SIZEOFARRAY] = {0.2,0.2,0.2,0.2,0.2}; // 0.2 is 1/5th therefore a 5 point average

//JMF initial array used for 5 term filtering. We used MATLAB to get coefficients that give us a cut off frequency of 50Hz. MATLAB code: b = fir1(5,.1) arraytoCformat(b)
//in fir1 function, .1 refers to 10% of the Nyquist frequency which is 500 since the sample rate is 1000 Hz
//float b[SIZEOFARRAY]={    3.3833240118424500e-02, 2.4012702387971543e-01, 4.5207947200372001e-01, 2.4012702387971543e-01, 3.3833240118424500e-02};

//JMF new MATLAB code given to give filtering coefficients for 21 term filter. This is a better low pass filter than 5 terms and used in EX2 and EX3
//b = fir1(21,75/500) arraytoCformat(b) is the code for how we get the output b array
//float b[22]={   -2.3890045153263611e-03,
//    -3.3150057635348224e-03,
//    -4.6136191242627002e-03,
//    -4.1659855521681268e-03,
//    1.4477422497795286e-03,
//    1.5489414225159667e-02,
//    3.9247886844071371e-02,
//    7.0723964095458614e-02,
//    1.0453473887246176e-01,
//    1.3325672639406205e-01,
//    1.4978314227429904e-01,
//    1.4978314227429904e-01,
//    1.3325672639406205e-01,
//    1.0453473887246176e-01,
//    7.0723964095458614e-02,
//    3.9247886844071371e-02,
//    1.5489414225159667e-02,
//    1.4477422497795286e-03,
//    -4.1659855521681268e-03,
//    -4.6136191242627002e-03,
//    -3.3150057635348224e-03,
//    -2.3890045153263611e-03};
//JMF for 21st order. This array will store the last 21 scaled to volt values from the ADC that will be filtered by their respective weights given in b[].
//Note that setting the first 5 values to 0 is usually required, but since the first value moves along the array, putting just the 0th value as 0 will eventually fill the entire array, but usually
//we would have to set them all to 0 after declaring the array
float xk_n[SIZEOFARRAY] = {0,0,0,0,0};

//ex3
//JMF scaled volt value array for storing the values of how much he joy-stick is in the X direction and Y direction
float dim1[SIZEOFARRAY] = {0,0,0,0,0};
float dim2[SIZEOFARRAY] = {0,0,0,0,0};

//ex4
//JMF scaled volt value array that hold the values of the sound waves passed to the ADCB by the microphone
float sound[SIZEOFARRAY] = {0};
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
//JMF lastly, this array stores the filtering coefficients for a Bandpass filter that allows frequencies between 1950 and 2050 Hz through.
//This is because we want to locate a 2000 Hz signal. We chose a 31 coefficient filter to do this. The Matlab function was bandPass=fir1(100,[.39,.41])
float bandPass[101]={   2.0089131384901197e-03, 6.4032873499578040e-04, -1.7662310132503288e-03,    -1.8966231855838251e-03,    7.9038298787438197e-04, 2.8250866960543826e-03, 9.7274726769560108e-04, -2.8535932093218977e-03,    -3.2069079180517828e-03,    1.3777460739364028e-03, 5.0108857805228734e-03, 1.7369488778204004e-03, -5.0869489066624630e-03,    -5.6717260737981379e-03,    2.4066077632725297e-03, 8.6179538038498871e-03, 2.9352017836365030e-03, -8.4357135384937401e-03,    -9.2235281203421979e-03,    3.8369713729420702e-03, 1.3470983718227284e-02, 4.4992711557421761e-03, -1.2684979985041140e-02,    -1.3611937750688167e-02,    5.5600514925787251e-03, 1.9176967391055018e-02, 6.2956283333650978e-03, -1.7455271677881148e-02,    -1.8429536833842467e-02,    7.4103785848253561e-03, 2.5171457314971404e-02, 8.1418571044648731e-03, -2.2250769713411937e-02,    -2.3165078063428872e-02,    9.1879041586407240e-03, 3.0795414085640505e-02, 9.8318928762857697e-03, -2.6528873794684965e-02,    -2.7276081156801475e-02,    1.0686709091186523e-02, 3.5390668308456406e-02, 1.1166118673320274e-02, -2.9780034614308684e-02,    -3.0269173855075916e-02,    1.1725680290077527e-02, 3.8398491060813049e-02, 1.1981403290429368e-02, -3.1604759414221834e-02,    -3.1774940699058361e-02,    1.2176082500102338e-02, 3.9444917234878515e-02, 1.2176082500102338e-02, -3.1774940699058361e-02,    -3.1604759414221834e-02,    1.1981403290429368e-02, 3.8398491060813049e-02, 1.1725680290077527e-02, -3.0269173855075916e-02,    -2.9780034614308684e-02,    1.1166118673320274e-02, 3.5390668308456406e-02, 1.0686709091186523e-02, -2.7276081156801475e-02,    -2.6528873794684965e-02,    9.8318928762857697e-03, 3.0795414085640505e-02, 9.1879041586407240e-03, -2.3165078063428872e-02,    -2.2250769713411937e-02,    8.1418571044648731e-03, 2.5171457314971404e-02, 7.4103785848253561e-03, -1.8429536833842467e-02,    -1.7455271677881148e-02,    6.2956283333650978e-03, 1.9176967391055018e-02, 5.5600514925787251e-03, -1.3611937750688167e-02,    -1.2684979985041140e-02,    4.4992711557421761e-03, 1.3470983718227284e-02, 3.8369713729420702e-03, -9.2235281203421979e-03,    -8.4357135384937401e-03,    2.9352017836365030e-03, 8.6179538038498871e-03, 2.4066077632725297e-03, -5.6717260737981379e-03,    -5.0869489066624630e-03,    1.7369488778204004e-03, 5.0108857805228734e-03, 1.3777460739364028e-03, -3.2069079180517828e-03,    -2.8535932093218977e-03,    9.7274726769560108e-04, 2.8250866960543826e-03, 7.9038298787438197e-04, -1.8966231855838251e-03,    -1.7662310132503288e-03,    6.4032873499578040e-04, 2.0089131384901197e-03};

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

    //JMF Setup for using GPIO52 to measure the time is takes the processor to execute the ADCB interrupt
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;


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

    //JMF these next 3 lines tell the code to add the interrupts for ADCD, ADCA, and ADCB to the PIE table. This basically gives them the ability to have priority that they can execute their work on the processor.
    // PieVectTable.ADCD1_INT =&ADCD_ISR;
    // PieVectTable.ADCA1_INT = &ADCA_ISR;
    PieVectTable.ADCB1_INT = &ADCB_ISR;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

	init_serialSCIA(&SerialA,115200);

    // ex1.1
	//JMF instead of using the cpu timers to determine the period of how often the ADC will do its job, we want to use a PWM pin to do this. Here we are setting up EPWM 5 with a period value
	//that we determine is fitting for our ADCs to sample at. the input clock is set at 50 MHz, so we use the TBPRD counting bit field to determine the time span between samples
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (�pulse� is the same as �trigger�)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    //for ex2,3
    //EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    //for ex4
    //EPwm5Regs.TBPRD = 12500; // Set Period to .25ms sample. (4000Hz) Input clock is 50MHz.
    //JMF for last section of ex4
    EPwm5Regs.TBPRD = 5000; // Set Period to .1ms sample. (10000Hz) Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    // Ex1.2 set up ADCs
    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
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

    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4; //SOC0 will convert Channel you choose Does not have to be B0, Look at manual for ADCINB4, this is where 4 originates. Sourced from microphone spec sheet
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    //AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???; //SOC1 will convert Channel you choose Does not have to be B1
    //AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???; //SOC2 will convert Channel you choose Does not have to be B2
    //AdcbRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
    //AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???; //SOC3 will convert Channel you choose Does not have to be B3
    //AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCD
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0; // set SOC0 to convert pin D0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC0
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1; //set SOC1 to convert pin D1
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC1
    //AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
    //AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

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

    //turns on the ADC to the correct A B or D one to use?
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //refer to PIE channel Map
    //1.4d, ADCD
//    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    //EX3 ADCA
    //
    //EX4 ABCB
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;


	//init_serialSCIB(&SerialB,115200);
	init_serialSCIC(&SerialC,115200);
	init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global real-time interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            //prints the useful information from our filter that is then also shown on the oscilloscope as a reconstructed waveform after using setDAC()
			//serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            serial_printf(&SerialA,"Channel 1: %.3f, Channel 0: %.3f\r\n",ADCIND1Volts,yk);
            //yk2 is Y direction, yk1 is X direction
            //serial_printf(&SerialA,"Channel 1 (X hat): %.3f, Channel 0 (Y hat): %.3f\r\n",yk1,yk2);
            //ex4
            serial_printf(&SerialA,"Channel 1: %.3f\r\n",sound[0]);
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

    numSWIcalls++;
    
    DINT;

}

// cpu_timer0 ISR - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
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

    if ((numTimer0calls%50) == 0) {
		// Blink LaunchPad Red LED
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//1.3 set functions
//This function sets DACA to the voltage between 0V and 3V passed to this function.
//If outside 0V to 3V the output is saturated at 0V to 3V
//Example code
//float myu = 2.25;
//setDACA(myu); // DACA will now output 2.25 Volts
//they are used to turn the digital signal we have recieved and filtered back into a analog one so that we can show the signal on the oscilloscope
void setDACA(float dacouta0) {
    int16_t DACOutInt = 0;
    DACOutInt = (4096.0/3.0)*dacouta0; // perform scaling of 0 � almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;

    DacaRegs.DACVALS.bit.DACVALS = DACOutInt;
}

void setDACB(float dacoutb0) {
    int16_t DACOutInt = 0;
    DACOutInt = (4096.0/3.0)*dacoutb0; // perform scaling of 0 � almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;

    DacbRegs.DACVALS.bit.DACVALS = DACOutInt;
}

//E1.4
//adcd1 pie interrupt WITH NO FILERING
//__interrupt void ADCD_ISR (void) {
//    adcd0result = AdcdResultRegs.ADCRESULT0;
//    adcd1result = AdcdResultRegs.ADCRESULT1;
//
//    // Here covert ADCIND0 to volts
//    scaledadcd0result = (3.0/4096.0)*adcd0result;
//    // Here write voltages value to DACA
//    setDACA(scaledadcd0result);
//    // Print ADCIND0�s voltage value to TeraTerm every 100ms
//    //PWM period 1ms
//    ADCD1_count++;
//    if((ADCD1_count % 100) == 0){
//        UARTPrint = 1;
//        //Called every 100ms
//    }
//
//    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//}
//Ex2
//acdc1 pie interrupt with simple average filtering
//xk is the current ADC reading, xk_1 is the ADC reading one millisecond ago, xk_2 two milliseconds ago, etc
float xk = 0;
float xk_1 = 0;
float xk_2 = 0;
float xk_3 = 0;
float xk_4 = 0;
//b is the filter coefficients
//first array is the average filter coefficients

// Exercise 3
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

// Print ADCIND0 and ADCIND1�s voltage value to TeraTerm every 100ms
    //PWM period 1ms
    ADCA1_count++;
    if((ADCA1_count % 100) == 0){
        UARTPrint = 1;
    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
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
//// Print ADCIND0 and ADCIND1�s voltage value to TeraTerm every 100ms
//    //PWM period 1ms
//    ADCD1_count++;
//    if((ADCD1_count % 100) == 0){
//        UARTPrint = 1;
//        //Called every 100ms
//    }
//AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
//PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//}

__interrupt void ADCB_ISR (void) {
    //JMF Set GPIO52 pin high to measure on oscilloscope:
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;

    yk3 = 0;
    adcb0result = AdcbResultRegs.ADCRESULT0;
    // Here covert ADCIND0, ADCIND1 to volts
    sound[0] = (3.0/4096.0)*adcb0result;
    //JMF Runs 31 coefficient filtering for loop on low pass filter
//    for(int i = 0; i < SIZEOFARRAY; i++) {
//        yk3 += b[i]*sound[i];
//    }
    //JMF Run 31 coefficient filtering for loop for band pass filter (1750 to 2500 Hz allowed)
    for(int i = 0; i < SIZEOFARRAY; i++) {
        yk3 += bandPass[i]*sound[i];
    }
    //JMF allow sound values to continue into the array for further filtering
    for(int i = SIZEOFARRAY - 1; i >=1; i--){
        sound[i] = sound[i-1];
    }


// Here write yk1 to DACA channel and yk2 to DACB channel
    //JMF set un-filtered signal to DACA which will be read by the oscilloscope
    //setDACA(sound[0]);
    
    //JMF set 31 coefficient filtered low pass or band pass signal to DACA which will be read by the Oscilloscope in Ex4
    setDACA(yk3+1.5);
    
// Print ADCIND0 and ADCIND1�s voltage value to TeraTerm every 100ms
    //PWM period .25ms
    ADCB1_count++;
    if((ADCB1_count % 400) == 0){
        UARTPrint = 1;
    }
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    //JMF Set GPIO52 pin LOW so that time can be recorded on how long this interrupt took
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
		
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
	
//	if ((CpuTimer2.InterruptCount % 10) == 0) {
//		UARTPrint = 1;
//	}
}

