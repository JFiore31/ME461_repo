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


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);

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
int16_t X_ACCEL = 0;
int16_t Y_ACCEL = 0;
int16_t Z_ACCEL = 0;
int16_t temperature = 0;
int16_t X_GYRO = 0;
int16_t Y_GYRO = 0;
int16_t Z_GYRO = 0;
float scldXAccel = 0.0;
float scldYAccel = 0.0;
float scldZAccel = 0.0;
float scldXGyro = 0.0;
float scldYGyro = 0.0;
float scldZGyro = 0.0;

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
            //            serial_printf(&SerialA, "X_ACCEL:%.3f(g) Y_ACCEL:%.3f(g) Z_ACCEL:%.3f(g) \r\nX_GYRO:%.3f(deg per sec) Y_GYRO:%.3f(deg per sec) Z_GYRO %.3fdeg per sec(deg per sec)\r\n\n",scldXAccel, scldYAccel, scldZAccel, scldXGyro, scldYGyro, scldZGyro);
            //            serial_printf(&SerialA, "Wheel (L,R) %.3f %.3f\r\n",LeftWheel, RightWheel); //Lab6 Ex1
            //JMF Print current velocity of the two wheels
//            serial_printf(&SerialA, "Wheel velocity (L,R) %.3f %.3f\r\n",VLeftCur, VRightCur); //Lab6 Ex2
            serial_printf(&SerialA, "X= %.3f Y= %.3f bearing= %.3f\r\n",x, y, bearing);
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


    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    //JMF Exercise 3
    // call every 1ms
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

    if ((numTimer0calls%50) == 0) {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    if ((CpuTimer0.InterruptCount % 200) == 0) {
        UARTPrint = 1;
    }
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
    // 1200 counts per 1 wheel spin, ?count * (2pi (rad) / (400*30) (counts)) = ?rad. We want to return rad
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
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
    numTimer2calls++;

    //JMF store rad value of the wheels rotation
    LeftWheel = readEncLeft();
    RightWheel = readEncRight();

    //Turn rad into ft
    PosLeftCur = LeftWheel*leftFoot;
    PosRightCur = RightWheel*rightFoot;

    //JMF this interrupt is called every .004 s, so we are taking delta x over delta t which gives us velocity
    VLeftCur = (PosLeftCur - PosLeftPrev)/0.004;
    VRightCur = (PosRightCur - PosRightPrev)/0.004;
    //JMF pass back the velocity to the previous velocity
    PosLeftPrev = PosLeftCur;
    PosRightPrev = PosRightCur;

    //JMF first, calculate what the turn error is based on the two wheels velo and the turn desired
    et = VLeftCur - VRightCur + turn;

    // Controller Left
    //ek_L = refVelo - VLeftCur;
    //JMF this is the updated error to include turning.
    //Now we are comparing the reference to the actual velocity and if there is a turn desired we add it in
    ek_L = refVelo - VLeftCur - Kturn*et;
    //JMF solve for the integral error to improve motor function to actually get to vref
    Ik_L = Ik_1_L + 0.004*((ek_L+ek_1_L)/2.0);

    //Set uLeft to the respective gains and errors. This s the value we actually set the motors to
    uLeft = Kp*ek_L + Ki*Ik_L;
    //ek_1_L = ek_L;

    //JMF check if uLeft is saturated. This is a sign that we have integral windup which we want to prevent
    if(uLeft >= 10.0) {
        uLeft = 10.0;
        Ik_L = Ik_1_L;
    } else if(uLeft <= -10.0) {
        uLeft = -10.0;
        Ik_L = Ik_1_L;
    }
    // Controller Right
    //ek_R = refVelo - VRightCur;
    //JMF this is the updated error to include turning.
    //Now we are comparing the reference to the actual velocity and if there is a turn desired we add it in
    ek_R = refVelo - VRightCur + Kturn*et;
    //JMF solve for the integral error to improve motor function to actually get to vref
    Ik_R = Ik_1_R + 0.004*((ek_R+ek_1_R)/2.0);

    //Set uRight to the respective gains and errors. This is the value we actually set the motors to
    uRight = Kp*ek_R + Ki*Ik_R;
    //ek_1_R = ek_R;

    //JMF check if uRight is saturated. This is a sign that we have integral windup which we want to prevent
    if(uRight >= 10.0) {
        uRight = 10.0;
        Ik_R = Ik_1_R;
    } else if(uRight <= -10.0) {
        uRight = -10.0;
        Ik_R = Ik_1_R;
    }

    //JMF need to pass -uLeft since the motors are wired the same, but are orientated 180 deg from each other so their positive directions are inverted.
    setEPWM2A(uRight);
    setEPWM2B(-uLeft);
    //    if ((CpuTimer2.InterruptCount % 10) == 0) {
    //        UARTPrint = 1;
    //    }

    //JMF WIFI Commuications through Labview used to control robot:
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
    address = SpibRegs.SPIRXBUF; // JMF Discard this value, it is nothing from address write byte and the INT_STATUS which we don't care about
    X_ACCEL = SpibRegs.SPIRXBUF; // JMF X_ACCEL
    Y_ACCEL = SpibRegs.SPIRXBUF; // JMF Y_ACCEL
    Z_ACCEL = SpibRegs.SPIRXBUF; // JMF Z_ACCEL
    temperature = SpibRegs.SPIRXBUF; // JMF Discard this value, it is the temperature reading that we do not care about here. Just nice to take so we don't have to SS again.
    X_GYRO = SpibRegs.SPIRXBUF; // JMF X_GYRO
    Y_GYRO = SpibRegs.SPIRXBUF; // JMF Y_GYRO
    Z_GYRO = SpibRegs.SPIRXBUF; // JMF Z_GYRO
    //JMF X AND Y ACCEL SATURATED. GYRO SEEMS A LITLE OFF

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Set GPI66 high to end Slave Select.



    scldXAccel = X_ACCEL*(4.0/32768.); //JMF X_ACCEL is 16 bit signed int. We want to map this range of +-32768 to the range of +-4
    scldYAccel = Y_ACCEL*(4.0/32768.);
    scldZAccel = Z_ACCEL*(4.0/32768.);
    scldXGyro = X_GYRO*(250.0/32768.); //JMF X_Gyro is 16 bit signed int. We want to map this range of +-32768 to the range of +-250
    scldYGyro = Y_GYRO*(250.0/32768.);
    scldZGyro = Z_GYRO*(250.0/32768.);

    //EX2
    //    //JMF this interrupt is entered when we have sent over the amount of data values (in this case 3) to surpass the threshold to send to the DAN chip
    //    //JMF these are the values we read back from the Dan chip
    //    spivalue1 = SpibRegs.SPIRXBUF; // JMF Nothing signal from sending 0x00DA. Discard
    //    spivalue2 = SpibRegs.SPIRXBUF; // JMF ADC1 value from Dan Datasheet.
    //    spivalue3 = SpibRegs.SPIRXBUF; // JMF ADC2 value from Dan Datasheet.
    //
    //    GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO9 high to end Slave Select.
    //
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
}

void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
//JMF this only needs to be called once to set up SPI communication with MPU chip.
{
    int16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in between each transfer to 0. Also don�t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65, 66 which are also a part of the SPIB setup.
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
    // 50MHZ. And this setting divides that base clock to create SCLK�s period
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
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don�t think this is needed. Need to Test
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
   //JMF below are set the correct send values to each regigister we want to transmit to.
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
    //JMF temp used go read off the recieve FIFO
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
