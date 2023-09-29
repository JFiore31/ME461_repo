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


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void ADCD_ISR (void);
// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t upDown = 1;
int16_t upDown_2 = 1;
float controleffort = 0;
float controleffort2 = 0;
//JMF predefine functions that will control the PWM signal for RC servo control. They take a float that will be mapped to the integer values allowed to control
//The motors.
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);


//Exercise 3 Predefinition
int16_t upDown_3 = 1;
float angle2 = 0;
float angle = 0;
void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);

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
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5);
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

    //JMF RC Servos PinMux setup. Set to 1 so that it is a PWM signal and not a GPIO. EPWM8A (EX 2)
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1);

    //JMF RC Servos PinMux setup. Set to 1 so that it is a PWM signal and not a GPIO. EPWM8B (EX 2)
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1);

    //JMF Noise Buzzer PinMux setup. Set to 1 so that it is a PWM signal and not a GPIO. EPWM9A (EX 3)
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5);


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

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 125000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

	init_serialSCIA(&SerialA,115200);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

//	init_serialSCIB(&SerialB,115200);
	init_serialSCIC(&SerialC,115200);
	init_serialSCID(&SerialD,115200);

	//JMF here we will set up the registers for the EPWM pin fields
    //We begin by setting the bit registers in out EPWM pins. These registers allow us to tell the processor what the period is, what the frequency divisor is
    //What to do when the count equals the period, etc.
	//It is required to have the technical reference to set these. On this table, values are given in binary as to what setting corresponds to what mode for each
	//register bit(s). We enter the decimal equivalent to set this on the right side of the equal sign.

	//JMF Set EPwm12 register to dim and brighten the LED in exercise 1
	EPwm12Regs.TBCTL.bit.FREE_SOFT = 2; //JMF anything with 1x bit (stop when counter completes a whole cycle)
	EPwm12Regs.TBCTL.bit.CTRMODE = 0; //JMF this sets to control up mode which tells the counter to go up every clock cycle
	EPwm12Regs.TBCTL.bit.CLKDIV = 0; //JMF this value determines if the input frequency will be divided by a value. We chose 1 since we want to 50 MHz Freq.
	EPwm12Regs.TBCTL.bit.PHSEN = 0; //JMF count down after the synchronization event

	EPwm12Regs.TBCTR = 0; //JMF sets the counter bit field to 0 to start before counting

	EPwm12Regs.TBPRD = 2500; //JMF to set period that the counter will count to

	EPwm12Regs.CMPA.bit.CMPA = 0; //set duty cycle to 0

	EPwm12Regs.AQCTLA.bit.CAU = 1; //JMF Tells the processor what to do when TBCTR = CMPA. In this case we chose to clear and set the EPWM value to LOW.
    EPwm12Regs.AQCTLA.bit.ZRO = 2; //JMF tell the processor what to do when TBCTR - 0. In this case we chose to set the EPWM value to HIGH.

    EPwm12Regs.TBPHS.bit.TBPHS = 0; //JMF sets the signal phase to 0. not sure if necessary

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

    //JMF Set EPwm8 register for Exercise 2 servos
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2; //anything with 1x bit
    EPwm8Regs.TBCTL.bit.CTRMODE = 0;  //JMF this sets to control up mode which tells the counter to go up every clock cycle
    EPwm8Regs.TBCTL.bit.CLKDIV = 4; //JMF this value determines if the input frequency will be divided by a value. We chose 16 since we want to 50/16 MHz Freq.
    EPwm8Regs.TBCTL.bit.PHSEN = 0; //JMF count down after the synchronization event

    EPwm8Regs.TBCTR = 0; //JMF sets the counter bit field to 0 to start before counting

    EPwm8Regs.TBPRD = 62500; //JMF to set period that the counter will count to

    EPwm8Regs.CMPA.bit.CMPA = 2500; //JMF set duty cycle to 0 different then before because the mapping is different due to CLKDIV
    EPwm8Regs.CMPB.bit.CMPB = 2500; //JMF set duty cycle to 0 different then before because the mapping is different due to CLKDIV

    EPwm8Regs.AQCTLA.bit.CAU = 1; //JMF Tells the processor what to do when TBCTR = CMPA. In this case we chose to clear and set the EPWM value to LOW.
    EPwm8Regs.AQCTLA.bit.ZRO = 2; //JMF tell the processor what to do when TBCTR - 0. In this case we chose to set the EPWM value to HIGH.
    EPwm8Regs.AQCTLB.bit.CBU = 1; //JMF Tells the processor what to do when TBCTR = CMPA. In this case we chose to clear and set the EPWM value to LOW.
    EPwm8Regs.AQCTLB.bit.ZRO = 2; //JMF tell the processor what to do when TBCTR - 0. In this case we chose to set the EPWM value to HIGH.

    EPwm8Regs.TBPHS.bit.TBPHS = 0; //JMF sets the signal phase to 0. not sure if necessary

    //JMF Set EPwm9 register for buzzer exercise 3
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 2; //anything with 1x bit
    EPwm9Regs.TBCTL.bit.CTRMODE = 0; //JMF this sets to control up mode which tells the counter to go up every clock cycle
    EPwm9Regs.TBCTL.bit.CLKDIV = 1; //JMF this value determines if the input frequency will be divided by a value. We chose 2 since we want to 25 MHz Freq.
    EPwm9Regs.TBCTL.bit.PHSEN = 0; //JMF count down after the synchronization event

    EPwm9Regs.TBCTR = 0; //JMF sets the counter bit field to 0 to start before counting

    EPwm9Regs.TBPRD = 2500; //JMF to set period that the counter will count to

//    EPwm9Regs.CMPA.bit.CMPA = 0; //set duty cycle to 0

    EPwm9Regs.AQCTLA.bit.CAU = 0; //Set to 0 to do nothing
    EPwm9Regs.AQCTLA.bit.ZRO = 3; //JMF set to 3 to toggle the LOW or HIGH output of the PMW. This is to show on the oscilliscope the operation signal

    EPwm9Regs.TBPHS.bit.TBPHS = 0; //JMF sets the signal phase to 0. not sure if necessary

    //JMF this is to disable pull down resistors for power consumption purposes
    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1; // For EPWM9A
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;

	//global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
			serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
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
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%25) == 0) {
        //displayLEDletter(LEDdisplaynum);
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

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    int16_t increment = 0;
    EPwm9Regs.TBPRD = piratesarray[CpuTimer1.InterruptCount];
    if(CpuTimer1.InterruptCount < SONG_LENGTH){
        CpuTimer1.InterruptCount++;
        increment++;
    }

    if(CpuTimer1.InterruptCount == SONG_LENGTH){
        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0);
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
    }
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
	
	if ((CpuTimer2.InterruptCount % 10) == 0) {
		UARTPrint = 1;
	}

	//Exercise 1
	//JMF this code is made to dim and brighten the LED on the breadboard
	//We have made a state machine that when we are counting up (upDown variable) will increment until we reach the period of the EPWM register
	//Once we reach this period, we will count back down then dimming the LED. This creates a saw tooth graph of LED brightness versus time.
//	if (upDown == 1){
//	    EPwm12Regs.CMPA.bit.CMPA++;
//	    if (EPwm12Regs.CMPA.bit.CMPA == EPwm12Regs.TBPRD) {
//	        upDown = 0;
//	    }
//	} else {
//	    EPwm12Regs.CMPA.bit.CMPA--;
//        if (EPwm12Regs.CMPA.bit.CMPA == 0) {
//            upDown = 1;
//        }
//	}

	// JMF Exercise 2.3
	//similar to brightening and dimming the LED, this function counts up and down in a state machine to control the wheel speed and direction from full forward
	//to full backward in a linear progression and regression. the setEPWMxx functions are used to do this and the added logic in this function is the upDown2
	//variable that is used in the state machine that is incremented or decremented
	//We notice due to motor lag this is not as linear input to output as the LED
	if (upDown_2 == 1){
	    controleffort2 = controleffort2 +0.01;
	        if (controleffort2 > 10) {
	            upDown_2 = 0;
	        }
	    } else {
	        controleffort2 = controleffort2 - 0.01;
	        if (controleffort2 < -10) {
	            upDown_2 = 1;
	        }
	    }
	setEPWM2A(controleffort2);
	setEPWM2B(controleffort2);

	// Exercise 3
	//JMF this state machine is made to make the DC servos go back and forth between their max 90 degrees and min -90 degrees orientations
	//It is similar to the LED dimming/brightening and the motor Full Speed forward/backward increment and decrement logic
	//We used a variable upDown_3 to do this
    if (upDown_3 == 1){
        angle2 = angle2 +0.05;
            if (angle2 > 90) {
                upDown_3 = 0;
            }
        } else {
            angle2 = angle2 - 0.05;
            if (angle2 < -90) {
                upDown_3 = 1;
            }
        }
    setEPWM8A_RCServo(angle2);
    setEPWM8B_RCServo(angle2);
}
// JMF Exercise 2.2
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

// Exercise 3
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


