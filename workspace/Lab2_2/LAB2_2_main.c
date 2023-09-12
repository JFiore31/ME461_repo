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

//JMF these variables are random that hold a lot of memory and slow down our code. The purpose of including them is to play around and show the importance of breakpoints in your code
float x1 = 6.0;
float x2 = 2.3;
float x3 = 7.3;
float x4 = 7.1;

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

//JMF worker function predefinition
//C makes us predefine functions for the compiler to work on our code. This functions are defined with the information of what they actually do after the main() function
void SetLEDRowsOnOff(int16_t rows);
int16_t ReadSwitches(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

//JMF new extern variable that tells us when the code enters the cuptimer2 interrupt. This is important because we want to print to serialprint based on how many times this interrupt is entered, so we want to count it
extern int32_t timer2IsInterrupt = 0;

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
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    //JMF we changed timer2 frequency to be .001 s so that we can increment the IsInterrupt very quickly and show that the human eye cannot see the LEDs blink when they have a frequency faster than .005 s
    //Because we use transistors to get this frequency, we can go to very fast rates and the limiting factor will be with the cpu
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    //comment about enabling only 2
//    CpuTimer0Regs.TCR.all = 0x4000;
//    CpuTimer1Regs.TCR.all = 0x4000;
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
	
	init_serialSCIB(&SerialB,115200);
	init_serialSCIC(&SerialC,115200);
	init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
			serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
			//JMF This must be set back to 0 or else it would continuously print which we don't want.
			//UARTPrint is how we make the program only print every interval we want it to. See timer2 interrupt for how it increments.
            UARTPrint = 0;
        }
    }
}

// worker function calls
void SetLEDRowsOnOff(int16_t rows){
    //JMF starting from the bottom row and moving to the top, this set of if, else statements turns on of off the rows of LEDs based on what the rows int is past to it
    //Because of how rows increments, this set up makes the rows "fill in" in a nice order where you can see the LED turn on "falling down" to fill the lower rows. This is achieved by using the bitwise and operator on the
    //rows int and the associated Hex values tied to those first 4 bits that we use to control the GPASET for out GIO pins connected to the LEDs
    if((rows & 0x1) == 0x1){
        GpioDataRegs.GPASET.bit.GPIO22 = 1;
        GpioDataRegs.GPESET.bit.GPIO130 = 1;
        GpioDataRegs.GPBSET.bit.GPIO60 = 1;
    } else {
        GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
        GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
    }
    if((rows & 0x2) == 0x2){
        GpioDataRegs.GPCSET.bit.GPIO94 = 1;
        GpioDataRegs.GPESET.bit.GPIO131 = 1;
        GpioDataRegs.GPBSET.bit.GPIO61 = 1;
    } else {
        GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;
        GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
    }
    if((rows & 0x4) == 0x4){
        GpioDataRegs.GPCSET.bit.GPIO95 = 1;
        GpioDataRegs.GPASET.bit.GPIO25 = 1;
        GpioDataRegs.GPESET.bit.GPIO157 = 1;
    } else {
        GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
        GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;
    }
    if((rows & 0x8) == 0x8){
        GpioDataRegs.GPDSET.bit.GPIO97 = 1;
        GpioDataRegs.GPASET.bit.GPIO26 = 1;
        GpioDataRegs.GPESET.bit.GPIO158 = 1;
    } else {
        GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
        GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
    }
    if((rows & 0x10) == 0x10){
        GpioDataRegs.GPDSET.bit.GPIO111 = 1;
        GpioDataRegs.GPASET.bit.GPIO27 = 1;
        GpioDataRegs.GPESET.bit.GPIO159 = 1;
    } else {
        GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;
        GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;
    }
}

int16_t ReadSwitches(void){
    int16_t switchStatus = 0;
    // button pressed (may be different than doc)

    //JMF This function uses the bitwise or operator to deliver the status of which switches are presses on our green board
    //A less effiecent way of doing this would be to create boolean values for each button, but by using bitwise logic, we can use the last 4 bits of a 16 bit number to tell if the button is pressed or not.
    //Each if statement corresponds to a button press since when they are open the GIO pins is not connected to ground, but when pressed it is. This is why we chose to set it == to 0 since ground is 0
    //Once in the if statement we use the Hex value corresponding to each bit in the first 4 bits of the number to determine if those buttons are pressed (namely 1,2,4, and 8)
    if (GpioDataRegs.GPADAT.bit.GPIO4 == 0) {
        switchStatus = switchStatus | 0x1;
    }

    if (GpioDataRegs.GPADAT.bit.GPIO5 == 0) {
        switchStatus = switchStatus | 0x2;
    }

    if (GpioDataRegs.GPADAT.bit.GPIO6 == 0) {
        switchStatus = switchStatus | 0x4;
    }

    if (GpioDataRegs.GPADAT.bit.GPIO7 == 0) {
        switchStatus = switchStatus | 0x8;
    }
    //At the end we return the now potentially edited switvchStatus 16 bit int
    return switchStatus;
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

    //JMF this is random code that is process intensive and meant to shw us how to use breakpoints. Float variables are large in storage so operating with them is difficult on the cpu, so breakpoints are helpful.
    x4 = x3 + 2.0;
    x3 = x4 + 1.3;
    x1 = 9 * x2;
    x2 = 34 * x3;

    //JMF this if statement used our readswitches return value to help us determine if buttons 2 and 3 are pressed in any configuratation. We do this using bit wise and operators and the == operator.
    //If buttons 2 and 3 are pressed in any configuration, we stop incrementing timer2IsInterrupt. This prevents the LED configuration from changing because it is also what is passed to the SetLEDRowsOnOff function
    //This value is what we use as "rows' as I talked about in a comment above
    //The value of the interrupts count is still stored and when the buttons are released it will go back to its normal incrementation from where it left off. This code allows you to pause incrementation though
    if ((ReadSwitches() & 0x6) == 0x6)
    {
        timer2IsInterrupt = timer2IsInterrupt;
    } else {
        timer2IsInterrupt++;
    }

    SetLEDRowsOnOff(timer2IsInterrupt);

    CpuTimer2.InterruptCount++;
	
    //JMF this modulos is set to 100 since we are running a very fast CPU frequency. This gives us the ability to execute things on the same cpu timer, but at different rates. This means a too fast cpu cannot be a problem
    //another solution would be to attached this if statement to a different CPU timer that was set with a lower frequency.
	if ((CpuTimer2.InterruptCount % 100) == 0) {
		UARTPrint = 1;
	}
}

