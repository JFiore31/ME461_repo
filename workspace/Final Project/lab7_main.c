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
#include "xy.h"

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

// dti lab5 : function prototypes
__interrupt void SPIB_isr(void); // dti lab5 : SPI interrupt function
int16_t ramp_up_down(int16_t var, int16_t min, int16_t max, int16_t incr); // dti lab5 : function to ramp up and down a variable between a maximum and a minimum value at a given increment rate
void setupSpib(void); // dti lab5 : Ex3 SPIB setup for MPU9250

// dti lab6: function prototypes
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setDutyEPWM2A(float controleffort);
void setDutyEPWM2B(float controleffort);
void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);

// dti lab7: function prototypes
__interrupt void ADCD_ISR (void);
__interrupt void ADCA_ISR (void);
__interrupt void ADCB_ISR (void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// dti lab5 : global variables
int16_t spivalue1 = 0; // dti lab5 : Ex1.2
int16_t spivalue2 = 0; // dti lab5 : Ex1.2
int16_t spivalue3 = 0; // dti lab5 : Ex2
int16_t updown = 0; // dti lab5 : Ex2
int16_t min_DAN_pwmdty = 0; // dti lab5 : Ex2
int16_t max_DAN_pwmdty = 3000; // dti lab5 : Ex2
int16_t DAN_pwmdty = 0; // dti lab5 : Ex2
float toTeraTerm2 = 0.0; // dti lab5 : Ex2
float toTeraTerm3 = 0.0; // dti lab5 : Ex2
// dti lab5 : MPU variables
int16_t accel_xraw = 0;
int16_t accel_yraw = 0;
int16_t accel_zraw = 0;
int16_t temperature = 0;
int16_t gyro_xraw = 0;
int16_t gyro_yraw = 0;
int16_t gyro_zraw = 0;
float accelx = 0;
float accely = 0;
float accelz = 0;
float gyrox = 0;
float gyroy = 0;
float gyroz = 0;
float gyrorate_dot = 0;
float gyro_value_1 = 0;

// dti : lab6 global variables
float LeftWheel_Enc = 0.0;
float RightWheel_Enc = 0.0;
float vel_Left = 0;
float vel_Right = 0;
float RightWheel_Enc_1 = 0;
float LeftWheel_Enc_1 = 0;
//float RadPerFeetLWheel = 5.01; // [rad/ft] wheel turn in radians per feet of travel
//float RadPerFeetRWheel = 5.13; // [rad/ft] wheel turn in radians per feet of travel
float uLeft_k = 0.0;
float uRight_k = 0.0;
float posLeft_k = 0.0;
float posLeft_k_1 = 0.0;
float posRight_k = 0.0;
float posRight_k_1 = 0.0;
float vLeftk = 0.0;
float vRightk = 0.0;
float Ki = 25.0;
float Kp = 3.0;
float ILeft_k = 0.0;
float eLeft_k = 0.0;
float eLeft_k_1 = 0.0;
float Vref = 0;
float ILeft_k_1 = 0.0;
float IRight_k = 0.0;
float eRight_k = 0.0;
float eRight_k_1 = 0.0;
float IRight_k_1 = 0.0;
float turn = 0.0;
float eTurn = 0.0;
float Kturn = 3.0;

// Ex5 : lab6 define communication variables
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

// Ex6 : define pose calculation variables
float WR = 0.56759;
float Rwh = 0.19460;
float th_avg = 0;
float th_avg_dot = 0;
float vx_k = 0;
float vy_k = 0;
float vx_k_1 = 0;
float vy_k_1 = 0;

// dti lab7 : variables from lab 4
int32_t count_ADCD1 = 0; // dti : Ex1.4.f to count the number of interrupt calls of ADCD1
int16_t adcdin0_raw = 0; // dti : Ex1.4.e read ADCDIN0 raw value
int16_t adcdin1_raw = 0; // dti : Ex1.4.e read ADCDIN1 raw value
float adcdin0_result = 0; // dti : Ex1.4.e read ADCDIN0 value in volts (0 - 3.0)

int32_t count_ADCA1 = 0; // dti : Ex3 counter for ADCA1 interrupt calls
int32_t countSPIB = 0; // dti : lab 7, count SPIB interrupt calls
int16_t adcain2_raw = 0; // dti : Ex3 read ADCAIN0 raw value
int16_t adcain3_raw = 0; // dti : Ex3 read ADCAIN1 raw value
float adcain2_result = 0; // dti : Ex3 read ADCAIN0 value in volts (0 - 3.0)
float adcain3_result = 0; // dti : Ex3 read ADCAIN0 value in volts (0 - 3.0)

int32_t count_ADCB1 = 0; //dti : Ex4 to count the number of interrupt calls of ADCB1
int16_t adcbin0_raw = 0; // dti : Ex4 read ADCBIN0 raw value
float adcbin0_result = 0; // dti : Ex4 read ADCBIN0 value in volts (0 - 3.0)

// dti lab7 ex2 Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -0.64; //-0.65; //-0.63; //-0.60; //-0.699999988;
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_value_1 = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheel = 0;
float RightWheel = 0;
float ubal = 0;
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

// dti : filter constants for Kalman filtering
float K1 = -60.0;
float K2 = -8; //-11; // -4.5;
float K3 = -1.1; //-0.69; //-1.1;
float K4 = -0.1; //-0.05; //-0.1;

//DTI EX. 7 Turn Control
float WhlDiff = 0.0;
float WhlDiff_1 = 0.0;
float vel_WhlDif = 0.0;
float vel_WhlDiff_1 = 0.0;
float turnref = 0.0;
float errorDiff = 0.0;
float errorDiff_1 = 0.0;
float uTurn = 0.0;
float KpTurn = 3.0;
float KiTurn = 20.0;
float KdTurn = 0.08;
float iK_Diff = 0.0;
float iK_Diff_1 = 0.0;
float dK_Diff = 0.0;
float dK_Diff_1 = 0.0;
float turnrate = 0.0;
float turnrate_1 = 0.0;
float turnref_1 = 0.0;
float Segbot_refSpeed = 0.0;
float KpSpeed = 0.35;
float KiSpeed = 1.5;
float eSpeed = 0.0;
float eSpeed_1 = 0.0;
float ForwardBackwardCommand = 0.0;
float IK_eSpeed = 0.0;
float IK_eSpeed_1 = 0.0; //IK_espeed old used for trapezoidal rule

// dti : project
uint16_t segMode = 0;
float bal_at_tilt = 0.24;
float servoDefault = -90.0; // dti : minimum servo angle (castor position)
float servoMax = 40.0; // dti : maximum servo angle (segbot position)
float controlServoA = -90.0; // dti: control input for EPWM8A
float controlServoB = -90.0; // dti: control input for EPWM8B
float servoAngle = 0.0; // dti : servo angle to receive from labVIEW
int16_t robotState = 5;
int16_t stateCount = 0;
int16_t danNear = 0;
int16_t state5Count = 0;
float Segbot_refSpeedLV = 0.0;
float turnrateLV = 0.0;
float bal_at_tiltLV = 0.0;

#define SIZEOFARRAY 22 //dti Ex2 Size of FIR coefficient array
#define SIZEOFARRAY31 32 //dti Ex4 Size of 31st order FIR coefficient array
#define SIZEOFARRAY_BP 101 // dti Ex4 Size of a high order bandpass filter coefficient array
//float b[SIZEOFARRAY] = {3.3833240118424500e-02,
//              2.4012702387971543e-01,
//              4.5207947200372001e-01,
//              2.4012702387971543e-01,
//              3.3833240118424500e-02}; // 4th order FIR, cutoff frequency 0.1 Nyquist (set SIZEOFARRAY to 5!)
float b[SIZEOFARRAY]={-2.3890045153263611e-03,
                      -3.3150057635348224e-03,
                      -4.6136191242627002e-03,
                      -4.1659855521681268e-03,
                      1.4477422497795286e-03,
                      1.5489414225159667e-02,
                      3.9247886844071371e-02,
                      7.0723964095458614e-02,
                      1.0453473887246176e-01,
                      1.3325672639406205e-01,
                      1.4978314227429904e-01,
                      1.4978314227429904e-01,
                      1.3325672639406205e-01,
                      1.0453473887246176e-01,
                      7.0723964095458614e-02,
                      3.9247886844071371e-02,
                      1.5489414225159667e-02,
                      1.4477422497795286e-03,
                      -4.1659855521681268e-03,
                      -4.6136191242627002e-03,
                      -3.3150057635348224e-03,
                      -2.3890045153263611e-03}; // 21st order FIR, cutoff frequency 0.15 Nyquist (set SIZEOFARRAY to 22!)
float b31[SIZEOFARRAY31]={   -6.3046914864397922e-04,
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
                             -6.3046914864397922e-04};  // 31st order FIR, cutoff frequency 0.25 Nyquist (set SIZEOFARRAY31 to 32!)

float bp[SIZEOFARRAY_BP]={  2.0089131384901197e-03,
                            6.4032873499578040e-04,
                            -1.7662310132503288e-03,
                            -1.8966231855838251e-03,
                            7.9038298787438197e-04,
                            2.8250866960543826e-03,
                            9.7274726769560108e-04,
                            -2.8535932093218977e-03,
                            -3.2069079180517828e-03,
                            1.3777460739364028e-03,
                            5.0108857805228734e-03,
                            1.7369488778204004e-03,
                            -5.0869489066624630e-03,
                            -5.6717260737981379e-03,
                            2.4066077632725297e-03,
                            8.6179538038498871e-03,
                            2.9352017836365030e-03,
                            -8.4357135384937401e-03,
                            -9.2235281203421979e-03,
                            3.8369713729420702e-03,
                            1.3470983718227284e-02,
                            4.4992711557421761e-03,
                            -1.2684979985041140e-02,
                            -1.3611937750688167e-02,
                            5.5600514925787251e-03,
                            1.9176967391055018e-02,
                            6.2956283333650978e-03,
                            -1.7455271677881148e-02,
                            -1.8429536833842467e-02,
                            7.4103785848253561e-03,
                            2.5171457314971404e-02,
                            8.1418571044648731e-03,
                            -2.2250769713411937e-02,
                            -2.3165078063428872e-02,
                            9.1879041586407240e-03,
                            3.0795414085640505e-02,
                            9.8318928762857697e-03,
                            -2.6528873794684965e-02,
                            -2.7276081156801475e-02,
                            1.0686709091186523e-02,
                            3.5390668308456406e-02,
                            1.1166118673320274e-02,
                            -2.9780034614308684e-02,
                            -3.0269173855075916e-02,
                            1.1725680290077527e-02,
                            3.8398491060813049e-02,
                            1.1981403290429368e-02,
                            -3.1604759414221834e-02,
                            -3.1774940699058361e-02,
                            1.2176082500102338e-02,
                            3.9444917234878515e-02,
                            1.2176082500102338e-02,
                            -3.1774940699058361e-02,
                            -3.1604759414221834e-02,
                            1.1981403290429368e-02,
                            3.8398491060813049e-02,
                            1.1725680290077527e-02,
                            -3.0269173855075916e-02,
                            -2.9780034614308684e-02,
                            1.1166118673320274e-02,
                            3.5390668308456406e-02,
                            1.0686709091186523e-02,
                            -2.7276081156801475e-02,
                            -2.6528873794684965e-02,
                            9.8318928762857697e-03,
                            3.0795414085640505e-02,
                            9.1879041586407240e-03,
                            -2.3165078063428872e-02,
                            -2.2250769713411937e-02,
                            8.1418571044648731e-03,
                            2.5171457314971404e-02,
                            7.4103785848253561e-03,
                            -1.8429536833842467e-02,
                            -1.7455271677881148e-02,
                            6.2956283333650978e-03,
                            1.9176967391055018e-02,
                            5.5600514925787251e-03,
                            -1.3611937750688167e-02,
                            -1.2684979985041140e-02,
                            4.4992711557421761e-03,
                            1.3470983718227284e-02,
                            3.8369713729420702e-03,
                            -9.2235281203421979e-03,
                            -8.4357135384937401e-03,
                            2.9352017836365030e-03,
                            8.6179538038498871e-03,
                            2.4066077632725297e-03,
                            -5.6717260737981379e-03,
                            -5.0869489066624630e-03,
                            1.7369488778204004e-03,
                            5.0108857805228734e-03,
                            1.3777460739364028e-03,
                            -3.2069079180517828e-03,
                            -2.8535932093218977e-03,
                            9.7274726769560108e-04,
                            2.8250866960543826e-03,
                            7.9038298787438197e-04,
                            -1.8966231855838251e-03,
                            -1.7662310132503288e-03,
                            6.4032873499578040e-04,
                            2.0089131384901197e-03}; // dti : High order bandpass filter


/*// dti : Ex2 - Variables for filtering
float xk = 0; // dti : current ADC value
float xk_1 = 0; // dti : ADC value 1 ms ago
float xk_2 = 0; // dti : ADC value 2 ms ago
float xk_3 = 0; // dti : ADC value 3 ms ago
float xk_4 = 0; // dti : ADC value 4 ms ago*/
//yk is the filtered value
float yk = 0; // dti : average of 5 ADC values
float yk2 = 0; //dti: Ex3 filtered value for adca2
float yk3 = 0; //dti: Ex3 filtered value for adca3
//b is the filter coefficients
// float b[5] = {0.2,0.2,0.2,0.2,0.2}; // 0.2 is 1/5th therefore a 5 point average
float xk[SIZEOFARRAY] = {0}; //dti: Ex2: array for higher order FIR filters
float xk2[SIZEOFARRAY] = {0}; //dti: Ex3: array for higher order FIR filters
float xk3[SIZEOFARRAY] = {0}; //dti: Ex3: array for higher order FIR filters

float ykb0 = 0; // dti : Ex4 filtered microphone voltage
//float xkb0[SIZEOFARRAY31] = {0}; //dti: Ex4: array for higher order FIR filters for frequency filtering
float xkb0[SIZEOFARRAY_BP] = {0}; //dti: Ex4: array for higher order band pass filter


#define SIZEOFARRAY_PATH 20
float pathpointsX[SIZEOFARRAY_PATH]={-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.};
float pathpointsY[SIZEOFARRAY_PATH]={-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.,-1.};
int path_input_count = 0;
int16_t XYPoint = 0;
bool path_end_recieved = false;


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

    // dti: set up EPWM8A and B
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1); // dti: PinMux to 1 so that it works as a EPWM8A and not as a GPIO pin
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1); // dti: PinMux to 1 so that it works as a EPWM8B and not as a GPIO pin

    // dti: set up EPWM9A
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5); // dti: PinMux to 1 so that it works as a EPWM9A

    // disable pullup resistors
    EALLOW; //
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    EDIS;

    EPwm9Regs.TBCTL.bit.CLKDIV = 1;// dti: clock divide as given
    EPwm9Regs.TBCTL.bit.CTRMODE = 0; // dti: Count up mode
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 2; // dti: Free soft emulation mode to free run so PWM continues when reaching breakpoint
    EPwm9Regs.TBCTL.bit.PHSEN = 0; // dti: disable phase loading
    EPwm9Regs.TBCTR = 0;  //dti: start timer at 0
    EPwm9Regs.TBPRD = 2500; //dti: initializing period of pwm signal to 20 KHz for clock frequency of 50 MHz
    //EPwm9Regs.CMPA.bit.CMPA = 0; //dti: comparing count for duty cycle
    EPwm9Regs.AQCTLA.bit.CAU = 0; // dti: disabling action when TBCTR = CMPA on Up Count
    EPwm9Regs.AQCTLA.bit.ZRO = 3; // dti: Action When TBCTR = 0: Toggle EPWMxA output
    EPwm9Regs.TBPHS.bit.TBPHS = 0; // dti: set the phase to zero

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
    PieVectTable.ADCD1_INT = &ADCD_ISR; // dti lab7: add ADCD interrupt to vector table
    PieVectTable.ADCA1_INT = &ADCA_ISR; // dti lab7: add ADCA interrupt to vector table
    PieVectTable.ADCB1_INT = &ADCB_ISR; // dti lab7: add ADCB interrupt to vector table

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    // dti lab5 : add SPIB_isr interrupt function to Pie Vector Table
    PieVectTable.SPIB_RX_INT = &SPIB_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000); // dti lab5 :  Ex3 transmit 16 bit values to MPU9250 every 1 ms
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000); // dti lab6 : Ex1 interrupt every 4 ms
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);

    // dti lab7: EPWM settings for ADC triggering
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD//dti: Ex 1.1 Enable event time base-counter equal to period
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)// dti: Ex1.1 Generate the EPWMxSOCA pulse on the first event: ETPS[SOCACNT] = 0,1
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // dti lab7: Set Period to 1ms sample. Input clock is 50MHz.

    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode //dti: Ex1.1 just good old up count mode
    EDIS;

    //dti lab7: given code from lab 4
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

    //Many statements commented out, To be used when using ADCA or ADCB

    // dti lab7: enable ADCA to sample joystick readings
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x2; //set SOC0 to convert pin 26 //dti: Ex3 Select adcain2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0D; // EPWM5 ADCSOCA will trigger SOC0 //dti: Ex3 Select ePWM5 for ADCSOCA
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x3; //set SOC1 to convert pin 29 //dti: Ex3 Select adcain3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0D; // EPWM5 ADCSOCA will trigger SOC1 //dti: Ex3 Select ePWM5 for ADCSOCA
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //    //dti lab7: enable ADCB to sample microphone
    //    //ADCB
    //    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0x4; //SOC0 will convert Channel you choose Does not have to be B0//dti: Ex4 Select adcinb4
    //    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0D; // dti: Ex4 EPWM5 ADCSOCA used to trigger SOC0
    //    // dti: Ex4 SOC1-3 not used
    //    //    AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???; //SOC1 will convert Channel you choose Does not have to be B1
    //    //    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    //    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    //    //    AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???; //SOC2 will convert Channel you choose Does not have to be B2
    //    //    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    //    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC2
    //    //    AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???; //SOC3 will convert Channel you choose Does not have to be B3
    //    //    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    //    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will trigger SOC3
    //    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0x0; //dti: Ex3 SOC0 is last that is converted and will set INT1 flag ADCB1
    //    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    //    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //
    //    //ADCD
    //
    //    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0x0; // set SOC0 to convert pin D0 //dti: Ex1.2 Select adcdin0
    //    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0D; // EPWM5 ADCSOCA will trigger SOC0 //dti: Ex1.2 Select ePWM5 for ADCSOCA
    //    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 0x1; //set SOC1 to convert pin D1 //dti: Ex1.2 Select adcdin1
    //    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0D; // EPWM5 ADCSOCA will trigger SOC1 //dti: Ex1.2 Select ePWM5 for ADCSOCA
    //
    //    //AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
    //    //AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    //AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    //    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    //    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    //
    //    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 0x1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1 // dti : Ex1.2 1h EOC1 is trigger for ADCINT1
    //    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    //    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    // dti lab7: initialization of EPWM2A and EPWM2B
    // dti lab7: set up EPWM2A and B
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); // dti: PinMux to 1 so that it works as a EPWM2A and not as a GPIO pin
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); // dti: PinMux to 1 so that it works as a EPWM2B and not as a GPIO pin

    EPwm2Regs.TBCTL.bit.CLKDIV = 0;// dti: clock divide
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // dti: Count up mode
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; // dti: Free soft emulation mode to free run so PWM continues when reaching breakpoint
    EPwm2Regs.TBCTL.bit.PHSEN = 0; // dti: disable phase loading
    EPwm2Regs.TBCTR = 0;  //dti: start timer at 0
    EPwm2Regs.TBPRD = 2500; //dti: set period of pwm signal to 20 KHz for clock frequency of 50 MHz
    EPwm2Regs.CMPA.bit.CMPA = 0; //dti: comparing count for duty cycle
    EPwm2Regs.AQCTLA.bit.CAU = 1; // dti: clear the signal bit when TBCTR register is equal to CMPA
    EPwm2Regs.AQCTLA.bit.ZRO = 2; // dti: forces EPWMxA high when TBCTR is zero
    EPwm2Regs.TBPHS.bit.TBPHS = 0; // dti: set the phase to zero
    EPwm2Regs.AQCTLB.bit.CBU = 1; // dti: additional register initialization for EPWM2B
    EPwm2Regs.AQCTLB.bit.ZRO = 2; // dti: forces EPWMxA high when TBCTR is zero
    EPwm2Regs.CMPB.bit.CMPB = 0; // dti: additional register initialization for EPWM2B

    // dti: initialization of EPWM8A and EPWM8B
    EPwm8Regs.TBCTL.bit.CTRMODE = 0; // dti: Count up mode
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2; // dti: Free soft emulation mode to free run so PWM continues when reaching breakpoint
    EPwm8Regs.TBCTL.bit.PHSEN = 0; // dti: disable phase loading
    EPwm8Regs.TBCTR = 0;  //dti: start timer at 0
    EPwm8Regs.TBPRD = 62500; //dti: set period of pwm signal for Hz frequency running on a clock frequency of 50 MHz
    EPwm8Regs.TBCTL.bit.CLKDIV = 4;// dti: clock divide to 16 to accomodate slow carrier frequency
    EPwm8Regs.CMPA.bit.CMPA = 0; //dti: comparing count for duty cycle
    EPwm8Regs.AQCTLA.bit.CAU = 1; // dti: clear the signal bit when TBCTR register is equal to CMPA
    EPwm8Regs.AQCTLA.bit.ZRO = 2; // dti: forces EPWMxA high when TBCTR is zero
    EPwm8Regs.TBPHS.bit.TBPHS = 0; // dti: set the phase to zero
    EPwm8Regs.AQCTLB.bit.CBU = 1; // dti: additional register initialization for EPWM8B
    EPwm8Regs.AQCTLB.bit.ZRO = 2; // dti: forces EPWMxA high when TBCTR is zero
    EPwm8Regs.CMPB.bit.CMPB = 0; // dti: additional register initialization for EPWM8B

    // dti: disable the pull up resistors
    EALLOW; //
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    EDIS;

    init_eQEPs();

    setupSpib();
    //  //dti lab5: Ex1.1 given Code
    //  GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    //  GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    //  GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    //  GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    //  GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    //  GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    //  GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //dti lab5: Ex1.1 Set GPIO63 pin to SPISIMOB
    //  GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //dti lab5: Ex1.1 Set GPIO64 pin to SPISOMIB
    //  GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //dti lab5: Ex1.1 Set GPIO65 pin to SPICLKB
    //  EALLOW;
    //  GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    //  GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    //  GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    //  GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //  GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //  GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    //  EDIS;
    //  // ---------------------------------------------------------------------------
    //  SpibRegs.SPICCR.bit.SPISWRESET = 0x0; // dti lab5: Ex1.1 Put SPI in Reset
    //  SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    //  SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    //  SpibRegs.SPICTL.bit.MASTER_SLAVE = 0x1; // dti lab5: Ex1.1: Set to SPI Master
    //  SpibRegs.SPICCR.bit.SPICHAR = 0xF; // dti lab5: Ex1.1 Set to transmit and receive 16-bits each write to SPITXBUF
    //  SpibRegs.SPICTL.bit.TALK = 0x1; // dti lab5: Ex1.1 Enable transmission
    //  SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    //  SpibRegs.SPICTL.bit.SPIINTENA = 0x0; // dti lab5: Ex1.1 Disables the SPI interrupt
    //  SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 0x31; //dti lab5: Ex1.1 divide base clk rate by 49(=0x31)+1, Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    //  // 50MHZ. And this setting divides that base clock to create SCLK’s period
    //  SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    //  SpibRegs.SPIFFTX.bit.SPIRST = 0x1;// dti lab5: Ex1.1 Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    //  SpibRegs.SPIFFTX.bit.SPIFFENA = 0x1; // dti lab5: Ex1.1 Enable SPI FIFO enhancements
    //  SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    //  SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    //  SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    //  SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    //  SpibRegs.SPIFFRX.bit.RXFFINTCLR = 0x1; // dti lab5: Ex1.1 Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    //  SpibRegs.SPIFFRX.bit.RXFFIENA = 0x1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    //  SpibRegs.SPIFFCT.bit.TXDLY = 0x10; // dti lab5: Ex1.1 Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    //  SpibRegs.SPICCR.bit.SPISWRESET = 0x1; // dti lab5: Ex1.1 Pull the SPI out of reset
    //  SpibRegs.SPIFFTX.bit.TXFIFO = 0x1; // dti lab5: Ex1.1 Release transmit FIFO from reset.
    //  SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    //  SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
    //  SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; // dti lab5: Ex1.1 Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6; // dti lab5 : Ex1.2 SCIB

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    // dti lab5: Ex1.2 Enable SPIB_RX
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;

    // dti lab7: Enable ADCD1 in the PIE: Group 1 interrupt 6
    //PieCtrlRegs.PIEIER1.bit.INTx6 = 1; // dti : commented for ex3
    // dti lab7: Enable ADCA1 in the PIE: Group 1 interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    // dti lab7: Enable ADCB1 in the PIE: Group 1 interrupt 2
    // PieCtrlRegs.PIEIER1.bit.INTx2 = 1;

    //init_serialSCIB(&SerialB,115200);
    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"Joy1:%.2f Joy2:%.2f accelz:%.2f gyrox:%.2f readEnc_Left:%.2f readEnc_Right:%.2f tilt_value:%.2f\r\n",adcain2_result, adcain3_result, accelz, gyrox, LeftWheel_Enc, RightWheel_Enc, tilt_value);
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

////     dti : Ex5 lab 6 receive 8 values from LabVIEW
//        if (NewLVData == 1) {
//            NewLVData = 0;
//            Segbot_refSpeedLV = fromLVvalues[0];
//            turnrateLV = fromLVvalues[1];
//            Rwh = fromLVvalues[2]; //dti Radius of wheels from LabView
//            WR = fromLVvalues[3]; // dti Wheelbase from Labview
//            servoAngle = fromLVvalues[4];
//            bal_at_tiltLV = fromLVvalues[5];
//            printLV7 = fromLVvalues[6];
//            printLV8 = fromLVvalues[7];
//        }

//    dti : Ex5 lab 6 receive 8 values from LabVIEW
    if (NewLVData == 1) {
        NewLVData = 0;

        // check identifier first entry from Labview
        // if path mode, and new path points are being received
        if (fromLVvalues[0] >= 3 && pathpointsX[path_input_count] != fromLVvalues[1] && pathpointsY[path_input_count] != fromLVvalues[2]){



            // check last entry from Labview as identifier, 1 as start/end point, 0 as intermediate points
            if (fromLVvalues[7] >= 1){
                pathpointsX[path_input_count] = fromLVvalues[1] * 15.0/500.0;
                pathpointsY[path_input_count] = fromLVvalues[2] * 15.0/500.0;
                path_end_recieved = true;

                for(int16_t i=0; i<=path_input_count; i++){
                    pathpointsX[i] = pathpointsX[i] - pathpointsX[0];
                    pathpointsY[i] = pathpointsY[i] - pathpointsY[0];
                }

            } else {
                pathpointsX[path_input_count] = fromLVvalues[1] * 15.0 / 500.0;
                pathpointsY[path_input_count] = fromLVvalues[2] * 15.0/500.0;
            }
            path_input_count++;


        } else if (fromLVvalues[0]==2){
            Segbot_refSpeedLV = fromLVvalues[1];
            turnrateLV = fromLVvalues[2];
            Rwh = fromLVvalues[3]; //dti Radius of wheels from LabView
            WR = fromLVvalues[4]; // dti Wheelbase from Labview
            servoAngle = fromLVvalues[5];
            bal_at_tiltLV = fromLVvalues[6];
            printLV7 = fromLVvalues[7];

        }
    }






    switch(robotState){
    case 5: //wait for 10 seconds to configure IMU // place the robot horizontally
        stateCount++;
        segMode = 0;
        controlServoA = -100;
        if(stateCount >= 10*250){
            robotState = 10;
            stateCount = 0;
            int16_t songTimer = 0;
            while(songTimer < 500) {
                EPwm9Regs.TBPRD = our_songarray[1];
                songTimer++;
            }
            //            if (state5Count < OUR_SONG_LENGTH) //Our Song being The Benny Hill Theme aka Yakety Sax as defined in song.h
            //                {
            //                    EPwm9Regs.TBPRD = our_songarray[state5Count];
            //                }
            //            else if (state5Count == OUR_SONG_LENGTH)
            //            {
            //                GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0); // dti: PinMux to 0 so that it works as a GPIO Pin again after playing the song
            //            }
            //            state5Count++;
        }
        break;
    case 10: //Stand the Segbot up
        // check the state of the robot (caster or segbot)
        if(controlServoA <= servoMax ){
            controlServoA += 0.05;
        }
        segMode = 0;
        setEPWM8A_RCServo(controlServoA); //dti: pass effort values to duty cycle setting functions
        setEPWM8B_RCServo(-controlServoA); //dti: pass effort values to duty cycle setting functions
        if(fabs(tilt_value) <= bal_at_tilt){
            // the robot now is in segbot position.
            // hold the robot using servo support
            // wait for 5 seconds
            stateCount++;
            if(stateCount >= 5*250){
                robotState = 20; // activate balance control
                iK_Diff_1 = 0.0;
                iK_Diff = 0.0;
                IK_eSpeed = 0.0;
                IK_eSpeed_1 = 0.0;
                //LeftWheel_Enc = readEncLeft();
                //RightWheel_Enc = readEncRight();
                WhlDiff = LeftWheel_Enc - RightWheel_Enc;
                turnref = WhlDiff;
                stateCount = 0;
                vel_Right = .6 * vel_Right + 100 * (RightWheel_Enc - RightWheel_Enc_1);
                vel_Left = .6 * vel_Left + 100 * (LeftWheel_Enc - LeftWheel_Enc_1);
                Segbot_refSpeed = (vel_Left + vel_Right)/2.0;
            }
        }
        break;
    case 20: //Balance the Segbot
        segMode = 1;
        controlServoA = servoDefault;
        setEPWM8A_RCServo(controlServoA); //dti: pass effort values to duty cycle setting functions
        setEPWM8B_RCServo(-controlServoA); //dti: pass effort values to duty cycle setting functions
        robotState = 30;
        //        turnref = 0;
//        if (stateCount > 250*10) {
//            //.2 is how muchh it will turn before moving
//            //xRk and yRk aer current robot position
//            //4 and 4 are the location we want the robot to go to
//            //PhiR is the bearing angle of the robot that wraps around
//            //.5 is if you are within .5 tiles from the target to stop
//            //.75 is if you are within .75 tiles of the target then return 1 which allows you to send a new location
//            danNear = xy_control(&Segbot_refSpeed, &turnrate, 0.2, x, y, 0.0, 0.0, bearing, 0.5, 0.75);
//
//        }
//        stateCount++;
//        if (danNear == 1) {
//            robotState = 30;
//            stateCount = 0;
//            danNear = 0;
////            Segbot_refSpeed = 0;
////            turnrate = 0;
//        }
//        if (danNear == 1 && path_end_recieved == true) {
//            robotState = 30;
//            stateCount = 0;
////            Segbot_refSpeed = 0;
////            turnrate = 0;
//        }
        break;
    case 30: //Automatically move to XY position
        //need to change state to 30 somewhere else if we recognize there is a new XY coordinate to move to
        stateCount++;
        if (stateCount > 250*10) {
            ////            //.2 is how much it will turn before moving
            ////            //xRk and yRk aer current robot position
            ////            //4 and 4 are the location we want the robot to go to
            ////            //PhiR is the bearing angle of the robot that wraps around
            ////            //.5 is if you are within .5 tiles from the target to stop
            ////            //.75 is if you are within .75 tiles of the target then return 1 which allows you to send a new location
            danNear = xy_control(&Segbot_refSpeed, &turnrate, 0.2, x, y, pathpointsX[XYPoint], pathpointsY[XYPoint], bearing, 0.5, 0.75);
            if(danNear == 1){
                XYPoint++;
                danNear = 0;
            }
            if (XYPoint > path_input_count){
                robotState = 40;
            }
        }
        //        stateCount = 0;
        break;
    case 40:
        break;
    }

    // dti lab 7 calculate wheel speed via (low pass filter) transfer function 125s/(s+125) at .004 s
    vel_Right = .6 * vel_Right + 100 * (RightWheel_Enc - RightWheel_Enc_1);
    vel_Left = .6 * vel_Left + 100 * (LeftWheel_Enc - LeftWheel_Enc_1);
    RightWheel_Enc_1 = RightWheel_Enc;
    LeftWheel_Enc_1 = LeftWheel_Enc;

    gyrorate_dot = .6 * gyrorate_dot + 100 * (gyro_value - gyro_value_1);

    ubal = -K1*tilt_value - K2*gyro_value - K3*(vel_Left + vel_Right)/2.0 - K4*gyrorate_dot;
    gyro_value_1 = gyro_value;

    turnref = turnref_1 + 0.002 * (turnrate + turnrate_1); //case 40 turn rte comes from labview

    WhlDiff =LeftWheel_Enc - RightWheel_Enc;
    errorDiff = turnref - WhlDiff; // dti error between reference turn angle and output // case 40
    if(fabs(uTurn) < 3.0) { //dti if statement to prevent integral windup
        iK_Diff = iK_Diff_1 + (.002)*(errorDiff + errorDiff_1); // dti : integral control
    }
    dK_Diff = 0.3333*dK_Diff_1 + 166.6667*(WhlDiff - WhlDiff_1); // dti : compute derivative control
    uTurn = errorDiff*KpTurn + iK_Diff*KiTurn - dK_Diff*KdTurn; //

    //dti saturation if function so Turn control effect does not drown balancing control effect
    if (uTurn > 4.0){
        uTurn = 4.0;
    } else if (uTurn < -4.0) {
        uTurn = -4.0;
    }

    // Feed-forward computation
    eSpeed = (Segbot_refSpeed - (vel_Left + vel_Right)/2.0);

    if(fabs((ForwardBackwardCommand)) <= 3.0) {
        IK_eSpeed = IK_eSpeed_1 + 0.002 * (eSpeed + eSpeed_1);
    }

    ForwardBackwardCommand = KpSpeed*eSpeed + KiSpeed*IK_eSpeed;

    if (ForwardBackwardCommand >= 4){
        ForwardBackwardCommand = 4;
    }
    if (ForwardBackwardCommand <= -4){
        ForwardBackwardCommand = -4;
    }
    eSpeed_1 = eSpeed ;
    IK_eSpeed_1 = IK_eSpeed;

    if(segMode == 0){
        // do not move the robot wheels until robot stands straight
        uLeft_k = 0;
        uRight_k = 0;
        //        tilt_value_1 = tilt_value;
    }else if(segMode == 1){
        uLeft_k = ubal/2.0 + uTurn - ForwardBackwardCommand; // dti : left wheel control action
        uRight_k = ubal/2.0 - uTurn - ForwardBackwardCommand; // dti : right wheel control action
    }

    setDutyEPWM2A(uRight_k); // dti lab6 ex3 : send duty to right wheel
    setDutyEPWM2B(-uLeft_k); // dti lab6 ex3 : send duty to left wheel
    dK_Diff_1 = dK_Diff; // dti : update derivative control
    errorDiff_1 = errorDiff; // dti : update error
    iK_Diff_1 = iK_Diff; // dti : update integral control
    WhlDiff_1 = WhlDiff; // dti : update wheel angle difference

    turnref_1 = turnref;
    turnrate_1 = turnrate;

    // dti : lab 6, ex6, compute pose of the robot - x, y, bearing
    bearing = Rwh/WR*(RightWheel_Enc - LeftWheel_Enc); // dti : calculate pose angle or bearing
    th_avg = 0.5*(LeftWheel_Enc + RightWheel_Enc); // compute average wheel position (angular)
    th_avg_dot = 0.5*(vel_Left + vel_Right)/Rwh;
    vx_k = Rwh*th_avg_dot*cos(bearing); // dti : update robot x speed
    vy_k = Rwh*th_avg_dot*sin(bearing); // dti : update robot y speed
    x = x + 0.5*(vx_k+vx_k_1)*0.004; // dti : calculate pose x coordinate using trapezoidal rule
    y = y + 0.5*(vy_k+vy_k_1)*0.004; // dti : calculate pose y coordinate using trapezoidal rule

    //    posLeft_k_1 = posLeft_k; // dti lab6 ex3: update previous time step value
    //    posRight_k_1 = posRight_k; // dti lab6 ex3: update previous time step value
    //    eLeft_k_1 = eLeft_k;
    //    ILeft_k_1 = ILeft_k;
    //    eRight_k_1 = eRight_k;
    //    IRight_k_1 = IRight_k;
    //    vx_k_1 = vx_k;
    //    vy_k_1 = vy_k;

    if((numSWIcalls%62) == 0) { // change to the counter variable of you selected 4ms. timer
        DataToLabView.floatData[0] = x;
        DataToLabView.floatData[1] = y;
        DataToLabView.floatData[2] = bearing;
        DataToLabView.floatData[3] = robotState;
        DataToLabView.floatData[4] = danNear;
        DataToLabView.floatData[5] = pathpointsX[XYPoint];
        DataToLabView.floatData[6] = pathpointsY[XYPoint];
        DataToLabView.floatData[7] = XYPoint;
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
        //        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    //
    //    // dti : Ex5 lab 6 receive 8 values from LabVIEW
    //    if (NewLVData == 1) {
    //        NewLVData = 0;
    //        Vref = fromLVvalues[0];
    //        turn = fromLVvalues[1];
    //        Rwh = fromLVvalues[2]; //dti Radius of wheels from LabView
    //        WR = fromLVvalues[3]; // dti Wheelbase from Labview
    //        printLV5 = fromLVvalues[4];
    //        printLV6 = fromLVvalues[5];
    //        printLV7 = fromLVvalues[6];
    //        printLV8 = fromLVvalues[7];
    //    }
    //
    //    CpuTimer1.InterruptCount++;
    //    numTimer1calls++;
    //    LeftWheel_Enc = readEncLeft(); // dti lab6 ex3: read left wheel position in rad
    //    RightWheel_Enc = readEncRight(); // dti lab6 ex3: read right wheel position in rad
    //    posLeft_k = LeftWheel_Enc * Rwh; // dti lab6 ex3: convert rad to ft
    //    posRight_k = RightWheel_Enc * Rwh; // dti lab6 ex3: convert rad to ft
    //    vLeftk = (posLeft_k - posLeft_k_1)*250; //dti lab6 ex3: compute left velocity (every 4 ms)
    //    vRightk = (posRight_k - posRight_k_1)*250; // dti lab6 ex3: compute right velocity (every 4 ms)
    //
    //    //dti lab6 ex3: Wheel speed controller
    //    eTurn = turn + (vLeftk - vRightk); // dti lab 6 ex4 : compute the turn error
    //    eLeft_k = Vref-vLeftk - Kturn*eTurn; // dti lab6 ex3: controller error Left wheel velocities // ex4 : add eTurn to steer the robot
    //    if ((uLeft_k <= 10) && (uLeft_k >= -10)) { // dti lab6 ex3 anti windup: checking if control command is saturated before integrating error
    //        ILeft_k = ILeft_k_1 + 0.002*(eLeft_k + eLeft_k_1); // dti lab6 ex3: integral term left wheel
    //    }
    //    uLeft_k = Kp*eLeft_k + Ki*ILeft_k; // dti lab6 ex3: control effort left wheel
    //
    //    eRight_k = Vref-vRightk + Kturn*eTurn; // dti lab6 ex3: same as for left wheel // ex4 : add eTurn to steer the robot
    //    if ((uRight_k <= 10) && (uRight_k >= -10)) {
    //        IRight_k = IRight_k_1 + 0.002*(eRight_k + eRight_k_1);
    //    }
    //    uRight_k = Kp*eRight_k + Ki*IRight_k;
    //
    //
    //    setDutyEPWM2A(uRight_k); // dti lab6 ex3 : send duty to right wheel
    //    setDutyEPWM2B(-uLeft_k); // dti lab6 ex3 : send duty to left wheel
    //
    //    // dti : lab 6, ex6, compute pose of the robot - x, y, bearing
    //    bearing = Rwh/WR*(RightWheel_Enc - LeftWheel_Enc); // dti : calculate pose angle or bearing
    //    th_avg = 0.5*(LeftWheel_Enc + RightWheel_Enc); // compute average wheel position (angular)
    //    th_avg_dot = 0.5*(vLeftk + vRightk)/Rwh;
    //    vx_k = Rwh*th_avg_dot*cos(bearing); // dti : update robot x speed
    //    vy_k = Rwh*th_avg_dot*sin(bearing); // dti : update robot y speed
    //    x = x + 0.5*(vx_k+vx_k_1)*0.004; // dti : calculate pose x coordinate using trapezoidal rule
    //    y = y + 0.5*(vy_k+vy_k_1)*0.004; // dti : calculate pose y coordinate using trapezoidal rule
    //
    //    posLeft_k_1 = posLeft_k; // dti lab6 ex3: update previous time step value
    //    posRight_k_1 = posRight_k; // dti lab6 ex3: update previous time step value
    //    eLeft_k_1 = eLeft_k;
    //    ILeft_k_1 = ILeft_k;
    //    eRight_k_1 = eRight_k;
    //    IRight_k_1 = IRight_k;
    //    vx_k_1 = vx_k;
    //    vy_k_1 = vy_k;
    //
    //    if((numTimer1calls%62) == 0) { // change to the counter variable of you selected 4ms. timer
    //        DataToLabView.floatData[0] = x;
    //        DataToLabView.floatData[1] = y;
    //        DataToLabView.floatData[2] = bearing;
    //        DataToLabView.floatData[3] = 2.0*((float)numTimer0calls)*.001;
    //        DataToLabView.floatData[4] = 3.0*((float)numTimer0calls)*.001;
    //        DataToLabView.floatData[5] = (float)numTimer0calls;
    //        DataToLabView.floatData[6] = (float)numTimer0calls*4.0;
    //        DataToLabView.floatData[7] = (float)numTimer0calls*5.0;
    //        LVsenddata[0] = '*'; // header for LVdata
    //        LVsenddata[1] = '$';
    //        for (int i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
    //            if (i%2==0) {
    //                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
    //            } else {
    //                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
    //            }
    //        }
    //        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    //    }

}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    //    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 10) == 0) {
        //        UARTPrint = 1;
    }
}

// dti lab5 : Ex1.2 SPIB interrupt function
__interrupt void SPIB_isr(void) {

    countSPIB++;
    // dti : MPU9250's slave select high
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // dti lab5 : Ex3 Set GPIO66 high to end Slave Select. Deselect MPU9250
    spivalue1 = SpibRegs.SPIRXBUF; // dti lab5 : Ex1.3 Read first 16-bit value off RX FIFO. Probably is zero since no chip // Ex2//Ex3
    //spivalue2 = SpibRegs.SPIRXBUF; // dti lab5 : Ex1.3 Read second 16-bit value off RX FIFO. Again probably zero // Ex2//Ex3
    //spivalue3 = SpibRegs.SPIRXBUF; // dti lab5 : Ex2 Read second 16-bit value off RX FIFO. Again probably zero
    //toTeraTerm2 = spivalue2/4096.0 * 3.0; // dti lab5 : Ex2 convert ADC1 from DAN to voltage
    //toTeraTerm3 = spivalue3/4096.0 * 3.0; // dti lab5 : Ex2 convert ADC2 from DAN to voltage

    accel_xraw = SpibRegs.SPIRXBUF; // dti lab5 : Ex3
    accel_yraw = SpibRegs.SPIRXBUF; // dti lab5 : Ex3
    accel_zraw = SpibRegs.SPIRXBUF; // dti lab5 : Ex3
    temperature = SpibRegs.SPIRXBUF; // dti lab5 : Ex3
    gyro_xraw = SpibRegs.SPIRXBUF; // dti lab5 : Ex3
    gyro_yraw = SpibRegs.SPIRXBUF; // dti lab5 : Ex3
    gyro_zraw = SpibRegs.SPIRXBUF; // dti lab5 : Ex3

    // dti lab5 : Scale the above values
    // 0 to 65,536 --->  -39.24 to 39.24 for accel
    // 0 to 65,536 --->  -250 to 250 dps for gyros
    accelx = 4/32767.0 * accel_xraw;
    accely = 4/32767.0 * accel_yraw;
    accelz = 4/32767.0 * accel_zraw;
    gyrox = 250/32767.0 * gyro_xraw;
    gyroy = 250/32767.0 * gyro_yraw;
    gyroz = 250/32767.0 * gyro_zraw;

    // dti : read motor angle values in radians
    LeftWheel_Enc = readEncLeft(); // dti lab6 ex3: read left wheel position in rad
    RightWheel_Enc = readEncRight(); // dti lab6 ex3: read right wheel position in rad

    //dti lab 7 Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected for Kalman filtering.
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
        UARTPrint = 1; // Tell While loop to print
    }
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

    //    if(countSPIB % 200 == 0) UARTPrint = 1;

    // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt

}

// dti lab5 : function to ramp up and down a variable between a maximum and a minimum value at a given increment rate
int16_t ramp_up_down(int16_t var, int16_t min, int16_t max, int16_t incr){

    switch(updown){
    case 0:
        var+=incr;
        if (var >= max) updown = 1;
        break;
    case 1:
        var-=incr;
        if (var <= min) updown = 0;
        break;
    }
    return var;
}

// dti lab5 : Ex3 Setup SPIB for MPU9250
void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in between each transfer to 0. Also don’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65, 66 which are also a part of the SPIB setup.
    //dti lab5: Ex1.1 given Code
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //dti lab5: Ex1.1 Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //dti lab5: Ex1.1 Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //dti lab5: Ex1.1 Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;
    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0x0; // dti lab5: Ex1.1 Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 0x1; // dti lab5: Ex1.1: Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // dti lab5: Ex1.1 Set to transmit and receive 16-bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 0x1; // dti lab5: Ex1.1 Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0x0; // dti lab5: Ex1.1 Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 0x31; //dti lab5: Ex1.1 divide base clk rate by 49(=0x31)+1, Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 0x1;// dti lab5: Ex1.1 Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 0x1; // dti lab5: Ex1.1 Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 0x1; // dti lab5: Ex1.1 Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 0x1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0x0; // dti lab5: Ex1.1 Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 0x1; // dti lab5: Ex1.1 Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 0x1; // dti lab5: Ex1.1 Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; // dti lab5: Ex1.1 Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below

    SpibRegs.SPICCR.bit.SPICHAR = 0xF;
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16-bit transfers, so two registers at a time after the first 16-bit transfer.
    // To address 00x13 write 0x00
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = 0x1300; //dti lab5 : Ex3 write 0x00 to 13 address. Since first (read or write) bit is low anyways its going to write
    SpibRegs.SPITXBUF = 0x0000; //dti lab5 : Ex 3 write 0x00 to 14 and 15 address
    SpibRegs.SPITXBUF = 0x0000; //dti lab5 : Ex 3 write 0x00 to 16 and 17 address
    SpibRegs.SPITXBUF = 0x0013; //dti lab5 : Ex 3 write 0x00 to 18 and 0x13 to 19 address
    SpibRegs.SPITXBUF = 0x0200; //dti lab5 : Ex 3 write 0x02 to 1A and 0x00 to 1B address
    SpibRegs.SPITXBUF = 0x0806; //dti lab5 : Ex 3 write 0x08 to 1C and 0x06 to 1D address
    SpibRegs.SPITXBUF = 0x0000; //dti lab5 : Ex 3 write 0x00 to 1E and 1F address
    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7); // dti lab5 : Ex3 Run while loop until all 7 16-bit values are received

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High

    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    // dti lab5 : Ex3 Clear out the Receiving FIFO by reading out all 7 16-bit values
    // for now temp stores garbage values
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    SpibRegs.SPITXBUF = 0x2300;// dti lab5: Ex3 To address 00x23 write 0x00
    SpibRegs.SPITXBUF = 0x408C;// dti lab5: Ex3 To address 00x24 write 0x40 and To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = 0x0288;// dti lab5: Ex3 To address 00x26 write 0x02 and To address 00x27 write 0x88
    SpibRegs.SPITXBUF = 0x0C0A;// dti lab5: Ex3 To address 00x28 write 0x0C and To address 00x29 write 0x0A

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=4); // dti lab5 : Ex3 Run while loop until all 4 16-bit values are received
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High

    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    // dti lab5 : Ex3 Clear out the Receiving FIFO by reading out all 7 16-bit values
    // for now temp stores garbage values
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    // perform a single 16-bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // dti lab5 : Ex3 Select slave to low
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = 0x2A81; // dti lab5 : Ex3
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    // dti lab5 : Ex3 Clear out the Receiving FIFO by reading out the 16-bit values
    // for now temp stores garbage values
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
    SpibRegs.SPITXBUF = (0x7700 | 0x00E9); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0034); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x0010); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x002E); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0022); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0058); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

}

void init_eQEPs(void) { // dti :  lab6 given code
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

float readEncLeft(void) {// dti :  lab6 given code
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (-raw*2*M_PI/12000.0); // dti lab6 : 400 counts per motor revolution and 30 motor revolutions per wheel revolution
}

float readEncRight(void) {// dti :  lab6 given code
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*2*M_PI/12000.0); // dti lab6 : 400 counts per motor revolution and 30 motor revolutions per wheel revolution
}

// dti : Ex2 functions to set the duty cycle of EPWM2A/B
void setDutyEPWM2A(float controleffort)
{
    if(controleffort <= -10.0) controleffort = -10.0;// dti: exclude values outside the speed range
    else if (controleffort >= 10.0) controleffort = 10.0; //dti: exclude values outside the speed range
    EPwm2Regs.CMPA.bit.CMPA = EPwm2Regs.TBPRD/20.0 * (controleffort + 10.0); //dti: assigning duty cicle according to control effort via linear interpolation
}
void setDutyEPWM2B(float controleffort)
{
    if(controleffort <= -10.0) controleffort = -10.0;
    else if (controleffort >= 10.0) controleffort = 10.0;
    EPwm2Regs.CMPB.bit.CMPB = EPwm2Regs.TBPRD/20.0 * (controleffort + 10.0);
}

// dti lab7: adcd1 pie interrupt
__interrupt void ADCD_ISR (void) {

    count_ADCD1++; // dti : Ex1.4.f increment with interrupt calls
    adcdin0_raw = AdcdResultRegs.ADCRESULT0; // dti : reads ADCA SOC0 (channel 2)
    adcdin1_raw = AdcdResultRegs.ADCRESULT1; // dti : reads ADCA SOC1 (channel 3)
    // Here covert ADCIND0 to volts
    adcdin0_result = 3.0/4096.0*adcdin0_raw; // dti : Ex1.4.e scale adc output to 0-3.0 volts

    // Here covert ADCIND0, ADCIND1 to volts
    //xk = adcdin0_result; // dti : Ex2 adcdin0 xk allocation of current voltage value for moving mean and fixed 4th Order FIR
    xk[0] = adcdin0_result; // dti: EX2 allocation of current voltage value for any order FIR defined by b-array
    yk = 0.0; //dti Ex2 resetting filtered value before calculation
    for (int i=0;i<SIZEOFARRAY;i++) // dti: Ex2 calculating filtered voltage value
    {
        yk += b[i]*xk[i]; // dti: Ex2 summing up the n weighted array elements containing the n last voltage values for nth order FIR filter
    }
    for (int j = SIZEOFARRAY-1; j >= 1; j--)
    {
        xk[j] = xk[j-1]; // dti: Ex2 updating last voltage values
    }

    /*yk = b[0]*xk + b[1]*xk_1 + b[2]*xk_2 + b[3]*xk_3 + b[4]*xk_4; // dti : Ex2 average of previous 5 values of adcdin0
//Save past states before exiting from the function so that next sample they are the older state
xk_4 = xk_3; // dti : Ex2 update adcdin0 value 4 ms ago
xk_3 = xk_2; // dti : Ex2 update adcdin0 value 3 ms ago
xk_2 = xk_1; // dti : Ex2 update adcdin0 value 2 ms ago
xk_1 = xk; // dti : Ex2 update adcdin0 value 1 ms ago*/

    // Here write voltages value to DACA
    //setDACA(adcdin0_result); // dti : Ex1.4.f pass the voltage value to DAC on the robot
    //setDACA(yk); // dti : Ex2 pass the moving average voltage value to DAC on the robot
    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    //if (count_ADCD1 % 100 == 0) UARTPrint = 1; // dti : Ex1.4.f update UARTPrint every 100 ms

    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

// dti lab7: adca1 pie interrupt
__interrupt void ADCA_ISR (void) {

    count_ADCA1++; // dti : Ex3 increment with interrupt calls
    adcain2_raw = AdcaResultRegs.ADCRESULT0;
    adcain3_raw = AdcaResultRegs.ADCRESULT1;
    // Here covert ADCINA0 to volts
    adcain2_result = 3.0/4096.0*adcain2_raw; // dti : Ex3 scale adc output to 0-3.0 volts
    adcain3_result = 3.0/4096.0*adcain3_raw;

    // dti : lab 7 transmit SPIB
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // dti lab5: Ex3 set GPI66 to low to MPU slave
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Ex3 wait for 2 values to be received
    SpibRegs.SPITXBUF = 0xBA00; // dti lab5 Ex3 (0x80 + 0x3A) requesting a read starting at register 0x3A (INT_STATUS)
    // sending to read out 0x3B through 0x48 (Accelerometer xyz, temp out and gyro xyz values)
    SpibRegs.SPITXBUF = 0x0000; // dti lab5 : EX3 Accel_XOUT
    SpibRegs.SPITXBUF = 0x0000; // dti lab5 : EX3 Accel_YOUT
    SpibRegs.SPITXBUF = 0x0000; // dti lab5 : EX3 Accel_ZOUT
    SpibRegs.SPITXBUF = 0x0000; // dti lab5 : EX3 TEMP_OUT
    SpibRegs.SPITXBUF = 0x0000; // dti lab5 : EX3 GYRO_XOUT
    SpibRegs.SPITXBUF = 0x0000; // dti lab5 : EX3 GYRO_YOUT
    SpibRegs.SPITXBUF = 0x0000; // dti lab5 : EX3 GYRO_ZOUT

    //    xk2[0] = adcain2_result; // dti: EX3 allocation of current voltage value for any order FIR defined by b-array
    //    xk3[0] = adcain3_result; // dti: EX3 allocation of current voltage value for any order FIR defined by b-array
    //    yk2 = 0.0; //dti Ex3 resetting filtered value before calculation
    //    yk3 = 0.0; //dti Ex3 resetting filtered value before calculation
    //    for (int i=0;i<SIZEOFARRAY;i++) // dti: Ex3 calculating filtered voltage value
    //    {
    //        yk2 += b[i]*xk2[i]; // dti: Ex3 summing up the n weighted array elements containing the n last voltage values for nth order FIR filter
    //        yk3 += b[i]*xk3[i]; // dti: Ex3 summing up the n weighted array elements containing the n last voltage values for nth order FIR filter
    //    }
    //    for (int j = SIZEOFARRAY-1; j >= 1; j--)
    //    {
    //        xk2[j] = xk2[j-1]; // dti: Ex3 updating last voltage values
    //        xk3[j] = xk3[j-1]; // dti: Ex3 updating last voltage values
    //    }
    //
    //    // dti: Print ADCs voltage value to TeraTerm every 100ms
    //    if (count_ADCA1 % 100 == 0) UARTPrint = 1; // dti : Ex3 update UARTPrint every 100 ms

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

// dti lab7: adcb1 pie interrupt
__interrupt void ADCB_ISR (void) {

    // dti lab7: set GPIO52
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;

    count_ADCB1++; // dti : Ex4 increment with interrupt calls
    adcbin0_raw = AdcbResultRegs.ADCRESULT0;
    // Here covert ADCIND0 to volts
    adcbin0_result = 3.0/4096.0*adcbin0_raw; // dti : Ex1.4.e scale adc output to 0-3.0 volts

    // Here covert ADCIND0, ADCIND1 to volts
    xkb0[0] = adcbin0_result; // dti: EX4 allocation of current voltage value for any order FIR defined by b-array
    ykb0 = 0.0; //dti Ex2 resetting filtered value before calculation
    for (int i=0;i<SIZEOFARRAY_BP;i++) // dti: Ex2 calculating filtered voltage value // changed to SIZEOFARRAY_BP for bandpass filter
    {
        //    ykb0 += b31[i]*xkb0[i]; // dti: Ex2 summing up the n weighted array elements containing the n last voltage values for nth order FIR filter
        ykb0 += bp[i]*xkb0[i]; // dti: Ex4 summing up the n weighted array elements containing the n last voltage values for nth order band pass filter
    }
    for (int j = SIZEOFARRAY_BP-1; j >= 1; j--)
    {
        xkb0[j] = xkb0[j-1]; // dti: Ex2 updating last voltage values
    }



    // Here write voltages value to DACA
    //setDACA(xkb0[0]); // dti : Ex4 pass the unfiltered voltage value to DAC on the robot
    //setDACA(ykb0 + 1.5); // dti : Ex4 pass the filtered voltage value to DAC on the robot

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    // dti : clear GPIO52
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

}

// dti: functions to set duty cycles of EPWM8A/B in order to set the servos to desired angle
void setEPWM8A_RCServo(float angle)
{
    EPwm8Regs.CMPA.bit.CMPA = (62500.0/2250.0)*(angle + 180.0);// dti: assigning duty cycle according to angle via linear interpolation
}

void setEPWM8B_RCServo(float angle)
{
    EPwm8Regs.CMPB.bit.CMPB = (62500.0/2250.0)*(angle + 180.0);
}

