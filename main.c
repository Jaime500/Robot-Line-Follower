/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "Clock.h"
#include "TimerA1.h"
#include "LED.h"
#include "Bump.h"
#include "FSM.h"
#include "FSM2.h"
#include "PWM.h"
#include "Motor.h"
#include "Nokia5110.h"
#include "bmi160_support.h"
#include "bmi160.h"
#include "opt3001.h"
#include "uart_driver.h"
#include "i2c_driver.h"

#define NUM_AVGR_SUMS               2 //x^2 frames

void LaunchPad_Init(void){
    InitializeLEDPortPin();
    Bump_Init();
    //setup push buttons as input with pull up resistor
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1|GPIO_PIN4); //pin 1.1 and 1.4
}

//------------LaunchPad_Input------------
// Input from Switches
// Input: none
// Output: 0x00 none
//         0x01 Button1
//         0x02 Button2
//         0x03 both Button1 and Button2
uint8_t LaunchPad_Input(void){
    //Read pin 1.1 and 1.4
    uint8_t button1 = ~GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) & 0x01;
    uint8_t button2 = ~GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) & 0x01;
    return (button2<<1)|(button1);
}

FSMType FSM1;
FSMType2 FSM2;
void Task(void){
    FSM1.CurrentState = NextStateFunction(&FSM1);
    FSM2.CurrentState = NextStateFunction2(&FSM2);
    OutputFunction(&FSM1);
    OutputFunction2(&FSM2);
}

int32_t movingAvg(int prevAvg, int16_t newValue);
void startWakeUpTimerA(uint16_t ulClockMS);
void stopWakeUpTimerA(void);


// BMI160/BMM150
s32 returnRslt;
BMI160_RETURN_FUNCTION_TYPE returnValue;
struct bmi160_accel_t       s_accelXYZ;

//Calibration off-sets
int8_t accel_off_x;
int8_t accel_off_y;
int8_t accel_off_z;

uint16_t deltaAccelZ;
int16_t prevAccelZ = 0;
int32_t accelAvgZ = 0;

// OPT3001
uint16_t rawData;
float convertedLux = 200;

uint8_t paused = 1;
int count = 0;

extern uint8_t counterr;
extern uint8_t counter;
extern float SCALE;

int main(void) {
    // Init
    WDT_A_holdTimer();
    Clock_Init();
    LaunchPad_Init();

    InitializeFSM(&FSM1);
    InitializeFSM2(&FSM2);
    //TimerA1_Init(&Task, 1000);
    Motor_Init();
    Nokia5110_Init();
    count = 1; //
    uartInit();
    initI2C();
    count = 2; //
    sensorOpt3001Init();
    sensorOpt3001Enable(true);
    count = 5; //
    // Initialize bmi160 sensor
    bmi160_initialize_sensor();
    returnRslt = bmi160_config_running_mode(APPLICATION_NAVIGATION);
    bmi160_accel_foc_trigger_xyz(0x03, 0x03, 0x01, &accel_off_x, &accel_off_y, &accel_off_z);
    // Main loop
    while(1){
        if (LaunchPad_Input() == 1 || count > 2) { // pause if button one is pressed
            paused = 1;
            count = 0; // don't automatically reenter the the paused state after the first time
            counter = 0; // reset the counter used for being picked up or not
            counterr = 0; // reset the counter used to  help reduce swaying
            TimerA1_Stop(); // stop both FSMs
            Motor_Stop(); // stop the motors
        }
        if (paused){ // change the speed of the robot based on bump presses if the robot is paused
            uint8_t bump = Bump_Read();
            if (bump & 0x20)
                SCALE = .75;
            if (bump & 0x10)
                SCALE = .9;
            if (bump & 0x8)
                SCALE = 1.05;
            if (bump & 0x4)
                SCALE = 1.2;
            if (bump & 0x2)
                SCALE = 1.35;
            if (bump & 0x1)
                SCALE = 1.5;
            Nokia5110_Clear();
            Nokia5110_SetCursor(0, 0);
            Nokia5110_OutString("Paused");
            Nokia5110_SetCursor(0, 1);
            Nokia5110_OutUDec((uint16_t) (SCALE * 100));
        }
        if (LaunchPad_Input() == 2 && paused) { // unpause if the second button is pressed
            paused = 0;
            if (FSM2.CurrentState == Picked_Up || FSM2.CurrentState == Picked_Up2 || FSM2.CurrentState == Picked_Up3 || FSM2.CurrentState == Picked_Up4 || FSM2.CurrentState == Picked_Up5 || FSM2.CurrentState == Picked_Up6)
                FSM2.CurrentState = Picked_Up7; // exit the picked up cycle if picked up
            TimerA1_Init(&Task, 1000); // start the FSMs
        }
        sensorOpt3001Read(&rawData); // read light and acceleration data to determine if dark or picked up (in FSM2)
        sensorOpt3001Convert(rawData, &convertedLux);
        returnRslt = bmi160_read_accel_xyz(&s_accelXYZ);

        deltaAccelZ = abs(s_accelXYZ.z - prevAccelZ);
        accelAvgZ = movingAvg(accelAvgZ, deltaAccelZ); // measure average acceleration
        prevAccelZ = s_accelXYZ.z;
    }
}

int32_t movingAvg(int prevAvg, int16_t newValue)
{
    return (((prevAvg << NUM_AVGR_SUMS) + newValue - prevAvg) >> NUM_AVGR_SUMS);
}

void startWakeUpTimerA(uint16_t ulClockMS)
{
    ulClockMS = (ulClockMS * 32768)/1000;

    /* TimerA UpMode Configuration Parameter */
    Timer_A_UpModeConfig upConfig =
    {
            TIMER_A_CLOCKSOURCE_ACLK,              // ACLK Clock Source
            TIMER_A_CLOCKSOURCE_DIVIDER_1,         // ACLK/1 = 32KHz
            ulClockMS,                             // tick period
            TIMER_A_TAIE_INTERRUPT_DISABLE,        // Disable Timer interrupt
            TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,   // Enable CCR0 interrupt
            TIMER_A_SKIP_CLEAR                     // Clear value
    };

    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A0_BASE,
            TIMER_A_CAPTURECOMPARE_REGISTER_0);

    MAP_Interrupt_enableInterrupt(INT_TA0_0);
    MAP_Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
}

/***********************************************************
  Function:
 */
void stopWakeUpTimerA(void)
{
    MAP_Interrupt_disableInterrupt(INT_TA0_0);
    MAP_Timer_A_stopTimer(TIMER_A0_BASE);
}
