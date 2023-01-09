/* DriverLib Includes */
#include "driverlib.h"

/* Project Includes */
#include "PWM.h"

/* Application Defines  */
#define TIMERA0_PERIOD  127
uint16_t TimerA0_Period = 127;


/* Timer_A UpDown Configuration Parameter */
Timer_A_UpDownModeConfig upDownConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,               // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // ACLK/1 = 32kHz
        TIMERA0_PERIOD,                         // 127 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

/* Timer_A Compare Configuration Parameter (PWM3) */
Timer_A_CompareModeConfig compareConfig_PWM3 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_3,          // Use CCR3
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_SET,              // Toggle output bit
        TIMERA0_PERIOD                              // Initial 0% Duty Cycle
};

/* Timer_A Compare Configuration Parameter (PWM3) */
Timer_A_CompareModeConfig compareConfig_PWM4 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_4,          // Use CCR3
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_SET,              // Toggle output bit
        TIMERA0_PERIOD                              // Initial 0% Duty Cycle
};

void PWM_Init(uint16_t period, uint16_t duty3, uint16_t duty4) {
    // Set Motors for CCR Output
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    upDownConfig.timerPeriod = period;
    TimerA0_Period = period;
    // Configure Timer_A0 for UpDown Mode
    Timer_A_configureUpDownMode(TIMER_A0_BASE, &upDownConfig);

    // Start TimerA0 in UpDown Mode
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UPDOWN_MODE);

    // Initialize compare registers to generate PWM1
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM3);

    // Initialize compare registers to generate PWM2
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM4);

    PWM_Duty_Right(duty3);
    PWM_Duty_Left(duty4);
}

void PWM_Duty_Right(uint16_t duty1) {
    uint16_t CCR_Value =  (((100-duty1)*TimerA0_Period)/100);
    compareConfig_PWM3.compareValue = CCR_Value;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM3);
}

void PWM_Duty_Left(uint16_t duty4) {
    uint16_t CCR_Value =  (((100-duty4)*TimerA0_Period)/100);
    compareConfig_PWM4.compareValue = CCR_Value;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM4);
}
