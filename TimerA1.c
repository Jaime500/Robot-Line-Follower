/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "TimerA1.h"

// Pointer to user function that gets called on timer interrupt
void (*TimerA1Task)(void);

Timer_A_UpModeConfig Config = {
                               TIMER_A_CLOCKSOURCE_SMCLK,
                               TIMER_A_CLOCKSOURCE_DIVIDER_24,
                               99,
                               TIMER_A_TAIE_INTERRUPT_DISABLE,
                               TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
                               TIMER_A_DO_CLEAR
};


// Initializes Timer A1 in up mode and triggers a periodic interrupt at a desired frequency
void TimerA1_Init(void(*task)(void), uint16_t period){
    TimerA1Task = task;
    Config.timerPeriod = period;
    Timer_A_configureUpMode(TIMER_A1_BASE, &Config);
    Interrupt_enableInterrupt(INT_TA1_0);
    
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}

// stops Timer A1
void TimerA1_Stop(void){
    TIMER_A1 -> CTL &= 0xFFCF;
}

// ISR function for Timer A1 periodic interrupt
void TA1_0_IRQHandler(void){
    TimerA1Task();
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}
