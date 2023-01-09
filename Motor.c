#include "driverlib.h"
#include "PWM.h"
#include "Motor.h"

//   Left Motor:                Right Motor:
//    - Direction: P5.4         - Direction: P5.5
//    - PWM:       P2.7         - PWM        P2.6
//    - Enable:    P3.7         - Enable     P3.6


// initialize PWM outputs to 0% duty cycle on the two motor PWM pins (P2.6 and P2.7)
// initialize motor enable and direction pins as GPIO outputs
// set motors to sleep mode
// set motor direction to forward initially
void Motor_Init(void){
    PWM_Init(127, 0, 0);
    GPIO_setAsOutputPin(3, GPIO_PIN6 | GPIO_PIN7);
    GPIO_setAsOutputPin(5, GPIO_PIN4 | GPIO_PIN5);
    Motor_Stop();
    GPIO_setOutputLowOnPin(5, GPIO_PIN4 | GPIO_PIN5); // Forwards
}


// Drive both motors forwards at the given duty cycles
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty){
    GPIO_setOutputLowOnPin(5, GPIO_PIN4 | GPIO_PIN5); // Forwards
    GPIO_setOutputHighOnPin(3, GPIO_PIN6 | GPIO_PIN7); // Wake up
    PWM_Duty_Right(rightDuty);
    PWM_Duty_Left(leftDuty);
}


// Drive both motors backwards at the given duty cycles
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty){
    GPIO_setOutputHighOnPin(5, GPIO_PIN4 | GPIO_PIN5); // Backwards
    GPIO_setOutputHighOnPin(3, GPIO_PIN6 | GPIO_PIN7); // Wake up
    PWM_Duty_Right(rightDuty);
    PWM_Duty_Left(leftDuty);
}


// Drive the right motor forwards and the left motor backwards at the given duty cycles
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty){
    GPIO_setOutputLowOnPin(5, GPIO_PIN5); // Forwards
    GPIO_setOutputHighOnPin(5, GPIO_PIN4); // Backwards
    GPIO_setOutputHighOnPin(3, GPIO_PIN6 | GPIO_PIN7); // Wake up
    PWM_Duty_Right(rightDuty);
    PWM_Duty_Left(leftDuty);
    }


// Drive the right motor backwards and the left motor forwards at the given duty cycles
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty){
    GPIO_setOutputLowOnPin(5, GPIO_PIN4); // Forwards
    GPIO_setOutputHighOnPin(5, GPIO_PIN5); // Backwards
    GPIO_setOutputHighOnPin(3, GPIO_PIN6 | GPIO_PIN7); // Wake up
    PWM_Duty_Right(rightDuty);
    PWM_Duty_Left(leftDuty);
}

// Stop the motors and enter sleep mode
void Motor_Stop(void){
    PWM_Duty_Right(0);
    PWM_Duty_Left(0);
    GPIO_setOutputLowOnPin(3, GPIO_PIN6 | GPIO_PIN7); // Sleep
}
