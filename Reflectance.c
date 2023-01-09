#include <stdint.h>
/*
 * Reflectance.c
 * Provides functions to
 * 1. Initialize Pololu reflectance sensor
 * 2. Read Pololu reflectance sensor
 * 3. Determine robot position
 * 4. Determine robot status
 *
 */

#include "driverlib.h"

#include "Clock.h"
#include "Reflectance.h"

#define ALLBITS  0xFF
#define BITSHIFT 0x01

//------------Reflectance_Init------------
// Initialize sensor array to GPIO, set LEDs (P5.3 and P9.2) 
// as output and sensors (P7.0-P7.7) as output
// Input: none
// Output: none
void Reflectance_Init(void){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN5);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN7);
}


//------------Reflectance_Read------------
// Read reflectance sensor
// Input: the delay time in us
// Output: result the sensor readings, bit 0 corresponds 
//         to the rightmost sensor, bit 7 to the leftmost sensor
void Reflectance_Start(){
    // 1. Turn on even and odd LEDs
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2);
    // 2. Charge reflectance sensor capacitors (set as output and high on P7.0-P7.7)
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN5);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN7);
    // 3. Wait 10us for capacitors to charge
    Clock_Delay1us(10);
    // 4. Set reflectance sensor (P7.0-P7.7) as input
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN0);
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN1);
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN2);
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN3);
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN4);
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN5);
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN6);
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN7);

}
uint8_t Reflectance_End() {
    // 6. Read reflectance sensor values and assign to result
    uint8_t p7 = GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN0);
    p7 = p7 | (GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN1) << 1);
    p7 = p7 | (GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN2) << 2);
    p7 = p7 | (GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN3) << 3);
    p7 = p7 | (GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN4) << 4);
    p7 = p7 | (GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN5) << 5);
    p7 = p7 | (GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN6) << 6);
    p7 = p7 | (GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN7) << 7);
    // 8. Turn off even and odd LEDs
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);
    return p7;
}

/*
//------------Reflectance_Center------------
// Determine robot's status over the line
// Input: the delay time in us
// Output: result the robot location status (LOST/RIGHT/LEFT/ON LINE)
uint8_t Reflectance_Center(uint32_t time){
    //Get reflectance read data
    uint8_t data = Reflectance_Read(time);
    //Get value from left center sensor
    uint8_t midLeft = data & 16;
    //Get value from right center sensor
    uint8_t midRight = data & 8;
    /* Check robot status truth table
     * INPUT (L,R) | OUTPUT
     * ------------|------------
     *      11     | ON_LINE (3)
     *      10     | LEFT    (2)
     *      01     | RIGHT   (1)
     *      00     | LOST    (0)
     *
    if(midLeft && midRight) {return 3;}
    else if(midLeft && !midRight) {return 2;}
    else if(!midLeft && midRight) {return 1;}
    else {return 0;}
}
*/

//------------Reflectance_Position------------
// Determine robot's status over the line
// Input: the collected sensor data 
// Output: the position value between +345 (most left)
//         to -345 (most right)
int32_t Reflectance_Position(uint8_t data){
    int32_t W_i[8] = {334, 238, 142, 48, -48, -142, -238, -334};
    int32_t d = 0;
    int32_t b = 0;
    d += W_i[0] * (data & 1);
    b += data & 1;
    d += W_i[1] * ((data & 2) >> 1);
    b += (data & 2) >> 1;
    d += W_i[2] * ((data & 4) >> 2);
    b += (data & 4) >> 2;
    d += W_i[3] * ((data & 8) >> 3);
    b += (data & 8) >> 3;
    d += W_i[4] * ((data & 16) >> 4);
    b += (data & 16) >> 4;
    d += W_i[5] * ((data & 32) >> 5);
    b += (data & 32) >> 5;
    d += W_i[6] * ((data & 64) >> 6);
    b += (data & 64) >> 6;
    d += W_i[7] * ((data & 128) >> 7);
    b += (data & 128) >> 7;
    return d / b;
}
