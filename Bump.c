/* DriverLib Includes */
#include "driverlib.h"
#include "msp.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Project Includes */
#include "Bump.h"

// Define statements for bump switch pins
#define BUMP_PORT   GPIO_PORT_P4
#define BUMP0       GPIO_PIN0   // P4.0
#define BUMP1       GPIO_PIN2   // P4.2
#define BUMP2       GPIO_PIN3   // P4.3
#define BUMP3       GPIO_PIN5   // P4.5
#define BUMP4       GPIO_PIN6   // P4.6
#define BUMP5       GPIO_PIN7   // P4.7
#define BUMP_PINS   (BUMP0 | BUMP1 | BUMP2| BUMP3| BUMP4 | BUMP5)

// Initialize the bump switch pins as GPIO inputs with pull up resistors
// Switches are active low
void Bump_Init(void){
    //Write this for Interrupt Module warm-up
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, BUMP_PINS);
}

// reads values of bump switches
//gives result with positive logic
uint8_t Bump_Read(void){
    //write this for Interrupt Module warm-up
    uint8_t input = GPIO_getInputPinValue(GPIO_PORT_P4, BUMP0);
    input = input | (GPIO_getInputPinValue(GPIO_PORT_P4, BUMP1)) << 1;
    input = input | (GPIO_getInputPinValue(GPIO_PORT_P4, BUMP2)) << 2;
    input = input | (GPIO_getInputPinValue(GPIO_PORT_P4, BUMP3)) << 3;
    input = input | (GPIO_getInputPinValue(GPIO_PORT_P4, BUMP4)) << 4;
    input = input | (GPIO_getInputPinValue(GPIO_PORT_P4, BUMP5)) << 5;
    return (~input) & 63;
}


#define BUMP_INTERRUPT_PRIORITY 0
void (*BumpTask)(uint8_t bumpData); // function pointer for user task when bump interrupt is detected

// Initializes the bump switch pins and enables Port 4 GPIO interrupts
void Bump_Interrupt_Init(void(*task)(uint8_t)){
    //Write this for Interrupt Module Race-Day
    // set BumpTask to be then user function to be called in ISR,
    BumpTask = task;

    // initialize bump pins as GPIO inputs with pull up resistors
    Bump_Init();
    // configure falling edge interrupts on bump pins
    GPIO_interruptEdgeSelect(GPIO_PORT_P4, BUMP_PINS, GPIO_HIGH_TO_LOW_TRANSITION);
    // clear interrupt flags on bump pins
    GPIO_clearInterruptFlag(GPIO_PORT_P4, BUMP_PINS);
    // enable interrupts with GPIO on the bump pins
    GPIO_enableInterrupt(GPIO_PORT_P4, BUMP_PINS);
    // enable the P4 interrupts in the NVIC
    Interrupt_enableInterrupt(INT_PORT4);
    // set the bump interrupts to the desired priority (remember to shift it to the corect location)
    Interrupt_setPriority(INT_PORT4, BUMP_INTERRUPT_PRIORITY << 5);
}


// ISR for bump interrupts
// clear interrupt flag, read bump switches, and call user function for handling a collision
// there is only one line of code for you to add to this function, that is the call to the DL function 
// that clears the interrupt flag.
void PORT4_IRQHandler(void){
    uint8_t bumpData;
    //write this for Interrupt Module Race-Day
    // read bump switch data to be passed to the bump task
    bumpData = Bump_Read();

    // Call the bump task with the bumpData
    BumpTask(bumpData);

    // clear interrupt flags
    GPIO_clearInterruptFlag(GPIO_PORT_P4, BUMP_PINS);
    //BumpTask(bumpData);
    //GPIO_clearInterruptFlag(GPIO_PORT_P4, BUMP_PINS);
}
