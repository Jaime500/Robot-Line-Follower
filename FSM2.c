#include "FSM2.h"
#include "Clock.h"
#include "Reflectance.h"
#include "Bump.h"
#include "PortPins.h"
#include "driverlib.h"
#include "Motor.h"
#include "Nokia5110.h"
#include "opt3001.h"

uint8_t normalOperation = 1;
uint8_t stop = 0;

// OPT3001 measured in main method
extern uint16_t rawData;
extern float convertedLux;

extern int32_t accelAvgZ;

uint8_t bump = 0;

int counter = 0;

//--------------------------------------------------------------------------
// Initialize FSM
//--------------------------------------------------------------------------
void InitializeFSM2(FSMType2 *FSM)
{
    FSM->CurrentState = Take_Measurements;
}

//--------------------------------------------------------------------------
// Determine next FSM state
//--------------------------------------------------------------------------
FSMState2 NextStateFunction2(FSMType2 *FSM)
{
    FSMState2 NextState = FSM->CurrentState;

    switch (FSM->CurrentState){
    case Take_Measurements:
        if (counter > 3) // check if accelerating up long enough to be considered picked up
            NextState = Picked_Up;
        else if (convertedLux < 20||bump) // go to stopped state if light is too low or bump switches are pressed
            NextState = Halt;
        else if (convertedLux < 80)
            NextState = Take_It_Slow;
        else
            NextState = Normal_Operation;
        break;
    case Normal_Operation:
        NextState = Take_Measurements;
        break;
    case Take_It_Slow:
        NextState = Take_Measurements;
        break;
    case Halt:
        NextState = Take_Measurements;
        break;
    case Picked_Up:
        NextState = Picked_Up2;
        break;
    case Picked_Up2: // this state resets the counter used for determining how long the robot accelerated
        if (counter < 40)
            NextState = Picked_Up2;
        else
            NextState = Picked_Up3;
        break;
    case Picked_Up3: // wait to make sure the robot is being held still (picked up)
        NextState = Picked_Up4;
        break;
    case Picked_Up4: // reset the counter
        if (accelAvgZ > 25 || counter < 5)
            NextState = Picked_Up4;
        else
            NextState = Picked_Up5;
        break;
    case Picked_Up5: // wait until the robot accelerates down a few frames in a row
        NextState = Picked_Up6;
        break;
    case Picked_Up6: // reset the timer and resume normal operation
        if (counter < 40)
            NextState = Picked_Up6;
        else
            NextState = Picked_Up7;
        break;
    case Picked_Up7:
        NextState = Take_Measurements;
        break;
    }
    return NextState;
}

//--------------------------------------------------------------------------
// Determine LED output based on state
//--------------------------------------------------------------------------
void OutputFunction2(FSMType2 *FSM)
{
    switch (FSM->CurrentState) {
    case Take_Measurements:
        bump = Bump_Read();
        if (accelAvgZ > 5)
            counter++;
        else
            counter = 0;
        break;
    case Normal_Operation:
        normalOperation = 1;
        stop = 0;
        break;
    case Take_It_Slow:
        normalOperation = 0;
        stop = 0;
        break;
    case Halt:
        normalOperation = 0;
        stop = 1;
        break;
    case Picked_Up:
        TURN_ON_LED2_RED;
        normalOperation = 0;
        stop = 1;
        counter = 0;
        break;
    case Picked_Up2:
        if (accelAvgZ == 0)
            counter++;
        else {
            counter--;
            if (counter < 0)
                counter = 0;
        }
        break;
    case Picked_Up3:
        counter = 0;
        break;
    case Picked_Up4:
        if (accelAvgZ > 5)
            counter++;
        else {
            counter--;
            if (counter < 0)
                counter = 0;
        }
        break;
    case Picked_Up5:
        counter = 0;
        break;
    case Picked_Up6:
        if (accelAvgZ == 0)
            counter++;
        else {
            counter -= 2;
            if (counter < 0)
                counter = 0;
        }
        break;
    case Picked_Up7:
        TURN_OFF_LED2_RED;
        counter = 0;
        break;
    }

}
