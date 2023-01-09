#include "FSM.h"
#include "Clock.h"
#include "Reflectance.h"
#include "Bump.h"
#include "PortPins.h"
#include "driverlib.h"
#include "Motor.h"
#include "Nokia5110.h"
#include "FSM2.h"
#include <stdlib.h>

float SCALE = 1.2;

int i;
uint8_t position;
uint8_t lastAveragePosition;
int32_t averagePosition = 0;
uint8_t lastPosition = 0;
uint8_t fromLost = 0;
uint8_t scale = 1;
uint8_t counterr = 0;
extern uint8_t normalOperation; // determine speed (based on light levels) from FSM2
extern uint8_t stop; // determine when to stop from FSM2
extern int32_t accelAvgZ; // average acceleration readings from the main method to determine when picked up
extern FSMType2 FSM2;

//--------------------------------------------------------------------------
// Initialize FSM
//--------------------------------------------------------------------------
void InitializeFSM(FSMType *FSM)
{
    FSM->CurrentState = Start;
}

//--------------------------------------------------------------------------
// Determine next FSM state
//--------------------------------------------------------------------------
FSMState NextStateFunction(FSMType *FSM)
{
    FSMState NextState = FSM->CurrentState;

    switch (FSM->CurrentState){
    case Start:
        NextState = ReadReflectance;
        break;
    case ReadReflectance:
        if (stop) {
            NextState = Stop;
        } else if (position == 0) {
            NextState = Lost;
        } else if (averagePosition == 0 || (lastAveragePosition > averagePosition && averagePosition > 0) || (lastAveragePosition = averagePosition && averagePosition > 0 && counterr > 2) || (lastAveragePosition < averagePosition && averagePosition < 0) || (lastAveragePosition = averagePosition && averagePosition < 0 && counterr > 2) || fromLost) { //once no longer lost, go straight until re-centered.  Also stop turning when getting closer to the line to reduce wiggle
            NextState = Center;
        } else if(averagePosition < 0){
            NextState = Right;
        } else if (averagePosition > 0) {
            NextState = Left;
        }else {
            NextState = Lost;
        }
        break;
//--Motor control states----------------------------------------------------
    case Left:
        NextState = Wait;
        break;
    case Center:
        NextState = Wait;
        break;
    case Right:
        NextState = Wait;
        break;
    case Lost:
        NextState = Wait;
        break;
    case Stop:
        NextState = Wait;
        break;
//--Wait----------------------------------------------------------------
    case Wait:
        if(i < 3){ // state to add delay, currently with no extra added delay
            NextState = Wait;
        }
        else {
            NextState = Start;
        }
        break;
    }
    return NextState;
}

//--------------------------------------------------------------------------
// Determine LED output based on state
//--------------------------------------------------------------------------
void OutputFunction(FSMType *FSM)
{
    switch (FSM->CurrentState) {
    case Start:
        Reflectance_Start();
        i = 0;
        break;
    case ReadReflectance:
        position = Reflectance_End();
        lastAveragePosition = averagePosition;
        averagePosition = Reflectance_Position(position);
        scale = 1;
        if (!normalOperation) // scale of 2 divides the the speed in half (when light levels are low, determined in FSM2 with normalOperation)
            scale = 2;
        i++;
        break;
    case Left: //left of line
        if ((uint8_t) (position << 7)) { // turn at different speeds depending on position
            Motor_Forward(45*SCALE/scale, 0*SCALE/scale);
        } else if ((uint8_t) (position << 6)) {
            Motor_Forward(45*SCALE/scale, 10*SCALE/scale);
        } else {
            Motor_Forward(50*SCALE/scale, 35*SCALE/scale);
        }
        TURN_OFF_LEDFR; // turn on appropriate lights to show which state we are in
        TURN_ON_LEDFL;
        TURN_OFF_LEDBR;
        TURN_OFF_LEDBL;
        fromLost = 0;
        lastPosition = 1;
        Nokia5110_Clear();
        Nokia5110_SetCursor(0, 0);
        Nokia5110_OutString("Left");
        Nokia5110_SetCursor(0, 1);
        Nokia5110_OutUDec((uint16_t) (SCALE * 100));
        if (lastAveragePosition = averagePosition) // increment counter to determine when to start going straight
            counterr++;
        else
            counterr = 0;
        i++;
        break;
    case Center:
        Motor_Forward(50*SCALE/scale, 50*SCALE/scale);
        TURN_ON_LEDFR; // turn on appropriate lights to show which state we are in
        TURN_ON_LEDFL;
        TURN_OFF_LEDBR;
        TURN_OFF_LEDBL;
        if (averagePosition >= -48 && averagePosition <= 48) { // if coming from lost, keep going straight until found again
            fromLost = 0;
        }
        Nokia5110_Clear();
        Nokia5110_SetCursor(0, 0);
        Nokia5110_OutString("Center");
        Nokia5110_SetCursor(0, 1);
        Nokia5110_OutUDec((uint16_t) (SCALE * 100));
        counterr = 0;
        i++;
        break;
    case Right: //right of line
        if (position < 64) { // turn at different speeds depending on position
            Motor_Forward(35*SCALE/scale, 50*SCALE/scale);
        } else if (position < 128) {
            Motor_Forward(10*SCALE/scale, 45*SCALE/scale);
        } else {
            Motor_Forward(0*SCALE/scale, 45*SCALE/scale);
        }
        TURN_ON_LEDFR; // turn on appropriate lights to show which state we are in
        TURN_OFF_LEDFL;
        TURN_OFF_LEDBR;
        TURN_OFF_LEDBL;
        fromLost = 0;
        lastPosition = 2;
        Nokia5110_Clear();
        Nokia5110_SetCursor(0, 0);
        Nokia5110_OutString("Right");
        Nokia5110_SetCursor(0, 1);
        Nokia5110_OutUDec((uint16_t) (SCALE * 100));
        if (lastAveragePosition = averagePosition)
            counterr++;
        else
            counterr = 0;
        i++;
        break;
    case Lost:
        if (lastPosition == 1) { //left of line
            Motor_Right(35*SCALE/scale, 15*SCALE/scale);
        } else if (lastPosition == 2) { //right of line
            Motor_Left(15*SCALE/scale, 35*SCALE/scale);
        } else
            Motor_Stop();
        TURN_OFF_LEDFR; // turn on appropriate lights to show which state we are in
        TURN_OFF_LEDFL;
        TURN_ON_LEDBR;
        TURN_ON_LEDBL;
        fromLost = 1;
        Nokia5110_Clear();
        Nokia5110_SetCursor(0, 0);
        Nokia5110_OutString("Lost");
        Nokia5110_SetCursor(0, 1);
        Nokia5110_OutUDec((uint16_t) (SCALE * 100));
        counterr = 0;
        i++;
        break;
    case Stop:
        Motor_Stop();
        TURN_ON_LEDFR; // turn on appropriate lights to show which state we are in
        TURN_ON_LEDFL;
        TURN_ON_LEDBR;
        TURN_ON_LEDBL;
        lastPosition = 0; // set last position to stop
        Nokia5110_Clear();
        Nokia5110_SetCursor(0, 0);
        Nokia5110_OutString("Stop");
        Nokia5110_SetCursor(0, 1);
        Nokia5110_OutUDec((uint16_t) (SCALE * 100));
        Nokia5110_SetCursor(0, 2);
        if (FSM2.CurrentState == Picked_Up2)
            Nokia5110_OutString("Picked_Up2");
        else if (FSM2.CurrentState == Picked_Up4)
            Nokia5110_OutString("Picked_Up4");
        else if (FSM2.CurrentState == Picked_Up6)
            Nokia5110_OutString("Picked_Up6");
        else
            Nokia5110_OutString("Other");
        counterr = 0;
        i++;
        break;
    case Wait:
        i++;
    }

}
