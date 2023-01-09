#ifndef FSM2_H_
#define FSM2_H_

#include "PortPins.h"

// Type Definitions
typedef enum {
    Take_Measurements,
    Normal_Operation,
    Take_It_Slow,
    Halt,
    Picked_Up,
    Picked_Up2,
    Picked_Up3,
    Picked_Up4,
    Picked_Up5,
    Picked_Up6,
    Picked_Up7
} FSMState2;

typedef struct {
    FSMState2     CurrentState;      // Current state of the FSM
} FSMType2;

// Function Prototypes
void InitializeFSM2(FSMType2 *FSM);
FSMState2 NextStateFunction2(FSMType2 *FSM);
void OutputFunction2(FSMType2 *FSM);

#endif /* FSM2_H_ */
