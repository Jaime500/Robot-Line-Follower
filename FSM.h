#ifndef FSM_H_
#define FSM_H_

#include "PortPins.h"

// Type Definitions
typedef enum {
    Start,
    ReadReflectance,
    Left,
    Center,
    Right,
    Lost,
    Stop,
    Wait
} FSMState;

typedef struct {
    FSMState     CurrentState;      // Current state of the FSM
} FSMType;

// Function Prototypes
void InitializeFSM(FSMType *FSM);
FSMState NextStateFunction(FSMType *FSM);
void OutputFunction(FSMType *FSM);

#endif /* FSM_H_ */
