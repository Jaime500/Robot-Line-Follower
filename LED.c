#include "LED.h"

void InitializeLEDPortPin(void)
{
    // Initialize the value and port pin direction of the LEDs.
    TURN_OFF_LED2_RED;
    SET_LED2_R_AS_AN_OUTPUT;
    TURN_OFF_LED2_GREEN;
    SET_LED2_G_AS_AN_OUTPUT;
    TURN_OFF_LED2_BLUE;
    SET_LED2_B_AS_AN_OUTPUT;

    TURN_OFF_LEDFR;
    SET_LEDFR_AS_AN_OUTPUT;
    TURN_OFF_LEDFL;
    SET_LEDFL_AS_AN_OUTPUT;
    TURN_OFF_LEDBR;
    SET_LEDBR_AS_AN_OUTPUT;
    TURN_OFF_LEDBL;
    SET_LEDBL_AS_AN_OUTPUT;
}
