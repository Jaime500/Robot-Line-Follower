#ifndef PORTPINS_H_
#define PORTPINS_H_

//LEDFR
#define LEDFR_PIN                   5
#define LEDFR_BIT                   (0x01 << LEDFR_PIN)
#define LEDFR_PORT                  P8
#define SET_LEDFR_AS_AN_OUTPUT      LEDFR_PORT->DIR |= LEDFR_BIT
#define TURN_ON_LEDFR               LEDFR_PORT->OUT |= LEDFR_BIT
#define TURN_OFF_LEDFR              LEDFR_PORT->OUT &= ~LEDFR_BIT
#define TOGGLE_LEDFR                LEDFR_PORT->OUT ^= LEDFR_BIT

//LEDFL
#define LEDFL_PIN                   0
#define LEDFL_BIT                   (0x01 << LEDFL_PIN)
#define LEDFL_PORT                  P8
#define SET_LEDFL_AS_AN_OUTPUT      LEDFL_PORT->DIR |= LEDFL_BIT
#define TURN_ON_LEDFL               LEDFL_PORT->OUT |= LEDFL_BIT
#define TURN_OFF_LEDFL              LEDFL_PORT->OUT &= ~LEDFL_BIT
#define TOGGLE_LEDFL                LEDFL_PORT->OUT ^= LEDFL_BIT

//LEDBR
#define LEDBR_PIN                   6
#define LEDBR_BIT                   (0x01 << LEDBR_PIN)
#define LEDBR_PORT                  P8
#define SET_LEDBR_AS_AN_OUTPUT      LEDBR_PORT->DIR |= LEDBR_BIT
#define TURN_ON_LEDBR               LEDBR_PORT->OUT |= LEDBR_BIT
#define TURN_OFF_LEDBR              LEDBR_PORT->OUT &= ~LEDBR_BIT
#define TOGGLE_LEDBR                LEDBR_PORT->OUT ^= LEDBR_BIT

//LEDBL
#define LEDBL_PIN                   7
#define LEDBL_BIT                   (0x01 << LEDBL_PIN)
#define LEDBL_PORT                  P8
#define SET_LEDBL_AS_AN_OUTPUT      LEDBL_PORT->DIR |= LEDBL_BIT
#define TURN_ON_LEDBL               LEDBL_PORT->OUT |= LEDBL_BIT
#define TURN_OFF_LEDBL              LEDBL_PORT->OUT &= ~LEDBL_BIT
#define TOGGLE_LEDBL                LEDBL_PORT->OUT ^= LEDBL_BIT

//  LED2 - Red
#define LED2_R_PIN                  0
#define LED2_R_BIT                 (0x01 << LED2_R_PIN)
#define LED2_R_PORT                 P2
#define SET_LED2_R_AS_AN_OUTPUT     LED2_R_PORT->DIR |= LED2_R_BIT
#define TURN_ON_LED2_RED            LED2_R_PORT->OUT |= LED2_R_BIT
#define TURN_OFF_LED2_RED           LED2_R_PORT->OUT &= ~LED2_R_BIT
#define TOGGLE_LED2_RED             LED2_R_PORT->OUT ^= LED2_R_BIT

//  LED2 - Green
#define LED2_G_PIN                  1
#define LED2_G_BIT                  (0x01 << LED2_G_PIN)
#define LED2_G_PORT                 P2
#define SET_LED2_G_AS_AN_OUTPUT     LED2_G_PORT->DIR |= LED2_G_BIT
#define TURN_ON_LED2_GREEN          LED2_G_PORT->OUT |= LED2_G_BIT
#define TURN_OFF_LED2_GREEN         LED2_G_PORT->OUT &= ~LED2_G_BIT
#define TOGGLE_LED2_GREEN           LED2_G_PORT->OUT ^= LED2_G_BIT

//  LED2 - Blue
#define LED2_B_PIN                  2
#define LED2_B_BIT                  (0x01 << LED2_B_PIN)
#define LED2_B_PORT                 P2
#define SET_LED2_B_AS_AN_OUTPUT     LED2_B_PORT->DIR |= LED2_B_BIT
#define TURN_ON_LED2_BLUE           LED2_B_PORT->OUT |= LED2_B_BIT
#define TURN_OFF_LED2_BLUE          LED2_B_PORT->OUT &= ~LED2_B_BIT
#define TOGGLE_LED2_BLUE            LED2_B_PORT->OUT ^= LED2_B_BIT

// Pushbutton S1
#define PUSHBUTTON_S1_PIN                       1
#define PUSHBUTTON_S1_BIT                       (0x01 << PUSHBUTTON_S1_PIN)
#define PUSHBUTTON_S1_PORT                      P1
#define SET_PUSHBUTTON_S1_TO_AN_INPUT           PUSHBUTTON_S1_PORT->DIR &= ~PUSHBUTTON_S1_BIT
#define ENABLE_PULL_UP_PULL_DOWN_RESISTORS_S1   PUSHBUTTON_S1_PORT->REN |= PUSHBUTTON_S1_BIT
#define SELECT_PULL_UP_RESISTORS_S1             PUSHBUTTON_S1_PORT->OUT |= PUSHBUTTON_S1_BIT

// Pushbutton S2
#define PUSHBUTTON_S2_PIN                       4
#define PUSHBUTTON_S2_BIT                       (0x01 << PUSHBUTTON_S2_PIN)
#define PUSHBUTTON_S2_PORT                      P1
#define SET_PUSHBUTTON_S2_TO_AN_INPUT           PUSHBUTTON_S2_PORT->DIR &= ~PUSHBUTTON_S2_BIT
#define ENABLE_PULL_UP_PULL_DOWN_RESISTORS_S2   PUSHBUTTON_S2_PORT->REN |= PUSHBUTTON_S2_BIT
#define SELECT_PULL_UP_RESISTORS_S2             PUSHBUTTON_S2_PORT->OUT |= PUSHBUTTON_S2_BIT

#endif /* PORTPINS_H_ */
