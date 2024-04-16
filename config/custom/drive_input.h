#ifndef DRIVE_INPUT_H
#define DRIVE_INPUT_H
#include "robot.h"
#define K_P 0.1
#define K_I 0.03
#define K_D 0

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

#define COUNTS_DIR_PER_REV1 600 // wheel1 direction encoder's no of ticks per rev
#define COUNTS_DIR_PER_REV2 600 // wheel2 direction encoder's no of ticks per rev
#define COUNTS_DIR_PER_REV3 600 // wheel3 direction encoder's no of ticks per rev
#define COUNTS_DIR_PER_REV4 600 // wheel4 direction encoder's no of ticks per rev

// DIRECTION ENCODER PINS
#define MOTOR1_DIR_ENCODER_A 11
#define MOTOR1_DIR_ENCODER_B 12

#define MOTOR2_DIR_ENCODER_A 34
#define MOTOR2_DIR_ENCODER_B 35

#define MOTOR3_DIR_ENCODER_A 26
#define MOTOR3_DIR_ENCODER_B 27

#define MOTOR4_DIR_ENCODER_A 30
#define MOTOR4_DIR_ENCODER_B 31

// INVERT DIRECTION ENCODER COUNTS
#define MOTOR1_DIR_ENCODER_INV true
#define MOTOR2_DIR_ENCODER_INV false
#define MOTOR3_DIR_ENCODER_INV true
#define MOTOR4_DIR_ENCODER_INV false

#endif