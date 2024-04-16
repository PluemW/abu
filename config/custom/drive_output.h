#ifndef DRIVE_OUTPUT_H
#define DRIVE_OUTPUT_H
#include "robot.h"

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

#ifdef USE_SMILE_DRIVER
// INVERT DIR MOTOR DIRECTIONS
#define MOTOR1_DIR_INV true
#define MOTOR2_DIR_INV false
#define MOTOR3_DIR_INV true
#define MOTOR4_DIR_INV true

#define MOTOR1_DIR_PWM 14
#define MOTOR1_DIR_IN_A 15
#define MOTOR1_DIR_IN_B 16

#define MOTOR2_DIR_PWM 8
#define MOTOR2_DIR_IN_A 9
#define MOTOR2_DIR_IN_B 10

#define MOTOR3_DIR_PWM 5
#define MOTOR3_DIR_IN_A 6
#define MOTOR3_DIR_IN_B 7

#define MOTOR4_DIR_PWM 2
#define MOTOR4_DIR_IN_A 3
#define MOTOR4_DIR_IN_B 4

// INVERT DIR MOTOR DIRECTIONS
#define MOTOR1_ROT_INV false
#define MOTOR2_ROT_INV false
#define MOTOR3_ROT_INV false
#define MOTOR4_ROT_INV false

#define MOTOR1_ROT 22
#define MOTOR2_ROT 23
#define MOTOR3_ROT 24
#define MOTOR4_ROT 25

#define Emergency 32

#endif

#endif