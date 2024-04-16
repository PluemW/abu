#ifndef ROBOT_H
#define ROBOT_H

#define USE_GY87_IMU
#define USE_SMILE_DRIVER // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE)

#define MOTOR_MAX_RPM 5500               // motor's max RPM
#define MAX_RPM_RATIO 3.0               // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define MOTOR_OPERATING_VOLTAGE 24      // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 24      // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 24 // current voltage reading of the power connected to the motor (used for calibration)
#define WHEEL_DIAMETER 0.0756            // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.6        // distance between left and right wheels
#define PWM_BITS 10                     // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000             // PWM Frequency
#define PWM_MAX pow(2, PWM_BITS) - 1 * 0.8
#define PWM_MIN (pow(2, PWM_BITS) - 1) * -0.8
#define ROT_SAFE_MIN 0.3*1023
#define ROT_SAFE_MAX 0.7*1023

#endif