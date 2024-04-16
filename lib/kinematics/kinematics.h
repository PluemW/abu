#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"

#define RPM_TO_RPS 1 / 60

class Kinematics
{
public:
    enum base
    {
        SWERVE
    };

    base base_platform_;

    struct rpm
    {
        float motor1;
        float rot1;
        float motor2;
        float rot2;
        float motor3;
        float rot3;
        float motor4;
        float rot4;
    };

    struct velocities
    {
        float linear_x;
        float linear_y;
        float angular_z;
    };

    struct pwm
    {
        int motor1;
        int motor2;
        int motor3;
        int motor4;
    };

    Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
               float motor_operating_voltage, float motor_power_max_voltage,
               float wheel_diameter, float wheels_y_distance,
               float min_pwm, float max_pwm);
    velocities getVelocities(float rpm[]);
    rpm getRPM(float linear_x, float linear_y, float angular_z);
    pwm getPWM(int pwm_motor_compute[]);
    void calculateVxVy(float retVxVy[], float currentSpeed, float currentAngle);
    float calculateVelocity(float rpm1, float rpm2);
    float calculateRot(float rot,float x_rpm, float y_rpm);
    float getMaxRPM();

private:
    rpm calculateRPM(float linear_x, float linear_y, float angular_z);
    pwm calculatePWM(int pwm_motor_compute[]);
    int getTotalWheels(base robot_base);

    float min_pwm_;
    float max_pwm_;
    float max_rpm_;
    float max_rpm_ratio;
    float wheels_y_distance_;
    float pwm_res_;
    float wheel_circumference_;
    float temp_rot_;
    int total_wheels_;
};

#endif