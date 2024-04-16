#include "Arduino.h"
#include "kinematics.h"

Kinematics::Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                       float motor_operating_voltage, float motor_power_max_voltage,
                       float wheel_diameter, float wheels_y_distance, float min_pwm,
                       float max_pwm) : base_platform_(robot_base),
                                        wheels_y_distance_(wheels_y_distance),
                                        wheel_circumference_(PI * wheel_diameter),
                                        total_wheels_(getTotalWheels(robot_base)),
                                        min_pwm_(min_pwm),
                                        max_pwm_(max_pwm)
{
    motor_power_max_voltage = constrain(motor_power_max_voltage, 0, motor_operating_voltage);
    max_rpm_ = ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rpm) * max_rpm_ratio;
}

Kinematics::rpm Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
    return calculateRPM(linear_x, linear_y, angular_z);
}

Kinematics::rpm Kinematics::calculateRPM(float linear_x, float linear_y, float angular_z)
{

    float tangential_vel = angular_z * (wheels_y_distance_ / 2.0);

    // convert m/s to m/min
    float linear_vel_x_mins = 3 * linear_x * 60.0;
    float linear_vel_y_mins = 3 * linear_y * 60.0;
    // convert rad/s to rad/min
    float tangential_vel_mins = 3 * tangential_vel * 60.0;

    float x_rpm = linear_vel_x_mins / wheel_circumference_;
    float y_rpm = linear_vel_y_mins / wheel_circumference_;
    float tan_rpm = tangential_vel_mins / wheel_circumference_;

    float a_x_rpm = fabs(x_rpm);
    float a_y_rpm = fabs(y_rpm);
    float a_tan_rpm = fabs(tan_rpm);

    float xy_sum = a_x_rpm + a_y_rpm;
    float xtan_sum = a_x_rpm + a_tan_rpm;

    // calculate the scale value how much each target velocity
    // must be scaled down in such cases where the total required RPM
    // is more than the motor's max RPM
    // this is to ensure that the required motion is achieved just with slower speed
    if (xy_sum >= max_rpm_ && angular_z == 0)
    {
        float vel_scaler = max_rpm_ / xy_sum;

        x_rpm *= vel_scaler;
        y_rpm *= vel_scaler;
    }

    else if (xtan_sum >= max_rpm_ && linear_y == 0)
    {
        float vel_scaler = max_rpm_ / xtan_sum;

        x_rpm *= vel_scaler;
        tan_rpm *= vel_scaler;
    }

    Kinematics::rpm rpm;

    // calculate for the target encoder RPM and direction
    // Inverse Swerve Drive
    float rep_a = x_rpm + tan_rpm;
    float rep_b = x_rpm - tan_rpm;
    float rep_c = y_rpm + tan_rpm;
    float rep_d = y_rpm - tan_rpm;

    //front-left motor
    rpm.motor1 = calculateVelocity(rep_b, rep_c);
    rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);
    rpm.rot1 = calculateRot(rpm.rot1, rep_b, rep_c);
    rpm.rot1 = constrain(rpm.rot1, -max_rpm_, max_rpm_);

    //front-right motor
    rpm.motor2 = calculateVelocity(rep_a, rep_c);
    rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);
    rpm.rot2 = calculateRot(rpm.rot2, rep_a, rep_c);
    rpm.rot2 = constrain(rpm.rot2, -max_rpm_, max_rpm_);

    //back-left motor
    rpm.motor3 = calculateVelocity(rep_b, rep_d);
    rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);
    rpm.rot3 = calculateRot(rpm.rot3, rep_b, rep_d);
    rpm.rot3 = constrain(rpm.rot3, -max_rpm_, max_rpm_);

    //back-right motor
    rpm.motor4 = calculateVelocity(rep_a, rep_d);
    rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);
    rpm.rot4 = calculateRot(rpm.rot4, rep_a, rep_d);
    rpm.rot4 = constrain(rpm.rot4, -max_rpm_, max_rpm_);

    return rpm;
}

Kinematics::pwm Kinematics::getPWM(int pwm[])
{
    return calculatePWM(pwm);
}

Kinematics::pwm Kinematics::calculatePWM(int pwm_cal[])
{
    Kinematics::pwm pwm;

    pwm.motor1 = pwm_cal[0];
    pwm.motor1 = constrain(pwm.motor1, min_pwm_, max_pwm_);
    pwm.motor2 = pwm_cal[2];
    pwm.motor2 = constrain(pwm.motor2, min_pwm_, max_pwm_);
    pwm.motor3 = pwm_cal[4];
    pwm.motor3 = constrain(pwm.motor3, min_pwm_, max_pwm_);
    pwm.motor4 = pwm_cal[6];
    pwm.motor4 = constrain(pwm.motor4, min_pwm_, max_pwm_);

    return pwm;
}

// <--change current vel/odom formal input-->
Kinematics::velocities Kinematics::getVelocities(float rpm[])
{
    Kinematics::velocities vel;

    // Forward Swerve Drive
    //OutputSpeeds(vx, vy) for every module, calculated from module speed, angle and wheel rotation direction
    float vel_robot[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float VxVys[2];
    // Front Left
    calculateVxVy(VxVys, (rpm[0]* wheel_circumference_)/ 60.0, rpm[1]);
    vel_robot[0] = VxVys[0]; //vx
    vel_robot[1] = VxVys[1]; //vy
    // Front Right
    calculateVxVy(VxVys, (rpm[2]* wheel_circumference_)/ 60.0, rpm[3]);
    vel_robot[2] = VxVys[0];
    vel_robot[3] = VxVys[1];
    // Back Left
    calculateVxVy(VxVys, (rpm[4]* wheel_circumference_)/ 60.0, rpm[5]);
    vel_robot[4] = VxVys[0];
    vel_robot[5] = VxVys[1];
    // Back Right
    calculateVxVy(VxVys, (rpm[6]* wheel_circumference_)/ 60.0, rpm[7]);
    vel_robot[6] = VxVys[0];
    vel_robot[7] = VxVys[1];
    
    vel.linear_x = ((vel_robot[0] + vel_robot[2] + vel_robot[4] + vel_robot[6]) / 4.0) ; // m/s
    vel.linear_y = ((vel_robot[1] + vel_robot[3] + vel_robot[5] + vel_robot[7]) / 4.0) ; // m/s
    vel.angular_z = calculateRot(vel.angular_z, vel.linear_x, vel.linear_y);

    return vel;
    }

float Kinematics::calculateRot(float rot, float x_rpm, float y_rpm){
    if (y_rpm != 0 || x_rpm != 0 )
    {
        rot = atan2(y_rpm,x_rpm);
        temp_rot_ = rot;
    }
    else
    {
        rot = temp_rot_;
    }

    if(fabs(roundf(rot * RAD_TO_DEG)) == 90){
        rot = -PI/2;
    }
    else if (fabs(rot) > 1.58){
        rot = (rot > 0 ) ? rot - PI : rot + PI;
    }
    return rot;
}

float Kinematics::calculateVelocity(float rpm1, float rpm2){
    float temp1 = (atan2(rpm1, rpm2) >= 0) ? 1.0 : -1.0;
    return (round(atan2(rpm2, rpm1) * RAD_TO_DEG) == 90) ? sqrt(pow(rpm1, 2) + pow(rpm2, 2)) * -1 : sqrt(pow(rpm1, 2) + pow(rpm2, 2)) * temp1;
}

//input angle:rad
void Kinematics::calculateVxVy(float retVxVy[], float currentSpeed, float currentAngle)
{
    float temp_vx, temp_vy;
    temp_vx = currentSpeed * cos(currentAngle);
    temp_vy = tan(currentAngle) * temp_vx;
    
    retVxVy[0] = temp_vx;
    retVxVy[1] = temp_vy;
}

int Kinematics::getTotalWheels(base robot_base)
{
    switch (robot_base)
    {
    case SWERVE:
        return 4;
    default:
        return 2;
    }
}

float Kinematics::getMaxRPM()
{
    return max_rpm_;
}