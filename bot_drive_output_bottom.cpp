#include <micro_ros_platformio.h>
#include <Arduino.h>
#include <stdio.h>

#include "config.h"
#include "robot.h"
#include <kinematics.h>
#include <pid.h>
#include <odometry.h>
#include "Encoder.h"
#include <motor.h>
#include <Servo.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int8.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            return false;            \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

//------------------------------ < Variable Define > -----------------------------------//
unsigned long long time_offset = 0;
unsigned long prev_velocity_time = 0;
unsigned long prev_odom_update = 0;

Encoder motor1_encoder(MOTOR1_DIR_ENCODER_A, MOTOR1_DIR_ENCODER_B, COUNTS_DIR_PER_REV1, MOTOR1_DIR_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_DIR_ENCODER_A, MOTOR2_DIR_ENCODER_B, COUNTS_DIR_PER_REV2, MOTOR2_DIR_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_DIR_ENCODER_A, MOTOR3_DIR_ENCODER_B, COUNTS_DIR_PER_REV3, MOTOR3_DIR_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_DIR_ENCODER_A, MOTOR4_DIR_ENCODER_B, COUNTS_DIR_PER_REV4, MOTOR4_DIR_ENCODER_INV);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Motor_Base motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_DIR_INV, MOTOR1_DIR_PWM, MOTOR1_DIR_IN_A, MOTOR1_DIR_IN_B);
Motor_Base motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_DIR_INV, MOTOR2_DIR_PWM, MOTOR2_DIR_IN_A, MOTOR2_DIR_IN_B);
Motor_Base motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_DIR_INV, MOTOR3_DIR_PWM, MOTOR3_DIR_IN_A, MOTOR3_DIR_IN_B);
Motor_Base motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_DIR_INV, MOTOR4_DIR_PWM, MOTOR4_DIR_IN_A, MOTOR4_DIR_IN_B);
Servo rot1_controller;
Servo rot2_controller;
Servo rot3_controller;
Servo rot4_controller;

Kinematics kinematics(
    Kinematics::SWERVE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE,
    PWM_MIN, PWM_MAX);

Odometry odometry;

float current_rpm1 = 0.0;
float current_rpm2 = 0.0;   
float current_rpm3 = 0.0;
float current_rpm4 = 0.0;

float current_rot1 = 0.0;
float current_rot2 = 0.0;
float current_rot3 = 0.0;
float current_rot4 = 0.0;

float vel_cal[] = { current_rpm1,
                    current_rot1,
                    current_rpm2,
                    current_rot2,
                    current_rpm3,
                    current_rot3,
                    current_rpm4,
                    current_rot4
                };

//------------------------------ < Fuction Prototype > ------------------------------//
void moveBase();
void syncTime();
void publishData();
struct timespec getTime();
int lim_switch();
void motorstop();
// float rot();

//------------------------------ < Ros Fuction Prototype > --------------------------//
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void sub_velocity_callback(const void *msgin);
bool create_entities();
void destroy_entities();
void renew();

//------------------------------ < Ros Define > -------------------------------------//
// basic
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rcl_allocator_t allocator;
rclc_executor_t executor;

// ? define msg
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist pwm_msg;
geometry_msgs__msg__Twist rot_msg;
geometry_msgs__msg__Twist debug_msg;
geometry_msgs__msg__Twist velocity_msg;

// ? define publisher
rcl_publisher_t pub_debug;
rcl_publisher_t pub_odom;
rcl_publisher_t pub_pwm;
rcl_publisher_t pub_rot;

// std_msgs__msg__Int8 start_msg;
// std_msgs__msg__Int8 team_msg;
// std_msgs__msg__Int8 retry_msg;
// rcl_publisher_t pub_start;
// rcl_publisher_t pub_team;
// rcl_publisher_t pub_retry;

// ? define subscriber
rcl_subscription_t sub_velocity;

rcl_init_options_t init_options;

bool micro_ros_init_successful;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

//------------------------------ < Main > -------------------------------------------//
void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    rot1_controller.attach(MOTOR1_ROT);
    rot2_controller.attach(MOTOR2_ROT);
    rot3_controller.attach(MOTOR3_ROT);
    rot4_controller.attach(MOTOR4_ROT);

    pinMode(Emergency, OUTPUT);
    
    // pinMode(START_BUTTON, INPUT_PULLUP);
    // pinMode(TEAM_BUTTON, INPUT_PULLUP);
    // pinMode(RETRY_BUTTON, INPUT_PULLUP);
}

void loop()
{
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroy_entities();
        };
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
        digitalWrite(Emergency, HIGH);
    case AGENT_DISCONNECTED:
        destroy_entities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }

    if (state == AGENT_CONNECTED)
    {
    }
    else
    {
        renew();
    }
}

//------------------------------ < Fuction > ----------------------------------------//
int lim_switch(int lim_pin)
{
    return !digitalRead(lim_pin);
}

void motorstop(float vel_x, float vel_y, float vel_tan)
{
    if((vel_x && vel_y && vel_tan) == 0.0){
        motor1_controller.spin(0);
        motor2_controller.spin(0);
        motor3_controller.spin(0);
        motor4_controller.spin(0);
        rot1_controller.write(rot(-90, MOTOR1_ROT_INV));
        rot2_controller.write(rot(90, MOTOR2_ROT_INV));
        rot3_controller.write(rot(-90, MOTOR3_ROT_INV));
        rot4_controller.write(rot(90, MOTOR4_ROT_INV));
    }
}

float rot(float rpm_rot, bool invert)
{
    return (!invert) ? 90 + (rpm_rot/4.5) : 90 - (rpm_rot/4.5);
}

void moveBase()
{
    // Input msg to calculate rpm to target point
    if (((millis() - prev_velocity_time) >= 200))
    {
        velocity_msg.linear.x = 0.0;
        velocity_msg.linear.y = 0.0;
        velocity_msg.angular.z = 0.0;
    }
    
    Kinematics::rpm req_rpm = kinematics.getRPM(
        velocity_msg.linear.x,
        velocity_msg.linear.y,
        velocity_msg.angular.z);
    
    motorstop(velocity_msg.linear.x, velocity_msg.linear.y, velocity_msg.angular.z);

    // Read located robot data
    current_rpm1 = motor1_encoder.getRPM();
    current_rpm2 = motor2_encoder.getRPM();   
    current_rpm3 = motor3_encoder.getRPM();
    current_rpm4 = motor4_encoder.getRPM();
    
    // Tuning pwm of motor for going to target point
    int pwm_tuning[] = {  motor1_pid.compute(req_rpm.motor1, current_rpm1),
                          motor2_pid.compute(req_rpm.motor2, current_rpm2),
                          motor3_pid.compute(req_rpm.motor3, current_rpm3),
                          motor4_pid.compute(req_rpm.motor4, current_rpm4)
                        };

    Kinematics::pwm motor_pwm = kinematics.getPWM(pwm_tuning);

    // PWM-Base motor msg
    pwm_msg.linear.x = req_rpm.rot1;
    pwm_msg.linear.y = req_rpm.rot2;
    pwm_msg.linear.z = req_rpm.rot3;
    pwm_msg.angular.x = req_rpm.rot4;

    // Current ROT msg
    rot_msg.linear.x = current_rot1;
    rot_msg.linear.y = current_rot2;
    rot_msg.linear.z = current_rot3;
    rot_msg.angular.x = current_rot4;

    // Debug msg
    // debug_msg.linear.x = req_rpm.rot2 * RAD_TO_DEG + 5;
    // debug_msg.linear.y = req_rpm.rot2 * RAD_TO_DEG - 5;
    // debug_msg.linear.z = current_rot2;
    // debug_msg.angular.x = req_rpm.rot2;
    // debug_msg.angular.y = req_rpm.rot3;
    // debug_msg.angular.z = req_rpm.rot4;

    // Command motor-spin
    motor1_controller.spin(motor_pwm.motor1);
    motor2_controller.spin(motor_pwm.motor2);
    motor3_controller.spin(motor_pwm.motor3);
    motor4_controller.spin(motor_pwm.motor4);
    rot1_controller.write(rot(req_rpm.rot1 * RAD_TO_DEG, MOTOR1_ROT_INV));
    rot2_controller.write(rot(req_rpm.rot2 * RAD_TO_DEG, MOTOR2_ROT_INV));
    rot3_controller.write(rot(req_rpm.rot3 * RAD_TO_DEG, MOTOR3_ROT_INV));
    rot4_controller.write(rot(req_rpm.rot4 * RAD_TO_DEG, MOTOR4_ROT_INV));

    // Return located stat of robot
    Kinematics::velocities current_vel = kinematics.getVelocities(vel_cal);

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt,
        current_vel.linear_x,
        current_vel.linear_y,
        current_vel.angular_z);
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCSOFTCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void publishData()
{
    odom_msg = odometry.getData();

    // start_msg.data = lim_switch(START_BUTTON);
    // team_msg.data = lim_switch(TEAM_BUTTON);
    // retry_msg.data = lim_switch(RETRY_BUTTON);

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&pub_pwm, &pwm_msg, NULL));
    RCSOFTCHECK(rcl_publish(&pub_rot, &rot_msg, NULL));
    RCSOFTCHECK(rcl_publish(&pub_odom, &odom_msg, NULL));
    RCSOFTCHECK(rcl_publish(&pub_debug, &debug_msg, NULL));
    
    // RCSOFTCHECK(rcl_publish(&pub_team, &team_msg, NULL));
    // RCSOFTCHECK(rcl_publish(&pub_start, &start_msg, NULL));
    // RCSOFTCHECK(rcl_publish(&pub_retry, &retry_msg, NULL));
}

//------------------------------ < Ros Fuction > ------------------------------------//
bool create_entities()
{
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);
    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // create node
    RCCHECK(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));

    // TODO: create timer,
    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // TODO: create publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &pub_debug,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/drive/input"));
    RCCHECK(rclc_publisher_init_default(
        &pub_pwm,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "drive/pwm"));
    RCCHECK(rclc_publisher_init_default(
        &pub_rot,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "drive/rot"));
    RCCHECK(rclc_publisher_init_default(
        &pub_odom,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"));
    // RCCHECK(rclc_publisher_init_default(
    //     &pub_start,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    //     "button/start"));
    // RCCHECK(rclc_publisher_init_default(
    //     &pub_team,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    //     "button/team"));
    // RCCHECK(rclc_publisher_init_default(
    //     &pub_retry,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    //     "button/retry"));

    // TODO: create subscriber
    RCCHECK(rclc_subscription_init_default(
        &sub_velocity,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // TODO: create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_velocity, &velocity_msg, &sub_velocity_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    return true;
}

void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&pub_pwm, &node);
    rcl_publisher_fini(&pub_rot, &node);
    rcl_publisher_fini(&pub_odom, &node);
    rcl_publisher_fini(&pub_debug, &node);

    // rcl_publisher_fini(&pub_team, &node);
    // rcl_publisher_fini(&pub_start, &node);
    // rcl_publisher_fini(&pub_retry, &node);
    
    rcl_subscription_fini(&sub_velocity, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

void renew()
{
    digitalWrite(Emergency, HIGH);
}

//------------------------------ < Publisher Fuction > ------------------------------//

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL)
    {
        moveBase();
        publishData();
    }
}

//------------------------------ < Subscriber Fuction > -----------------------------//

void sub_velocity_callback(const void *msgin)
{
    prev_velocity_time = millis();
}
