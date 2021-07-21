#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <AFMotor.h>

enum motor_event_t
{
    MOTOR_IDLE = 0,
    MOTOR_FORWARD,
    MOTOR_RIGHT,
    MOTOR_LEFT,
    MOTOR_BACKWARD
};

struct motor_message_t
{
    motor_event_t event;
    int timeout_ms;
};

void motor_task(void *pv_parameters);

class MyCar
{
private:
    AF_DCMotor *motor_1;
    AF_DCMotor *motor_2;
    AF_DCMotor *motor_3;
    AF_DCMotor *motor_4;
    uint8_t motor_speed;

public:
    MyCar();
    bool set_speed(uint8_t speed);
    uint8_t get_speed(void);
    bool set_direction(motor_event_t event);
    void send_event(motor_event_t event, int timeout_ms);
};

#endif