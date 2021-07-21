#include <Arduino.h>
#include "motor_control.h"
#include "Arduino_FreeRTOS.h"
// Include queue support
#include <queue.h>

const int tick_ms = 100; // ms tick

// define motor queue
QueueHandle_t motor_queue;

// global variable for motor car
MyCar *mycar;

MyCar::MyCar()
{
    Serial.println("Creating MyCar!");
    motor_1 = new AF_DCMotor(1);
    motor_2 = new AF_DCMotor(2);
    motor_3 = new AF_DCMotor(3);
    motor_4 = new AF_DCMotor(4);

    // create queue
    motor_queue = xQueueCreate(10,                     // Queue length
                               sizeof(motor_message_t) // Queue item size
    );

    // set motor speed
    set_speed(255);
}

bool MyCar::set_direction(motor_event_t event)
{
    uint8_t right_motor;
    uint8_t left_motor;

    switch (event)
    {
    case MOTOR_FORWARD:
        Serial.println("Motor forward");
        right_motor = FORWARD;
        left_motor = FORWARD;
        break;
    case MOTOR_IDLE:
        Serial.println("Motor idle");
        right_motor = RELEASE;
        left_motor = RELEASE;
        break;
    case MOTOR_RIGHT:
        Serial.println("Motor right");
        right_motor = BACKWARD;
        left_motor = FORWARD;
        break;
    case MOTOR_LEFT:
        Serial.println("Motor left");
        right_motor = FORWARD;
        left_motor = BACKWARD;
        break;
    case MOTOR_BACKWARD:
        Serial.println("Motor backward");
        right_motor = FORWARD;
        left_motor = BACKWARD;
        break;
    default:
        Serial.print("Event: ");
        Serial.print(event);
        Serial.println(" not recognized!");
        return false;
    }

    motor_1->run(right_motor);
    motor_2->run(right_motor);
    motor_3->run(left_motor);
    motor_4->run(left_motor);
}

bool MyCar::set_speed(uint8_t speed)
{
    Serial.print("Set motor speed: ");
    Serial.println(speed);
    motor_speed = speed;

    motor_1->setSpeed(speed);
    motor_2->setSpeed(speed);
    motor_3->setSpeed(speed);
    motor_4->setSpeed(speed);

    return true;
}

uint8_t MyCar::get_speed()
{
    return motor_speed;
}

void MyCar::send_event(motor_event_t event, int timeout_ms)
{
    motor_message_t msg;
    msg.event = event;
    msg.timeout_ms = timeout_ms;

    if (xQueueSend(motor_queue, &msg, portMAX_DELAY) == pdFAIL)
    {
        Serial.println("Failure to send event!");
    }
    else
    {
        //Serial.println("Event send!");
    };
}

void motor_task(void *pv_parameters)
{
    // motor
    mycar = new MyCar();

    motor_message_t msg;

    Serial.println("Motor task created!");

    for (;;)
    {
        if (xQueueReceive(motor_queue, &msg, tick_ms / portTICK_PERIOD_MS) == pdPASS)
        {
            Serial.println(msg.event);
            switch (msg.event)
            {
            case MOTOR_FORWARD:
            case MOTOR_BACKWARD:
            case MOTOR_LEFT:
            case MOTOR_RIGHT:
            case MOTOR_IDLE:
                mycar->set_direction(msg.event);
                /* code */
                break;

            default:
                break;
            }
        }
    }
}