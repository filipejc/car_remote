#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "motor_control.h"

// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <Ultrasonic.h>

Ultrasonic ultrasonic1(A1, A0); // An ultrasonic sensor HC-04

const uint8_t MIN_SPEED = 150;
const uint8_t MAX_SPEED = 255;
bool state = false;

bool MAKE_LOOP = false;

// obstancle distance
unsigned int obstacle_dist = 20; // distance

extern MyCar *mycar;
extern QueueHandle_t motor_queue;

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200); // set up Serial library at 9600 bps
  Serial.println("Motor test!");

  TaskHandle_t motor_handle;
  // create motor task
  // Create task that consumes the queue if it was created.
  xTaskCreate(motor_task, // Task function
              "Motor",    // A name just for humans
              128,        // This stack size can be checked & adjusted by reading the Stack Highwater
              NULL,
              2, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
              &motor_handle);

  if (motor_handle == NULL)
  {
    Serial.println("Problem creating task");
  }
  Serial.println("End of task creation");
}

motor_event_t last_event = MOTOR_IDLE;
bool obstacle = false;
uint8_t obstacle_count = 0;
const uint8_t number_obstacle = 3;

void loop()
{
  unsigned int distance = ultrasonic1.read();

  if (distance < obstacle_dist)
  {
    Serial.print("Obstacle detected at ");
    Serial.println(distance);
    obstacle = true;
  }
  else
  {
    Serial.print("Obstacle not detected at ");
    Serial.println(distance);
    obstacle = false;
  }

  switch (last_event)
  {
  case MOTOR_IDLE:
    mycar->send_event(MOTOR_FORWARD, 100);
    last_event = MOTOR_FORWARD;
    break;
  case MOTOR_FORWARD:
    if (obstacle)
    {
      mycar->send_event(MOTOR_RIGHT, 100);
      last_event = MOTOR_RIGHT;
    }
    /* code */
    break;
  case MOTOR_RIGHT:
    if (!obstacle)
    {
      obstacle_count++;
    }
    else if (obstacle_count > 0)
    {
      obstacle_count--;
    }

    if (obstacle_count >= number_obstacle)
    {
      mycar->send_event(MOTOR_FORWARD, 100);
      last_event = MOTOR_FORWARD;
      obstacle_count = 0;
    }
    break;

  default:
    Serial.println("Not recognized state");
    break;
  }

  digitalWrite(LED_BUILTIN, state);
  state = !state;
  delay(300);
}