// Michelino
// Robot Vehicle firmware for the Arduino platform
//
// Copyright (c) 2013 by Miguel Grinberg
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
// AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

/**
 * @file michelino.ino
 * @brief Arduino robot vehicle firmware.
 * @author Miguel Grinberg
 */

#define LOGGING

// Device drivers
// Enable one driver in each category

// Motor controller:
//#define ENABLE_ADAFRUIT_MOTOR_DRIVER
#define ENABLE_ADAFRUIT_MOTOR_DRIVER_V2
//#define ENABLE_ARDUINO_MOTOR_DRIVER

// Distance sensor
#define ENABLE_NEWPING_DISTANCE_SENSOR_DRIVER
//#define ENABLE_PARALLAX_PING_DISTANCE_SENSOR_DRIVER

// Remote control:
#define ENABLE_BLUESTICK_REMOTE_CONTROL_DRIVER
//#define ENABLE_ROCKETBOT_REMOTE_CONTROL_DRIVER

// constants
#define DOHYO_DISTANCE 90                    /**< distance to obstacle in centimeters */
#define RANDOM_ANALOG_PIN 5             /**< unused analog pin to use as random seed */
#define BT_RX_PIN 15                    /**< RX pin for Bluetooth communcation */
#define BT_TX_PIN 16                    /**< TX pin for Bluetooth communcation */

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

#ifdef ENABLE_ADAFRUIT_MOTOR_DRIVER
#include <AFMotor.h>
#include "adafruit_motor_driver.h"
#define LEFT_MOTOR_INIT 1
#define RIGHT_MOTOR_INIT 3
#endif

#ifdef ENABLE_ADAFRUIT_MOTOR_DRIVER_V2
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Arduino.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include "adafruit_motor_driver_v2.h"


#define MOTOR_INIT 1,4
#endif

#ifdef ENABLE_ARDUINO_MOTOR_DRIVER
#include "arduino_motor_driver.h"
#define LEFT_MOTOR_INIT 12, 3, 9
#define RIGHT_MOTOR_INIT 13, 11, 8
#endif

#ifdef ENABLE_NEWPING_DISTANCE_SENSOR_DRIVER
#include <NewPing.h>
#include "newping_distance_sensor.h"
#define DISTANCE_SENSOR_INIT 14,14
#endif

#ifdef ENABLE_PARALLAX_PING_DISTANCE_SENSOR_DRIVER
#include "parallax_distance_sensor.h"
#define DISTANCE_SENSOR_INIT 15,MAX_DISTANCE
#endif

#ifdef ENABLE_BLUESTICK_REMOTE_CONTROL_DRIVER
#include "bluestick_remote_control.h"
#define REMOTE_CONTROL_INIT
#endif

#ifdef ENABLE_ROCKETBOT_REMOTE_CONTROL_DRIVER
#include "rocketbot_remote_control.h"
#define REMOTE_CONTROL_INIT 10,50
#endif

#include "logging.h"
#include "moving_average.h"
#define LED_PIN 13

namespace Michelino {

    class Robot {
    public:

        /*
         * @brief Class constructor.
         */
        Robot()
        :
        motor(MOTOR_INIT),
        distanceSensor(DISTANCE_SENSOR_INIT),
        distanceAverage(DOHYO_DISTANCE),
        remoteControl(REMOTE_CONTROL_INIT),
        distanceToOpponent(-1) {
        }

        /*
         * @brief Initialize the robot state.
         */
        void initialize() {
            //                        randomSeed(analogRead(RANDOM_ANALOG_PIN));
            motor.initialize();
            remote();
        }

        /*
         * @brief Update the state of the robot based on input from sensor and remote control.
         *  Must be called repeatedly while the robot is in operation.
         */
        void run() {
            unsigned long currentTime = millis();
            int rawDistance = distanceSensor.getDistance();
            int distance = distanceAverage.add(rawDistance);
            RemoteControlDriver::command_t remoteCmd;
            bool haveRemoteCmd = remoteControl.getRemoteCommand(remoteCmd);
            log("state: %d, currentTime: %lu, distance: %u remote: (%d,k:%d)\n",
                    state, currentTime, distance,
                    haveRemoteCmd, remoteCmd.key);

            if (remoteControlled()) {
                if (haveRemoteCmd) {
                    switch (remoteCmd.key) {
                        case RemoteControlDriver::command_t::keyF1:
                            // start "roomba" mode
                            move();
                            break;
                        case RemoteControlDriver::command_t::keyNone:
                            motor.stop();
                            break;
                        case RemoteControlDriver::command_t::forward:
                            motor.forward();
                            break;
                        case RemoteControlDriver::command_t::backward:
                            motor.backward();
                            break;
                        case RemoteControlDriver::command_t::turnLeft:
                            motor.left();
                            break;
                        case RemoteControlDriver::command_t::turnRight:
                            motor.right();
                            break;
                        default:
                            break;
                    }
                }
            } else {
                // "auto" mode
                if (haveRemoteCmd && remoteCmd.key == RemoteControlDriver::command_t::keyF1) {
                    // switch back to remote mode
                    remote();
                } else {
                    if (moving()) {
                        if (missingOpponent(distance)) {
                            turn();
                        }
                    } else if (turning()) {
                        if (doneTurning(distance))
                            move();
                    }
                }
            }
        }

    protected:

        void remote() {
            state = stateRemote;
            BTSerial.println("Switching to remote mode");
        }

        void move() {
            motor.forward();
            state = stateMoving;
            BTSerial.println("Switching to sensor mode");
        }

        void stop() {
            motor.stop();
            state = stateStopped;
        }

        bool missingOpponent(unsigned int distance) {
            return (distanceToOpponent < 0 || distance > (distanceToOpponent + 5));
        }

        bool turn() {
            if (random(2) == 0) {
                motor.left();
            } else {
                motor.right();
            }
            state = stateTurning;
        }

        bool doneTurning(unsigned int distance) {
            if (distance < DOHYO_DISTANCE) {
                distanceToOpponent = distance;
                return true;
            }
            return false;
        }

        bool moving() {
            return (state == stateMoving);
        }

        bool turning() {
            return (state == stateTurning);
        }

        bool stopped() {
            return (state == stateStopped);
        }

        bool remoteControlled() {
            return (state == stateRemote);
        }

    private:
        Motor motor;
        DistanceSensor distanceSensor;
        MovingAverage<unsigned int, 3> distanceAverage;
        RemoteControl remoteControl;
        int distanceToOpponent;

        enum state_t {
            stateStopped, stateMoving, stateTurning, stateRemote
        };
        state_t state;
    };
};

Michelino::Robot robot;

void setup() {
    Serial.begin(9600);
    BTSerial.begin(9600);
    BTSerial.println("Robot initialized");

    robot.initialize();

}

void loop() {
    //        BTSerial.println("Looping");

    robot.run();
}


