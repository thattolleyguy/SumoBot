/**
 * @file adafruit_motor_driver.h
 * @brief Motor device driver for the Adafruit motor shield.
 * @author Miguel Grinberg
 */

#include "motor_driver.h"

namespace Michelino {

    class Motor : public MotorDriver {
    public:

        /*
         * @brief Class constructor.
         * @param number the DC motor number to control, from 1 to 4.
         */
        Motor(int leftNumber, int rightNumber)
        : MotorDriver(), currentSpeed(0), leftMotorNumber(leftNumber), rightMotorNumber(rightNumber), leftMotorDirection(RELEASE),rightMotorDirection(RELEASE) {
           AFMS  = Adafruit_MotorShield();

        }

        void initialize() {
            leftMotor = AFMS.getMotor(1);
            rightMotor = AFMS.getMotor(4);
            AFMS.begin();
            setSpeed(255);

        }

        void forward() {
            leftMotorDirection = FORWARD;
            rightMotorDirection = FORWARD;
            setSpeed(255);
            updateDirection();
        }

        void backward() {
            leftMotorDirection = BACKWARD;
            rightMotorDirection = BACKWARD;
            setSpeed(255);
            updateDirection();
        }

        void left() {
            leftMotorDirection = BACKWARD;
            rightMotorDirection = FORWARD;
            setSpeed(150);
            updateDirection();
        }

        void right() {
            leftMotorDirection = FORWARD;
            rightMotorDirection = BACKWARD;
            setSpeed(150);
            updateDirection();
        }

        void stop() {
            leftMotorDirection = FORWARD;
            rightMotorDirection = FORWARD;
            setSpeed(0);
            updateDirection();
        }

        void updateDirection() {
            leftMotor->run(leftMotorDirection);
            rightMotor->run(rightMotorDirection);
        }

        void setSpeed(int speed) {
            BTSerial.print("Setting speed ");
            BTSerial.println(speed);
            currentSpeed = speed;
            leftMotor->setSpeed(speed);
            rightMotor->setSpeed(speed);
        }
        int getSpeed() const {
            return currentSpeed;
        }

    private:
        Adafruit_MotorShield AFMS;
        Adafruit_DCMotor *leftMotor;
        Adafruit_DCMotor *rightMotor;
        int currentSpeed;
        int leftMotorNumber;
        int rightMotorNumber;
        int leftMotorDirection;
        int rightMotorDirection;
    };
};
