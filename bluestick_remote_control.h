/**
 * @file bluestick_remote_control.h
 * @brief remote control driver for the BlueStick Android remote control app.
 * @author Miguel Grinberg
 */

#include "remote_control.h"

namespace Michelino
{
    class RemoteControl : RemoteControlDriver
    {
    public:
        /**
          * @brief Class constructor.
          */
        RemoteControl() : RemoteControlDriver(), lastKey(command_t::keyNone) {}

        virtual bool getRemoteCommand(command_t& cmd)
        {
            cmd.key = command_t::keyNone;
            if (BTSerial.available() <= 0)
            {
                return false; // no commands available
            }
            char ch = BTSerial.read();
            BTSerial.print("Received char:");
            BTSerial.println(ch);
            switch (ch) {
                case '8': // up
                    BTSerial.println("Moving forward");
                    cmd.key = command_t::forward;
                    break;
                case '2': // down
                    cmd.key = command_t::backward;
                    break;
                case '4': // right
                    cmd.key = command_t::turnRight;
                    break;
                case '6': // left
                    cmd.key = command_t::turnLeft;
                    break;
                case 'A': // function key #1
                case 'C':
                    cmd.key = command_t::keyF1;
                    break;
                case 'B': // function key #2
                case 'D':
                    cmd.key = command_t::keyF2;
                    break;
                case 'E': // function key #3
                    cmd.key = command_t::keyF3;
                    break;
                case 'F': // function key #4
                    cmd.key = command_t::keyF4;
                    break;
                default:
                    break;
            }
            if (cmd.key != command_t::keyNone && cmd.key == lastKey) {
                // repeated key, ignore it
                return false; 
            }
            lastKey = cmd.key;
            return true;
        }
    
    private:
        command_t::key_t lastKey;
    };
};
