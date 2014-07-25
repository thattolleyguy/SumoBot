/**
 * @file remote_control.h
 * @brief remote control driver definition for the Michelino robot.
 * @author Miguel Grinberg
 */

namespace Michelino
{
    class RemoteControlDriver
    {
    public:
        /**
          * @brief abstract representation of a remote command.
          */
        struct command_t {
            enum key_t { keyNone, keyF1, keyF2, keyF3, keyF4, forward, turnLeft, turnRight, backward };
            key_t key;  /**< function key. */
            
            command_t() : key(keyNone) {}
            
        };
        
        /**
          * @brief Class constructor.
          */
        RemoteControlDriver() {}

        /**
         * @brief Return the next remote command, if available.
         * @param cmd a reference to a command_t struct where the command
         *   information will be stored.
         * @return true if a remote command is available, false if not.
         */
        virtual bool getRemoteCommand(command_t& cmd) = 0;
    };
};
