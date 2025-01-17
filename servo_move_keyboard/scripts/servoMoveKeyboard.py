#!/usr/bin/python

"""
Class for testing control of 12 servos. It assumes ros-12cpwmboard has been
installed
"""
import rospy
import sys, select, termios, tty  # For terminal keyboard key press reading
from i2cpwm_board.msg import Servo, ServoArray

# Global variable for number of servos
numServos = 12

default_extremes = [
    (235, 505),
    (189, 459),
    (286, 376),
    (475, 185),
    (173, 443),
    (326, 236),
    (96, 366),
    (191, 461),
    (316, 226),
    (96, 366),
    (191, 461),
    (266, 356),
]

msg = """
Servo Control Module for 12 Servos.

Enter one of the following options:
-----------------------------
quit: stop and quit the program
oneServo: Move one servo manually, all others will be commanded to their center position
allServos: Move all servo's manually together

Keyboard commands for One Servo Control 
---------------------------
   q                y    
            f   g       j   k
    z   x       b   n   m

  q: Quit current command mode and go back to Option Select
  z: Command servo min value
  y: Command servo center value
  x: Command servo max value
  f: Manually decrease servo command value by 10
  g: Manually decrease servo command value by 1
  j: Manually increase servo command value by 1
  k: Manually increase servo command value by 10
  b: Save new min command value
  n: Save new center command value
  m: Save new max command value


  anything else : Prompt again for command


CTRL-C to quit
"""
# Dictionary with anonomous helper functions to execute key commands
keyDict = {
    "q": None,
    "z": lambda x: x.set_value(x._min),
    "y": lambda x: x.set_value(x._center),
    "x": lambda x: x.set_value(x._max),
    "f": lambda x: x.set_value(x.value - 10),
    "g": lambda x: x.set_value(x.value - 1),
    "j": lambda x: x.set_value(x.value + 1),
    "k": lambda x: x.set_value(x.value + 10),
    "b": lambda x: x.set_min(x.value),
    "n": lambda x: x.set_center(x.value),
    "m": lambda x: x.set_max(x.value),
}

validCmds = ["quit", "oneServo", "allServos", "customPositions"]


class ServoConvert:
    """
    ServoConvert Class encapsulates a servo
    Servo has a center value, and range, and is commanded by a value between 0 and 4095.
    This coorsponds to the duty cycle in a 12 bit pwm cycle. Nominally, a servo is commanded with pulses of
    1 to 2 ms in a 20 ms cycle, with 1.5 ms being the value for center position.
    These nominal values would coorespond to integer values of approximately 204, 306, and 409
    for 1 ms, 1.5 ms, and 2 ms, respectively
    """

    def __init__(
        self, id=1, center_value=306, min_value=83, max_value=520, direction=1
    ):
        self.value = center_value
        self._center = center_value
        self._min = min_value
        self._max = max_value
        self._dir = direction
        self.id = id

    def set_value(self, value_in):
        """
        Set Servo value
        Input: Value between 0 and 4095
        """
        if value_in not in range(4096):
            print("Servo value not in range [0,4095]")
        else:
            self.value = value_in

    def set_center(self, center_val):
        """
        Set Servo center value
        Input: Value between 0 and 4095
        """
        if center_val not in range(4096):
            print("Servo value not in range [0,4095]")
        else:
            self._center = center_val
            print("Servo %2i center set to %4i" % (self.id + 1, center_val))

    def set_max(self, max_val):
        """
        Set Servo max value
        Input: Value between 0 and 4095
        """
        if max_val not in range(4096):
            print("Servo value not in range [0,4095]")
        else:
            self._max = max_val
            print("Servo %2i max set to %4i" % (self.id + 1, max_val))

    def set_min(self, min_val):
        """
        Set Servo min value
        Input: Value between 0 and 4095
        """
        if min_val not in range(4096):
            print("Servo value not in range [0,4095]")
        else:
            self._min = min_val
            print("Servo %2i min set to %4i" % (self.id + 1, min_val))


class SpotMicroServoControl:
    def __init__(self):
        rospy.loginfo("Setting Up the Spot Micro Servo Control Node...")

        # Set up and title the ros node for this code
        rospy.init_node("spot_micro_servo_control")

        # Intialize empty servo dictionary
        self.servos = {}

        # Default servo values
        default_centers = [404, 264, 346, 405, 233, 286, 196, 381, 307, 196, 381, 346]
        default_directions = [1, -1, -1, 1, 1, 1, -1, 1, 1, -1, 1, -1]

        # Create a servo dictionary with 12 ServoConvert objects
        # keys: integers 0 through 12
        # values: ServoConvert objects
        for i in range(numServos):
            self.servos[i] = ServoConvert(
                id=i,
                center_value=default_centers[i],
                min_value=min(default_extremes[i]),
                max_value=max(default_extremes[i]),
                direction=default_directions[i],
            )
        rospy.loginfo("> Servos corrrectly initialized")

        # Create empty ServoArray message with n number of Servos in its array
        self._servo_msg = ServoArray()
        for i in range(numServos):
            self._servo_msg.servos.append(Servo())

        # Create the servo array publisher
        self.ros_pub_servo_array = rospy.Publisher(
            "/servos_absolute", ServoArray, queue_size=1
        )
        rospy.loginfo("> Publisher corrrectly initialized")

        rospy.loginfo("Initialization complete")

        # Setup terminal input reading, taken from teleop_twist_keyboard
        self.settings = termios.tcgetattr(sys.stdin)

    def send_servo_msg(self):
        for servo_key, servo_obj in self.servos.items():
            self._servo_msg.servos[servo_obj.id].servo = servo_obj.id + 1
            self._servo_msg.servos[servo_obj.id].value = servo_obj.value
            # rospy.loginfo("Sending to %s command %d"%(servo_key, servo_obj.value))

        self.ros_pub_servo_array.publish(self._servo_msg)

    def reset_all_servos_center(self):
        """
        Reset all servos to their center value
        """
        for s in self.servos:
            self.servos[s].value = self.servos[s]._center

    def reset_all_servos_off(self):
        """Set all servos to off/freewheel value (pwm of 0)"""
        for s in self.servos:
            self.servos[s].value = 0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def set_servo_positions(self, positions):
        if len(positions) != len(self.servos):
            print("Error: The number of positions provided does not match the number of servos.")
            return

        for i, pos in enumerate(positions):
            minVal = min(default_extremes[i])
            maxVal = max(default_extremes[i])
            if 0 <= pos <= 4095 and minVal <= pos <= maxVal:
                self.servos[i].set_value(int(pos))
            else:
                print(f"Warning: Position {pos} for servo {i} is out of range. Skipping.")

        self.send_servo_msg()

    def run(self):
        # Set all servos to their center values
        self.reset_all_servos_center()
        self.send_servo_msg()

        # Prompt user with keyboard command information
        # Ability to control individual servo to find limits and center values
        # and ability to control all servos together

        while not rospy.is_shutdown():
            print(msg)
            userInput = input("Command?: ")

            if userInput not in validCmds:
                print("Valid command not entered, try again...")
            else:
                if userInput == "quit":
                    print("Ending program...")
                    print("Final Servo Values")
                    print("--------------------")
                    for i in range(numServos):
                        print(
                            "Servo %2i:   Min: %4i,   Center: %4i,   Max: %4i"
                            % (
                                i,
                                self.servos[i]._min,
                                self.servos[i]._center,
                                self.servos[i]._max,
                            )
                        )
                    break
                

                elif userInput == 'customPositions':
                    positions_str = input("Enter servo positions as a comma-separated list: ")
                    try:
                        positions = [float(pos.strip()) for pos in positions_str.split(',')]
                        self.set_servo_positions(positions)
                    except ValueError:
                        print("Invalid input. Please enter numeric values separated by commas.")

                elif userInput == "oneServo":
                    # Reset all servos to center value, and send command
                    self.reset_all_servos_off()
                    self.send_servo_msg()

                    # First get servo number to command
                    nSrv = -1
                    while 1:
                        userInput = int(
                            input(
                                "Which servo to control? Enter a number 1 through 12: "
                            )
                        )

                        if userInput not in range(1, numServos + 1):
                            print("Invalid servo number entered, try again")
                        else:
                            nSrv = userInput - 1
                            break

                    # Loop and act on user command
                    print("Enter command, q to go back to option select: ")
                    while 1:
                        userInput = self.getKey()

                        if userInput == "q":
                            break
                        elif userInput not in keyDict:
                            print("Key not in valid key commands, try again")
                        else:
                            keyDict[userInput](self.servos[nSrv])
                            print(
                                "Servo %2i cmd: %4i" % (nSrv, self.servos[nSrv].value)
                            )
                            self.send_servo_msg()

                elif userInput == "center":
                    # Reset all servos to center value, and send command
                    self.reset_all_servos_center()
                    self.send_servo_msg()
                elif userInput == "allServos":
                    # Reset all servos to center value, and send command
                    self.reset_all_servos_center()
                    self.send_servo_msg()

                    print("Enter command, q to go back to option select: ")
                    while 1:
                        userInput = self.getKey()

                        if userInput == "q":
                            break
                        elif userInput not in keyDict:
                            print("Key not in valid key commands, try again")
                        elif userInput in ("b", "n", "m"):
                            print(
                                "Saving values not supported in all servo control mode"
                            )
                        else:
                            for s in self.servos.values():
                                keyDict[userInput](s)
                            print("All Servos Commanded")
                            self.send_servo_msg()

            # print self._last_time_cmd_rcv, self.is_controller_connected
            # if not self.is_controller_connected:
            #     self.set_actuators_idle()

            # Set the control rate in Hz
            rate = rospy.Rate(10)
            rate.sleep()


if __name__ == "__main__":
    smsc = SpotMicroServoControl()
    smsc.run()
