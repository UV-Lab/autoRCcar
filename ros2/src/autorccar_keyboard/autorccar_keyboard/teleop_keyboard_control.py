## Reference) https://github.com/ros2/teleop_twist_keyboard

import sys, os

import std_msgs.msg
import rclpy

import termios, tty
from autorccar_interfaces.msg import ControlCommand

settings = termios.tcgetattr(sys.stdin)

disable = 0.0

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Input key layout:
   q    w    e
   a    s    d
   z         c

1~9 : increase/decrease speed step size

q/e : reset speed/turn value
z/c : minimum/maximum servo direction

w/s : increase/decrease only linear speed by step size
a/d : increase/decrease only angular speed by 1

i/o/p : publish command_topic (i=Stop, o=Start, p=Manual)

CTRL-C to quit
"""
keyBindings = {
    "i": (0, 0),
    "o": (1, 0),
    "p": (2, 0),
}

speedBindings = {
    "1": (1, 0),
    "2": (2, 0),
    "3": (3, 0),
    "4": (4, 0),
    "5": (5, 0),
    "6": (6, 0),
    "7": (10, 0),
    "8": (50, 0),
    "9": (100, 0),
}

moveBindings = {
    "w": (1, 0),
    "s": (-1, 0),
    "a": (0, -1),
    "d": (0, 1),
    "q": (0, 0),
    "e": (0, 0),
    "z": (0, -90),
    "c": (0, 90),
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, steering_angle):
    return "currently:\tspeed %s\tsteering angle %s " % (
        speed,
        steering_angle * 180 / 3.141592,
    )


def main():
    rclpy.init()

    node = rclpy.create_node("teleop_twist_keyboard")

    node.declare_parameter("max_steering_angle", 0.7854)
    max_steering_angle = node.get_parameter("max_steering_angle").value

    pub_control_command = node.create_publisher(
        ControlCommand, "teleop/control_command", 10
    )
    pub_command = node.create_publisher(std_msgs.msg.Int8, "teleop/command", 10)

    speed = 0.0
    steering_angle = 0.0
    step = 1.0
    status = 0.0

    try:
        print(msg)
        print(vels(speed, steering_angle))
        while True:
            key = getKey(settings)
            if key in speedBindings.keys():
                step = speedBindings[key][0]

            elif key in keyBindings.keys():
                # command = keyBindings[key][0]
                command = std_msgs.msg.Int8()
                command.data = keyBindings[key][0]
                pub_command.publish(command)

                if command.data == 0:
                    status_cm = "Stop"
                elif command.data == 1:
                    status_cm = "Manual"
                elif command.data == 2:
                    status_cm = "Auto"
                print("[Publish] command_topic = %d (%s)" % (command.data, status_cm))

            elif key in moveBindings.keys():
                speed = speed + moveBindings[key][0] * step
                steering_angle = steering_angle + moveBindings[key][1] * -1 * 0.0175

                if key == "q":
                    speed = 0.0
                elif key == "e":
                    steering_angle = 0.0

                if key == "z":
                    steering_angle = -max_steering_angle
                elif key == "c":
                    steering_angle = max_steering_angle

                if speed < 0:
                    speed = 0.0
                if steering_angle > max_steering_angle:
                    steering_angle = max_steering_angle
                elif steering_angle < -max_steering_angle:
                    steering_angle = -max_steering_angle

                print(vels(speed, steering_angle), "\tstep : ", step)
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                if key == "\x03":
                    break
            control_command = ControlCommand()
            control_command.speed = speed
            control_command.steering_angle = steering_angle
            pub_control_command.publish(control_command)

    except Exception as e:
        print(e)

    finally:
        control_command = ControlCommand()
        control_command.speed = 0.0
        control_command.steering_angle = 0.0
        pub_control_command.publish(control_command)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
