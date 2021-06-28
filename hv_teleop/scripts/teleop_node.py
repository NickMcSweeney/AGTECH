#!/usr/bin/env python

from __future__ import print_function

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import sys, select, termios, tty
import argparse, time


def resource_name(name):
    # formats a hostname to be legaly used in a ros pubisher/subscriber name
    return ((name.replace("-", "")).replace(".", "_")).lower()


def progressbar(it, prefix="", size=60, file=sys.stdout):
    count = len(it)

    def show(j):
        x = int(size * j / count)
        p = int(100 * j / count)
        file.write("%s[%s%s] %i%s\r" % (prefix, "#" * x, "." * (size - x), p, "%"))
        file.flush()

    show(0)
    for i, item in enumerate(it):
        yield item
        show(i + 1)
    file.write("\n")
    file.flush()


# variables
pub_drive = None
pub_gripper = None
pub_arm = None

# set the velocity mesage
vel_msg = Twist()

# The control instructions
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Emergancy stop (without deceleration) SPACEBAR

anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

t/b : arm out/in (10%)
r/v : gripper open/close (10%)

CTRL-C to quit
"""

# Set key bindings
moveBindings = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
}

speedBindings = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1),
    "x": (0.9, 1),
    "e": (1, 1.1),
    "c": (1, 0.9),
}

posBindings = {
    "t": (-1, 0),
    "b": (1, 0),
    "r": (0, 1),
    "v": (0, -1),
}

# functions
def getKey():  # get the key that was pressed
    tty.setraw(sys.stdin.fileno())
    # if there is no keypress within 0.033 seconds then the reading defaults to a press of the k key.
    rlist, _, _ = select.select([sys.stdin], [], [], 0.033)
    key = sys.stdin.read(1) if rlist else "k"
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def pos(arm, gripper):
    return "currently:\tgripper %s\tarm %s " % (gripper, arm)


class VControl:
    ###
    # Velocity management class
    # this allows the robot to accelerate and decelerate in a controlled manner
    # the goal it to prevent jerk or rock in the robot when stopping and starting
    ###
    def __init__(self, s):
        self.dt = 0.01  # Set a time interval. base this on the data publish rate.
        self.vel = 0
        self.speed = s

    def getVel(self, set_point):
        # set the velocity
        diff = 0  # the distance between the current/origin point and the goal/set point
        inc = 0  # the ammount that the velocity should change in one time interval

        # Set the diff and inc variables
        if set_point == 0 and abs(self.vel) >= 0.05:
            # Use this block for when the robot is slowing down to a stop
            diff = abs(self.vel)
            inc = (
                abs((self.speed + abs(self.vel)) / 25)
                if self.vel < 0
                else (-1) * abs((self.speed + abs(self.vel)) / 25)
            )
        else:
            # use this for when the robot is speeding up
            diff = abs(set_point - self.vel)
            inc = (abs(set_point) - diff / 2) / 20 if diff <= abs(set_point) else 0.1

        # update the velocities
        if diff < 0.05:  # when slow enough set to 0
            self.vel = set_point
        elif set_point > 0.05:  # when accelerating to a forward velocity
            vel = self.vel + inc
            self.vel = vel if abs(vel) < abs(set_point) else set_point
        elif set_point < -0.05:  # when accelerating to a reverse velocity
            vel = self.vel - inc
            self.vel = vel if abs(vel) < abs(set_point) else set_point
        elif set_point == 0:  # when deccelerating to a stop
            vel = self.vel + inc
            self.vel = vel
        else:  # if confused (this block shoud not get called) stop
            self.vel = 0

        return self.vel


# main fn
if __name__ == "__main__":
    ###
    # Teleoperation node
    # uses keyboard commands to publish velocity, gipper and arm values with ROS
    # should be run allong with services node
    ###
    parser = argparse.ArgumentParser(
        description="ROS Connection Application for comunication and control of the HV Robots"
    )
    # Add the arguments to handle setting various sensor and controller connections
    parser.add_argument(
        "-n",
        "--name",
        "--hostname",
        metavar="hostname",
        dest="hostname",
        type=str,
        default="hai-1095.local",
        nargs="?",
        help="The name of the robot, generally of the form hai-####.local (default: hai-1095.local",
    )
    parser.add_argument(
        "sys",
        type=str,
        nargs="*",
        help="System included variables from launch file (not set bu user)",
    )
    args = parser.parse_args()

    # set the hostname based on the argument inputed by the user (default: hai-1095.local)
    host_name = args.hostname

    if len(args.sys) > 0:
        # started from launch file - need to wait for bridge to connect
        for i in progressbar(range(100), "STARTING: ", 50):
            time.sleep(0.1)

    print("Controling %s robot" % host_name)

    settings = termios.tcgetattr(sys.stdin)

    name = resource_name(host_name)

    # create the 3 publishers
    pub_drive = rospy.Publisher("/" + name + "/cmd_vel", Twist, queue_size=1)
    pub_gripper = rospy.Publisher(
        "/" + name + "/set_gripper_pos", Float32, queue_size=1
    )
    pub_arm = rospy.Publisher("/" + name + "/set_arm_pos", Float32, queue_size=1)

    # init ros
    rospy.init_node("hv_teleop_node", anonymous=True)

    # set default values
    speed = rospy.get_param("~speed", 1.5)
    turn = rospy.get_param("~turn", 1.0)
    arm = rospy.get_param("~arm", 100.0)
    gripper = rospy.get_param("~gripper", 0.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    # create velocity control instance
    c = VControl(speed)

    try:
        # print out the instructions
        print(msg)
        print(vels(speed, turn))
        print(pos(arm, gripper))

        # Create the msgs for arm and gripper
        arm_msg = Float32()
        gripper_msg = Float32()

        rate = rospy.Rate(100)  # set update rate
        # Start the ROS main loop
        while not rospy.is_shutdown():
            key = getKey()  # get the key press
            if key in moveBindings.keys():  # if key is a movement key
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():  # if key is for setting the speed
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                # print out the new velocity
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif (
                key in posBindings.keys()
            ):  # is key is for setting the arm or gripper position
                arm = (
                    arm + posBindings[key][0]
                    if 0 <= arm + posBindings[key][0] <= 100
                    else arm
                )
                gripper = (
                    gripper + posBindings[key][1]
                    if 0 <= gripper + posBindings[key][1] <= 100
                    else gripper
                )

                # print out the new positions
                print(pos(arm, gripper))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:  # otherwise set movement to 0
                x = 0
                y = 0
                z = 0
                th = 0
                if key == "\x03":
                    break

            # Velocity message to send to the robot
            vel_msg = Twist()
            vel_msg.linear.x = c.getVel(x * speed)  # ...linear velocity
            vel_msg.angular.z = th * turn
            # ...rotational velocity

            # if there is novel position data publish, otherwise skip
            # this saves resources for the robot since transmitting duplicate information
            # in mp consumes resources for no reason
            if arm_msg.data != arm:
                arm_msg.data = arm
                pub_arm.publish(arm_msg)
            if gripper_msg.data != gripper:
                gripper_msg.data = gripper
                pub_gripper.publish(gripper_msg)

            if key == " ":
                # if the SPACEBAR is pressed the velocity is set to 0 regardless of anything else
                # do Emergancy stop
                print("E STOP")
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                pub_drive.publish(vel_msg)
            elif vel_msg.linear.x != 0 or vel_msg.angular.z != 0:
                # if the velocity is not 0 publish it.
                print("control vel out: %s" % vel_msg.linear.x)
                pub_drive.publish(vel_msg)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(e)
    finally:
        # set velocity to 0 before ending the script
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub_drive.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
