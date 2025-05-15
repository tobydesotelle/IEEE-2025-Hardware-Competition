#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import sys, select, termios, tty

# Set maximum velocities
MAX_LIN_VEL = 3.0  # Maximum linear velocity (adjust as needed)
MAX_ANG_VEL = 1.0  # Maximum angular velocity (adjust as needed)

# Increments for velocity changes
LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.1

beacon = True
tilt_grab = True

def getKey():
    """Capture keyboard input."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    try:
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':  # Arrow keys start with an escape character
                key += sys.stdin.read(2)  # Read additional characters
        else:
            key = ''
    except Exception as e:
        key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input_vel, low, high):
    """Ensure the velocity stays within the specified limits."""
    if input_vel < low:
        input_vel = low
    elif input_vel > high:
        input_vel = high
    return input_vel

def checkVelocityLimit(vel):
    """Check linear velocity limits."""
    return constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)

def checkAngularVelocityLimit(omega):
    """Check angular velocity limits."""
    return constrain(omega, -MAX_ANG_VEL, MAX_ANG_VEL)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_velocity_control')

    # Publisher for velocity commands
    vel_pub = rospy.Publisher('local_velocities', Vector3, queue_size=10)

    # Publishers for other actions
    scoop_pub = rospy.Publisher('/OpenCR/scoop', Bool, queue_size=10)
    scoop_tilt_pub = rospy.Publisher('/OpenCR/scoop_tilt', Bool, queue_size=10)
    beacon_pub = rospy.Publisher('/OpenCR/beacon_place', Bool, queue_size=10)
    dump_geo_pub = rospy.Publisher('/OpenCR/dump_geo', Bool, queue_size=10)
    dump_neo_pub = rospy.Publisher('/OpenCR/dump_neo', Bool, queue_size=10)
    grab_tilt_pub = rospy.Publisher('/OpenCR/tilt_grab_up', Bool, queue_size=10)

    # Initialize velocities
    vx = 0.0  # Linear velocity in x-direction
    vy = 0.0  # Linear velocity in y-direction
    omega = 0.0  # Angular velocity around z-axis

    try:
        print("Control Your Robot!")
        print("---------------------------")
        print("Moving around:")
        print("   W    ")
        print(" A   D ")
        print("   S    ")
        print("Left/Right arrows to rotate")
        print("Numbers 1-6 for actions")
        print("1: Activate Scoop")
        print("2: Tilt Scoop")
        print("3: Place Beacon")
        print("4: Dump Geo")
        print("5: Dump Neo")
        print("6: Tilt Grab Up")
        print("Spacebar to stop")
        print("CTRL+C to quit")
        print("---------------------------")

        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w':
                vy = checkVelocityLimit(vy + LIN_VEL_STEP_SIZE)
            elif key == 's':
                vy = checkVelocityLimit(vy - LIN_VEL_STEP_SIZE)
            elif key == 'a':
                vx = checkVelocityLimit(vx - LIN_VEL_STEP_SIZE)
            elif key == 'd':
                vx = checkVelocityLimit(vx + LIN_VEL_STEP_SIZE)
            elif key == '\x1b[D':  # Left arrow key
                omega = checkAngularVelocityLimit(omega - ANG_VEL_STEP_SIZE)
            elif key == '\x1b[C':  # Right arrow key
                omega = checkAngularVelocityLimit(omega + ANG_VEL_STEP_SIZE)
            elif key == ' ':
                # Stop all movement
                vx = 0.0
                vy = 0.0
                omega = 0.0
            elif key == '1':
                # Activate Scoop
                scoop_cmd = Bool()
                scoop_cmd.data = True
                scoop_pub.publish(scoop_cmd)
                rospy.loginfo("Scoop activated.")
            elif key == '2':
                # Tilt Scoop
                scoop_tilt_cmd = Bool()
                scoop_tilt_cmd.data = True
                scoop_tilt_pub.publish(scoop_tilt_cmd)
                rospy.loginfo("Scoop tilted.")
            elif key == '3':
                # Place Beacon
                beacon_cmd = Bool()
                beacon_cmd.data = beacon
                beacon = not beacon
                beacon_pub.publish(beacon_cmd)
                rospy.loginfo("Beacon placed.")
            elif key == '4':
                # Dump Geo
                dump_cmd = Bool()
                dump_cmd.data = True
                dump_geo_pub.publish(dump_cmd)
                rospy.loginfo("Dumped GEObag.")
            elif key == '5':
                # Dump Neo
                dump_cmd = Bool()
                dump_cmd.data = True
                dump_neo_pub.publish(dump_cmd)
                rospy.loginfo("Dumped NEObag.")
            elif key == '6':
                # Toggle Tilt Grab Up
                grab_cmd = Bool()
                grab_cmd.data = tilt_grab # Toggle between True and False
                tilt_grab = not tilt_grab
                grab_tilt_pub.publish(grab_cmd)
                rospy.loginfo("Grab toggled. Current state: {}".format(grab_cmd.data))
            elif key == '\x03':  # CTRL+C
                break

            # Publish the velocities
            vel_msg = Vector3()
            vel_msg.x = vx
            vel_msg.y = vy
            vel_msg.z = omega  # Omega assigned to z-component for angular velocity

            vel_pub.publish(vel_msg)

    except Exception as e:
        print("An error occurred: {}".format(e))

    finally:
        # Reset velocities before exiting
        vel_msg = Vector3()
        vel_msg.x = 0.0
        vel_msg.y = 0.0
        vel_msg.z = 0.0
        vel_pub.publish(vel_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
