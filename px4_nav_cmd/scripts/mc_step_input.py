#!/usr/bin/env python
# ROS python API
import rospy

# import needed geometry messages
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# import quat and eul transformation
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# import for script argument parsing
import argparse

# import other system/utils
import time, sys, math


# Constants
ALL_STEP_TYPES = ["pitch_rate", "roll_rate", "yaw_rate", "pitch", "roll", "yaw", "vn", "ve", "vd", "n", "e", "d"]
INDEX_PITCH_RATE = 0
INDEX_ROLL_RATE = 1
INDEX_YAW_RATE = 2
INDEX_PITCH = 3
INDEX_ROLL = 4
INDEX_YAW = 5
INDEX_VN = 6
INDEX_VE = 7
INDEX_VD = 8
INDEX_N = 9
INDEX_E = 10
INDEX_D = 11
INDICES_ATTITUDE = [INDEX_PITCH_RATE, INDEX_ROLL_RATE, INDEX_YAW_RATE, INDEX_PITCH, INDEX_ROLL, INDEX_YAW]
INDICES_POSITION = [INDEX_VN, INDEX_VE, INDEX_VD, INDEX_N, INDEX_E, INDEX_D]


#Import Classes
from FlightModes import FlightModes
from FlightParameter import FlightParams
from controller import Controller


# Offboard controller for sending setpoints
def publish_setpoint(cnt, step_type, pub_pos, pub_att):
    if ALL_STEP_TYPES.index(step_type) in [INDEX_VN, INDEX_VE, INDEX_VD, INDEX_N, INDEX_E, INDEX_D]:
        pub_pos.publish(cnt.pos_sp)
    elif ALL_STEP_TYPES.index(step_type) in [INDEX_PITCH_RATE, INDEX_ROLL_RATE, INDEX_YAW_RATE, INDEX_PITCH, INDEX_ROLL, INDEX_YAW]:
        pub_att.publish(cnt.att_sp)

def run(argv):
    # Parse arguments
    parser = argparse.ArgumentParser()

    parser.add_argument("-t", "--type", help="step input type, should be one of the following: [pitch|roll|yaw]_rate, pitch, roll, yaw, v[x|y|z], x, y, z", type=str, nargs=1)
    parser.add_argument("-v", "--value", help="the step input final value in degrees[per second] or meters[per second], depending on the type", type=float, nargs=1)
    parser.add_argument("-d", "--duration", help="the duration of the step input in seconds, if not provided the duration is infinity", type=float, nargs="?")

    args = parser.parse_args()
    if args.type != None and args.value != None:
        step_type = str(args.type[0])
        print(step_type)
        value = args.value[0]
    else:
        print("Please provide a step type and a step final value")
        sys.exit(2)
    duration = float("inf")

    # Validate arguments
    if args.duration != None:
        duration = args.duration
    elif step_type == None or value == None:
        print("Please provide a step type and a step final value")
        sys.exit(2)
    elif step_type not in ALL_STEP_TYPES:
        print("Please provide a valid step type")
        sys.exit(2)

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = FlightModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(200.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Subscribe to drone's linear velocity
    rospy.Subscriber('mavros/local_position/velocity', TwistStamped, cnt.velCb)

    # Setpoint publishers
    sp_pos_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    sp_att_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

    # Arm the drone
    print("Arming")
    while not (cnt.state.armed or rospy.is_shutdown()):
        modes.setArm()
        rate.sleep()
    print("Armed\n")

    # activate OFFBOARD mode
    print("Activate OFFBOARD mode")
    while not (cnt.state.mode == "OFFBOARD" or rospy.is_shutdown()):
        # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
        k=0
        while k<10:
            sp_pos_pub.publish(cnt.pos_sp)
            rate.sleep()
            k = k + 1

        modes.setOffboardMode()
        rate.sleep()
    print("OFFBOARD mode activated\n")

    # Takeoff
    print("Taking off")
    while not (abs(cnt.local_pos.z - cnt.takeoffHeight) < 0.2 or rospy.is_shutdown()):
        cnt.updateSp(ALL_STEP_TYPES[INDEX_D], -cnt.takeoffHeight)
        sp_pos_pub.publish(cnt.pos_sp)
        rate.sleep()
    print("Reached takeoff height of " + str(cnt.takeoffHeight) + "m")

    # Start and end the step at 0. Well, really close to 0...
    final_val = 1e-6
    # If step input is in D Direction
    if ALL_STEP_TYPES.index(step_type) == INDEX_D:
        final_val = -cnt.takeoffHeight
    elif ALL_STEP_TYPES.index(step_type) == INDEX_YAW:
        final_val = cnt.init_yaw

    # ROS main loop - first set value to zero before stepping
    zero_time = 5
    start = time.time()
    print("Holding position for 5 seconds")
    while not ((time.time() - start >= duration + zero_time) or rospy.is_shutdown()):
        if time.time() - start <= zero_time:
            cnt.updateSp(step_type, final_val)
        else:
            cnt.updateSp(step_type, value)
            sys.stdout.write("Executing progress: %d%%   \r" % ( ((time.time()-start)/(2*duration+2*zero_time))*100 ))
            sys.stdout.flush()
        publish_setpoint(cnt, step_type, sp_pos_pub, sp_att_pub)
        rate.sleep()

    # Step down for the same duration
    start = time.time()
    while not ((time.time() - start >= duration) or rospy.is_shutdown()):
        cnt.updateSp(step_type, final_val)
        sys.stdout.write("Executing progress: %d%%   \r" % ( ( (time.time()-start) / (2*duration)+0.5)*100))
        sys.stdout.flush()
        publish_setpoint(cnt, step_type, sp_pos_pub, sp_att_pub)
        rate.sleep()

    # Land quadrotor
    print("\n\nActivate LAND mode\n")
    while not (cnt.state.mode == "AUTO.LAND" or rospy.is_shutdown()):
        modes.setLandMode()
        rate.sleep()
    print("LAND mode activated\n")

def main(argv):
    try:
		run(argv)
    except rospy.ROSInterruptException:
        pass
    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
