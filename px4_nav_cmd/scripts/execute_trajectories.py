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
from tqdm import trange
import numpy
from generate_random_trajectories import *


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

    parser.add_argument("-n", "--number", help="the number of trajectories to execute, default: 1", type=int, nargs=1,default=1)

    args = parser.parse_args()
    numberOfTrajectories = int(args.number[0])

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

    #Extended States of the drone
    rospy.Subscriber('mavros/extended_state', ExtendedState, cnt.ExtendedstateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Subscribe to drone's linear velocity
    rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, cnt.velCb)

    # Setpoint publishers
    sp_pos_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    sp_att_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)


    i = 0
    while i < numberOfTrajectories:
        rospy.loginfo("\n--------------------------------\nTrajectory: " + str(i) + " of " + str(numberOfTrajectories) +"\n--------------------------------")
    # for i in numSim:
    # Arm the drone
        rospy.loginfo("ARMING")
        # print("Arming")
        while not (cnt.state.armed or rospy.is_shutdown()):

            modes.setArm()
            rate.sleep()

        rospy.loginfo("ARMED")
        # print("Armed\n")

        # activate OFFBOARD mode
        # print("Activate OFFBOARD mode")
        rospy.loginfo("Activate OFFBOARD mode")
        while not (cnt.state.mode == "OFFBOARD" or rospy.is_shutdown()):
            # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
            k=0
            while k<10:
                sp_pos_pub.publish(cnt.pos_sp)
                rate.sleep()
                k = k + 1

            modes.setOffboardMode()
            rate.sleep()
        rospy.loginfo("OFFBOARD mode activated")

        # Takeoff to Hover
        rospy.loginfo("Taking Off")
        while not (abs(cnt.local_pos.z - cnt.takeoffHeight) < 0.01 or rospy.is_shutdown()):
            # print(cnt.local_pos.x, cnt.local_pos.y, cnt.local_pos.z)
            cnt.updateSp(ALL_STEP_TYPES[INDEX_D], -cnt.takeoffHeight)
            sp_pos_pub.publish(cnt.pos_sp)
            rate.sleep()
        # print("Reached takeoff height of " + str(cnt.takeoffHeight) + "m")

        # Start and end the step at 0. Well, really close to 0...
        final_val = 1e-6



        x_vel_trajectory = generateCombinationInput(100000,0,0,20)
        y_vel_trajectory = generateCombinationInput(100000,0,0,20)
        z_vel_trajectory = generateCombinationInput(100000,0,0,20)
        yaw_trajectory = generateCombinationInput(100000,0,0,180)


        # ROS main loop - first set value to zero before steppig
        zero_time = 2
        start = time.time()
        delta_t = int((time.time()-start)*1000)
        # print("Holding position for 2 seconds")
        rospy.loginfo("Executing trajectory")
        while not (delta_t >= 90000 or rospy.is_shutdown() ):
            delta_t = int((time.time()-start)*1000)
            # cnt.updateTrajSp(float(roll_rate_trajectory[delta_t]),float(pitch_rate_trajectory[delta_t]),float(yaw_rate_trajectory[delta_t]))
            cnt.updateVel_andYawSp(float(x_vel_trajectory[delta_t]),float(y_vel_trajectory[delta_t]),float(z_vel_trajectory[delta_t]),float(yaw_trajectory[delta_t]))
            sp_pos_pub.publish(cnt.pos_sp)
            rate.sleep()
            # sp_att_pub.publish(cnt.att_sp)
            # rate.sleep()


        rospy.loginfo("Stabilising before landing")


        start = time.time()
        delta_t = int((time.time()-start)*1000)


        while not ( ((cnt.local_vel.x)**2 + (cnt.local_vel.y)**2 + (cnt.local_vel.z)**2)**0.5 < 0.1 or rospy.is_shutdown()   ):
            delta_t = int((time.time()-start)*1000)
            cnt.Stabilise()
            sp_pos_pub.publish(cnt.pos_sp)
            rate.sleep()
        rospy.loginfo("Reached hovering position of " + str(cnt.takeoffHeight) + "m")

        # Land quadrotor
        # print("\n\nActivate LAND mode\n")
        rospy.loginfo("Activate LAND mode")
        while not (cnt.state.mode == "AUTO.LAND" or rospy.is_shutdown()):
            modes.setLandMode()
            rate.sleep()
        # print("LAND mode activated\n")
        rospy.loginfo("LAND mode activated")


        # Waiting until drone has landed
        rospy.loginfo("Waiting for UAV to land")
        while not( cnt.extended_state.landed_state == 1):
            # print(cnt.local_pos.z)
            rate.sleep()

        rospy.loginfo("UAV on ground")
        i += 1

def main(argv):
    try:
		run(argv)
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Terminated Successfully")

if __name__ == "__main__":
    main(sys.argv[1:])
