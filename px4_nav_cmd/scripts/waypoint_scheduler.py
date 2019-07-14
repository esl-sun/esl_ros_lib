#!/usr/bin/env python
# ROS python API
import rospy

# import needed geometry messages
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# import other system/utils
import time, sys, math

# Global variables
# Waypoints = [N, E, D, Yaw (deg)]
waypoints = [
             [0, 0, 0, 0],
             [0, 0, 2, 0],
             [16, 0, 2, 0],
             [16, 12, 2, 0],
             [16, 0, 2, 0],
             [8, 0, 2, 0],
             [8, 8, 2, 0],
             [8, 0, 2, 0],
             [0, 0, 2, 0],

             [0, 22, 2, 0],

             [4, 26, 2, 0],
             [8, 22, 2, 0],
             [8, 18, 2, 0],
             [12, 14, 2, 0],
             [16, 18, 2, 0],

             [16, 30, 2, 0],

             [0, 30, 2, 0],
             [0, 42, 2, 0],
            ]
threshold = 0.3 # How small the position error and velocity should be before sending the next waypoint
waypoint_time = -1 # If < 0, will send next waypoint when current one is reached. If >= 0, will send next waypoint after the amount of time has passed

# Flight modes class
# Flight modes are activated using ROS services
class FlightModes:
    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Land Mode could not be set."%e

# Offboard controller for sending setpoints
class Controller:
    def __init__(self):
        # Drone state
        self.state = State()

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)

        # A Message for the current linear velocity of the drone
        self.local_vel = Vector3(0.0, 0.0, 0.0)

        # Instantiate the position setpoint message
        self.pos_sp = PositionTarget()
        # set the flag to control height
        self.pos_sp.type_mask = int('100111111000', 2)
        # LOCAL_NED
        self.pos_sp.coordinate_frame = 1
        # initial values for setpoints
        self.pos_sp.position.x = 0.0
        self.pos_sp.position.y = 0.0
        self.pos_sp.position.z = 0.0
        self.pos_sp.yaw = math.radians(90)

    # Update setpoint message
    def updateSp(self, n_step, e_step, d_step, yaw_step):
        # Set step value
        self.pos_sp.position.y = n_step
        self.pos_sp.position.x = e_step
        self.pos_sp.position.z = -d_step
        self.pos_sp.yaw = math.radians(90 + yaw_step)

        # Set mask
        self.pos_sp.type_mask = int('100111111000', 2) 

    # Callbacks.

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Drone local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone linear velocity callback
    def velCb(self, msg):
        self.local_vel.x = msg.twist.linear.x
        self.local_vel.y = msg.twist.linear.y
        self.local_vel.z = msg.twist.linear.z

def publish_setpoint(cnt, pub_pos):
    pub_pos.publish(cnt.pos_sp)

def run(argv):
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = FlightModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)

    # Subscribe to drone's linear velocity
    rospy.Subscriber('mavros/local_position/velocity', TwistStamped, cnt.velCb)

    # Setpoint publishers   
    sp_pos_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

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

    # ROS main loop - first set value to zero before stepping
    current_wp = 0
    last_time = time.time()
    print("Following waypoints...")
    print("Executing waypoint %d / %d" % (current_wp + 1, len(waypoints)))
    while current_wp < len(waypoints) and not rospy.is_shutdown():
        y = waypoints[current_wp][0]
        x = waypoints[current_wp][1]
        z = waypoints[current_wp][2]

        cnt.updateSp(waypoints[current_wp][0], waypoints[current_wp][1], -waypoints[current_wp][2], -waypoints[current_wp][3])
        publish_setpoint(cnt, sp_pos_pub)
        rate.sleep()

        if waypoint_time < 0:
            if abs(cnt.local_pos.x - x) < threshold and abs(cnt.local_vel.x) < threshold and abs(cnt.local_pos.y - y) < threshold and abs(cnt.local_vel.y) < threshold and abs(cnt.local_pos.z - z) < threshold and abs(cnt.local_vel.z) < threshold:
                current_wp = current_wp + 1
                print("Executing waypoint %d / %d" % (current_wp + 1, len(waypoints)))
        else:
            current_time = time.time()
            if current_time - last_time >= waypoint_time:
                current_wp = current_wp + 1
                print("Executing waypoint %d / %d" % (current_wp + 1, len(waypoints)))
                last_time = current_time
    print("Last waypoint reached\n")

    # wait before landing
    print("Waiting for user to terminate (press Ctrl-C)...")
    init_time = time.time()
    current_time = time.time()
    while not rospy.is_shutdown():
        cnt.updateSp(waypoints[current_wp - 1][0], waypoints[current_wp - 1][1], -waypoints[current_wp - 1][2], -waypoints[current_wp - 1][3])
        publish_setpoint(cnt, sp_pos_pub)
        rate.sleep()

        current_time = time.time()
    print("Done\n")

def main(argv):
    try:
		run(argv)
    except rospy.ROSInterruptException:
        pass
    print("Terminated.\n")

if __name__ == "__main__":
    main(sys.argv[1:])
