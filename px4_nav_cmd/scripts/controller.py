
import rospy
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

# import quat and eul transformation
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import time, sys, math

from FlightModes import FlightModes
from FlightParameter import FlightParams


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

class Controller:
    def __init__(self):
        # Drone state
        self.state = State()

        self.extended_state = ExtendedState()

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 0.0)

        # A Message for the current linear velocity of the drone
        self.local_vel = Vector3(0.0, 0.0, 0.0)

        # A Message for the current attitude of the drone
        self.quat = Quaternion(0.0, 0.0, 0.0, 1.0)

        # A Message for the current angular rate of the drone
        self.ang_rate = Vector3(0.0, 0.0, 0.0)

        # Instantiate the position setpoint message
        self.pos_sp = PositionTarget()
        # set the flag to control height
        self.pos_sp.type_mask = int('110111111000', 2)
        # LOCAL_NED
        self.pos_sp.coordinate_frame = 1
        # initial values for setpoints
        self.pos_sp.position.x = 0.0
        self.pos_sp.position.y = 0.0
        self.pos_sp.position.z = 0.0

        # Instantiate the attitude setpoint message
        self.att_sp = AttitudeTarget()
        # set the default flag
        self.att_sp.type_mask = int('11000111', 2)
        # initial values for setpoints
        self.att_sp.orientation.w = 1.0
        self.att_sp.orientation.x = 0.0
        self.att_sp.orientation.y = 0.0
        self.att_sp.orientation.z = 0.0

        # Obtain flight parameters
        params = FlightParams()
        self.takeoffHeight = params.getTakeoffHeight()
        self.hoverThrust = params.getHoverThrust()

        # self.maxPitchRate = params.getMaxPitchRate()
        # self.maxRollRate = params.getMaxRollRate()
        # self.maxYawRate = params.getMaxYawRate()


        # Set initial yaw angle to unknown
        self.init_yaw = None

        # Hover position for drone
        self.hover_pos = Point(0.0, 0.0, -self.takeoffHeight)

    # Update setpoint message
    def updateSp(self, step_type, step_val):
        # Set default values
        self.pos_sp.position.x = 0
        self.pos_sp.position.y = 0
        self.pos_sp.position.z = self.takeoffHeight

        self.pos_sp.velocity.x = 0
        self.pos_sp.velocity.y = 0
        self.pos_sp.velocity.z = 0

        self.att_sp.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, math.radians(90 + self.init_yaw)))

        self.att_sp.body_rate.x = 0
        self.att_sp.body_rate.y = 0
        self.att_sp.body_rate.z = 0

        self.att_sp.thrust = self.hoverThrust

        # Set step value
        if ALL_STEP_TYPES.index(step_type) == INDEX_VN:
            self.pos_sp.velocity.y = step_val
        elif ALL_STEP_TYPES.index(step_type) == INDEX_VE:
            self.pos_sp.velocity.x = step_val
        elif ALL_STEP_TYPES.index(step_type) == INDEX_VD:
            self.pos_sp.velocity.z = -step_val
        elif ALL_STEP_TYPES.index(step_type) == INDEX_N:
            self.pos_sp.position.y = step_val
        elif ALL_STEP_TYPES.index(step_type) == INDEX_E:
            self.pos_sp.position.x = step_val
        elif ALL_STEP_TYPES.index(step_type) == INDEX_D:
            self.pos_sp.position.z = -step_val
        elif ALL_STEP_TYPES.index(step_type) == INDEX_ROLL_RATE:
            self.att_sp.body_rate.x = math.radians(step_val)
        elif ALL_STEP_TYPES.index(step_type) == INDEX_PITCH_RATE:
            self.att_sp.body_rate.y = math.radians(step_val)
        elif ALL_STEP_TYPES.index(step_type) == INDEX_YAW_RATE:
            self.att_sp.body_rate.z = math.radians(step_val)
        elif ALL_STEP_TYPES.index(step_type) == INDEX_ROLL:
            self.att_sp.orientation = Quaternion(*quaternion_from_euler(math.radians(step_val), 0.0, math.radians(90 + self.init_yaw)))
        elif ALL_STEP_TYPES.index(step_type) == INDEX_PITCH:
            self.att_sp.orientation = Quaternion(*quaternion_from_euler(0.0, math.radians(step_val), math.radians(90 + self.init_yaw)))
        elif ALL_STEP_TYPES.index(step_type) == INDEX_YAW:
            self.att_sp.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, math.radians(90 + step_val)))

        # Set mask
        if ALL_STEP_TYPES.index(step_type) in [INDEX_VN, INDEX_VE, INDEX_VD]:
            self.pos_sp.type_mask = int('110111000111', 2)
        elif ALL_STEP_TYPES.index(step_type) in [INDEX_N, INDEX_E, INDEX_D]:
            self.pos_sp.type_mask = int('110111111000', 2)
        elif ALL_STEP_TYPES.index(step_type) in [INDEX_PITCH_RATE, INDEX_ROLL_RATE, INDEX_YAW_RATE]:
            self.att_sp.type_mask = int('10111000', 2)
        elif ALL_STEP_TYPES.index(step_type) in [INDEX_PITCH, INDEX_ROLL, INDEX_YAW]:
            self.att_sp.type_mask = int('00111111', 2)

    # Callbacks.

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg


    def ExtendedstateCb(self,msg):
        self.extended_state = msg;
    ## Drone local position callback
    def posCb(self, msg):
        # print("meep")
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

        self.quat.w = msg.pose.orientation.w
        self.quat.x = msg.pose.orientation.x
        self.quat.y = msg.pose.orientation.y
        self.quat.z = msg.pose.orientation.z

        # Set initial yaw angle
        if self.init_yaw is None:
            self.init_yaw = -90 + math.degrees(euler_from_quaternion([self.quat.x, self.quat.y, self.quat.z, self.quat.w])[2])

    ## Drone linear velocity callback
    def velCb(self, msg):
        # self.local_vel.x = msg.twist.linear.x
        # self.local_vel.y = msg.twist.linear.y
        # self.local_vel.z = msg.twist.linear.z

        self.local_vel = msg.twist.linear
        self.ang_rate = msg.twist.angular

        # self.ang_rate.x = msg.twist.angular.x
        # self.ang_rate.y = msg.twist.angular.y
        # self.ang_rate.z = msg.twist.angular.z


    # Update setpoint message
    def returnToHoveringPosition(self):
        # Set default values
        # self.pos_sp.position.x = 0
        # self.pos_sp.position.y = 0

        self.pos_sp.position.x = self.local_pos.x
        self.pos_sp.position.y = self.local_pos.y
        self.pos_sp.position.z = self.local_pos.z + 1

        self.pos_sp.velocity.x = 0
        self.pos_sp.velocity.y = 0
        self.pos_sp.velocity.z = 0

        self.att_sp.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, math.radians(90 + self.init_yaw)))

        self.att_sp.body_rate.x = 0
        self.att_sp.body_rate.y = 0
        self.att_sp.body_rate.z = 0

        self.att_sp.thrust = self.hoverThrust

        # Set mask
        self.pos_sp.type_mask = int('110111111000', 2)


    def Stabilise(self):

        self.pos_sp.position.x = 0
        self.pos_sp.position.y = 0
        self.pos_sp.position.z = self.takeoffHeight

        self.pos_sp.velocity.x = 0
        self.pos_sp.velocity.y = 0
        self.pos_sp.velocity.z = 0

        self.att_sp.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, math.radians(90 + self.init_yaw)))

        self.att_sp.body_rate.x = 0
        self.att_sp.body_rate.y = 0
        self.att_sp.body_rate.z = 0

        self.att_sp.thrust = self.hoverThrust

        self.pos_sp.type_mask = int('110111000111', 2)



    # Update next trajectory setpoint
    def updateTrajSp(self,x,y,z):
        # Set default values
        self.pos_sp.position.x = 0
        self.pos_sp.position.y = 0
        self.pos_sp.position.z = self.takeoffHeight

        self.pos_sp.velocity.x = 0
        self.pos_sp.velocity.y = 0
        self.pos_sp.velocity.z = 0

        self.att_sp.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, math.radians(90 + self.init_yaw)))

        self.att_sp.body_rate.x = math.radians(x)
        self.att_sp.body_rate.y = math.radians(y)
        self.att_sp.body_rate.z = math.radians(z)

        self.att_sp.thrust = self.hoverThrust

        # Set mask for angular controller
        self.att_sp.type_mask = int('10111000', 2)
        # self.att_sp.type_mask = 7




    # Update velocity setpoint
    def updateVel_andYawSp(self, velx,vely,velz,yaw):
        # Set default values
        self.pos_sp.position.x = 0
        self.pos_sp.position.y = 0
        self.pos_sp.position.z = self.takeoffHeight

        self.pos_sp.velocity.x = 0
        self.pos_sp.velocity.y = 0
        self.pos_sp.velocity.z = 0

        self.att_sp.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, math.radians(90 + self.init_yaw)))

        self.att_sp.body_rate.x = 0
        self.att_sp.body_rate.y = 0
        self.att_sp.body_rate.z = 0

        self.att_sp.thrust = self.hoverThrust

        # Set step value
        self.pos_sp.velocity.y = velx
        self.pos_sp.velocity.x = vely
        self.pos_sp.velocity.z = -velz

        # self.att_sp.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, math.radians(90 + yaw)))

        # Set mask for velocity and yaw setpoint
        self.pos_sp.type_mask = int('110111000111', 2)
