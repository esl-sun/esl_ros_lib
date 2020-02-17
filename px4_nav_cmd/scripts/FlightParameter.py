
import rospy
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

import time, sys, math
# import quat and eul transformation
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Flight parameters class
# Flight parameters are activated using ROS services
class FlightParams:
    def __init__(self):
        # pull all parameters
        # rospy.wait_for_service('mavros/param/pull')
        pulled = False
        while not pulled:
            try:
                paramService = rospy.ServiceProxy('mavros/param/pull', mavros_msgs.srv.ParamPull)
                parameters_recv = paramService(True)
                pulled = parameters_recv.success
            except rospy.ServiceException, e:
                print "service param_pull call failed: %s. Could not retrieve parameter list."%e

    def getTakeoffHeight(self):
        return rospy.get_param('mavros/param/MIS_TAKEOFF_ALT')

    def getHoverThrust(self):
        return rospy.get_param('mavros/param/MPC_THR_HOVER')

    def getMaxRollRate(self):
        return rospy.get_param('mavros/param/FW_ACRO_X_MAX')

    def getMaxPitchRate(self):
        return rospy.get_param('mavros/param/FW_ACRO_Y_MAX')

    def getMaxYawRate(self):
        return rospy.get_param('mavros/param/FW_ACRO_Z_MAX')
