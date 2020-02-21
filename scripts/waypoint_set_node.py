#!/usr/bin/env python
import rospy
from pyquaternion import Quaternion
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget

#class Control:
#    def __init__(self, boat_object):
#        self.object = boat_object


class Boat:
    #important: there are two different coordinate-frames for the boat!
    #A body fixed system and a local_ground system
    #we want to transform both into the NED system we are working in

    #for ENU_local_ground to NED conversion
    qx_180 = Quaternion(axis=[1, 0, 0], angle=np.pi)
    qz_90p = Quaternion(axis=[0, 0, 1], angle=np.pi / 2)
    q_rot = qz_90p*qx_180
    q = quaternion_from_euler(np.pi, 0, np.pi/2)
    print(q_rot)
    print(q)

    def __init__(self):
        self.position = np.array([0, 0, 0])
        self.orientation_quat = Quaternion()
        self.orientation_euler = euler_from_quaternion(self.orientation_quat.elements)
        self._pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self._sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.callback)

    def callback(self, data):
        #ENU_local_ground into NED
        self.position = self.q_rot.rotate(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]))
        self.orientation_quat = self.gen_quat_ned(Quaternion(w=data.pose.orientation.w, x=data.pose.orientation.x, y=data.pose.orientation.y, z=data.pose.orientation.z))
        self.orientation_euler = euler_from_quaternion(np.array([self.orientation_quat.x, self.orientation_quat.y, self.orientation_quat.z, self.orientation_quat.w]))
        # print("ENU_ROS:", euler_from_quaternion(np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])))

    def gen_quat_ned(self, quaternion):

        # ENU_body_fixed to NED
        # q_rot from ENU to NED
        # qx_180 from uuv_bodyframe to baselink
        quaternion = self.q_rot*quaternion*self.qx_180

        return quaternion

    def gen_quat_enu(self, quaternion):
        # NED to ENU_body_fixed
        # q_rot.inverse from NED to ENU
        # qx_180.inverse from baselink to uuv_bodyframe
        quaternion = self.q_rot.inverse*(quaternion*self.qx_180.inverse)

        return quaternion


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v

    return v / norm


def main():
    rospy.init_node('attitude_set')
    uuv = Boat()
    set = AttitudeTarget()

    roll = 0
    pitch = 0
    yaw = 0

    waypoint = np.array([3, 3, 0])

    q_roll = Quaternion(axis=[1, 0, 0], angle=roll)
    q_pitch = Quaternion(axis=[0, 1, 0], angle=pitch)
    q_yaw = Quaternion(axis=[0, 0, 1], angle=yaw)

    quat_euler = uuv.gen_quat_enu(q_yaw*q_pitch*q_roll)

    rate = rospy.Rate(20)


    while not rospy.is_shutdown():
        # waypoint
        uuv_orientation = normalize(uuv.orientation_quat.rotate(np.array([10, 0, 0])))
        target = normalize(np.subtract(waypoint, uuv.position))

        quat_axis = normalize(np.cross(uuv_orientation, target))
        quat_angle = np.arccos(np.dot(uuv_orientation, target))
        print("orientation:", uuv_orientation)
        if not((np.linalg.norm(quat_axis) <= 0.01) or (quat_angle == 0)):
            quat = uuv.gen_quat_enu(Quaternion(axis=quat_axis, angle=2*quat_angle))
        else:
            rot_target = uuv.gen_quat_enu(uuv.orientation_quat)
            quat = rot_target

        set.orientation.x = quat.x
        set.orientation.y = quat.y
        set.orientation.z = quat.z
        set.orientation.w = quat.w
        #set.thrust =

        if np.linalg.norm(uuv.position-waypoint) <= 0.5:
            set.thrust = 0
        else:
            set.thrust = 0.4


        uuv._pub.publish(set)
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()