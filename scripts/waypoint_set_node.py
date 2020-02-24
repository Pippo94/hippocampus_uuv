#!/usr/bin/env python
import rospy
from pyquaternion import Quaternion
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from transformations import unit_vector
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

    def __init__(self):
        self.position = np.array([0, 0, 0])
        self.orientation_quat = Quaternion()
        self.orientation_euler = euler_from_quaternion(self.orientation_quat.elements)
        self._pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self._sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.callback)
        self.is_done = False

    def callback(self, data):
        #ENU_local_ground into NED
        self.position = self.q_rot.rotate(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]))
        self.orientation_quat = self.gen_quat_ned(Quaternion(w=data.pose.orientation.w, x=data.pose.orientation.x, y=data.pose.orientation.y, z=data.pose.orientation.z))
        self.orientation_euler = euler_from_quaternion(np.array([self.orientation_quat.x, self.orientation_quat.y, self.orientation_quat.z, self.orientation_quat.w]))
        # print("ENU_ROS:", euler_from_quaternion(np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])))

    def set_waypoint(self, waypoint):
        # expect a numpy array [x, y, z]
        self.is_done = False
        set_target = AttitudeTarget()
        rate = rospy.Rate(25)

        while not self.is_done:
            uuv_orientation = unit_vector(self.orientation_quat.rotate(np.array([10, 0, 0])))
            target = unit_vector(10*np.subtract(waypoint, self.position))
            quat_axis = unit_vector(np.cross(uuv_orientation, target))
            quat_angle = np.arccos(np.dot(uuv_orientation, target))
            quat = self.gen_quat_enu(self.orientation_quat)

            if np.linalg.norm(self.position-waypoint) <= 0.1:
                set_target.thrust = 0
                self.is_done = True
                print("Waypoint set!")
            else:
                if not ((np.linalg.norm(quat_axis) <= 0.01) or (quat_angle == 0)):
                    quat = self.gen_quat_enu(Quaternion(axis=quat_axis, angle=2 * quat_angle))
                    set_target.thrust = 0.4

            set_target.orientation.x = quat.x
            set_target.orientation.y = quat.y
            set_target.orientation.z = quat.z
            set_target.orientation.w = quat.w
            self._pub.publish(set_target)
            rate.sleep()
        self.is_done = False

    def set_attitude(self, euler_angles):
        # expects numpy array euler_angles = [roll, pitch, yaw] in radiant
        self.is_done = False
        rate = rospy.Rate(25)

        q_roll = Quaternion(axis=[1, 0, 0], angle=euler_angles[0])  # roll
        q_pitch = Quaternion(axis=[0, 1, 0], angle=euler_angles[1])  # pitch
        q_yaw = Quaternion(axis=[0, 0, 1], angle=euler_angles[2])  # yaw

        quat = self.orientation_quat
        quat_euler = self.gen_quat_enu(q_yaw*q_pitch*q_roll)

        set_attitude = AttitudeTarget()

        while not self.is_done:
            print("request", quat_euler.elements, "ist", self.orientation_quat.elements)
            print("norm", np.linalg.norm(quat_euler.elements-self.orientation_quat.elements))
            if np.linalg.norm(self.orientation_quat.elements-self.gen_quat_ned(quat_euler).elements) <= 0.01:
                print("Attitude set!")
                self.is_done = True
            else:
                quat = quat_euler

            set_attitude.orientation.x = quat.x
            set_attitude.orientation.y = quat.y
            set_attitude.orientation.z = quat.z
            set_attitude.orientation.w = quat.w
            self._pub.publish(set_attitude)
            rate.sleep()
        self.is_done = False

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


def main():
    rospy.init_node('attitude_set')
    uuv = Boat()

    roll = 0
    pitch = 0.3
    yaw = 0

    #waypoint = np.array([3, 3, 1])

    rate = rospy.Rate(0.5)

    while not uuv.is_done:
        uuv.set_attitude(np.array([roll, pitch, yaw]))
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    main()