#!/usr/bin/env python
import rospy
from pyquaternion import Quaternion
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from transformations import unit_vector
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget


class Boat:
    # important: there are two different coordinate-frames for the boat!
    # A body fixed system and a local_ground system
    # we want to transform both into the NED system we are working in

    # for ENU_local_ground to NED conversion
    qx_180 = Quaternion(axis=[1, 0, 0], angle=np.pi)
    qz_90p = Quaternion(axis=[0, 0, 1], angle=np.pi / 2)
    q_rot = qz_90p*qx_180

    def __init__(self):
        self.position = np.array([0, 0, 0])
        self.orientation_quat = Quaternion(axis=[1, 0, 0], angle=0)
        self.orientation_euler = euler_from_quaternion(np.array([self.orientation_quat.x, self.orientation_quat.y, self.orientation_quat.z, self.orientation_quat.w]))
        self._pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self._sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.callback)
        self.is_done = False

    def callback(self, data):
        #ENU_local_ground into NED
        self.position = self.q_rot.rotate(np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]))
        self.orientation_quat = self.gen_quat_ned(Quaternion(w=data.pose.orientation.w, x=data.pose.orientation.x, y=data.pose.orientation.y, z=data.pose.orientation.z))
        self.orientation_euler = euler_from_quaternion(np.array([self.orientation_quat.x, self.orientation_quat.y, self.orientation_quat.z, self.orientation_quat.w]))

    def set_waypoint(self, waypoint):
        # expect a numpy array [x, y, z]
        self.is_done = False
        set_target = AttitudeTarget()
        rate = rospy.Rate(25)

        while not self.is_done:
            # initialise vector orientated as the uuv
            uuv_orientation = unit_vector(self.orientation_quat.rotate(np.array([10, 0, 0])))
            # target points from uuv to waypoint
            target = unit_vector(100*np.subtract(waypoint, self.position))  # 100 for rounding errors
            quat_axis = unit_vector(np.cross(uuv_orientation, target))  # turning axis for quaternion rotation
            quat_angle = np.arccos(np.dot(uuv_orientation, target))  # turning angle for quaternion rotation

            # attitude control
            if not ((np.linalg.norm(quat_axis) <= 0.01) or (quat_angle >= 0.8*np.pi)):
                # update uuv's orientation to point towards waypoint
                quat = self.gen_quat_enu(Quaternion(axis=quat_axis, angle=2 * quat_angle)*self.orientation_quat)
            else:
                # uuv might be heading 180 deg. away from waypoint (problem for quaternion rotation)
                rotate = Quaternion(axis=quat_axis, angle=2 * (quat_angle+1.1*np.pi/2))*self.orientation_quat  # rotate uuv away from 180deg. problem
                self.set_attitude(euler_from_quaternion(np.array([rotate.x, rotate.y, rotate.z, rotate.w])))  # wait until orientation is fixed
                quat = self.orientation_quat  # keep updated orientation

            # thrust control
            if not np.linalg.norm(np.subtract(waypoint, self.position)) <= 0.5:
                # if uuv is far away, constant thrust/velocity
                thrust = 0.5
            else:
                if not np.linalg.norm(np.subtract(waypoint, self.position)) <= 0.1:
                    # slow down while approaching waypoint
                    thrust = np.linalg.norm(np.subtract(waypoint, self.position))  # distance is 0.5 or less -> thrust gets 0.5 or less
                else:
                    # stop the uuv as distance from waypoint is about 0.1
                    thrust = 0
                    self.is_done = True
                    print("Waypoint set")

            set_target.thrust = thrust
            set_target.orientation.x = quat.x
            set_target.orientation.y = quat.y
            set_target.orientation.z = quat.z
            set_target.orientation.w = quat.w
            # update attitude and thrust
            self._pub.publish(set_target)
            rate.sleep()
        self.is_done = False
        return True  # return makes main function wait for finished job

    def set_attitude(self, euler_angles):
        # expects numpy array euler_angles = [roll, pitch, yaw] in radiant
        self.is_done = False
        rate = rospy.Rate(25)

        # roll, pitch, yaw in quaternion notation
        q_roll = Quaternion(axis=[1, 0, 0], angle=euler_angles[0])  # roll
        q_pitch = Quaternion(axis=[0, 1, 0], angle=euler_angles[1])  # pitch
        q_yaw = Quaternion(axis=[0, 0, 1], angle=euler_angles[2])  # yaw

        quat = self.orientation_quat
        quat_euler = self.gen_quat_enu(q_yaw*q_pitch*q_roll)

        set_attitude = AttitudeTarget()  # message type to update pose/orientation

        while not self.is_done:
            if np.linalg.norm(np.subtract(euler_angles, np.array(self.orientation_euler))) <= 0.01:
                print("Attitude set!")
                self.is_done = True
            else:
                # rotate into desired attitude
                quat = quat_euler

            set_attitude.orientation.x = quat.x
            set_attitude.orientation.y = quat.y
            set_attitude.orientation.z = quat.z
            set_attitude.orientation.w = quat.w
            # update attitude
            self._pub.publish(set_attitude)
            rate.sleep()
        self.is_done = False
        return True # return makes main function wait for finished job

    # transform quaternions to NED(working) system
    def gen_quat_ned(self, quaternion):

        # ENU_body_fixed to NED
        # q_rot from ENU to NED
        # qx_180 from uuv_bodyframe to baselink
        quaternion = self.q_rot*quaternion*self.qx_180

        return quaternion

    # transform quaternions to ENU(ROS) system
    def gen_quat_enu(self, quaternion):
        # NED to ENU_body_fixed
        # q_rot.inverse from NED to ENU
        # qx_180.inverse from baselink to uuv_bodyframe
        quaternion = self.q_rot.inverse*(quaternion*self.qx_180.inverse)

        return quaternion


def main():
    rospy.init_node('attitude_set')
    uuv = Boat()
    waypoint = np.array([0.000, 0.000, 0.000])
    rpy = np.array([0.000, 0.000, 0.000])

    while not rospy.is_shutdown():
        print("    ")
        waypoint[0] = raw_input("What's your desired x-position? x= ")
        waypoint[1] = raw_input("What's your desired y-position? y= ")
        waypoint[2] = raw_input("What's your desired z-position? z= ")
        choose = raw_input("Do you want to set the attitude? [y/n]: ")
        if choose == "y":
            rpy[0] = raw_input("What's your desired roll? roll= ")
            rpy[1] = raw_input("What's your desired pitch? pitch= ")
            rpy[2] = raw_input("What's your desired yaw? yaw= ")
            uuv.set_waypoint(waypoint)
            uuv.set_attitude(rpy)
        else:
            uuv.set_waypoint(waypoint)


if __name__ == '__main__':
    main()