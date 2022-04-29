import logging
import numpy as np
import cvxpy as cp
import rospy
import sys
import time
import tf
from geometry_msgs.msg import TransformStamped
from tcp_interface import TCPBridge
from controller import Controller
from robot import Robot

class HardwareWrap:
    def __init__(self, N):
        self.body_length = 0.05

        self.N = N
        self.ctl = Controller(N)
        self.bhav = self.ctl.go_to_goal
        self.state_id= 0 
        self.robot = None
        self.use_vicon = True

        rospy.init_node('robot_control', anonymous=True) 
        self.sub = [None] * N
        
        rospy.loginfo('Robot controller init')

    def init_subscribers(self, ips):
        # vicon callback on/off
        if self.use_vicon:
            for i in range(self.N):
                ip = ips[i]
                self.sub[i] = rospy.Subscriber('vicon/p%d/p%d' % (ip, ip), TransformStamped, self.vicon_cb, i)

    def vicon_cb(self, data, i):
        if self.robot is None:
            return

        pose = np.zeros(3)
        pose[0] = data.transform.translation.x
        pose[1] = data.transform.translation.y
        q =  data.transform.rotation
        quat = (q.x, q.y, q.z, q.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        pose[2] = euler[2]

        self.robot.prev_pose[:, i] = self.robot.pose[:, i]
        self.robot.pose[:, i] = pose

    def start(self):
        N = self.N
        tcp_com = TCPBridge(N)
        tcp_com.start_listen()
        self.init_subscribers(tcp_com.robot_ips)
        self.robot = Robot(N, tcp_com.robot_ips)
        robot = self.robot

        rate = rospy.Rate(10)

        state = 0
        pose_stable = False

        while not rospy.is_shutdown():
            rate.sleep()

            if self.use_vicon and (np.count_nonzero(robot.pose) < 3*N):
                continue

            if np.all((robot.prev_pose - robot.pose) < 1e-4):
                pose_stable = True
                print 'Pose is stable'

            if self.use_vicon and (not pose_stable): continue

            robot.dts = tcp_com.dts
            #  vel = robot.keyboard_input_each(pilot_ids=[])
            #  vel = robot.keyboard_input_lr()
            vel, state = robot.behav_sequence(state)
            print 'state: ', state

            if vel is None:
                continue
            
            for i in range(N):
                if len(tcp_com.data_qs[i]) < 1:
                    continue
                try:
                    robot.update_prev_cmd_vel(tcp_com.data_qs[i].pop(), i)
                except Exception as e:
                    print e

            tcp_com.send(vel)
            rospy.loginfo(robot.pose)

        tcp_com.end()

