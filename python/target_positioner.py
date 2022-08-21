#!/usr/bin/env python3

import rospy
from tf_conversions import transformations as tf_trans
import numpy as np
from std_msgs.msg import Header
from moveo_ps3.msg import PS3State, TargetPose


class TargetPositioner:
    '''Class for calc target pose - position and orientation, works in spin mode

    Subscriber:
        * /ps3_state
    
    Publisher:
        * /target_pose
    '''

    def __init__(self) -> None:
        # meters
        self.position     = [0, 0, 0]
        # euler angles, degrees - for inner calc
        self.orientation  = [0, 0, 0]
        # quaternion for publication
        self.quaternion   = [0, 0, 0, 1]
        self.base_frame   = ''
        # msg counter
        self.out_msg_seq  = 0
        # start node
        self.run()

    def reset_pose(self, pos: bool=False, orient: bool=False) -> None:
        '''Reset target position or/and orientation'''
        if pos:    self.position    = [0., -0.22, 0.7956]
        if orient: self.orientation = [0., 0., 0.]
        
    def ps3_state_callback(self, msg) -> None:
        '''PS3 state messages handler'''    
        self.base_frame = msg.header.frame_id

        if any(msg.reset):
            self.reset_pose(msg.reset[0], msg.reset[1])
            return

        speed_pos, speed_orient = msg.speed
        self.position = list(map(lambda x, y: x + y*speed_pos, self.position, msg.change_position))
        
        # Rotation - multiplication of current and received
        euler_rot = list(map(lambda x: x*speed_orient, msg.change_orientation))
        
        # http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
        # Rotation matrices - https://www.youtube.com/watch?v=wg9bI8-Qx2Q
        # R_curr = tf_trans.euler_matrix(*np.radians(self.orientation))
        # R_rot = tf_trans.euler_matrix(*np.radians(euler_rot))
        # R_res =tf_trans.concatenate_matrices(R_curr, R_rot)
        # self.orientation = np.degrees(tf_trans.euler_from_matrix(R_res))
        
        # http://wiki.ros.org/tf2/Tutorials/Quaternions
        q_curr = tf_trans.quaternion_from_euler(*np.radians(self.orientation))
        q_rot = tf_trans.quaternion_from_euler(*np.radians(euler_rot))
        # spherical rotation, if multiply(q_rot, q_curr) - rotation about glogal frame axis
        q_res = tf_trans.quaternion_multiply(q_curr, q_rot)
        self.orientation = np.degrees(tf_trans.euler_from_quaternion(q_res))
        self.quaternion = q_res

        self.pub_target_pose()

    def pub_target_pose(self) -> None:
        '''Publishing to topic /target_pose'''
        pub = rospy.Publisher('target_pose', TargetPose, queue_size=1)
        msg = TargetPose()
        msg.header = Header()
        self.out_msg_seq += 1
        msg.header.seq      = self.out_msg_seq
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = self.base_frame
        msg.position        = self.position[:]
        msg.quaternion      = self.quaternion[:]
        pub.publish(msg)

    def run(self) -> None:
        '''Start running node'''
        # init pose
        self.reset_pose(pos=True, orient=True)
        rospy.init_node('target_positioner', log_level=rospy.INFO)
        rospy.Subscriber("ps3_state", PS3State, self.ps3_state_callback, queue_size=1)
        rospy.spin()
                

if __name__ == '__main__':
    try:
        target_positioner = TargetPositioner()
    except rospy.ROSInterruptException as err:
        rospy.logerr(str(err))