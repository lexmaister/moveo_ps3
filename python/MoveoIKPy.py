#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from quaternion_convert import get_quaternion_from_euler
from euler import *
import ikpy.chain, ikpy.urdf
import numpy as np
from moveo_ps3.msg import MoveTarget


class MoveoIKPy:
    '''Class for calculating inverse kinematics of Moveo manipulator
    Subscriber:
        * /target_coords
    
    Publisher:
        * /joint_states
        * /geometry_msgs/PoseStamped
        * /visualization_marker
    '''

    def __init__(self, urdf) -> None:
        # x, y, z - meters
        self.target_position    = [0, 0, 0]
        self.position_error     = 0.0
        self.speed_position     = 0.0
        # r, p, y - degrees     
        self.target_orientation = [0, 0, 0]
        self.orientation_error  = 0.0
        self.speed_orientation  = 0.0
        self.reset([True, False])
        self.joint_angles       = [0, 0, 0, 0, 0]
        self.active             = True
        self.calc_orientation   = True
        self.gripper            = False
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf, base_elements = ['base_link'])
        # full 7 chain links (aka urdf joints)
        self.chain_links = []
        # track ps3 messages counter 
        self.ps3_msg_seq        = 0

    def reset(self, rst: list) -> None:
        '''Reset target position or/and orientation'''
        if rst[0]: self.target_position = [0., 0., 0.756]
        if rst[1]: self.target_orientation = [0., 0., 0.]
        
    def run(self) -> None:
        '''Start running node "MoveoIKPy"'''
        # http://wiki.ros.org/rospy/Overview/Logging
        rospy.init_node('MoveoIKPy', log_level=rospy.INFO)
        rate = rospy.Rate(10) #10 Hz
        while not rospy.is_shutdown():
            rate.sleep()
            #subscribe
            rospy.Subscriber('target_coords', MoveTarget, self.move_target_callback, queue_size=1)
            # calc
            if self.active:
                self.calc_IK()
                self.calc_pos_error()
            # publish
            self.pub_target_axis()
            self.pub_target_marker()
            self.pub_joint_states()
            
    def move_target_callback(self, msg):
        '''Get move target parameters from /target_coords'''
        if msg.header.seq == self.ps3_msg_seq: return
        else: self.ps3_msg_seq = msg.header.seq
        rospy.logdebug(f'joy_message_seq: {self.ps3_msg_seq}') 

        rospy.logdebug(f'Received TargetCoordinates Message:\n{msg}')
        self.active           = msg.active
        self.calc_orientation = msg.calc_orient 
        self.gripper          = msg.gripper 
        self.speed_position, self.speed_orientation = msg.speed
        self.target_position    = list(map(lambda x, y: x + y*self.speed_position, self.target_position, msg.position))
        self.target_orientation = list(map(lambda x, y: x + y*self.speed_orientation, self.target_orientation, msg.orientation))
        self.reset(msg.reset)

    def calc_IK(self) -> None:
        '''Inverse kinematic calculation'''
        # two step calc for more sustainable result: https://github.com/Phylliade/ikpy/wiki/Orientation
        self.chain_links = self.chain.inverse_kinematics(self.target_position)
        r , p, y = self.target_orientation
        orientation_matrix = rotation_matrix(r, p, y, order='xyz')
        if self.calc_orientation:
            self.chain_links = self.chain.inverse_kinematics(self.target_position, orientation_matrix, orientation_mode='all')
        # exclude base_link and end_effector fixed joints and round for messaging
        self.joint_angles = np.round(self.chain_links[1:-1],3)

    def calc_pos_error(self) -> None:
        '''Calculation the difference between target and reached position'''
        # pos error
        actual_position = self.chain.forward_kinematics(self.chain_links)[:3, 3]
        # https://stackoverflow.com/questions/1401712/how-can-the-euclidean-distance-be-calculated-with-numpy
        self.position_error = np.linalg.norm(self.target_position-actual_position)
        rospy.loginfo(f'target_position: {np.round(self.target_position,3)}')
        rospy.logdebug(f'actual_position: {np.round(actual_position,3)}')
        rospy.logdebug(f'position_error: {np.round(self.position_error,4)}')
        
        # orient error
        actual_orientation = rotation_angles(self.chain.forward_kinematics(self.chain_links)[:3, :-1], order='xyz')
        actual_quaternion = get_quaternion_from_euler(actual_orientation[0], actual_orientation[1], actual_orientation[2])
        # calc orientation error as summ of errors on each axis
        error_arr = np.subtract(self.target_orientation, actual_orientation)
        self.orientation_error = np.sum(np.abs(error_arr))
        rospy.loginfo(f'target_orientation: {np.round(self.target_orientation,3)}')
        rospy.loginfo(f'actual_orientation - euler: {np.round(actual_orientation, 3)}')
        rospy.logdebug(f'actual_orientation - quaternion: {np.round(actual_quaternion, 3)}')
        rospy.loginfo(f'orientation_error: {np.round(self.orientation_error,3)}')

    def pub_joint_states(self) -> None:
        '''Publishing joints angles to topic "joint_states"''' 
        pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/base_link'
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        msg.position = self.joint_angles
        msg.velocity = []
        msg.effort   = []
        pub.publish(msg)
        rospy.logdebug(f'JointState Message:\n{msg}')

    def pub_target_axis(self) -> None:
        '''Publishing target pose to topic "geometry_msgs/PoseStamped"''' 
        pub = rospy.Publisher("geometry_msgs/PoseStamped", PoseStamped, queue_size = 10)
        msg = PoseStamped()
        msg.header.frame_id = "/base_link"
        msg.header.stamp = rospy.Time.now()
        # Set the position of the target
        msg.pose.position.x = self.target_position[0]
        msg.pose.position.y = self.target_position[1]
        msg.pose.position.z = self.target_position[2]
        # Set the orientation of the target as quaternion
        q = get_quaternion_from_euler(  self.target_orientation[0], 
                                        self.target_orientation[1], 
                                        self.target_orientation[2])
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        pub.publish(msg)
        rospy.logdebug(f'PoseStamped Message:\n{msg}')

    def pub_target_marker(self) -> None:
        '''Publishing target marker to topic "visualization_marker"''' 
        pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 10)
        msg = Marker()
        msg.header.frame_id = "/base_link"
        msg.header.stamp = rospy.Time.now()
        msg.type = msg.CUBE if self.gripper else msg.SPHERE
        msg.id = 0
        # Set the position of the target
        msg.pose.position.x = self.target_position[0]
        msg.pose.position.y = self.target_position[1]
        msg.pose.position.z = self.target_position[2]
        q = get_quaternion_from_euler(  self.target_orientation[0], 
                                        self.target_orientation[1], 
                                        self.target_orientation[2])
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        # Set the scale of the marker
        msg.scale.x = 0.05
        msg.scale.y = 0.05
        msg.scale.z = 0.05
        # Set the color depending on error
        max_pos_error    = 0.005 # 5 mm
        max_orient_error = 0.1   # 0.1 degree
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 0.7
        if not self.active: 
            msg.color.r = 1.0
            msg.color.g = 1.0
            msg.color.b = 1.0
        elif self.position_error <= max_pos_error and self.orientation_error <= max_orient_error:
            msg.color.g = 1.0
        elif self.position_error <= max_pos_error and not self.calc_orientation:
            msg.color.b = 1.0
        elif self.position_error <= max_pos_error and self.calc_orientation > max_orient_error:
            msg.color.r = 1.0
            msg.color.g = 0.65
        else:
            msg.color.r = 1.0
        pub.publish(msg)
        rospy.logdebug(f'Marker Message:\n{msg}')    


if __name__ == '__main__':
    # get path to urdf from launch file
    # https://answers.ros.org/question/384669/use-arguments-from-roslaunch-in-python/?answer=384696
    urdf_file = rospy.get_param('/MoveoIKPy/model') 
    try:
        moveo_ik_pub = MoveoIKPy(urdf_file)
        moveo_ik_pub.run()
    except rospy.ROSInterruptException:
        pass
    