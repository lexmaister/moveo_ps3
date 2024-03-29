#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import ikpy.chain, ikpy.urdf
import numpy as np
from moveo_ps3.msg import PS3State
import tf_conversions
import tf2_ros


def log_show_transform(tf, base_frame: str, moving_frame: str, axs:str = 'sxyz', log_lvl:int = rospy.INFO):
    '''Show transform data in /rosout'''
    target_position = ( tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z)
    target_rotation = ( tf.transform.rotation.x,
                        tf.transform.rotation.y,
                        tf.transform.rotation.z,
                        tf.transform.rotation.w)
    euler = tf_conversions.transformations.euler_from_quaternion(target_rotation, axes=axs)
    tf_str = f'tf /{base_frame} /{moving_frame}:'\
             f'\ntranslation, m: {np.round(target_position, 3)}'\
             f'\nrotation - euler({axs}), deg): {np.round(np.degrees(euler), 1)}'
    if log_lvl == rospy.INFO:
        # rospy.loginfo(tf_str)
        pass
    elif log_lvl == rospy.DEBUG:
        rospy.logdebug(tf_str)


class MoveoIKPy:
    '''Class for calculating inverse kinematics of Moveo manipulator with IKPy, works in loop mode

    Subscriber:
        * /ps3_state
        * /tf
    
    Publisher:
        * /joint_states
    '''

    def __init__(self, urdf) -> None:
        # target pose from tf
        self.target_position    = (0, 0, 0)   
        self.target_rotation    = (0, 0, 0, 0) # quaternion
        self.joint_angles       = (0, 0, 0, 0, 0)
        self.calc_active        = True
        self.calc_orient        = True
        self.base_frame         = ''
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf, base_elements = ['world'])
        # full 8 chain links (aka urdf joints)
        self.chain_links        = [0] * 8
        # track ps3 messages counter 
        self.ps3_msg_seq        = 0
        # start node
        self.run()
        
    def run(self) -> None:
        '''Start running node "moveo_ikpy"'''
        rospy.init_node('moveo_ikpy', log_level=rospy.INFO)

        tf_buffer   = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)

        publish_rate = rospy.get_param('joint_states_publish_rate', 10.0) #10 Hz by default
        rate = rospy.Rate(publish_rate) 
        while not rospy.is_shutdown():
            #subscribe
            rospy.Subscriber('ps3_state', PS3State, self.ps3_state_callback, queue_size=1)
            try:
                target_tf = tf_buffer.lookup_transform(self.base_frame, 'target', rospy.Time())
                self.target_position = (target_tf.transform.translation.x,
                                        target_tf.transform.translation.y,
                                        target_tf.transform.translation.z)
                self.target_rotation = (target_tf.transform.rotation.x,
                                        target_tf.transform.rotation.y,
                                        target_tf.transform.rotation.z,
                                        target_tf.transform.rotation.w)
                log_show_transform(target_tf, self.base_frame, 'target', axs='rxyz', log_lvl=rospy.DEBUG)
            except (tf2_ros.LookupException, 
                    tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException,
                    tf2_ros.InvalidArgumentException) as err:
                rospy.logerr(f'{err}')
                rate.sleep()
                continue
            # calc
            if self.calc_active:
                self.calc_IK()
            # publish
            self.pub_joint_states()
            rate.sleep()
            
    def ps3_state_callback(self, msg):
        '''Get calc parameters from /ps3_state'''
        if msg.header.seq == self.ps3_msg_seq: return
        else: self.ps3_msg_seq = msg.header.seq
        rospy.logdebug(f'ps3_state_msg_seq: {self.ps3_msg_seq}')         
        self.base_frame  = msg.header.frame_id
        self.calc_active = msg.calc_active
        self.calc_orient = msg.calc_orient 

    def calc_IK(self) -> None:
        '''Inverse kinematic calculation'''
        init_pos = self.chain_links
        if self.calc_orient:
            orientation_matrix = tf_conversions.transformations.quaternion_matrix(self.target_rotation)
            self.chain_links = self.chain.inverse_kinematics(self.target_position, orientation_matrix[:-1, :-1], 
                                                             orientation_mode='all', initial_position=init_pos)
        else:
            self.chain_links = self.chain.inverse_kinematics(self.target_position, initial_position=init_pos)

        # exclude world, base_link and end_effector fixed joints and round for messaging
        self.joint_angles = np.round(self.chain_links[2:-1],4)

    def pub_joint_states(self) -> None:
        '''Publishing joints angles to topic "/joint_states"''' 
        pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.base_frame
        msg.name = [f'joint_{i}' for i in range(1,6)]
        msg.position = self.joint_angles
        msg.velocity = []
        msg.effort   = []
        pub.publish(msg)


if __name__ == '__main__':
    # get path to urdf from launch file
    # https://answers.ros.org/question/384669/use-arguments-from-roslaunch-in-python/?answer=384696
    urdf_file = rospy.get_param('/moveo_ikpy/model') 
    try:
        MoveoIKPy(urdf_file)
    except rospy.ROSInterruptException as err:
        rospy.logerr(str(err))
    