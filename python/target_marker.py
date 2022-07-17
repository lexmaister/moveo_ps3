#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from moveo_ps3.msg import PS3State
from moveo_ikpy import log_show_transform
import numpy as np
import tf_conversions
import tf2_ros


class TargetMarker:
    '''
    Class for publishing target visualisation marker depending transform error /target vs /end_effector, 
    works in spin mode

    Subscriber:
        * /tf
        * /ps3_state

    Publisher:
        * /visualization_marker
    '''
    def __init__(self) -> None:
        self.pos_error          = 0
        self.orient_error       = 0
        self.max_pos_error      = 0.005 # 5 mm
        self.max_orient_error   = 0.5   # 0.5 degree
        self.calc_active        = True
        self.calc_orient        = True
        self.base_frame         = ''
        self.gripper            = False
        # start node
        self.tf_buffer          = tf2_ros.Buffer()
        self.run()

    def run(self):
        '''Start running node'''
        rospy.init_node('target_marker', log_level=rospy.INFO)
        tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber('ps3_state', PS3State, self.handle_ps3_state, queue_size=1)
        rospy.spin()

    def handle_ps3_state(self, msg):
        '''Get calc and gripper states of ps3 controller'''
        self.base_frame  = msg.header.frame_id
        self.calc_active = msg.calc_active
        self.calc_orient = msg.calc_orient 
        self.gripper     = msg.gripper
        rospy.logdebug(f'calc active, {self.calc_active}, orient: {self.calc_orient}, gripper: {self.gripper}')

        # publish in callback because of spin mode
        self.calc_pos_error()
        self.pub_marker()

    def pub_marker(self):
        '''visualization marker publiser, based on transform /target vs /world'''
        pub = rospy.Publisher("visualization_marker", Marker, queue_size = 1)
        msg = Marker()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = rospy.Time.now()
        msg.id = 0
        try:
            target_tf = self.tf_buffer.lookup_transform(self.base_frame, 'target', rospy.Time())
            log_show_transform(target_tf, self.base_frame, 'target', axs='rxyz', log_lvl=rospy.DEBUG)
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException,
                tf2_ros.InvalidArgumentException) as err:
            rospy.logerr(f'{err}')
        # Set the target marker parameters
        msg.type = msg.CUBE if self.gripper else msg.SPHERE
        msg.pose.position = target_tf.transform.translation
        msg.pose.orientation = target_tf.transform.rotation
        msg.scale.x = 0.05
        msg.scale.y = 0.05
        msg.scale.z = 0.05
        # color depends on error
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 0.7
        if not self.calc_active: 
            msg.color.r = 1.0
            msg.color.g = 1.0
            msg.color.b = 1.0
        elif self.pos_error <= self.max_pos_error and self.orient_error <= self.max_orient_error:
            msg.color.g = 1.0
        elif self.pos_error <= self.max_pos_error and not self.calc_orient:
            msg.color.b = 1.0
        elif self.pos_error <= self.max_pos_error and self.calc_orient > self.max_orient_error:
            msg.color.r = 1.0
            msg.color.g = 0.65
        else:
            msg.color.r = 1.0
        pub.publish(msg)

    def calc_pos_error(self):
        '''Calculate position and orientation error from transform /target vs /end_effector'''
        try:
            target_tf = self.tf_buffer.lookup_transform('end_effector', 'target', rospy.Time())
            log_show_transform(target_tf, 'end_effector', 'target', axs='rxyz', log_lvl=rospy.INFO)
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException,
                tf2_ros.InvalidArgumentException) as err:
            rospy.logerr(f'{err}')
        self.pos_error = np.linalg.norm((target_tf.transform.translation.x,
                                         target_tf.transform.translation.y,
                                         target_tf.transform.translation.z))
        # https://math.stackexchange.com/questions/59629/the-euclidean-norm-r-of-a-rotation
        target_end_effector_rotation = (target_tf.transform.rotation.x,
                                        target_tf.transform.rotation.y,
                                        target_tf.transform.rotation.z,
                                        target_tf.transform.rotation.w)
        euler = tf_conversions.transformations.euler_from_quaternion(target_end_effector_rotation, axes='rxyz')
        self.orient_error = np.linalg.norm(np.degrees(euler))
        rospy.loginfo(f'pos_error: {np.round(self.pos_error, 3)}, orient_error: {np.round(self.orient_error, 1)}')


if __name__ == '__main__':
    try:
        TargetMarker()
    except rospy.ROSInterruptException as err:
        rospy.logerr(str(err))
    