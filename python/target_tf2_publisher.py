#!/usr/bin/env python3  

'''
Handlind target messages and broadcast tf of target frame, works in spin mode

Subscriber:
    * /target_pose

Publisher:
    * /tf
'''

import rospy
import tf2_ros
import geometry_msgs.msg
from moveo_ps3.msg import TargetPose

def handle_target_state(msg):
    pub = tf2_ros.TransformBroadcaster()
    tf = geometry_msgs.msg.TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = msg.header.frame_id
    tf.child_frame_id  = '/target'
    rospy.logdebug(f'received - position: {msg.position}, quaternion: {msg.quaternion}')
    tf.transform.translation.x = msg.position[0]
    tf.transform.translation.y = msg.position[1]
    tf.transform.translation.z = msg.position[2]
    tf.transform.rotation.x = msg.quaternion[0]
    tf.transform.rotation.y = msg.quaternion[1]
    tf.transform.rotation.z = msg.quaternion[2]
    tf.transform.rotation.w = msg.quaternion[3]
    pub.sendTransform(tf)

if __name__ == '__main__':
    try:
        rospy.init_node('target_tf2_publisher', log_level=rospy.INFO)
        rospy.Subscriber('target_pose', TargetPose, handle_target_state, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException as err:
        rospy.logerr(str(err))