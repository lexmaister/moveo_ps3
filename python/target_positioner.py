#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from moveo_ps3.msg import PS3State, TargetPose


class TargetPositioner:
    '''Class for calc target pose - position and orientation, works in loop mode

    Subscriber:
        * /ps3_state
    
    Publisher:
        * /target_pose
    '''

    def __init__(self) -> None:
        # meters
        self.position     = [0, 0, 0]
        # euler angles, degrees
        self.orientation  = [0, 0, 0]
        self.base_frame   = ''
        # msg counters
        self.in_msg_seq   = 0
        self.out_msg_seq  = 0
        # start node
        self.run()

    def reset_pose(self, pos: bool=False, orient: bool=False) -> None:
        '''Reset target position or/and orientation'''
        if pos:    self.position    = [0., -0.22, 0.7956]
        if orient: self.orientation = [0., 0., 0.]
        
    def ps3_state_callback(self, msg) -> None:
        '''PS3 state messages handler'''
        if msg.header.seq == self.in_msg_seq: return
        else: self.in_msg_seq = msg.header.seq
        rospy.logdebug(f'ps3_state_msg_seq: {self.in_msg_seq}') 
        
        self.base_frame = msg.header.frame_id

        if any(msg.reset):
            self.reset_pose(msg.reset[0], msg.reset[1])
            return

        speed_pos, speed_orient = msg.speed
        self.position    = list(map(lambda x, y: x + y*speed_pos,    self.position,    msg.change_position))
        self.orientation = list(map(lambda x, y: x + y*speed_orient, self.orientation, msg.change_orientation))

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
        msg.orientation     = self.orientation[:]
        pub.publish(msg)

    def run(self) -> None:
        '''Start running node'''
        # init pose
        self.reset_pose(pos=True, orient=True)
        rospy.init_node('target_positioner', log_level=rospy.INFO)
        rate = rospy.Rate(10) #10 Hz
        while not rospy.is_shutdown():
            rate.sleep()
            #subscribe
            rospy.Subscriber("ps3_state", PS3State, self.ps3_state_callback, queue_size=1)
            # publish
            self.pub_target_pose()
                

if __name__ == '__main__':
    try:
        target_positioner = TargetPositioner()
    except rospy.ROSInterruptException as err:
        rospy.logerr(str(err))