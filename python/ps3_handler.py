#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from moveo_ps3.msg import PS3State


class PS3Handler:
    '''Class for handling joystick messages, works in loop mode

    Subscriber:
        * /joy
    
    Publisher:
        * /ps3_state
    '''

    def __init__(self) -> None:
        # meters
        self.change_position    = [0, 0, 0]
        # euler angles, degrees
        self.change_orientation = [0, 0, 0]
        # true - pos, false - orient 
        self.switch_pos_orient  = True 
        # speed / reset: 0 - pos, 1 - orient
        self.speed              = [0, 0]
        self.reset              = [False, False]
        self.calc_active        = True
        self.calc_orient        = True
        # manipulator ctrl
        self.move               = False
        self.gripper            = False
        # msg counters
        self.in_msg_seq         = 0
        self.out_msg_seq        = 0
        # start node
        self.run()

    def change_speed(self, increase: True) -> None:
        '''Changing speed for position and orientation'''
        speed_max  = (0.03, 8)
        speed_min  = (0.001, 0.5)
        speed_step = (0.001, 0.2)
        curr = 0 if self.switch_pos_orient else 1
        if not all(self.speed):
            self.speed = list(speed_min)
        elif increase and self.speed[curr] < speed_max[curr]:
            self.speed[curr] += speed_step[curr]
        elif not increase and self.speed[curr] > speed_min[curr]:
            self.speed[curr] -= speed_step[curr]            
        rospy.logdebug(f'speed_pos: {self.speed[0]}, speed_orient: {self.speed[1]}')
        
    def joy_callback(self, msg) -> None:
        '''Joy messages handler'''
        if msg.header.seq == self.in_msg_seq: return
        else: self.in_msg_seq = msg.header.seq
        rospy.logdebug(f'joy_msg_seq: {self.in_msg_seq}')

        rospy.logdebug(f'joy_axes: {msg.axes}\n joy_buttons: {msg.buttons}')

        if msg.buttons[0] : self.calc_orient        = not self.calc_orient
        if msg.buttons[1] : self.calc_active        = not self.calc_active        
        if msg.buttons[2] : self.switch_pos_orient  = not self.switch_pos_orient
        if msg.buttons[3] : self.gripper            = not self.gripper        
        if msg.buttons[9] : self.move               = not self.move        
        if msg.buttons[11]: self.reset[0]           = not self.reset[0]     
        if msg.buttons[12]: self.reset[1]           = not self.reset[1]
        if msg.buttons[13]: self.change_speed(increase=True)
        if msg.buttons[14]: self.change_speed(increase=False)

        x = msg.axes[3]
        y = msg.axes[4]
        z = msg.axes[1]
        input_coords = (x, y, z)

        if self.switch_pos_orient:
            self.change_position    = input_coords
        else:
            self.change_orientation = input_coords

    def pub_target_state(self) -> None:
        '''Publishing to topic /ps3_state'''
        pub = rospy.Publisher('ps3_state', PS3State, queue_size=1)
        msg = PS3State()
        msg.header = Header()
        self.out_msg_seq += 1
        msg.header.seq      = self.out_msg_seq
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = 'world'
        msg.change_position    = self.change_position[:]
        msg.change_orientation = self.change_orientation[:]
        msg.speed           = self.speed[:]
        msg.reset           = self.reset[:]
        msg.calc_active     = self.calc_active
        msg.calc_orient     = self.calc_orient
        msg.move            = self.move
        msg.gripper         = self.gripper
        pub.publish(msg)
        # flush reset - must be sending only once
        self.reset = [False, False]

    def run(self) -> None:
        '''Start running node'''
        # set minimum speed
        self.change_speed(increase=True)
        rospy.init_node('ps3_handler', log_level=rospy.INFO)
        publish_rate = rospy.get_param('ps3_state_publish_rate', 10.0) #10 Hz by default
        rate = rospy.Rate(publish_rate) 
        while not rospy.is_shutdown():
            rate.sleep()
            #subscribe
            rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
            # publish
            self.pub_target_state()


if __name__ == '__main__':
    try:
        PS3Handler()
    except rospy.ROSInterruptException as err:
        rospy.logerr(str(err))