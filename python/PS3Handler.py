#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from moveo_ps3.msg import MoveTarget


class PS3Handler:
    '''Class for handlind joystic messages
    Subscriber:
        * /joy
    
    Publisher:
        * /target_coords
    '''

    def __init__(self) -> None:
        # range 0 - 1
        self.position           = [0, 0, 0]
        self.orientation        = [0, 0, 0]
        # speed: 0 - pos, 1 - orient
        self.speed              = [0, 0]
        # reset: 0 - pos, 1 - orient
        self.reset              = [False, False]
        self.active             = True
        self.calc_orient        = True
        self.gripper            = False
        # True - pos, False - orient 
        self.switch_pos_orient  = True
        # track joy messages counter 
        self.joy_msg_seq        = 0
        self.out_msg_seq        = 0
        # set minimum speed
        self.change_speed(True)

    def change_speed(self, increase: True) -> None:
        '''Changing speed for position and orientation'''
        speed_max  = (0.02, 10)
        speed_min  = (0.001, 0.5)
        speed_step = (0.001, 0.2)
        curr = 0 if self.switch_pos_orient else 1
        if not all(self.speed):
            self.speed = list(speed_min)
        elif increase and self.speed[curr] < speed_max[curr]:
            self.speed[curr] += speed_step[curr]
        elif not increase and self.speed[curr] > speed_min[curr]:
            self.speed[curr] -= speed_step[curr]            
        rospy.logdebug('speed_pos:' if not curr else 'speed_orient:' + f'{self.speed[curr]}')
        
    def joy_callback(self, msg) -> None:
        '''Joy messages handler'''
        if msg.header.seq == self.joy_msg_seq: return
        else: self.joy_msg_seq = msg.header.seq
        rospy.logdebug(f'joy_message_seq: {self.joy_msg_seq}')

        rospy.logdebug(f'joy_axes: {msg.axes}\n joy_buttons: {msg.buttons}')

        if msg.buttons[0] : self.calc_orient = not self.calc_orient
        if msg.buttons[1] : self.active = not self.active        
        if msg.buttons[2] : self.switch_pos_orient = not self.switch_pos_orient
        if msg.buttons[3] : self.gripper = not self.gripper        
        if msg.buttons[11]: self.reset[0] = not self.reset[0]        
        if msg.buttons[12]: self.reset[1] = not self.reset[1]        
        if msg.buttons[13]: self.change_speed(increase=True)
        if msg.buttons[14]: self.change_speed(increase=False)

        x = msg.axes[3]
        y = msg.axes[4]
        z = msg.axes[1]
        v = (x, y, z)

        if self.switch_pos_orient:
            self.position = v
        else:
            self.orientation = v

    def run(self) -> None:
        '''Start running node'''
        rospy.init_node('PS3Handler', log_level=rospy.INFO)
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            rate.sleep()
            #subscribe
            rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
            # publish
            self.pub_target_coords()
                
    def pub_target_coords(self) -> None:
        '''Publishing to topic /target_coords'''
        pub = rospy.Publisher('target_coords', MoveTarget, queue_size=1)
        msg = MoveTarget()
        msg.header = Header()
        msg.header.seq      = self.out_msg_seq  
        self.out_msg_seq    += 1
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = '/base_link'
        msg.position        = self.position[:]
        msg.orientation     = self.orientation[:]
        msg.speed           = self.speed[:]
        msg.reset           = self.reset[:]
        msg.active          = self.active
        msg.calc_orient     = self.calc_orient
        msg.gripper         = self.gripper
        pub.publish(msg)
        rospy.logdebug(f'MoveTarget Message:\n{msg}')
        if any(self.reset): self.reset = [False, False]

if __name__ == '__main__':
    try:
        ps3_handler = PS3Handler()
        ps3_handler.run()
    except rospy.ROSInterruptException:
        pass