# -*- coding: utf-8 -*-
"""
Created on Sun May  8 16:33:55 2022

@author: Jaime
"""


import sys

import os
import signal
import time

# pynput throws an error if we import it before $DISPLAY is set on LINUX
from pynput.keyboard import KeyCode

if sys.platform not in ('darwin', 'win32'):
    import os
    os.environ.setdefault('DISPLAY', ':0')
    
from win32gui import GetWindowText, GetForegroundWindow

from pynput import keyboard

import rclpy
from rclpy.parameter import Parameter
import std_msgs.msg

Title = "Robominer keyboard teleop"

os.system("title " + Title)

msg = """
Reading from the keyboard:

   q    w    e
   a    s    d
   z    x    c
   
---------------------------
Moving around:
        w     
   a    s    d
---------------------------
Rotate rightwards: e
Rotate leftwards:  q
----------------------------
Shrink robot:      z
Expand robot:      c
----------------------------
Drill:             x


CTRL-C or ESC to quit
"""

moveBindings = {
    'w':(1, 0),
    'a':(1, 0),
    's':(1, 0),
    'd':(1, 0),
    'q':(1, 0),
    'e':(1, 0),
    'z':(1, 0),
    'c':(-1, 0),
    'x':(1, 0)
}
        
class KeystrokeListen:
    def __init__(self, name=None):
        self.node = rclpy.create_node(name or type(self).__name__)
        self._publish_w = self.node.create_publisher(std_msgs.msg.Int8, 'forward', 10)
        self._publish_a = self.node.create_publisher(std_msgs.msg.Int8, 'left', 10)
        self._publish_s = self.node.create_publisher(std_msgs.msg.Int8, 'backwards', 10)
        self._publish_d = self.node.create_publisher(std_msgs.msg.Int8, 'right', 10)
        self._publish_q = self.node.create_publisher(std_msgs.msg.Int8, 'turnNeg', 10)
        self._publish_e = self.node.create_publisher(std_msgs.msg.Int8, 'turnPos', 10)
        self._publish_muscles = self.node.create_publisher(std_msgs.msg.Int8, 'muscles', 10)
        self._publish_x = self.node.create_publisher(std_msgs.msg.Int8, 'drill', 10) 
        # if self.exit_on_esc:
        #     self.logger.info('To end this node, press the escape key')

    def spin(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            while rclpy.ok() and listener.running:
                rclpy.spin_once(self.node, timeout_sec=0.1)

    @property
    def logger(self):
        return self.node.get_logger()


    def on_release(self, key):
        # todo: implement this
        #pass
        if GetWindowText(GetForegroundWindow()) == Title:
            try:
                char = getattr(key, 'char', None)
                if isinstance(char, str):                    
                    if char in moveBindings.keys():                        
                        msg = std_msgs.msg.Int8()
                        msg.data = moveBindings[char][1]
                        if char == ('w'):
                            self._publish_w.publish(msg)
                        if char == ('a'):
                            self._publish_a.publish(msg)
                        if char == ('s'):
                            self._publish_s.publish(msg)
                        if char == ('d'):
                            self._publish_d.publish(msg)
                        if char == ('q'):
                            self._publish_q.publish(msg)
                        if char == ('e'):
                            self._publish_e.publish(msg)
                        if char == ('z'):
                            self._publish_muscles.publish(msg)
                        if char == ('c'):
                            self._publish_muscles.publish(msg)                        
                        if char == ('x'):
                            self._publish_x.publish(msg)
            except Exception as e:
                self.logger.error(str(e))
                raise

    def on_press(self, key):
            
        #if all(win_name in GetWindowText(GetForegroundWindow()) for win_name in ["cmd.exe", __file__]):
        # for win_name in ["cmd.exe", __file__]:
        #     self.logger.info('window ' + win_name)
        #     self.logger.info('current' + GetWindowText(GetForegroundWindow()))
        if GetWindowText(GetForegroundWindow()) == Title:
            try:
                char = getattr(key, 'char', None)
                if isinstance(char, str):                    
                    if char in moveBindings.keys():                        
                        msg = std_msgs.msg.Int8()
                        msg.data = moveBindings[char][0]
                        if char == ('w'):
                            self._publish_w.publish(msg)
                        if char == ('a'):
                            self._publish_a.publish(msg)
                        if char == ('s'):
                            self._publish_s.publish(msg)
                        if char == ('d'):
                            self._publish_d.publish(msg)
                        if char == ('q'):
                            self._publish_q.publish(msg)
                        if char == ('e'):
                            self._publish_e.publish(msg)
                        if char == ('z'):
                            self._publish_muscles.publish(msg)
                        if char == ('c'):
                            self._publish_muscles.publish(msg)
                        if char == ('x'):
                            self._publish_x.publish(msg)
                    # self.logger.info('pressed ' + char)
                    # self.pub_glyph.publish(self.pub_glyph.msg_type(data=char))
                # else:
                    # try:
                        ##known keys like spacebar, ctrl
                        # name = key.name
                        # vk = key.value.vk
                    # except AttributeError:
                        ##unknown keys like headphones skip song button
                        # name = 'UNKNOWN'
                        # vk = key.vk
                    # self.logger.info('pressed {} ({})'.format(name, vk))
                    ## todo: These values are not cross-platform. When ROS2 supports Enums, use them instead
                    ## self.pub_code.publish(self.pub_code.msg_type(data=vk))
            except Exception as e:
                self.logger.error(str(e))
                raise
    
            if key == keyboard.Key.esc:
                self.logger.info('stopping listener')
                raise keyboard.Listener.StopException            
                os.kill(os.getpid(), signal.SIGINT)


def main(args=None):
    rclpy.init(args=args)
    print(msg)
    KeystrokeListen().spin()


if __name__ == '__main__':
    main()