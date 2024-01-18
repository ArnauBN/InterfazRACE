#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 13:15:05 2024

@author: arnau
"""

import rospy
from std_msgs.msg import String
import serial

    
#%%
class EndostitchDevice:
    def __init__(self, state=0, port='COM3', baudrate=9600, timeout=None):
        self._state = state
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.timeout = timeout
        
    def abrirPinza(self):
        return self.ser.write(bytes('1'))
    
    def cerrarPinza(self):
        return self.ser.write(bytes('2'))
    
    def moverMariposaDerecha(self):
        return self.ser.write(bytes('3'))
    
    def moverMariposaCentro(self):
        return self.ser.write(bytes('4'))
    
    def moverMariposaIzquierda(self):
        return self.ser.write(bytes('5'))

    def close(self):
        try:
            if self.ser.isOpen():
                self.ser.close()
                print(f'Serial communication with {self.board} at port {self.port} closed successfully.')
            else:
                print(f'Serial communication with {self.board} at port {self.port} was already closed.')
        except Exception as e:
            print(e)
    
    def open(self):
        if not self.ser.isOpen():
            self.ser.open()
    
    @property
    def state(self):
        return self._state
    
    @state.setter
    def state(self, newState):
        if newState==0:
            self.close()
        elif newState==1:
            self.open()
        else:
            print('Wrong state for Endostitch: available states are 0 (off) or 1 (on).')
            return
        self._state = newState
    
    def changeState(self):
        self.state = self.state ^ 1 # XOR

def callback(self, data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


if __name__ == '__main__':
    rospy.init_node('endostitch_node', anonymous=True)
    endo =  EndostitchDevice(state=0, port='COM3', baudrate=9600, timeout=None)
    endo.open()
    
    rospy.loginfo("node init")
    rospy.Subscriber("endo_grip",String,callback)
    rospy.spin()
