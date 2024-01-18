#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 13:15:05 2024

@author: arnau
"""

import rospy
from std_msgs.msg import String
from PyQt5.QtCore import QObject, pyqtSignal


#%%
class EndostitchDevice:
    def __init__(self, state=0):
        super().__init__()
        self._state = state

        self.pub = rospy.Publisher('endo_grip',String,queue_size=10)
        rospy.Subscriber("endo_grip",String,self.callback)
        
        self.com = Communicate()

    def abrirPinza(self):
        return self.pub.publish("1")
    
    def cerrarPinza(self):
        return self.pub.publish("2")
    
    def moverDerecha(self):
        return self.pub.publish("3")
    
    def moverCentro(self):
        return self.pub.publish("4")
    
    def moverIzquierda(self):
        return self.pub.publish("5")
     
    def callback(self,data):
        if data.data == "0" :
            self.state = 0
            #rospy.loginfo("state: 0")
        else:
            self.state = 1
            #rospy.loginfo("state: 1")

    @property
    def state(self):
        return self._state
    
    @state.setter
    def state(self, newState):
        #if newState==0:
        #    if self.ser is not None: self.close()
        #elif newState==1:
        #    if self.ser is not None:
        #        self.open()
        #    else:
        #        print('Serial communication not initiated.')
        #        return
        #else:
        #    print('Wrong state for Endostitch: available states are 0 (off) or 1 (on).')
        #    return
        self._state = newState
        self.com.stateChanged.emit(newState)
    
    def changeState(self):
        self.state = self.state ^ 1 # XOR

class Communicate(QObject):
    """Simple auxiliary class to handle custom signals for EndostitchDevice"""
    stateChanged = pyqtSignal(int)