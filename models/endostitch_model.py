#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 13:15:05 2024

@author: arnau
"""

from PyQt5.QtCore import QObject, pyqtSignal

startROS=True
try:
    import rospy
    from std_msgs.msg import String
except ModuleNotFoundError as e:
    print(e)
    print('Not using ROS')
    startROS = False


#%%
class EndostitchDevice:
    def __init__(self, state=0, startROS=True):
        super().__init__()
        self._state = state
        if startROS: self._startROS()
        self.com = Communicate()

    def _startROS(self):
        self.pub = rospy.Publisher('endo_grip', String, queue_size=10)
        rospy.Subscriber("endo_grip", String, self.callback)

    def _savePublish(self, data):
        try:
            self.pub.publish(data)
        except Exception as e:
            print("ROS Publish Exception in endostitch:")
            print(e)

    def abrirPinza(self):
        return self._savePublish("1")
    
    def cerrarPinza(self):
        return self._savePublish("2")
    
    def moverDerecha(self):
        return self._savePublish("3")
    
    def moverCentro(self):
        return self._savePublish("4")
    
    def moverIzquierda(self):
        return self._savePublish("5")
     
    def callback(self, data):
        if data.data == "0" :
            self.state = 0
        else:
            self.state = 1

    @property
    def state(self):
        return self._state
    
    @state.setter
    def state(self, newState):
        self._state = newState
        self.com.stateChanged.emit(newState)
    
    def changeState(self):
        self._state = self._state ^ 1 # XOR
        self.com.stateChanged.emit(self._state)
        
        # This doesn't seem to emit the signal or maybe it is emitted twice...
        # self.state = self.state ^ 1
        
class Communicate(QObject):
    """Simple auxiliary class to handle custom signals for EndostitchDevice"""
    stateChanged = pyqtSignal(int)