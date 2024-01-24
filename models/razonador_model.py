# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 13:16:46 2024

@author: arnau
"""
from .device_model import Device
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 13:15:05 2024

@author: arnau
"""

from PyQt5.QtCore import QObject, pyqtSignal
import re

startROS=True
try:
    import rospy
    from std_msgs.msg import UInt8MultiArray, String
except ModuleNotFoundError as e:
    print(e)
    print('Not using ROS')
    startROS = False


#%%
class RazonadorDevice:
    def __init__(self, state=0, startROS=True):
        super().__init__()
        self._state = state
        if startROS: self._startROS()
        self.com = Communicate()

    def _startROS(self):
        self.pub1 = rospy.Publisher('cmd_key', UInt8MultiArray, queue_size=10)
        rospy.Subscriber("stateMachine", String, self.callback)

    def _savePublish(self, data):
        try:
            self.pub.publish(None, data)
        except Exception as e:
            print("ROS Publish Exception in razonador:")
            print(e)

    def changeFase(self, newFase):
        vector = self.fase2code(newFase)
        self._savePublish(vector)
    
    def fase2code(self, fase):
        if str(fase)=='1':
            return [0, 0, 1, 0, 1]
        elif str(fase)=='2':
            return [0, 1, 1, 0, 1]
        elif str(fase)=='3':
            return [1, 1, 0, 0, 1]
        elif str(fase)=='4':
            return [1, 1, 0, 1, 1]
        elif str(fase)=='5':
            return [1, 1, 0, 1, 0]
        elif str(fase)=='6':
            return [1, 1, 0, 0, 0]
        elif str(fase)=='7':
            return [0, 1, 1, 0, 0]
        elif str(fase)=='8':
            return [0, 1, 1, 1, 0]
        elif str(fase)=='9':
            return [0, 0, 1, 1, 1]
        elif str(fase)=='0':
            return [1, 1, 1, 1, 1] # RESET
        else:
            return
    
    def callback(self, data):
        try: 
            numero = re.search(r'\d+', data.data).group()
            nint = int(numero)
            self.com.faseChanged.emit(nint)
        except Exception as e:
            print("Exception in razonador:")
            print(e)

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
        
class Communicate(QObject):
    """Simple auxiliary class to handle custom signals for EndostitchDevice"""
    stateChanged = pyqtSignal(int)
    faseChanged = pyqtSignal(int)
