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
    """ROS node for the endostitch.
    
    Can be executed without ROS. It'll print a warning.
    Has a state attribute.
    Has a com.stateChanged signal.
    Only starts ROS if startROS is True.
    """
    def __init__(self, state=0, startROS=True):
        """EndostitchDevice constructor.
        
        Calls super().__init__(). Starts ROS node if startROS is true. Creates
        an instance of Communicate, which adds a com.stateChanged signal.

        Parameters
        ----------
        state : int, optional
            State of the device (0=off, 1=on). The default is 0.
        startROS : bool, optional
            If true, start the ROS node. The default is True.

        Returns
        -------
        None.

        """
        super().__init__()
        self._state = state
        if startROS: self._startROS()
        self.com = Communicate()

    def _startROS(self):
        """
        Starts ROS node. Publisher and subscriber.

        Returns
        -------
        None.

        """
        self.pub = rospy.Publisher('endo_grip', String, queue_size=10)
        rospy.Subscriber("endo_grip", String, self.callback)

    def _savePublish(self, data):
        """
        Publishes the data on the endostitch's ROS topic. Any exception is
        caught.

        Parameters
        ----------
        data : str
            Data to publish ('0', '1', '2', '3', '4' or '5').

        Returns
        -------
        None.

        """
        try:
            self.pub.publish(data)
        except Exception as e:
            print("ROS Publish Exception in endostitch:")
            print(e)

    def abrirPinza(self):
        """
        Publishes a '1'.

        Returns
        -------
        None.

        """
        self._savePublish("1")
    
    def cerrarPinza(self):
        """
        Publishes a '2'.

        Returns
        -------
        None.

        """
        self._savePublish("2")
    
    def moverDerecha(self):
        """
        Publishes a '3'.

        Returns
        -------
        None.

        """
        self._savePublish("3")
    
    def moverCentro(self):
        """
        Publishes a '4'.

        Returns
        -------
        None.

        """
        self._savePublish("4")
    
    def moverIzquierda(self):
        """
        Publishes a '5'.

        Returns
        -------
        None.

        """
        self._savePublish("5")
     
    def callback(self, data):
        """
        Callback method for ROS subscriber. Changes the device's state to 0 if
        data is '0'. If not, the state is changed to 1.

        Parameters
        ----------
        data : ROS data class instance
            New device state.

        Returns
        -------
        None.

        """
        if data.data == "0" :
            self.state = 0
        else:
            self.state = 1

    @property
    def state(self):
        """
        Getter method for property: state.

        Returns
        -------
        int
            The current state (0=off, 1=on).

        """
        return self._state
    
    @state.setter
    def state(self, newState):
        """
        Setter method for property: state. Emits the com.stateChanged signal.

        Parameters
        ----------
        newState : int
            New desired state (0=off, 1=on).

        Returns
        -------
        None.

        """
        self._state = newState
        self.com.stateChanged.emit(newState)
    
    def changeState(self):
        """
        Changes the state from 0 to 1 or 1 to 0. Emits the com.stateChanged 
        signal.

        Returns
        -------
        None.

        """
        self._state = self._state ^ 1 # XOR
        self.com.stateChanged.emit(self._state)
        
        # The following line doesn't work. Seems like it doesn't emit the
        # signal or maybe it is emitted twice...
        # 
        # self.state = self.state ^ 1
        
class Communicate(QObject):
    """Simple auxiliary class to handle custom signals for EndostitchDevice"""
    stateChanged = pyqtSignal(int)