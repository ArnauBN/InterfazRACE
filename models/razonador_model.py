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
    print('razonador_model: Not using ROS')
    startROS = False


#%%
class RazonadorDevice:
    """ROS node for the razonador.
    
    Can be executed without ROS. It'll print a warning.
    Has a state attribute.
    Has a com.stateChanged signal.
    Has a com.faseChanged signal.
    Only starts ROS if startROS is True.
    """
    def __init__(self, state=0, startROS=True, experimentsList=None, experiment=None):
        """RazonadorDevice constructor.
        
        Calls super().__init__(). Starts ROS node if startROS is true. Creates
        an instance of Communicate, which adds a com.stateChanged signal and a
        com.faseChanged signal.

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
        self.experimentsList = experimentsList
        self._experiment = experiment
        self._experimentIndex = self.experimentsList.index(experiment) if experiment is not None else 0

    @property
    def experiment(self):
        """
        Getter method for property: experiment.

        Returns
        -------
        str
            Currently selected experiment.

        """
        return self._experiment

    @experiment.setter
    def experiment(self, newExpName):
        """
        Setter method for property: experiment.

        Parameters
        ----------
        newExpName : str
            New experiment name.

        Returns
        -------
        None.

        """
        self._experiment = newExpName
        self.experimentIndex = self.experimentsList.index(newExpName)

    @property
    def experimentIndex(self):
        """
        Getter method for property: experimentIndex.

        Returns
        -------
        int
            Index of currently selected experiment.

        """
        return self._experimentIndex

    @experimentIndex.setter
    def experimentIndex(self, newExpIdx):
        """
        Setter method for property: experimentIndex.

        Parameters
        ----------
        newExpIdx : int
            New experiment index.

        Returns
        -------
        None.

        """
        self._experiment = self.experimentsList[newExpIdx]
        self._experimentIndex = newExpIdx

    
    def _startROS(self):
        """
        Starts ROS node. Publisher and subscriber.

        Returns
        -------
        None.

        """
        self.pub1 = rospy.Publisher('cmd_key', UInt8MultiArray, queue_size=10)
        rospy.Subscriber("stateMachine", String, self.callback)

    def _savePublish(self, data: list):
        """
        Publishes the data on the razonador's ROS topic. Any exception is
        caught.

        Parameters
        ----------
        data : list
            Data to publish. Must be a list of 5 elements. Each element must be
            a 1 or a 0. See self.fase2code for available codes.

        Returns
        -------
        None.

        """
        try:
            self.pub.publish(None, data)
        except Exception as e:
            print("ROS Publish Exception in razonador:")
            print(e)

    def changeFase(self, newFase):
        """
        Publishes a new phase.

        Parameters
        ----------
        newFase : str or int
            Available phases range from 0 to 9.

        Returns
        -------
        None.

        """
        vector = self.fase2code(newFase)
        if self.state == 1:
            self._savePublish(vector)
        else:
            print(f"Razonador state is {self.state}. Cannot publish.")

    def fase2code(self, fase):
        """
        Parses the phase to the correct code (list of 1s or 0s). Each 
        experiment has a different set of codes. Returns None if the fase is
        not available.

        Parameters
        ----------
        fase : str | int
            Available phases range from 0 to 9.
        
        Returns
        -------
        list or None
            Equivalent 5bit code.

        """
        if self.experimentIndex > len(self.experimentsList)-1:
            return
        
        if fase == '0':
            return [1, 1, 1, 1, 1] # RESET
        
        if self.experimentIndex==0:
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
            else:
                return
        elif self.experimentIndex==1:
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
            else:
                return
    
    def callback(self, data):
        """
        Callback method for the ROS subscriber.
        Catches any exceptions.
        Emits the com.faseChanged signal.

        Parameters
        ----------
        data : ROS data class instance
            The current phase.

        Returns
        -------
        None.

        """
        try: 
            numero = re.search(r'\d+', data.data).group()
            nint = int(numero)
            self.com.faseChanged.emit(nint)
        except Exception as e:
            print("Exception in razonador:")
            print(e)

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
        
class Communicate(QObject):
    """Simple auxiliary class to handle custom signals for EndostitchDevice"""
    stateChanged = pyqtSignal(int)
    faseChanged = pyqtSignal(int)
