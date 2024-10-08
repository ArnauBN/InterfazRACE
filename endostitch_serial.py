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
    """Endostitch serial class. Handles the serial communication.
    
    Uses pyserial.
    """
    def __init__(self, state=0, port='/dev/ttyACM0', baudrate=9600, timeout=None):
        """EndostitchDevice Constructor.
        
        Sets the initial state (0 or 1), creates an instance of Serial without
        opening the port, sets the port, baudrate and timeout.
        """
        self._state = state
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.timeout = timeout
        self.prev = None
        
    def abrirPinza(self):
        """
        Sends a '1' via serial.

        Returns
        -------
        int
            Number of bytes written.
        
        """
        return self.ser.write(bytes('1'.encode('ascii')))
    
    def cerrarPinza(self):
        """
        Sends a '2' via serial.

        Returns
        -------
        int
            Number of bytes written.
        
        """
        return self.ser.write(bytes('2'.encode('ascii')))
    
    def moverMariposaDerecha(self):
        """
        Sends a '3' via serial.

        Returns
        -------
        int
            Number of bytes written.
        
        """
        return self.ser.write(bytes('3'.encode('ascii')))
    
    def moverMariposaCentro(self):
        """
        Sends a '4' via serial.

        Returns
        -------
        int
            Number of bytes written.
        
        """
        return self.ser.write(bytes('4'.encode('ascii')))
    
    def moverMariposaIzquierda(self):
        """
        Sends a '5' via serial.

        Returns
        -------
        int
            Number of bytes written.
        
        """
        return self.ser.write(bytes('5'.encode('ascii')))

    def close(self):
        """
        Closes the serial communication if it's open.

        Returns
        -------
        None.

        """
        try:
            if self.ser.isOpen():
                self.ser.close()
                print(f'Serial communication with {self.board} at port {self.port} closed successfully.')
            else:
                print(f'Serial communication with {self.board} at port {self.port} was already closed.')
        except Exception as e:
            print(e)
    
    def open(self):
        """
        Opens the serial communication if it's closed.

        Returns
        -------
        None.

        """
        if not self.ser.isOpen():
            self.ser.open()
    
    @property
    def state(self):
        """
        Getter method of property: state.

        Returns
        -------
        int
            State of the device (0=off, 1=on).

        """
        return self._state
    
    @state.setter
    def state(self, newState: int):
        """
        Setter method of property: state. If 0, the serial communication is
        closed. If 1, the serial communication is opened.

        Parameters
        ----------
        newState : int
            New desired state, can be 0 (off) or 1 (on).

        Returns
        -------
        None.

        """
        if newState==0:
            self.close()
        elif newState==1:
            self.open()
        else:
            print('Wrong state for Endostitch: available states are 0 (off) or 1 (on).')
            return
        self._state = newState
    
    def changeState(self):
        """
        Swaps the current state from 0 to 1 or from 1 to 0.

        Returns
        -------
        None.

        """
        self.state = self.state ^ 1 # XOR

    def callback(self, data):
        """
        Callback function for ROS communication. Receiving the same data twice
        is disallowed.

        Parameters
        ----------
        data : ROS data class instance
            ROS data. A different action is taken depending on data.data.

        Returns
        -------
        None.

        """
        if self.prev != data.data:
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            if data.data == "1":
                self.abrirPinza()
            elif data.data == "2":
                self.cerrarPinza()
            elif data.data == "3":
                self.moverMariposaDerecha()
            elif data.data == "4":
                self.moverMariposaCentro()
            elif data.data == "5":
                self.moverMariposaIzquierda()
            self.prev = data.data

if __name__ == '__main__':
    # Modifiable parameters
    port = '/dev/ttyACM0'
    baudrate = 9600
    timeout = None
    state = 0
    
    # Init ROS node
    rospy.init_node('endostitch_node', anonymous=True)
    endo =  EndostitchDevice(state=state, port=port, baudrate=baudrate, timeout=timeout)
    endo.open()
    
    # Init ROS subscriber
    rospy.loginfo("node init")
    rospy.Subscriber("endo_grip", String, endo.callback)
    rospy.spin()
