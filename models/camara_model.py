# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 13:12:03 2024

@author: arnau
"""
import sys

from utils.globals import PATH_TO_INTERFAZ_CLIENT_CAMERA
sys.path.append(PATH_TO_INTERFAZ_CLIENT_CAMERA)

startROS=True
try:
    from interfaz_client_camera import interface_client
except ModuleNotFoundError as e:
    print(e)
    print('camara_model: Not using ROS')
    startROS = False


#%%
class CamaraDevice:
    """Handles the camera state."""
    def __init__(self, state=0):
        """CamaraDevice constructor.
        
        Sets the initial state.

        Parameters
        ----------
        state : int, optional
            State of the device (0=off, 1=on, 2=waiting, -1=error/interrupted).
            The default is 0.

        Returns
        -------
        None.

        """
        self._state = state    
    
    @property
    def state(self):
        """
        Getter method for property: state.

        Returns
        -------
        int
            The current state (0=off, 1=on, 2=waiting, -1=error/interrupted).

        """
        return self._state
    
    @state.setter
    def state(self, newState):
        """
        Setter method for property: state.

        Parameters
        ----------
        newState : int
            New desired state (0=off, 1=on, 2=wait, -1=error/interrupt).

        Returns
        -------
        None.

        """
        self._state = newState
    
    def changeState(self):
        """
        Changes the state to 0 (off) or 1 (on). If the current state is 0 or 2,
        the state is changed to 1. If the current state is 1 or -1, the state
        is changed to 0.

        Returns
        -------
        None.

        """
        if self.state in [0,2]: # off/waiting -> turn on
            aimedState = 1
        else: # running/busy/error -> turn off
            aimedState = 0
        self.state = aimedState

import numpy as np
from utils.globals import PATH_TO_PROJECT
import pathlib
def getStitches():
    stitches_path = str(PATH_TO_PROJECT / pathlib.Path('config', 'PuntosCam.txt'))
    points = []
    with open(stitches_path, 'r') as f:
        for i, line in enumerate(f):
            line_list = line.split()
            point = [float(p) for p in line_list]
            points.append(point)
    return i+1, np.array(points)
    
    
    
    if startROS:
        resp = interface_client()
        num = resp['num']
        # stitches = resp['stitches'] # unsused in the interface
        raw = resp['raw']
        # print(raw)
        # print('-----')
        # print(resp['stitches'])
        raw_matrix = np.array(raw).reshape(len(raw)//2, 2)
        # print(raw_matrix)
        return num, raw_matrix
    else:
        return None, None