# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 13:12:03 2024

@author: arnau
"""

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