# -*- coding: utf-8 -*-
"""
Created on Wed Jan 17 10:23:20 2024

@author: arnau
"""

class Device:
    def __init__(self, state=0):
        self._state = state
    
    @property
    def state(self):
        return self._state
    
    @state.setter
    def state(self, newState):
        code = state2code(newState)
        self._state = 2 # intermediate (orange)
        # ackCode = self.write(code)
        ackCode = 'OK' # temporary
        if ackCode=='OK':
            self._state = newState
        else:
            self._state = -1
    
    def changeState(self):
        if self.state in [0,2]: # off/waiting -> turn on
            aimedState = 1
        else: # running/busy/error -> turn off
            aimedState = 0
        self.state = aimedState


    # This should probably be Async
    def write(self, code):
        pass
    
    # This should probably be Async
    def read(self):
        pass
    
    def start(self):
        if self.state==1:
            pass
        elif self.state==0:
            pass


def state2code(state):
    #TODO
    if state==0:
        pass
    if state==1:
        pass
    if state==2:
        pass
    pass
