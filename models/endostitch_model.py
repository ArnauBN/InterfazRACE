# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 13:15:05 2024

@author: arnau
"""
from .serialDevice_model import SerialDevice


#%%
class EndostitchDevice(SerialDevice):
    def __init__(self, state=0, boardName='Arduino UNO'):
        super().__init__(boardName)
        self._state = state

    def abrirPinza(self):
        return self.write('1')
    
    def cerrarPinza(self):
        return self.write('2')
    
    def moverMariposaDerecha(self):
        return self.write('3')
    
    def moverMariposaCentro(self):
        return self.write('4')
    
    def moverMariposaIzquierda(self):
        return self.write('5')

    @property
    def state(self):
        return self._state
    
    @state.setter
    def state(self, newState):
        if newState==0:
            if self.ser is not None: self.close()
        elif newState==1:
            if self.ser is not None:
                self.open()
            else:
                print('Serial communication not initiated.')
                return
        else:
            print('Wrong state for Endostitch: available states are 0 (off) or 1 (on).')
            return
        self._state = newState
    
    def changeState(self):
        self.state = self.state ^ 1 # XOR

