# -*- coding: utf-8 -*-
"""
Created on Wed Jan 10 13:15:29 2024

@author: arnau
"""
from .experiments_model import ExperimentsModel
from .urteleoperado_model import URTeleoperadoDevice
from .camara_model import CamaraDevice
from .urautonomo_model import URautonomoDevice
from .endostitch_model import EndostitchDevice, startROS
from .razonador_model import RazonadorDevice
from .phantom_model import PhantomDevice

from PyQt5.QtCore import QObject, pyqtSignal


#%%
class MainModel:
    def __init__(self):
        self.URteleoperado = URTeleoperadoDevice()
        self.URautonomo    = URautonomoDevice()
        self.camara        = CamaraDevice()
        self.endostitch    = EndostitchDevice(state=0, startROS=startROS)
        self.razonador     = RazonadorDevice()
        self.phantom       = PhantomDevice()
        
        self.experiments   = ExperimentsModel()
        self.experiments.loadExperiments()
        
        self.endostitch.com.stateChanged.connect(self.updateEndostitchState)
        
        self.com = Communicate()

    def setupURteleoperado(self):
        pass
    
    def setupURautonomo(self):
        pass
    
    def setupCamara(self):
        pass
    
    def setupEndostitch(self):
        pass
        # try:
        #     self.endostitch.Serial(port='COM3', baudrate=9600, timeout=None)
        # except Exception as e:
        #     print('Cannot communicate with endostitch.')
        #     print(e)
    
    def setupRazonador(self):
        pass
    
    def setupPhantom(self):
        pass
    
    def updateEndostitchState(self, newState):
        self.com.endostitchStateChanged.emit(newState)
    
    
class Communicate(QObject):
    """Simple auxiliary class to handle custom signals for MainModel"""
    endostitchStateChanged = pyqtSignal(int)
