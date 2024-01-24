# -*- coding: utf-8 -*-
"""
Created on Wed Jan 10 13:15:29 2024

@author: arnau
"""
from .experiments_model import ExperimentsModel
from .urteleoperado_model import URTeleoperadoDevice
from .camara_model import CamaraDevice
from .urautonomo_model import URautonomoDevice
from .endostitch_model import EndostitchDevice
from .razonador_model import RazonadorDevice
from .phantom_model import PhantomDevice

from .endostitch_model import startROS as endo_startROS
from .razonador_model import startROS as razonador_startROS

from PyQt5.QtCore import QObject, pyqtSignal


#%%
class MainModel:
    def __init__(self):
        self.URteleoperado = URTeleoperadoDevice()
        self.URautonomo    = URautonomoDevice()
        self.camara        = CamaraDevice()
        self.endostitch    = EndostitchDevice(state=0, startROS=endo_startROS)
        self.razonador     = RazonadorDevice(state=0, startROS=razonador_startROS)
        self.phantom       = PhantomDevice()
        
        self.experiments   = ExperimentsModel()
        self.experiments.loadExperiments()
        
        self.endostitch.com.stateChanged.connect(self.updateEndostitchState)
        self.razonador.com.faseChanged.connect(self.updateRazonadorFase)
        
        self.com = Communicate()

    def setupURteleoperado(self):
        pass
    
    def setupURautonomo(self):
        pass
    
    def setupCamara(self):
        pass
    
    def setupEndostitch(self):
        pass
    
    def setupRazonador(self):
        pass
    
    def setupPhantom(self):
        pass
    
    def updateEndostitchState(self, newState):
        self.com.endostitchStateChanged.emit(newState)

    def updateRazonadorFase(self, newFase):
        self.com.razonadorFaseChanged.emit(newFase)
    
    
class Communicate(QObject):
    """Simple auxiliary class to handle custom signals for MainModel"""
    endostitchStateChanged = pyqtSignal(int)
    razonadorFaseChanged = pyqtSignal(int)
