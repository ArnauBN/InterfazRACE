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
    """Handles all the diferent devices.
    
    Creates an instance of every device.
    Loads all experiments.
    Connects devices signals to slots for upwards propagation.
    """
    def __init__(self):
        """
        MainModel consturctor.
        
        Creates instances of every device.
        Loads all experiments.
        Connects devices signals to slots for upwards propagation.
        Creates a Communicate instance for the handling of signals.

        Returns
        -------
        None.

        """
        self.experiments   = ExperimentsModel()
        self.experiments.loadExperiments()
        
        self.URteleoperado = URTeleoperadoDevice()
        self.URautonomo    = URautonomoDevice()
        self.camara        = CamaraDevice()
        self.endostitch    = EndostitchDevice(state=0, startROS=endo_startROS)
        self.razonador     = RazonadorDevice(state=0, startROS=razonador_startROS, experimentsList=self.experiments.names)
        self.phantom       = PhantomDevice()
        
        self.endostitch.com.stateChanged.connect(self.updateEndostitchState)
        self.razonador.com.faseChanged.connect(self.updateRazonadorFase)
        
        self.com = Communicate()

    def setupURteleoperado(self):
        """
        Initialization and setup of the URteleoperado device (if needed).
        Not implemented yet.

        Returns
        -------
        None.

        """
        pass
    
    def setupURautonomo(self):
        """
        Initialization and setup of the URautonomo device (if needed).
        Not implemented yet.

        Returns
        -------
        None.

        """
        pass
    
    def setupCamara(self):
        """
        Initialization and setup of the Camara device (if needed).
        Not implemented yet.

        Returns
        -------
        None.

        """
        pass
    
    def setupEndostitch(self):
        """
        Initialization and setup of the Endostitch device (if needed).
        Not implemented yet.

        Returns
        -------
        None.

        """
        pass
    
    def setupRazonador(self):
        """
        Initialization and setup of the Razonador device (if needed).
        Not implemented yet.

        Returns
        -------
        None.

        """
        pass
    
    def setupPhantom(self):
        """
        Initialization and setup of the Phantom device (if needed).
        Not implemented yet.

        Returns
        -------
        None.

        """
        pass
    
    def updateEndostitchState(self, newState):
        """
        Slot tied to endostitch.com.stateChanged. 
        Emits the com.endostitchStateChanged signal.

        Parameters
        ----------
        newState : int
            New state.

        Returns
        -------
        None.

        """
        self.com.endostitchStateChanged.emit(newState)

    def updateRazonadorFase(self, newFase):
        """
        Slot tied to razonador.com.faseChanged. 
        Emits the com.razonadorFaseChanged signal.

        Parameters
        ----------
        newFase : int
            New phase.

        Returns
        -------
        None.

        """
        self.com.razonadorFaseChanged.emit(newFase)
    
    
class Communicate(QObject):
    """Simple auxiliary class to handle custom signals for MainModel"""
    endostitchStateChanged = pyqtSignal(int)
    razonadorFaseChanged = pyqtSignal(int)
