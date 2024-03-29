# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 10:20:38 2024

@author: arnau
"""
from models.experiment_model import Experiment
from views.experiment_view import ExperimentView
from PyQt5.QtCore import Qt

from widgets.DFDGUIobjects import ProcessItem, ResetButton


#%%
class ExperimentController:
    """Handles the communication between the data model and the GUI view.
    
    Creates an instance of Experiment and ExperimentView when instantiated.
    Shows the view.
    Starts the camera threads.
    Adds functionality to the experiment's buttons.
    """
    def __init__(self, MainController, idx):
        """ExperimentController Constructor.
        
        Requires an instance of the main controller and an experiment index.
        Creates an instance of Experiment and ExperimentView, connects each
        button to a method. Starts the camera threads.
        
        Parameters
        ----------
        MainController : MainController
            MainController instance.
        idx : int
            Experiment index.

        Returns
        -------
        None.

        """
        self.model = Experiment()
        self.view = ExperimentView()
        self.mainController = MainController
        
        self.experiment = self.mainController.model.experiments[idx]
        self.mainController.model.razonador.experiment = self.experiment.name
        
        self.view.continuarButton.clicked.connect(self.on_click_continuarButton)
        self.view.terminarButton.clicked.connect(self.on_click_terminarButton)
        self.view.stopButton.clicked.connect(self.on_click_stopButton)
        self.mainController.model.com.razonadorFaseChanged.connect(self.on_razonador_fase_changed)
        
        self.view.cam1dialog.depthCheckBox.stateChanged.connect(self.on_stateChanged_depthCheckBox)
        self.view.cam1dialog.stitchesCheckBox.stateChanged.connect(self.on_stateChanged_stitchesCheckBox)

        self.addItems2DFD()

        self.view.show()
        self.startCameras()

    def addItems2DFD(self):
        """
        Creates and adds all DFD items to the view's scene based on the
        selected experiment.

        Returns
        -------
        None.

        """
        self.resetItem = ResetButton(200, 0, "Reset", self.mainController.model.razonador)
        self.view.scene.addItem(self.resetItem)
        
        for i,dfdItem in enumerate(self.experiment.DFD.itemList):
            dfdItem.guiObject = ProcessItem(dfdItem.id, 200, 100 + 50*i, dfdItem.string, self.mainController.model.razonador)
            self.view.scene.addItem(dfdItem.guiObject)

    def on_stateChanged_depthCheckBox(self, state):
        """
        Slot tied to profundidad Check Box. Sets self.view.cam1dialog.DEPTH.

        Parameters
        ----------
        state : int
            State of checkbox:
                0 - off
                1 - partially on
                2 - on

        Returns
        -------
        None.

        """
        if state==0:
            self.view.cam1dialog.DEPTH = False
        else:
            self.view.cam1dialog.DEPTH = True
    
    def on_stateChanged_stitchesCheckBox(self, state):
        """
        Slot tied to profundidad Check Box. Sets 
        self.view.cam1dialog.DRAW_STITCHES.

        Parameters
        ----------
        state : int
            State of checkbox:
                0 - off
                1 - partially on
                2 - on

        Returns
        -------
        None.

        """
        if state==0:
            self.view.cam1dialog.DRAW_STITCHES = False
        else:
            self.view.cam1dialog.DRAW_STITCHES = True

    def on_click_continuarButton(self):
        """
        Slot tied to continuar button. Calls model.start().

        Returns
        -------
        None.

        """
        self.model.start()
    
    def on_click_terminarButton(self):
        """
        Slot tied to terminar button. Calls model.end().

        Returns
        -------
        None.

        """
        self.model.end()
    
    def on_click_stopButton(self):
        """
        Slot tied to stop button. Calls model.stop().

        Returns
        -------
        None.

        """
        self.model.stop()
    
    def startCameras(self):
        """
        Changes the main controller's model camera state to 1 and starts both
        camera threads.

        Returns
        -------
        None.

        """
        if self.mainController.model.camara.state == 1:
            self.view.startCameras()

    def on_razonador_fase_changed(self, newFase):
        """
        Slot tied to the razonadorFaseChanged signal. Changes the color of the
        processes in the DFD view to green or lightgray.

        Returns
        -------
        None.

        """
        if newFase == "0":
            self.resetItem.setBrush(Qt.green)
        else:
            self.resetItem.setBrush(Qt.lightGray)
        
        for i,dfdItem in enumerate(self.experiment.DFD.itemList):
            if newFase == dfdItem.id:
                dfdItem.guiObject.setBrush(Qt.green)
            else:
                dfdItem.guiObject.setBrush(Qt.lightGray)



    
