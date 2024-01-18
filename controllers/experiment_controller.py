# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 10:20:38 2024

@author: arnau
"""
from models.experiment_model import Experiment
from views.experiment_view import ExperimentView


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
        """
        self.model = Experiment()
        self.view = ExperimentView()
        self.mainController = MainController
        
        self.experiment = self.mainController.model.experiments[idx]
        
        self.view.continuarButton.clicked.connect(self.on_click_continuarButton)
        self.view.terminarButton.clicked.connect(self.on_click_terminarButton)
        self.view.stopButton.clicked.connect(self.on_click_stopButton)
        
        self.view.show()
        self.startCameras()

    def on_click_continuarButton(self):
        """
        Method tied to continuar button. Calls model.start().

        Returns
        -------
        None.

        """
        self.model.start()
    
    def on_click_terminarButton(self):
        """
        Method tied to terminar button. Calls model.end().

        Returns
        -------
        None.

        """
        self.model.end()
    
    def on_click_stopButton(self):
        """
        Method tied to stop button. Calls model.stop().

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
    
    








    
