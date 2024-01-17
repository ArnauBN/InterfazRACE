# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 10:20:38 2024

@author: arnau
"""
from models.experiment_model import Experiment
from views.experiment_view import ExperimentView


#%%
class ExperimentController:
    def __init__(self, MainController, idx):
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
        self.model.start()
    
    def on_click_terminarButton(self):
        self.model.end()
    
    def on_click_stopButton(self):
        self.model.stop()
    
    def startCameras(self):
        if self.mainController.model.camara.state == 1:
            self.view.startCameras()
    
    








    
