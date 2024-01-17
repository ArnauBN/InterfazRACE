# -*- coding: utf-8 -*-
"""
Created on Wed Jan 10 13:13:39 2024

@author: arnau
"""
from models.main_model import MainModel
from views.main_view import MainView
from controllers.experiment_controller import ExperimentController

from controllers.endostitch_controller import EndostitchController # Endostitch


#%%
class MainController:
    """Handles the communication between the data model and the GUI view.
    
    Creates an instance of MainView and MainModel when instanced.
    Creates an instance of ExperimentController when the iniciar button is
    pressed.
    Adds functionality to the buttons.
    """
    def __init__(self):
        """MainController Constructor.
        
        Creates an instance of MainModel and MainView, connects each button to
        a method.
        When the experiment window is closed, the experimentController object
        is deleted.
        """
        self.model = MainModel()
        self.view = MainView()
        
        
        self.view.URteleoperadoButton.clicked.connect(self.on_click_URteleoperadoButton)
        self.view.URautonomoButton.clicked.connect(self.on_click_URautonomoButton)
        self.view.camaraButton.clicked.connect(self.on_click_camaraButton)
        self.view.endostitchButton.clicked.connect(self.on_click_endostitchButton)
        self.view.razonadorButton.clicked.connect(self.on_click_razonadorButton)
        self.view.phantomButton.clicked.connect(self.on_click_phantomButton)
        self.view.iniciarButton.clicked.connect(self.on_click_iniciarButton)
    
        for exp in self.model.experiments.names:
            self.view.experimentsListWidget.addItem(exp)
        self.view.experimentsListWidget.setCurrentRow(0)
        
        self.model.setupURteleoperado()
        self.model.setupURautonomo()
        self.model.setupCamara()
        self.model.setupEndostitch()
        self.model.setupRazonador()
        self.model.setupPhantom()
        
        
        # Endostitch
        self.endostitch_controller = EndostitchController(self)
        
    
    def on_click_URteleoperadoButton(self):
        self.model.URteleoperado.changeState()
        self.view.changeState(self.view.URteleoperadoButton, self.model.URteleoperado.state)
    
    def on_click_URautonomoButton(self):
        self.model.URautonomo.changeState()
        self.view.changeState(self.view.URautonomoButton, self.model.URautonomo.state)
    
    def on_click_camaraButton(self):
        self.model.camara.changeState()
        self.view.changeState(self.view.camaraButton, self.model.camara.state)
    
    def on_click_endostitchButton(self):
        self.model.endostitch.changeState()
        self.view.changeState(self.view.endostitchButton, self.model.endostitch.state)
        
        # Endostitch
        if self.model.endostitch.state==1:
            self.endostitch_controller.view.show()
        else:
            self.endostitch_controller.view.hide()
    
    def on_click_razonadorButton(self):
        self.model.razonador.changeState()
        self.view.changeState(self.view.razonadorButton, self.model.razonador.state)
    
    def on_click_phantomButton(self):
        self.model.phantom.changeState()
        self.view.changeState(self.view.phantomButton, self.model.phantom.state)

    def on_click_iniciarButton(self):
        idx = self.view.experimentsListWidget.currentRow()
        self.expController = ExperimentController(self, idx)
        self.expController.view.com.closed.connect(self.experimentWindowClosed)
    
    def experimentWindowClosed(self):
        del self.expController # maybe we don't want to do this