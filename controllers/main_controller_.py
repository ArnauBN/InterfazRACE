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
    
    Creates an instance of MainView and MainModel when instantiated.
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
        
        self.model.com.endostitchStateChanged.connect(self.on_endostitch_state_changed)
        
        # Endostitch
        self.endostitch_controller = EndostitchController(self)
    
    
    def on_endostitch_state_changed(self, newState):
        self.view.changeState(self.view.endostitchButton, newState)
        if newState==0:
            self.endostitch_controller.view.hide()
        else:
            self.endostitch_controller.view.show()
    
    def on_click_URteleoperadoButton(self):
        """
        Method tied to URteleoperado button. Changes the model's state and the
        view accordingly.

        Returns
        -------
        None.

        """
        self.model.URteleoperado.changeState()
        self.view.changeState(self.view.URteleoperadoButton, self.model.URteleoperado.state)
    
    def on_click_URautonomoButton(self):
        """
        Method tied to URautonomo button. Changes the model's state and the
        view accordingly.

        Returns
        -------
        None.

        """
        self.model.URautonomo.changeState()
        self.view.changeState(self.view.URautonomoButton, self.model.URautonomo.state)
    
    def on_click_camaraButton(self):
        """
        Method tied to camara button. Changes the model's state and the view
        accordingly.

        Returns
        -------
        None.

        """
        self.model.camara.changeState()
        self.view.changeState(self.view.camaraButton, self.model.camara.state)
    
    def on_click_endostitchButton(self):
        """
        Method tied to endostitch button. Changes the model's state and the
        view accordingly.
        
        Temporary: shows and hides endostitch controls.

        Returns
        -------
        None.

        """
        self.model.endostitch.changeState()
        
        # This might not be necessary since we have on_endostitch_state_changed
        self.view.changeState(self.view.endostitchButton, self.model.endostitch.state)
        if self.model.endostitch.state==0:
            self.endostitch_controller.view.hide()
        else:
            self.endostitch_controller.view.show()
    
    def on_click_razonadorButton(self):
        """
        Method tied to razonador button. Changes the model's state and the view
        accordingly.

        Returns
        -------
        None.

        """
        self.model.razonador.changeState()
        self.view.changeState(self.view.razonadorButton, self.model.razonador.state)
    
    def on_click_phantomButton(self):
        """
        Method tied to phantom button. Changes the model's state and the view
        accordingly.

        Returns
        -------
        None.

        """
        self.model.phantom.changeState()
        self.view.changeState(self.view.phantomButton, self.model.phantom.state)

    def on_click_iniciarButton(self):
        """
        Method tied to iniciar button. Using the currently selected experiment,
        creates an instance of ExperimentController (which will show the
        experiment window) and connects the experiment view's closed signal 
        with the experimentWindowClosed method.

        Returns
        -------
        None.

        """
        idx = self.view.experimentsListWidget.currentRow()
        self.expController = ExperimentController(self, idx)
        self.expController.view.com.closed.connect(self.experimentWindowClosed)
    
    def experimentWindowClosed(self):
        """
        Method tied to the experiment view's closed signal. Deletes the 
        experiment controller instance when the window is closed.

        Returns
        -------
        None.

        """
        del self.expController # maybe we don't want to do this
