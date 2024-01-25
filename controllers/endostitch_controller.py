# -*- coding: utf-8 -*-
"""
Created on Wed Jan 17 17:40:09 2024

@author: arnau
"""
from views.endostitch_view import EndostitchView


#%%
class EndostitchController:
    """Handles the communication between the endostitch methods and the buttons
    
    Needs an instance of the main controller.
    Creates and instance of the endostitch view (but does not show it).
    Adds functionality to the endostitch's buttons.
    """
    def __init__(self, MainController):
        """EndostitchController Constructor.
        
        Requires an instance of the main controller.
        Creates an instance of EndostitchView and connects each button to a
        method.

        Parameters
        ----------
        MainController : MainController
            MainController instance.

        Returns
        -------
        None.

        """
        self.view = EndostitchView()
        self.mainController = MainController
        
        self.view.abrirPinzaButton.clicked.connect(self.on_click_abrirPinzaButton)
        self.view.cerrarPinzaButton.clicked.connect(self.on_click_cerrarPinzaButton)
        self.view.moverCentroButton.clicked.connect(self.on_click_moverCentroButton)
        self.view.moverDerechaButton.clicked.connect(self.on_click_moverDerechaButton)
        self.view.moverIzquierdaButton.clicked.connect(self.on_click_moverIzquierdaButton)
        
    def on_click_abrirPinzaButton(self):
        """
        Slot tied to abrirPinzaButton button. Calls the main model's 
        endostitch.abrirPinza().

        Returns
        -------
        None.

        """
        self.mainController.model.endostitch.abrirPinza()
    
    def on_click_cerrarPinzaButton(self):
        """
        Slot tied to cerrarPinzaButton button. Calls the main model's 
        endostitch.cerrarPinzaButton().

        Returns
        -------
        None.

        """
        self.mainController.model.endostitch.cerrarPinza()
    
    def on_click_moverCentroButton(self):
        """
        Slot tied to moverCentroButton button. Calls the main model's 
        endostitch.moverCentroButton().

        Returns
        -------
        None.

        """
        self.mainController.model.endostitch.moverCentro()
    
    def on_click_moverDerechaButton(self):
        """
        Slot tied to moverDerechaButton button. Calls the main model's 
        endostitch.moverDerechaButton().

        Returns
        -------
        None.

        """
        self.mainController.model.endostitch.moverDerecha()
    
    def on_click_moverIzquierdaButton(self):
        """
        Slot tied to moverIzquierdaButton button. Calls the main model's 
        endostitch.moverIzquierdaButton().

        Returns
        -------
        None.

        """
        self.mainController.model.endostitch.moverIzquierda()