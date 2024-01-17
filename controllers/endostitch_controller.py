# -*- coding: utf-8 -*-
"""
Created on Wed Jan 17 17:40:09 2024

@author: arnau
"""
from views.endostitch_view import EndostitchView


#%%
class EndostitchController:
    def __init__(self, MainController):
        self.view = EndostitchView()
        self.mainController = MainController
        
        self.view.abrirPinzaButton.clicked.connect(self.on_click_abrirPinzaButton)
        self.view.cerrarPinzaButton.clicked.connect(self.on_click_cerrarPinzaButton)
        self.view.moverCentroButton.clicked.connect(self.on_click_moverCentroButton)
        self.view.moverDerechaButton.clicked.connect(self.on_click_moverDerechaButton)
        self.view.moverIzquierdaButton.clicked.connect(self.on_click_moverIzquierdaButton)
        
    def on_click_abrirPinzaButton(self):
        self.mainController.model.endostitch.abrirPinza()
    
    def on_click_cerrarPinzaButton(self):
        self.mainController.model.endostitch.cerrarPinza()
    
    def on_click_moverCentroButton(self):
        self.mainController.model.endostitch.moverCentro()
    
    def on_click_moverDerechaButton(self):
        self.mainController.model.endostitch.moverDerecha()
    
    def on_click_moverIzquierdaButton(self):
        self.mainController.model.endostitch.moverIzquierda()