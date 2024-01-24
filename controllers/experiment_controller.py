# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 10:20:38 2024

@author: arnau
"""
from models.experiment_model import Experiment
from views.experiment_view import ExperimentView
from PyQt5.QtCore import Qt

from utils.DFDobjects import ProcessItem

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
        self.mainController.model.com.razonadorFaseChanged.connect(self.on_razonador_fase_changed)
        


        self.addItems2DFD()

        self.view.show()
        self.startCameras()

    def addItems2DFD(self):
        self.p1 = ProcessItem(1, 200, 100, 'Fase 1', self.mainController.model.razonador)
        self.p2 = ProcessItem(2, 200, 150, 'Fase 2', self.mainController.model.razonador)
        self.p3 = ProcessItem(3, 200, 200, 'Fase 3', self.mainController.model.razonador)
        self.p4 = ProcessItem(4, 200, 250, 'Fase 4', self.mainController.model.razonador)
        self.p5 = ProcessItem(5, 200, 300, 'Fase 5', self.mainController.model.razonador)
        self.p6 = ProcessItem(6, 200, 350, 'Fase 6', self.mainController.model.razonador)
        self.p7 = ProcessItem(7, 200, 400, 'Fase 7', self.mainController.model.razonador)
        self.p8 = ProcessItem(8, 200, 450, 'Fase 8', self.mainController.model.razonador)
        self.p9 = ProcessItem(9, 200, 500, 'Fase 9', self.mainController.model.razonador)

        self.view.scene.addItem(self.p1)
        self.view.scene.addItem(self.p2)
        self.view.scene.addItem(self.p3)
        self.view.scene.addItem(self.p4)
        self.view.scene.addItem(self.p5)
        self.view.scene.addItem(self.p6)
        self.view.scene.addItem(self.p7)
        self.view.scene.addItem(self.p8)
        self.view.scene.addItem(self.p9)

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

    def on_razonador_fase_changed(self, newFase):
        # aqui cosas
        self.p1.setBrush(Qt.lightGray)
        self.p2.setBrush(Qt.lightGray)
        self.p3.setBrush(Qt.lightGray)
        self.p4.setBrush(Qt.lightGray)
        self.p5.setBrush(Qt.lightGray)
        self.p6.setBrush(Qt.lightGray)
        self.p7.setBrush(Qt.lightGray)
        self.p8.setBrush(Qt.lightGray)
        self.p9.setBrush(Qt.lightGray)
        if newFase == 1:
            self.p1.setBrush(Qt.green)
        if newFase == 2:
            self.p2.setBrush(Qt.green)
        if newFase == 3:
            self.p3.setBrush(Qt.green)
        if newFase == 4:
            self.p4.setBrush(Qt.green)
        if newFase == 5:
            self.p5.setBrush(Qt.green)
        if newFase == 6:
            self.p6.setBrush(Qt.green)
        if newFase == 7:
            self.p7.setBrush(Qt.green)
        if newFase == 8:
            self.p8.setBrush(Qt.green)
        if newFase == 9:
            self.p9.setBrush(Qt.green)



    
