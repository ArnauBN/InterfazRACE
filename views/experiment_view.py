# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 09:55:42 2024

@author: arnau
"""
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QGraphicsScene
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QPixmap, QBrush, QPen
import pathlib

from utils.globals import PATH_TO_PROJECT
from .cameras_view import CamView, RealSenseCamView


#%%
class ExperimentView(QWidget):
    """Controls the GUI elements and appearence of the Experiment Window.
    
    Inherits from QWidget.
    A .ui file is loaded and some images are added.
    The closeEvent() method is overrriden to stop the camera threads.
    """
    def __init__(self):
        """ExperimentView Constructor.
        
        Calls super().__init__(), loads experiment_view.ui file, sets the title
        and title format using html and a logo, adds three more logos to the
        side using the labels defined in the .ui file.
        
        Has two camera dialogs, one for a regular cam and another for a
        RealSense Depth camera.
        
        Has a QGraphicsView object for Data Flow Diagrams.
        
        Creates an instance of Communicate.

        Returns
        -------
        None.

        """
        super().__init__()
        self.setPaths()
        uic.loadUi(self.uiPath, self)

        self.label.setText(f'''
                           <html><head/><body><p><b>
                           <img src="{self.logoRACEPath}"
                           width="160"
                           height="117"
                           style="vertical-align:middle"/>
                           <span style="font-size:16pt;">
                           Robot Anastomosis Competence Evaulation
                           </span></b></p></body></html>
                           ''')

        UMHpixmap = QPixmap(self.logoUMHPath).scaled(64, 64, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        UVApixmap = QPixmap(self.logoUVAPath).scaled(64, 64, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        UMApixmap = QPixmap(self.logoUMAPath).scaled(64, 64, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.UMHLabel.setPixmap(UMHpixmap)
        self.UVALabel.setPixmap(UVApixmap)
        self.UMALabel.setPixmap(UMApixmap)
        
        self.com = Communicate()

        self.cam0dialog = CamView()
        self.cam1dialog = RealSenseCamView()
        
        self.drawGraphics()

    
    def setPaths(self):
        """
        Sets necessary paths using the global variable PATH_TO_PROJECT.
        OS agnostic.

        Returns
        -------
        None.

        """
        self.uiPath = str(PATH_TO_PROJECT / pathlib.Path('ui_files', 'experiment_view.ui'))
        
        self.logoRACEPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'RACE.png'))
        self.logoUMHPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'UMH.png'))
        self.logoUVAPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'UVA.png'))
        self.logoUMAPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'UMA.jpeg'))
    
    def stopCameras(self):
        """
        Stops both camera threads.

        Returns
        -------
        None.

        """
        self.cam0dialog.stopCam()
        self.cam1dialog.stopCam()
        self.cam0dialog.hide()
        self.cam1dialog.hide()

    def closeEvent(self, event):
        """
        Override closeEvent method of Qwidget. Calls stopCameras(), emits the
        closed signal, and closes the window.

        Parameters
        ----------
        event : QEvent
            Close event.

        Returns
        -------
        None.

        """
        self.stopCameras()
        self.com.closed.emit()
        event.accept()
    
    def startCameras(self):
        """
        Starts both camera threads.

        Returns
        -------
        None.

        """
        self.cam0dialog.show()
        self.cam1dialog.show()
        self.cam0dialog.setFixedSize(self.cam0dialog.size())
        self.cam1dialog.setFixedSize(self.cam1dialog.size())
        self.cam0dialog.startCam()
        self.cam1dialog.startCam()


    def drawGraphics(self):
        """
        Draws the DFD.
        
        TODO

        Returns
        -------
        None.

        """
        # self.scene = QGraphicsScene(-500, -1000, 1000, 2000)
        self.scene = QGraphicsScene()
        # self.scene = CustomScene()
        self.whiteBrush = QBrush(Qt.white)
        self.grayBrush = QBrush(Qt.gray)
        self.pen = QPen(Qt.black)
        
        # self.flujogramaView.setAlignment(Qt.AlignLeft | Qt.AlignTop) # aligns center of scene and view
        self.flujogramaView.setGeometry(0, 0, int(self.scene.width()), int(self.scene.height()))
        
        # data_flow12 = DataFlowItem(p1, p2)
        
        self.flujogramaView.setScene(self.scene)
        # self.flujogramaView.setSceneRect(self.scene.sceneRect())
        # self.resize(self.scene.sceneRect().size().toSize())
        
        # https://github.com/pbauermeister/dfd


class Communicate(QObject):
    """Simple auxiliary class to handle custom signals for ExperimentView"""
    closed = pyqtSignal()


