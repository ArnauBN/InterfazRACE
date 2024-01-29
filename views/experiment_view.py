# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 09:55:42 2024

@author: arnau
"""
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QGraphicsScene
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QMetaObject, Q_ARG
from PyQt5.QtGui import QPixmap, QColor, QBrush, QPen

import pathlib

from utils.camera_threads import CameraWorker, RealSenseCameraWorker
from utils.globals import PATH_TO_PROJECT, CAMERA_0_INDEX
from utils.DFDGUIobjects import CustomScene


#%%
Verbose = False  # very simple debugging/log

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
        
        Has two camera worker threads, one for a regular cam and another for a
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
        
        
        grey = QPixmap(640, 480)
        grey.fill(QColor('darkGray'))
        self.Cam0Label.setPixmap(grey)
        self.Cam1Label.setPixmap(grey)
        
        self.com = Communicate()
    
        self.CameraWorker0 = CameraWorker(CAMERA_0_INDEX, verbose=Verbose) # change index depending on number of cameras connected
        self.CameraWorker0.ImageUpdate.connect(self.ImageUpdateSlot0)
        
        self.CameraWorker1 = RealSenseCameraWorker(verbose=Verbose)
        self.CameraWorker1.frame_signal.connect(self.ImageUpdateSlot1)
        
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
        
    def ImageUpdateSlot0(self, Image):
        """
        Slot tied to camera worker 0 (regular camera). Updates frame.

        Parameters
        ----------
        Image : QImage
            New frame.

        Returns
        -------
        None.

        """
        if Verbose: print('recieve frames from cam 0')
        self.Cam0Label.setPixmap(QPixmap.fromImage(Image))
    
    def ImageUpdateSlot1(self, depth_qimage):
        """
        Slot tied to camera worker 1 (RealSense Depth camera). Updates frame.

        Parameters
        ----------
        Image : QImage
            New frame.

        Returns
        -------
        None.

        """
        if Verbose: print('recieve frames from cam 1')
        QMetaObject.invokeMethod(self.Cam1Label, 'setPixmap', Qt.QueuedConnection, Q_ARG(QPixmap, QPixmap.fromImage(depth_qimage)))

    def stopCameras(self):
        """
        Stops both camera threads.

        Returns
        -------
        None.

        """
        if Verbose: print('cancel feed')
        if self.CameraWorker0.isRunning(): self.CameraWorker0.stop()
        if self.CameraWorker1.isRunning(): self.CameraWorker1.stop()

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
        self.CameraWorker0.start()
        self.CameraWorker1.start()


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


