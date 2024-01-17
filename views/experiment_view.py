# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 09:55:42 2024

@author: arnau
"""
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QGraphicsScene
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QMetaObject, Q_ARG
from PyQt5.QtGui import QPixmap, QColor, QBrush, QPen

from utils.camera_threads import CameraWorker, RealSenseCameraWorker
from utils.globals import PATH_TO_PROJECT

import pathlib


#%%
Verbose = False  # very simple debugging/log

class ExperimentView(QWidget):
    def __init__(self):
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
    
        self.CameraWorker0 = CameraWorker(1, verbose=Verbose) # change index depending on number of cameras connected
        self.CameraWorker0.ImageUpdate.connect(self.ImageUpdateSlot0)
        
        self.CameraWorker1 = RealSenseCameraWorker(verbose=Verbose)
        self.CameraWorker1.frame_signal.connect(self.ImageUpdateSlot1)
        
        
        self.drawGraphics()
    
    def setPaths(self):
        self.uiPath = str(PATH_TO_PROJECT / pathlib.Path('ui_files', 'experiment_view.ui'))
        
        self.logoRACEPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'RACE.png'))
        self.logoUMHPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'UMH.png'))
        self.logoUVAPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'UVA.png'))
        self.logoUMAPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'UMA.png'))
        
    def ImageUpdateSlot0(self, Image):
        if Verbose: print('recieve frames from cam 0')
        self.Cam0Label.setPixmap(QPixmap.fromImage(Image))
    
    def ImageUpdateSlot1(self, depth_qimage):
        if Verbose: print('recieve frames from cam 1')
        QMetaObject.invokeMethod(self.Cam1Label, 'setPixmap', Qt.QueuedConnection, Q_ARG(QPixmap, QPixmap.fromImage(depth_qimage)))

    def stopCameras(self):
        if Verbose: print('cancel feed')
        if self.CameraWorker0.isRunning(): self.CameraWorker0.stop()
        if self.CameraWorker1.isRunning(): self.CameraWorker1.stop()

    def closeEvent(self, event):
        self.stopCameras()
        self.com.closed.emit()
        event.accept()
    
    def startCameras(self):
        self.CameraWorker0.start()
        self.CameraWorker1.start()


    def drawGraphics(self):
        self.scene = QGraphicsScene()
        self.whiteBrush = QBrush(Qt.white)
        self.grayBrush = QBrush(Qt.gray)
        self.pen = QPen(Qt.black)
        
        self.flujogramaView.setGeometry(0, 0, 600, 500)
        
        self.scene.addRect(200,200,200,200, self.pen, self.whiteBrush)
        self.flujogramaView.setScene(self.scene)
        
        # https://github.com/pbauermeister/dfd


class Communicate(QObject):
    closed = pyqtSignal()


