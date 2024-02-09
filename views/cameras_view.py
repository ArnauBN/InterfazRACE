# -*- coding: utf-8 -*-
"""
Created on Thu Feb  8 12:09:44 2024

@author: arnau
"""
import pathlib

from PyQt5 import uic
from PyQt5.QtWidgets import QDialog
from PyQt5.QtCore import Qt, QMetaObject, Q_ARG
from PyQt5.QtGui import QPixmap, QColor

from utils.camera_threads import CameraWorker, RealSenseCameraWorker
from utils.globals import PATH_TO_PROJECT
from utils.generic import drawStitches
from models.camara_model import getStitches


#%%
class CamView(QDialog):
    """Handles the GUI elements and appearence of the Camera dialog.
    
    Inherits from QDialog.
    A .ui file is loaded.
    The closeEvent() method is overrriden to stop the camera thread.
    """
    def __init__(self):
        """CamView constructor
        
        Calls super().__init__(), loads cam_dialog.ui file. Has a camera 
        thread.
        
        Returns
        -------
        None.

        """
        super().__init__()
        self.setPaths()
        uic.loadUi(self.uiPath, self)
        
        grey = QPixmap(640, 480)
        grey.fill(QColor('darkGray'))
        self.camLabel.setPixmap(grey)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setWindowFlag(Qt.WindowCloseButtonHint, False)
        self.setWindowFlag(Qt.WindowContextHelpButtonHint, False)
        
        self.CameraWorker = CameraWorker()
        self.CameraWorker.ImageUpdate.connect(self.ImageUpdateSlot)
        
        self.startCam()
    
    def setPaths(self):
        """
        Sets necessary paths using the global variable PATH_TO_PROJECT.
        OS agnostic.

        Returns
        -------
        None.

        """
        self.uiPath = str(PATH_TO_PROJECT / pathlib.Path('ui_files', 'cam_dialog.ui'))
        
    def ImageUpdateSlot(self, Image):
        """
        Slot tied to the camera worker. Updates frame.

        Parameters
        ----------
        Image : QImage
            New frame.

        Returns
        -------
        None.

        """
        self.camLabel.setPixmap(QPixmap.fromImage(Image))
    
    def stopCam(self):
        """
        Stops the camera thread.

        Returns
        -------
        None.

        """
        if self.CameraWorker.isRunning(): self.CameraWorker.stop()

    def closeEvent(self, event):
        """
        Overrides closeEvent method of QDialog. Calls stopCam() and closes the
        dialog.

        Parameters
        ----------
        event : QEvent
            Close event.

        Returns
        -------
        None.

        """
        self.stopCam()
        event.accept()
    
    def startCam(self):
        """
        Starts the camera thread.

        Returns
        -------
        None.

        """
        self.CameraWorker.start()
    
    
    
class RealSenseCamView(QDialog):
    """Handles the GUI elements and appearence of the RealSense camera dialog.
    
    Inherits from QDialog.
    A .ui file is loaded.
    The closeEvent() method is overrriden to stop the camera thread.
    """
    def __init__(self):
        """CamView constructor
        
        Calls super().__init__(), loads realsenseCam_dialog.ui file. Has a
        camera thread.
        
        Returns
        -------
        None.

        """
        super().__init__()
        self.setPaths()
        uic.loadUi(self.uiPath, self)
        
        grey = QPixmap(848, 480)
        grey.fill(QColor('darkGray'))
        self.camLabel.setPixmap(grey)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setWindowFlag(Qt.WindowCloseButtonHint, False)
        self.setWindowFlag(Qt.WindowContextHelpButtonHint, False)
        
        self.CameraWorker = RealSenseCameraWorker()
        self.CameraWorker.frame_signal.connect(self.ImageUpdateSlot)
        
        self.DRAW_STITCHES = False
        self.DEPTH = False
        
        self.startCam()

    def setPaths(self):
        """
        Sets necessary paths using the global variable PATH_TO_PROJECT.
        OS agnostic.

        Returns
        -------
        None.

        """
        self.uiPath = str(PATH_TO_PROJECT / pathlib.Path('ui_files', 'realsenseCam_dialog.ui'))
        
    def ImageUpdateSlot(self, color_qimage, depth_qimage):
        """
        Slot tied to the camera worker. Updates frame.

        Parameters
        ----------
        color_qimage : QImage
            New BGR frame.
        depth_qimage : QImage
            New depth frame.

        Returns
        -------
        None.

        """
        img = depth_qimage if self.DEPTH else color_qimage
        
        if self.DRAW_STITCHES:
            num, stitches_raw = getStitches()
            pixmapImg = QPixmap.fromImage(img) if stitches_raw is None else drawStitches(img, stitches_raw)
            QMetaObject.invokeMethod(self.camLabel, 'setPixmap', Qt.QueuedConnection, Q_ARG(QPixmap, pixmapImg))
        else:
            QMetaObject.invokeMethod(self.camLabel, 'setPixmap', Qt.QueuedConnection, Q_ARG(QPixmap, QPixmap.fromImage(img)))
    
    def stopCam(self):
        """
        Stops the camera thread.

        Returns
        -------
        None.

        """
        if self.CameraWorker.isRunning(): self.CameraWorker.stop()

    def closeEvent(self, event):
        """
        Overrides closeEvent method of QDialog. Calls stopCam() and closes the
        dialog.

        Parameters
        ----------
        event : QEvent
            Close event.

        Returns
        -------
        None.

        """
        self.stopCam()
        event.accept()
    
    def startCam(self):
        """
        Starts the camera thread.

        Returns
        -------
        None.

        """
        self.CameraWorker.start()