# -*- coding: utf-8 -*-
"""
Created on Wed Jan 10 13:16:36 2024

@author: arnau
"""
from PyQt5 import uic
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import Qt

from utils.globals import PATH_TO_PROJECT

import pathlib

#%%
class MainView(QMainWindow):
    """Controls the GUI elements and appearence of the MainWindow.
    
    Inherits from QMainWindow.
    A .ui file is loaded and some images and icons are added.
    The closeEvent() method is overrriden to close all other windows as well.
    """
    
    def __init__(self):
        """MainView Constructor.
        
        Calls super().__init__(), loads main_view.ui file, sets Icons for each
        button, sets the title and title format using html and a logo, adds
        three more logos to the side using the labels defined in the .ui file.
        """
        super().__init__()
        self.setPaths()
        uic.loadUi(self.uiPath, self)

        self.URteleoperadoButton.setIcon(QIcon(self.iconOffPath))
        self.URautonomoButton.setIcon(QIcon(self.iconOffPath))
        self.camaraButton.setIcon(QIcon(self.iconOffPath))
        self.endostitchButton.setIcon(QIcon(self.iconOffPath))
        self.razonadorButton.setIcon(QIcon(self.iconOffPath))
        self.phantomButton.setIcon(QIcon(self.iconOffPath))
        
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
    
    def setPaths(self):
        self.uiPath = str(PATH_TO_PROJECT / pathlib.Path('ui_files', 'main_view.ui'))
        
        self.iconOffPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'icons', 'status-offline.png'))
        self.iconOnPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'icons', 'status.png'))
        self.iconAwayPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'icons', 'status-away.png'))
        self.iconBusyPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'icons', 'status-busy.png'))
        
        self.logoRACEPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'RACE.png'))
        self.logoUMHPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'UMH.png'))
        self.logoUVAPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'UVA.png'))
        self.logoUMAPath = str(PATH_TO_PROJECT / pathlib.Path('resources', 'logos', 'UMA.png'))
    
    def changeState(self, button, newState):
        """
        Changes the icons of the buttons depending on the specified state.
        State 0: Off (grey)
        State 1: On (green)
        State 2: Away/processing (orange)
        State 3: Busy/error (red)

        Parameters
        ----------
        button : QPushButton
            Button instance.
        newState : int
            State to change to.

        Returns
        -------
        None.

        """
        button.state = newState
        if button.state==0:
            button.setIcon(QIcon(self.iconOffPath))
        elif button.state==1:
            button.setIcon(QIcon(self.iconOnPath))
        elif button.state==2:
            button.setIcon(QIcon(self.iconAwayPath))
        else:
            button.setIcon(QIcon(self.iconBusyPath))
    
    def closeEvent(self, event):
        """
        This method is called when the window is closed.
        
        It closes all other windows before closing this one, terminating the
        program.

        Parameters
        ----------
        event : QEvent
            Close event.

        Returns
        -------
        None.

        """
        for window in QApplication.topLevelWidgets():
            window.close()
        event.accept()
        
        