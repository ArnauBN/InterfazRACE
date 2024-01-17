# -*- coding: utf-8 -*-
"""
Created on Wed Jan 10 13:16:36 2024

@author: arnau
"""
from PyQt5 import uic
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import Qt


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
        uic.loadUi("./ui_files/main_view.ui", self)
        self.URteleoperadoButton.setIcon(QIcon("./resources/icons/status-offline.png"))
        self.URautonomoButton.setIcon(QIcon("./resources/icons/status-offline.png"))
        self.camaraButton.setIcon(QIcon("./resources/icons/status-offline.png"))
        self.endostitchButton.setIcon(QIcon("./resources/icons/status-offline.png"))
        self.razonadorButton.setIcon(QIcon("./resources/icons/status-offline.png"))
        self.phantomButton.setIcon(QIcon("./resources/icons/status-offline.png"))
        
        self.label.setText('''
                           <html><head/><body><p><b>
                           <img src="./resources/logos/RACE.png"
                           width="160"
                           height="117"
                           style="vertical-align:middle"/>
                           <span style="font-size:16pt;">
                           Robot Anastomosis Competence Evaulation
                           </span></b></p></body></html>
                           ''')

        UMHpixmap = QPixmap('./resources/logos/UMH.png').scaled(64, 64, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        UVApixmap = QPixmap('./resources/logos/UVA.png').scaled(64, 64, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        UMApixmap = QPixmap('./resources/logos/UMA.jpeg').scaled(64, 64, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.UMHLabel.setPixmap(UMHpixmap)
        self.UVALabel.setPixmap(UVApixmap)
        self.UMALabel.setPixmap(UMApixmap)
        
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
            button.setIcon(QIcon("./resources/icons/status-offline.png"))
        elif button.state==1:
            button.setIcon(QIcon("./resources/icons/status.png"))
        elif button.state==2:
            button.setIcon(QIcon("./resources/icons/status-away.png"))
        else:
            button.setIcon(QIcon("./resources/icons/status-busy.png"))
    
    
    
    # Modify this method to do something when closing the program
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
        
        