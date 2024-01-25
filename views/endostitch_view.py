# -*- coding: utf-8 -*-
"""
Created on Wed Jan 17 17:35:15 2024

@author: arnau
"""
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget

import pathlib

from utils.globals import PATH_TO_PROJECT


#%%
class EndostitchView(QWidget):
    """Controls the GUI elements and appearence of the Endostitch Window.
    
    Inherits from QWidget.
    A .ui file is loaded.
    """
    def __init__(self):
        """EndostitchView Constructor.
        
        Calls super().__init__() and loads endostitch_view.ui file.

        Returns
        -------
        None.

        """
        super().__init__()
        self.setPaths()
        uic.loadUi(self.uiPath, self)
    
    def setPaths(self):
        """
        Sets necessary paths using the global variable PATH_TO_PROJECT.
        OS agnostic.

        Returns
        -------
        None.

        """
        self.uiPath = str(PATH_TO_PROJECT / pathlib.Path('ui_files', 'endostitch_view.ui'))