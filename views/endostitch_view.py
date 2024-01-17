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
    def __init__(self):
        super().__init__()
        self.setPaths()
        uic.loadUi(self.uiPath, self)
    
    def setPaths(self):
        self.uiPath = str(PATH_TO_PROJECT / pathlib.Path('ui_files', 'endostitch_view.ui'))