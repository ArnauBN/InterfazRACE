# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 13:51:57 2024

@author: arnau
"""
from typing import List
import pathlib

from .experiment_model import Experiment, loadExperimentFromFile
from utils.globals import PATH_TO_PROJECT


#%%
class ExperimentsModel(List[Experiment]):
    def __str__(self) -> str:
        return "\n".join([str(x.name) for x in self])
    
    def loadExperiments(self):
        # Restrictions could be added to this function, e.g. file extension.
        Path = PATH_TO_PROJECT / pathlib.Path('config', 'experiments')
        filesList = [file_path for file_path in Path.iterdir() if file_path.is_file()]
        for f in filesList:
            self.append(loadExperimentFromFile(f))
    
    @property
    def names(self):
        return [x.name for x in self]