# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 13:51:57 2024

@author: arnau
"""
from typing import List
import os

from .experiment_model import Experiment, loadExperimentFromFile


#%%
class ExperimentsModel(List[Experiment]):
    def __str__(self) -> str:
        return "\n".join([str(x.name) for x in self])
    
    def loadExperiments(self):
        # Restrictions could be added to this function, e.g. file extension.
        Path = r"./config/experiments"
        filesList = [os.path.join(Path, f) for f in os.listdir(Path) if os.path.isfile(os.path.join(Path, f))]
        for f in filesList:
            self.append(loadExperimentFromFile(f))
    
    @property
    def names(self):
        return [x.name for x in self]