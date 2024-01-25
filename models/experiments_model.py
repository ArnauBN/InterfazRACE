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
    """Handles several experiments.
    
    Inherits from python's typing.List.
    Overrides __str__ method.
    """
    def __str__(self) -> str:
        """
        Returns the names of the experiments separated by newline ('\n').

        Returns
        -------
        str
            The string representation of the object.

        """
        return "\n".join([str(x.name) for x in self])
    
    def loadExperiments(self):
        """
        Loads all experiments found in ./config/experiments. Each file in this
        folder is considered an experiment.

        Returns
        -------
        None.

        """
        # Restrictions could be added to this function, e.g. file extension.
        Path = PATH_TO_PROJECT / pathlib.Path('config', 'experiments')
        filesList = [file_path for file_path in Path.iterdir() if file_path.is_file()]
        for f in filesList:
            self.append(loadExperimentFromFile(f))
    
    @property
    def names(self):
        """
        Getter method for property: names.

        Returns
        -------
        list
            List of all experiment names.

        """
        return [x.name for x in self]