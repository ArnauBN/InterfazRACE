# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 10:20:56 2024

@author: arnau
"""
# We could implement a DFD system, e.g.: https://github.com/pbauermeister/dfd

from pathlib import Path
from utils.DFDparsers import parseMultipleLines


#%%
class Experiment:
    """Handles the experiment itself.
    
    Has a state, a name, and a Data-Flow-Diagram.
    Has start, stop and end methods. These are not yet implemented.
    """
    def __init__(self, name: str=None, DFD=None):
        """
        Experiment constructor. Sets initial state, name and DFD.

        Parameters
        ----------
        name : str, optional
            Name of experiment. The default is None.
        DFD : DFD, optional
            DFD instance. The default is None.

        Returns
        -------
        None.

        """
        self.state = 0 # 0=not-started, 1=ended, 2=on-going, -1=stopped
        self.name = name
        self.DFD = DFD

    #TODO
    def start(self):
        """
        Could be implemented to a start an automatic experiment.

        Returns
        -------
        None.

        """
        pass
    
    #TODO
    def stop(self):
        """
        Could be implemented to a stop an automatic experiment. 
        
        For now, it sets the state to -1.

        Returns
        -------
        None.

        """
        self.state = -1
    
    #TODO
    def end(self):
        """
        Could be implemented to end an automatic experiment.

        Returns
        -------
        None.

        """
        pass

def loadExperimentFromFile(path: str) -> Experiment:
    """
    Loads an experiment from a text file.
    
    The name of the experiment is taken from the first line of the file only if
    it starts with '#' otherwise the name of the file itself is taken as the
    experiment name. The file should describe the Data-Flow-Diagram with the 
    syntax used in this public repository: https://github.com/pbauermeister/dfd
    
    For example:
        file 'exp1.txt':
            # Experiment 1
            process P1 Phase 1
            process P2 Phase 2
        would result in an experiment named 'Experiment 1' with 2 processes 
        named 'Phase 1' and 'Phase 2'.
        
        file 'exp2.txt':
            process P1 Phase 1
            process P2 Phase 2
        would result in an experiment named 'exp2' with 2 processes named 
        'Phase 1' and 'Phase 2'.
    
    Parameters
    ----------
    path : str
        Path (absolute or relative) of the experiment file.

    Returns
    -------
    Experiment
        Experiment object.

    """
    with open(path, 'r') as f:
        data = f.readlines()
    experimentName = data.pop(0)[1:].strip() if data[0][0] == '#' else Path(path).stem
    DFD = parseMultipleLines(data)
    return Experiment(experimentName, DFD)