# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 10:20:56 2024

@author: arnau
"""

# We could implement a DFD system, e.g.: https://github.com/pbauermeister/dfd

class Experiment:
    def __init__(self, name: str=None):
        self.state = 0 # 0=not-started, 1=ended, 2=on-going, -1=stopped
        self.name = name

    #TODO
    def start(self):
        pass
    
    #TODO
    def stop(self):
        self.state = -1
    
    #TODO
    def end(self):
        pass

#TODO
def loadExperimentFromFile(path: str) -> Experiment:
    # This is just a temporary example
    with open(path, 'r') as f:
        data = f.readline()
    return Experiment(data)