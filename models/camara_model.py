# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 13:12:03 2024

@author: arnau
"""
from .device_model import Device




#%%
class CamaraDevice(Device):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)