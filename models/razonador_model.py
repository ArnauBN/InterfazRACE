# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 13:16:46 2024

@author: arnau
"""
from .device_model import Device


#%%
class RazonadorDevice(Device):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)