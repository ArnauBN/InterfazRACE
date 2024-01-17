# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 13:13:48 2024

@author: arnau
"""
from .device_model import Device


#%%
class URautonomoDevice(Device):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)