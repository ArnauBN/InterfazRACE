#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 18 09:15:00 2024
Python version: Python 3.11

@author: Arnau Busqué Nadal <arnau.busque@goumh.umh.es>

This is the main ROS file, intended to be executed with ROS.
It calls the main entry point of the interface in mainInterface.py.
"""

from mainInterface import main

import rospy


#%%
def mainROS():
    rospy.init_node('interfaz', anonymous=True)
    main()

if __name__ == '__main__':
    mainROS()
