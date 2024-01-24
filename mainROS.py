#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 18 09:15:00 2024

@author: arnau
"""

from mainInterface import main

import rospy


#%%
def mainROS():
    rospy.init_node('interfaz',anonymous=True)
    main()

if __name__ == '__main__':
    mainROS()
