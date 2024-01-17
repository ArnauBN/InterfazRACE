# -*- coding: utf-8 -*-
"""
Created on Mon Jan 15 14:31:50 2024

@author: arnau
"""

import pyrealsense2 as rs
import numpy as np
import cv2


#%%
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        depth_image = np.asanyarray(depth_frame.get_data())
        
        cv2.imshow("Depth. Press 'q' to quit", depth_image)
        
        if cv2.waitKey(1) == ord('q'): # press 'q' to quit
            break
finally:
    cv2.destroyAllWindows()
    pipeline.stop()
