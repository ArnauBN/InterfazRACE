# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 11:06:33 2024

@author: arnau
"""

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage
import cv2
import numpy as np
import pyrealsense2 as rs

# Camera feed
# https://stackoverflow.com/questions/65583343/how-to-display-a-live-opencv-video-feed-using-a-pyqt5


#%%
class CameraWorker(QThread):
    ImageUpdate = pyqtSignal(QImage)
    
    def __init__(self, deviceIndex=0, verbose=False):
        super().__init__()
        self.deviceIndex = deviceIndex
        self.verbose = verbose
    
    def run(self):
        if self.verbose: print('\nrun feed')
        self.ThreadActive = True       
        Capture = cv2.VideoCapture(self.deviceIndex) 

        while self.ThreadActive:
            ret, frame = Capture.read()
            if ret:
                Image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                ConvertToQtFormat = QImage(Image.data, Image.shape[1], Image.shape[0], QImage.Format_RGB888)
                Pic = ConvertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                self.ImageUpdate.emit(Pic)
                if self.verbose: print('send good frames')
            
    def stop(self):
        if self.verbose: print('stop feed')
        self.ThreadActive = False
        self.wait()
        # self.quit()



class RealSenseCameraWorker(QThread):
    frame_signal = pyqtSignal(QImage)

    def __init__(self, verbose=False):
        super().__init__()
        self.verbose = verbose
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.running = True

    def run(self):
        if self.verbose: print("RealSenseCameraWorker started")
        self.pipeline.start(self.config)
        try:
            while self.running:
                frames = self.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
            
                depth_image = None
                if depth_frame:
                    depth_npframe = np.asanyarray(depth_frame.get_data())
                    depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_npframe, alpha=0.03), cv2.COLORMAP_JET)
                    depth_qimage = QImage(depth_image.data, depth_image.shape[1], depth_image.shape[0], QImage.Format_RGB888)
            
                self.frame_signal.emit(depth_qimage)
        except Exception as e:
            if self.verbose: print("Exception in RealSenseCameraWorker:", e)
            self.running = False
        finally:
            self.pipeline.stop()

    def stop(self):
        if self.verbose: print("Stopping RealSenseCameraWorker")
        self.running = False
        self.wait()
        # self.pipeline.stop()
        # self.quit()
        




