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


#%%
class CameraWorker(QThread):
    """Handles the camera thread.
    
    Inherits from QThread.
    Has an ImageUpdate signal that is emitted on every new frame.
    """
    ImageUpdate = pyqtSignal(QImage)
    
    def __init__(self):
        """CameraWorker constructor
        
        Sets the deviceIndex to -1.

        Returns
        -------
        None.

        """
        super().__init__()
        self.deviceIndex = -1
    
    def run(self):
        """
        Runs the thread. Emits the ImageUpdate signal on every new good frame.

        Returns
        -------
        None.

        """
        self.ThreadActive = True
        
        for i in range(-1,5): # try 6 camera indices (including -1)
            self.deviceIndex = i
            Capture = cv2.VideoCapture(self.deviceIndex)
            if Capture is not None and Capture.isOpened():
                break
        
        if Capture is not None and Capture.isOpened():
            while self.ThreadActive:
                ret, frame = Capture.read()
                if ret:
                    Image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    ConvertToQtFormat = QImage(Image.data, Image.shape[1], Image.shape[0], QImage.Format_RGB888)
                    Pic = ConvertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                    self.ImageUpdate.emit(Pic)
        else:
            print(f'Unable to open video camera {self.deviceIndex}')
        
    def stop(self):
        """
        Stops the thread.

        Returns
        -------
        None.

        """
        print('Calling stop() for Camera...')
        self.ThreadActive = False
        self.wait()
        print('Camera stopped...')


class RealSenseCameraWorker(QThread):
    """Handles the RealSense camera thread.
    
    Inherits from QThread.
    Has a frame_signal signal that is emitted on every new frame.
    """
    frame_signal = pyqtSignal(QImage, QImage)

    def __init__(self):
        """RealSenseCameraWorker constructor
        
        Sets the pipeline and the config. Enables the depth stream.

        Returns
        -------
        None.

        """
        super().__init__()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    def run(self):
        """
        Runs the thread. Emits the frame_signal signal on every new frame.

        Returns
        -------
        None.

        """
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        if self.config.can_resolve(pipeline_wrapper):
            self.pipeline.start(self.config)
            self.running = True
            try:
                while self.running:
                    frames = self.pipeline.wait_for_frames()
                    depth_frame = frames.get_depth_frame()
                    color_frame = frames.get_color_frame()
                
                    depth_image = None
                    if depth_frame:
                        depth_npframe = np.asanyarray(depth_frame.get_data())
                        depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_npframe, alpha=0.03), cv2.COLORMAP_JET)
                        depth_qimage = QImage(depth_image.data, depth_image.shape[1], depth_image.shape[0], QImage.Format_RGB888)
                    
                    color_image = None
                    if color_frame:
                        color_npframe = np.asanyarray(color_frame.get_data())
                        color_image = color_npframe
                        color_qimage = QImage(color_image.data, color_image.shape[1], color_image.shape[0], QImage.Format_BGR888)
                
                    self.frame_signal.emit(color_qimage, depth_qimage)
            except Exception as e:
                print(e)
                self.running = False
            finally:
                self.pipeline.stop()
        else:
            print('Unable to open RealSense depth camera.')

    def stop(self):
        """
        Stops the thread.

        Returns
        -------
        None.

        """
        print('Calling stop() for RealSense...')
        self.running = False
        self.wait()
        print('RealSense stopped...')
        




