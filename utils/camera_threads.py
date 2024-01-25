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
    
    def __init__(self, deviceIndex=0, verbose=False):
        """CameraWorker constructor
        
        Sets the deviceIndex. If verbose is True, the thread will print 
        something when the thread is ran, stopped, receiving good frames and
        when the camera is not accesible.

        Parameters
        ----------
        deviceIndex : int, optional
            Device index. Depends on OS and number of cameras connected. The
            default is 0.
        verbose : bool, optional
            If true, show some debugging info. The default is False.

        Returns
        -------
        None.

        """
        super().__init__()
        self.deviceIndex = deviceIndex
        self.verbose = verbose
    
    def run(self):
        """
        Runs the thread. Emits the ImageUpdate signal on every new good frame.

        Returns
        -------
        None.

        """
        if self.verbose: print('\nrun feed')
        self.ThreadActive = True       
        Capture = cv2.VideoCapture(self.deviceIndex) 
        if Capture is not None and Capture.isOpened():
            while self.ThreadActive:
                ret, frame = Capture.read()
                if ret:
                    Image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    ConvertToQtFormat = QImage(Image.data, Image.shape[1], Image.shape[0], QImage.Format_RGB888)
                    Pic = ConvertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
                    self.ImageUpdate.emit(Pic)
                    if self.verbose: print('send good frames')
        else:
            print(f'Unable to open video camera {self.deviceIndex}')
        
    def stop(self):
        """
        Stops the thread.

        Returns
        -------
        None.

        """
        if self.verbose: print('stop feed')
        self.ThreadActive = False
        self.wait()

class RealSenseCameraWorker(QThread):
    """Handles the RealSense camera thread.
    
    Inherits from QThread.
    Has a frame_signal signal that is emitted on every new frame.
    """
    frame_signal = pyqtSignal(QImage)

    def __init__(self, verbose=False):
        """RealSenseCameraWorker constructor
        
        Sets the pipeline and the config. Enables the depth stream.
        
        If verbose is True, the thread will print something when the thread is
        ran, stopped and when the camera is not accesible or an exception
        occurs.

        Parameters
        ----------
        verbose : bool, optional
            If true, show some debugging info. The default is False.

        Returns
        -------
        None.

        """
        super().__init__()
        self.verbose = verbose
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    def run(self):
        """
        Runs the thread. Emits the frame_signal signal on every new frame.

        Returns
        -------
        None.

        """
        if self.verbose: print("RealSenseCameraWorker started")
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        if self.config.can_resolve(pipeline_wrapper):
            self.pipeline.start(self.config)
            self.running = True
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
        else:
            print('Unable to open RealSense depth camera.')

    def stop(self):
        """
        Stops the thread.

        Returns
        -------
        None.

        """
        if self.verbose: print("Stopping RealSenseCameraWorker")
        self.running = False
        self.wait()
        




