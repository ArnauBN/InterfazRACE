# -*- coding: utf-8 -*-
"""
Created on Tue Jan 16 10:50:56 2024

@author: arnau
"""
import sys
import cv2
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget
import pyrealsense2 as rs


#%%
class CameraThread(QThread):
    frame_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        QThread.__init__(self)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.running = True

    def run(self):
        print("CameraThread started")
        self.pipeline.start(self.config)
        while self.running:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()

            depth_image = None
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())

            self.frame_signal.emit(depth_image)

    def stop(self):
        print("Stopping CameraThread")
        self.running = False
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        self.depth_label = QLabel(self.central_widget)

        layout = QVBoxLayout(self.central_widget)
        layout.addWidget(self.depth_label)

        self.camera_thread = CameraThread()
        self.camera_thread.frame_signal.connect(self.update_frames)
        self.camera_thread.start()

    def update_frames(self, depth_frame):
        # You can similarly update the depth label if needed
        depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)
        depth_qimage = QImage(depth_image.data, depth_image.shape[1], depth_image.shape[0], QImage.Format_RGB888)
        self.depth_label.setPixmap(QPixmap.fromImage(depth_qimage))

    def closeEvent(self, event):
        self.camera_thread.stop()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
