# -*- coding: utf-8 -*-
"""
Created on Thu Jan 25 09:43:05 2024

@author: arnau
"""
import re
import numpy as np
from PyQt5.QtGui import QImage, QColor, QPainter, QPixmap


#%%
def findFirstNumber(s: str):
    """
    Finds the first number (with any number of digits) in the string.
    Returns the number as a int or None if no number is found.

    Parameters
    ----------
    s : str
        Input string.

    Returns
    -------
    int or None
        The number found.

    """
    match = re.search(r'\d+', s)
    if match:
        return int(match.group())
    else:
        return None

def addOverlayCircle(img, x, y, r, color=(255, 0, 0, 128)):
    result_pixmap = QPixmap(img)
    
    # Create a QPainter to draw on the pixmap
    painter = QPainter(result_pixmap)
    painter.setRenderHint(QPainter.Antialiasing)

    # Set the color and transparency for the circle
    overlay_color = QColor(*color) # Red with 50% transparency by default

    # Draw the circle on the pixmap
    painter.setBrush(overlay_color)
    painter.drawEllipse(x, y, 2 * r, 2 * r)
    
    # End painting
    painter.end()

    return result_pixmap

def addOverlayCircles(img, x_coords, y_coords, r_values, color=(255, 0, 0, 128)):
    result_pixmap = QPixmap(img)
    painter = QPainter(result_pixmap)
    painter.setRenderHint(QPainter.Antialiasing)
    overlay_color = QColor(*color) # Red with 50% transparency by default
    painter.setBrush(overlay_color)
    
    for x, y, r in zip(x_coords, y_coords, r_values):
        painter.drawEllipse(x, y, 2 * r, 2 * r)
    
    painter.end()
    return result_pixmap   
    
def drawStitches(img, stitches_matrix):
    x_coords = stitches_matrix[:, 0]
    y_coords = stitches_matrix[:, 1]
    # z_coords = stitches_matrix[:, 2]
    # r_values = map_z_to_radius(z_coords, 60)
    r_values = np.array(np.ones(len(x_coords))*5)
    return addOverlayCircles(img, x_coords, y_coords, r_values, color=(255, 0, 0, 128))
    
def map_z_to_radius(z_coords, max_radius):
    return max_radius / z_coords