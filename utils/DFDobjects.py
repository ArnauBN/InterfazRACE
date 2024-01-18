# -*- coding: utf-8 -*-
"""
Created on Thu Jan 18 11:49:08 2024

@author: arnau
"""

from PyQt5.QtWidgets import QGraphicsItem, QGraphicsEllipseItem, QGraphicsTextItem, QGraphicsLineItem
from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QFont


class ProcessItem(QGraphicsEllipseItem):
    def __init__(self, process_id, x, y, text=None):
        super().__init__(x, y, 100, 50)
        self.setBrush(Qt.lightGray)
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.process_id = process_id
        self.state = 0
        
        t = str(process_id) if text == None else text
        self.textItem = TextItem(t, self)
        self.textItem.setPos(x + 5, y + 10)
        self.textItem.adjust_font_size(self.rect().width(), self.rect().height())

    def mousePressEvent(self, event):
        self.state ^= 1
        if self.state==0:
            self.setBrush(Qt.green)
        else:
            self.setBrush(Qt.lightGray)
    
    
    def resizeEvent(self, event):
            # Handle resizing of the ellipse, and adjust the font size accordingly
            super().resizeEvent(event)
            self.text_item.adjust_font_size(self.rect().width(), self.rect().height())

class TextItem(QGraphicsTextItem):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setFlag(QGraphicsTextItem.ItemIgnoresTransformations)
        
    def adjust_font_size(self, ellipse_width, ellipse_height):
        # Adjust font size to fit inside the ellipse
        text_width = self.boundingRect().width()
        text_height = self.boundingRect().height()
    
        scale_factor = min(ellipse_width / text_width, ellipse_height / text_height)
        current_font = self.font()
    
        # Set font size based on the minimum scale factor
        current_font.setPointSizeF(current_font.pointSizeF() * scale_factor)
        self.setFont(current_font)