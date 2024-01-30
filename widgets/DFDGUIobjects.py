# -*- coding: utf-8 -*-
"""
Created on Thu Jan 18 11:49:08 2024

@author: arnau
"""

from PyQt5.QtWidgets import QGraphicsItem, QGraphicsEllipseItem, QGraphicsTextItem, QGraphicsLineItem, QGraphicsScene, QGraphicsRectItem
from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QPen, QColor


#%%
class ProcessItem(QGraphicsEllipseItem):
    """"Handles the visual representation of a DFD Process
    
    Inherits from QGraphicsEllipseItem.
    Has text inside the ellipse.
    Overrides mousePressEvent and resizeEvent.
    """
    def __init__(self, process_id, x, y, text=None, razonadorOBj=None):
        """ProcessItem Constructor
        
        Calls super().__init__(x, y, 100, 50).
        
        Sets the process id, coordinates, text and the razonador instance.
        
        The razonador object is used to force a different phase.

        Parameters
        ----------
        process_id : int
            Process ID.
        x : int
            Horizontal coordinate. The origin is at the top-left corner of the
            QGraphicsScene.
        y : int
            Vertical coordinate. The origin is at the top-left corner of the
            QGraphicsScene.
        text : str, optional
            Text to be placed inside (the string of the DFDitem). If None, the
            id is used instead. The default is None.
        razonadorOBj : razonadorDevice, optional
            razonadorDevice instance. The default is None.

        Returns
        -------
        None.

        """
        super().__init__(x, y, 100, 50)
        self.setBrush(Qt.lightGray)
        # self.setFlag(QGraphicsItem.ItemIsMovable)
        # self.setFlag(QGraphicsItem.ItemIsSelectable)
        # self.setFlag(QGraphicsItem.ItemIsFocusable)
        # self.setAcceptHoverEvents(True)
        self.process_id = process_id
        self.state = 0
        self.razonadorOBj = razonadorOBj
        
        t = str(process_id) if text == None else text
        self.textItem = TextItem(t, self)
        self.textItem.setPos(x + 5, y + 10)
        self.textItem.adjust_font_size(self.rect().width(), self.rect().height())

    # def mouseReleaseEvent(self, event):
    #         # Check if there was minimal movement between press and release events
    #         if event.button() == Qt.LeftButton and event.scenePos() == event.buttonDownScenePos(Qt.LeftButton):
    #             self.state ^= 1
    #             if self.state == 0:
    #                 self.setBrush(Qt.green)
    #             else:
    #                 self.setBrush(Qt.lightGray)
    def mousePressEvent(self, event):
        """
        Overrides parent's mousePressEvent. Swaps the state and changes the
        color from green to gray and viceversa. The phase of the razonador is
        changed as well.

        Parameters
        ----------
        event : QEvent
            mousePressEvent event.

        Returns
        -------
        None.

        """
        self.state ^= 1
        if self.state==1:
            if self.razonadorOBj is not None:
                self.razonadorOBj.changeFase(self.process_id)
            self.setBrush(Qt.green)
        else:
            self.setBrush(Qt.lightGray)

    def resizeEvent(self, event):
        """
        Overrides parent's resizeEvent. Adjust the text size inside.

        Parameters
        ----------
        event : QEvent
            resizeEvent event.

        Returns
        -------
        None.

        """
        # Handle resizing of the ellipse, and adjust the font size accordingly
        super().resizeEvent(event)
        self.text_item.adjust_font_size(self.rect().width(), self.rect().height())



class TextItem(QGraphicsTextItem):
    """Handles the text inside a Process"""
    def __init__(self, text, parent=None):
        """TextItem constructor
        
        Calls super().__init__(text, parent).
        
        Ignores transformations.

        Parameters
        ----------
        text : str
            The text itself.
        parent : QGraphicsItem, optional
            Parent. The default is None.

        Returns
        -------
        None.

        """
        super().__init__(text, parent)
        self.setFlag(QGraphicsTextItem.ItemIgnoresTransformations)
        
    def adjust_font_size(self, ellipse_width, ellipse_height):
        """
        Adjusts the font size to fit the ellipse.

        Parameters
        ----------
        ellipse_width : int
            Ellipse Width.
        ellipse_height : int
            Ellipse Height.

        Returns
        -------
        None.

        """
        # Adjust font size to fit inside the ellipse
        text_width = self.boundingRect().width()
        text_height = self.boundingRect().height()
    
        scale_factor = min(ellipse_width / text_width, ellipse_height / text_height)
        current_font = self.font()
    
        # Set font size based on the minimum scale factor
        current_font.setPointSizeF(current_font.pointSizeF() * scale_factor)
        self.setFont(current_font)

class DataFlowItem(QGraphicsLineItem):
    """Handles a DataFlow item (line)
    
    Currently doesn't work.
    """
    def __init__(self, source, destination):
        """DataFlowItem constructor
        
        Calls super().__init__().
        
        Sets pen and line.

        Parameters
        ----------
        source : QGraphicsItem
            Source item. Must have x() and y() callable methods.
        destination : QGraphicsItem
            Destination item. Must have x() and y() callable methods.

        Returns
        -------
        None.

        """
        super().__init__()
        self.setPen(QPen(Qt.black, 2))
        # self.setLine(source.x() + 25, source.y() + 25, destination.x() + 25, destination.y() + 25)
        self.setLine(source.x(), source.y(), destination.x(), destination.y())


class ResetButton(QGraphicsRectItem):
    """"Handles the visual representation of the RESET button
    
    Inherits from QGraphicsRectItem.
    Overrides mousePressEvent, hoverEnterEvent and hoverLeaveEvent.
    """
    def __init__(self, x, y, text="Reset", razonadorOBj=None):
        """ResetButton Constructor
        
        Calls super().__init__(x, y, 100, 50).
        
        Sets the coordinates, text and the razonador instance.
        
        The razonador object is used to force the phase '0'.

        Parameters
        ----------
        x : int
            Horizontal coordinate. The origin is at the top-left corner of the
            QGraphicsScene.
        y : int
            Vertical coordinate. The origin is at the top-left corner of the
            QGraphicsScene.
        text : str, optional
            Text to be placed inside (the string of the DFDitem). The default
            is "Reset".
        razonadorOBj : razonadorDevice, optional
            razonadorDevice instance. The default is None.

        Returns
        -------
        None.

        """
        super().__init__(x, y, 100, 50)
        self.setAcceptHoverEvents(True)
        self.setBrush(Qt.lightGray)
        self.razonadorOBj = razonadorOBj  
        self.state = 0
        
        self.textItem = TextItem(text, self)
        self.textItem.setPos(x, y)
        self.textItem.adjust_font_size(self.rect().width(), self.rect().height())

    def hoverEnterEvent(self, event):
        """
        Overrides parent's hoverEnterEvent. Sets a pointing hand cursor and the
        gray color.

        Parameters
        ----------
        event : QEvent
            hoverEnterEvent event.
        
        Returns
        -------
        None.

        """
        if self.state == 0:
            self.setBrush(Qt.gray)
        self.setCursor(Qt.PointingHandCursor)

    def hoverLeaveEvent(self, event):
        """
        Overrides parent's hoverLeaveEvent. Sets an arrow cursor and the
        lightgray color.

        Parameters
        ----------
        event : QEvent
            hoverLeaveEvent event.
        
        Returns
        -------
        None.

        """
        if self.state == 0:
            self.setBrush(Qt.lightGray)
        self.setCursor(Qt.ArrowCursor)

    def mousePressEvent(self, event):
        """
        Overrides parent's mousePressEvent. Changes the razonador phase, the 
        button's state and its color.

        Parameters
        ----------
        event : QEvent
            mousePressEvent event.

        Returns
        -------
        None.

        """
        if self.razonadorOBj is not None:
            self.razonadorOBj.changeFase("0")
        
        self.state ^= 1
        if self.state==1:
            self.setBrush(Qt.green)
        else:
            self.setBrush(Qt.lightGray)






class CustomScene(QGraphicsScene):
    """Handles a custom scene with a border"""
    def __init__(self, parent=None):
        """CustomScene constructor
        
        Calls super().__init__(parent).
        
        Sets the scene Rect, the border color and the background color.

        Parameters
        ----------
        parent : QGraphicsView?, optional
            Parent. The default is None.

        Returns
        -------
        None.

        """
        super().__init__(parent)
        
        # Set the scene rectangle to ensure that the border is drawn within the specified bounds
        self.setSceneRect(QRectF(-250, -500, 500, 1000))
        self.border_color = QColor(Qt.black)
        self.background_color = QColor(Qt.white)

    def drawBackground(self, painter, rect):
        """
        Draws the background.
        
        Calls super().drawBackground(painter, rect).

        Parameters
        ----------
        painter : QPainter
            Painter.
        rect : QRect
            Rect.

        Returns
        -------
        None.

        """
        super().drawBackground(painter, rect)

        # Fill the background with a color
        painter.fillRect(self.sceneRect(), self.background_color)

        # Draw the border around the scene
        border_rect = self.sceneRect().adjusted(-1, -1, 1, 1)
        painter.setPen(self.border_color)
        painter.drawRect(border_rect)