# -*- coding: utf-8 -*-
"""
Created on Wed Jan 10 12:34:35 2023
Python version: Python 3.11

@author: Arnau Busqué Nadal <arnau.busque@goumh.umh.es>

This is the main file, intended to be executed.
The style can be changed.
The program uses an MVC architecture.
"""
import sys

from PyQt5.QtWidgets import QApplication, QStyleFactory

from controllers.main_controller_ import MainController


def main():
    style = 'Fusion'
    
    app = QApplication(sys.argv)
    if style in QStyleFactory.keys(): app.setStyle(style)
    
    # app.setQuitOnLastWindowClosed(True)

    controller = MainController()
    controller.view.show()
    
    width = controller.view.frameGeometry().width()
    height = controller.view.frameGeometry().height()
    controller.view.setFixedSize(width, height)

    app.exec()

if __name__ == '__main__':
    main()