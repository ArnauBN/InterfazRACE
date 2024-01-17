# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 13:51:48 2024

@author: arnau
"""
import serial


#%%
class SerialDevice:
    def __init__(self, boardName='Arduino UNO'):
        self.boardName = boardName
        self.ser = None
    
    def Serial(self, port='COM3', baudrate=9600, timeout=None):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)  # open comms
    
    @property
    def port(self):
        return self.ser.port

    @port.setter
    def port(self, p):
        self.ser.port = p

    @property
    def baudrate(self):
        return self.ser.baudrate

    @baudrate.setter
    def baudrate(self, b):
        self.ser.baudrate = b
    
    def open(self):
        if not self.ser.isOpen():
            self.ser.open()

    def close(self):
        try:
            if self.ser.isOpen():
                self.ser.close()
                print(f'Serial communication with {self.board} at port {self.port} closed successfully.')
            else:
                print(f'Serial communication with {self.board} at port {self.port} was already closed.')
        except Exception as e:
            print(e)
    
    def write(self, data):
        return self.ser.write(bytes(data))
    
    def read(self):
        return self.ser.read()
    
    def readline(self):
        return self.ser.readline()

    
            
        

        
    
    
    
    
