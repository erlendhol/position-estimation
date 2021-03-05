import numpy as np
import math
from abc import *

class Frame(ABC):
    
    
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation
    
    def GetPosition():
        return position

    def GetOrientation():
        return orientation
    
    def UpdatePosition(newPosition):
        self.position = newPosition

    def UpdateOrientation(newOrientation):
        self.orientation = newOrientation

    def ParentFrame(object):
        pass

class WorldFrame(Frame):

    pass

class BoatFrame(Frame):

    def __init__(self, position, orientation, parentFrame):
        self.position = position
        self.orientation = orientation
        self.parentFrame = parentFrame
    
    def GetPosition():
        return position

    def GetOrientation():
        return orientation
    
    def UpdatePosition(newPosition):
        self.position = newPosition

    def UpdateOrientation(newOrientation):
        self.orientation = newOrientation

class GWBaseFrame(Frame):

    def __init__(self, position, orientation, parentFrame):
        self.position = position
        self.orientation = orientation
        self.parentFrame = parentFrame
    
    def GetPosition():
        return position

    def GetOrientation():
        return orientation
    
    def UpdatePosition(newPosition):
        self.position = newPosition

    def UpdateOrientation(newOrientation):
        self.orientation = newOrientation

class GWTipFrame(Frame):

    def __init__(self, position, orientation, parentFrame):
        self.position = position
        self.orientation = orientation
        self.parentFrame = parentFrame
    
    def GetPosition():
        return position

    def GetOrientation():
        return orientation
    
    def UpdatePosition(newPosition):
        self.position = newPosition

    def UpdateOrientation(newOrientation):
        self.orientation = newOrientation






class A(object):
    __metaclass__ = ABCMeta
    @abstractmethod
    def __init__(self, n):
        self.n = n

if __name__ == '__main__':
    a = A(3)