##from PySide2.QtCore import QObject, pyqtSignal
from PyQt5.QtCore import QObject, pyqtSignal
import numpy as np
from enum import Enum

class ObjectSignals(QObject):
    '''
    Defines the pyqtSignals available from an object.

    Supported pyqtSignals are:

    finished
        No data
    
    error
        `tuple` (exctype, value, traceback.format_exc() )
    
    result
        `image` data returned from processing, np.ndarray

    progress
        `int` indicating % progress 

    '''
    finished = pyqtSignal()
    error = pyqtSignal(tuple)
    message = pyqtSignal(str)
    result = pyqtSignal(np.ndarray)
    ready = pyqtSignal()    
    progress = pyqtSignal(int)

    
    
