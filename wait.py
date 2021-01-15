"""@package docstring
wait functions
"""
 
#!/usr/bin/python3
# -*- coding: utf-8 -*-
from PyQt5.QtCore import QTimer, QEventLoop

def wait_signal(signal, timeout=1000):
    ''' Block loop until signal emitted, or timeout (ms) elapses.
    '''
    loop = QEventLoop()
    signal.connect(loop.quit) # only quit is a slot of QEventLoop
    QTimer.singleShot(timeout, loop.exit)
    loop.exec_()


def wait_ms(timeout):
    ''' Block loop until timeout (ms) elapses.
    '''        
    loop = QEventLoop()
    QTimer.singleShot(timeout, loop.exit)
    loop.exec_()
    
