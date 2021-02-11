"""@package docstring
wait functions
"""
 
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import subprocess
from PyQt5.QtCore import QTimer, QObject, pyqtSignal

## @brief Periodically check wifi connection and alarm if disconnected
## @author Jeroen Veen
class CheckWiFi(QObject):
    timer = QTimer()
    postMessage = pyqtSignal(str)
    
    def __init__(self, interval=100):
        super().__init__()
        self.interval = 1000*interval
        self.timer.timeout.connect(self.update)
        self.timer.start(self.interval)
        
    def update(self):
        ps = subprocess.Popen(['iwgetid'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            output = subprocess.check_output(('grep', 'ESSID'), stdin=ps.stdout)
        except subprocess.CalledProcessError:
            # grep did not match any lines
            self.postMessage.emit("{}: error; No wireless networks connected".format(self.__class__.__name__))
            print("No wireless networks connected")           
