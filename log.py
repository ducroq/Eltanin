"""@package docstring
Logging object
""" 
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import time
from PyQt5.QtCore import pyqtSlot, QSettings
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QTextEdit


class LogWindow(QWidget):

    def __init__(self):
        super().__init__()
        self.log_file_name = None
        self.setWindowTitle("Log")
        layout = QVBoxLayout()
        self.setLayout(layout)
        self.log = QTextEdit()
        layout.addWidget(self.log)

    @pyqtSlot(str)
    def append(self, s):
        self.log.append(s)
        if self.log_file_name is not None:
            with open(self.log_file_name, 'a+') as log_file:
                s = s if '\n' in s else s + '\n'
                s = str(round(time.time(),1)) + ";" + s
                log_file.write(s)            
            

    @pyqtSlot(str)
    def setLogFileName(self, s):
        self.log_file_name = s
