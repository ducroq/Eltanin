#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import numpy as np
# Note: PySide QThread implementation seems very buggy (2020), so reverted to PyQt
##from PySide2.QtGui import *
##from PySide2.QtWidgets import *
##from PySide2.QtCore import *
from PyQt5.QtCore import Qt, QSettings
from PyQt5.QtWidgets import QApplication
from mainWindow import MainWindow
from log import LogWindow
from checkOS import is_raspberry_pi
from printHat import PrintHat
from pyqtpicam import PiVideoStream
from sysTemp import SystemTemperatures
from imageProcessor import ImageProcessor
from batchProcessor import BatchProcessor


if __name__ == "__main__":
    if not is_raspberry_pi():
        print("This app is for RPi. Quitting")
        sys.exit(0)
        
    # Create event loop and instantiate objects
    settings = QSettings("settings.ini", QSettings.IniFormat)
    app = QApplication(sys.argv)
    mw = MainWindow()
    lw = LogWindow()
    vs = PiVideoStream()
    ph = PrintHat() # Connect to printhat virtual port (this links the klipper software as well)
    st = SystemTemperatures(interval=10, alarm_temperature=55)
    ip = ImageProcessor()
    bp = BatchProcessor("batch.ini") # filename should probably be selected with a QFileDialog

    # Start threads
    vs.start()
    ph.start()

    # Control signals, some explicitly of QueuedConnection type to prevent signal loss
    st.alarm.connect(lambda: ph.setFanPWM(1.0))
    st.alarmRemoved.connect(lambda: ph.setFanPWM(0.5))
    vs.signals.result.connect(ip.update, type=Qt.BlockingQueuedConnection)
    bp.setLightPWM.connect(ph.setLightPWM)
    bp.disableMotors.connect(ph.disableMotors)
    bp.gotoXY.connect(ph.gotoXY, type=Qt.QueuedConnection)
    bp.gotoX.connect(ph.gotoX, type=Qt.QueuedConnection)
    bp.gotoY.connect(ph.gotoY, type=Qt.QueuedConnection)
    bp.gotoZ.connect(ph.gotoZ, type=Qt.QueuedConnection)
    ph.positionReached.connect(bp.positionReached, type=Qt.QueuedConnection)
    bp.findDiaphragm.connect(ip.findDiaphragm, type=Qt.QueuedConnection)
    ip.diaphragmFound.connect(bp.diaphragmFound, type=Qt.QueuedConnection)
    bp.findWell.connect(ip.findWell, type=Qt.QueuedConnection)
    ip.wellFound.connect(bp.wellFound, type=Qt.QueuedConnection)
    bp.takeSnapshot.connect(vs.snapshot, type=Qt.QueuedConnection)
    vs.snapshotTaken.connect(bp.snapshotTaken, type=Qt.QueuedConnection)
    bp.recordClip.connect(vs.recordClip, type=Qt.QueuedConnection)
    vs.clipRecorded.connect(bp.clipRecorded, type=Qt.QueuedConnection)
    bp.computeSharpnessScore.connect(ip.computeSharpnessScore, type=Qt.QueuedConnection)
    ip.sharpnessScore.connect(bp.sharpnessScore, type=Qt.QueuedConnection)
    
    # Connect GUI signals
    mw.runButton.clicked.connect(bp.start)
    mw.runButton.released.connect(mw.disableButtons)    
    mw.breakButton.clicked.connect(ph.emergencyBreak)
    mw.restartButton.clicked.connect(ph.firmwareRestart)
    mw.homeXYZButton.clicked.connect(ph.homeXYZ)
    mw.stageOriginButton.clicked.connect(lambda: ph.gotoXY(0,0))
    mw.snapshotButton.clicked.connect(lambda: vs.snapshot(os.path.sep.join([settings.value('storage_main_path'), 'data','snapshot'])))
    mw.recordButton.clicked.connect(lambda: vs.recordClip(os.path.sep.join([settings.value('storage_main_path'), 'data','clip']), 10))
    mw.light.valueChanged.connect(lambda x: ph.setLightPWM(x/100))
    mw.stageXTranslation.valueChanged.connect(lambda x: ph.gotoXY(x, mw.stageYTranslation.value()))
    mw.stageYTranslation.valueChanged.connect(lambda y: ph.gotoXY(mw.stageXTranslation.value(),y))
    mw.stageZTranslation.valueChanged.connect(lambda z: ph.gotoZ(z))
    ip.signals.result.connect(mw.update) #, type=Qt.QueuedConnection)
    vs.signals.ready.connect(lambda: mw.kickTimer())
    vs.signals.progress.connect(mw.updateProgressBar)
    ph.signals.ready.connect(mw.enableButtons)
    ph.signals.ready.connect(st.resetAlarm)
    bp.signals.progress.connect(mw.updateProgressBar)
    bp.signals.ready.connect(mw.enableButtons)

    # temporary: 
##    bp.gotoXY.connect(lambda x,y: mw.stageXTranslation.setValue(x))
##    bp.gotoXY.connect(lambda x,y: mw.stageYTranslation.setValue(y))
##    bp.gotoZ.connect(mw.stageZTranslation.setValue)
##    ph.signals.home.connect(lambda: mw.stageXTranslation.setValue(0))
##    ph.signals.home.connect(lambda: mw.stageYTranslation.setValue(0))
##    ph.signals.home.connect(lambda: mw.stageZTranslation.setValue(0))
    
    
    # Connect logging signals
    vs.signals.message.connect(lw.append)
    vs.signals.error.connect(lambda s: "error;{}".format(lw.append(s[2])))
    ph.signals.message.connect(lw.append)
    ph.signals.error.connect(lambda s: "error;{}".format(lw.append(s[2])))
    st.signals.message.connect(lw.append)
    st.signals.error.connect(lambda s: "error;{}".format(lw.append(s[2])))
    ip.signals.message.connect(lw.append)
    ip.signals.error.connect(lambda s: "error;{}".format(lw.append(s[2])))
    bp.signals.message.connect(lw.append)
    bp.signals.error.connect(lambda s: "error;{}".format(lw.append(s[2])))
    bp.setLogFileName.connect(lw.setLogFileName)

    # Connect closing signals
    bp.signals.finished.connect(mw.close, type=Qt.QueuedConnection)
    mw.signals.finished.connect(vs.stop, type=Qt.QueuedConnection)
    mw.signals.finished.connect(ip.stop, type=Qt.QueuedConnection)
    mw.signals.finished.connect(st.stop, type=Qt.QueuedConnection)
    mw.signals.finished.connect(bp.stop, type=Qt.QueuedConnection)
    mw.signals.finished.connect(ph.stop, type=Qt.QueuedConnection)
    mw.signals.finished.connect(lw.close, type=Qt.QueuedConnection)

    # Start the show
    lw.append("App started")
    mw.move(50,0)
    mw.resize(1500, 700)
    lw.move(50, 750)
    lw.resize(1500, 200)
    mw.show()
    lw.show()
    sys.exit(app.exec_())



    
