#!/usr/bin/python3
# -*- coding: utf-8 -*-
from checkOS import is_raspberry_pi

if not is_raspberry_pi():
    print("ERROR: this app is for raspberrypi")
    exit()

import os, sys
import numpy as np
from PyQt5.QtCore import Qt, QSettings
from PyQt5.QtWidgets import QApplication
from mainWindow import MainWindow
from log import LogWindow
from printHat import PrintHat
from pyqtpicam import PiVideoStream
from sysTemp import SystemTemperatures
from imageProcessor import ImageProcessor
from batchProcessor import BatchProcessor
from autoFocus import AutoFocus
from checkWiFi import CheckWiFi

if __name__ == "__main__":
        
    # Create event loop and instantiate objects    
    app = QApplication(sys.argv)
    mw = MainWindow()
    lw = LogWindow()
    vs = PiVideoStream()
    ph = PrintHat() # Connect to printhat virtual port (this links the klipper software as well)
    st = SystemTemperatures(interval=10, alarm_temperature=55)
    ip = ImageProcessor()
    bp = BatchProcessor()
    af = AutoFocus(display=True)
    cf = CheckWiFi()

    # Start threads
    vs.start()
    ph.start()
    
    # Control signals, some explicitly of QueuedConnection type to prevent signal loss
    st.alarm.connect(lambda: ph.setFanPWM(1.0))
    st.alarmRemoved.connect(lambda: ph.setFanPWM(0.5))
    vs.frame.connect(ip.update, type=Qt.BlockingQueuedConnection)
    bp.setLightPWM.connect(ph.setLightPWM)
    bp.disableMotors.connect(ph.disableMotors)
    bp.gotoXY.connect(ph.gotoXY, type=Qt.QueuedConnection)
    bp.gotoX.connect(ph.gotoX, type=Qt.QueuedConnection)
    bp.gotoY.connect(ph.gotoY, type=Qt.QueuedConnection)
    bp.gotoZ.connect(ph.gotoZ, type=Qt.QueuedConnection)
    ph.positionReached.connect(bp.positionReached, type=Qt.QueuedConnection)
    bp.findDiaphragm.connect(ip.findDiaphragm, type=Qt.QueuedConnection)
    ip.diaphragmFound.connect(bp.diaphragmFound, type=Qt.QueuedConnection)
    ip.quality.connect(mw.imageQualityUpdate)
    bp.findWell.connect(ip.findWell, type=Qt.QueuedConnection)
    ip.wellFound.connect(bp.wellFound, type=Qt.QueuedConnection)
    bp.takeSnapshot.connect(vs.takeImage, type=Qt.QueuedConnection)
    vs.captured.connect(bp.snapshotTaken, type=Qt.QueuedConnection)
    bp.recordClip.connect(vs.recordClip, type=Qt.QueuedConnection)
    vs.captured.connect(bp.clipRecorded, type=Qt.QueuedConnection)
    bp.startAutoFocus.connect(af.start)
    af.focussed.connect(bp.focussedSlot)
    bp.stopCamera.connect(vs.stop, type=Qt.QueuedConnection)
    bp.startCamera.connect(vs.initStream, type=Qt.QueuedConnection)    
    mw.autoFocusButton.clicked.connect(lambda: af.start(mw.stageZTranslation.value()))
    af.setFocus.connect(mw.stageZTranslation.setValue)
    ph.positionReached.connect(af.positionReached, type=Qt.QueuedConnection)
    ip.quality.connect(af.imageQualityUpdate, type=Qt.DirectConnection)
    
    # Connect GUI signals
    mw.runButton.clicked.connect(bp.start)
    mw.runButton.released.connect(mw.disableButtons)    
    mw.breakButton.clicked.connect(ph.emergencyBreak)
    mw.restartButton.clicked.connect(ph.firmwareRestart)
    mw.homeXYZButton.clicked.connect(ph.homeXYZ)
    mw.stageOriginButton.clicked.connect(lambda: ph.gotoXY(0,0))
    mw.snapshotButton.clicked.connect(lambda: vs.takeImage())
#     mw.recordButton.clicked.connect(lambda: vs.recordClip(os.path.sep.join([settings.value('storage_main_path'), 'data','clip']), 10))
    mw.light.valueChanged.connect(lambda x: ph.setLightPWM(x/100))
    mw.stageXTranslation.valueChanged.connect(lambda x: ph.gotoX(x))
    mw.stageYTranslation.valueChanged.connect(lambda y: ph.gotoY(y))
    mw.stageZTranslation.valueChanged.connect(lambda z: ph.gotoZ(z))

    ip.signals.result.connect(mw.update) #, type=Qt.QueuedConnection)
    ph.signals.ready.connect(mw.enableButtons)
    ph.signals.ready.connect(st.resetAlarm)
    bp.signals.progress.connect(mw.updateProgressBar)
    bp.signals.ready.connect(mw.enableButtons)

    # Connect logging signals
    vs.postMessage.connect(lw.append)
    ph.signals.message.connect(lw.append)
    ph.signals.error.connect(lambda s: "error;{}".format(lw.append(s[2])))
    st.signals.message.connect(lw.append)
    st.signals.error.connect(lambda s: "error;{}".format(lw.append(s[2])))
    ip.signals.message.connect(lw.append)
    ip.signals.error.connect(lambda s: "error;{}".format(lw.append(s[2])))
    bp.signals.message.connect(lw.append)
    bp.signals.error.connect(lambda s: "error;{}".format(lw.append(s[2])))
    bp.setLogFileName.connect(lw.setLogFileName)
    af.postMessage.connect(lw.append)
    cf.postMessage.connect(lw.append)

    # Connect closing signals
    st.failure.connect(mw.close, type=Qt.QueuedConnection)
    bp.signals.finished.connect(mw.close, type=Qt.QueuedConnection)
    mw.signals.finished.connect(vs.stop, type=Qt.QueuedConnection)
    mw.signals.finished.connect(ip.stop, type=Qt.QueuedConnection)
    mw.signals.finished.connect(st.stop, type=Qt.QueuedConnection)
    mw.signals.finished.connect(bp.stop, type=Qt.QueuedConnection)
    mw.signals.finished.connect(ph.stop, type=Qt.QueuedConnection)
    mw.signals.finished.connect(lw.close, type=Qt.QueuedConnection)
    mw.signals.finished.connect(af.stop, type=Qt.QueuedConnection)

    # Start the show
    settings = QSettings("settings.ini", QSettings.IniFormat)
    lw.append("App started")
    vs.initStream()    
    vs.setStoragePath(settings.value('temp_folder'))
    mw.move(50,0)
    mw.resize(1500, 700)
    lw.move(50, 750)
    lw.resize(1500, 200)
    mw.show()
    lw.show()
    sys.exit(app.exec_())



    
