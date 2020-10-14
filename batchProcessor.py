"""@package docstring
batch processor implements QObject
@author Gert van Lagen, Robin Meekes, Jeroen Veen
"""
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import re
import time
import traceback
import numpy as np
from math import sqrt
from PyQt5.QtCore import QSettings, QObject, QTimer, QEventLoop, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QDialog, QFileDialog, QPushButton, QLabel, QSpinBox, QDoubleSpinBox, QVBoxLayout
from objectSignals import ObjectSignals


class Well:  
    def __init__(self, name, position, location, note=None, run=None, txt=None, plate=None):
        self.name = name  
        self.position = position
        self.location = location
        self.note = note
        self.run = run
        self.txt = txt
        self.plate = plate


class FocusDialog(QDialog):

    def __init__(self):
        super(FocusDialog, self).__init__()

        self.setWindowTitle("Focus dialog")
        self.instruction = QLabel("Please focus on the well content, adjust illumination intensity and press OK when done")
        self.stageZTranslation = QDoubleSpinBox(self)
        self.stageZTranslationTitle = QLabel("Z Translation")
        self.stageZTranslation.setSuffix("mm")
        self.stageZTranslation.setMinimum(0.0)
        self.stageZTranslation.setMaximum(50)
        self.stageZTranslation.setSingleStep(0.01)

        self.light = QSpinBox(self)
        self.lightTitle = QLabel("Light Intensity")
        self.light.setSuffix("%")
        self.light.setMinimum(0)
        self.light.setMaximum(100)
        self.light.setSingleStep(1)        

        self.okButton = QPushButton("OK")
        self.okButton.clicked.connect(self.okclicked)

        layout = QVBoxLayout()
        layout.addWidget(self.instruction)
        layout.addWidget(self.stageZTranslationTitle)
        layout.addWidget(self.stageZTranslation)
        layout.addWidget(self.lightTitle)
        layout.addWidget(self.light)
        layout.addWidget(self.okButton)
        self.setLayout(layout)

    @pyqtSlot()
    def okclicked(self):
        self.accept()
  
        
class BatchProcessor(QObject):
    '''
    Worker thread
    '''
    signals = ObjectSignals()
    setLightPWM = pyqtSignal(float) # control signal
    gotoXY = pyqtSignal(float, float, bool)
    gotoX = pyqtSignal(float, bool)
    gotoY = pyqtSignal(float, bool)
    findDiaphragm = pyqtSignal()
    disableMotors = pyqtSignal()
    findWell = pyqtSignal()
    computeSharpnessScore = pyqtSignal()
    rSnapshotTaken = pyqtSignal() # repeater signal
    rSharpnessScore = pyqtSignal() # repeater signal
    rWellFound = pyqtSignal() # repeat signal
    rDiaphragmFound = pyqtSignal() # repeat signal
    rClipRecorded = pyqtSignal() # repeat signal
    rPositionReached = pyqtSignal() # repeat signal
    gotoZ = pyqtSignal(float)
    takeSnapshot = pyqtSignal(str)
    recordClip = pyqtSignal(str, int)
    setLogFileName = pyqtSignal(str)
    stopCamera = pyqtSignal()
    startCamera = pyqtSignal()

    def __init__(self, batch_file_name):
        super().__init__()

        self.settings = QSettings("settings.ini", QSettings.IniFormat)        
        self.loadSettings()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.run)
        self.isInterruptionRequested = False
        self.foundDiaphragmLocation = None
        self.foundWellLocation = None
        self.lightLevel = 1.0 # initial value, will be set later by user
        self.adaptXY = False # Flag XY position update after initial adaption
        self.sharpnessScore = 0


    def loadSettings(self):
        self.msg("info;loading settings from {:s}".format(self.settings.fileName()))
        self.resolution = float(self.settings.value('camera/resolution_in_px_per_mm'))


    def loadBatchSettings(self):
        self.msg("info;loading batch settings from {:s}".format(self.batch_settings.fileName()))

        # load run info
        self.run_id = self.batch_settings.value('run/id')
        self.run_note = self.batch_settings.value('run/note')
        t = time.strptime(self.batch_settings.value('run/duration')[1],'%H:%M:%S')
        days = int(self.batch_settings.value('run/duration')[0].split('d')[0])
        self.run_duration_s = ((24*days + t.tm_hour)*60 + t.tm_min)*60 + t.tm_sec
        t = time.strptime(self.batch_settings.value('run/wait'),'%H:%M:%S')
        self.run_wait_s = (t.tm_hour*60 + t.tm_min)*60 + t.tm_sec
        s = str(self.batch_settings.value('run/shutdown'))
        self.shutdown = s.lower() in ['true', '1', 't', 'y', 'yes']
        s = str(self.batch_settings.value('run/snapshot'))
        self.snapshot = s.lower() in ['true', '1', 't', 'y', 'yes']
        s = str(self.batch_settings.value('run/videoclip'))
        self.videoclip = s.lower() in ['true', '1', 't', 'y', 'yes']
        self.videoclip_length = int(self.batch_settings.value('run/clip_length'))
        self.accepted_location_error_mm = float(self.batch_settings.value('run/accepted_location_error_mm'))
        
        # load well-plate dimensions
        self.plate_note = self.batch_settings.value('plate/note')
        plate_nr_of_cols = int(self.batch_settings.value('plate/columns'))
        plate_nr_of_rows = int(self.batch_settings.value('plate/rows'))
        plate_length = float(self.batch_settings.value('plate/length'))
        plate_width = float(self.batch_settings.value('plate/width'))
        plate_p1 = float(self.batch_settings.value('plate/p1'))
        plate_p2 = float(self.batch_settings.value('plate/p2'))
        plate_p3 = float(self.batch_settings.value('plate/p3'))
        plate_p4 = float(self.batch_settings.value('plate/p4'))
        self.first_well_location = [plate_p1, plate_p2]

        # load the wells to process
        nr_of_wells = self.batch_settings.beginReadArray("wells")
        self.wells = []
        for i in range(0, nr_of_wells):
            self.batch_settings.setArrayIndex(i)
            well_id = self.batch_settings.value('id')
            well_note = self.batch_settings.value('note')
            r = re.split('(\d+)', well_id)
            row = ord(r[0].lower()) - 96
            col = int(r[1])
            location_mm = [plate_p1 + (col-1)*plate_p2, plate_p3 + (row-1)*plate_p4, 0]
            self.wells.append(Well(name=well_id, position=[row, col], location=location_mm))


    def msg(self, text):
        if text:
            text = self.__class__.__name__ + ";" + str(text)
            print(text)
            self.signals.message.emit(text)
            
            
    @pyqtSlot()
    def start(self):        
        # open storage folder
        dlg = QFileDialog()
        self.storage_path = QFileDialog.getExistingDirectory(dlg, 'Open storage folder', '/media/pi/', QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)
        if self.storage_path == "" or self.storage_path is None:
            return

        # open batch definition file
        dlg = QFileDialog()
        self.batch_file_name = QFileDialog.getOpenFileName(dlg, 'Open batch definition file', os.getcwd(), "Ini file (*.ini)")[0]
        if self.batch_file_name == "" or self.batch_file_name is None:
            return
        self.batch_settings = QSettings(self.batch_file_name, QSettings.IniFormat)
        self.loadBatchSettings()

        # update storage path
        self.storage_path = os.path.sep.join([self.storage_path, self.run_id])
        if not os.path.exists(self.storage_path):
            os.makedirs(self.storage_path)        
        
        # copy batch definition file to storage folder
        os.system('cp {:s} {:s}'.format(self.batch_file_name, self.storage_path))

        log_file_name = os.path.sep.join([self.storage_path, self.run_id + ".log"])
        self.setLogFileName.emit(log_file_name)

        # start-up recipe
        self.msg("info;plate note: {:s}".format(self.plate_note))
        self.msg("info;run note: {:s}".format(self.run_note))
        self.msg("info;{:d} wells found in {:s}".format(len(self.wells),self.batch_settings.fileName()))
        self.msg("info;{:s} run during {:d}s with {:d}s interleave".format(self.run_id, self.run_duration_s, self.run_wait_s))

        self.setLightPWM.emit(self.lightLevel)
        self.startCamera.emit()
        self.msg("info;goto first well")
        self.gotoXY.emit(self.wells[0].location[0], self.wells[0].location[1], True)
        self.wait_signal(self.rPositionReached, 10000)
        
        # Let the user set the z-location manually by moving to first well and open dialog
        dialog = FocusDialog()
        dialog.stageZTranslation.valueChanged.connect(lambda z: self.gotoZ.emit(z))
        dialog.light.valueChanged.connect(lambda x: self.setLightPWM.emit(x/100))
        dialog.exec_()
        z = dialog.stageZTranslation.value()
        z = round(z,3)
        self.lightLevel = round(dialog.light.value()/100,3)
        self.msg("info;user set focus at z={:.3f}".format(z))
        self.msg("info;user set light intensity at {:.3f}".format(self.lightLevel))
        for well in self.wells:
            well.location[2] = z # copy z, later on we may do autofocus per well
            
        # go back home and find the diaphragm
        self.msg("info;goto home")
        self.gotoZ.emit(z)
        self.gotoXY.emit(0,0,False) # goto to system origin
        self.wait_signal(self.rPositionReached, 10000)
        self.msg("info;find diaphragm")
        self.findDiaphragm.emit()
        self.wait_signal(self.rDiaphragmFound, 10000)
        
        # start timer
        self.start_time_s = time.time()
        self.adaptXY = True # Flag XY position update after initial adaption
        self.timer.start(0)
        
            
    def run(self):
        ''' Timer call back function, als initiates next one-shot 
        '''
        start_run_time_s = time.time()
        self.startCamera.emit()
        for well in self.wells:
            x, y, z = tuple(well.location)
            self.msg("info;gauging well {:s} at ({:.3f}, {:.3f}, {:.3f})".format(well.name, x,y,z))
            self.setLightPWM.emit(self.lightLevel)

            # split goto xyz command 
            self.gotoX.emit(x,True)
            self.wait_signal(self.rPositionReached, 10000)
            self.gotoY.emit(y,True)
            self.wait_signal(self.rPositionReached, 10000)
            self.gotoZ.emit(z)
            self.wait_signal(self.rPositionReached, 10000)
            self.wait_ms(1000)
            
            # adapt from initial guess to actual well position
            if self.adaptXY:
                for i in range(0,10):
                    if self.isInterruptionRequested:                
                        return                

                    self.findWell.emit()
                    self.wait_signal(self.rWellFound, 10000)
                    # wellFound
                    if self.foundWellLocation is not None and self.foundDiaphragmLocation is not None: 
                        locationError = sqrt( (self.foundWellLocation[0] - self.foundDiaphragmLocation[0])**2 + (self.foundWellLocation[1] - self.foundDiaphragmLocation[1])**2 )
                        self.msg("info;location error = {:.3f} mm".format(locationError/self.resolution))
                        # check convergence criterion 
                        if locationError/self.resolution < self.accepted_location_error_mm:
                            # save last location
                            well.location[0] = round(x,3)
                            well.location[1] = round(y,3)
                            break
                        else:
                        # adapt planar position a bit
                            x -= (self.foundWellLocation[0] - self.foundDiaphragmLocation[0])/(self.resolution)
                            y -= (self.foundWellLocation[1] - self.foundDiaphragmLocation[1])/(self.resolution)
                            self.gotoXY.emit(x,y,True)
                            self.wait_signal(self.rPositionReached, 10000) # mostly, signal returns before event loop in wait_signal is even started

                    else:
                        self.msg("error;cannot find well {:s} at ({:.3f}, {:.3f})".format(well.name, x,y))
                        return
            self.msg("info;well {:s} at ({:.3f}, {:.3f})".format(well.name, x,y))
            
            # TODO: adapt focus
            self.computeSharpnessScore.emit()
            self.wait_signal(self.rSharpnessScore, 10000)
##            well.sharpnessScore = self.rSharpnessScore
            well.sharpnessScore = round(self.sharpnessScore,3)
            
            
            prefix = os.path.sep.join([self.storage_path, well.name, str(well.position) + "_" + str(well.location) + "_" + str(well.sharpnessScore)])
            if self.snapshot:
                self.takeSnapshot.emit(prefix)
                self.wait_signal(self.rSnapshotTaken) # snapshot taken
            if self.videoclip and self.videoclip_length > 0:
                self.recordClip.emit(prefix, self.videoclip_length)
                self.wait_signal(self.rClipRecorded) # clip recorded
                
        
        # TODO: make the behavior more clear and robust
        self.adaptXY = False # this is a lame workaround to let the adaptation occur only once,                 
                
        # Wrapup current round of acquisition     
        self.setLightPWM.emit(0.00)
        self.stopCamera.emit()
        elapsed_total_time_s = time.time() - self.start_time_s
        elapsed_run_time_s = time.time() - start_run_time_s
        self.msg("info;single run time={:.1f}s".format(elapsed_run_time_s))
        self.msg("info;total run time={:.1f}s".format(elapsed_total_time_s))

        progress_percentage = int(100*elapsed_total_time_s/self.run_duration_s)
        self.signals.progress.emit(progress_percentage)
        self.msg("info;progress={:d}%".format(progress_percentage))
        
        # check if we still have time to do another round
        if elapsed_total_time_s + self.run_wait_s < self.run_duration_s:
            self.timer.setInterval(self.run_wait_s*1000)
            self.msg("info;wait for {:.1f} s".format(self.run_wait_s))
        else:
            self.timer.stop()
            self.signals.ready.emit()
            self.msg("info;run finalized")
            if self.shutdown:
                self.msg("info;emitting finished")
                self.signals.finished.emit()
       

    def requestInterruption(self):
        self.isInterruptionRequested = True                    

    @pyqtSlot(np.ndarray)
    def diaphragmFound(self, location):
        self.foundDiaphragmLocation = location
        self.msg("info;diaphragmFound signal received")
        self.rDiaphragmFound.emit()

    @pyqtSlot(np.ndarray)
    def wellFound(self, location):
        self.foundWellLocation = location
        self.msg("info;wellFound signal received")
        self.rWellFound.emit()

    @pyqtSlot(float)
    def setSharpnessScore(self, score):
        self.sharpnessScore = score
        self.msg("info;sharpnessScore signal received")        
        self.rSharpnessScore.emit()
        
    @pyqtSlot()
    def snapshotTaken(self):
        self.msg("info;snapshotTaken signal received")
        self.rSnapshotTaken.emit()

    @pyqtSlot()
    def clipRecorded(self):
        self.msg("info;clipRecorded signal received")
        self.rClipRecorded.emit()

    @pyqtSlot()
    def positionReached(self):
        self.msg("info;positionReached signal received")
        self.rPositionReached.emit()
        
    @pyqtSlot()
    def stop(self):
        self.msg("info;stopping")
        self.requestInterruption()        
        if self.timer.isActive():
            self.timer.stop()
        self.signals.finished.emit()
       

    def wait_ms(self, timeout):
        ''' Block loop until timeout (ms) elapses.
        '''        
        loop = QEventLoop()
        QTimer.singleShot(timeout, loop.exit)
        loop.exec_()
        

    def wait_signal(self, signal, timeout=100):
        ''' Block loop until signal emitted, or timeout (ms) elapses.
        '''
        loop = QEventLoop()
        signal.connect(loop.quit) # only quit is a slot of QEventLoop
        QTimer.singleShot(timeout, loop.exit)
        loop.exec_()
        

