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
import smtplib, ssl
from webdav3.client import Client
from webdav3.exceptions import WebDavException
from math import sqrt
from PyQt5.QtCore import QSettings, QObject, QTimer, QEventLoop, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QDialog, QFileDialog, QPushButton, QLabel, QSpinBox, QDoubleSpinBox, QVBoxLayout, QGridLayout
from objectSignals import ObjectSignals
import subprocess


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


class ManualPositioningDialog(QDialog):

    def __init__(self, x=0,y=0,z=0):
        super(ManualPositioningDialog, self).__init__()

        self.setWindowTitle("Manual positioning")
        self.instruction = QLabel("Please focus on well A1, adjust illumination intensity and press OK when done")

        self.light = QSpinBox(self)
        self.lightTitle = QLabel("light")
        self.light.setSuffix("%")
        self.light.setMinimum(0)
        self.light.setMaximum(100)
        self.light.setSingleStep(1)
        self.stageXTranslation = QDoubleSpinBox(self)
        self.stageXTranslationTitle = QLabel("X Translation")
        self.stageXTranslation.setSuffix("mm")
        self.stageXTranslation.setMinimum(0.0)
        self.stageXTranslation.setMaximum(100)
        self.stageXTranslation.setSingleStep(0.1)
        self.stageXTranslation.setValue(x)
        self.stageYTranslation = QDoubleSpinBox(self)
        self.stageYTranslationTitle = QLabel("Y Translation")
        self.stageYTranslation.setSuffix("mm")
        self.stageYTranslation.setMinimum(0.0)
        self.stageYTranslation.setMaximum(100)
        self.stageYTranslation.setSingleStep(0.1)
        self.stageYTranslation.setValue(y)
        self.stageZTranslation = QDoubleSpinBox(self)
        self.stageZTranslationTitle = QLabel("Z Translation")
        self.stageZTranslation.setSuffix("mm")
        self.stageZTranslation.setMinimum(0.0)
        self.stageZTranslation.setMaximum(50)
        self.stageZTranslation.setSingleStep(0.01)
        self.stageZTranslation.setValue(z)

        self.okButton = QPushButton("OK")
        self.okButton.clicked.connect(self.okclicked)

        layout = QVBoxLayout()
        layout.addWidget(self.instruction)
        layout2 = QGridLayout()
        layout2.addWidget(self.stageXTranslationTitle,0,0)
        layout2.addWidget(self.stageXTranslation,0,1)
        layout2.addWidget(self.stageYTranslationTitle,1,0)
        layout2.addWidget(self.stageYTranslation,1,1)
        layout2.addWidget(self.stageZTranslationTitle,2,0)
        layout2.addWidget(self.stageZTranslation,2,1)
        layout.addLayout(layout2)
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
    gotoXY = pyqtSignal(float, float, bool) # boolean indicate relative to stage origin or not
    gotoX = pyqtSignal(float, bool)
    gotoY = pyqtSignal(float, bool)
    gotoZ = pyqtSignal(float)
    findDiaphragm = pyqtSignal()
    disableMotors = pyqtSignal()
    findWell = pyqtSignal()
#     computeSharpnessScore = pyqtSignal()
    rSnapshotTaken = pyqtSignal() # repeater signal
#     rSharpnessScore = pyqtSignal() # repeater signal
    rWellFound = pyqtSignal() # repeat signal
    rDiaphragmFound = pyqtSignal() # repeat signal
    rClipRecorded = pyqtSignal() # repeat signal
    rPositionReached = pyqtSignal() # repeat signal
    takeSnapshot = pyqtSignal(str)
    recordClip = pyqtSignal(str, int)
    setLogFileName = pyqtSignal(str)
    stopCamera = pyqtSignal()
    startCamera = pyqtSignal()
    startAutoFocus = pyqtSignal(float)
    focussed = pyqtSignal() # repeater signal    

    def __init__(self):
        super().__init__()

        self.settings = QSettings("settings.ini", QSettings.IniFormat) 
        self.conn_settings = QSettings("connections.ini", QSettings.IniFormat)
        self.loadSettings()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.run)
        self.isInterruptionRequested = False
        self.foundDiaphragmLocation = None
        self.foundWellLocation = None
        self.lightLevel = 1.0 # initial value, will be set later by user
#         self.sharpnessScore = 0


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
        
        # load well-plate dimensions and compute well locations
        self.plate_note = self.batch_settings.value('plate/note')
        self.nr_of_columns = int(self.batch_settings.value('plate/nr_of_columns'))
        self.nr_of_rows = int(self.batch_settings.value('plate/nr_of_rows'))
        self.A1_to_side_offset = float(self.batch_settings.value('plate/A1_to_side_offset'))
        self.column_well_spacing = float(self.batch_settings.value('plate/column_well_spacing'))
        self.A1_to_top_offset = float(self.batch_settings.value('plate/A1_to_top_offset'))
        self.row_well_spacing = float(self.batch_settings.value('plate/row_well_spacing'))

        self.computeWellLocations()


    def computeWellLocations(self):
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
            location_mm = [round(self.A1_to_side_offset + (col-1)*self.column_well_spacing, 2), \
                           round(self.A1_to_top_offset  + (row-1)*self.row_well_spacing, 2), \
                           0]
            self.wells.append(Well(name=well_id, position=[row, col], location=location_mm))
        self.batch_settings.endArray() # close array, also required when opening!


    def msg(self, text):
        if text:
            text = self.__class__.__name__ + ";" + str(text)
            print(text)
            self.signals.message.emit(text)
            
            
    @pyqtSlot()
    def start(self):        
        # open storage folder
        # dlg = QFileDialog()
        # self.local_storage_path = QFileDialog.getExistingDirectory(dlg, 'Open storage folder', '/media/pi/', QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)
        # if self.local_storage_path == "" or self.local_storage_path is None:
        #     return
        self.local_storage_path = os.path.sep.join([os.getcwd(), self.settings.value('temp_folder')])

        # clear temporary storage path
        os.system('rm {:s}'.format(os.path.sep.join([self.local_storage_path, "*.*"])))

        # open batch definition file
        dlg = QFileDialog()
        self.batch_file_name = QFileDialog.getOpenFileName(dlg, 'Open batch definition file', os.getcwd(), "Ini file (*.ini)")[0]
        if self.batch_file_name == "" or self.batch_file_name is None:
            return
        self.batch_settings = QSettings(self.batch_file_name, QSettings.IniFormat)
        self.loadBatchSettings()

        # copy batch definition file to storage folder
        os.system('cp {:s} {:s}'.format(self.batch_file_name, self.local_storage_path))

        note_file_name = os.path.sep.join([self.local_storage_path, self.run_id + ".log"])
        self.setLogFileName.emit(note_file_name)

        # set up connectivity
        if self.batch_settings.contains('connections/storage'):
            if self.batch_settings.value('connections/storage') == 'rclone':
                # rclone to path provided in connections.ini file
                self.server_storage_path = self.conn_settings.value('rclone/storage_path') + ':' + self.batch_settings.value('run/id')

                try:
                    subprocess.run(["rclone", "mkdir", self.server_storage_path])
                    subprocess.run(["rclone", "copy", "--no-traverse", self.local_storage_path, self.server_storage_path])
                    # create directory structure on server
                    for well in self.wells:
                        subprocess.run(["rclone", "mkdir", os.path.sep.join([self.server_storage_path, well.name])])                    
                except Exception as err:
                    self.msg("error;type: {}, args: {}".format(type(err), err.args))
                self.msg("info;rclone connection to {}".format(self.server_storage_path))

            elif self.batch_settings.value('connections/storage') == 'wbedav':
                # open WebDAV connection to server, using credentials from connections.ini file
                self.server_storage_path = os.path.sep.join([self.conn_settings.value('webdav/storage_path'), self.run_id])

                options = { 'webdav_hostname': self.conn_settings.value('webdav/hostname'),
                            'webdav_login': self.conn_settings.value('webdav/login'),
                            'webdav_password': self.conn_settings.value('webdav/password') }
                try:
                    self.webdav_client = Client(options)
                    self.webdav_client.mkdir(self.server_storage_path)
                    self.webdav_client.push(local_directory=self.local_storage_path, remote_directory=self.server_storage_path)
                except WebDavException as err:
                    self.msg("error;type: {}, args: {}".format(type(err), err.args))
                self.msg("info;WebDAV connection to {}: {}".format(self.conn_settings.value('webdav/hostname'),
                                                                   self.server_storage_path))
            else:
                self.msg("error; unknown remote")
                
        # create temporary image storage path
        self.image_storage_path = os.path.sep.join([self.local_storage_path,'img'])
        if not os.path.exists(self.image_storage_path):
            os.makedirs(self.image_storage_path)
        
        # start-up recipe
        self.msg("info;plate note: {:s}".format(self.plate_note))
        self.msg("info;run note: {:s}".format(self.run_note))
        self.msg("info;{:d} wells found in {:s}".format(len(self.wells),self.batch_settings.fileName()))
        self.msg("info;{:s} run during {:d}s with {:d}s interleave".format(self.run_id, self.run_duration_s, self.run_wait_s))

        self.setLightPWM.emit(0.2)
        self.startCamera.emit()
        self.msg("info;goto first well")
        x = self.wells[0].location[0]
        y = self.wells[0].location[1]
        z = 33 #self.wells[0].location[2]
        self.gotoXY.emit(x,y, True)
        self.wait_signal(self.rPositionReached, 10000)
        self.gotoZ.emit(z)
        self.wait_signal(self.rPositionReached, 10000)
        
        # Let the user set the x,y,z-location manually by moving to first well and open dialog
        dialog = ManualPositioningDialog(x,y,z)
        dialog.stageXTranslation.valueChanged.connect(lambda x: self.gotoX.emit(x, True))
        dialog.stageYTranslation.valueChanged.connect(lambda y: self.gotoY.emit(y, True))
        dialog.stageZTranslation.valueChanged.connect(lambda z: self.gotoZ.emit(z))
        dialog.light.valueChanged.connect(lambda x: self.setLightPWM.emit(x/100))
        dialog.exec_()
        self.A1_to_side_offset = round(dialog.stageXTranslation.value(), 3)
        self.A1_to_top_offset = round(dialog.stageYTranslation.value(), 3)
        z = round(dialog.stageZTranslation.value(), 3)
        self.lightLevel = round(dialog.light.value()/100,3)
        self.msg("info;user set well A1 at (x,y)=({:.3f},{:.3f})".format(self.A1_to_side_offset, self.A1_to_top_offset))
        self.msg("info;user set focus at z={:.3f}".format(z))
        self.msg("info;user set light intensity at {:.3f}".format(self.lightLevel))

        # recompute the well locations in our batch
        self.computeWellLocations()
        
        for well in self.wells:
            well.location[2] = z # copy z, later on we may do autofocus per well

        # start timer
        self.prev_note_nr = 0 # for logging
        self.start_time_s = time.time()
        self.timer.start(0)
        
            
    def run(self):
        ''' Timer call back function, als initiates next one-shot 
        '''
        start_run_time_s = time.time()
        self.setLightPWM.emit(self.lightLevel)
        self.startCamera.emit()
        self.msg("info;go home")
        self.gotoXY.emit(0, 0, False)
        self.wait_signal(self.rPositionReached, 10000)
        self.msg("info;goto first well")
        self.gotoXY.emit(self.wells[0].location[0], self.wells[0].location[1], True)
        self.wait_signal(self.rPositionReached, 10000)
#         self.wait_ms(60000) # wait for camera to adjust to light
        
        for well in self.wells:
            self.msg("info;gauging well {:s})".format(well.name))
                     
            # split goto xyz command 
            self.gotoX.emit(well.location[0],True)
            self.wait_signal(self.rPositionReached, 10000)
            self.gotoY.emit(well.location[1],True)
            self.wait_signal(self.rPositionReached, 10000)
            self.gotoZ.emit(well.location[2])
            self.wait_signal(self.rPositionReached, 10000)
            
            # autofocus
            if self.batch_settings.value('run/autofocus', False, type=bool):
                self.new_z = 0
                self.startAutoFocus.emit(well.location[2])
                self.wait_signal(self.focussed, 100000)
                if self.new_z != 0:
                    well.location[2] = self.new_z
                else:
                    # focus failed, so return to initial z position
                    self.gotoZ.emit(well.location[2])
                    self.wait_signal(self.rPositionReached, 10000)
                    
            try:
                # clear temporary storage path
                os.system('rm {:s}'.format(os.path.sep.join([self.image_storage_path, "*.*"])))
            except err:
                traceback.print_exc()
                self.signals.error.emit((type(err), err.args, traceback.format_exc()))            
        
            # take snapshot or video
            self.wait_ms(2000)
            prefix = os.path.sep.join([self.image_storage_path, str(well.position) + "_" + str(well.location)])
            
            if self.snapshot:
                self.takeSnapshot.emit(prefix)
                self.wait_signal(self.rSnapshotTaken) # snapshot taken
            if self.videoclip and self.videoclip_length > 0:
                self.recordClip.emit(prefix, self.videoclip_length)
                self.wait_signal(self.rClipRecorded) # clip recorded

            # push capture to remote
            temp_path = os.path.sep.join([self.server_storage_path,well.name])
            if self.batch_settings.contains('connections/storage'):
                try:
                    if self.batch_settings.value('connections/storage') == 'rclone':
                        subprocess.run(["rclone", "copy", "--no-traverse", self.local_image_storage_path, temp_path])
                    elif self.batch_settings.value('connections/storage') == 'wbedav':
                        self.webdav_client.push(local_directory=self.local_image_storage_path,
                                                remote_directory=temp_path)
                except Exception as err:
                    self.postMessage.emit("{}: error; type: {}, args: {}".format(self.__class__.__name__, type(err), err.args))
                
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

        # send a notification
        note_nr = int(progress_percentage/10)
        if note_nr != self.prev_note_nr:
            self.prev_note_nr = note_nr
            
            message = """Subject: Progress = {}% \n\n Still {} s left""".format(progress_percentage, int(self.run_duration_s - elapsed_total_time_s))
            # do something fancy here in future: https://realpython.com/python-send-email/#sending-fancy-emails
            self.sendNotification(message)           
                
        
        # check if we still have time to do another round
        if elapsed_total_time_s + self.run_wait_s < self.run_duration_s:
            self.timer.setInterval(self.run_wait_s*1000)
            self.msg("info;wait for {:.1f} s".format(self.run_wait_s))
        else:
            self.timer.stop()
            self.signals.ready.emit()
            self.msg("info;run finalized")
            message = """Subject: run finalized"""
            # do something fancy here in future: https://realpython.com/python-send-email/#sending-fancy-emails
            self.sendNotification(message)            
            if self.shutdown:
                self.msg("info;emitting finished")
                self.signals.finished.emit()
                
    def sendNotification(self, message):
        port = 465  # For SSL
        context = ssl.create_default_context()  # Create a secure SSL context
        try:
            with smtplib.SMTP_SSL("smtp.gmail.com", port, context=context) as server:
                login = self.conn_settings.value('smtp/login')
                password = self.conn_settings.value('smtp/password')
                server.login(login, password)
                server.sendmail(self.conn_settings.value('smtp/login'), \
                                self.conn_settings.value('subscriber/email'), \
                                message)
        except Exception as err:
            traceback.print_exc()
            self.signals.error.emit((type(err), err.args, traceback.format_exc()))

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

#     @pyqtSlot(float)
#     def setSharpnessScore(self, score):
#         self.sharpnessScore = score
#         self.msg("info;sharpnessScore signal received")        
#         self.rSharpnessScore.emit()
        
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
        
    @pyqtSlot(float)
    def focussedSlot(self, val):
        self.new_z = val
        self.focussed.emit()        
        
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
        

