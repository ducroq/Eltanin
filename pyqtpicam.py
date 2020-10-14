#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Based on
 https://picamera.readthedocs.io/en/latest/
 https://picamera.readthedocs.io/en/release-1.13/api_camera.html
 https://www.raspberrypi.org/documentation/hardware/camera/
    
version June 2020

"""
import os
import cv2
import time
import warnings
import traceback
import numpy as np
from fps import FPS
from picamera import PiCamera
from objectSignals import ObjectSignals
from picamera.array import PiRGBArray, PiYUVArray, PiArrayOutput
from PyQt5.QtCore import QThread, QSettings, pyqtSlot, QTimer, QEventLoop, pyqtSignal


class PiYArray(PiArrayOutput):
    """
    Produces a 2-dimensional Y only array from a YUV capture.
    Does not seem faster than PiYUV array...
    """
    def __init__(self, camera, size=None):
        super(PiYArray, self).__init__(camera, size)
        self.fwidth, self.fheight = raw_frame_size(self.size or self.camera.resolution)
        self.y_len = self.fwidth * self.fheight
##        uv_len = (fwidth // 2) * (fheight // 2)
##        if len(data) != (y_len + 2 * uv_len):
##            raise PiCameraValueError(
##            'Incorrect buffer length for frame_size %dx%d' % (width, height))

    def flush(self):
        super(PiYArray, self).flush()
        a = np.frombuffer(self.getvalue()[:self.y_len], dtype=np.uint8)
        self.array = a[:self.y_len].reshape((self.fheight, self.fwidth))
        

## PiVideoStream class streams camera images to a numpy array
class PiVideoStream(QThread):
    signals = ObjectSignals()
    snapshotTaken = pyqtSignal()
    clipRecorded = pyqtSignal()
    camera = PiCamera()

    ## @param ins is the number of instances created. This may not exceed 1.
    ins = 0
    
    def __init__(self):
        super().__init__()

        ## Instance limiter. Checks if an instance exists already. If so, it deletes the current instance.
        if PiVideoStream.ins >= 1:
            del self
            self.msg("error; multiple instances of {:s} created, while only 1 instance is allowed".format(__class__.__name__))
            return        
        try:
            PiVideoStream.ins+=1
        except Exception as err:
            traceback.print_exc()
            self.signals.error.emit((type(err), err.args, traceback.format_exc()))
        
        warnings.filterwarnings('default', category=DeprecationWarning)
        self.settings = QSettings("settings.ini", QSettings.IniFormat)
        self.loadSettings()
        self.initStream()
            
            
    def loadSettings(self):
        self.msg("info; loading camera settings from " + self.settings.fileName())
        frame_size_str = self.settings.value('camera/frame_size')
        (width, height) = frame_size_str.split('x')
        self.camera.resolution = raw_frame_size((int(width), int(height)))
        self.camera.framerate = int(self.settings.value('camera/frame_rate'))
        self.camera.image_effect = self.settings.value('camera/effect')
        self.camera.shutter_speed = int(self.settings.value('camera/shutter_speed'))
        self.camera.iso = int(self.settings.value('camera/iso')) # should force unity analog gain
        # set parameters for speed
        frame_size_str = self.settings.value('image_frame_size')
        (width, height) = frame_size_str.split('x')
        self.image_size = (int(width), int(height))        
        self.camera.video_denoise = False
        self.monochrome = True
        self.use_video_port = True
        # dunno if setting awb mode manually is really useful
##        self.camera.awb_mode = 'off'
##        self.camera.awb_gains = 5.0
##        self.camera.meter_mode = 'average'
##        self.camera.exposure_mode = 'auto'  # 'sports' to reduce motion blur, 'off'after init to freeze settings

    @pyqtSlot()
    def initStream(self):
        # Initialize the camera stream
        if self.isRunning():
            # in case init gets called, while thread is running
            self.msg("info; video stream is running already")
        else:
            # init camera and open stream
            if self.monochrome:
    ##            self.camera.color_effects = (128,128) # return monochrome image, not required if we take Y frame only.
                self.rawCapture = PiYArray(self.camera, size=self.camera.resolution)
                self.stream = self.camera.capture_continuous(self.rawCapture, 'yuv', self.use_video_port)
            else:
                self.rawCapture = PiRGBArray(self.camera, size=self.camera.resolution)
                self.stream = self.camera.capture_continuous(self.rawCapture, 'bgr', self.use_video_port)
            # allocate memory 
            self.frame = np.empty(self.camera.resolution + (1 if self.monochrome else 3,), dtype=np.uint8)
            # restart thread
            self.start()
            self.wait_ms(1000)
            self.msg("info; video stream initialized with frame size = " + str(self.camera.resolution))


    @pyqtSlot()
    def run(self):
        try:
            self.fps = FPS().start()
            for f in self.stream:
                if self.isInterruptionRequested():
                    self.signals.finished.emit()
                    return                   
##                self.rawCapture.truncate(0)  # Depricated: clear the stream in preparation for the next frame
                self.rawCapture.seek(0) 
                self.frame = f.array # grab the frame from the stream
                self.signals.ready.emit()
                self.signals.result.emit(cv2.resize(self.frame, self.image_size)) # resize to speed up processing
                self.fps.update()
        except Exception as err:
            traceback.print_exc()
            self.signals.error.emit((type(err), err.args, traceback.format_exc()))

        
    @pyqtSlot()
    def stop(self):
        self.msg("info; stopping")
        if self.isRunning():
            self.requestInterruption()
            self.wait_signal(self.signals.finished, 2000)        
        self.fps.stop()
        self.quit()
##        self.frame.fill(0) # clear frame, not allowed since frame is read-only?
##        self.signals.ready.emit()
##        self.signals.result.emit(np.zeros(self.image_size)) # could produce an information image here
        self.msg("info; video stream stopped, approx. acquisition speed: {:.2f} fps".format(self.fps.fps()))
        

    def msg(self, text):
        if text:
            text = self.__class__.__name__ + ";" + str(text)
            print(text)
            self.signals.message.emit(text)
            
        
    @pyqtSlot()
    def changeCameraSettings(self, frame_size=(640,480), frame_rate=24, format='bgr', effect='none', use_video_port=False, monochrome=True):
        '''
        The use_video_port parameter controls whether the camera’s image or video port is used to capture images.
        It defaults to False which means that the camera’s image port is used. This port is slow but produces better quality pictures.
        '''
        self.stop()
        self.camera.resolution = raw_frame_size(frame_size)
        self.camera.framerate = frame_rate
        self.camera.image_effect = effect
        self.use_video_port = use_video_port
        self.monochrome = monochrome
        self.initStream()


    def wait_ms(self, timeout):
        ''' Block loop until timeout (ms) elapses.
        '''        
        loop = QEventLoop()
        QTimer.singleShot(timeout, loop.exit)
        loop.exec_()
        

    def wait_signal(self, signal, timeout=1000):
        ''' Block loop until signal emitted, or timeout (ms) elapses.
        '''
        loop = QEventLoop()
        signal.connect(loop.quit) # only quit is a slot of QEventLoop
        QTimer.singleShot(timeout, loop.exit)
        loop.exec_()


    @pyqtSlot(str)
    def snapshot(self, filename_prefix=None):
        # open path
        (head, tail) = os.path.split(filename_prefix)
        if not os.path.exists(head):
            os.makedirs(head)
        filename = os.path.sep.join([head, '{:016d}_'.format(round(time.time() * 1000)) + tail + '.png'])

        # write image
        self.wait_signal(self.signals.ready, 5000) # wait for first frame to be shot
        cv2.imwrite(filename, self.frame)
        self.msg("info; image written to " + filename)
                

    @pyqtSlot(str, int)
    def recordClip(self, filename_prefix=None, duration=10):
        # open path
        (head, tail) = os.path.split(filename_prefix)
        if not os.path.exists(head):
            os.makedirs(head)
        filename = os.path.sep.join([head, '{:016d}_'.format(round(time.time() * 1000)) + tail + '.avi'])

        self.msg("TODO; changing camera settings may get the process killed after several hours, probably better to open the stream in video resolution from the start if the videorecording is required!")

        # set video clip parameters
        frame_size_str = self.settings.value('camera/clip_frame_size')
        frame_size_str.split('x')
        frame_size = raw_frame_size((int(frame_size_str.split('x')[0]),
                                     int(frame_size_str.split('x')[1])))
        frame_rate = int(self.settings.value('camera/clip_frame_rate'))
        self.changeCameraSettings(frame_size=frame_size, frame_rate=frame_rate, use_video_port=True, monochrome=False)

        # define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(filename, fourcc, frame_rate, frame_size)
        self.msg("info; start recording video to " + filename)
        
        # write file        
        for i in range(0,duration*frame_rate):
            self.signals.progress.emit(int(100*i/(duration*frame_rate-1)))
            self.wait_signal(self.signals.ready, 1000)
            if self.frame is not None:
                out.write(self.frame)
                
        # close   
        out.release()
        self.msg("info; recording done")

##        self.camera.start_recording(filename)
##        self.camera.wait_recording(duration)
##        self.camera.stop_recording()
        
        # revert to original parameters
        self.loadSettings()
        self.initStream()
        self.clipRecorded.emit()
        
   
def raw_frame_size(frame_size, splitter=False):
    """
    Round a (width, height) tuple up to the nearest multiple of 32 horizontally
    and 16 vertically (as this is what the Pi's camera module does for
    unencoded output).
    """
    width, height = frame_size
    if splitter:
        fwidth = (width + 15) & ~15
    else:
        fwidth = (width + 31) & ~31
    fheight = (height + 15) & ~15
    return fwidth, fheight
