#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Based on
 https://picamera.readthedocs.io/en/latest/
 https://picamera.readthedocs.io/en/release-1.13/api_camera.html
 https://www.raspberrypi.org/documentation/hardware/camera/
    
version December 2020
Note that there are different image sizes involved:
display_frame_size: size of frames that get emitted to the processing chain.
frame_size: intended image capture frame size, is set using sensor_mode
clip_frame_size: intended video capture frame size, is set using clip_sensor_mode
 
"""
import os
import cv2
import time
import warnings
import traceback
import numpy as np
from fps import FPS
from picamera import PiCamera
from picamera.array import PiRGBArray, PiYUVArray, PiArrayOutput
from PyQt5.QtCore import QThread, QSettings, pyqtSlot, QTimer, QEventLoop, pyqtSignal
from wait import wait_signal, wait_ms


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
##            raise PiCameraValueError
##            'Incorrect buffer length for frame_size %dx%d' % (width, height))

    def flush(self):
        super(PiYArray, self).flush()
        a = np.frombuffer(self.getvalue()[:self.y_len], dtype=np.uint8)
        self.array = a[:self.y_len].reshape((self.fheight, self.fwidth))
        

## PiVideoStream class streams camera images to a numpy array
class PiVideoStream(QThread):
    image = None
    finished = pyqtSignal()
    postMessage = pyqtSignal(str)
    frame = pyqtSignal(np.ndarray)
    progress = pyqtSignal(int)       
    captured = pyqtSignal()
    
    camera = PiCamera()
    storagePath = None
    cropRect = [0] * 4

    ## @param ins is the number of instances created. This may not exceed 1.
    ins = 0
    
    def __init__(self):
        super().__init__()

        ## Instance limiter. Checks if an instance exists already. If so, it deletes the current instance.
        if PiVideoStream.ins >= 1:
            del self            
            self.postMessage.emit("{}: error; multiple instances of created, while only 1 instance is allowed".format(__class__.__name__))
            return        
        try:
            PiVideoStream.ins+=1
        except Exception as err:
            self.postMessage.emit("{}: error; type: {}, args: {}".format(self.__class__.__name__, type(err), err.args))
        else:
            warnings.filterwarnings('default', category=DeprecationWarning)
            self.settings = QSettings("settings.ini", QSettings.IniFormat)
            self.loadSettings()
##            self.initStream()
            
            
    def loadSettings(self):
        self.postMessage.emit("{}: info; loading camera settings from {}".format(self.__class__.__name__, self.settings.fileName()))

        # load
        self.monochrome = self.settings.value('camera/monochrome', False, type=bool)
        self.use_video_port = self.settings.value('camera/use_video_port', False, type=bool)
        self.sensor_mode = int(self.settings.value('camera/sensor_mode'))

        # set frame sizes
        if self.sensor_mode == 0:
            self.frame_size = (4056,3040)
        elif self.sensor_mode == 1:
            self.frame_size = (1920,1080)
        elif self.sensor_mode == 2 or self.sensor_mode == 3:
            self.frame_size = (3280, 2464)               
        elif self.sensor_mode == 4:
            self.frame_size = (1640, 1232)               
        elif self.sensor_mode == 5:
            self.frame_size = (1640,922)
        elif self.sensor_mode == 6:
            self.frame_size = (1280, 720)
        elif self.sensor_mode == 7:
            self.frame_size = (640, 480)
        else:
            raise ValueError

        frame_size_str = self.settings.value('display_frame_size')
        (width, height) = frame_size_str.split('x')
        self.display_frame_size = (int(width), int(height))
        if not self.monochrome:
            self.display_frame_size = self.display_frame_size + (3,)

        # set more camera parameters
        self.camera.resolution = self.frame_size
        self.camera.sensor_mode = self.sensor_mode
        self.camera.framerate = int(self.settings.value('camera/frame_rate'))
        self.camera.image_effect = self.settings.value('camera/effect')
        self.camera.shutter_speed = int(self.settings.value('camera/shutter_speed'))
        self.camera.iso = int(self.settings.value('camera/iso')) # should force unity analog gain       
        self.camera.video_denoise = self.settings.value('camera/video_denoise', False, type=bool)

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
            self.postMessage.emit("{}: error; video stream is already running".format(__class__.__name__))
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
            self.image = np.empty(self.camera.resolution + (1 if self.monochrome else 3,), dtype=np.uint8)
            # init crop rectangle
            if self.cropRect[2] == 0:
                self.cropRect[2] = self.image.shape[1]
            if self.cropRect[3] == 0:
                self.cropRect[3] = self.image.shape[0]
            # restart thread
            self.start()
            wait_ms(1000)
            msg = "{}: info; video stream initialized with frame size = {} and {:d} channels".format(\
                __class__.__name__, str(self.camera.resolution), 1 if self.monochrome else 3)
            self.postMessage.emit(msg)


    @pyqtSlot()
    def run(self):
        try:
            self.fps = FPS().start()
            for f in self.stream:
                if self.isInterruptionRequested():
                    self.finished.emit()
                    return                   
                self.rawCapture.seek(0) 
                self.image = f.array # grab the frame from the stream
##                # Crop
##                if (self.cropRect[2] > self.cropRect[0]) and (self.cropRect[3] > self.cropRect[1]):
##                    self.frame.emit(self.image[self.cropRect[0]:self.cropRect[2], self.cropRect[1]:self.cropRect[3]])
                # Emit resized frame for speed 
                self.frame.emit(cv2.resize(self.image, self.display_frame_size[:2]))
                self.fps.update()
        except Exception as err:
            self.postMessage.emit("{}: error; type: {}, args: {}".format(self.__class__.__name__, type(err), err.args))            

        
    @pyqtSlot()
    def stop(self):
        self.postMessage.emit("{}: info; stopping".format(__class__.__name__))
        if self.isRunning():
            self.requestInterruption()
            wait_signal(self.finished, 2000)        
        self.fps.stop()
        msg = "{}: info; approx. processing speed: {:.2f} fps".format(self.__class__.__name__, self.fps.fps())
        self.postMessage.emit(msg)
        print(msg)
        self.quit()
       

    @pyqtSlot(str)
    def takeImage(self, filename_prefix=None):
        if filename_prefix is not None:
            (head, tail) = os.path.split(filename_prefix)
            if not os.path.exists(head):
                os.makedirs(head)
            filename = os.path.sep.join([head, '{:016d}_'.format(round(time.time() * 1000)) + tail + '.png'])
        else:
            filename = '{:016d}'.format(round(time.time() * 1000)) + '.png'
            # open path
            if self.storagePath is not None:
                filename = os.path.sep.join([self.storagePath, filename])

        # write image
        wait_signal(self.frame, 5000) # wait for first frame to be shot
        cv2.imwrite(filename, self.image)
        self.captured.emit()
        self.postMessage.emit("{}: info; image written to {}".format(__class__.__name__, filename))

    @pyqtSlot(str, int)
    def recordClip(self, filename_prefix=None, duration=10):
        # open path
        (head, tail) = os.path.split(filename_prefix)
        if not os.path.exists(head):
            os.makedirs(head)
        filename = os.path.sep.join([head, '{:016d}_'.format(round(time.time() * 1000)) + tail + '.avi'])

##"TODO; changing camera settings may get the process killed after several hours, probably better to open the stream in video resolution from the start if the videorecording is required!")

        # set video clip parameters
        self.stop()
        frame_size_str = self.settings.value('camera/clip_frame_size')
        (width, height) = frame_size_str.split('x')
        self.camera.resolution = (int(width), int(height))        
        self.camera.sensor_mode = int(self.settings.value('camera/clip_sensor_mode'))        
        self.camera.framerate = int(self.settings.value('camera/clip_frame_rate'))
        self.camera.image_effect = effect
        self.use_video_port = True
        self.monochrome = True
        self.initStream()        

        # define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(filename, fourcc, frame_rate, frame_size)
        self.msg("info; start recording video to " + filename)
        
        # write file        
        for i in range(0,duration*frame_rate):
            self.progress.emit(int(100*i/(duration*frame_rate-1)))
            wait_signal(self.frame, 1000)
            if self.image is not None:
                out.write(self.image)
                
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

    @pyqtSlot(str)
    def setStoragePath(self, path):
        self.storagePath = path
        
    @pyqtSlot(int)
    def setCropXp1(self, val):
        if 0 <= val <= self.cropRect[3]:        
            self.cropRect[1] = val
        else:
            raise ValueError('crop x1')
            
    @pyqtSlot(int)
    def setCropXp2(self, val):
        if self.cropRect[1] < val < self.camera.resolution[1]:            
            self.cropRect[3] = val
        else:
            raise ValueError('crop x2')
            
    @pyqtSlot(int)
    def setCropYp1(self, val):
        if 0 <= val <= self.cropRect[2]:        
            self.cropRect[0] = val            
        else:
            raise ValueError('crop y1')
            
    @pyqtSlot(int)
    def setCropYp2(self, val):
        if self.cropRect[0] < val < self.camera.resolution[0]:
            self.cropRect[2] = val            
        else:
            raise ValueError('crop y2')   

