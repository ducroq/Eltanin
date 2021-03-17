#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Based on
 https://picamera.readthedocs.io/en/latest/
 https://picamera.readthedocs.io/en/release-1.13/api_camera.html
 https://www.raspberrypi.org/documentation/hardware/camera/
    
version December 2020, January 2021
Note that there are different image sizes involved:
1. captureFrameSize: image capture frame size, obtained from the sensor mode
2. videoFrameSize: video capture frame size
3. frameSize: size of frames that get emitted to the processing chain.
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
from io import BytesIO
from subprocess import run, check_output


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
    return (fwidth, fheight)

def frame_size_from_sensor_mode(sensorMode):
    if sensorMode == 0:
        frameSize = (4056,3040)
    elif sensorMode == 1:
        frameSize = (1920,1080)
    elif sensorMode == 2 or sensorMode == 3:
        frameSize = (3280, 2464)               
    elif sensorMode == 4:
        frameSize = (1640, 1232)               
    elif sensorMode == 5:
        frameSize = (1640,922)
    elif sensorMode == 6:
        frameSize = (1280, 720)
    elif sensorMode == 7:
        frameSize = (640, 480)
    else:
        raise ValueError
    return frameSize

def frame_size_from_string(frameSizeStr):
    (width, height) = frameSizeStr.split('x')
    return (int(width), int(height))

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
    """
    Thread that produces frames for further processing as a PyQtSignal.
    Picamera is set-up according to sensormode and splitter_port 0 is used for capturing image data.
    A video stream is set-up, using picamera splitter port 1 and resized to frameSize.
    Splitter_port 2 is used for capturing video at videoFrameSize.
    """ 
    image = None
    finished = pyqtSignal()
    postMessage = pyqtSignal(str)
    frame = pyqtSignal(np.ndarray)
    progress = pyqtSignal(int)       
    captured = pyqtSignal()
    
    camera = PiCamera()
    videoStream = BytesIO()
    
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
        self.sensorMode = int(self.settings.value('camera/sensor_mode'))
        self.frameRate = int(self.settings.value('camera/frame_rate'))

        # set frame sizes
        self.frameSize = frame_size_from_string(self.settings.value('frame_size'))
        self.captureFrameSize = frame_size_from_sensor_mode(self.sensorMode)
        self.videoFrameSize = frame_size_from_string(self.settings.value('camera/video_frame_size'))

        if not self.monochrome:
            self.frameSize = self.frameSize + (3,)

    @pyqtSlot()
    def initStream(self):
        if self.isRunning():
            self.requestInterruption()
            wait_signal(self.finished, 10000)            
        # Set camera parameters
        self.camera.exposure_mode = 'backlight' # 'auto'
        self.camera.awb_mode = 'flash' # 'auto'
        self.camera.meter_mode = 'backlit' # 'average'
        self.camera.sensor_mode = self.sensorMode
        self.camera.resolution = self.captureFrameSize
        self.camera.framerate = self.frameRate
        self.camera.image_effect = self.settings.value('camera/effect')
        self.camera.iso = int(self.settings.value('camera/iso')) # should force unity analog gain       
        self.camera.video_denoise = self.settings.value('camera/video_denoise', False, type=bool)

        # Wait for the automatic gain control to settle
        wait_ms(3000)

#         # Now fix the values
#         self.camera.shutter_speed = self.camera.exposure_speed
#         self.camera.exposure_mode = 'off'
#         g = self.camera.awb_gains
#         self.camera.awb_mode = 'off'
#         self.camera.awb_gains = g        

##            # Setup video port, GPU resizes frames, and compresses to mjpeg stream
##            self.camera.start_recording(self.videoStream, format='mjpeg', splitter_port=1, resize=self.frameSize)

        # Setup capture from video port 1
        if self.monochrome:
            self.rawCapture = PiYArray(self.camera, size=self.frameSize)
            self.captureStream = self.camera.capture_continuous(self.rawCapture, 'yuv', use_video_port=True, splitter_port=1, resize=self.frameSize)
        else:
            self.rawCapture = PiRGBArray(self.camera, size=self.frameSize)
            self.captureStream = self.camera.capture_continuous(self.rawCapture, 'bgr', use_video_port=True, splitter_port=1, resize=self.frameSize)
        # init crop rectangle
        if self.cropRect[2] == 0:
            self.cropRect[2] = self.camera.resolution[1]
        if self.cropRect[3] == 0:
            self.cropRect[3] = self.camera.resolution[0]
            
        # start the thread
        self.start(QThread.HighPriority)
        msg = "{}: info; video stream initialized with frame size = {} and {:d} channels".format(\
            __class__.__name__, str(self.camera.resolution), 1 if self.monochrome else 3)
        self.postMessage.emit(msg)

    @pyqtSlot()
    def run(self):
        try:
            self.fps = FPS().start()
            for f in self.captureStream:
                if self.isInterruptionRequested():
                    break
                self.rawCapture.seek(0) 
                img = f.array # grab the frame from the stream
                self.frame.emit(img)#cv2.resize(img, self.frameSize[:2]))
                self.fps.update()                

##                # Grab jpeg from an mpeg video stream
##                self.videoStream.seek(0)
##                buf = self.videoStream.read()
##                if buf.startswith(b'\xff\xd8'):
##                    # jpeg magic number is detected
##                    flag = cv2.IMREAD_GRAYSCALE if self.monochrome else cv2.IMREAD_COLOR
##                    img = cv2.imdecode(np.frombuffer(buf, dtype=np.uint8), flag) 
##                    self.frame.emit(img)
##                    self.fps.update()
##                    self.videoStream.truncate(0)

        except Exception as err:
            self.postMessage.emit("{}: error; type: {}, args: {}".format(self.__class__.__name__, type(err), err.args))
        finally:
            self.fps.stop()
            img = np.zeros(shape=(self.frameSize[1],self.frameSize[0]), dtype=np.uint8)
            cv2.putText(img,'Camera suspended', (int(self.frameSize[0]/2)-150,int(self.frameSize[1]/2)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255),1)
            for i in range(5):
                wait_ms(100)
                self.frame.emit(img)
            msg = "{}: info; finished, approx. processing speed: {:.2f} fps".format(self.__class__.__name__, self.fps.fps())
            self.postMessage.emit(msg)
            self.finished.emit()
        
    @pyqtSlot()
    def stop(self):
        self.postMessage.emit("{}: info; stopping".format(__class__.__name__))
        try:
            if self.isRunning():
                self.requestInterruption()
                wait_signal(self.finished, 10000)
        except Exception as err:
            msg = "{}: error; stopping method".format(self.__class__.__name__)
            print(msg)
        finally:
            self.quit() # Note that thread quit is required, otherwise strange things happen.
       
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
        try:
            self.camera.capture(filename, use_video_port=False, splitter_port=0, format='png')
        except Exception as err:
            self.postMessage.emit("{}: error; type: {}, args: {}".format(self.__class__.__name__, type(err), err.args))

        self.captured.emit()
        self.postMessage.emit("{}: info; image written to {}".format(__class__.__name__, filename))

    @pyqtSlot(str, int)
    def recordClip(self, filename_prefix=None, duration=10):
        """
        Captures a videoclip of duration at resolution videoFrameSize.
        The GPU resizes the captured video to the intended resolution.
        Note that while it seems possble to change the sensormode, reverting to the original mode fails when capturing an image.
        In many cases, the intended framerate is not achieved. For that reason, ffprobe counts
        the total number of frames that were actually taken.        
        Next, the h264 video file is boxed using MP4box.
        For some reason, changing the framerate with MP4Box did not work out.        
        """
        if filename_prefix is not None:
            (head, tail) = os.path.split(filename_prefix)
            if not os.path.exists(head):
                os.makedirs(head)
            filename = os.path.sep.join([head, '{:016d}_{}s'.format(round(time.time() * 1000), round(duration)) + tail])
        else:
            filename = '{:016d}_{}s'.format(round(time.time() * 1000), round(duration))
            # open path
            if self.storagePath is not None:
                filename = os.path.sep.join([self.storagePath, filename])

        # stop current video stream, maybe mpeg and h264 compression cannot run simultaneously?
        self.postMessage.emit("{}: info; starting recording for {} s".format(__class__.__name__, duration))
        
        try:
            # GPU resizes frames, and compresses to h264 stream
            self.camera.start_recording(filename + '.h264', format='h264', splitter_port=2, resize=self.videoFrameSize, sps_timing=True)
            wait_ms(duration*1000)
            self.camera.stop_recording(splitter_port=2)
            
            # Wrap an MP4 box around the video
            nr_of_frames = check_output(["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0", "-show_entries", "stream=nb_read_frames", "-of", "default=nokey=1:noprint_wrappers=1", filename + '.h264'])
            real_fps = duration/float(nr_of_frames)
            self.postMessage.emit("{}: info; video clip captured with real framerate: {} fps".format(__class__.__name__, real_fps))
##            run(["MP4Box", "-fps", str(self.frameRate), "-add", filename + '.h264:fps=' + str(real_fps), "-new", filename + '.mp4'])
            run(["MP4Box", "-fps", str(self.frameRate), "-add", filename + ".h264", "-new", filename + "_{}fr.mp4".format(int(nr_of_frames))])
            run(["rm", filename + '.h264'])
        except Exception as err:
            self.postMessage.emit("{}: error; type: {}, args: {}".format(self.__class__.__name__, type(err), err.args))            
            
        self.captured.emit()
        self.postMessage.emit("{}: info; video written to {}".format(__class__.__name__, filename))

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

