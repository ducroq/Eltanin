"""@package docstring
Image processor implements QThread
images are passed via wrapper
"""
 
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import cv2
import traceback
import numpy as np
from fps import FPS
from math import sqrt
from scipy.spatial import distance as dist
from objectSignals import ObjectSignals
from PyQt5.QtCore import QSettings, QThread, QTimer, QEventLoop, pyqtSignal, pyqtSlot
from rectangle import Rectangle

def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    # return the edged image
    return edged


class ImageProcessor(QThread):
    '''
    Worker thread

    :param callback: The function callback to run on this worker thread. Supplied args and 
                     kwargs will be passed through to the runner.
    :type callback: function
    :param args: Arguments to pass to the callback function
    :param kwargs: Keywords to pass to the callback function

    '''
    signals = ObjectSignals()
    confirmed = pyqtSignal() # internal signal
    diaphragmFound = pyqtSignal(np.ndarray)
    wellFound = pyqtSignal(np.ndarray)
#     returnSharpnessScore = pyqtSignal(float)
    quality = pyqtSignal(float)
    

    def __init__(self):
        super().__init__()

        self.settings = QSettings("settings.ini", QSettings.IniFormat)
        self.loadSettings()
        self.image = None
        self.diaphragm = None
        self.ROI = None
        self.well = None
        self.fps = FPS().start()
        

    def loadSettings(self):
        self.msg("info;loading settings from {:s}".format(self.settings.fileName()))
                

    @pyqtSlot(np.ndarray)
    # Note that we need this wrapper around the Thread run function, since the latter will not accept any parameters
    def update(self, image=None):
        try:            
            if self.isRunning():
                # thread is already running
                # drop frame
                self.msg('error;busy, frame dropped')
            elif image is not None:
                # we have a new image
                self.image = image #.copy()        
                self.start()                
        except Exception as err:
            traceback.print_exc()
            self.signals.error.emit((type(err), err.args, traceback.format_exc()))
            

    def msg(self, text):
        if text:
            text = self.__class__.__name__ + ";" + str(text)
            print(text)
            self.signals.message.emit(text)
            
            
    @pyqtSlot()
    def run(self):
        '''
        Initialise the runner function with passed args, kwargs.
        '''
        try:
            if self.isInterruptionRequested():
                self.finished.emit()
                return            
            if self.image is not None:
                self.fps.update()
                
                qual = self.computeQuality()                
                self.quality.emit(qual)
                self.msg("info;image quality = {:.2f}".format(qual))                

                # annotate the image
                if self.diaphragm is not None or self.well is not None or self.ROI is not None:
                    img = cv2.cvtColor(self.image, cv2.COLOR_GRAY2RGB).copy() if len(self.image.shape) < 3 else self.image.copy()
                    if self.diaphragm is not None:
                        cv2.circle(img, (self.diaphragm[0],self.diaphragm[1]), self.diaphragm[2], (0,255,0), 2)
                    if self.well is not None:
                        cv2.circle(img, (self.well[0],self.well[1]), self.well[2], (0,0,255), 2)
                    if self.ROI is not None:
                        cv2.rectangle(img, self.ROI.p1, self.ROI.p2, (255, 0, 0), 2)
                else:
                    img = self.image
                    
                self.signals.result.emit(img)                
                self.confirmed.emit()
                
        except Exception as err:
            traceback.print_exc()
            self.signals.error.emit((type(err), err.args, traceback.format_exc()))            
        finally:
            self.signals.finished.emit()  # Done

   
    @pyqtSlot()
    def stop(self):
        self.msg("info;stopping")
        self.requestInterruption()        
        self.wait(500)
        self.fps.stop()
        self.quit()        
        self.msg("info;approx. processing speed: {:.2f} fps".format(self.fps.fps()))
        

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
        

    @pyqtSlot()
    def findDiaphragm(self):
        if self.image is not None:
            try:
                self.msg("info;looking for diaphragm")
                img = None
                
                # Average one second of images
                self.fps.stop()
                for i in range(0,int(self.fps.fps())):
                    self.wait_signal(self.confirmed, 1000)
                    new_img = self.image if len(self.image.shape) < 3 else cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY) # convert to gray image
                    img = new_img if img is None else cv2.addWeighted(img, 0.5, new_img, 0.5, 0)

                # Detect strong edge
                img = cv2.medianBlur(img, 5)
                img[np.where( img < np.mean(img) )] = 0
                img = auto_canny(img)

                # Find outer contour                
                contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if len(contours)>0 and len(contours[0]) > 10:
                    # Parameterize circle
                    c = cv2.fitEllipse(contours[0])
                    self.diaphragm = np.array((round(c[0][0]), round(c[0][1]), round((c[1][0]+c[1][1])/4)), dtype='int')
                else:
                    # fake a diaphragm
                    self.msg("error;no diaphragm found, so faking one at image centre")
                    self.diaphragm = np.array((round(self.image.shape[1]/2), round(self.image.shape[0]/2), \
                                                round(0.9*self.image.shape[0]/2)), dtype='int')
                    
                # Compute camera resolution and shift from known diaphragm size and position
                resolution = 2*self.diaphragm[2] / float(self.settings.value('diaphragm/diameter_in_mm')) # [px/mm]
                shift_x_mm = (self.diaphragm[0] - .5*img.shape[0])/resolution
                shift_y_mm = (self.diaphragm[1] - .5*img.shape[1])/resolution

                # Compute expected well parameters
                self.minRadius = int(.9*self.diaphragm[2])
                self.maxRadius = int(1.1*self.diaphragm[2])
                self.minDistance = int(min(img.shape)/self.minRadius) # find as many circles as reasonably possible

                # Store system settings
                self.settings.setValue('camera/resolution_in_px_per_mm', '{:.1f}'.format(resolution))
                self.settings.setValue('camera/shift_x_in_mm', '{:.1f}'.format(shift_x_mm))
                self.settings.setValue('camera/shift_y_in_mm', '{:.1f}'.format(shift_y_mm))
                self.settings.setValue('diaphragm/centre_wrt_image_origin_in_px', '{:d},{:d}'.format(self.diaphragm[0], self.diaphragm[1]))
                self.settings.setValue('diaphragm/diameter_in_px', '{:d}'.format(self.diaphragm[2]))

                self.msg("info;diaphragm found at: (" + str(self.diaphragm[0]) + " | " + str(self.diaphragm[1]) + "). Radius: " + str(self.diaphragm[2]))
                self.msg("info;camera resolution estimated at {:.1f} px/mm".format(resolution))

                self.diaphragmFound.emit(self.diaphragm)
            except Exception as err:
                traceback.print_exc()
                self.signals.error.emit((type(err), err.args, traceback.format_exc()))            

    @pyqtSlot()
    def findWell(self):
        if self.image is not None:
            try:
                self.msg("info;looking for well")
                img = None
                
                # Average one second of images
                self.fps.stop()
                for i in range(0,int(self.fps.fps())):
                    self.wait_signal(self.confirmed, 1000)
                    new_img = self.image if len(self.image.shape) < 3 else cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY) # convert to gray image
                    img = new_img if img is None else cv2.addWeighted(img, 0.5, new_img, 0.5, 0)

                # Detect strong circles
                img = cv2.medianBlur(img, 5)
                clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8, 8))
                img = clahe.apply(img)
                circles = cv2.HoughCircles(image=img, method=cv2.HOUGH_GRADIENT, dp=1, param1=40, param2=80, minDist=self.minDistance, minRadius=self.minRadius, maxRadius=self.maxRadius)
                
                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")

                    # Suppress circles that drop outside the image
                    circles = circles[
                        np.where( ((circles[:,0] - circles[:,2]) > 0) &
                                  ((circles[:,1] - circles[:,2]) > 0) &
                                  ((circles[:,0] + circles[:,2]) < img.shape[1]) &
                                  ((circles[:,1] + circles[:,2]) < img.shape[0]) )]
                    
                    # Select circle with maximum radius
##                    index = np.argmax(circles[:,2])
##                    self.well = circles[index]
                    # We are expecting one circle, so averaging all results seems to do the trick
                    self.well = np.mean(circles, axis=0).astype('int')
                    self.wellFound.emit(self.well)
                    
                    self.msg("info;well found at: (" + str(self.well[0]) + " | " + str(self.well[1]) + "). Radius: " + str(self.well[2]))
                else:
                    self.msg("error;no well found")
                        
            except Exception as err:
                traceback.print_exc()
                self.signals.error.emit((type(err), err.args, traceback.format_exc()))


    def computeQuality(self):
        # Compute ROI
        if self.diaphragm is None:
            ROI_leg = int(min(self.image.shape)/4)
            x, y = int(self.image.shape[1]/2), int(self.image.shape[0]/2)
        else:
            ROI_leg = int(round(self.diaphragm[2]/sqrt(2)))
            x, y = self.diaphragm[0], self.diaphragm[1]
        
        # crop image
        self.ROI = Rectangle(x - ROI_leg, y - ROI_leg, x + ROI_leg, y + ROI_leg)
        img = self.image[self.ROI.y1:self.ROI.y2, self.ROI.x1:self.ROI.x2]
        
        # further adapt ROI depending on target to focus on
        focustarget = self.settings.value('autofocus/focustarget', None, type=int)
        
        if focustarget == 1:
            # find blobs
            filtered_img = cv2.bilateralFilter(img,10,20,50)
            bw_img = cv2.adaptiveThreshold(filtered_img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,5,2)
            _, _, stats, centroids = cv2.connectedComponentsWithStats(bw_img)
            # select blobs with right sizes
            minBlobArea = 10**2
            maxBlobArea = int((ROI_leg/10)**2)
            ROIareas = stats[:,2]*stats[:,3]
            indices = np.where( (ROIareas > minBlobArea) & (ROIareas < maxBlobArea) )[0]
            if len(indices) > 0:
                # select single blob closest to centroid
                dist = np.linalg.norm(centroids - [ROI_leg,ROI_leg],axis=1)
                index = indices[np.argmin(dist[indices])]
                # grab image 
                blobROI = stats[index]
                img = img[blobROI[1]:blobROI[1]+blobROI[3], blobROI[0]:blobROI[0]+blobROI[2]]
                # redefine ROI
                self.ROI = Rectangle(self.ROI.x1 + blobROI[0],
                                     self.ROI.y1 + blobROI[1],
                                     self.ROI.x1 + blobROI[0] + blobROI[2],
                                     self.ROI.y1 + blobROI[1] + blobROI[3])
                    
        # Compute variance of Laplacian in RoI
        img = cv2.normalize(img,dst=None,dtype=cv2.CV_32F)
#         img = cv2.normalize(img,dst=None,alpha=1,beta=1,norm_type=cv2.NORM_L2,dtype=cv2.CV_32F)
        # see https://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html#cv2.normalize
        edge_laplace = cv2.Laplacian(img, ddepth=cv2.CV_32F, ksize=5)
        sharpness = 100*np.percentile(edge_laplace, 90)
        variance = 1e5*np.var(edge_laplace)
        # maximum and variance of Laplacian are indicators of sharpness, maximum maybe noisy, so choose 90% percentile instead
        return variance               
   