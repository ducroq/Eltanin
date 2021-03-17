#!/usr/bin/python3
# -*- coding: utf-8 -*-
import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
import matplotlib.pyplot as plt
from wait import wait_ms, wait_signal

## Autofocus
## Perform a naive grid search to optimize the image quality parameter, imgQual, by varying the camera focus.
## A grid of size P, centred around the current focus, with grid spacing dP is searched.
## When the maximum IQ is found, the focus is to the value where max IQ occured.
## Then, the procedure is repeated N_n times, where in every iteration the grid spacing is halved.
##

class AutoFocus(QObject):
##    Gridsearch of a hyperparameter H (image quality) over a process variable P (focus).
##    Start signal initiates a search around a given point P_centre, with gridsize N_p and gridspacing dP.
##    The search is repeated N_n times, where the gridspacing is halved with each step.
    setFocus = pyqtSignal(float)  # Focus signal
    postMessage = pyqtSignal(str)
    focussed = pyqtSignal(float)
    rPositionReached = pyqtSignal() # repeat signal
    rImageQualityUpdated = pyqtSignal() # repeat signal
    
    def __init__(self,display=False):
        super().__init__()
        self.display = display
        self.k = 0 # plot position counter
        
    @pyqtSlot(float)
    def start(self, P_centre=0):
        N_p = 5 # half of total grid points
        dP = .25 # actuater step size
        avg_H = 5 # number of quality gauges to average

        self.postMessage.emit("{}: info; running".format(self.__class__.__name__))

        if self.display and (self.k == 0): # we have not plotted before
            self.fig, (self.ax1, self.ax2) = plt.subplots(2,1)
            self.graph1 = None
            self.ax1.grid(True)
            self.ax1.set_ylabel("Image quality")
            self.graph2 = None
            self.ax2.grid(True)
            self.ax2.set_ylabel("Voice coil value")
            plt.show(block=False)        
        
        P = P_centre + dP*(np.arange(2*N_p, dtype=float) - N_p)
        H = np.zeros_like(P)

        self.setFocus.emit(P[0])  # Move to starting point of grid search
        wait_signal(self.rPositionReached, 10000)
        wait_ms(3000)        
        
        for i,p in enumerate(P):
            self.setFocus.emit(p)
            wait_signal(self.rPositionReached, 10000)
            wait_ms(500)
            # average a few image quality values
            H[i] = 0
            for j in range(avg_H):
                wait_signal(self.rImageQualityUpdated, 10000)
                H[i] += self.imgQual
            H[i] /= avg_H
            self.postMessage.emit("{}: info; average image quality: {} at position: {}".format(self.__class__.__name__, H[i], p))
            # plot measurement
            if self.display:
                # draw grid lines
                self.graph1 = self.ax1.plot(self.k, H[i], 'bo')[0]
                self.graph2 = self.ax2.plot(self.k, p, 'bo')[0]
                # We need to draw *and* flush
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                self.k += 1
        # wrap up        
        coef_variance = np.std(H) / np.mean(H)
        if coef_variance > 0.1:
            # check if there is sufficient variation
            max_ind = np.argmax(H)
            P_centre = P[max_ind] # set new grid centre point
        self.postMessage.emit("{}: info; optimal image quality at position: {}".format(self.__class__.__name__, P_centre))
        value = round(P_centre,2)
        self.setFocus.emit(value)  # set next focus
        wait_signal(self.rPositionReached, 10000)
        self.focussed.emit(value) # publish focus

    @pyqtSlot(float)
    def imageQualityUpdate(self, imgQual):
        self.imgQual = imgQual
        self.rImageQualityUpdated.emit()
          
    @pyqtSlot()
    def stop(self):
        try:
            if self.display:
                plt.close()                
            
            self.running = False
        except Exception as err:
            self.postMessage.emit("{}: error; type: {}, args: {}".format(self.__class__.__name__, type(err), err.args))

    @pyqtSlot()
    def positionReached(self):
        self.rPositionReached.emit()            

