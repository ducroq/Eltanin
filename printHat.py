#!/usr/bin/python3
# -*- coding: utf-8 -*-
### @package printhat.py
## @brief printhat.py contains classes for stepper motor control, G-code string creation, homing and well positioning.
## @author Gert van Lagen, Jeroen Veen
import os
import io
import re
import serial
import traceback
from math import sqrt
from PyQt5.QtWidgets import QMessageBox
from objectSignals import ObjectSignals
from PyQt5.QtCore import QSettings, QThread, QMutex, QTimer, QEventLoop, pyqtSignal, pyqtSlot

   
class PrintHat(QThread):
    signals = ObjectSignals()    
    confirmed = pyqtSignal() # internal signal
    homed = pyqtSignal()
    positionReached = pyqtSignal()
    mutex = QMutex()
                       
    sio = None
    eps = 1e-3

    ## @param ins is the number of instances created. This may not exceed 1.
    ins = 0

    def __init__(self):
        super().__init__()

        self.settings = QSettings("settings.ini", QSettings.IniFormat)
        self.loadSettings()
        self.buf = bytearray()
        self.is_paused = True # Serial read communication thread is pauses
        self.is_homed = False # PrintHat motors have been homed
        self.is_ready = False # Printhat and klipper are ready
        self.has_arrived = False # latest position ahs been arrived
        self.position_x = None # last known position
        self.position_y = None
        self.position_z = None        
        
        ## Instance limiter. Checks if an instance exists already. If so, it deletes the current instance.
        if PrintHat.ins >= 1:
            del self
            self.msg("error;multiple instances of {:s} created, while only 1 instance is allowed".format(__class__.__name__))
            return        
        try:
            self.connectKlipper(self.port)
            PrintHat.ins+=1
        except Exception as err:
            traceback.print_exc()
            self.signals.error.emit((type(err), err.args, traceback.format_exc()))
            self.stop()

        
    def loadSettings(self):
        self.msg("info;loading settings from {:s}".format(self.settings.fileName()))
        self.port = self.settings.value('printhat/port')
        

    @pyqtSlot()
    def run(self):
        """
        Initialise the runner function with passed args, kwargs.
        """
        if self.sio:                
            while True:
                if self.isInterruptionRequested():
                    self.signals.finished.emit()
                    return
                try:
                    if not self.is_paused:
                        reply_msg = self.sio.readline()                        
                        if reply_msg:
                            self.msg("info;printHat replied: " + reply_msg)
                            if 'Klipper state' in reply_msg:
                                if 'Ready' in reply_msg:
                                    self.is_ready = True
                                    self.signals.ready.emit()
                                elif 'Shutdown' in reply_msg or 'Disconnect' in reply_msg:
                                    self.is_ready = False
                            elif '!!' in reply_msg:
                                if 'Must home axis' in reply_msg:
                                    self.is_homed = False
                            elif 'ok' in reply_msg:
                                self.confirmed.emit()
                            elif 'X:' in reply_msg:
                                r = re.findall(r"[-+]?\d*\.\d+|\d+", reply_msg)
                                self.position_x = float(r[0])
                                self.position_y = float(r[1])
                                self.position_z = float(r[2])
                                
                except Exception as err:
                    traceback.print_exc()
                    self.signals.error.emit((type(err), err.args, traceback.format_exc()))        
    
    @pyqtSlot()
    def stop(self):
        self.msg("info;stopping")
        self.disableMotors()
        self.setLightPWM(0.0)
        self.setFanPWM(0.0)
        self.wait_signal(self.confirmed, 1000)
        self.requestInterruption()
        self.wait_signal(self.signals.finished, 10000)
        self.wait_ms(500) # some signals may need to settle
        self.sio = None
        self.disconnectKlipper()
        self.quit()

        
    def msg(self, text):
        if text:
            text = self.__class__.__name__ + ";" + str(text)
            print(text)
            self.signals.message.emit(text)


    ## @brief PrintHat::connect connects to the pseudo serial port /tmp/printer.
    ## This port is the link with the klipper library which handles all the g-code and communication with the printHat.
    # @param port is the port to be connected to. 
    def connectKlipper(self, port):
        try:
            self.mutex.tryLock(1000)
            
            self.msg("info;starting klipper service")            
            # make sure klipper service is active
            os.system('sudo service klipper restart && sudo service klipper status | more')

            ## Try to open the serial port
            self.wait_ms(1000)
            ser = serial.Serial(self.port, 250000, timeout=1)
            self.sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser), line_buffering = True) #, newline="\r\n")
            if self.sio: #port.is_open:
                self.msg("info;connected to printHat via serial port {}".format(self.port))
                self.is_paused = False
            else:
                self.msg("error;cannot connect to printHat via serial port {}".format(self.port))
                
            self.mutex.unlock()
                    
        except Exception as err:
            traceback.print_exc()
            self.signals.error.emit((type(err), err.args, traceback.format_exc()))
            
    
    def disconnectKlipper(self):
        try:
            self.mutex.tryLock(1000)
            
            ## Stop klipper service and show its status
            self.msg("info;stopping klipper service")
            os.system('sudo service klipper stop && sudo service klipper status | more')
            
            self.mutex.unlock()
        except Exception as err:
            traceback.print_exc()
            self.signals.error.emit((type(err), err.args, traceback.format_exc()))            
        finally:
            return


    def wait_ms(self, timeout):
        ''' Block loop until timeout (ms) elapses.
        '''        
        loop = QEventLoop()
        QTimer.singleShot(timeout, loop.exit)
        loop.exec_()
        

    def wait_signal(self, signal, timeout=1000):
        ''' Block loop until signal received, or until timeout (ms) elapsed.
        '''
        loop = QEventLoop()
        signal.connect(loop.quit) # only quit is a slot of QEventLoop
        QTimer.singleShot(timeout, loop.exit)
        loop.exec_()
        

    ## @brief PrintHat::sendGcode writes a byte array containing a G-code to the serial port.
    # @param gcode_string is the string to be written to the serial port.
    def sendGcode(self, gcode_string):
        if self.sio:
            try:
                self.sio.write(gcode_string + "\r\n")
                self.sio.flush() # it is buffering. required to get the data out *now*
                self.msg("info;" + self.sendGcode.__name__ + " " + gcode_string)
                
                self.wait_signal(self.confirmed, 10000)
                
            except Exception as err:
                traceback.print_exc()
                self.signals.error.emit((type(err), err.args, traceback.format_exc()))            
        else:
            self.msg("error;no serial connection with printHat.")


    ## @brief PrintHat::firmwareRestart restarts the firmware and reloads the config in the klipper software.
    @pyqtSlot()
    def firmwareRestart(self):
        self.sendGcode("FIRMWARE_RESTART")
        

    ## @brief PrintHat::emergencyBreak stops all motors and shuts down the printHat. A firmware restart command is necessary to restart the system.
    @pyqtSlot()
    def emergencyBreak(self):
        self.msg("error;emergency break! restart the firmware")
        self.sendGcode("M112")
        

    @pyqtSlot(float)
    def setLightPWM(self, val):
        ''' Set PrintHAT light output pin to PWM value.
            Args:
                val (float): PWM dutycycle, between 0.0 and 1.0.
            Raises:
            Returns:
        '''
        self.sendGcode("SET_PIN PIN=light VALUE=" + str(val))
        

    @pyqtSlot(float)
    def setFanPWM(self, val):
        ''' Set PrintHAT fan output pin to PWM value.
            Args:
                val (float): PWM dutycycle, between 0.0 and 1.0.
            Raises:
            Returns:
        '''
        clip_val = 1.0
        val = val if val < clip_val else clip_val
        self.sendGcode("SET_PIN PIN=rpi_fan VALUE={:1.2f}".format(val))
        
        
    @pyqtSlot()
    def enableMotors(self):
## why can we not re-enable motors during steps? A: Because M17 is not implemented by Klipper
        self.msg("info;enable motors")
        self.sendGcode("M17")
        

    @pyqtSlot()
    def disableMotors(self):
        # see https://reprap.org/wiki/G-code#M84:_Stop_idle_hold
##        On Klipper M84 is equivalent to G-code#M18:_Disable_all_stepper_motors
        self.msg("info;stop the idle and hold on all axes")
        self.sendGcode("M84")
        self.sendGcode("M18")

    @pyqtSlot()
    def homeXYZ(self):
        if self.is_ready:
            self.msg("info;homeXY called")
            self.position_x = self.position_y = self.position_z = None
            self.sendGcode("G28 X Y Z")
            # wait until we have really reached home, this can take a while
            for i in range(0,20): # limit the number of tries
                self.wait_ms(10000)
                self.getPosition()
                if self.position_x is None or self.position_y is None or self.position_z is None:
                    self.wait_ms(1000) # homing is slow, so wait a bit more
                elif abs(self.position_x) < self.eps and abs(self.position_y) < self.eps and abs(self.position_z) < self.eps:
                    self.msg("info;homeXY confirmed")
                    self.is_homed = True
                    self.homed.emit()
                    break            
            if not self.is_homed:
                self.msg("error;homeXY failed")
        else:
            self.msg("error;printhat not ready")
            
    @pyqtSlot()
    def getPosition(self):
        self.sendGcode("M400") # Wait for current moves to finish
        self.sendGcode("M114")        
        if self.position_x is not None and self.position_y is not None and self.position_z is not None:
            self.msg("info;current position = ({:.3f}, {:.3f}, {:.3f})".format(self.position_x, self.position_y, self.position_z))
        else:
            self.msg("error;current position unknown")

    @pyqtSlot(float, float, float)
    def gotoXYZ(self, x=None, y=None, z=None):
        
        if self.mutex.tryLock(100):
            gcode_string = "G1"
            gcode_string += " X{:.3f}".format(x) if x is not None else ""
            gcode_string += " Y{:.3f}".format(y) if y is not None else ""
            gcode_string += " Z{:.3f}".format(z) if z is not None else ""
            self.sendGcode(gcode_string)
            
            # if printhat returns error, initiate homing, and resend G-code
            if not self.is_homed:
                self.homeXYZ()
                self.sendGcode(gcode_string)
            
            # wait until we have really reached the desired location
            prev_x, prev_y, prev_z = 0,0,0
            for i in range(0,10): # limit the number of tries
                self.wait_ms(10)
                self.getPosition()
                if self.position_x is not None and self.position_y is not None and self.position_z is not None:
                    error = 0
                    error += (self.position_x-x)**2 if x is not None else 0
                    error += (self.position_y-y)**2 if y is not None else 0
                    error += (self.position_z-z)**2 if z is not None else 0
                    if sqrt(error) < self.eps \
                       or ( abs(self.position_x-prev_x) < self.eps and \
                            abs(self.position_y-prev_y) < self.eps and \
                            abs(self.position_z-prev_z) < self.eps ):
                        self.msg("info;gotoXYZ confirmed")
                        self.positionReached.emit()
                        break
                    else:
                        prev_x, prev_y, prev_z = self.position_x, self.position_y, self.position_z
            
            self.mutex.unlock()
        else:
            self.msg("error;mutex lock failed")
            
    @pyqtSlot(float, float, bool)
    def gotoXY(self, x, y, relative=True):
        ''' Move to a postion in the horizontal plane relative to the stage origin (relative=True) or to home (relative=False)
        '''
        if relative:
            x += float(self.settings.value('camera/centre_wrt_home_in_mm')[0]) - float(self.settings.value('stage/origin_wrt_home_in_mm')[0])
            y += float(self.settings.value('camera/centre_wrt_home_in_mm')[1]) - float(self.settings.value('stage/origin_wrt_home_in_mm')[1])
        self.gotoXYZ(x=x, y=y)

    @pyqtSlot(float, bool)
    def gotoX(self, x, relative=True):
        ''' Move to x relative to the stage origin (relative=True) or to home (relative=False)
        '''
        if relative:
            x += float(self.settings.value('camera/centre_wrt_home_in_mm')[0]) - float(self.settings.value('stage/origin_wrt_home_in_mm')[0])
        self.gotoXYZ(x=x)

    @pyqtSlot(float, bool)
    def gotoY(self, y, relative=True):
        ''' Move to y relative to the stage origin (relative=True) or to home (relative=False)
        '''
        if relative:
            y += float(self.settings.value('camera/centre_wrt_home_in_mm')[1]) - float(self.settings.value('stage/origin_wrt_home_in_mm')[1])
        self.gotoXYZ(y=y)        

    @pyqtSlot(float)
    def gotoZ(self, z):
        self.gotoXYZ(z=z)


