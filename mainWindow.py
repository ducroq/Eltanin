"""@package docstring
MainWindow
""" 
#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import cv2
import time
import numpy as np
import matplotlib
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, pyqtSlot, QTimer, QEventLoop, QSettings
from PyQt5.QtGui import QCloseEvent, QImage, QPixmap
from objectSignals import ObjectSignals

# Make sure that we are using QT5
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

current_milli_time = lambda: int(round(time.time() * 1000))

class MainWindow(QMainWindow):
    '''
    GUI
    '''
    signals = ObjectSignals()

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        
        self.central_widget = QWidget()               # define central widget
        self.setCentralWidget(self.central_widget)    # set QMainWindow.centralWidget

        self.appName = "Eltanin"
        self.image = None

        # Store constructor arguments (re-used for processing)
        self.args = args
        self.kwargs = kwargs
        
        self.prevClockTime = None
        self.settings = QSettings("settings.ini", QSettings.IniFormat)
        self.initUI()
        self.loadSettings()
        self.disableButtons()
        

    def initUI(self):
        self.setWindowTitle(self.appName)
        screen = QDesktopWidget().availableGeometry()
        self.imageWidth = round(screen.height() * 0.8)
        self.imageHeight = round(screen.width() * 0.8)
        self.imageScalingFactor = 1.0
        self.imageScalingStep = 0.1
        
        # Labels
        self.PixImage = QLabel()
        self.timerLabel = QLabel()
        self.imageQualityLabel = QLabel()
        self.temperatureLabel = QLabel()
        
        # a figure instance to plot on
        self.canvas = FigureCanvas(Figure()) #(figsize=(5, 3)))
        self.axes = self.canvas.figure.subplots(2, 2, sharex=False, sharey=False)

        # this is the Navigation widget
        # it takes the Canvas widget and a parent
##        self.toolbar = NavigationToolbar(self.canvas, self)
        
        # Buttons
        self.breakButton = QPushButton("Emergency break")
        self.restartButton = QPushButton("Restart firmware")
        self.homeXYZButton = QPushButton("Home XYZ")
        self.snapshotButton = QPushButton("Snapshot")
        self.autoFocusButton = QPushButton("AutoFocus")
        self.recordButton = QPushButton("Record")
        self.stageOriginButton = QPushButton("Stage Origin")
        self.runButton = QPushButton("Run")

        # Progress bar
        self.pbar = QProgressBar(self)
        self.pbar.setValue(0)        
        
        # Spinboxes
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
        self.stageYTranslation = QDoubleSpinBox(self)
        self.stageYTranslationTitle = QLabel("Y Translation")
        self.stageYTranslation.setSuffix("mm")
        self.stageYTranslation.setMinimum(0.0)
        self.stageYTranslation.setMaximum(100)
        self.stageYTranslation.setSingleStep(0.1)
        self.stageZTranslation = QDoubleSpinBox(self)
        self.stageZTranslationTitle = QLabel("Z Translation")
        self.stageZTranslation.setSuffix("mm")
        self.stageZTranslation.setMinimum(0.0)
        self.stageZTranslation.setMaximum(50)
        self.stageZTranslation.setSingleStep(0.01)
        
        # Compose layout grid
        self.keyWidgets = [self.stageXTranslationTitle, self.stageYTranslationTitle, self.stageZTranslationTitle, self.lightTitle]
        self.valueWidgets = [self.stageXTranslation, self.stageYTranslation, self.stageZTranslation, self.light]

        widgetLayout = QGridLayout()
        for index, widget in enumerate(self.keyWidgets):
            if widget is not None:
                widgetLayout.addWidget(widget, index, 0, Qt.AlignCenter)
        for index, widget in enumerate(self.valueWidgets):
            if widget is not None:
                widgetLayout.addWidget(widget, index, 1, Qt.AlignCenter)

        widgetLayout.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum,QSizePolicy.Expanding))  # variable space
        widgetLayout.addWidget(self.homeXYZButton,  index+1,0,alignment=Qt.AlignLeft)
        widgetLayout.addWidget(self.stageOriginButton,index+1,1,alignment=Qt.AlignLeft)
        widgetLayout.addWidget(self.autoFocusButton, index+4,0,alignment=Qt.AlignLeft)
        widgetLayout.addWidget(self.snapshotButton, index+3,0,alignment=Qt.AlignLeft)
        widgetLayout.addWidget(self.recordButton,   index+3,1,alignment=Qt.AlignLeft)
        widgetLayout.addWidget(self.runButton,      index+5,0,alignment=Qt.AlignLeft)
        widgetLayout.addWidget(self.pbar,           index+5,1,alignment=Qt.AlignLeft)
        widgetLayout.addWidget(self.breakButton,    index+6,0,alignment=Qt.AlignLeft)
        widgetLayout.addWidget(self.restartButton,  index+6,1,alignment=Qt.AlignLeft)

        widgetLayout.addWidget(QLabel("Processing time [ms]: "),index+7,0,alignment=Qt.AlignLeft)
        widgetLayout.addWidget(self.timerLabel,index+7,1,alignment=Qt.AlignLeft)
        widgetLayout.addWidget(QLabel("Image quality [au]: "),index+8,0,alignment=Qt.AlignLeft)
        widgetLayout.addWidget(self.imageQualityLabel,index+8,1,alignment=Qt.AlignLeft)
##        widgetLayout.addWidget(QLabel("Temperature [°C]: "),index+8,0,alignment=Qt.AlignLeft)
##        widgetLayout.addWidget(self.temperatureLabel,index+8,1,alignment=Qt.AlignLeft)
        
        # Compose final layout
        layout = QHBoxLayout()
        layout.addLayout(widgetLayout, Qt.AlignTop|Qt.AlignCenter)
        layout.addWidget(self.PixImage, Qt.AlignTop|Qt.AlignCenter)
        layout.addWidget(self.canvas, Qt.AlignTop|Qt.AlignCenter)
        self.centralWidget().setLayout(layout)


    @pyqtSlot(int)
    def updateProgressBar(self, val):
        if val >= 0 and val <= 100:
            self.pbar.setValue(val)
        
    @pyqtSlot(np.float)
    def imageQualityUpdate(self, image_quality=None):
        self.imageQualityLabel.setNum(round(image_quality,2))

    def progress_fn(self, n):
        print("%d%% done" % n)


    def msg(self, text):
        if text:
            text = self.__class__.__name__ + ": " + str(text)
            print(text)
            self.signals.message.emit(text)
            

    @pyqtSlot(np.ndarray)
    def update(self, image=None):
        self.kickTimer() # Measure time delay
        if not(image is None):  # we have a new image
            height, width = image.shape[:2]  # get dimensions
            self.image = image if self.image_size[0] == width and self.image_size[1] == height else cv2.resize(image, self.image_size)
            if self.imageScalingFactor > 0 and self.imageScalingFactor < 1:  # Crop the image to create a zooming effect
                height, width = image.shape[:2]  # get dimensions
                delta_height = round(height * (1 - self.imageScalingFactor) / 2)
                delta_width = round(width * (1 - self.imageScalingFactor) / 2)
                image = image[delta_height:height - delta_height, delta_width:width - delta_width]
            height, width = image.shape[:2]  # get dimensions
            if self.imageHeight != height or self.imageWidth != width:  # we need scaling
                scaling_factor = self.imageHeight / float(height)  # get scaling factor
                if self.imageWidth / float(width) < scaling_factor:
                    scaling_factor = self.imageWidth / float(width)
                    image = cv2.resize(image, None, fx=scaling_factor, fy=scaling_factor,
                                       interpolation=cv2.INTER_AREA)  # resize image
            if len(image.shape) < 3:  # check nr of channels
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)  # convert to color image
            height, width = image.shape[:2]  # get dimensions
            qImage = QImage(image.data, width, height, width * 3, QImage.Format_RGB888)  # Convert from OpenCV to PixMap
            self.PixImage.setPixmap(QPixmap(qImage))
            self.PixImage.show()
            

    @pyqtSlot(int, np.ndarray, np.ndarray)
    def updatePlot(self, figType, quadrant, x, y):
        if not (y is None) or not (x is None):
            # select axes
            if quadrant == 1:
                axes = self.axes[0, 1]
            elif quadrant == 2:
                axes = self.axes[0, 0]
            elif quadrant == 3:
                axes = self.axes[1, 0]
            elif quadrant == 4:
                axes = self.axes[1, 1]
            # plot new data
            axes.clear()
            if (figType is FigureTypes.LINEAR) or (figType is None): 
                if x is None:
                    axes.plot(y)
                else:
                    axes.plot(x, y)

            elif figType == FigureTypes.SCATTER:
                if not(x is None):
                    #scatter plot
                    if len(x) >= 100:
                        t = np.arange(0, 100, 1)
                        length = len(x) - 100

                        axes.scatter(t, x[length:len(x)], c="blue", alpha=0.5)
                        # axes.set_xlabel('frame')
                        # axes.set_ylabel('Distance')

            elif figType == FigureTypes.HISTOGRAM:
                #hist
                if not(x is None):
                    if len(x) > 0:
                        axes.hist(x, 30, density=True, facecolor="blue", alpha=0.5)
                    #axes.set_xlabel('Distance')
                    #axes.set_ylabel('Occurance')
            axes.figure.canvas.draw()
            

    @pyqtSlot()
    def disableButtons(self):
        self.homeXYZButton.setEnabled(False)
        self.snapshotButton.setEnabled(False)
        self.recordButton.setEnabled(False)
        self.stageOriginButton.setEnabled(False)
        self.runButton.setEnabled(False)
        self.light.setEnabled(False)
        self.stageXTranslation.setEnabled(False)
        self.stageYTranslation.setEnabled(False)
        self.stageZTranslation.setEnabled(False)
        

    @pyqtSlot()
    def enableButtons(self):
        self.homeXYZButton.setEnabled(True)
        self.snapshotButton.setEnabled(True)
        self.recordButton.setEnabled(True)
        self.runButton.setEnabled(True)
        self.stageOriginButton.setEnabled(True)
        self.light.setEnabled(True)
        self.stageXTranslation.setEnabled(True)
        self.stageYTranslation.setEnabled(True)
        self.stageZTranslation.setEnabled(True)
        
        
    @pyqtSlot()
    def kickTimer(self):
        clockTime = current_milli_time() # datetime.now()
        if self.prevClockTime is not None:
            timeDiff = clockTime - self.prevClockTime
            self.timerLabel.setNum(round(timeDiff)) # Text("Processing time: " + "{:4d}".format(round(timeDiff)) + " ms")
##            self.timerLabel.setText("Processing time: " + "{:4d}".format(round(1000*timeDiff.total_seconds())) + " ms")
        self.prevClockTime = clockTime
        

    @pyqtSlot(np.float)
    def temperatureUpdate(self, temp=None):
        self.temperatureLabel.setNum(round(temp)) # Text("Processing time: " + "{:4d}".format(round(timeDiff)) + " ms")

    def wheelEvent(self, event):
        if (event.angleDelta().y() > 0) and (self.imageScalingFactor > self.imageScalingStep):  # zooming in
            self.imageScalingFactor -= self.imageScalingStep
        elif (event.angleDelta().y() < 0) and (self.imageScalingFactor < 1.0):  # zooming out
            self.imageScalingFactor += self.imageScalingStep        
        self.imageScalingFactor = round(self.imageScalingFactor, 2)  # strange behaviour, so rounding is necessary
        self.update()  # redraw the image with different scaling
        

    def closeEvent(self, event: QCloseEvent):
        self.saveSettings()
        self.signals.finished.emit()
        event.accept()
        

    def snapshot(self):
##        a mutex is needed here?
        if not(self.image is None):
            filename = self.appName + '_'+ str(current_milli_time()) + '.png'
            cv2.imwrite(filename, self.image)
            self.msg("Image written to " + filename)
            

    def loadSettings(self):
        self.msg("info;loading settings from " + self.settings.fileName())
        frame_size_str = self.settings.value('frame_size')
        (width, height) = frame_size_str.split('x')
        self.image_size = (int(width), int(height))
        for index, widget in enumerate(self.keyWidgets):  # retreive all labeled parameters
            if isinstance(widget, QLabel):
                key = "mainwindow/" + widget.text()
                if self.settings.contains(key):
                    self.valueWidgets[index].setValue(float(self.settings.value(key)))
                    

    def saveSettings(self):
        self.msg("info;saving settings to " + self.settings.fileName())
        for index, widget in enumerate(self.keyWidgets):  # save all labeled parameters
            if isinstance(widget, QLabel):
                key = "mainwindow/" + widget.text()
                self.settings.setValue(key, self.valueWidgets[index].value())
        for index, widget in enumerate(self.valueWidgets):  # save all labeled parameters
            if isinstance(widget, QCheckBox):
                key = "mainwindow/" + widget.text()
                self.settings.setValue(key, widget.isChecked())       
