import sys, toupcam, logging
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QTimer, QSignalBlocker, Qt, QRect
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QLabel, QApplication, QWidget, QDesktopWidget, QCheckBox, QMessageBox, QMainWindow, QPushButton, QComboBox, QSlider, QGroupBox, QGridLayout, QBoxLayout, QHBoxLayout, QVBoxLayout, QMenu, QAction

import numpy as np
import cv2
import qimage2ndarray

from threading import Thread, Event
from pathlib import Path
import time

def adjust_gamma(image, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)

class MainWindow(QMainWindow):
    evtCallback = pyqtSignal(int)

    @staticmethod
    def makeLayout(lbl_1, sli_1, val_1, lbl_2, sli_2, val_2):
        hlyt_1 = QHBoxLayout()
        hlyt_1.addWidget(lbl_1)
        hlyt_1.addStretch()
        hlyt_1.addWidget(val_1)
        hlyt_2 = QHBoxLayout()
        hlyt_2.addWidget(lbl_2)
        hlyt_2.addStretch()
        hlyt_2.addWidget(val_2)
        vlyt = QVBoxLayout()
        vlyt.addLayout(hlyt_1)
        vlyt.addWidget(sli_1)
        vlyt.addLayout(hlyt_2)
        vlyt.addWidget(sli_2)
        return vlyt

    def __init__(self):
        super().__init__()
        self.setMinimumSize(1400, 1600)
        self.image_name = None
        self.hcam = None
        self.timer = QTimer(self)
        self.imgWidth = 0
        self.imgHeight = 0
        self.pData = None
        self.res = 0
        self.temp = toupcam.TOUPCAM_TEMP_DEF
        self.tint = toupcam.TOUPCAM_TINT_DEF
        self.count = 0

        #gbox_res = QGroupBox("Resolution")
        #self.cmb_res = QComboBox()
        #self.cmb_res.setEnabled(False)
        #vlyt_res = QVBoxLayout()
        #vlyt_res.addWidget(self.cmb_res)
        #gbox_res.setLayout(vlyt_res)
        #self.cmb_res.currentIndexChanged.connect(self.onResolutionChanged)

        gbox_exp = QGroupBox("Exposure")
        self.cbox_auto = QCheckBox()
        self.cbox_auto.setEnabled(False)
        lbl_auto = QLabel("Auto exposure")
        hlyt_auto = QHBoxLayout()
        hlyt_auto.addWidget(self.cbox_auto)
        hlyt_auto.addWidget(lbl_auto)
        hlyt_auto.addStretch()
        lbl_time = QLabel("Time(us):")
        lbl_gain = QLabel("Gain(%):")
        self.lbl_expoTime = QLabel("0")
        self.lbl_expoGain = QLabel("0")
        self.slider_expoTime = QSlider(Qt.Horizontal)
        self.slider_expoGain = QSlider(Qt.Horizontal)
        self.slider_expoTime.setEnabled(False)
        self.slider_expoGain.setEnabled(False)
        vlyt_exp = QVBoxLayout()
        vlyt_exp.addLayout(hlyt_auto)
        vlyt_exp.addLayout(self.makeLayout(lbl_time, self.slider_expoTime, self.lbl_expoTime, lbl_gain, self.slider_expoGain, self.lbl_expoGain))
        gbox_exp.setLayout(vlyt_exp)
        self.cbox_auto.stateChanged.connect(self.onAutoExpo)
        self.slider_expoTime.valueChanged.connect(self.onExpoTime)
        self.slider_expoGain.valueChanged.connect(self.onExpoGain)

        gbox_wb = QGroupBox("White balance")
        self.btn_autoWB = QPushButton("White balance")
        self.btn_autoWB.setEnabled(False)
        self.btn_autoWB.clicked.connect(self.onAutoWB)
        lbl_temp = QLabel("Temperature:")
        lbl_tint = QLabel("Tint:")
        self.lbl_temp = QLabel(str(toupcam.TOUPCAM_TEMP_DEF))
        self.lbl_tint = QLabel(str(toupcam.TOUPCAM_TINT_DEF))
        self.slider_temp = QSlider(Qt.Horizontal)
        self.slider_tint = QSlider(Qt.Horizontal)
        self.slider_temp.setRange(toupcam.TOUPCAM_TEMP_MIN, toupcam.TOUPCAM_TEMP_MAX)
        self.slider_temp.setValue(toupcam.TOUPCAM_TEMP_DEF)
        self.slider_tint.setRange(toupcam.TOUPCAM_TINT_MIN, toupcam.TOUPCAM_TINT_MAX)
        self.slider_tint.setValue(toupcam.TOUPCAM_TINT_DEF)
        self.slider_temp.setEnabled(False)
        self.slider_tint.setEnabled(False)
        vlyt_wb = QVBoxLayout()
        vlyt_wb.addLayout(self.makeLayout(lbl_temp, self.slider_temp, self.lbl_temp, lbl_tint, self.slider_tint, self.lbl_tint))
        vlyt_wb.addWidget(self.btn_autoWB)
        gbox_wb.setLayout(vlyt_wb)
        self.slider_temp.valueChanged.connect(self.onWBTemp)
        self.slider_tint.valueChanged.connect(self.onWBTint)

        self.btn_open = QPushButton("Open")
        self.btn_open.clicked.connect(self.onBtnOpen)
        self.btn_snap = QPushButton("Snap")
        self.btn_snap.setEnabled(False)
        self.btn_snap.clicked.connect(self.onBtnSnap)

        self.lbl_frame = QLabel()

        hlyt_ctrl = QHBoxLayout()
        #hlyt_ctrl.addWidget(gbox_res)
        hlyt_ctrl.addWidget(gbox_exp)
        hlyt_ctrl.addWidget(gbox_wb)
        hlyt_ctrl.addWidget(self.btn_open)
        hlyt_ctrl.addWidget(self.btn_snap)
        hlyt_ctrl.addStretch()
        wg_ctrl = QWidget()
        wg_ctrl.setLayout(hlyt_ctrl)

        # Place overview and preview in the same V-box
        self.lbl_video = QLabel()
        self.lbl_fullres = QLabel()
        v_video = QVBoxLayout()
        v_video.addWidget(self.lbl_video)
        v_video.addWidget(self.lbl_fullres)
        v_widget = QWidget()
        v_widget.setLayout(v_video)
        self.v_video = v_video

        grid_main = QGridLayout()
        grid_main.setRowStretch(0, 10) # video is on row 0, have it try to be as big as possible
        grid_main.addWidget(v_widget)
        grid_main.addWidget(self.lbl_frame, 1, 0) # optional?
        grid_main.addWidget(wg_ctrl, 3, 0)
        w_main = QWidget()
        w_main.setLayout(grid_main)
        self.setCentralWidget(w_main)

        self.timer.timeout.connect(self.onTimer)
        self.evtCallback.connect(self.onevtCallback)

    def onTimer(self):
        if self.hcam:
            nFrame, nTime, nTotalFrame = self.hcam.get_FrameRate()
            self.lbl_frame.setText("{}, fps = {:.1f}".format(nTotalFrame, nFrame * 1000.0 / nTime))

    def closeCamera(self):
        if self.hcam:
            self.hcam.Close()
        self.hcam = None
        self.pData = None

        self.btn_open.setText("Open")
        self.timer.stop()
        self.lbl_frame.clear()
        self.cbox_auto.setEnabled(False)
        self.slider_expoGain.setEnabled(False)
        self.slider_expoTime.setEnabled(False)
        self.btn_autoWB.setEnabled(False)
        self.slider_temp.setEnabled(False)
        self.slider_tint.setEnabled(False)
        self.btn_snap.setEnabled(False)
        #self.cmb_res.setEnabled(False)
        #self.cmb_res.clear()

    def closeEvent(self, event):
        self.closeCamera()

    def onResolutionChanged(self, index):
        if self.hcam: #step 1: stop camera
            self.hcam.Stop()

        self.res = index
        self.imgWidth = self.cur.model.res[index].width
        self.imgHeight = self.cur.model.res[index].height

        if self.hcam: #step 2: restart camera
            self.hcam.put_eSize(self.res)
            self.startCamera()

    def onAutoExpo(self, state):
        if self.hcam:
            self.hcam.put_AutoExpoEnable(1 if state else 0)
            self.slider_expoTime.setEnabled(not state)
            self.slider_expoGain.setEnabled(not state)

    def onExpoTime(self, value):
        if self.hcam:
            self.lbl_expoTime.setText(str(value))
            if not self.cbox_auto.isChecked():
                self.hcam.put_ExpoTime(value)

    def onExpoGain(self, value):
        if self.hcam:
            self.lbl_expoGain.setText(str(value))
            if not self.cbox_auto.isChecked():
                self.hcam.put_ExpoAGain(value)

    def onAutoWB(self):
        if self.hcam:
            self.hcam.AwbOnce()

    def wbCallback(nTemp, nTint, self):
        self.slider_temp.setValue(nTemp)
        self.slider_tint.setValue(nTint)

    def onWBTemp(self, value):
        if self.hcam:
            self.temp = value
            self.hcam.put_TempTint(self.temp, self.tint)
            self.lbl_temp.setText(str(value))

    def onWBTint(self, value):
        if self.hcam:
            self.tint = value
            self.hcam.put_TempTint(self.temp, self.tint)
            self.lbl_tint.setText(str(value))

    def startCamera(self):
        self.pData = bytes(toupcam.TDIBWIDTHBYTES(self.imgWidth * 32) * self.imgHeight)
        uimin, uimax, uidef = self.hcam.get_ExpTimeRange()
        self.slider_expoTime.setRange(uimin, uimax)
        #self.slider_expoTime.setValue(uidef)
        usmin, usmax, usdef = self.hcam.get_ExpoAGainRange()
        self.slider_expoGain.setRange(usmin, usmax)
        #self.slider_expoGain.setValue(usdef)
        if self.cur.model.flag & toupcam.TOUPCAM_FLAG_MONO == 0:
            self.handleTempTintEvent()
        try:
            self.hcam.StartPullModeWithCallback(self.eventCallBack, self)
        except toupcam.HRESULTException:
            self.closeCamera()
            QMessageBox.warning(self, "Warning", "Failed to start camera.")
        else:
            #self.cmb_res.setEnabled(True)
            self.cbox_auto.setEnabled(True)
            self.btn_autoWB.setEnabled(True)
            self.slider_temp.setEnabled(True)
            self.slider_tint.setEnabled(True)

            self.btn_open.setText("Quit")
            self.btn_snap.setEnabled(True)
            # bAuto = self.hcam.get_AutoExpoEnable()
            self.cbox_auto.setChecked(False)
            self.btn_autoWB.setChecked(False)
            self.handleExpoEvent()

            self.timer.start(1000)
        
    def openCamera(self):
        self.hcam = toupcam.Toupcam.Open(self.cur.id)
        if self.hcam:
            self.res = self.hcam.get_eSize()
            self.imgWidth = self.cur.model.res[self.res].width
            self.imgHeight = self.cur.model.res[self.res].height
            # with QSignalBlocker(self.cmb_res):
            #     self.cmb_res.clear()
            #     for i in range(0, self.cur.model.preview):
            #         self.cmb_res.addItem("{}*{}".format(self.cur.model.res[i].width, self.cur.model.res[i].height))
            #     self.cmb_res.setCurrentIndex(self.res)
            #     self.cmb_res.setEnabled(True)
            self.hcam.put_Option(toupcam.TOUPCAM_OPTION_BYTEORDER, 0) #Qimage use RGB byte order
            self.hcam.put_AutoExpoEnable(0)
            self.startCamera()

            # set some defaults
            INIT_EXPO_TIME=160_000
            INIT_EXPO_GAIN=300
            INIT_TEMP=6325
            INIT_TINT=1907
            self.slider_temp.setValue(INIT_TEMP)
            self.slider_tint.setValue(INIT_TINT)
            self.slider_expoGain.setValue(INIT_EXPO_GAIN)
            self.slider_expoTime.setValue(INIT_EXPO_TIME)
            self.handleExpoEvent()
            self.handleTempTintEvent()

    def onBtnOpen(self):
        if self.hcam:
            self.closeCamera()
            self.close()
        else:
            arr = toupcam.Toupcam.EnumV2()
            if 0 == len(arr):
                QMessageBox.warning(self, "Warning", "No camera found.")
            elif 1 == len(arr):
                self.cur = arr[0]
                self.openCamera()
            else:
                menu = QMenu()
                for i in range(0, len(arr)):
                    action = QAction(arr[i].displayname, self)
                    action.setData(i)
                    menu.addAction(action)
                action = menu.exec(self.mapToGlobal(self.btn_open.pos()))
                if action:
                    self.cur = arr[action.data()]
                    self.openCamera()

    def onBtnSnap(self):
        if self.hcam:
            # if 0 == self.cur.model.still:    # not support still image capture
            #     if self.pData is not None:
            #         image = QImage(self.pData, self.imgWidth, self.imgHeight, QImage.Format_RGB888)
            #         self.count += 1
            #         image.save("pyqt{}.jpg".format(self.count))
            # else:
                # menu = QMenu()
                # for i in range(0, self.cur.model.still):
                #     action = QAction("{}*{}".format(self.cur.model.res[i].width, self.cur.model.res[i].height), self)
                #     action.setData(i)
                #     menu.addAction(action)
                # action = menu.exec(self.mapToGlobal(self.btn_snap.pos()))

            self.hcam.Snap(0) # argument is the resolution of the snap

    @staticmethod
    def eventCallBack(nEvent, self):
        '''callbacks come from toupcam.dll/so internal threads, so we use qt signal to post this event to the UI thread'''
        self.evtCallback.emit(nEvent)

    def onevtCallback(self, nEvent):
        '''this run in the UI thread'''
        if self.hcam:
            if toupcam.TOUPCAM_EVENT_IMAGE == nEvent:
                self.handleImageEvent()
            elif toupcam.TOUPCAM_EVENT_EXPOSURE == nEvent:
                self.handleExpoEvent()
                # manual exposure only
                pass
            elif toupcam.TOUPCAM_EVENT_TEMPTINT == nEvent:
                self.handleTempTintEvent()
            elif toupcam.TOUPCAM_EVENT_STILLIMAGE == nEvent:
                self.handleStillImageEvent()
            elif toupcam.TOUPCAM_EVENT_ERROR == nEvent:
                self.closeCamera()
                QMessageBox.warning(self, "Warning", "Generic Error.")
            elif toupcam.TOUPCAM_EVENT_STILLIMAGE == nEvent:
                self.closeCamera()
                QMessageBox.warning(self, "Warning", "Camera disconnect.")

    def handleImageEvent(self):
        try:
            self.hcam.PullImageV3(self.pData, 0, 32, 0, None)
        except toupcam.HRESULTException:
            pass
        else:
            image = QImage(self.pData, self.imgWidth, self.imgHeight, QImage.Format_RGBX8888)

            # extract bounds of the original image (before OpenCV processing)
            qr = image.rect()
            assert(qr.width() > 1920) # just check that we're in the higher resolution mode
            # extract bounds from the enclosing V-box
            video_bounds = self.v_video.geometry()
            # cache video bounds because for some reason Qt sets it to 0 sometimes??
            if video_bounds.width() != 0:
                self.cached_bounds = video_bounds
            else:
                video_bounds = self.cached_bounds
            target_width = video_bounds.width()
            target_height = video_bounds.height() // 2

            # scale, preserving aspect ratio
            newimage = image.scaled(target_width, target_height, Qt.KeepAspectRatio, Qt.FastTransformation)
            newimage_bounds = newimage.rect()
            # newimage = image.scaledToHeight(target_height, Qt.FastTransformation)

            # gamma adjust - run this on the down-scaled image, our CPU is not fast enough to handle 4K
            if self.gamma.gamma == 1.0:
                # bypass computations if it's 1.0
                self.lbl_video.setPixmap(QPixmap.fromImage(newimage))
            else:
                gammaimage = qimage2ndarray.array2qimage(
                    adjust_gamma(qimage2ndarray.rgb_view(newimage), self.gamma.gamma))
                self.lbl_video.setPixmap(QPixmap.fromImage(
                    gammaimage.copy(QRect(0, 0, newimage_bounds.width(), newimage_bounds.height()))
                ))

            # extract a full-res preview image of the centroid
            w = qr.width()
            h = qr.height()
            dest_w = self.lbl_fullres.width()
            dest_h = self.lbl_fullres.height()

            # handle case of screen res higher than source image
            if dest_w > w:
                dest_w = w
            if dest_h > h:
                dest_h = h
            crop_rect = QRect(w//2 - dest_w//2, h//2 - dest_h//2, w//2 + dest_w//2, h//2 + dest_h//2)
            centerimage = image.copy(crop_rect)
            centerimage_rect = centerimage.rect()
            if self.gamma.gamma == 1.0:
                self.lbl_fullres.setPixmap(QPixmap.fromImage(centerimage))
            else:
                gamma_centerimage = qimage2ndarray.array2qimage(
                    adjust_gamma(qimage2ndarray.rgb_view(centerimage), self.gamma.gamma))
                self.lbl_fullres.setPixmap(QPixmap.fromImage(
                    gamma_centerimage.copy(QRect(0, 0, centerimage_rect.width(), centerimage_rect.height()))
                    ))

    def handleExpoEvent(self):
        return
        time = self.hcam.get_ExpoTime()
        gain = self.hcam.get_ExpoAGain()
        with QSignalBlocker(self.slider_expoTime):
            self.slider_expoTime.setValue(time)
        with QSignalBlocker(self.slider_expoGain):
            self.slider_expoGain.setValue(gain)
        self.lbl_expoTime.setText(str(time))
        self.lbl_expoGain.setText(str(gain))

    def handleTempTintEvent(self):
        nTemp, nTint = self.hcam.get_TempTint()
        with QSignalBlocker(self.slider_temp):
            self.slider_temp.setValue(nTemp)
        with QSignalBlocker(self.slider_tint):
            self.slider_tint.setValue(nTint)
        self.lbl_temp.setText(str(nTemp))
        self.lbl_tint.setText(str(nTint))

    def handleStillImageEvent(self):
        curtime = self.hcam.get_ExpoTime()
        curgain = self.hcam.get_ExpoAGain()
        new_gain = 100
        self.hcam.put_ExpoAGain(new_gain)
        new_exp = int(2.0 * curtime * (curgain / new_gain))
        self.hcam.put_ExpoTime(new_exp)
        logging.debug(f"Capture at gain={new_gain}, exp={new_exp}")
        info = toupcam.ToupcamFrameInfoV3()
        try:
            self.hcam.PullImageV3(None, 1, 32, 0, info) # peek
        except toupcam.HRESULTException:
            pass
        else:
            if info.width > 0 and info.height > 0:
                buf = bytes(toupcam.TDIBWIDTHBYTES(info.width * 32) * info.height)
                try:
                    self.hcam.PullImageV3(buf, 1, 32, 0, info)
                except toupcam.HRESULTException:
                    pass
                else:
                    if self.image_name is None:
                        image = QImage(buf, info.width, info.height, QImage.Format_RGBX8888)

                        self.count += 1
                        # image.save("pyqt{}.png".format(self.count))
                        image.save("pyqt{}.jpg".format(self.count), None, 90)

                        if self.gamma.gamma != 1.0:
                            gamma_image = qimage2ndarray.array2qimage(
                                adjust_gamma(qimage2ndarray.rgb_view(image), self.gamma.gamma))
                            gamma_image.save("pyqt-gamma{}.jpg".format(self.count), None, 90)

                        self.hcam.put_ExpoAGain(curgain)
                        self.hcam.put_ExpoTime(curtime)
                        logging.debug(f"Revert gain={curgain}, exp={curtime}")
                    else:
                        if self.image_name.is_init():
                            # ensure that the target directory exists
                            Path(self.image_name.name).mkdir(exist_ok=True)
                            np_array = np.frombuffer(buf, dtype=np.uint8)
                            np_array = np_array.reshape((info.height, info.width, 4))
                            grayscale = cv2.cvtColor(np_array, cv2.COLOR_RGBA2GRAY)
                            fname = self.image_name.name + '/' + self.image_name.get_name() + '.png'
                            logging.info(f"Writing {fname}")
                            cv2.imwrite(fname, grayscale, [cv2.IMWRITE_PNG_COMPRESSION, 5])
                            logging.debug("done")
                            if self.image_name.dummy:
                                # For some reason, the file has to be *written* for the exposure to set
                                # a dummy delay doesn't do it. Talk about spookey side effects...
                                logging.debug("Deleting")
                                p = Path(fname)
                                p.unlink(missing_ok=True)
                            self.hcam.put_ExpoAGain(curgain)
                            self.hcam.put_ExpoTime(curtime)
                        else:
                            logging.error("Image name was not fully initalized, cannot save!")
                        self.single_snap_done.set()


def snapper(w, image_name, auto_snap_event, auto_snap_done):
    while True:
        auto_snap_event.wait()
        auto_snap_event.clear()
        if image_name.quit:
            break
        w.image_name = image_name
        if image_name.rep is not None:
            rep = int(image_name.rep) + 1 # +1 for the dummy exposure
        else:
            rep = 2
        for i in range(rep):
            if i == 0:
                w.image_name.dummy = True
            else:
                w.image_name.dummy = False
            w.image_name.cur_rep = i
            w.hcam.Snap(0)
            w.single_snap_done.wait()
            w.single_snap_done.clear()
            w.image_name.cur_rep = None
        w.image_name = None
        auto_snap_done.set()


def cam(cam_quit, gamma, image_name, auto_snap_event, auto_snap_done):
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    w.gamma = gamma
    # auto-open the camera on boot
    w.btn_open.click()
    w.single_snap_done = Event()

    # Run a thread to forward/manage snapshotting events
    b = Thread(target=snapper, args=[w, image_name, auto_snap_event, auto_snap_done])
    b.start()

    # run the application. execution blocks at this line, until app quits
    ret = app.exec_()
    cam_quit.set()
    logging.debug("UI closed, quit Event set")
    return ret