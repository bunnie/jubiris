import sys, toupcam, logging
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QTimer, QSignalBlocker, Qt, QRect
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QLabel, QApplication, QWidget, QDesktopWidget, \
    QCheckBox, QMessageBox, QMainWindow, QPushButton, QComboBox, QSlider, QGroupBox, \
    QGridLayout, QBoxLayout, QHBoxLayout, QVBoxLayout, QMenu, QAction, QSpinBox, \
    QFormLayout

import numpy as np
import qimage2ndarray

from threading import Thread, Event
import queue
from pathlib import Path
import statistics

import os
import cv2
envpath = '/home/bunnie/.local/lib/python3.10/site-packages/cv2/qt/plugins/platforms'
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = envpath

from collections import deque
from math import log10
from datetime import datetime

from piezo import PIEZO_UM_PER_LSB

DEFAULT_LAPLACIAN_5X = 7
DEFAULT_FILTER_5X = 11
DEFAULT_LAPLACIAN_10X = 7 # 9  note: fiddle with focus and see if we can manually use the graph to focus. Params depends on the chip node?
DEFAULT_FILTER_10X = 7    # 17 SET VARIANCE if changing these!
DEFAULT_LAPLACIAN_20X = 15
DEFAULT_FILTER_20X = 21
DEFAULT_LAPLACIAN = None
DEFAULT_FILTER = None

FOCUS_AREA_PX = 1536
GRAPH_WINDOW = 50
USE_GAMMA = False

INIT_EXPO_TIME=60
INIT_EXPO_GAIN=300
INIT_TEMP=6100
INIT_TINT=1880

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

    def __init__(self, mag):
        # defaults based on installed lens
        if mag == 5:
            DEFAULT_FILTER = DEFAULT_FILTER_5X
            DEFAULT_LAPLACIAN = DEFAULT_LAPLACIAN_5X
        elif mag == 10:
            DEFAULT_FILTER = DEFAULT_FILTER_10X
            DEFAULT_LAPLACIAN = DEFAULT_LAPLACIAN_10X
        elif mag == 20:
            DEFAULT_FILTER = DEFAULT_FILTER_20X
            DEFAULT_LAPLACIAN = DEFAULT_LAPLACIAN_20X
            logging.warning("20x values are estimated and need tuning.")

        super().__init__()
        # instance variables
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
        self.focus_queue = None # this must be initialized to a Queue for image queuing code to run
        self.fine_focus_event = None
        self.jubilee_state = None # This is a Queue that contains Poi records
        self.oneshot_expo_print = False
        self.focus_scores = deque([0] * GRAPH_WINDOW, maxlen=GRAPH_WINDOW)
        # track values a little beyond the graph window so we "average in" to new zoom scales
        self.min_y_window = deque([1e50] * int(GRAPH_WINDOW * 0.2), maxlen=int(GRAPH_WINDOW * 0.2))
        self.max_y_window = deque([0.0] * int(GRAPH_WINDOW * 0.2), maxlen=int(GRAPH_WINDOW * 0.2))

        # exposure panel
        self.cbox_auto = QCheckBox()
        self.cbox_auto.setChecked(False)
        self.spinbox_expoTime = QSpinBox()
        self.spinbox_expoTime.setValue(INIT_EXPO_TIME)
        self.spinbox_expoGain = QSpinBox()
        self.spinbox_expoGain.setValue(INIT_EXPO_GAIN)
        self.spinbox_expoTime.setEnabled(True)
        self.spinbox_expoGain.setEnabled(True)
        self.spinbox_expoGain.setSingleStep(50)
        self.spinbox_expoTime.setSingleStep(10)
        exposure_layout = QFormLayout()
        exposure_layout.addRow("Auto Exposure", self.cbox_auto)
        exposure_layout.addRow("Time(ms)", self.spinbox_expoTime)
        exposure_layout.addRow("Gain(%):", self.spinbox_expoGain)
        self.cbox_auto.stateChanged.connect(self.onAutoExpo)
        self.spinbox_expoTime.valueChanged.connect(self.onExpoTime)
        self.spinbox_expoGain.valueChanged.connect(self.onExpoGain)

        # global state control buttons
        self.btn_fine_focus = QPushButton("Fine Focus")
        self.btn_fine_focus.clicked.connect(self.onFineFocus)
        self.btn_snap = QPushButton("Snap")
        self.btn_snap.setEnabled(False)
        self.btn_snap.clicked.connect(self.onBtnSnap)
        self.btn_quit = QPushButton("Quit")
        self.btn_quit.clicked.connect(self.onBtnQuit)
        button_cluster = QVBoxLayout()
        button_cluster.addWidget(self.btn_fine_focus)
        button_cluster.addWidget(self.btn_snap)
        button_cluster.addWidget(self.btn_quit)

        # autofocus image processing
        self.laplacian_spin = QSpinBox()
        self.laplacian_spin.setValue(DEFAULT_LAPLACIAN)
        self.laplacian_spin.setRange(1, 31)
        self.laplacian_spin.setSingleStep(2)
        self.filter_spin = QSpinBox()
        self.filter_spin.setValue(DEFAULT_FILTER)
        self.filter_spin.setRange(1, 31)
        self.filter_spin.setSingleStep(2)
        self.filter_cbox = QCheckBox()
        self.filter_cbox.setChecked(True)
        self.preview_cbox = QCheckBox()
        self.preview_cbox.setChecked(True)
        self.normalize_cbox = QCheckBox()
        self.normalize_cbox.setChecked(False)
        self.focus_variance = QLabel("0.0")
        adjustment_fields_layout = QFormLayout()
        adjustment_fields_layout.addRow("Laplacian", self.laplacian_spin)
        adjustment_fields_layout.addRow("Filter", self.filter_spin)
        adjustment_fields_layout.addRow("Filter Enable", self.filter_cbox)
        adjustment_fields_layout.addRow("Normalize Enable", self.normalize_cbox)
        adjustment_fields_layout.addRow("Raw Preview", self.preview_cbox)
        adjustment_fields_layout.addRow("Focus Variance", self.focus_variance)
        self.laplacian = DEFAULT_LAPLACIAN
        self.laplacian_spin.valueChanged.connect(self.onLaplacian)
        self.filter_value = DEFAULT_FILTER
        self.filter_spin.valueChanged.connect(self.onFilterValue)
        self.filter_enable = True
        self.filter_cbox.stateChanged.connect(self.onFilterState)
        self.raw_preview_enable = True
        self.preview_cbox.stateChanged.connect(self.onPreviewState)
        self.normalize_enable = False
        self.normalize_cbox.stateChanged.connect(self.onNormalizeState)

        # autofocus control
        self.label_x = QLabel("0.0")
        self.label_y = QLabel("0.0")
        self.label_z_offset = QLabel("0.0")
        self.label_piezo_offset = QLabel("0")
        self.label_z_total = QLabel("0.0")
        autofocus_panel_layout = QFormLayout()
        autofocus_panel_layout.addRow("X", self.label_x)
        autofocus_panel_layout.addRow("Y", self.label_y)
        autofocus_panel_layout.addRow("Z-mach", self.label_z_offset)
        autofocus_panel_layout.addRow("Z-piezo", self.label_piezo_offset)
        autofocus_panel_layout.addRow("Z-total", self.label_z_total)

        # fps rating
        self.lbl_frame = QLabel()

        # build a graph area
        self.graph = QLabel()

        # assemble the control panel horizontally
        hlyt_ctrl = QHBoxLayout()
        hlyt_ctrl.addLayout(exposure_layout)
        hlyt_ctrl.addStretch()
        hlyt_ctrl.addLayout(autofocus_panel_layout)
        hlyt_ctrl.addLayout(adjustment_fields_layout)
        hlyt_ctrl.addWidget(self.graph)
        hlyt_ctrl.addStretch()
        hlyt_ctrl.addLayout(button_cluster)
        wg_ctrl = QWidget()
        wg_ctrl.setLayout(hlyt_ctrl)
        wg_ctrl.setMinimumHeight(500)

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
        grid_main.setRowStretch(0, 4) # video is on row 0, have it try to be as big as possible
        grid_main.addWidget(v_widget)
        grid_main.addWidget(self.lbl_frame, 1, 0) # optional?
        grid_main.addWidget(wg_ctrl, 3, 0)
        w_main = QWidget()
        w_main.setLayout(grid_main)
        self.setCentralWidget(w_main)

        self.timer.timeout.connect(self.onTimer)
        self.evtCallback.connect(self.onevtCallback)

    # these must be odd numbers, so we wrap them in a function to ensure that
    def get_laplacian(self):
        return self.laplacian
    def get_filter_value(self):
        return self.filter_value
    def onLaplacian(self, value):
        self.laplacian = value
    def onFilterValue(self, value):
        self.filter_value = value
    def onFilterState(self, state):
        self.filter_enable = state
    def onPreviewState(self, state):
        self.raw_preview_enable = state
    def onNormalizeState(self, state):
        self.normalize_enable = state
    def onFineFocus(self):
        self.fine_focus_event.set()

    def draw_graph(self, data, w=500, h=500):
        MARGIN = 0.1
        MIN_Y_RANGE = 500
        canvas = np.full((h, w, 3), 255, dtype=np.uint8)
        min_y = min(data)
        self.min_y_window.popleft()
        self.min_y_window.append(min_y)
        max_y = max(data)
        self.max_y_window.popleft()
        self.max_y_window.append(max_y)

        min_y = min(self.min_y_window)
        max_y = max(self.max_y_window)
        if max_y - min_y < MIN_Y_RANGE:
            delta = MIN_Y_RANGE - (max_y - min_y)
            max_y += delta / 2.0
            min_y -= delta / 2.0

        scale = (h * (1 - MARGIN * 2)) / (max_y - min_y)
        offset = MARGIN * h
        x_tick = w * (1 - MARGIN * 2) / (len(data) + 1)

        last_x = w * MARGIN
        last_y = h - (offset + (data[0] - min_y) * scale)
        thickness = 2
        color = (180, 40, 60)
        for y in data[1:]:
            cur_x = last_x + x_tick
            cur_y = h - (offset + (y - min_y) * scale)
            cv2.line(canvas, (int(last_x), int(last_y)), (int(cur_x), int(cur_y)), color, thickness)
            last_x = cur_x
            last_y = cur_y
        # add legends for Y-axis
        cv2.putText(
            canvas,
            f"{min_y:0.4f}",
            (int(offset), int(h-offset)),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (128, 128, 128),
            bottomLeftOrigin = False,
            thickness=2,
        )
        cv2.putText(
            canvas,
            f"{max_y:0.4f}",
            (int(offset), int(offset*2)),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (128, 128, 128),
            bottomLeftOrigin = False,
            thickness=2,
        )
        return canvas

    def onTimer(self):
        if self.hcam:
            nFrame, nTime, nTotalFrame = self.hcam.get_FrameRate()
            self.lbl_frame.setText("{}, fps = {:.1f}".format(nTotalFrame, nFrame * 1000.0 / nTime))

    def closeCamera(self):
        if self.hcam:
            self.hcam.Close()
        self.hcam = None
        self.pData = None

        self.timer.stop()
        self.lbl_frame.clear()
        self.cbox_auto.setEnabled(False)
        self.spinbox_expoGain.setEnabled(False)
        self.spinbox_expoTime.setEnabled(False)
        self.btn_snap.setEnabled(False)

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
            self.spinbox_expoTime.setEnabled(not state)
            self.spinbox_expoGain.setEnabled(not state)

    def onExpoTime(self, value):
        if self.hcam:
            if not self.cbox_auto.isChecked():
                self.hcam.put_ExpoTime(value * 1000)

    def onExpoGain(self, value):
        if self.hcam:
            if not self.cbox_auto.isChecked():
                self.hcam.put_ExpoAGain(value)

    def startCamera(self):
        self.pData = bytes(toupcam.TDIBWIDTHBYTES(self.imgWidth * 32) * self.imgHeight)
        uimin, uimax, uidef = self.hcam.get_ExpTimeRange()
        self.spinbox_expoTime.setRange(uimin // 1000, uimax // 1000)
        self.spinbox_expoTime.setValue(uidef // 1000)
        usmin, usmax, usdef = self.hcam.get_ExpoAGainRange()
        self.spinbox_expoGain.setRange(usmin, usmax)
        self.spinbox_expoGain.setValue(usdef)

        try:
            self.hcam.StartPullModeWithCallback(self.eventCallBack, self)
        except toupcam.HRESULTException:
            self.closeCamera()
            QMessageBox.warning(self, "Warning", "Failed to start camera.")
        else:
            self.cbox_auto.setEnabled(True)
            self.btn_snap.setEnabled(True)
            self.cbox_auto.setChecked(False)
            self.handleExpoEvent()

            self.timer.start(1000)

    def openCamera(self):
        self.hcam = toupcam.Toupcam.Open(self.cur.id)
        if self.hcam:
            self.res = self.hcam.get_eSize()
            self.imgWidth = self.cur.model.res[self.res].width
            self.imgHeight = self.cur.model.res[self.res].height
            self.hcam.put_Option(toupcam.TOUPCAM_OPTION_BYTEORDER, 0) #Qimage use RGB byte order
            self.hcam.put_AutoExpoEnable(0)
            self.startCamera()

            # set some defaults
            self.spinbox_expoGain.setValue(INIT_EXPO_GAIN)
            self.spinbox_expoTime.setValue(INIT_EXPO_TIME)
            self.handleExpoEvent()
            # peg the tint at a sane default and leave it there.
            self.hcam.put_TempTint(INIT_TEMP, INIT_TINT)

    def onBtnQuit(self):
        if self.hcam:
            self.closeCamera()
        self.close()

    def setupCamera(self):
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
            profiling = False
            if profiling:
                start = datetime.now()

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
            if USE_GAMMA: # turn off gamma for now - we need the performance to do the focus routine, and we haven't used gamma in a while
                if self.gamma.gamma == 1.0:
                    # bypass computations if it's 1.0
                    self.lbl_video.setPixmap(QPixmap.fromImage(newimage))
                else:
                    gammaimage = qimage2ndarray.array2qimage(
                        adjust_gamma(qimage2ndarray.rgb_view(newimage), self.gamma.gamma))
                    self.lbl_video.setPixmap(QPixmap.fromImage(
                        gammaimage.copy(QRect(0, 0, newimage_bounds.width(), newimage_bounds.height()))
                    ))
            else:
                self.lbl_video.setPixmap(QPixmap.fromImage(newimage))

            # extract a full-res preview image of the focus area
            w = qr.width()
            h = qr.height()
            if self.image_name is None:
                centerimage = image.copy(w//2 - FOCUS_AREA_PX//2, h//2 - FOCUS_AREA_PX//2, FOCUS_AREA_PX, FOCUS_AREA_PX)
                centerimage_rect = centerimage.rect()
            else:
                self.image_name.w = w
                self.image_name.h = h
                centerimage_rect = self.image_name.get_focus_rect()
                centerimage = image.copy(centerimage_rect)

            # add focus rectangle mark
            # painter = QPainter(centerimage)
            # painter.setPen(QPen(QColor(128, 255, 128)))
            # painter.drawRect(focus_rect)
            # painter.end()

            if profiling:
                logging.info(f"before: {datetime.now() - start}")
                start = datetime.now()
            # autofocus mechanisms
            if False:
                if self.image_queue is not None:
                    if not self.image_queue.full():
                        cv2_image = qimage2ndarray.rgb_view(image)
                        self.image_queue.put(cv2_image)
                    else:
                        logging.debug("Image queue overflow, image dropped")
                if self.ui_queue is not None:
                    try:
                        ui_img = self.ui_queue.get(block=False)
                    except queue.Empty:
                        pass
                    else:
                        cv2.imshow("focus", ui_img)

            cv2_image = cv2.cvtColor(qimage2ndarray.rgb_view(centerimage), cv2.COLOR_RGB2GRAY)
            if self.filter_enable:
                cv2_image = cv2.GaussianBlur(cv2_image, (self.get_filter_value(), self.get_filter_value()), 0)
                # cv2_image = cv2.medianBlur(cv2_image, self.get_filter_value())
            if self.normalize_enable:
                #cv2_image = cv2.normalize(cv2_image, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                cv2_image = cv2.normalize(cv2_image, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            laplacian = cv2.Laplacian(cv2_image, -1, ksize=self.get_laplacian())
            self.focus_scores.popleft()
            try:
                score = abs(laplacian.var())
                self.focus_scores.append(score)
                if not self.focus_queue.full():
                    self.focus_queue.put((datetime.now(), score, self.spinbox_expoGain.value(), self.spinbox_expoTime.value()), block=False)
                self.focus_variance.setText(f"{statistics.stdev(list(self.focus_scores)):0.2f}")
            except ValueError:
                logging.debug("Laplacian had 0 variance, inserting bogus value for focus")
                self.focus_scores.append(0)
            if profiling:
                logging.info(f"laplacian: {datetime.now() - start}")
                start = datetime.now()

            graph = self.draw_graph(list(self.focus_scores))
            self.graph.setPixmap(QPixmap.fromImage(
                QImage(graph, graph.shape[1], graph.shape[0], graph.shape[1] * 3, QImage.Format_RGB888)
            ))
            if profiling:
                logging.info(f"graph: {datetime.now() - start}")

            # update the image
            if USE_GAMMA:
                if self.gamma.gamma == 1.0:
                    self.lbl_fullres.setPixmap(QPixmap.fromImage(centerimage))
                else:
                    gamma_centerimage = qimage2ndarray.array2qimage(
                        adjust_gamma(qimage2ndarray.rgb_view(centerimage), self.gamma.gamma))
                    self.lbl_fullres.setPixmap(QPixmap.fromImage(
                        gamma_centerimage.copy(QRect(0, 0, centerimage_rect.width(), centerimage_rect.height()))
                        ))
            else:
                if self.raw_preview_enable:
                    self.lbl_fullres.setPixmap(QPixmap.fromImage(centerimage))
                else:
                    #lap_u8 = cv2.normalize(laplacian, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    self.lbl_fullres.setPixmap(QPixmap.fromImage(qimage2ndarray.array2qimage(laplacian, normalize=True)))

            # update machine state UI
            try:
                poi = None
                while not self.jubilee_state.empty(): # drain the POI queue entirely, keeping only the latest value
                    poi = self.jubilee_state.get(block = False)
                if poi is not None:
                    update_z = False
                    if poi.piezo:
                        self.label_piezo_offset.setText(f"{poi.piezo}({(poi.piezo * PIEZO_UM_PER_LSB):0.2f}um)")
                        update_z = True
                    if poi.z:
                        self.label_z_offset.setText(f"{poi.z:0.2f}mm")
                        update_z = True
                    if poi.x:
                        self.label_x.setText(f"{poi.x:0.2f}mm")
                    if poi.y:
                        self.label_y.setText(f"{poi.y:0.2f}mm")
                    if update_z:
                        if poi.piezo and poi.z:
                            self.label_z_total.setText(f"{(poi.z - (poi.piezo * PIEZO_UM_PER_LSB) / 1000.0):0.4f}mm")
                        else:
                            logging.warning(f"Tried to update total Z text but something was None: piezo {poi.piezo}, z {poi.z}")
            except queue.Empty:
                pass


    def handleExpoEvent(self):
        time = self.hcam.get_ExpoTime()
        gain = self.hcam.get_ExpoAGain()
        with QSignalBlocker(self.spinbox_expoTime):
            self.spinbox_expoTime.setValue(time // 1000)
        with QSignalBlocker(self.spinbox_expoGain):
            self.spinbox_expoGain.setValue(gain)

    def handleStillImageEvent(self):
        curtime_us = self.hcam.get_ExpoTime()
        curgain = self.hcam.get_ExpoAGain()
        # Set gain to 100 for minumum noise
        new_gain = 100
        # compensate for lower gain with longer exposure time
        new_exp = int(2.0 * curtime_us * (curgain / new_gain))
        if not self.oneshot_expo_print:
            logging.info(f"Capture at gain={new_gain}, exp={new_exp}")
            self.oneshot_expo_print = True

        self.hcam.put_ExpoAGain(new_gain)
        self.hcam.put_ExpoTime(new_exp)
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
                        self.hcam.put_ExpoTime(curtime_us)
                        logging.debug(f"Revert gain={curgain}, exp={curtime_us}")
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
                            self.hcam.put_ExpoTime(curtime_us)
                        else:
                            logging.error("Image name was not fully initalized, cannot save!")
                        self.single_snap_done.set()


def snapper(w, auto_snap_event, auto_snap_done):
    while True:
        auto_snap_event.wait()
        auto_snap_event.clear()
        if w.image_name.quit:
            break
        # throw away the first rep because the exposure is off
        rep = 2
        for i in range(rep):
            if i == 0:
                w.image_name.dummy = True
            else:
                w.image_name.dummy = False
            w.image_name.cur_rep = w.image_name.rep
            w.hcam.Snap(0)
            w.single_snap_done.wait()
            w.single_snap_done.clear()
        auto_snap_done.set()
    # close the main window upon reaching quit
    w.closeCamera()
    w.close()


def cam(cam_quit, gamma, image_name,
        auto_snap_event, auto_snap_done,
        focus_queue, mag, jubilee_state, fine_focus_event):
    app = QApplication(sys.argv)
    w = MainWindow(mag)
    w.image_name = image_name
    w.setGeometry(50, 500, 2200, 3000)
    w.show()
    w.gamma = gamma
    # auto-open the camera on boot
    w.setupCamera()
    w.single_snap_done = Event()
    w.focus_queue = focus_queue
    w.jubilee_state = jubilee_state
    w.fine_focus_event = fine_focus_event

    # Run a thread to forward/manage snapshotting events
    b = Thread(target=snapper, args=[w, auto_snap_event, auto_snap_done])
    b.start()

    # run the application. execution blocks at this line, until app quits
    ret = app.exec_()
    cam_quit.set()
    logging.debug("UI closed, quit Event set")
    return ret