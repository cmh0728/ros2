# -*- coding: utf-8 -*-
# Camera thread tuned for RealSense RGB on /dev/video4 (YUYV), low-latency streaming

import os
import cv2
import glob
import threading
import base64
import time
import numpy as np

from src.utils.messages.allMessages import (
    mainCamera,
    serialCamera,
    Recording,
    Record,
    Brightness,
    Contrast,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.templates.threadwithstop import ThreadWithStop


def _to_int_device(path_or_index):
    if isinstance(path_or_index, int):
        return path_or_index
    if isinstance(path_or_index, str):
        if path_or_index.startswith("/dev/video"):
            try:
                return int(path_or_index.replace("/dev/video", ""))
            except Exception:
                return None
        try:
            return int(path_or_index)
        except Exception:
            return None
    return None


class UsbCamera:
    """
    USB 카메라 래퍼 (Picamera2 호환 API 흉내)
    - 기본 장치: /dev/video4 (RealSense RGB, YUYV)
    - CAM_DEVICE=/dev/videoX 로 재지정 가능
    - CAM_GST 로 GStreamer 파이프라인 강제 사용 가능
    """
    def __init__(self, device=None, main_size=(848, 480), lores_size=(424, 240), fps=30):
        self.main_size = tuple(main_size)
        self.lores_size = tuple(lores_size)
        self.fps = int(fps)

        self.cap = None
        self.backend_used = None
        self.device_index = None

        # 1) GStreamer 파이프라인이 있으면 우선 사용
        gst = os.environ.get("CAM_GST")
        if gst:
            self._open_gstreamer(gst)
        else:
            # 2) 장치 고정: 기본은 /dev/video4 (RGB), 환경변수/인자로 재정의 가능
            forced_dev = os.environ.get("CAM_DEVICE") or device or "/dev/video4"
            idx = _to_int_device(forced_dev)
            if idx is None:
                # '/dev/video4' 같은 문자열일 때
                if os.path.exists(str(forced_dev)):
                    idx = _to_int_device(forced_dev)
            if idx is None:
                raise RuntimeError(f"Unrecognized CAM_DEVICE={forced_dev}")

            if not self._try_open_index(idx):
                raise RuntimeError(f"USB camera open failed on device {forced_dev}")

        # 캡쳐 속성 적용 (해상도/FPS/버퍼/포맷/컬러변환)
        self._apply_props()

        print(f"[UsbCamera] using /dev/video{self.device_index if self.device_index is not None else '?'} "
              f"backend={self._backend_name(self.backend_used)} "
              f"target={self.main_size}@{self.fps}fps")

    # ---------- Helpers ----------
    @staticmethod
    def _backend_name(be):
        if be == cv2.CAP_V4L2:
            return "V4L2"
        if be == cv2.CAP_GSTREAMER:
            return "GSTREAMER"
        if be == cv2.CAP_ANY or be is None:
            return "ANY"
        return str(be)

    def _apply_props(self):
        # 해상도/FPS
        try:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.main_size[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.main_size[1])
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        except Exception:
            pass

        # 최신 프레임만: 버퍼 최소화
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        # YUYV를 선호 (장치가 받으면 색감 정상화에 도움)
        try:
            fourcc = cv2.VideoWriter_fourcc('Y','U','Y','2')  # YUYV
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        except Exception:
            pass

        # BGR 변환 보장
        try:
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        except Exception:
            pass

    def _read_ok(self, cap):
        ok = cap.grab()
        if not ok:
            return False
        ok, frame = cap.retrieve()
        return bool(ok and frame is not None and frame.size > 0)

    def _try_open_index(self, index):
        for be in (cv2.CAP_V4L2, cv2.CAP_ANY, cv2.CAP_GSTREAMER):
            cap = cv2.VideoCapture(index, be)
            if cap.isOpened() and self._read_ok(cap):
                self.cap = cap
                self.backend_used = be
                self.device_index = index
                return True
            if cap.isOpened():
                cap.release()
        return False

    def _open_gstreamer(self, pipeline):
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not (cap.isOpened() and self._read_ok(cap)):
            if cap.isOpened():
                cap.release()
            raise RuntimeError(f"GStreamer open failed: {pipeline}")
        self.cap = cap
        self.backend_used = cv2.CAP_GSTREAMER
        self.device_index = None

    # ---------- Public API ----------
    def start(self):
        pass

    def capture_array(self, which: str):
        # 항상 최신 프레임 (grab → retrieve)
        if not self.cap.grab():
            # 실패 시 검은 프레임
            if which == "main":
                return np.zeros((self.main_size[1], self.main_size[0], 3), dtype=np.uint8)
            else:
                return np.zeros((self.lores_size[1], self.lores_size[0], 3), dtype=np.uint8)
        ok, frame = self.cap.retrieve()
        if not ok or frame is None:
            if which == "main":
                return np.zeros((self.main_size[1], self.main_size[0], 3), dtype=np.uint8)
            else:
                return np.zeros((self.lores_size[1], self.lores_size[0], 3), dtype=np.uint8)

        if which == "main":
            if (frame.shape[1], frame.shape[0]) != self.main_size:
                frame = cv2.resize(frame, self.main_size, interpolation=cv2.INTER_AREA)
            return frame
        elif which == "lores":
            lo = cv2.resize(frame, self.lores_size, interpolation=cv2.INTER_AREA)
            return lo
        else:
            raise ValueError("which must be 'main' or 'lores'")

    def set_controls(self, _controls: dict):
        # 필요 시 CAP_PROP_BRIGHTNESS/CONTRAST로 매핑 가능
        pass

    def stop(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass


class threadCamera(ThreadWithStop):
    """카메라 스레드 (저지연/정상 색감 설정)"""
    def __init__(self, queuesList, logger, debugger):
        super(threadCamera, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.frame_rate = 30   # 30fps로 올림 (지연↓)
        self.recording = False

        self.video_writer = None

        self.recordingSender = messageHandlerSender(self.queuesList, Recording)
        self.mainCameraSender = messageHandlerSender(self.queuesList, mainCamera)
        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)

        self.subscribe()
        self._init_camera()
        self.Queue_Sending()
        self.Configs()

    def subscribe(self):
        self.recordSubscriber = messageHandlerSubscriber(self.queuesList, Record, "lastOnly", True)
        self.brightnessSubscriber = messageHandlerSubscriber(self.queuesList, Brightness, "lastOnly", True)
        self.contrastSubscriber = messageHandlerSubscriber(self.queuesList, Contrast, "lastOnly", True)

    def Queue_Sending(self):
        if not self._running:
            return
        self.recordingSender.send(self.recording)
        threading.Timer(1, self.Queue_Sending).start()

    def stop(self):
        if self.recording and self.video_writer:
            try:
                self.video_writer.release()
            except Exception:
                pass
            finally:
                self.video_writer = None
        try:
            self.camera.stop()
        except Exception:
            pass
        super(threadCamera, self).stop()

    def Configs(self):
        if not self._running:
            return
        if self.brightnessSubscriber.isDataInPipe():
            message = self.brightnessSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set_controls({
                "AeEnable": False, "AwbEnable": False,
                "Brightness": max(0.0, min(1.0, float(message))),
            })
        if self.contrastSubscriber.isDataInPipe():
            message = self.contrastSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set_controls({
                "AeEnable": False, "AwbEnable": False,
                "Contrast": max(0.0, min(32.0, float(message))),
            })
        threading.Timer(1, self.Configs).start()

    def run(self):
        # JPEG 품질을 낮춰 전송 지연 감소
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 60]

        while self._running:
            try:
                recordRecv = self.recordSubscriber.receive()
                if recordRecv is not None:
                    self.recording = bool(recordRecv)
                    if not self.recording:
                        if self.video_writer:
                            try:
                                self.video_writer.release()
                            except Exception:
                                pass
                            self.video_writer = None
                    else:
                        if self.video_writer is None:
                            fourcc = cv2.VideoWriter_fourcc(*"XVID")
                            main_w, main_h = self.camera.main_size
                            self.video_writer = cv2.VideoWriter(
                                f"output_video{time.time()}.avi",
                                fourcc,
                                self.frame_rate,
                                (main_w, main_h),
                            )
            except Exception as e:
                if self.debugger:
                    self.logger.exception("record control error: %s", e)

            # 프레임 수집/전송 (스킵 없음 → 실효 FPS↑)
            mainRequest = self.camera.capture_array("main")
            serialRequest = self.camera.capture_array("lores")

            if self.recording and self.video_writer is not None:
                try:
                    self.video_writer.write(mainRequest)
                except Exception:
                    pass

            _, mainEncodedImg   = cv2.imencode(".jpg", mainRequest, encode_params)
            _, serialEncodedImg = cv2.imencode(".jpg", serialRequest, encode_params)

            mainEncodedImageData = base64.b64encode(mainEncodedImg).decode("utf-8")
            serialEncodedImageData = base64.b64encode(serialEncodedImg).decode("utf-8")

            self.mainCameraSender.send(mainEncodedImageData)
            self.serialCameraSender.send(serialEncodedImageData)

    def start(self):
        super(threadCamera, self).start()

    def _init_camera(self):
        # 기본 /dev/video4 (RGB, YUYV), 필요시 CAM_DEVICE로 변경
        self.camera = UsbCamera(
            device="/dev/video4",
            main_size=(848, 480),
            lores_size=(424, 240),
            fps=self.frame_rate or 30
        )
        self.camera.start()
