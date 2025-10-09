# -*- coding: utf-8 -*-
# Camera thread tuned for your system: prefers /dev/video2 and /dev/video4 (RealSense RGB)
#
# - Auto-probes devices in priority order [2, 4, 0, 1, 3, 5]
# - Tries V4L2 first, then falls back to ANY/GStreamer
# - Verifies by actually reading a frame
# - Keeps the previous safety fixes (clean stop, timers guarded, writer size, etc.)
#
# You can override with:
#   export CAM_DEVICE=/dev/video4
# or provide a GStreamer pipeline:
#   export CAM_GST='v4l2src device=/dev/video2 ! image/jpeg,framerate=30/1 ! jpegdec ! videoconvert ! videoscale ! video/x-raw,width=2048,height=1080 ! appsink drop=true sync=false'

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
    """'/dev/video2' -> 2, '2' -> 2, 2 -> 2; 실패 시 None"""
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
    USB 카메라 래퍼: Picamera2의 capture_array('main'/'lores')와 유사한 API 제공.
    - 우선순위 목록 [2,4,0,1,3,5] 에 따라 /dev/video* 오픈 시도
    - 열리면 첫 프레임을 실제로 읽어 검증
    - 환경변수 CAM_DEVICE, CAM_GST 로 강제 지정 가능
    """
    def __init__(self, device=None, main_size=(2048, 1080), lores_size=(512, 270), fps=30):
        self.main_size = tuple(main_size)
        self.lores_size = tuple(lores_size)
        self.fps = int(fps)

        self.cap = None
        self.backend_used = None
        self.device_index = None

        # 1) 환경변수 GStreamer 파이프라인 강제
        gst = os.environ.get("CAM_GST")
        if gst:
            self._open_gstreamer(gst)
        else:
            # 2) 환경변수/인자로 장치 강제
            forced_dev = os.environ.get("CAM_DEVICE") or device
            if forced_dev is not None:
                idx = _to_int_device(forced_dev)
                if idx is not None:
                    if not self._try_open_index(idx):
                        raise RuntimeError(f"USB camera open failed on forced device {forced_dev}")
                else:
                    if os.path.exists(str(forced_dev)):
                        idx = _to_int_device(forced_dev)
                        if idx is not None:
                            if not self._try_open_index(idx):
                                raise RuntimeError(f"USB camera open failed on forced device {forced_dev}")
                        else:
                            raise RuntimeError(f"Unrecognized CAM_DEVICE={forced_dev}")
                    else:
                        raise RuntimeError(f"Forced device not found: {forced_dev}")
            else:
                # 3) 자동 탐색: 우선순위 [2,4,0,1,3,5] → 그 외 /dev/video*
                if not self._auto_probe_priority():
                    raise RuntimeError("USB camera open failed (auto-probe)")

        # 해상도/프레임레이트 힌트
        self._apply_props()

        # 사용 장치/백엔드 로그
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
        try:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.main_size[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.main_size[1])
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        except Exception:
            pass

    def _read_ok(self, cap):
        ok, frame = cap.read()
        return bool(ok and frame is not None and frame.size > 0)

    def _try_open_index(self, index):
        """하나의 인덱스에 대해 V4L2 → ANY → GSTREAMER 순으로 시도."""
        for be in (cv2.CAP_V4L2, cv2.CAP_ANY, cv2.CAP_GSTREAMER):
            cap = cv2.VideoCapture(index, be)
            # 일부 장치는 open은 True지만 첫 프레임 read 실패 → read로 검증
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

    def _auto_probe_priority(self):
        # 우선순위 목록 (당신 시스템에서 동작 확인: 2,4 OK)
        preferred = [2, 4, 0, 1, 3, 5]

        # 먼저 우선순위로 시도
        for idx in preferred:
            devpath = f"/dev/video{idx}"
            if os.path.exists(devpath):
                if self._try_open_index(idx):
                    return True

        # 그 다음 남은 /dev/video* 전체 시도
        all_devs = sorted(glob.glob("/dev/video*"))
        for d in all_devs:
            idx = _to_int_device(d)
            if idx is None or idx in preferred:
                continue
            if self._try_open_index(idx):
                return True
        return False

    # ---------- Public API ----------
    def start(self):
        # Picamera2 인터페이스 호환용
        pass

    def capture_array(self, which: str):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            # 프레임 실패 시 검은 화면으로 유지
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
        # 필요 시 CAP_PROP_BRIGHTNESS/CONTRAST 매핑 가능
        pass

    def stop(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass


class threadCamera(ThreadWithStop):
    """카메라 스레드"""
    def __init__(self, queuesList, logger, debugger):
        super(threadCamera, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.frame_rate = 5
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
        send = True
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

            if send:
                mainRequest = self.camera.capture_array("main")
                serialRequest = self.camera.capture_array("lores")

                if self.recording and self.video_writer is not None:
                    try:
                        self.video_writer.write(mainRequest)
                    except Exception:
                        pass

                _, mainEncodedImg = cv2.imencode(".jpg", mainRequest)
                _, serialEncodedImg = cv2.imencode(".jpg", serialRequest)

                mainEncodedImageData = base64.b64encode(mainEncodedImg).decode("utf-8")
                serialEncodedImageData = base64.b64encode(serialEncodedImg).decode("utf-8")

                self.mainCameraSender.send(mainEncodedImageData)
                self.serialCameraSender.send(serialEncodedImageData)

            send = not send

    def start(self):
        super(threadCamera, self).start()

    def _init_camera(self):
        # device=None → 자동 프로빙(우선순위 [2,4,0,1,3,5])
        self.camera = UsbCamera(
            device=None,
            main_size=(2048, 1080),
            lores_size=(512, 270),
            fps=self.frame_rate or 30
        )
        self.camera.start()
