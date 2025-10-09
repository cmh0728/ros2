# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.

import cv2
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


class UsbCamera:
    """Picamera2의 capture_array('main'/'lores') 인터페이스를 흉내내는 USB 카메라 래퍼."""
    def __init__(self, device=0, main_size=(2048, 1080), lores_size=(512, 270), fps=30):
        self.main_size = tuple(main_size)
        self.lores_size = tuple(lores_size)
        self.fps = int(fps)

        # OpenCV VideoCapture (Jetson이면 GStreamer 파이프라인으로 대체 가능)
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        # 해상도/프레임레이트 힌트
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.main_size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.main_size[1])
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            raise RuntimeError(f"USB camera open failed on device {device}")

    def start(self):
        # Picamera2 호환용 더미
        pass

    def capture_array(self, which: str):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            # 마지막 프레임이 없을 때는 검은 프레임 반환 (프로토콜 유지)
            if which == "main":
                return np.zeros((self.main_size[1], self.main_size[0], 3), dtype=np.uint8)
            else:
                return np.zeros((self.lores_size[1], self.lores_size[0], 3), dtype=np.uint8)

        # OpenCV는 BGR 프레임
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
        # Picamera2 API 자리만 유지 (USB: 필요시 CAP_PROP_BRIGHTNESS/CONTRAST로 매핑 가능)
        # 예: self.cap.set(cv2.CAP_PROP_BRIGHTNESS, value)
        pass

    def stop(self):
        try:
            self.cap.release()
        except Exception:
            pass


class threadCamera(ThreadWithStop):
    """카메라와 관련된 기능을 백그라운드 스레드에서 처리한다."""
    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadCamera, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.frame_rate = 5
        self.recording = False

        self.video_writer = None  # 안전 가드

        self.recordingSender = messageHandlerSender(self.queuesList, Recording)
        self.mainCameraSender = messageHandlerSender(self.queuesList, mainCamera)
        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)

        # 제어 메시지 구독 및 카메라 초기화 진행
        self.subscribe()
        self._init_camera()
        self.Queue_Sending()
        self.Configs()

    def subscribe(self):
        """게이트웨이에 필요한 메시지를 모두 구독한다."""
        self.recordSubscriber = messageHandlerSubscriber(self.queuesList, Record, "lastOnly", True)
        self.brightnessSubscriber = messageHandlerSubscriber(self.queuesList, Brightness, "lastOnly", True)
        self.contrastSubscriber = messageHandlerSubscriber(self.queuesList, Contrast, "lastOnly", True)

    def Queue_Sending(self):
        """녹화 상태 플래그를 주기적으로 전송한다."""
        if not self._running:
            return
        self.recordingSender.send(self.recording)
        threading.Timer(1, self.Queue_Sending).start()

    # =============================== STOP ================================================
    def stop(self):
        # 녹화 중이면 writer 정리
        if self.recording and self.video_writer:
            try:
                self.video_writer.release()
            except Exception:
                pass
            finally:
                self.video_writer = None

        # 카메라 릴리즈
        try:
            self.camera.stop()
        except Exception:
            pass

        super(threadCamera, self).stop()

    # =============================== CONFIG ==============================================
    def Configs(self):
        """파이프를 통해 전달된 카메라 설정 값을 읽어 적용한다."""
        if not self._running:
            return

        if self.brightnessSubscriber.isDataInPipe():
            message = self.brightnessSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set_controls(
                {
                    "AeEnable": False,
                    "AwbEnable": False,
                    "Brightness": max(0.0, min(1.0, float(message))),
                }
            )
        if self.contrastSubscriber.isDataInPipe():
            message = self.contrastSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set_controls(
                {
                    "AeEnable": False,
                    "AwbEnable": False,
                    "Contrast": max(0.0, min(32.0, float(message))),
                }
            )
        threading.Timer(1, self.Configs).start()

    # ================================ RUN ================================================
    def run(self):
        """스레드가 동작 중일 때 카메라에서 프레임을 읽고 인코딩해 게이트웨이로 전달한다."""
        send = True
        while self._running:
            # 녹화 제어 수신
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
                        # 녹화 시작
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

            # 프레임 수집/전송
            if send:
                mainRequest = self.camera.capture_array("main")
                serialRequest = self.camera.capture_array("lores")

                # 녹화 중이면 저장
                if self.recording and self.video_writer is not None:
                    try:
                        self.video_writer.write(mainRequest)
                    except Exception:
                        pass

                # USB(UVC)는 이미 BGR이므로 변환 생략 (Picamera2만 I420 변환 필요)
                # 직렬 전송용 JPEG 인코딩
                _, mainEncodedImg = cv2.imencode(".jpg", mainRequest)
                _, serialEncodedImg = cv2.imencode(".jpg", serialRequest)

                mainEncodedImageData = base64.b64encode(mainEncodedImg).decode("utf-8")
                serialEncodedImageData = base64.b64encode(serialEncodedImg).decode("utf-8")

                self.mainCameraSender.send(mainEncodedImageData)
                self.serialCameraSender.send(serialEncodedImageData)

            # 부하 감소: 한 프레임 건너뛰기
            send = not send

    # =============================== START ===============================================
    def start(self):
        super(threadCamera, self).start()

    # ================================ INIT CAMERA ========================================
    def _init_camera(self):
        """카메라 객체를 초기화하고 main/lores 두 채널로 구성한다."""
        # USB 카메라로 대체
        self.camera = UsbCamera(device=0, main_size=(2048, 1080), lores_size=(512, 270), fps=self.frame_rate or 30)
        self.camera.start()
