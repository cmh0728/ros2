# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
# All rights reserved.
if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

import os
import re
import json
import psutil
import logging
import inspect
import subprocess
from pathlib import Path
from enum import Enum
from flask import Flask, request
from flask_cors import CORS
from flask_socketio import SocketIO, emit

from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.templates.workerprocess import WorkerProcess
from src.utils.messages.allMessages import Semaphores
from src.dashboard.threads.threadStartFrontend import ThreadStartFrontend
import src.utils.messages.allMessages as allMessages

# ------------------------ Async backend selection (eventlet -> threading fallback) ------------------------
_ASYNC_MODE = os.getenv("DASHBOARD_ASYNC", "eventlet").lower()
if _ASYNC_MODE == "eventlet":
    try:
        import eventlet  # noqa: F401
        _ASYNC_MODE = "eventlet"
    except Exception:
        _ASYNC_MODE = "threading"

class processDashboard(WorkerProcess):
    """대시보드 프로세스 전반을 담당하며 시스템 상태를 UI에 반영한다.
    Args:
        queueList (dict[multiprocessing.Queue]): 메시지 타입별 큐 모음.
        logging (logging.Logger): 디버깅용 로거.
        deviceID (int): 대상 장치 식별자.
    """
    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, debugging=False):
        super(processDashboard, self).__init__(queueList)
        self.running = True
        self.queueList = queueList
        self.logger = logging
        self.debugging = debugging

        self.messages = {}
        self.sendMessages = {}
        self.messagesAndVals = {}

        self.memoryUsage = 0
        self.cpuCoreUsage = 0
        self.cpuTemperature = -1  # unknown

        self.sessionActive = False
        self.activeUser = None

        # setup Flask and SocketIO
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode=_ASYNC_MODE)
        CORS(self.app, supports_credentials=True)

        # 메시지 맵 구성 후 대시보드에서 직접 다루지 않는 항목 제거
        self.getNamesAndVals()
        self.messagesAndVals.pop("mainCamera", None)
        self.messagesAndVals.pop("Semaphores", None)
        self.subscribe()

        # define WebSocket event handlers
        self.socketio.on_event('message', self.handleMessage)
        self.socketio.on_event('save', self.handleSaveTableState)
        self.socketio.on_event('load', self.handleLoadTableState)

        # start hardware monitoring and continuous message sending
        self.sendContinuousHardwareData()
        # spawn sender loop
        if self.socketio.async_mode == "eventlet":
            import eventlet
            eventlet.spawn(self.sendContinuousMessages)
        else:
            # threading fallback
            import threading
            threading.Thread(target=self.sendContinuousMessages, daemon=True).start()

        super(processDashboard, self).__init__(self.queueList)

    # ===================================== STOP ==========================================
    def stop(self):
        super(processDashboard, self).stop()
        self.running = False

    # ===================================== RUN ==========================================
    def run(self):
        """초기화 로직을 수행하고 스레드를 시작한 뒤 소켓 서버를 구동한다."""
        self._init_threads()
        for th in self.threads:
            th.daemon = self.daemon
            th.start()

        self.socketio.run(self.app, host='0.0.0.0', port=5005)

    def subscribe(self):
        """게이트웨이로부터 수신할 메시지를 구독하고 대시보드에서 보낼 채널을 등록한다."""
        for name, enum in self.messagesAndVals.items():
            if enum["owner"] != "Dashboard":
                subscriber = messageHandlerSubscriber(self.queueList, enum["enum"], "lastOnly", True)
                self.messages[name] = {"obj": subscriber}
            else:
                sender = messageHandlerSender(self.queueList, enum["enum"])
                self.sendMessages[str(name)] = {"obj": sender}

        subscriber = messageHandlerSubscriber(self.queueList, Semaphores, "fifo", True)
        self.messages["Semaphores"] = {"obj": subscriber}

    def getNamesAndVals(self):
        """모든 메시지 enum을 순회해 이름과 소유 정보를 사전에 저장한다."""
        classes = inspect.getmembers(allMessages, inspect.isclass)
        for name, cls in classes:
            if name != "Enum" and issubclass(cls, Enum):
                self.messagesAndVals[name] = {"enum": cls, "owner": cls.Owner.value}

    def sendMessageToBackend(self, dataName, dataDict):
        """프론트엔드로부터 받은 데이터를 해당 백엔드 채널로 전달한다."""
        if dataName in self.sendMessages:
            self.sendMessages[dataName]["obj"].send(dataDict.get("Value"))

    def handleMessage(self, data):
        """웹소켓으로 들어온 메시지를 처리하고 필요한 동작을 수행한다."""
        if self.debugging:
            self.logger.info("Received message: " + str(data))

        dataDict = json.loads(data)
        dataName = dataDict["Name"]
        socketId = request.sid

        if dataName == "SessionAccess":
            self.handleSingleUserSession(socketId)
        elif dataName == "SessionEnd":
            self.handleSessionEnd(socketId)
        else:
            self.sendMessageToBackend(dataName, dataDict)

        emit('response', {'data': 'Message received: ' + str(data)}, room=socketId)

    def handleSingleUserSession(self, socketId):
        """동시에 한 명만 제어할 수 있도록 세션 상태를 관리한다."""
        if not self.sessionActive:
            self.sessionActive = True
            self.activeUser = socketId
            self.socketio.emit('session_access', {'data': True}, room=socketId)
        elif self.activeUser == socketId:
            self.socketio.emit('session_access', {'data': True}, room=socketId)
        else:
            self.socketio.emit('session_access', {'data': False}, room=socketId)

    def handleSessionEnd(self, socketId):
        """세션 종료 요청을 받아 잠금 상태를 해제한다."""
        if self.sessionActive and self.activeUser == socketId:
            self.sessionActive = False
            self.activeUser = None

    # ------------------------ File path helpers (RPi 경로 제거) ------------------------
    @staticmethod
    def _table_state_path() -> Path:
        # 사용자 홈 하위로 저장 (없으면 생성)
        base = Path.home() / ".cache" / "Brain" / "dashboard"
        base.mkdir(parents=True, exist_ok=True)
        return base / "table_state.json"

    def handleSaveTableState(self, data):
        """대시보드 테이블 상태를 JSON 파일로 저장한다."""
        if self.debugging:
            self.logger.info("Received save message: " + data)

        dataDict = json.loads(data)
        file_path = self._table_state_path()
        with open(file_path, 'w') as json_file:
            json.dump(dataDict, json_file, indent=4)

    def handleLoadTableState(self, data):
        """저장된 테이블 상태를 읽어 프론트엔드로 돌려준다."""
        file_path = self._table_state_path()
        try:
            with open(file_path, 'r') as json_file:
                dataDict = json.load(json_file)
            emit('loadBack', {'data': dataDict})
        except FileNotFoundError:
            emit('response', {'error': 'File not found. Please save the table state first.'})
        except json.JSONDecodeError:
            emit('response', {'error': 'Failed to parse JSON data from the file.'})

    # ------------------------ Jetson-safe temperature readers ------------------------
    @staticmethod
    def _safe_cpu_temp() -> int | None:
        """Jetson/일반 리눅스에서 안전하게 CPU(혹은 대표) 온도(°C 정수)를 반환. 실패 시 None."""
        # 1) psutil 우선
        try:
            temps = psutil.sensors_temperatures(fahrenheit=False) or {}
            preferred_keys = (
                'cpu_thermal', 'cpu-thermal', 'soc_thermal', 'soc-thermal',
                'coretemp', 'acpitz', 'k10temp', 'thermal-fan-est', 'GPU-therm'
            )
            # 우선 키 탐색
            for k in preferred_keys:
                entries = temps.get(k)
                if not entries:
                    continue
                for e in entries:
                    cur = getattr(e, "current", None)
                    if cur is not None:
                        try:
                            return round(float(cur))
                        except Exception:
                            pass
            # 전체에서 최대값(대표치) 선택
            candidates = []
            for _, entries in temps.items():
                for e in entries:
                    cur = getattr(e, "current", None)
                    if cur is not None:
                        try:
                            candidates.append(float(cur))
                        except Exception:
                            continue
            if candidates:
                return round(max(candidates))
        except Exception:
            pass

        # 2) tegrastats 파싱 (Jetson)
        try:
            out = subprocess.check_output(["tegrastats", "--interval", "1000", "--once"], text=True, timeout=2)
            # 예: CPU@39.5C GPU@38.0C AO@... 형식에서 첫 숫자 추출
            m = re.search(r"CPU@(\d+(?:\.\d+)?)C", out)
            if not m:
                m = re.search(r"GPU@(\d+(?:\.\d+)?)C", out)
            if m:
                return round(float(m.group(1)))
        except Exception:
            pass

        # 3) /sys/class/thermal 직접
        try:
            tz_base = Path("/sys/class/thermal")
            candidates = []
            if tz_base.exists():
                for tz in tz_base.glob("thermal_zone*/temp"):
                    try:
                        s = tz.read_text().strip()
                        if s:
                            candidates.append(float(s) / 1000.0)
                    except Exception:
                        continue
            if candidates:
                return round(max(candidates))
        except Exception:
            pass

        return None

    def sendContinuousHardwareData(self):
        """주기적으로 시스템 자원 정보를 수집한다."""
        try:
            self.memoryUsage = psutil.virtual_memory().percent
        except Exception:
            self.memoryUsage = -1

        try:
            # 빠르게 반환하려면 interval=0.0; 부하 줄이려면 소폭 대기
            self.cpuCoreUsage = psutil.cpu_percent(interval=0.05, percpu=True)
        except Exception:
            self.cpuCoreUsage = []

        try:
            t = self._safe_cpu_temp()
            self.cpuTemperature = int(t) if t is not None else -1
        except Exception:
            self.cpuTemperature = -1

        # 재스케줄
        if self.socketio.async_mode == "eventlet":
            import eventlet
            eventlet.spawn_after(1, self.sendContinuousHardwareData)
        else:
            import threading, time
            threading.Timer(1.0, self.sendContinuousHardwareData).start()

    def sendContinuousMessages(self):
        """수집된 메시지와 하드웨어 상태를 지속적으로 프론트엔드에 전송한다."""
        counter = 0.0
        socketSleep = 0.1
        sendTime = 1.0

        sleeper = None
        if self.socketio.async_mode == "eventlet":
            import eventlet
            def _sleep(dt): eventlet.sleep(dt)
        else:
            import time
            def _sleep(dt): time.sleep(dt)

        while self.running:
            for msg, subscriber in self.messages.items():
                resp = subscriber["obj"].receive()
                if resp is not None:
                    self.socketio.emit(msg, {"value": resp})
                    if self.debugging:
                        self.logger.info(f"{msg}: {resp}")

            # 일정 주기마다 자원 사용률을 별도 채널로 전송
            if counter >= sendTime:
                self.socketio.emit('memory_channel', {'data': self.memoryUsage})
                self.socketio.emit('cpu_channel', {'data': {'usage': self.cpuCoreUsage, 'temp': self.cpuTemperature}})
                counter = 0.0
            else:
                counter += socketSleep

            _sleep(socketSleep)

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """대시보드를 띄우는 프론트엔드 스레드를 초기화한다."""
        dashboardThreadFrontend = ThreadStartFrontend(self.logger)
        self.threads.append(dashboardThreadFrontend)
