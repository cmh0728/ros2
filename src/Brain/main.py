# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# To start the project: 
#
#       sudo apt update
#       sudo apt upgrade
#       xargs sudo apt install -y < "requirement.txt" 
#       cd src/dashboard/frontend/
#       curl -fsSL https://fnm.vercel.app/install | bash
#       source ~/.bashrc
#       fnm install --lts
#       npm install -g @angular/cli@17
#       npm install
#       if needed: npm audit fix
#
# ===================================== GENERAL IMPORTS ==================================
# ===================================== 일반 임포트 ======================================
import sys
import time
import os
import psutil

# Pin to CPU cores 0–3
# CPU 코어 0~3에 프로세스를 고정하여 스케줄링 안정화
psutil.Process(os.getpid()).cpu_affinity([0, 1, 2, 3, 4, 5])

sys.path.append(".")
from multiprocessing import Queue, Event
import logging

logging.basicConfig(level=logging.INFO)

# ===================================== PROCESS IMPORTS ==================================
# ===================================== 프로세스 임포트 ==================================

from src.gateway.processGateway import processGateway
from src.dashboard.processDashboard import processDashboard
from src.hardware.camera.processCamera import processCamera
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
from src.data.Semaphores.Semaphores import processSemaphores
from src.data.TrafficCommunication.processTrafficCommunication import processTrafficCommunication
from src.utils.ipManager.IpReplacement import IPManager
# ------ New component imports starts here ------#

# ------ New component imports ends here ------#
# ======================================== SETTING UP ====================================
# ======================================== 기본 설정 =====================================
allProcesses = list()

# 프로세스 간 메시지 전달을 위한 우선순위별 큐 초기화
queueList = {
    "Critical": Queue(),
    "Warning": Queue(),
    "General": Queue(),
    "Config": Queue(),
}
logging = logging.getLogger()


# 모듈별 실행 여부를 제어하는 플래그
Dashboard = True
Camera = True
Semaphores = False
TrafficCommunication = False
SerialHandler = True

# ------ New component flags starts here ------#
 
# ------ New component flags ends here ------#

# ===================================== SETUP PROCESSES ==================================
# ===================================== 프로세스 준비 ====================================

# Initializing gateway
# 게이트웨이 프로세스를 가장 먼저 준비
processGateway = processGateway(queueList, logging)
processGateway.start()

# Ip replacement
# 프런트엔드 웹소켓 서비스 파일에서 IP 주소를 최신 값으로 교체
path = './src/dashboard/frontend/src/app/webSocket/web-socket.service.ts'
IpChanger = IPManager(path)
IpChanger.replace_ip_in_file()


# Initializing dashboard
# 대시보드 프로세스를 조건적으로 실행
if Dashboard:
    processDashboard = processDashboard( queueList, logging, debugging = False)
    allProcesses.append(processDashboard)

# Initializing camera
# 카메라 프로세스를 조건적으로 실행
if Camera:
    processCamera = processCamera(queueList, logging , debugging = False)
    allProcesses.append(processCamera)

# Initializing semaphores
# 신호등(세마포어) 프로세스를 조건적으로 실행
if Semaphores:
    processSemaphores = processSemaphores(queueList, logging, debugging = False)
    allProcesses.append(processSemaphores)

# Initializing GPS
# 교통 통신(GPS) 프로세스를 조건적으로 실행
if TrafficCommunication:
    processTrafficCommunication = processTrafficCommunication(queueList, logging, 3, debugging = False)
    allProcesses.append(processTrafficCommunication)

# Initializing serial connection NUCLEO - > PI
# 마이크로컨트롤러와 라즈베리 파이 간 시리얼 통신 프로세스를 실행
if SerialHandler:
    processSerialHandler = processSerialHandler(queueList, logging, debugging = False)
    allProcesses.append(processSerialHandler)

# ------ New component runs starts here ------#
 
# ------ New component runs ends here ------#

# ===================================== START PROCESSES ==================================
# ===================================== 프로세스 시작 =====================================
for process in allProcesses:
    process.daemon = True
    process.start()

time.sleep(10)
# 초기화가 완료될 시간을 확보
c4_bomb = r"""
  _______________________
 /                       \
| [██████]    [██████]    |
| [██████]    [██████]    |
| [██████]    [██████]    |
|       TIMER: 00:10      |
|_________________________|
 \_______________________/
        LET'S GO!!!

        Press ctrl+C to close
"""

print(c4_bomb)

# ===================================== STAYING ALIVE ====================================
# ===================================== 메인 루프 유지 ====================================
blocker = Event()
# 메인 스레드를 대기 상태로 유지하여 하위 프로세스를 실행
try:
    blocker.wait()
except KeyboardInterrupt:
    # Ctrl+C 입력 시 모든 프로세스를 정리하고 종료
    print("\nCatching a KeyboardInterruption exception! Shutdown all processes.\n")
    big_text = """
    PPPP   L        EEEEE    A    SSSS  EEEEE       W      W   A   III TTTTT
    P   P  L        E       A A   S     E           W      W  A A   I    T  
    PPPP   L        EEEE   A   A   SSS  EEEE        W  W   W A   A  I    T  
    P      L        E      AAAAA      S E           W W W W  AAAAA  I    T  
    P      LLLLLL   EEEEE  A   A  SSSS  EEEEE        W   W   A   A III   T  
    """

    print(big_text)
    # 역순으로 순회하여 의존 관계가 있는 프로세스를 안전하게 종료
    for proc in reversed(allProcesses):
        print("Process stopped", proc)
        proc.stop()
    print("Process stopped", processGateway)
    processGateway.stop()

    big_text = """
    PPPP   RRRR   EEEEE  SSSS  SSSS       CCCC  TTTTT RRRR    L          ++      CCCC      !!! 
    P   P  R   R  E     S     S          C        T   R   R   L          ++      C         !!! 
    PPPP   RRRR   EEEE   SSS   SSS       C        T   RRRR    L      ++++++++++  C         !!! 
    P      R R    E         S     S      C        T   R R     L          ++      C         !!! 
    P      R  R   EEEEE  SSSS  SSSS       CCCC    T   R  R    LLLLL      ++      CCCC      !!!
    """

    print(big_text)
