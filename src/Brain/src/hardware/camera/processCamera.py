# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.hardware.camera.threads.threadCamera import threadCamera

from multiprocessing import Pipe


class processCamera(WorkerProcess):
    """카메라 스레드를 묶어서 관리하는 워커 프로세스.
    Args:
        queueList (dict[multiprocessing.Queue]): 메시지 타입별 큐 모음.
        logging (logging.Logger): 디버깅용 로거.
        debugging (bool): 디버그 로그 활성화 여부.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processCamera, self).__init__(self.queuesList)

    # ===================================== RUN ==========================================
    def run(self):
        """필요한 초기화 후 스레드를 시작한다."""
        super(processCamera, self).run()

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """카메라 퍼블리셔 스레드를 생성해 워커 관리 리스트에 등록한다."""
        camTh = threadCamera(
         self.queuesList, self.logging, self.debugging
        )
        self.threads.append(camTh)


# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processCamera.py
if __name__ == "__main__":
    from multiprocessing import Queue, Event
    import time
    import logging
    import cv2
    import base64
    import numpy as np

    allProcesses = list()

    debugg = True

    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }

    logger = logging.getLogger()

    process = processCamera(queueList, logger, debugg)

    process.daemon = True
    process.start()

    time.sleep(4)
    if debugg:
        logger.warning("getting")
    img = {"msgValue": 1}
    # 제너럴 큐에서 카메라 프레임(문자열) 수신될 때까지 대기
    while type(img["msgValue"]) != type(":text"):
        img = queueList["General"].get()
    image_data = base64.b64decode(img["msgValue"])
    img = np.frombuffer(image_data, dtype=np.uint8)
    image = cv2.imdecode(img, cv2.IMREAD_COLOR)
    if debugg:
        logger.warning("got")
    cv2.imwrite("test.jpg", image)
    process.stop()
