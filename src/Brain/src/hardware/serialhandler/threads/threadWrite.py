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

import json
import time
import threading

from src.hardware.serialhandler.threads.messageconverter import MessageConverter
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    Klem,
    Control,
    SteerMotor,
    SpeedMotor,
    Brake,
    DrivingMode,
    ToggleBatteryLvl,
    ToggleImuData,
    ToggleInstant,
    ToggleResourceMonitor
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender


class threadWrite(ThreadWithStop):
    """This thread write the data that Raspberry PI send to NUCLEO.\n

    Args:
        queues (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        serialCom (serial.Serial): Serial connection between the two boards.
        logFile (FileHandler): The path to the history file where you can find the logs from the connection.
        example (bool, optional): Flag for exmaple activation. Defaults to False.
    """

    # ===================================== INIT =========================================
    def __init__(self, queues, serialCom, logFile, logger, debugger = False, example=False):
        super(threadWrite, self).__init__()
        self.queuesList = queues
        self.serialCom = serialCom
        self.logFile = logFile
        self.exampleFlag = example
        self.logger = logger
        self.debugger = debugger

        self.running = False
        self.engineEnabled = False
        self.forceStop = False  # 대시보드 STOP 모드 여부
        self.stopApplied = False  # 정지 명령을 한 번만 전송하기 위한 플래그
        self.messageConverter = MessageConverter()
        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor) # steer
        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor) # speed
        self.brakeSender = messageHandlerSender(self.queuesList, Brake) # brake
        self.drivingModeSender = messageHandlerSender(self.queuesList, DrivingMode) # driving mode 
        self.configPath = "src/utils/table_state.json"

        self.loadConfig("init")
        self.subscribe()

        if example:
            self.i = 0.0
            self.j = -1.0
            self.s = 0.0
            self.example()

    def subscribe(self): # 대쉬보드에서 gateway를 통해서 넘어오는 데이터 구독 
        """Subscribe function. In this function we make all the required subscribe to process gateway"""

        # 대시보드에서 넘어오는 KL/주행모드/조향/속도/토글 신호를 구독 (fifo or last only 모드)
        self.klSubscriber = messageHandlerSubscriber(self.queuesList, Klem, "lastOnly", True)
        self.drivingModeSubscriber = messageHandlerSubscriber(self.queuesList, DrivingMode, "lastOnly", True)
        self.controlSubscriber = messageHandlerSubscriber(self.queuesList, Control, "lastOnly", True)
        self.steerMotorSubscriber = messageHandlerSubscriber(self.queuesList, SteerMotor, "lastOnly", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(self.queuesList, SpeedMotor, "lastOnly", True)
        self.brakeSubscriber = messageHandlerSubscriber(self.queuesList, Brake, "lastOnly", True)
        self.instantSubscriber = messageHandlerSubscriber(self.queuesList, ToggleInstant, "lastOnly", True)
        self.batterySubscriber = messageHandlerSubscriber(self.queuesList, ToggleBatteryLvl, "lastOnly", True)
        self.resourceMonitorSubscriber = messageHandlerSubscriber(self.queuesList, ToggleResourceMonitor, "lastOnly", True)
        self.imuSubscriber = messageHandlerSubscriber(self.queuesList, ToggleImuData, "lastOnly", True)

    # ==================================== SENDING =======================================

    def sendToSerial(self, msg):
        command_msg = self.messageConverter.get_command(**msg)
        if command_msg != "error":
            self.serialCom.write(command_msg.encode("ascii"))
            self.logFile.write(command_msg)

    def loadConfig(self, configType):
        with open(self.configPath, "r") as file:
            data = json.load(file)

        if configType == "init":
            data = data[len(data)-1]
            command = {"action": "batteryCapacity", "capacity": data["batteryCapacity"]["capacity"]}
            self.sendToSerial(command)      
        else:
            for e in range(4):
                if data[e]["value"] == "False":
                    value = 0
                else:
                    value = 1 
                command = {"action": data[e]['command'], "activate": value}
                self.sendToSerial(command)
                time.sleep(0.05)

    def convertFc(self,instantRecv):
        if instantRecv =="True":
            return 1
        else :
            return 0

    def _handleDrivingMode(self, mode):
        modeLower = mode.lower()
        if modeLower == "stop":
            self.forceStop = True
            if not self.stopApplied:
                self._applyStopCommands()
                self.stopApplied = True
        else:
            self.forceStop = False
            self.stopApplied = False

    def _applyStopCommands(self):
        """Send zeroed commands to guarantee the vehicle halts."""
        try:
            self.sendToSerial({"action": "speed", "speed": 0})
            self.sendToSerial({"action": "steer", "steerAngle": 0})
            self.sendToSerial({"action": "brake", "steerAngle": 0})
        except Exception as exc:
            if self.debugger:
                self.logger.error(f"Failed to apply stop commands: {exc}")
        
    # ===================================== RUN ==========================================
    def run(self): # kl이 30일때만 engineEnabled True가 됨. 
        """In this function we check if we got the enable engine signal. After we got it we will start getting messages from raspberry PI. It will transform them into NUCLEO commands and send them."""

        while self._running:
            try:
                drivingModeRecv = self.drivingModeSubscriber.receive()
                if drivingModeRecv is not None:
                    if self.debugger:
                        self.logger.info(drivingModeRecv)
                    self._handleDrivingMode(drivingModeRecv)

                klRecv = self.klSubscriber.receive()
                if klRecv is not None:
                    if self.debugger:
                        self.logger.info(klRecv)
                    if klRecv == "30":
                        self.running = True
                        self.engineEnabled = True
                        command = {"action": "kl", "mode": 30}
                        self.sendToSerial(command)
                        self.loadConfig("sensors")
                    elif klRecv == "15":
                        self.running = True
                        self.engineEnabled = False
                        command = {"action": "kl", "mode": 15}
                        self.sendToSerial(command)
                        self.loadConfig("sensors")
                    elif klRecv == "0":
                        self.running = False
                        self.engineEnabled = False
                        command = {"action": "kl", "mode": 0}
                        self.sendToSerial(command)

                # 엔진이 활성화되고 STOP 모드가 아닐 때만 주행 명령을 송신
                if self.running and not self.forceStop:
                    if self.engineEnabled:
                        brakeRecv = self.brakeSubscriber.receive()
                        if brakeRecv is not None:
                            if self.debugger:
                                self.logger.info(brakeRecv)
                            command = {"action": "brake", "steerAngle": int(brakeRecv)}
                            self.sendToSerial(command)

                        speedRecv = self.speedMotorSubscriber.receive()
                        if speedRecv is not None: 
                            if self.debugger:
                                self.logger.info(speedRecv)
                            command = {"action": "speed", "speed": int(speedRecv)}
                            self.sendToSerial(command)

                        steerRecv = self.steerMotorSubscriber.receive()
                        if steerRecv is not None:
                            if self.debugger:
                                self.logger.info(steerRecv) 
                            command = {"action": "steer", "steerAngle": int(steerRecv)}
                            self.sendToSerial(command)

                        controlRecv = self.controlSubscriber.receive()
                        if controlRecv is not None:
                            if self.debugger:
                                self.logger.info(controlRecv) 
                            command = {
                                "action": "vcd",
                                "time": int(controlRecv["Time"]),
                                "speed": int(controlRecv["Speed"]),
                                "steer": int(controlRecv["Steer"]),
                            }
                            self.sendToSerial(command)

                    instantRecv = self.instantSubscriber.receive()
                    if instantRecv is not None: 
                        if self.debugger:
                            self.logger.info(instantRecv) 
                        command = {"action": "instant", "activate": int(instantRecv)}
                        self.sendToSerial(command)

                    batteryRecv = self.batterySubscriber.receive()
                    if batteryRecv is not None: 
                        if self.debugger:
                            self.logger.info(batteryRecv)
                        command = {"action": "battery", "activate": int(batteryRecv)}
                        self.sendToSerial(command)

                    resourceMonitorRecv = self.resourceMonitorSubscriber.receive()
                    if resourceMonitorRecv is not None: 
                        if self.debugger:
                            self.logger.info(resourceMonitorRecv)
                        command = {"action": "resourceMonitor", "activate": int(resourceMonitorRecv)}
                        self.sendToSerial(command)

                    imuRecv = self.imuSubscriber.receive()
                    if imuRecv is not None: 
                        if self.debugger:
                            self.logger.info(imuRecv)
                        command = {"action": "imu", "activate": int(imuRecv)}
                        self.sendToSerial(command)

            except Exception as e:
                print(e)

    # ==================================== START =========================================
    def start(self):
        super(threadWrite, self).start()

    # ==================================== STOP ==========================================
    def stop(self):
        """This function will close the thread and will stop the car."""

        self.exampleFlag = False
        command = {"action": "kl", "mode": 0}
        self.sendToSerial(command)
        time.sleep(2)
        super(threadWrite, self).stop()

    # ================================== EXAMPLE =========================================
    def example(self):
        """This function simulte the movement of the car."""

        if self.exampleFlag:
            self.speedMotorSender.send({"Type": "Speed", "value": self.s})
            self.steerMotorSender.send({"Type": "Steer", "value": self.i})
            self.i += self.j
            if self.i >= 21.0:
                self.i = 21.0
                self.s = self.i / 7
                self.j *= -1
            if self.i <= -21.0:
                self.i = -21.0
                self.s = self.i / 7
                self.j *= -1.0
            threading.Timer(0.01, self.example).start()
