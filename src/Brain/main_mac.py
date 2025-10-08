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
# macOS 환경에서 프론트엔드만 빠르게 확인할 수 있는 간소화된 진입점입니다.
# 라즈베리 파이 전용 프로세스는 모두 비활성화하고 Angular 개발 서버만 실행합니다.

from multiprocessing import Event
import logging
from pathlib import Path

from src.dashboard.threads.threadStartFrontend import ThreadStartFrontend
from src.utils.ipManager.IpReplacement import IPManager


def _resolve_frontend_root() -> Path:
    """Angular 대시보드 소스 디렉터리 경로를 계산."""
    return Path(__file__).resolve().parent / "src" / "dashboard" / "frontend"


def _replace_dashboard_ip(logger: logging.Logger) -> None:
    """웹소켓 서비스 파일에 기록된 IP를 현재 머신 IP로 교체."""
    web_socket_file = _resolve_frontend_root() / "src" / "app" / "webSocket" / "web-socket.service.ts"

    if not web_socket_file.exists():
        logger.warning("웹소켓 서비스 파일을 찾을 수 없습니다: %s", web_socket_file)
        return

    logger.info("웹소켓 서비스 IP 자동 치환을 시도합니다.")
    IPManager(str(web_socket_file)).replace_ip_in_file()


def _start_frontend(logger: logging.Logger) -> ThreadStartFrontend:
    """Angular 개발 서버를 백그라운드 스레드로 실행."""
    frontend_root = _resolve_frontend_root()
    if not frontend_root.exists():
        raise FileNotFoundError(f"Angular 프로젝트 디렉터리가 존재하지 않습니다: {frontend_root}")

    thread = ThreadStartFrontend(logger, project_path=str(frontend_root))
    thread.daemon = True
    thread.start()
    logger.info("Angular 개발 서버를 시작했습니다. 종료하려면 Ctrl+C를 누르세요.")
    return thread


def main() -> None:
    """macOS에서 프론트엔드 전용 개발 환경을 구동."""
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("brain.main_mac")

    _replace_dashboard_ip(logger)
    frontend_thread = _start_frontend(logger)

    wait_token = Event()
    try:
        wait_token.wait()
    except KeyboardInterrupt:
        logger.info("중지 신호를 받았습니다. 프론트엔드 개발 서버를 종료합니다.")
    finally:
        frontend_thread.stop()
        if frontend_thread.is_alive():
            frontend_thread.join(timeout=5)
        logger.info("정상적으로 종료했습니다.")


if __name__ == "__main__":
    main()
