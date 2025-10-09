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

import re
import subprocess

class IPManager:
    def __init__(self, file_path):
        """IP 주소를 갱신할 대상 파일 경로를 저장한다."""
        self.file_path = file_path

    def get_ip_address(self):
        """현재 장비의 IP 주소를 조회한다."""
        try:
            # hostname -I 는 모든 IPv4 주소를 반환하므로 첫 번째 항목을 사용한다.
            ip_output = subprocess.check_output("hostname -I", shell=True)
            ip_address = ip_output.decode('utf-8').strip().split()[0]
            return ip_address
        except subprocess.CalledProcessError:
            print("Could not retrieve IP address.")
            return None

    def replace_ip_in_file(self):
        """현재 IP가 파일에 기록된 값과 다르면 새 IP로 치환한다."""
        new_ip = self.get_ip_address()
        print("Success to retrieve IP address.")
        if not new_ip:
            print("Failed to retrieve IP address.")
            return

        # IPv4 형태의 문자열을 찾기 위한 정규식 패턴
        ip_pattern = r'\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b'

        # 대상 파일을 읽어서 IP가 포함된 부분을 찾는다.
        try:
            with open(self.file_path, 'r') as file:
                content = file.read()
        except FileNotFoundError:
            print(f"File {self.file_path} not found.")
            return

        # 파일 내 기존 IP 주소를 추출한다.
        current_ip_match = re.search(ip_pattern, content)
        
        if current_ip_match:
            current_ip = current_ip_match.group()
            
            # 현재 파일의 IP와 새로 조회한 IP가 동일한지 확인한다.
            if current_ip == new_ip:
                print(f"The IP address in {self.file_path} is already {new_ip}. No changes made.")
                return
            else:
                # 기존 IP 문자열을 새 IP로 치환한다.
                updated_content = re.sub(ip_pattern, new_ip, content)
                
                # 변경된 내용을 파일에 다시 기록한다.
                with open(self.file_path, 'w') as file:
                    file.write(updated_content)
                
                print(f"Replaced IP address in {self.file_path} with {new_ip}")
        else:
            print(f"No IP address found in {self.file_path}.")
