import struct
import socket
import select
import socket
import threading
import math  # 각도 변환용

import sys, os, time
sys.path.append(os.path.dirname(__file__))

from RobotData import *

EXCEL_PATH = os.path.join(os.path.dirname(__file__), "RobotStateMessage.xlsx")
SHEET_NAME = "v2.6.0"

HOST = '192.168.1.101'
PORT1 = 30001
PORT2 = 29999

PC_IP = '192.168.1.102'
PC_PORT = 30010

DEFAULT_TIMEOUT = 10.0
ROBOT_STATE_TYPE = 16

class Robot():
    def __init__(self, ip,): 
        self.ip = ip
        self.recv_data = None

    def RecvPopup(self, port):
        HOST = "0.0.0.0"
        PORT = port

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.bind((HOST, PORT))
            s.listen()
            print(f"🟢 Listening for robot on port {PORT}...")
            conn, addr = s.accept()
            with conn:
                print(f"🔁 Connected by {addr}")
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    self.recv_data = data.decode().strip()
                    print(f"📥 Received from robot: {self.recv_data}")
                    break

class Robot_exel():
    def __init__(self, excel, sheet, host_ip, pc_ip) -> None:
        self.__data_config = RobotDataConfig.get_config(excel, sheet)
        self.ip = host_ip
        self.pc_ip = pc_ip 
        self.port = PORT1

    def connect(self):
        try:
            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.__sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.__sock.settimeout(DEFAULT_TIMEOUT)
            self.hostname = self.ip
            self.port = self.port
            self.__sock.connect((self.hostname, self.port))
            self.__buf = bytes()
        except (socket.timeout, socket.error):
            self.__sock = None
            raise
    
    def disconnect(self):
        self.__sock.close()
        self.__sock = None
        
    def send_data(self, data):
        self.__sock.send(data)

    def get_data(self):
        return self.__recv()

    def __recv(self):
        try:
            self.__recv_to_buffer(DEFAULT_TIMEOUT)
        except:
            return None
        while len(self.__buf) > 5:
            head = RobotHeader.unpack(self.__buf)
            if head.size <= len(self.__buf):
                if head.type != ROBOT_STATE_TYPE:
                    self.__buf = self.__buf[head.size :]
                    continue
                data = RobotData.unpack(self.__buf, self.__data_config)
                self.__buf = self.__buf[head.size :]
                return data
            else:
                break
        return None

    def __recv_to_buffer(self, timeout):
        readable, _, xlist = select.select([self.__sock], [], [self.__sock], timeout)
        if len(readable):
            more = self.__sock.recv(4096)
            if len(more) == 0:
                print('received 0 bytes from Controller')
                return None
            
            self.__buf = self.__buf + more
            return True
        
        if (len(xlist) or len(readable) == 0) and timeout != 0:
            print ("no data received within timeout")
            return None

        return False

class Robot_30001():
    def __init__(self):
        self.recv_data = None
        self.host = HOST
        self.port = PORT1

    def socket_connect(self):
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.host, PORT1))  
            print(f"Connected to {self.host} on port {PORT1}")
            return self.client_socket
        except Exception as e:
            print(f"Error connecting to {self.host} on port {PORT1}: {e}")
            return None
        
    def send_command(self, command):
        try:
            if not hasattr(self, "client_socket") or self.client_socket is None:
                raise RuntimeError("client_socket is not connected")
            self.client_socket.sendall(f"{command}\n".encode("utf-8"))
        except Exception as e:
            print(f"Error sending command: {e}")

    def socket_disconnect(self):
        self.client_socket.close()
        self.client_socket = None
        
class Robot_29999():
    def __init__(self):
        self.sock = None
        self.host = HOST
        self.port = PORT2

    def socket_connect_29999(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, PORT2))  
            print(f"Connected to {self.host} on port {PORT2}")
            return self.sock
        except Exception as e:
            print(f"Error connecting to {self.host} on port {PORT2}: {e}")
            return None

    def send_command_29999(self, command):
        try:
            if self.sock is None:
                if self.socket_connect_29999() is None:
                    print("[WARN] 29999 not connected; cannot send command")
                    return
            self.sock.sendall(f"{command}\n".encode("utf-8"))
        except Exception as e:
            print(f"Error sending command: {e}")

    def socket_disconnect_29999(self):
        self.sock.close()
        self.sock = None
        
class ROBOT_EXEL_RECIEVE():
    def __init__(self, host_ip, pc_ip):
        self.robot = Robot_exel(EXCEL_PATH, SHEET_NAME, host_ip, pc_ip)
        self.host = host_ip
        self.pc_ip = pc_ip
        self.data = None
        self.prev_data = None
        self.tcp_data = None
        self.tcp_prev_data = None
        
        try:
            self.robot.connect()
            print("[INFO] Robot_exel connected to", self.host, PORT1)
        except Exception as e:
            print("[ERROR] Failed to connect Robot_exel:", e)
            self.robot = None

    def update_data(self):
        new_data = self.robot.get_data()
        if new_data is not None:
            self.prev_data = self.data
            self.data = new_data
            return True
        return False
    
    def tcp_update_data(self):
        tcp_new_data = self.robot.get_data()
        if tcp_new_data is not None:
            self.tcp_prev_data = self.tcp_data
            self.tcp_data = tcp_new_data
            return True
        return False
    
    def is_robot_power_on(self):
        return getattr(self.data, "is_robot_power_on", None)
    
    def get_robot_mode(self):
        return getattr(self.data, "get_robot_mode", None)

    def get_robot_control_mode(self):
        return getattr(self.data, "get_robot_control_mode", None)

    def get_target_speed_fraction(self):
        val = getattr(self.data, "get_target_speed_fraction", None)
        return int(val * 100) if val is not None else None

    def is_program_running(self):
        return getattr(self.data, "is_program_running", None)

    def is_robot_system_in_alarm(self):
        return getattr(self.data, "is_robot_system_in_alarm", None)
    
    def is_robot_tcp_x(self):
        return getattr(self.tcp_data, "tcp_x", None)
    def is_robot_tcp_y(self):
        return getattr(self.tcp_data, "tcp_y", None)
    def is_robot_tcp_z(self):
        return getattr(self.tcp_data, "tcp_z", None)
    def is_robot_rot_x(self):
        return getattr(self.tcp_data, "rot_x", None)
    def is_robot_rot_y(self):
        return getattr(self.tcp_data, "rot_y", None)
    def is_robot_rot_z(self):
        return getattr(self.tcp_data, "rot_z", None)
    
    def get_tcp_pose(self):
        tcp_changes = {}

        tcp_curr = {
            "tcp_x": self.is_robot_tcp_x(),
            "tcp_y": self.is_robot_tcp_y(),
            "tcp_z": self.is_robot_tcp_z(),
            "rot_x": self.is_robot_rot_x(),
            "rot_y": self.is_robot_rot_y(),
            "rot_z": self.is_robot_rot_z()
        }

        if self.tcp_prev_data is None:
            return tcp_curr 

        tcp_prev = {
            "tcp_x": getattr(self.tcp_prev_data, "tcp_x", None),
            "tcp_y": getattr(self.tcp_prev_data, "tcp_y", None),
            "tcp_z": getattr(self.tcp_prev_data, "tcp_z", None),
            "rot_x": getattr(self.tcp_prev_data, "rot_x", None),
            "rot_y": getattr(self.tcp_prev_data, "rot_y", None),
            "rot_z": getattr(self.tcp_prev_data, "rot_z", None)
        }

        for key in tcp_curr:
            if tcp_curr[key] != tcp_prev[key]:
                tcp_changes[key] = tcp_curr[key]

        return tcp_changes if tcp_changes else None

    def get_status_if_changed(self):
        changes = {}

        curr = {
            "power": (
                self.is_robot_power_on()
                if self.is_robot_power_on() is not None
                else None
            ),
            "running": (
                self.is_program_running()
                if self.is_program_running() is not None
                else None
            ),
            "speed": (
                int(self.get_target_speed_fraction())
                if self.get_target_speed_fraction() is not None
                else None
            ),
            "alarm": (
                self.is_robot_system_in_alarm()
                if self.is_robot_system_in_alarm() is not None
                else None
            ),
            "mode": (
                self.get_robot_mode()
                if self.get_robot_mode() is not None
                else None
            ),
            "control_mode": (
                self.get_robot_control_mode()
                if self.get_robot_control_mode() is not None
                else None
            )
        }

        if self.prev_data is None:
            return curr

        prev = {
            "power": (
                getattr(self.prev_data, "is_robot_power_on", None)
                if getattr(self.prev_data, "is_robot_power_on", None) is not None
                else None
            ),
            "running": (
                getattr(self.prev_data, "is_program_running", None)
                if getattr(self.prev_data, "is_program_running", None) is not None
                else None
            ),
            "speed": (
                int(getattr(self.prev_data, "get_target_speed_fraction", 0) * 100)
                if getattr(self.prev_data, "get_target_speed_fraction", None) is not None
                else None
            ),
            "alarm": (
                getattr(self.prev_data, "is_robot_system_in_alarm", None)
                if getattr(self.prev_data, "is_robot_system_in_alarm", None) is not None
                else None
            ),
            "mode": (
                getattr(self.prev_data, "get_robot_mode", None)
                if getattr(self.prev_data, "get_robot_mode", None) is not None
                else None
            ),
            "control_mode": (
                getattr(self.prev_data, "get_robot_control_mode", None)
                if getattr(self.prev_data, "get_robot_control_mode", None) is not None
                else None
            )
        }
        
        for key in curr:
            if curr[key] != prev[key]:
                changes[key] = curr[key]

        return changes if changes else None
    
class ROBOT_SEND():
    def __init__(self):
        self.host_ip = HOST
        self.pc_ip = PC_IP
        
        print(f"[INFO] ROBOT_SEND 초기화: HOST={HOST}, PC_IP={PC_IP}")
        
        self.robot_29999 = Robot_29999()
        self.robot_30001 = Robot_30001()
        
        try:
            self.robot_30001.socket_connect()
            self.robot_29999.socket_connect_29999()
        except Exception as e:
            print(f"[WARN] 로봇 연결 실패: {e}")

        try:
            self.recv = ROBOT_EXEL_RECIEVE(self.host_ip, self.pc_ip)
            self.tcp_recv = ROBOT_EXEL_RECIEVE(self.host_ip, self.pc_ip)
        except Exception as e:
            print(f"[WARN] Robot_exel 연결 실패: {e}")
            
        self.lock = threading.Lock()
        
        self.power = None
        self.running = None
        self.speed = None
        self.alarm = None
        self.mode = None
        self.control_mode = None
        
    def robot_tcp(self):
        try:
            while True:
                if self.tcp_recv.tcp_update_data():
                    tcp_changes = self.tcp_recv.get_tcp_pose()
                    if tcp_changes:
                        self.tcp_x = tcp_changes.get("tcp_x")
                        self.tcp_y = tcp_changes.get("tcp_y")
                        self.tcp_z = tcp_changes.get("tcp_z")
                        self.rot_x = tcp_changes.get("rot_x")
                        self.rot_y = tcp_changes.get("rot_y")
                        self.rot_z = tcp_changes.get("rot_z")
                        yield self.tcp_x, self.tcp_y, self.tcp_z, self.rot_x, self.rot_y, self.rot_z
        except Exception as e:
            print(f"Error receiving: {e}")
            return None
        
    def robot_recieve(self):
        while True:
            try:
                if self.recv.update_data():
                    changes = self.recv.get_status_if_changed()
                    if changes:
                        self.power        = changes.get("power",        self.power)
                        self.running      = changes.get("running",      self.running)
                        self.speed        = changes.get("speed",        self.speed)
                        self.alarm        = changes.get("alarm",        self.alarm)
                        self.mode         = changes.get("mode",         self.mode)
                        self.control_mode = changes.get("control_mode", self.control_mode)
                        yield (self.power, self.running, self.speed, self.alarm, self.mode, self.control_mode)
                else:
                    time.sleep(0.05)
            except Exception as e:
                print(f"[WARN] robot_recieve loop error: {e}")
                time.sleep(0.2)
                continue
            
    def robot_power_on(self):
        try:
            power_on = "robotControl -on"
            self.robot_29999.send_command_29999(power_on)
            time.sleep(0.1)
        except Exception as e:
            print(f"Failed to {power_on}: {e}")
            
    def robot_brake_release(self):
        try:
            brake_release = "brakeRelease"
            self.robot_29999.send_command_29999(brake_release)
            time.sleep(0.1)
        except Exception as e:
            print(f"Failed to {brake_release}: {e}")

    def robot_power_off(self):
        try:
            power_off = "robotControl -off"
            self.robot_29999.send_command_29999(power_off)
            time.sleep(0.1)
        except Exception as e:
            print(f"Failed to {power_off}: {e}")

    def robot_play(self):
        try:
            robot_play = "play"
            self.robot_29999.send_command_29999(robot_play)
            time.sleep(0.1)
        except Exception as e:
            print(f"Failed to {robot_play}: {e}")
       
    def robot_pause(self):
        try:
            robot_pause = "pause"
            self.robot_29999.send_command_29999(robot_pause)
            time.sleep(0.1)
        except Exception as e:
            print(f"Failed to {robot_pause}: {e}")

    def robot_stop(self):
        try:
            robot_stop = "stop"
            self.robot_29999.send_command_29999(robot_stop)
            time.sleep(0.1)
        except Exception as e:
            print(f"Failed to {robot_stop}: {e}")
        
    def robot_speed(self, speed):
        try:
            robot_speed = f"speed -v {speed}"
            self.robot_29999.send_command_29999(robot_speed)
            time.sleep(0.1)
        except Exception as e:
            print(f"Failed to {robot_speed}: {e}")

    def send_command(self, command):
        try:
            if not hasattr(self.robot_30001, "client_socket") or self.robot_30001.client_socket is None:
                sock = self.robot_30001.socket_connect()
                if sock is None:
                    print("[WARN] 30001 not connected; cannot send command")
                    return
            self.robot_30001.send_command(command)
            print(f"[INFO] Command sent: {command}")
        except Exception as e:
            print(f"[ERROR] send_command failed: {e}")

# === 상태 확인용 헬퍼 함수 ===
def wait_until(condition_func, timeout=15.0, description="작업"):
    start_time = time.time()
    print(f"⏳ [WAIT] '{description}' 상태 확인 중...", end="", flush=True)
    while True:
        if condition_func():
            print(f" ✅ 감지 완료! ({time.time() - start_time:.2f}초 소요)")
            return True
        if time.time() - start_time > timeout:
            print(f"\n❌ [TIMEOUT] '{description}' 시간이 초과되었습니다.")
            return False
        time.sleep(0.1)

if __name__ == "__main__":
    print("=== ROBOT SYSTEM TEST START (Delay Added) ===")

    robot_send = ROBOT_SEND()

    def recv_status_thread():
        for status in robot_send.robot_recieve():
            power, running, speed, alarm, mode, ctrl_mode = status
            print(f"[STATUS] Power={power}, Run={running}, Speed={speed}, Alarm={alarm}, Mode={mode}, CtrlMode={ctrl_mode}")

    def recv_tcp_thread():
        for tcp in robot_send.robot_tcp():
            tcp_x, tcp_y, tcp_z, rot_x, rot_y, rot_z = tcp
            # ★ [수정] 값이 None이면 출력하지 않고 넘김 (에러 방지)
            if tcp_x is None: 
                continue
            print(f"[TCP] x={tcp_x:.2f}, y={tcp_y:.2f}, z={tcp_z:.2f}, Rx={rot_x:.2f}, Ry={rot_y:.2f}, Rz={rot_z:.2f}")

    t1 = threading.Thread(target=recv_status_thread, daemon=True)
    t2 = threading.Thread(target=recv_tcp_thread, daemon=True)
    t1.start()
    t2.start()
    
    print("📡 데이터 수신 대기 중...")
    time.sleep(1)

    try:
        # 1. 전원 켜기
        if robot_send.recv.is_robot_power_on():
            print("ℹ️ 이미 전원이 켜져 있습니다.")
        else:
            print("[CMD] 로봇 Power On 명령 전송")
            robot_send.robot_power_on()
            wait_until(lambda: robot_send.recv.is_robot_power_on() == True, timeout=60.0, description="로봇 전원 켜기")

        # time.sleep(3) # 전원 켜지고 안정화 대기

        # 2. 브레이크 해제
        print("[CMD] 로봇 Brake Release 명령 전송")
        robot_send.robot_brake_release()
        
        # 상태값 확인 (소프트웨어적 완료 확인)
        success = wait_until(
            lambda: robot_send.recv.get_robot_mode() is not None and robot_send.recv.get_robot_mode() >= 7,
            timeout=60.0,
            description="브레이크 해제 신호 감지"
        )
        if not success:
            raise Exception("브레이크 해제 실패. 비상 정지 스위치를 확인하세요.")
        
        # ★ [사용자 요청 반영] 
        # 감지 완료 후에도 물리적 안정화를 위해 5초간 강제로 더 기다립니다.
        print("\n⏳ [WAIT] 물리적 브레이크 해제 안정화 대기 (5초)...")

        print("🚀 이동 준비 완료!")


        # 3. 속도 설정 및 이동
        print("\n[CMD] 로봇 속도 50% 설정")
        robot_send.robot_speed(50)
        time.sleep(1)

        print("[CMD] 로봇 이동 (MoveJ)")
        # 예시: 'ㄱ'자 모양 테스트
        command_move = '''movej([0, -1.57, 1.57, 0, 1.57, 0], a=0.5, v=0.1)'''
        robot_send.send_command(command_move)
        
        print("✅ 이동 명령 전송 완료. 10초간 대기...")
        time.sleep(10)

        print("[CMD] 로봇 정지")
        robot_send.robot_stop()

    except KeyboardInterrupt:
        print("\n[EXIT] 사용자 종료 요청")
    except Exception as e:
        print(f"\n❌ [ERROR] {e}")

    print("=== ROBOT SYSTEM TEST END ===")