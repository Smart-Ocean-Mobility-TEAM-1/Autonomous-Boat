#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import sys, termios, tty, select
import time
import numpy as np

class HybridBoatController(Node):
    def __init__(self):
        super().__init__('hybrid_boat_controller')

        # 모터 속도 초기화
        self.left_speed = 0
        self.right_speed = 0
        self.speed_step = 10
        self.arduino = None
        self.arduino_connected = False

        # 제어 모드 (True: 자동, False: 수동)
        self.auto_mode = False
        self.emergency_stop = False

        # 라이다 회피 파라미터
        self.danger_threshold = 0.4   # 위험 거리
        self.safe_threshold = 0.8     # 안전 거리
        self.front_angle = 45         # 전방 감지 각도 (±45도)
        self.side_angle = 90          # 좌우 감지 각도
        
        self.auto_command = 'F'
        self.command_count = 0
        self.previous_auto_command = 'F'
        
        # 디버깅 출력 제어
        self.last_scan_time = 0
        self.scan_output_interval = 5.0  # 5초마다 출력
        
        # ⚡ 반응속도 향상 설정
        self.fast_response = True  # 빠른 반응 모드
        self.stability_threshold = 1 if self.fast_response else 2  # 안정화 임계값 줄임

        # 터미널 설정
        try:
            self.settings = termios.tcgetattr(sys.stdin)
        except Exception as e:
            self.get_logger().error(f"터미널 설정 실패: {e}")
            self.settings = None

        # 아두이노 연결
        self.connect_arduino()

        # 라이다 구독
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 자동 모드용 타이머 (반응속도 향상을 위해 0.05초)
        self.auto_timer = self.create_timer(0.05, self.auto_control_update)
        
        # 상태 표시 타이머 (5초마다)
        self.status_timer = self.create_timer(5.0, self.update_status_display)

        self.print_instructions()

    def connect_arduino(self):
        possible_ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
        for port in possible_ports:
            try:
                self.arduino = serial.Serial(port, 115200, timeout=0.1)
                time.sleep(2)
                self.arduino_connected = True
                self.get_logger().info(f"🟢 아두이노 연결 성공! 포트: {port}")
                break
            except Exception as e:
                continue

        if not self.arduino_connected:
            self.get_logger().error("🛑 아두이노 연결 실패! 시뮬레이션 모드")

    def print_instructions(self):
        status = "연결완료!!" if self.arduino_connected else "시뮬레이션 모드"
        mode = "🤖 자동모드" if self.auto_mode else "🕹️  수동모드"
        print(f"""
{status} - 하이브리드 보트 제어 시스템
========================================
현재 모드: {mode}

모드 전환:
F1 : 수동 모드 (키보드 조종)
F2 : 자동 모드 (라이다 회피)
ESC: 긴급 정지

=== 수동 모드 조작법 ===
w : 전진     s : 후진
a : 좌회전   d : 우회전
space : 정지

개별 모터 제어:
q/z : 좌측 모터 +/-
e/c : 우측 모터 +/-
k/l : 현재 방향 가속/감속

=== 자동 모드 ===
라이다로 장애물 자동 회피
위험거리: {self.danger_threshold}m
안전거리: {self.safe_threshold}m

r : 리셋    Ctrl+C : 종료
========================================
현재 속도 - 좌측: {self.left_speed}, 우측: {self.right_speed}
        """)

    def get_key(self):
        if not self.settings:
            return ''
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
            # F1, F2, ESC 키 처리
            if key == '\x1b':  # ESC 시퀀스 시작
                next_chars = sys.stdin.read(2)
                if next_chars == 'OP':  # F1
                    key = 'F1'
                elif next_chars == 'OQ':  # F2
                    key = 'F2'
                elif next_chars == '':  # ESC만 눌림
                    key = 'ESC'
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def clamp_speed(self, speed):
        return max(-255, min(255, speed))

    def send_motor_command(self):
        if self.emergency_stop:
            self.left_speed = 0
            self.right_speed = 0

        if not self.arduino_connected:
            mode_str = "🤖자동" if self.auto_mode else "🕹️수동"
            print(f"💻 시뮬레이션 [{mode_str}]: 좌측={self.left_speed}, 우측={self.right_speed}")
            return

        try:
            command = f"L{self.left_speed},R{self.right_speed}\n"
            self.arduino.write(command.encode())
            
            if self.auto_mode:
                print(f"⚡ Arduino 전송: {command.strip()}")
            
            # 빠른 반응을 위해 지연 시간 줄임
            time.sleep(0.005)  # 5ms로 단축
            if self.arduino.in_waiting:
                response = self.arduino.readline().decode().strip()
                if self.auto_mode and response:
                    print(f"📨 Arduino 응답: {response}")
                
        except Exception as e:
            self.get_logger().error(f"통신 에러: {e}")

    def scan_callback(self, msg):
        if not self.auto_mode:
            return

        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), 10.0, ranges)
        ranges = np.where(np.isnan(ranges), 10.0, ranges)
        
        total_points = len(ranges)
        
        # 각도별 영역 계산
        front_start = total_points - self.front_angle
        front_end = self.front_angle
        
        left_start = self.front_angle
        left_end = self.side_angle + self.front_angle
        
        right_start = total_points - (self.side_angle + self.front_angle)
        right_end = total_points - self.front_angle
        
        # 각 영역의 최소 거리
        front_ranges = np.concatenate([ranges[0:front_end], ranges[front_start:]])
        left_ranges = ranges[left_start:left_end]
        right_ranges = ranges[right_start:right_end]
        
        front_min = np.min(front_ranges) if len(front_ranges) > 0 else 10.0
        left_min = np.min(left_ranges) if len(left_ranges) > 0 else 10.0
        right_min = np.min(right_ranges) if len(right_ranges) > 0 else 10.0
        
        # 🔍 장애물 위치 찾기
        front_obstacle_idx = np.argmin(front_ranges) if len(front_ranges) > 0 else -1
        left_obstacle_idx = np.argmin(left_ranges) + left_start if len(left_ranges) > 0 else -1
        right_obstacle_idx = np.argmin(right_ranges) + right_start if len(right_ranges) > 0 else -1
        
        # 각도 계산 (라이다 0도 = 정면)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        
        front_obstacle_angle = 0
        left_obstacle_angle = 0
        right_obstacle_angle = 0
        
        if front_obstacle_idx >= 0:
            if front_obstacle_idx < front_end:
                front_obstacle_angle = (front_obstacle_idx * angle_increment + angle_min) * 180 / 3.14159
            else:
                adjusted_idx = front_obstacle_idx - len(ranges[:front_end])
                front_obstacle_angle = ((front_start + adjusted_idx) * angle_increment + angle_min) * 180 / 3.14159
        
        if left_obstacle_idx >= 0:
            left_obstacle_angle = (left_obstacle_idx * angle_increment + angle_min) * 180 / 3.14159
            
        if right_obstacle_idx >= 0:
            right_obstacle_angle = (right_obstacle_idx * angle_increment + angle_min) * 180 / 3.14159
        
        # 자동 회피 결정
        new_command = self.decide_auto_movement(front_min, left_min, right_min)
        decision_reason = self.get_decision_reason(front_min, left_min, right_min, new_command)
        
        # 명령 안정화 (빠른 반응을 위해 임계값 낮춤)
        command_changed = new_command != self.previous_auto_command
        if command_changed:
            self.command_count = 0
        else:
            self.command_count += 1
            
        # ⚡ 빠른 반응: 1번만 같으면 바로 실행, 정지는 즉시 실행
        if self.command_count >= self.stability_threshold or new_command == 'S':
            self.auto_command = new_command
            
        self.previous_auto_command = new_command
        
        # 🔍 5초마다 또는 명령이 변경될 때만 상세 출력
        current_time = time.time()
        should_output_details = (
            current_time - self.last_scan_time >= self.scan_output_interval or
            command_changed or 
            new_command == 'S'  # 정지 명령은 항상 출력
        )
        
        if should_output_details:
            self.last_scan_time = current_time
            
            # 🔍 상세한 디버깅 정보 출력
            print("\n" + "="*80)
            print(f"🤖 자동 모드 - 라이다 스캔 분석")
            print(f"전체 포인트: {total_points}개, 스캔 범위: {msg.angle_min*180/3.14159:.1f}° ~ {msg.angle_max*180/3.14159:.1f}°")
            print(f"각도 해상도: {msg.angle_increment*180/3.14159:.2f}°")
            print("-"*80)
            
            # 영역별 상세 정보
            print(f"📍 감지 영역 설정:")
            print(f"  전방 영역: ±{self.front_angle}° (인덱스 {front_start}~{total_points-1}, 0~{front_end-1})")
            print(f"  좌측 영역: {self.front_angle}°~{self.side_angle + self.front_angle}° (인덱스 {left_start}~{left_end-1})")
            print(f"  우측 영역: {360-self.side_angle-self.front_angle}°~{360-self.front_angle}° (인덱스 {right_start}~{right_end-1})")
            
            print(f"📏 거리 측정 결과:")
            print(f"  전방 최단거리: {front_min:.3f}m (각도: {front_obstacle_angle:.1f}°)")
            print(f"  좌측 최단거리: {left_min:.3f}m (각도: {left_obstacle_angle:.1f}°)")
            print(f"  우측 최단거리: {right_min:.3f}m (각도: {right_obstacle_angle:.1f}°)")
            
            # 위험도 평가
            front_status = "🔴위험" if front_min < self.danger_threshold else "🟡주의" if front_min < self.safe_threshold else "🟢안전"
            left_status = "🔴위험" if left_min < self.danger_threshold else "🟡주의" if left_min < self.safe_threshold else "🟢안전"
            right_status = "🔴위험" if right_min < self.danger_threshold else "🟡주의" if right_min < self.safe_threshold else "🟢안전"
            
            print(f"⚠️  위험도 평가:")
            print(f"  전방: {front_status} (기준: 위험<{self.danger_threshold}m, 안전>{self.safe_threshold}m)")
            print(f"  좌측: {left_status}")
            print(f"  우측: {right_status}")
            
            print(f"🧠 의사결정:")
            print(f"  이전 명령: {self.previous_auto_command}")
            print(f"  새 명령: {new_command}")
            print(f"  명령 변경: {'예' if command_changed else '아니오'}")
            print(f"  안정화 카운트: {self.command_count}/2")
            print(f"  최종 명령: {self.auto_command}")
            print(f"  결정 이유: {decision_reason}")
        
        # 간단한 로그는 항상 출력 (하지만 덜 자주)
        if current_time - getattr(self, 'last_simple_log', 0) >= 1.0:  # 1초마다로 단축
            self.last_simple_log = current_time
            front_status = "🔴위험" if front_min < self.danger_threshold else "🟡주의" if front_min < self.safe_threshold else "🟢안전"
            left_status = "🔴위험" if left_min < self.danger_threshold else "🟡주의" if left_min < self.safe_threshold else "🟢안전"
            right_status = "🔴위험" if right_min < self.danger_threshold else "🟡주의" if right_min < self.safe_threshold else "🟢안전"
            self.get_logger().info(f"⚡ AUTO | F:{front_min:.2f}m({front_status}) L:{left_min:.2f}m({left_status}) R:{right_min:.2f}m({right_status}) -> {self.auto_command}")

    def get_decision_reason(self, front, left, right, command):
        """의사결정 이유를 반환"""
        if front < self.danger_threshold:
            if left > right and left > self.danger_threshold:
                return f"전방위험({front:.2f}m) → 좌측이 더 안전({left:.2f}m > {right:.2f}m)"
            elif right > left and right > self.danger_threshold:
                return f"전방위험({front:.2f}m) → 우측이 더 안전({right:.2f}m > {left:.2f}m)"
            else:
                return f"전방위험({front:.2f}m) → 좌우 모두 위험, 후진"
        
        if left < self.danger_threshold and right < self.danger_threshold:
            if front > self.safe_threshold:
                return f"좌우 모두 위험 → 전방 안전({front:.2f}m), 전진"
            else:
                return f"좌우 모두 위험, 전방도 불안전 → 후진"
        
        if left < self.danger_threshold:
            return f"좌측 위험({left:.2f}m) → 우회전"
        if right < self.danger_threshold:
            return f"우측 위험({right:.2f}m) → 좌회전"
            
        if front > self.safe_threshold:
            return f"모든 방향 안전 → 직진"
        else:
            wider_side = "좌측" if left > right else "우측"
            return f"전방 주의({front:.2f}m) → {wider_side}이 더 넓음"

    def decide_auto_movement(self, front, left, right):
        # 위험 상황 체크
        if front < self.danger_threshold:
            if left > right and left > self.danger_threshold:
                return 'L'
            elif right > left and right > self.danger_threshold:
                return 'R'
            else:
                return 'B'  # 후진
        
        # 좁은 공간
        if left < self.danger_threshold and right < self.danger_threshold:
            if front > self.safe_threshold:
                return 'F'
            else:
                return 'B'
        
        # 한쪽 벽 회피
        if left < self.danger_threshold:
            return 'R'
        if right < self.danger_threshold:
            return 'L'
            
        # 안전한 상황
        if front > self.safe_threshold:
            return 'F'
        else:
            return 'L' if left > right else 'R'

    def auto_control_update(self):
        if not self.auto_mode:
            return

        # 자동 명령을 모터 속도로 변환
        prev_left = self.left_speed
        prev_right = self.right_speed
        
        if self.auto_command == 'F':      # 전진
            self.left_speed = 150
            self.right_speed = -150
        elif self.auto_command == 'B':    # 후진
            self.left_speed = -120
            self.right_speed = 120
        elif self.auto_command == 'L':    # 좌회전
            self.left_speed = -130
            self.right_speed = -130
        elif self.auto_command == 'R':    # 우회전
            self.left_speed = 130
            self.right_speed = 130
        elif self.auto_command == 'S':    # 정지
            self.left_speed = 0
            self.right_speed = 0

        # 속도가 변경된 경우에만 출력
        if prev_left != self.left_speed or prev_right != self.right_speed:
            direction_map = {
                'F': '⬆️전진', 'B': '⬇️후진', 'L': '⬅️좌회전', 
                'R': '➡️우회전', 'S': '⏹️정지'
            }
            
            print(f"\n🎮 모터 제어 업데이트:")
            print(f"  명령: {self.auto_command} ({direction_map.get(self.auto_command, '알수없음')})")
            print(f"  새로운 속도: 좌측={self.left_speed}, 우측={self.right_speed}")
            print(f"  변경사항: 좌측 {self.left_speed - prev_left:+d}, 우측 {self.right_speed - prev_right:+d}")
            
            # 즉시 상태바 업데이트
            mode_indicator = "🤖"
            auto_cmd_info = f" [{self.auto_command}]"
            print(f"{mode_indicator} 현재속도 - 좌측: {self.left_speed:4d}, 우측: {self.right_speed:4d}{auto_cmd_info}")

        self.send_motor_command()

    def update_status_display(self):
        """자동 모드에서도 현재 속도를 5초마다 표시"""
        if self.auto_mode and not self.emergency_stop:
            mode_indicator = "🤖"
            auto_cmd_info = f" [{self.auto_command}]"
            print(f"\n{mode_indicator} 현재 상태 - 좌측: {self.left_speed:4d}, 우측: {self.right_speed:4d}{auto_cmd_info} | 자동모드 실행중...")

    def run(self):
        if not self.settings:
            self.get_logger().error("터미널 설정 실패")
            return

        try:
            while True:
                key = self.get_key()

                # 모드 전환 및 특수 키
                if key == 'F1':
                    self.auto_mode = False
                    self.emergency_stop = False
                    self.left_speed = 0
                    self.right_speed = 0
                    print("\n🕹️  수동 모드로 전환!")
                    
                elif key == 'F2':
                    self.auto_mode = True
                    self.emergency_stop = False
                    print("\n🤖 자동 모드로 전환!")
                    
                elif key == 'ESC':
                    self.emergency_stop = True
                    self.left_speed = 0
                    self.right_speed = 0
                    print("\n🚨 긴급 정지!")

                elif key == '\x03':  # Ctrl+C
                    break

                # 수동 모드에서만 키보드 조작 허용
                if not self.auto_mode and not self.emergency_stop:
                    if key == 'w':
                        self.left_speed = 175
                        self.right_speed = -175
                    elif key == 's':
                        self.left_speed = -175
                        self.right_speed = 175
                    elif key == 'a':
                        self.left_speed = -175
                        self.right_speed = -175
                    elif key == 'd':
                        self.left_speed = 175
                        self.right_speed = 175
                    elif key == ' ':
                        self.left_speed = 0
                        self.right_speed = 0
                    elif key == 'r':
                        self.left_speed = 0
                        self.right_speed = 0
                    elif key == 'q':
                        self.left_speed = self.clamp_speed(self.left_speed + self.speed_step)
                    elif key == 'z':
                        self.left_speed = self.clamp_speed(self.left_speed - self.speed_step)
                    elif key == 'e':
                        self.right_speed = self.clamp_speed(self.right_speed + self.speed_step)
                    elif key == 'c':
                        self.right_speed = self.clamp_speed(self.right_speed - self.speed_step)
                    elif key == 'k':  # 가속
                        if self.left_speed > 0:
                            self.left_speed = self.clamp_speed(self.left_speed + 10)
                        elif self.left_speed < 0:
                            self.left_speed = self.clamp_speed(self.left_speed - 10)
                        if self.right_speed > 0:
                            self.right_speed = self.clamp_speed(self.right_speed + 10)
                        elif self.right_speed < 0:
                            self.right_speed = self.clamp_speed(self.right_speed - 10)
                    elif key == 'l':  # 감속
                        if self.left_speed < 0:
                            self.left_speed = self.clamp_speed(self.left_speed + 10)
                        elif self.left_speed > 0:
                            self.left_speed = self.clamp_speed(self.left_speed - 10)
                        if self.right_speed < 0:
                            self.right_speed = self.clamp_speed(self.right_speed + 10)
                        elif self.right_speed > 0:
                            self.right_speed = self.clamp_speed(self.right_speed - 10)

                # 수동 모드에서 키 입력 시 명령 전송
                if key and key != '\x03' and not self.auto_mode:
                    self.send_motor_command()
                    
                # 모드에 상관없이 현재 속도 표시
                if key and key != '\x03':
                    mode_indicator = "🤖" if self.auto_mode else "🕹️ "
                    emergency_indicator = " 🚨" if self.emergency_stop else ""
                    auto_cmd_info = f" [{self.auto_command}]" if self.auto_mode else ""
                    print(f"\r{mode_indicator} 좌측: {self.left_speed:4d}, 우측: {self.right_speed:4d}{auto_cmd_info}{emergency_indicator} | 키: {key}", end='', flush=True)

        except KeyboardInterrupt:
            pass
        finally:
            # 종료 시 모터 정지
            self.left_speed = 0
            self.right_speed = 0
            self.send_motor_command()

            if self.settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
            if self.arduino_connected and self.arduino:
                self.arduino.close()
                
            self.get_logger().info("🛑 시스템 종료")

def main(args=None):
    rclpy.init(args=args)
    controller = HybridBoatController()

    if not controller.settings:
        controller.destroy_node()
        rclpy.shutdown()
        return

    # ROS2 스핀을 별도 스레드로 실행
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(controller,))
    ros_thread.daemon = True
    ros_thread.start()

    # 메인 스레드에서 키보드 처리
    controller.run()
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
