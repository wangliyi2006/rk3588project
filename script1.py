import cv2
import mediapipe as mp
from flask import Flask, Response
import serial
import time
import os
import threading
import math
import re

app = Flask(__name__)

# ================= 1. 连接底盘小脑 =================
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
    print(" 成功连接底盘小脑 (STM32)！")
except Exception as e:
    print("串口未连接:", e)
    ser = None

last_command = 'S'


def send_car_cmd(cmd):
    global last_command
    if not ser: return

    # 疯狂喂狗，防止 STM32 看门狗锁死
    ser.write(cmd.encode('utf-8'))

    if cmd != last_command:
        if cmd == 'S':
            print(" 状态: 刹车锁定")
        else:
            print(f" 发送动作: {cmd}")
        last_command = cmd


# ================= 2. 里程计与轨迹全局变量 =================
robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0
last_m2, last_m4 = 0, 0
is_first_reading = True
is_returning = False

trajectory = []
last_record_x, last_record_y = 0.0, 0.0

METERS_PER_TICK = 0.0005
WHEEL_BASE = 0.25


def odometry_worker():
    global robot_x, robot_y, robot_theta, last_m2, last_m4, is_returning, is_first_reading
    global trajectory, last_record_x, last_record_y

    print(" 里程计系统启动 (搭载防溢出金钟罩)...")
    while True:
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                match = re.search(r'M2:(-?\d+)\s+M4:(-?\d+)', line)
                if match:
                    current_m2 = int(match.group(1))
                    current_m4 = int(match.group(2))

                    if is_first_reading:
                        last_m2, last_m4 = current_m2, current_m4
                        is_first_reading = False
                        continue

                        # 计算增量
                    delta_m2 = current_m2 - last_m2
                    delta_m4 = current_m4 - last_m4

                    #  终极防爆表机制 (处理 16位 STM32 溢出)
                    # 当数值越过 32767 变成负数时，强行接上，防止坐标暴走到 -100米！
                    if delta_m2 < -30000:
                        delta_m2 += 65536
                    elif delta_m2 > 30000:
                        delta_m2 -= 65536

                    if delta_m4 < -30000:
                        delta_m4 += 65536
                    elif delta_m4 > 30000:
                        delta_m4 -= 65536

                    # 更新历史记录
                    last_m2, last_m4 = current_m2, current_m4

                    # 计算距离
                    dist_left = delta_m2 * METERS_PER_TICK
                    dist_right = delta_m4 * METERS_PER_TICK

                    dist_center = (dist_left + dist_right) / 2.0
                    delta_theta = (dist_right - dist_left) / WHEEL_BASE

                    robot_x += dist_center * math.cos(robot_theta)
                    robot_y += dist_center * math.sin(robot_theta)
                    robot_theta += delta_theta

                    if robot_theta > math.pi: robot_theta -= 2 * math.pi
                    if robot_theta < -math.pi: robot_theta += 2 * math.pi

                    # 记录轨迹点（10厘米记录一个，不密不疏刚刚好）
                    if not is_returning:
                        dist_moved = math.hypot(robot_x - last_record_x, robot_y - last_record_y)
                        if dist_moved > 0.10:
                            trajectory.append((robot_x, robot_y))
                            last_record_x, last_record_y = robot_x, robot_y
            except:
                pass
        time.sleep(0.01)


# ================= 3.  轨迹回放倒车控制器 (多线程安全+丝滑微调版)  =================
def trajectory_replay_controller():
    global is_returning, trajectory, robot_x, robot_y, robot_theta
    if len(trajectory) < 1:
        send_car_cmd('S')
        is_returning = False
        return

    #  修正 1：使用 .copy() 建立局部快照，防止多线程同时读写冲突
    local_trajectory = trajectory.copy()

    #  修正 2：立刻清空全局轨迹，防止倒车过程中里程计继续往里面写入倒车轨迹
    trajectory.clear()

    local_trajectory.reverse()
    print(f"\n 开始沿轨迹原路倒车！共 {len(local_trajectory)} 个路径点。")

    for target_x, target_y in local_trajectory:
        if not is_returning: break

        #  用于记录离这个点最近的距离
        min_dist_to_pt = 999.0

        while is_returning:
            # 1. 全局上帝视角：只要离原点小于 0.35 米，立刻拉闸，不管剩下多少点！
            if math.hypot(robot_x, robot_y) < 0.35:
                send_car_cmd('S')
                print(" 成功退回原点，强制结束返航！")
                is_returning = False
                return

            dist = math.hypot(target_x - robot_x, target_y - robot_y)
            # 2. 正常吃到面包屑
            if dist < 0.20:
                break

            #  3. 过冲检测
            if dist < min_dist_to_pt:
                min_dist_to_pt = dist  # 还在靠近，更新最近距离
            elif dist > min_dist_to_pt + 0.15:  # 修正 3：适当放宽到 0.15米，允许小车微步调整时的物理摆动
                print(f"检测到过冲 (当前:{dist:.2f}m, 最近:{min_dist_to_pt:.2f}m)，跳过该点")
                break

            target_angle = math.atan2(target_y - robot_y, target_x - robot_x)
            expected_theta = target_angle + math.pi
            if expected_theta > math.pi: expected_theta -= 2 * math.pi
            if expected_theta < -math.pi: expected_theta += 2 * math.pi

            angle_error = expected_theta - robot_theta
            while angle_error > math.pi: angle_error -= 2 * math.pi
            while angle_error < -math.pi: angle_error += 2 * math.pi

            #  修正 4：配合硬件微步，将转向死区从 0.3 稍微收紧到 0.25，让倒车轨迹更精准
            if angle_error > 0.25:
                send_car_cmd('E')
            elif angle_error < -0.25:
                send_car_cmd('Q')
            else:
                send_car_cmd('B')

            time.sleep(0.05)

    send_car_cmd('S')
    print("✅ 轨迹走完，退回起点！")
    is_returning = False


# ================= 4. 摄影师 =================
latest_raw_frame = None
latest_drawn_jpeg = None


def camera_worker():
    global latest_raw_frame
    cap = cv2.VideoCapture(11, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    while True:
        ret, frame = cap.read()
        if ret:
            latest_raw_frame = frame.copy()
        else:
            time.sleep(0.01)


# ================= 5. AI 大脑 =================
def analyze_person(landmarks, mp_pose):
    r_eye = landmarks.landmark[mp_pose.PoseLandmark.RIGHT_EYE]
    r_shoulder = landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    l_shoulder = landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
    r_wrist = landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]
    l_wrist = landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST]

    gesture = "WAITING"
    person_center_x = (l_shoulder.x + r_shoulder.x) / 2.0
    shoulder_width = abs(l_shoulder.x - r_shoulder.x)

    if r_wrist.visibility > 0.6 and l_wrist.visibility > 0.6:
        if abs(r_wrist.x - l_wrist.x) < 0.15 and r_wrist.y < r_shoulder.y and l_wrist.y < l_shoulder.y:
            return "RETURN_HOME_HOLDING"

    if r_wrist.visibility > 0.6 and r_wrist.y < r_eye.y and l_wrist.y > l_shoulder.y:
        #  加宽直行死区 + 方向逻辑镜像修复
        if shoulder_width > 0.45:
            gesture = "ARRIVED"
        else:
            if person_center_x < 0.35:
                gesture = "COME_LEFT"  # 车向左靠拢
            elif person_center_x > 0.65:
                gesture = "COME_RIGHT"  # 车向右靠拢
            else:
                gesture = "COME_FORWARD"  # 人在正前方宽阔区域内，稳稳直行

    return gesture


def ai_brain_worker():
    global latest_raw_frame, latest_drawn_jpeg, is_returning, robot_x, robot_y

    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(static_image_mode=False, model_complexity=1, min_detection_confidence=0.5,
                        min_tracking_confidence=0.5)
    mp_draw = mp.solutions.drawing_utils
    print(" AI 大脑已启动！")

    return_hold_time = 0.0

    while True:
        if latest_raw_frame is None:
            time.sleep(0.01)
            continue

        try:
            frame = latest_raw_frame.copy()
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(img_rgb)
            status_text = "Status: WAITING"

            if is_returning:
                status_text = "Auto Reversing..."
            elif results.pose_landmarks:
                gesture = analyze_person(results.pose_landmarks, mp_pose)

                # 1.5秒读条防误触
                if gesture == "RETURN_HOME_HOLDING":
                    if return_hold_time == 0:
                        return_hold_time = time.time()

                    hold_duration = time.time() - return_hold_time
                    status_text = f"HOLD X: {hold_duration:.1f}s"
                    send_car_cmd('S')

                    if hold_duration > 1.5:
                        status_text = "Reversing!"
                        is_returning = True
                        threading.Thread(target=trajectory_replay_controller, daemon=True).start()
                        return_hold_time = 0
                else:
                    return_hold_time = 0

                    if gesture == "COME_FORWARD":
                        status_text = "GO STRAIGHT";
                        send_car_cmd('F')

                        #  恢复纯粹指令：底层 STM32 会负责微步点刹，Python 只管发
                    elif gesture == "COME_LEFT":
                        status_text = "TURN LEFT";
                        send_car_cmd('L')
                    elif gesture == "COME_RIGHT":
                        status_text = "TURN RIGHT";
                        send_car_cmd('R')

                    elif gesture == "ARRIVED":
                        status_text = "Goal Reached!";
                        send_car_cmd('S')
                    else:
                        status_text = "WAITING";
                        send_car_cmd('S')

                mp_draw.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
            else:
                return_hold_time = 0
                send_car_cmd('S')

                # OSD 字幕：显示坐标
            cv2.putText(frame, status_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            cv2.putText(frame, f"X:{robot_x:.2f}m Y:{robot_y:.2f}m", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (255, 255, 0), 2)

            ret, buffer = cv2.imencode('.jpg', frame)
            if ret: latest_drawn_jpeg = buffer.tobytes()
        except:
            pass


# ================= 6. 网页客服 =================
def generate_frames():
    global latest_drawn_jpeg
    while True:
        if latest_drawn_jpeg is not None:
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + latest_drawn_jpeg + b'\r\n')
        time.sleep(0.05)


@app.route('/')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    send_car_cmd('S')
    threading.Thread(target=camera_worker, daemon=True).start()
    threading.Thread(target=odometry_worker, daemon=True).start()
    threading.Thread(target=ai_brain_worker, daemon=True).start()

    print(" 系统启动！【STM32 硬件微步控制 + 多线程安全回放】")
    app.run(host='0.0.0.0', port=5000)
