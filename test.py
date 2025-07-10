from re import search, escape

import cv2
import subprocess
import uptech
import time
import apriltag
import numpy as np
import signal
import threading

your_team_blue = 1  # 1为蓝队，0为黄队
down_time_value = 200 * 400  # 灰度误差累加
down_value = 0.5*(2000+1700)  # 灰度台上台下临界值
escape_value = 100  # 逃逸时间重置
dead_area = 250  # 死区电压
escape_time = 100  # 逃逸时间
tag_lock_time_value = 100  # 持续锁定

execution_time = 0
go_flag = 0
black_detect = 0
io_data = []
adc_value = []
camera_reset = 0
camera_reload = 1
camera_time = 0
camera_safe = 0
tag_safe = 0
tag_flag = 1
blue_detected = 0
mid = 0
tag_width = 0
tags = []
distance = 0
tag_lock_time = tag_lock_time_value
tag_lock_flag = 0
index = 0
flag = 0
cnt = 0
cx = 0
cy = 0
c_time=0

last_time = 0
tai_flag = 0
tai_flag_time = 0
escape_flag_right = 0
escape_flag_left = 0
down = 1
up_flag = 0
t = 0
check_right_time = 0
check_left_time = 0
check_down_time = 0
adc_last_0 = 0
adc_last_1 = 0
adc_last_2 = 0
adc_last_3 = 0
adc_last_4 = 0
unify_all = 0
buffer = 0
if your_team_blue:
    di_fang_kuai = 2  # 敌方块
    zhong_li_kuai = 0  # 中立块
    zha_dan_kuai = 1  # 炸弹块
else:
    di_fang_kuai = 1  # 敌方块
    zhong_li_kuai = 0  # 中立块
    zha_dan_kuai = 2  # 炸弹块


class ApriltagDetect:
    def __init__(self):
        self.target_id = 0
        self.at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9'))

    def update_frame(self, frame):  # 敌方优先
        h0 = 0  # shi fou you 0 ma
        h1 = 0  # shi fou you 1 ma
        h2 = 0
        m1 = 0  # zui zhong xin 1 ma zhong xin zuo biao
        m0 = 0  # zui zhong xin 0 ma zhong xin zuo biao
        m2 = 0
        mx0 = 0
        mx1 = 0  # ma zhi zhong xin zuo biao
        mx2 = 0
        mid0 = 0
        mid1 = 0
        mid2 = 0
        global tag_flag, tag_safe, go_flag
        global index
        global mid
        global tag_width
        global tags
        global distance
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)
        tag_flag = 0
        index = 0

        # 新增：初始化敌方块计数器
        enemy_count = 0

        if tags:
            tag_flag = 1  # 这是个标志位

            # 新增：首先统计所有敌方块数量
            for tag in tags:
                if tag.tag_id == di_fang_kuai:
                    enemy_count += 1

            # 原有逻辑保持不变
            for i in range(1, len(tags)):
                # 循环从第二个（results[1]）索引开始，进行冒泡排序。（因为前方index是从零开始的）所以排序没有遗漏
                if tags[i].tag_id == di_fang_kuai or tags[i].tag_id == zhong_li_kuai:
                    # 如果tag码块id是敌方或中立才进行距离比较，炸弹不管
                    if tags[index].tag_id == zha_dan_kuai:
                        # 这一步确保index=0的那个id不是炸弹，若是炸弹，则将索引就改成i（i现在肯定不是炸弹）
                        index = i
                    if tags[i].tag_id == zhong_li_kuai:
                        if tags[index].tag_id == zhong_li_kuai:
                            if (self.get_distance(tags[index].homography, 4300) >
                                    self.get_distance(tags[i].homography, 4300)):  # 冒泡排序
                                index = i
                    elif tags[i].tag_id == di_fang_kuai:
                        if tags[index].tag_id == di_fang_kuai:
                            if (self.get_distance(tags[index].homography, 4300) >
                                    self.get_distance(tags[i].homography, 4300)):  # 冒泡排序
                                index = i
                        elif tags[index].tag_id == zhong_li_kuai:
                            index = i
                else:
                    x_distance = int(self.get_distance(tags[index].homography, 4300))
                    x_mid = tuple(tags[index].corners[0].astype(int))[0] / 2 + \
                            tuple(tags[index].corners[2].astype(int))[0] / 2  # 计算tag的横向位置
                    if x_distance < 120 and 140 < x_mid < 180:
                        index = i
                if tags[0].tag_id == zha_dan_kuai:
                    x_distance = int(self.get_distance(tags[0].homography, 4300))
                    x_mid = tuple(tags[0].corners[0].astype(int))[0] / 2 + \
                            tuple(tags[0].corners[2].astype(int))[0] / 2  # 计算tag的横向位置
                    if x_distance < 120 and 140 < x_mid < 180:
                        index = 0
            if tags[index].tag_id == di_fang_kuai or tags[index].tag_id == zhong_li_kuai:  # 冒泡后如果最近的id是中立或敌方
                tag_safe = 1
            else:  # 冒泡后如果的id是炸弹块(侧面证明了没有检测到敌方和中立)
                tag_safe = 0
            distance = int(self.get_distance(tags[index].homography, 4300))
            mid = tuple(tags[index].corners[0].astype(int))[0] / 2 + \
                  tuple(tags[index].corners[2].astype(int))[0] / 2  # 计算tag的横向位置
            tag_width = abs(tuple(tags[index].corners[0].astype(int))[0] - tuple(tags[index].corners[2].astype(int))[0])
        else:
            tag_flag = 0

        # 新增：根据敌方块数量设置go_flag
        go_flag = 1 if enemy_count == 2 else 0

    def get_distance(self, H, t):
        ss = 0.5
        src = np.array([[-ss, -ss, 0],
                        [ss, -ss, 0],
                        [ss, ss, 0],
                        [-ss, ss, 0]])
        Kmat = np.array([[700, 0, 0],
                         [0, 700, 0],
                         [0, 0, 1]]) * 1.0
        disCoeffs = np.zeros([4, 1]) * 1.0
        ipoints = np.array([[-1, -1],
                            [1, -1],
                            [1, 1],
                            [-1, 1]])
        for point in ipoints:
            x = point[0]
            y = point[1]
            z = H[2, 0] * x + H[2, 1] * y + H[2, 2]
            point[0] = (H[0, 0] * x + H[0, 1] * y + H[0, 2]) / z * 1.0
            point[1] = (H[1, 0] * x + H[1, 1] * y + H[1, 2]) / z * 1.0
        campoint = ipoints * 1.0
        opoints = np.array([[-1.0, -1.0, 0.0],
                            [1.0, -1.0, 0.0],
                            [1.0, 1.0, 0.0],
                            [-1.0, 1.0, 0.0]])
        opoints = opoints * 0.5
        rate, rvec, tvec = cv2.solvePnP(opoints, campoint, Kmat, disCoeffs)
        point, jac = cv2.projectPoints(src, np.zeros(rvec.shape), tvec, Kmat, disCoeffs)
        points = np.int32(np.reshape(point, [4, 2]))
        distance = np.abs(t / np.linalg.norm(points[0] - points[1]))
        return distance


def April_start_detect():
    global frame, blue_detected, cx, cy, camera_safe, camera_reload, last_time, camera_time, camera_reset
    global black_detect, tag_lock_flag
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(3, 320)
    cap.set(4, 240)
    cap.set(cv2.CAP_PROP_FPS, 60)
    ad = ApriltagDetect()
    while True:
        ret, frame = cap.read()
        if camera_reset:
            cap.set(3, 320)
            cap.set(4, 240)
            cap.set(cv2.CAP_PROP_FPS, 60)
            time.sleep(0.5)
            camera_reset = 0
        # if camera_reload:
        #     camera_reload = 0
        #     ret = 0
        if not ret or frame is None:
            print("摄像头断开连接")
            camera_safe = 0
            cap.release()
            time.sleep(0.1)
            print("正在尝试重连")
            subprocess.check_call("sudo modprobe -rf uvcvideo", shell=True)
            time.sleep(0.4)
            subprocess.check_call("sudo modprobe uvcvideo", shell=True)
            time.sleep(0.2)
            cap = cv2.VideoCapture('/dev/video0')
            camera_reset = 1
            continue
        else:
            camera_safe = 1
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        ad.update_frame(frame)
        if down:
            # 转换为HSV颜色空间
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 定义黑色的HSV范围（黑色检测）
            # 黑色通常具有较低的亮度值（V通道）
            lower_black = np.array([0, 0, 0])
            upper_black = np.array([179, 162, 168])  # V值上限设为50（较暗的区域）

            # 创建掩膜
            mask = cv2.inRange(hsv, lower_black, upper_black)

            # 形态学操作（可选，用于降噪）
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=1)

            # ========== 新增：计算黑色区域占比 ==========
            total_pixels = frame.shape[0] * frame.shape[1]
            black_pixels = cv2.countNonZero(mask)
            black_ratio = black_pixels / total_pixels
            black_percentage = black_ratio * 100

            # 检查黑色占比是否超过阈值
            if black_percentage > 85:
                # print(f"黑色区域占比: {black_percentage:.2f}% > 70% - 设置 black_detect=1")
                black_detect = 1  # 设置全局变量为1
            else:
                black_detect = 0  # 设置全局变量为-1

            # 在画面中显示黑色占比信息
            cv2.putText(frame, f"Black: {black_percentage:.2f}%", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 0), 2)
            # ========================================

            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # 标记坐标的列表
            coordinates = []
            if contours:
                # 找到最大轮廓
                largest_contour = max(contours, key=cv2.contourArea)
                # 计算轮廓中心
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    blue_detected = 1  # 1表示检测到黑色物体
                    coordinates.append((cx, cy))
                    # 在画面中标记中心点
                    cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                    cv2.putText(frame, f"({cx}, {cy})", (cx - 50, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # time.sleep(0.01)
        # 显示结果
        # cv2.imshow('Camera', frame)
        # cv2.imshow('Mask', mask)
        # if tags:
        #     # print(tags)
        #     # print(index)
        #     print(f"中心位置{mid}")
        #     print(f"距离{distance}")
        #     print(f"宽度{tag_width}")
        #     if tag_safe == 0:
        #         print("炸弹")
        #     else:
        #         if tags[index].tag_id == 1:
        #             print("敌方")
        #         elif tags[index].tag_id == 0:
        #             print("中立")
        # cv2.imshow("img", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


def April_tag_move():
    global tag_lock_flag
    tag_lock_flag = 1
    if distance > 170:
        if mid < 160 - tag_width / 3:
            left(500)
        elif mid > 160 + tag_width / 3:
            right(500)
        else:
            straight_if()
    else:
        if io_data[0] == 1 and io_data[1] == 0 and not escape_flag_right:
            right(500)
        elif io_data[0] == 0 and io_data[1] == 1 and not escape_flag_left:
            left(500)
        else:
            straight_if()


def April_tag_escape():
    global escape_flag_right, escape_flag_left
    if distance < 220:
        stop()
        time.sleep(0.05)
        back(800)
        time.sleep(0.05)
        right(1000)
        time.sleep(0.5)


def signal_handler(handler_signal, handler_frame):
    stop()
    exit(0)


def straight(speed):
    up.CDS_SetSpeed(1, -speed)
    up.CDS_SetSpeed(2, -speed)


def straight_if():
    if unify_all > down_value + 4000:
        straight(1000)
    elif unify_all > down_value + 3000:
        straight(700)
    else:
        straight(600)


def stop():
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)


def back(speed):
    up.CDS_SetSpeed(1, speed)
    up.CDS_SetSpeed(2, speed)


def back_sleep():
    stop()
    time.sleep(0.01)
    back(400)
    time.sleep(0.01)
    back(1000)
    time.sleep(0.2)


def left(speed):
    up.CDS_SetSpeed(1, speed)
    up.CDS_SetSpeed(2, -speed)


def right(speed):
    up.CDS_SetSpeed(1, -speed)
    up.CDS_SetSpeed(2, speed)


def get_io_data(up):
    io_all_input = up.ADC_IO_GetAllInputLevel()
    io_array = '{:08b}'.format(io_all_input)
    io_data = []
    for index, value in enumerate(io_array):
        io = int(value)
        io_data.insert(0, io)
    return io_data


def check_time():
    global check_left_time, check_right_time, check_down_time, down, escape_time, escape_flag_right, \
        escape_flag_left, tag_lock_time, tag_lock_flag

    if io_data[6] == 0:
        check_left_time += 1
    else:
        check_left_time = 0
    if io_data[7] == 0:
        check_right_time += 1
    else:
        check_right_time = 0
    # if unify_all < down_value:
    #     check_down_time += down_value - unify_all
    # else:
    #     check_down_time = 0
    #     down = 0
    if unify_all<down_value:
        down=1
    else:
        down=0
    # if not down:
    #     if check_down_time >= down_time_value:
    #         down = 1
    if escape_flag_left or escape_flag_right:
        escape_time -= 1
        if escape_time <= 0:
            escape_time = escape_value
            escape_flag_left = 0
            escape_flag_right = 0
    if tag_lock_flag:
        tag_lock_time -= 1
        if tag_lock_time <= 0:
            tag_lock_time = tag_lock_time_value
            tag_lock_flag = 0


def down_act():
    global tai_flag, up_flag, buffer, down,c_time
    if up_flag:
        stop()
        time.sleep(0.5)
        back(400)
        time.sleep(1)
        back(1000)
        time.sleep(1)
        # while_sleep_up(130)
        stop()
        time.sleep(0.1)
        right(1000)
        time.sleep(0.4)
        up_flag = 0
        down = 0
        buffer = 20
    else:
        if io_data[0] == 0 and io_data[1] == 0:
            up_flag = 1
        else:
            c_time+=1
            if c_time<300:
                right(800)
            else:
                left(800)
            if c_time==1200:
                c_time=0


def up_act():
    if io_data[3] == 0 and io_data[4] == 0:
        if tag_flag:
            if tag_safe:
                April_tag_move()
            else:
                April_tag_escape()
        else:
            if io_data[0] == 0 and io_data[1] == 0:
                straight_if()
            elif io_data[0] == 1 and io_data[1] == 0 and not escape_flag_right:
                 right(700)
            elif io_data[0] == 0 and io_data[1] == 1 and not escape_flag_left:
                 left(700)
            else:
                search_left_and_right()
    elif io_data[3] == 1 and io_data[4] == 0:
        back_sleep()
        right(1000)
        if tag_safe and tag_flag:
            time.sleep(0.2)
        else:
            time.sleep(0.3)
    elif io_data[3] == 0 and io_data[4] == 1:
        back_sleep()
        left(1000)
        if tag_safe and tag_flag:
            time.sleep(0.2)
        else:
            time.sleep(0.3)
    else:
        back_sleep()
        right(1000)



def search_left_and_right():
    global check_left_time, check_right_time, t, io_data, adc_value
    if check_right_time >= 4:
        while True:
            t += 1
            io_data = get_io_data(up)
            right(700)
            if io_data[0] == 0 and io_data[1] == 0 or t >= 200:
                t = 0
                check_right_time = 0
                break
    elif check_left_time >= 4:
        while True:
            t += 1
            io_data = get_io_data(up)
            left(700)
            if io_data[0] == 0 and io_data[1] == 0 or t >= 200:
                t = 0
                check_left_time = 0
                break
    else:
        straight_if()


def Print():
    while True:
        print(execution_time)


if __name__ == "__main__":
    up = uptech.UpTech()
    up.LCD_Open(2)
    up.ADC_IO_Open()
    up.CDS_Open()
    up.ADC_Led_SetColor(0, 0x2F0000)
    up.ADC_Led_SetColor(1, 0x002F00)
    up.CDS_SetMode(1, 1)
    up.CDS_SetMode(2, 1)
    print("test succeed")
    signal.signal(signal.SIGINT, signal_handler)
    target2 = threading.Thread(target=April_start_detect)
    target2.start()
    # target3 = threading.Thread(target=Print)
    # target3.start()
    print("Ready——")
    while True:
        io_data = get_io_data(up)
        if io_data[6] == 0 and io_data[7] == 0:
            break
    print("Go!!")
    while True:
        # start_time = time.time()
        adc_value = up.ADC_Get_All_Channle()
        unify_all = adc_value[0] + adc_value[1] + adc_value[2] + adc_value[3] + adc_value[4]
        io_data = get_io_data(up)
        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_GBLUE)
        # up.LCD_PutString(0, 0, 'Go North All')
        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_YELLOW)
        up.LCD_PutString(0, 0, f'{unify_all:.2f}')
        up.LCD_Refresh()
        # 0、1 正前方红外   3、4斜向下   6、7左右
        # print(io_data)
        # print(adc_value)
        # straight(500,500)
        # time.sleep(3)
        # print(unify_all)
        if camera_safe:
            check_time()
            if down:
                down_act()
            else:
                up_act()
        else:
            stop()
        # end_time = time.time()  # 记录循环结束的时间
        # execution_time = end_time - start_time  # 计算执行时间
