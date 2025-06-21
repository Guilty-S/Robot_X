import cv2
import subprocess
import uptech
import time
import apriltag
import numpy as np
import signal
import threading

camera_safe = 0
tag_safe = 0
tag_flag = 1
blue_detected = 0
mid = 0
tag_width = 0
tags = []
distance = 0
# di_fang_kuai = 1  # 敌方块
# zhong_li_kuai = 0  # 中立块
# zha_dan_kuai = 2  # 炸弹块
di_fang_kuai = 1  # 敌方块
zhong_li_kuai = 2  # 中立块
zha_dan_kuai = 0  # 炸弹块
down_time_value = 250
down_value = -700
escape_value = 100
dead_area = 250
index = 0
flag = 0
cnt = 0
cx = 0
cy = 0

camera_reload = 1
tai_flag = 0
tai_flag_time = 0
escape_flag_right = 0
escape_flag_left = 0
escape_time = 50
tag_lock_time_value = 20
tag_lock_time = tag_lock_time_value
tag_lock_flag = 0
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


class PIDController:
    def __init__(self, Kp, Ki, Kd, gkd, out_limit):
        """
        初始化PID控制器。

        参数:
        Kp -- 比例增益
        Ki -- 积分增益
        Kd -- 微分增益
        gkd -- 微分滤波系数
        out_limit -- 输出限制
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.gkd = gkd
        self.out_limit = out_limit

        self.err = 0.0
        self.err_last = 0.0
        self.output = 0.0
        self.setpoint = 0.0
        self.current_value = 0.0

    def calculate(self, target, current):
        """
        计算PID输出。

        参数:
        target -- 目标值
        current -- 当前值

        返回:
        计算得到的PID输出
        """
        self.err = target - current
        self.output = self.Kp * self.err + self.Kd * (self.err - self.err_last)
        self.err_last = self.err

        # 输出限幅
        if self.output > self.out_limit:
            self.output = self.out_limit
        elif self.output < -self.out_limit:
            self.output = -self.out_limit

        return self.output


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
        global tag_flag, tag_safe
        global index
        global mid
        global tag_width
        global tags
        global distance
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)
        tag_flag = 0
        index = 0
        if tags:
            tag_flag = 1  # 这是个标志位
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
                    if x_distance < 120:
                        index = i
                if tags[0].tag_id == zha_dan_kuai:
                    x_distance = int(self.get_distance(tags[0].homography, 4300))
                    x_mid = tuple(tags[0].corners[0].astype(int))[0] / 2 + \
                            tuple(tags[0].corners[2].astype(int))[0] / 2  # 计算tag的横向位置
                    if x_distance < 120:
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
    global frame, blue_detected, cx, cy, camera_safe, camera_reload
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(3, 320)
    cap.set(4, 240)
    cap.set(cv2.CAP_PROP_FPS, 60)
    ad = ApriltagDetect()
    while True:
        ret, frame = cap.read()
        if camera_reload:
            camera_reload = 0
            ret = 0
        if not ret or frame is None:
            print("摄像头断开连接")
            camera_safe = 0
            cap.release()
            time.sleep(0.2)
            print("正在尝试重连")
            subprocess.check_call("sudo modprobe -rf uvcvideo", shell=True)
            time.sleep(0.5)
            subprocess.check_call("sudo modprobe uvcvideo", shell=True)
            time.sleep(0.5)
            cap = cv2.VideoCapture('/dev/video0')
            continue
        else:
            camera_safe = 1
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        # time.sleep(0.05)
        ad.update_frame(frame)
        # 转换为HSV颜色空间（更适合颜色检测）
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 定义蓝色的HSV范围（示例值，需根据实际调整）
        # lower_blue = np.array([100, 150, 50])
        # upper_blue = np.array([140, 255, 255])
        lower_blue = np.array([97, 115, 72])
        upper_blue = np.array([140, 255, 255])
        # 创建掩膜
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        # 形态学操作（可选，用于降噪）
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_detected = 0
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
                blue_detected = 1  # 1表示检测到蓝色物体
                coordinates.append((cx, cy))
                # 在画面中标记中心点
                cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                cv2.putText(frame, f"({cx}, {cy})", (cx - 50, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
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


def color_blue_move():
    if cy > 120:
        if cx < 160 - 40:
            left(500)
            # print("左")
        elif cx > 160 + 40:
            right(500)
            # print("右")
        else:
            straight_if()
            # print("前进")
    elif cy > 80:
        if cx < 160 - 30:
            left(500)
            # print("左")
        elif cx > 160 + 30:
            right(500)
            # print("右")
        else:
            straight_if()
            # print("前进")
    else:
        if cx < 160 - 10:
            left(500)
            # print("左")
        elif cx > 160 + 10:
            right(500)
            # print("右")
        else:
            straight_if()
            # print("前进")


def color_blue_move_pid():
    global control_output, control_output_final
    pid = PIDController(Kp=4, Ki=0, Kd=0.5, gkd=0.0, out_limit=1000.0)
    control_output = pid.calculate(160, cx)  # kp 0~10 kd
    if control_output > 0:
        control_output_final = control_output + dead_area
    else:
        control_output_final = control_output - dead_area
    # print(control_output_final)

    if cx >= 160 - 10 and cx < 160 + 10:
        straight_if()
    else:
        if control_output_final > 1000:
            up.CDS_SetSpeed(1, 1000)
            up.CDS_SetSpeed(2, -1000)
        elif control_output_final < -1000:
            up.CDS_SetSpeed(1, -1000)
            up.CDS_SetSpeed(2, 1000)
        else:
            up.CDS_SetSpeed(1, int(control_output_final))
            up.CDS_SetSpeed(2, -int(control_output_final))


def April_tag_move():
    global tag_lock_flag
    tag_lock_flag=1
    if distance > 170:
        if mid < 160 - tag_width / 3:
            left(500)
            # print("左")
        elif mid > 160 + tag_width / 3:
            right(500)
            # print("右")
        else:
            straight_if()
            # print("前进")
    else:
        if io_data[0] == 1 and io_data[1] == 0 and not escape_flag_right:
            right(500)
        elif io_data[0] == 0 and io_data[1] == 1 and not escape_flag_left:
            left(500)
        else:
            straight_if()


def April_tag_move_pid():
    global tag_control_output, tag_control_output_final, mid, tag_width
    tag_pid = PIDController(Kp=4, Ki=0, Kd=0.5, gkd=0.0, out_limit=1000.0)
    tag_control_output = tag_pid.calculate(160, mid)  # kp 0~10 kd
    if tag_control_output > 0:
        tag_control_output_final = tag_control_output + dead_area
    else:
        tag_control_output_final = tag_control_output - dead_area
    # print(control_output_final)
    if distance > 170:
        if mid >= 160 - tag_width and mid < 160 + tag_width:
            straight(500, 500)
        else:
            if tag_control_output_final > 1000:
                up.CDS_SetSpeed(1, 1000)
                up.CDS_SetSpeed(2, -1000)
            elif tag_control_output_final < -1000:
                up.CDS_SetSpeed(1, -1000)
                up.CDS_SetSpeed(2, 1000)
            else:
                up.CDS_SetSpeed(1, int(tag_control_output_final))
                up.CDS_SetSpeed(2, -int(tag_control_output_final))
    else:
        if io_data[0] == 1 and io_data[1] == 0 and not escape_flag_right:
            right(500)
        elif io_data[0] == 0 and io_data[1] == 1 and not escape_flag_left:
            left(500)
        else:
            straight_if()


def April_tag_escape():
    global escape_flag_right, escape_flag_left
    if distance < 170:
        if mid < 160:
            right(1000)
            escape_flag_left = 1
        else:
            left(1000)
            escape_flag_right = 1


def signal_handler(handler_signal, handler_frame):
    stop()
    exit(0)


def straight(speed_1, speed_2):
    up.CDS_SetSpeed(1, -speed_1)
    up.CDS_SetSpeed(2, -speed_2)


def straight_if():
    global buffer
    # if buffer > 0:
    #     straight(400, 400)
    #     buffer -= 1
    # else:
    # if unify_all > 3500:
    #     straight(900, 900)
    # elif unify_all > 2800:
    #     straight(700, 700)
    # else:
    if unify_all > down_value + 2000:
        straight(600, 600)
    else:
        straight(500, 500)


def stop():
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)


def back(speed):
    up.CDS_SetSpeed(1, speed)
    up.CDS_SetSpeed(2, speed)


def back_sleep():
    # stop()
    # time.sleep(0.1)
    # back(400)
    # time.sleep(0.05)
    # back(600)
    # time.sleep(0.05)
    stop()
    while_sleep(10)
    back(400)
    while_sleep(5)
    back(600)
    while_sleep(5)


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


def mix_all_gray():  # 低通滤波
    alpha = 0.7
    global adc_last_0
    global adc_last_1
    global adc_last_2
    global adc_last_3
    global adc_last_4
    global mix_adc_0
    global mix_adc_1
    global mix_adc_2
    global mix_adc_3
    global mix_adc_4
    mix_adc_0 = (1 - alpha) * adc_value[0] + alpha * adc_last_0
    mix_adc_1 = (1 - alpha) * adc_value[1] + alpha * adc_last_1
    mix_adc_2 = (1 - alpha) * adc_value[2] + alpha * adc_last_2
    mix_adc_3 = (1 - alpha) * adc_value[3] + alpha * adc_last_3
    mix_adc_4 = (1 - alpha) * adc_value[4] + alpha * adc_last_4
    adc_last_0 = mix_adc_0
    adc_last_1 = mix_adc_1
    adc_last_2 = mix_adc_2
    adc_last_3 = mix_adc_3
    adc_last_4 = mix_adc_4
    # print(f'{mix_adc_0:.2f},{mix_adc_1:.2f},{mix_adc_2:.2f},{mix_adc_3:.2f},{mix_adc_4:.2f}')
    # print(adc_value)


def unify_gray(value, min, max):  # 灰度线性校准
    value_unify = (value - min) * 1000 / (max - min)
    return value_unify


def unify_all_gray():
    global unify_adc_0
    global unify_adc_1
    global unify_adc_2
    global unify_adc_3
    global unify_adc_4
    global unify_all
    unify_adc_0 = (unify_gray(mix_adc_0, 437, 1011))
    unify_adc_1 = (unify_gray(mix_adc_1, 296, 732))
    unify_adc_2 = (unify_gray(mix_adc_2, 322, 680))
    unify_adc_3 = (unify_gray(mix_adc_3, 377, 840))
    unify_adc_4 = (unify_gray(mix_adc_4, 333, 780))
    unify_all = unify_adc_0 + unify_adc_1 + unify_adc_2 + unify_adc_3 + unify_adc_4
    # unify_all = adc_value[0]+adc_value[1]+adc_value[2]+adc_value[3]+adc_value[4]
    # print(unify_all)


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
    if unify_all < down_value:
        check_down_time += 1
    else:
        check_down_time = 0
        down = 0
    #
    if not down:
        if check_down_time >= down_time_value:
            down = 1
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
    global tai_flag, up_flag, buffer, down
    if tai_flag:
        up.CDS_SetAngle(3, 205, 700)  # 最高
        up.CDS_SetAngle(4, 600, 700)
        time.sleep(1)
        tai_flag = 0
    if up_flag:
        back(900)
        up.CDS_SetAngle(3, 400, 700)  #
        up.CDS_SetAngle(4, 380, 700)
        time.sleep(0.4)
        up.CDS_SetAngle(3, 620, 700)  # 最低
        up.CDS_SetAngle(4, 180, 700)
        time.sleep(0.8)
        stop()
        time.sleep(0.3)
        up_flag = 0
        tai_flag = 1
        down = 0
        buffer = 20
    else:
        up.CDS_SetAngle(3, 205, 700)  # 最高
        up.CDS_SetAngle(4, 600, 700)
        if io_data[0] == 0 and io_data[1] == 0:
            up_flag = 1
        else:
            right(700)


def up_act():
    global tai_flag
    up.CDS_SetAngle(3, 620, 700)  # 最低
    up.CDS_SetAngle(4, 180, 700)
    tai_flag = 1

    if io_data[3] == 0 and io_data[4] == 0:
        if tag_flag:
            if tag_safe:
                April_tag_move()
            else:
                April_tag_escape()
        elif blue_detected and not tag_lock_flag:
            color_blue_move()
        else:
            if io_data[0] == 0 and io_data[1] == 0:
                straight_if()
            elif io_data[0] == 1 and io_data[1] == 0 and not escape_flag_right:
                right(500)
            elif io_data[0] == 0 and io_data[1] == 1 and not escape_flag_left:
                left(500)
            else:
                search_left_and_right()
    elif io_data[3] == 1 and io_data[4] == 0:
        back_sleep()
        right(1000)
        while_sleep_break(20)
    elif io_data[3] == 0 and io_data[4] == 1:
        back_sleep()
        left(1000)
        while_sleep_break(20)
    else:
        back_sleep()


def search_left_and_right():
    global check_left_time, check_right_time, t, io_data, adc_value
    if check_right_time >= 3 and escape_flag_right == 0:
        while True:
            t += 1
            adc_value = up.ADC_Get_All_Channle()
            io_data = get_io_data(up)
            right(600)
            if io_data[0] == 0 and io_data[1] == 0 or t >= 300:
                t = 0
                check_right_time = 0
                break
    elif check_left_time >= 3 and escape_flag_left == 0:
        while True:
            t += 1
            adc_value = up.ADC_Get_All_Channle()
            io_data = get_io_data(up)
            left(600)
            if io_data[0] == 0 and io_data[1] == 0 or t >= 300:
                t = 0
                check_left_time = 0
                break
    else:
        straight_if()


def search_inf():
    global adc_value, io_data
    adc_value = up.ADC_Get_All_Channle()
    mix_all_gray()
    unify_all_gray()
    io_data = get_io_data(up)


def while_sleep(sleep_t):
    global cnt, adc_value, io_data
    while sleep_t >= 0:
        cnt += 1
        if cnt % 5000 == 0:  # 0.01秒钟打印一次
            cnt = 0
            adc_value = up.ADC_Get_All_Channle()
            mix_all_gray()
            unify_all_gray()
            io_data = get_io_data(up)
            check_time()
            sleep_t -= 1
        # if io_data[0] == 0 or io_data[1] == 0 or io_data[6] == 0 or io_data[7] == 0:
        #     break


def while_sleep_break(sleep_t):
    global cnt, adc_value, io_data
    while sleep_t >= 0:
        cnt += 1
        if cnt % 5000 == 0:  # 0.01秒钟打印一次
            cnt = 0
            adc_value = up.ADC_Get_All_Channle()
            mix_all_gray()
            unify_all_gray()
            io_data = get_io_data(up)
            check_time()
            sleep_t -= 1
        if blue_detected and 150 < cx < 170:
            break


if __name__ == "__main__":
    up = uptech.UpTech()
    up.LCD_Open(2)
    up.ADC_IO_Open()
    up.CDS_Open()
    up.ADC_Led_SetColor(0, 0x2F0000)
    up.ADC_Led_SetColor(1, 0x002F00)
    # up.ADC_Led_SetColor(0, 0xE67E223)  # FF0000(纯红)FF5733(橙红)C70039(深红)000000(纯黑)FFFF00(纯黄)
    # up.ADC_Led_SetColor(1, 0x800080)  # 0000FF(纯蓝)3498DB(天蓝)2C3E50(深蓝)E67E22(萝卜橙)8B4513(马棕)
    # # F9E79F(纯白)00FF00(纯绿色)800080(纯紫)E67E223(青色)
    up.CDS_SetMode(1, 1)
    up.CDS_SetMode(2, 1)
    up.CDS_SetMode(3, 0)
    up.CDS_SetMode(4, 0)
    # FONT_8X14   = 8
    # FONT_10X16  = 9
    # FONT_12X16  = 10
    # FONT_12X20  = 11
    print("test succeed")
    signal.signal(signal.SIGINT, signal_handler)
    target2 = threading.Thread(target=April_start_detect)
    target2.start()
    # target3 = threading.Thread(target=search_inf)
    # target3.start()
    print("Ready————")
    # while True:
    #     io_data = get_io_data(up)
    #     if io_data[6] == 0 and io_data[7] == 0:
    #         break
    print("Go!!!")
    # go_around(1000)
    while True:
        adc_value = up.ADC_Get_All_Channle()
        mix_all_gray()
        unify_all_gray()
        io_data = get_io_data(up)
        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_GBLUE)
        # up.LCD_PutString(0, 0, 'Go North All')

        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_YELLOW)
        up.LCD_PutString(0, 0, f'{unify_all:.2f}')
        # up.LCD_PutString(0, 20, f'{unify_adc_1:.2f}')
        # up.LCD_PutString(0, 40, f'{unify_adc_2:.2f}')
        # print(unify_all)
        up.LCD_Refresh()
        # print(f'unify_adc{unify_all}')
        # print(f'unify_adc{unify_adc_0, unify_adc_1, unify_adc_2, unify_adc_3, unify_adc_4}')

        # up.CDS_SetAngle(3, 620, 700)  # 最低
        # up.CDS_SetAngle(4, 180, 700)
        # up.CDS_SetAngle(3, 205, 700)  # 最高
        # up.CDS_SetAngle(4, 600, 700)
        # print(mix_adc_0)
        # 0、1 正前方红外   3、4斜向下   6、7左右
        print(adc_value)
        # print(io_data)
        # print(escape_time)
        # print(camera_safe)
        if camera_safe:
            check_time()
            if down:
                down_act()
            else:
                up_act()
        else:
            # print("stop")
            stop()
