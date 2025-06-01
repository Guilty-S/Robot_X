from xml.sax.saxutils import escape

import cv2
import subprocess
import uptech
import time
import apriltag
import numpy as np
import signal
import threading

tag_safe = 0
tag_flag = 0
mid = 0
tag_width = 0
tags = []
distance = 0
di_fang_kuai = 1  # 敌方块
zhong_li_kuai = 0  # 中立块
zha_dan_kuai = 2  # 炸弹块
index = 0
flag = 0
cnt = 0


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

    def update_frame_new(self, frame):  # 中立优先
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
                    if tags[i].tag_id == di_fang_kuai:
                        if tags[index].tag_id == di_fang_kuai:
                            if (self.get_distance(tags[index].homography, 4300) >
                                    self.get_distance(tags[i].homography, 4300)):  # 冒泡排序
                                index = i
                    elif tags[i].tag_id == zhong_li_kuai:
                        if tags[index].tag_id == zhong_li_kuai:
                            if (self.get_distance(tags[index].homography, 4300) >
                                    self.get_distance(tags[i].homography, 4300)):  # 冒泡排序
                                index = i
                        elif tags[index].tag_id == di_fang_kuai:
                            index = i
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


def get_io_data(up):
    io_all_input = up.ADC_IO_GetAllInputLevel()
    io_array = '{:08b}'.format(io_all_input)
    io_data = []
    for index, value in enumerate(io_array):
        io = int(value)
        io_data.insert(0, io)
    return io_data


def April_start_detect():
    global frame
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(3, 320)
    cap.set(4, 240)
    cap.set(cv2.CAP_PROP_FPS, 60)
    ad = ApriltagDetect()
    while True:
        ret, frame = cap.read()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        ad.update_frame_new(frame)
        if ret is False:
            cap.release()
            time.sleep(0.1)
            print("reconnect to camera")
            subprocess.check_call("sudo modprobe -rf uvcvideo", shell=True)
            time.sleep(0.4)
            subprocess.check_call("sudo modprobe uvcvideo", shell=True)
            time.sleep(0.2)
            cap = cv2.VideoCapture('/dev/video0')

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
    if distance > 200:
        if mid < 160 - tag_width:
            left_low_low()
            # print("左")
        elif mid > 160 + tag_width:
            right_low_low()
            # print("右")
        else:
            straight_if()
            # print("前进")


def April_tag_move_pid():
    pid = PIDController(Kp=5.0, Ki=0, Kd=0.5, gkd=0.0, out_limit=1000.0)
    control_output = pid.calculate(160, mid)  # kp 0~10 kd
    if mid < 160 - tag_width:
        up.CDS_SetSpeed(1, control_output)
        up.CDS_SetSpeed(2, -control_output)
    elif mid > 160 + tag_width:
        up.CDS_SetSpeed(1, control_output)
        up.CDS_SetSpeed(2, -control_output)
    else:
        straight_if()


def April_tag_escape():
    if distance < 200:
        if mid < 160:
            right()
            time.sleep(0.2)
        else:
            left()
            time.sleep(0.2)


def signal_handler(handler_signal, handler_frame):
    stop()
    exit(0)


def straight_low():
    up.CDS_SetSpeed(1, 500)
    up.CDS_SetSpeed(2, 500)


def straight():
    up.CDS_SetSpeed(1, 550)
    up.CDS_SetSpeed(2, 600)


def straight_fast():
    up.CDS_SetSpeed(1, 700)
    up.CDS_SetSpeed(2, 700)


def straight_if():
    if adc_value[0] > 660 or adc_value[1] > 460:
        straight_fast()
    else:
        straight_low()
    # if adc_value[0] > 480 and adc_value[1] > 460 or adc_value[0] > 360 and adc_value[1] > 660:
    #     straight_fast()
    # else:
    #     straight_low()


def back():
    up.CDS_SetSpeed(1, -1000)
    up.CDS_SetSpeed(2, -1000)


def back_low():
    up.CDS_SetSpeed(1, -500)
    up.CDS_SetSpeed(2, -500)


def back_up():
    up.CDS_SetSpeed(1, -600)
    up.CDS_SetSpeed(2, -600)


def stop():
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)


def left():
    up.CDS_SetSpeed(1, 1000)
    up.CDS_SetSpeed(2, -1000)


def left_x():
    up.CDS_SetSpeed(1, 600)
    up.CDS_SetSpeed(2, -900)


def left_low():
    up.CDS_SetSpeed(1, 600)
    up.CDS_SetSpeed(2, -600)


def left_low_low():
    up.CDS_SetSpeed(1, 400)
    up.CDS_SetSpeed(2, -400)


def right():
    up.CDS_SetSpeed(1, -1000)
    up.CDS_SetSpeed(2, 1000)


def right_x():
    up.CDS_SetSpeed(1, -900)
    up.CDS_SetSpeed(2, 600)


def right_low():
    up.CDS_SetSpeed(1, -600)
    up.CDS_SetSpeed(2, 600)


def right_low_low():
    up.CDS_SetSpeed(1, -400)
    up.CDS_SetSpeed(2, 400)


if __name__ == "__main__":
    up = uptech.UpTech()
    up.LCD_Open(2)
    up.ADC_IO_Open()
    up.CDS_Open()
    up.ADC_Led_SetColor(0, 0xE67E223)  # FF0000(纯红)FF5733(橙红)C70039(深红)000000(纯黑)FFFF00(纯黄)
    up.ADC_Led_SetColor(1, 0x800080)  # 0000FF(纯蓝)3498DB(天蓝)2C3E50(深蓝)E67E22(萝卜橙)8B4513(马棕)
    # F9E79F(纯白)00FF00(纯绿色)800080(纯紫)E67E223(青色)
    io_data = []
    up.CDS_SetMode(1, 1)
    up.CDS_SetMode(2, 1)
    up.CDS_SetMode(3, 0)  # 最低 600 平放 420  最高 140
    up.CDS_SetMode(4, 0)  # 最低 140 平放 320  最高 600
    # FONT_8X14   = 8
    # FONT_10X16  = 9
    # FONT_12X16  = 10
    # FONT_12X20  = 11
    print("test succeed")
    signal.signal(signal.SIGINT, signal_handler)
    target2 = threading.Thread(target=April_start_detect)
    target2.start()
    # while True:
    #     io_data = get_io_data(up)
    #     if io_data[6] == 0 and io_data[7] == 0:
    #         break
    tai_flag = 1
    tai_flag_time = 0
    escape_flag = 0
    escape_time = 20
    down = 0
    up_flag = 0
    t = 0
    # 初始化PID控制器
    # 在控制循环中计算输出
    while True:
        adc_value = up.ADC_Get_All_Channle()
        io_data = get_io_data(up)
        # result = "[{}]".format(",".join(map(str, adc_value)))
        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_GBLUE)
        up.LCD_PutString(0, 0, 'Go North All')

        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_YELLOW)
        # up.LCD_PutString(0, 20, f'{adc_value[0] + adc_value[1] + adc_value[2]}')

        up.LCD_Refresh()

        if adc_value[0] + adc_value[1] + adc_value[2] < 845:  # 185,222 #315,306
            down = 1
        else:
            down = 0
        # print(flag)
        # up.CDS_SetAngle(3, 630, 500)  # 最低
        # up.CDS_SetAngle(4, 130, 500)
        # up.CDS_SetAngle(3, 200, 700)#最高
        # up.CDS_SetAngle(4, 540, 700)

        # print(tag_flag)
        # print(io_data)
        # 0、1 正前方红外   3、4斜向下   6、7左右
        # if adc_value_min<adc_value[0]+adc_value[1]+adc_value[2]:#846
        # #     adc_value_min=adc_value[0]+adc_value[1]+adc_value[2]
        # print(up_flag)
        if escape_flag:
            escape_time -= 1
            if escape_time <= 0:
                escape_time = 20
                escape_flag = 0
        if down:
            if tai_flag:
                up.CDS_SetAngle(3, 200, 700)  # 最高
                up.CDS_SetAngle(4, 540, 700)
                time.sleep(2)
                tai_flag = 0
            else:
                tai_flag_time += 1
                if tai_flag_time >= 30:
                    tai_flag_time = 0
                    tai_flag = 1
            if up_flag:
                back_low()
                time.sleep(0.3)
                up.CDS_SetAngle(3, 630, 500)  # 最低
                up.CDS_SetAngle(4, 130, 500)
                time.sleep(1.5)
                up_flag = 0
            else:
                up.CDS_SetAngle(3, 200, 700)  # 最高
                up.CDS_SetAngle(4, 540, 700)
                if io_data[0] == 0 and io_data[1] == 0:
                    up_flag = 1
                else:
                    right_low()
        else:
            up.CDS_SetAngle(3, 630, 500)  # 最低
            up.CDS_SetAngle(4, 130, 500)
            tai_flag = 1
            if io_data[3] == 0 and io_data[4] == 0:
                if tag_flag:
                    if tag_safe:
                        April_tag_move()
                    else:
                        April_tag_escape()
                        escape_flag = 1
                else:
                    if io_data[0] == 0 and io_data[1] == 0:
                        straight_if()
                    elif io_data[0] == 1 and io_data[1] == 0:
                        right_low()
                    elif io_data[0] == 0 and io_data[1] == 1:
                        left_low()
                    else:
                        if io_data[6] == 1 and io_data[7] == 0 and escape_flag == 0:
                            while True:
                                t += 1
                                io_data = get_io_data(up)
                                right_low()
                                if io_data[0] == 0 and io_data[1] == 0 or t >= 800:
                                    t = 0
                                    break
                        elif io_data[6] == 0 and io_data[7] == 1 and escape_flag == 0:
                            while True:
                                t += 1
                                io_data = get_io_data(up)
                                left_low()
                                if io_data[0] == 0 and io_data[1] == 0 or t >= 800:
                                    t = 0
                                    break
                        elif io_data[6] == 0 and io_data[7] == 0 and escape_flag == 0:
                            while True:
                                t += 1
                                io_data = get_io_data(up)
                                right_low()
                                if io_data[0] == 0 and io_data[1] == 0 or t >= 800:
                                    t = 0
                                    break
                        else:
                            straight_if()
            elif io_data[3] == 1 and io_data[4] == 0:
                back_low()
                time.sleep(0.02)
                right_x()
            elif io_data[3] == 0 and io_data[4] == 1:
                back_low()
                time.sleep(0.02)
                left_x()
            else:
                back_low()
                time.sleep(0.1)
        # print(f'adc{adc_value}')
