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


class ApriltagDetect:
    def __init__(self):
        self.target_id = 0
        self.at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9'))

    def update_frame(self, frame):
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
        ad.update_frame(frame)
        if ret is False:
            cap.release()
            time.sleep(0.1)
            print("reconnect to camera")
            subprocess.check_call("sudo modprobe -rf uvcvideo", shell=True)
            time.sleep(0.4)
            subprocess.check_call("sudo modprobe uvcvideo", shell=True)
            time.sleep(0.2)
            cap = cv2.VideoCapture('/dev/video0')

        if tags:
            # print(tags)
            # print(index)
            print(f"中心位置{mid}")
            print(f"距离{distance}")
            print(f"宽度{tag_width}")
            if tag_safe == 0:
                print("炸弹")
            else:
                if tags[index].tag_id == 1:
                    print("敌方")
                elif tags[index].tag_id == 0:
                    print("中立")
        cv2.imshow("img", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


def April_tag_move():
    if mid < 160 - tag_width / 2:
        left_low_low()
        print("左")
    elif mid > 160 + tag_width / 2 :
        right_low_low()
        print("右")
    else:
        straight()
        print("前进")


def April_tag_escape():
    if distance < 50:
        back()
        time.sleep(0.2)
        left()
    elif distance < 120 and distance >= 60:
        if mid < 160:
            right()
        else:
            left()


def signal_handler(handler_signal, handler_frame):
    stop()
    exit(0)


def straight():
    up.CDS_SetSpeed(1, 500)
    up.CDS_SetSpeed(2, 500)


def back():
    up.CDS_SetSpeed(1, -1000)
    up.CDS_SetSpeed(2, -1000)


def back_low():
    up.CDS_SetSpeed(1, -500)
    up.CDS_SetSpeed(2, -500)


def stop():
    up.CDS_SetSpeed(1, 0)
    up.CDS_SetSpeed(2, 0)


def left():
    up.CDS_SetSpeed(1, -1000)
    up.CDS_SetSpeed(2, 1000)


def left_low():
    up.CDS_SetSpeed(1, -700)
    up.CDS_SetSpeed(2, 700)


def left_low_low():
    up.CDS_SetSpeed(1, -400)
    up.CDS_SetSpeed(2, 400)


def right():
    up.CDS_SetSpeed(1, 1000)
    up.CDS_SetSpeed(2, -1000)


def right_low():
    up.CDS_SetSpeed(1, 700)
    up.CDS_SetSpeed(2, -700)


def right_low_low():
    up.CDS_SetSpeed(1, 400)
    up.CDS_SetSpeed(2, -400)


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
    up.CDS_SetMode(3, 0)  # 最低 700 最高 400 平放 550
    up.CDS_SetMode(4, 0)  # 最低 200 最高 500 平放 350
    # FONT_8X14   = 8
    # FONT_10X16  = 9
    # FONT_12X16  = 10
    # FONT_12X20  = 11
    print("test succeed")
    signal.signal(signal.SIGINT, signal_handler)
    target2 = threading.Thread(target=April_start_detect)
    target2.start()
    # while True:
    #     adc_value = up.ADC_Get_All_Channle()
    #     io_data = get_io_data(up)
    #     if io_data[6] == 0 and io_data[7] == 0:
    #         break
    while True:
        adc_value = up.ADC_Get_All_Channle()
        io_data = get_io_data(up)
        # result = "[{}]".format(",".join(map(str, adc_value)))
        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_GBLUE)
        up.LCD_PutString(0, 0, 'Go North All')

        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_YELLOW)
        up.LCD_PutString(0, 20, f'{adc_value}')

        up.LCD_Refresh()
        while adc_value[0] < 220 and adc_value[1] < 250:  # 185,222 #315,306
            adc_value = up.ADC_Get_All_Channle()
        #     # back()
            stop()
            # up.CDS_SetAngle(3, 600, 500)
            # up.CDS_SetAngle(4, 300, 500)

        up.CDS_SetAngle(3, 700, 500)  # 最低
        up.CDS_SetAngle(4, 200, 500)  # 最低

        # 0、1 正前方红外   3、4斜向下   6、7左右
        if io_data[3] == 0 and io_data[4] == 0:
            if tag_flag:
                if tag_safe:
                    April_tag_move()
                else:
                    April_tag_escape()
            else:
                if io_data[0] == 0 and io_data[1] == 0:
                    straight()
                elif io_data[0] == 1 and io_data[1] == 0:
                    right_low_low()
                elif io_data[0] == 0 and io_data[1] == 1:
                    left_low_low()
                else:
                    if io_data[6] == 1 and io_data[7] == 0:
                        while not tag_flag == 0 or io_data[0] == 0 and io_data[1] == 0:
                            right()
                            io_data = get_io_data(up)
                    elif io_data[6] == 0 and io_data[7] == 1:
                        while not tag_flag == 0 or io_data[0] == 0 and io_data[1] == 0:
                            left()
                            io_data = get_io_data(up)
                    elif io_data[6] == 0 and io_data[7] == 0:
                        while not tag_flag == 0 or io_data[0] == 0 and io_data[1] == 0:
                            left()
                            io_data = get_io_data(up)
                    else:
                        straight()
        elif io_data[3] == 1 and io_data[4] == 0:
            right_low()
        elif io_data[3] == 0 and io_data[4] == 1:
            left_low()
        else:
            back()
            time.sleep(0.1)
            right()
        # print(f'adc{adc_value}')

        # if IO_3 == 0 and IO_4 == 0:
        #     if IO_0 == 0 and IO_1 == 1:
        #         left_low()
        #     elif IO_0 == 1 and IO_1 == 0:
        #         right_low()
        #     elif IO_0 == 1 and IO_1 == 1:
        #         if IO_6 == 0 and IO_7 == 1:
        #             while not (io_data[0] == 0 and io_data[1] == 0):
        #                 left_low()
        #                 io_data = get_io_data(up)
        #         elif IO_6 == 1 and IO_7 == 0:
        #             while not (io_data[0] == 0 and io_data[1] == 0):
        #                 right_low()
        #                 io_data = get_io_data(up)
        #         elif IO_6 == 0 and IO_7 == 0:
        #             while not (io_data[0] == 0 and io_data[1] == 0):
        #                 left_low()
        #                 io_data = get_io_data(up)
        #         else:
        #             straight()
        #     else:
        #         straight()
        # elif IO_3 == 1 and IO_4 == 0:
        #     right_low()
        # elif IO_3 == 0 and IO_4 == 1:
        #     left_low()
        # else:
        #     back_low()
