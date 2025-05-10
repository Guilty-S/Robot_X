import uptech
import time

flag = 0
cnt = 0


def get_io_data(up):
    io_all_input = up.ADC_IO_GetAllInputLevel()
    io_array = '{:08b}'.format(io_all_input)
    io_data = []
    for index, value in enumerate(io_array):
        io = int(value)
        io_data.insert(0, io)
    return io_data


def straight():
    up.CDS_SetSpeed(1, 500)
    up.CDS_SetSpeed(2, 500)


def back():
    up.CDS_SetSpeed(1, -700)
    up.CDS_SetSpeed(2, -1000)


def back_low():
    up.CDS_SetSpeed(1, -500)
    up.CDS_SetSpeed(2, -500)


def left():
    up.CDS_SetSpeed(1, -1000)
    up.CDS_SetSpeed(2, 1000)


def right():
    up.CDS_SetSpeed(1, 1000)
    up.CDS_SetSpeed(2, -1000)


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
    while True:
        adc_value = up.ADC_Get_All_Channle()
        io_data = get_io_data(up)
        if io_data[6] == 0 and io_data[7] == 0:
            break
    while True:
        adc_value = up.ADC_Get_All_Channle()
        io_data = get_io_data(up)
        IO_0 = io_data[0]
        IO_1 = io_data[1]
        IO_3 = io_data[3]
        IO_4 = io_data[4]
        IO_6 = io_data[6]
        IO_7 = io_data[7]
        # result = "[{}]".format(",".join(map(str, adc_value)))

        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_GBLUE)
        up.LCD_PutString(0, 0, 'Go North All')

        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_YELLOW)
        up.LCD_PutString(0, 20, f'{adc_value}')

        up.LCD_Refresh()
        while adc_value[0] < 270 and adc_value[1] < 270:  # 185,222 #315,306
            adc_value = up.ADC_Get_All_Channle()
            back()
            up.CDS_SetAngle(3, 600, 500)
            up.CDS_SetAngle(4, 300, 500)

        up.CDS_SetAngle(3, 700, 500)  # 最低
        up.CDS_SetAngle(4, 200, 500)  # 最低

        if IO_3 == 0 and IO_4 == 0:
            if IO_0 == 0 and IO_1 == 1:
                left()
            elif IO_0 == 1 and IO_1 == 0:
                right()
            elif IO_0 == 1 and IO_1 == 1:
                if IO_6 == 0 and IO_7 == 1:
                    while not (io_data[0] == 0 and io_data[1] == 0):
                        left()
                        io_data = get_io_data(up)
                elif IO_6 == 1 and IO_7 == 0:
                    while not (io_data[0] == 0 and io_data[1] == 0):
                        right()
                        io_data = get_io_data(up)
                elif IO_6 == 0 and IO_7 == 0:
                    while not (io_data[0] == 0 and io_data[1] == 0):
                        right()
                        io_data = get_io_data(up)
                else:
                    straight()
            else:
                straight()
        elif IO_3 == 1 and IO_4 == 0:
            right()
        elif IO_3 == 0 and IO_4 == 1:
            left()
        else:
            back_low()

        cnt += 1
        print(f'adc{adc_value}')
