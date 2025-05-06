import uptech
import time
flag=0
cnt=0
if __name__ == "__main__":
    up = uptech.UpTech()
    up.LCD_Open(2)
    up.ADC_IO_Open()
    up.CDS_Open()
    up.ADC_Led_SetColor(0, 0xE67E223)#FF0000(纯红)FF5733(橙红)C70039(深红)000000(纯黑)FFFF00(纯黄)
    up.ADC_Led_SetColor(1, 0x800080)#0000FF(纯蓝)3498DB(天蓝)2C3E50(深蓝)E67E22(萝卜橙)8B4513(马棕)
                                               #F9E79F(纯白)00FF00(纯绿色)800080(纯紫)E67E223(青色)

    up.CDS_SetMode(1, 1)
    up.CDS_SetMode(2, 1)
    up.CDS_SetMode(3, 0)#最低 700 最高 400 平放 550
    up.CDS_SetMode(4, 0)#最低 200 最高 500 平放 350
    # FONT_8X14   = 8
    # FONT_10X16  = 9
    # FONT_12X16  = 10
    # FONT_12X20  = 11
    print("test succeed")
    while True:
        adc_value = up.ADC_Get_All_Channle()
        result = "[{}]".format(",".join(map(str, adc_value)))

        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_GBLUE)
        up.LCD_PutString(0, 0, 'Go North All')
        up.LCD_SetFont(up.FONT_12X20)
        up.LCD_SetForeColor(up.COLOR_YELLOW)
        up.LCD_PutString(0, 20, f'{result}')
        up.LCD_Refresh()

        up.CDS_SetSpeed(1,650)
        up.CDS_SetSpeed(2,700)
        if flag == 0:
            up.CDS_SetAngle(3,600,500)
            up.CDS_SetAngle(4,300,500)
        else:
            up.CDS_SetAngle(3,700,500)#最低
            up.CDS_SetAngle(4,200,500)#最低
        # up.CDS_SetAngle(3,700,500)#最低
        # up.CDS_SetAngle(4,200,500)#最低
        # up.CDS_SetAngle(3,400,500)#最高
        # up.CDS_SetAngle(4,500,500)#最高

        cnt+=1
        print(f'adc{result}')





