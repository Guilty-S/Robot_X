import uptech
import time
if __name__ == "__main__":
    up = uptech.UpTech()
    up.LCD_Open(2)
    up.ADC_IO_Open()
    up.CDS_Open()
    up.ADC_Led_SetColor(0, 0x8B4513) #FF0000（纯红） FF5733（橙红） C70039（深红） 000000（纯黑）
    up.ADC_Led_SetColor(1, 0x800080) #0000FF（纯蓝） 3498DB（天蓝） 2C3E50（深蓝） E67E22（萝卜橙）
    # FFFF00（纯黄）  F9E79F（纯白） 00FF00 纯绿色  800080（纯紫） 8B4513（马棕）
    up.CDS_SetMode(1, 1)
    up.CDS_SetMode(2, 1)
    up.LCD_SetFont(up.FONT_12X20)
    print("test succeed")
    while True:
        up.CDS_SetSpeed(1,-500)#2025/4/27
        up.CDS_SetSpeed(2,-500)#
        for i in range(0,100):
            up.LCD_PutString(0, 0, 'Go North All')
            up.LCD_PutString(0, 20, f'ADC {i}')
            up.LCD_PutString(0, 40, f'IO {i/10}')
            up.LCD_Refresh()


