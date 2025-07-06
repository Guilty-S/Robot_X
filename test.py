import cv2
import numpy as np

# 创建窗口
cv2.namedWindow('HSV Range Selector')
cv2.namedWindow('Mask Preview')
cv2.namedWindow('Original')

h_min, s_min, v_min = 0, 0, 0
h_max, s_max, v_max = 179, 34, 198
# h_max, s_max, v_max = 179, 28, 96

# 创建滑块
cv2.createTrackbar('H Min', 'HSV Range Selector', h_min, 179, lambda x: None)
cv2.createTrackbar('S Min', 'HSV Range Selector', s_min, 255, lambda x: None)
cv2.createTrackbar('V Min', 'HSV Range Selector', v_min, 255, lambda x: None)
cv2.createTrackbar('H Max', 'HSV Range Selector', h_max, 179, lambda x: None)
cv2.createTrackbar('S Max', 'HSV Range Selector', s_max, 255, lambda x: None)
cv2.createTrackbar('V Max', 'HSV Range Selector', v_max, 255, lambda x: None)

# 初始化摄像头
cap = cv2.VideoCapture(0)

print("使用滑块调整HSV范围，直到目标颜色在掩膜中显示为白色")
print("按 's' 键保存当前范围并退出")
print("按 'q' 键退出而不保存")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 获取滑块值
    h_min = cv2.getTrackbarPos('H Min', 'HSV Range Selector')
    s_min = cv2.getTrackbarPos('S Min', 'HSV Range Selector')
    v_min = cv2.getTrackbarPos('V Min', 'HSV Range Selector')
    h_max = cv2.getTrackbarPos('H Max', 'HSV Range Selector')
    s_max = cv2.getTrackbarPos('S Max', 'HSV Range Selector')
    v_max = cv2.getTrackbarPos('V Max', 'HSV Range Selector')

    # 确保最小值小于最大值
    if h_min > h_max:
        h_max, h_min = h_min, h_max
        cv2.setTrackbarPos('H Min', 'HSV Range Selector', h_min)
        cv2.setTrackbarPos('H Max', 'HSV Range Selector', h_max)

    # 转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 创建掩膜
    lower_bound = np.array([h_min, s_min, v_min])
    upper_bound = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # 在原始图像上显示范围信息
    info_frame = frame.copy()
    cv2.putText(info_frame, f"Lower: [{h_min}, {s_min}, {v_min}]", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(info_frame, f"Upper: [{h_max}, {s_max}, {v_max}]", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # 显示图像
    cv2.imshow('Original', info_frame)
    cv2.imshow('Mask Preview', mask)

    # 按键处理
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):  # 保存设置并退出
        print("\n保存的HSV范围:")
        print(f"lower_bound = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"upper_bound = np.array([{h_max}, {s_max}, {v_max}])")
        break
    elif key == ord('q'):  # 退出而不保存
        print("\n退出而不保存")
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()