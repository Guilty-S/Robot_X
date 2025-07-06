import cv2
import numpy as np


def detect_camera_blockage(frame, block_size=15, c_value=0, ratio_thresh=0.7):
    """
    检测摄像头是否被遮挡

    参数:
        frame: 输入图像 (BGR格式)
        block_size: 邻域大小 (奇数，建议15-25)
        c_value: 阈值偏移量 (-10到10)
        ratio_thresh: 黑色像素比例阈值 (0-1)

    返回:
        black_flag: 1表示遮挡, 0表示正常
        black_ratio: 检测到的黑色像素比例
        display_frame: 带检测结果的可视化图像
    """
    # 确保block_size是奇数且>=3
    block_size = max(3, block_size)
    if block_size % 2 == 0:
        block_size += 1

    # 转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 应用自适应阈值
    thresh = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        block_size,
        c_value
    )

    # 计算黑色像素比例
    total_pixels = frame.shape[0] * frame.shape[1]
    black_pixels = np.count_nonzero(thresh)
    black_ratio = black_pixels / total_pixels

    # 设置标志
    black_flag = 1 if black_ratio > ratio_thresh else 0

    # 创建可视化结果
    display_frame = frame.copy()

    # 显示参数和结果
    param_text = f"Block: {block_size}, C: {c_value}, RatioThresh: {ratio_thresh:.2f}"
    cv2.putText(display_frame, param_text, (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    ratio_text = f"Black: {black_ratio:.2%} ({black_pixels}/{total_pixels})"
    cv2.putText(display_frame, ratio_text, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    flag_text = f"Blocked: {'YES' if black_flag else 'NO'}"
    flag_color = (0, 0, 255) if black_flag else (0, 255, 0)
    cv2.putText(display_frame, flag_text, (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, flag_color, 2)

    # 显示阈值图像（缩小尺寸）
    thresh_rgb = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    thresh_resized = cv2.resize(thresh_rgb, (320, 240))
    display_frame[display_frame.shape[0] - 240:, :320] = thresh_resized

    # 添加遮挡警告
    if black_flag:
        cv2.putText(display_frame, "CAMERA BLOCKED!",
                    (display_frame.shape[1] // 2 - 100, display_frame.shape[0] // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return black_flag, black_ratio, display_frame


def adjust_blockage_detection():
    """
    实时调整摄像头遮挡检测参数的工具 (统一c_value范围为-10到10)
    """
    # 创建参数调整窗口
    cv2.namedWindow("Blockage Detection")
    cv2.namedWindow("Parameters")

    # 初始化参数 (c_value偏移量，范围-10到10)
    block_size = 15
    c_value = 0  # 中心值0
    ratio_thresh = 70  # 百分比表示

    # 创建滑动条 (使用统一范围)
    cv2.createTrackbar('Block Size', 'Parameters', block_size, 50, lambda x: None)
    cv2.createTrackbar('C Value [-10,10]', 'Parameters', 10, 20, lambda x: None)  # 0-20映射到-10到10
    cv2.createTrackbar('Ratio Thresh %', 'Parameters', ratio_thresh, 100, lambda x: None)

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("使用说明:")
    print("1. 调整滑动条优化检测参数")
    print("2. Block Size: 邻域大小 (奇数)")
    print("3. C Value: 阈值偏移量 (-10到10)")
    print("4. Ratio Thresh: 遮挡比例阈值 (0-100%)")
    print("5. 按 's' 保存当前参数")
    print("6. 按 'q' 退出")

    while True:
        # 获取当前参数值
        block_size = max(3, cv2.getTrackbarPos('Block Size', 'Parameters'))

        # 统一c_value范围：滑动条0-20映射到-10到10
        c_value_slider = cv2.getTrackbarPos('C Value [-10,10]', 'Parameters')
        c_value = c_value_slider - 10

        ratio_thresh = cv2.getTrackbarPos('Ratio Thresh %', 'Parameters') / 100.0

        # 确保block_size是奇数
        if block_size % 2 == 0:
            block_size += 1

        # 读取摄像头帧
        ret, frame = cap.read()
        if not ret:
            print("无法获取帧")
            break

        # 检测遮挡
        blocked, ratio, display_frame = detect_camera_blockage(
            frame,
            block_size=block_size,
            c_value=c_value,
            ratio_thresh=ratio_thresh
        )

        # 显示结果
        cv2.imshow("Blockage Detection", display_frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('s'):
            print("\n保存的参数:")
            print(f"block_size = {block_size}")
            print(f"c_value = {c_value}")  # 显示实际使用的c_value
            print(f"ratio_thresh = {ratio_thresh:.2f}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    adjust_blockage_detection()