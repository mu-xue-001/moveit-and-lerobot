import cv2
import numpy as np

# 打开摄像头2（/dev/video2）
cap = cv2.VideoCapture(2)

if not cap.isOpened():
    print("无法打开摄像头2")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("读取失败")
        break

    # 转换到 HSV 色彩空间（比RGB更适合颜色识别）
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 红色有两个范围（HSV是环状的）
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # 生成红色mask
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # 去噪
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

    # 找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)

        # 过滤小噪声
        if area > 500:
            x, y, w, h = cv2.boundingRect(cnt)

            # 画框
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "Red Object", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 显示结果
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    # 按 q 退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()