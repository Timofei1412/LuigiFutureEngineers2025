import socket
import time
import keyboard
import cv2
import cv2.aruco as aruco
import numpy as np

esp_start = False
cam = True

if esp_start:
    print("подключение esp")
    ESP32_IP = "192.168.0.100"  # Замените на реальный IP
    PORT = 1234
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ESP32_IP, PORT))

flag = True
flag1 = True
flag2 = True

colors = (0, 255, 0)

data_str = ""

def key_connect():
    global mot_left
    global mot_right
    global baraban
    global sost
    global flag
    global flag1
    global flag2
    global speed_baraban
    speed = 70
    if keyboard.is_pressed('w') and not (keyboard.is_pressed('a') or keyboard.is_pressed('d')): # прямо
        mot_left = speed
        mot_right = speed
    elif keyboard.is_pressed('a') and not keyboard.is_pressed('w'): # танк влево
        mot_left = -speed
        mot_right = speed
    elif keyboard.is_pressed('d') and not keyboard.is_pressed('w'): # танк вправо
        mot_left = speed
        mot_right = -speed
    elif keyboard.is_pressed('a') and keyboard.is_pressed('w'):  # одной влево
        mot_left = 0
        mot_right = speed
    elif keyboard.is_pressed('d') and keyboard.is_pressed('w'):  # одной вправо
        mot_left = speed
        mot_right = 0
    elif keyboard.is_pressed('s'):
        mot_left = -speed
        mot_right = -speed
    else:
        mot_left = 0
        mot_right = 0
    if keyboard.is_pressed('u'):
        sost = max_otklon_serva

    if keyboard.is_pressed('b') and flag:
        flag = False
    if not keyboard.is_pressed('b') and not flag:
        flag = True
        if baraban:
            baraban = 0
        else:
            baraban = speed_baraban

    if keyboard.is_pressed('y') and flag1:
        flag1 = False
    if not keyboard.is_pressed('y') and not flag1:
        flag1 = True
        sost += 1

    if keyboard.is_pressed('h') and flag2:
        flag2 = False
    if not keyboard.is_pressed('h') and not flag2:
        flag2 = True
        sost -= 1

    if sost > max_otklon_serva:
        sost = max_otklon_serva
    elif sost < -max_otklon_serva:
        sost = -max_otklon_serva

    if keyboard.is_pressed('1'):
        speed_baraban = 25 * 1
    if keyboard.is_pressed('2'):
        speed_baraban = 25 * 2
    if keyboard.is_pressed('3'):
        speed_baraban = 25 * 3
    if keyboard.is_pressed('4'):
        speed_baraban = 25 * 4
    if keyboard.is_pressed('5'):
        speed_baraban = 25 * 5
    if keyboard.is_pressed('6'):
        speed_baraban = 25 * 6
    if keyboard.is_pressed('7'):
        speed_baraban = 25 * 7
    if keyboard.is_pressed('8'):
        speed_baraban = 25 * 8
    if keyboard.is_pressed('9'):
        speed_baraban = 25 * 9
    if keyboard.is_pressed('0'):
        speed_baraban = 255
    print("клавиатура ")

def send():
    global data_str
    global mot_left
    global mot_right
    global baraban
    global sost
    data_str = str(mot_left) + ' ' + str(mot_right) + ' '
    if baraban:
        data_str += str(speed_baraban) + ' '
    else:
        data_str += str(baraban) + ' '
    data_str += str(sost) + ' \n'
    s.send(data_str.encode())
    #print("отправил " + data_str)

def robot_is_destroy(destroy = False):
    global mot_left
    global mot_right
    global baraban
    global speed_baraban
    global sost
    if keyboard.is_pressed('q') or destroy:
        mot_left = 0
        mot_right = 0
        baraban = 0
        speed_baraban = 175
        sost = 21
        cap.release()
        cv2.destroyAllWindows()
        if esp_start:
            send()
            s.close()
        exit(0)

def detect_red_square():
    # Загрузка изображения
    image_path = "photo.jpg"  # Укажите свой путь
    img = cv2.imread(image_path)
    if img is None:
        print("Не удалось загрузить изображение")
        return

    # Преобразование в HSV цветовое пространство
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Определение диапазонов красного цвета в HSV
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Создание масок для красного цвета
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Находим координаты всех красных пикселей
    red_pixels = np.where(red_mask == 255)
    if len(red_pixels[0]) == 0:
        print("Красные пиксели не найдены")
        return

    # Преобразуем координаты в удобный формат
    points = np.column_stack((red_pixels[1], red_pixels[0]))  # (x, y)

    # Находим крайние точки (углы квадрата)
    if len(points) > 0:
        # 1. Левый верхний угол (минимальная сумма x и y)
        top_left = min(points, key=lambda p: p[0] + p[1])

        # 2. Правый верхний угол (максимальный x и минимальный y)
        top_right = max(points, key=lambda p: p[0] - p[1])

        # 3. Правый нижний угол (максимальная сумма x и y)
        bottom_right = max(points, key=lambda p: p[0] + p[1])

        # 4. Левый нижний угол (минимальный x и максимальный y)
        bottom_left = min(points, key=lambda p: p[0] - p[1])

        # Собираем углы квадрата
        square_corners = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.int32)

        # Рисуем квадрат
        cv2.polylines(img, [square_corners], isClosed=True, color=(0, 255, 0), thickness=3)

        # Рисуем углы
        for i, corner in enumerate(square_corners):
            cv2.circle(img, tuple(corner), 5, (0, 0, 255), -1)
            cv2.putText(img, f"Corner {i + 1}", (corner[0] + 10, corner[1] + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Выводим координаты углов
        print("Обнаруженные углы квадрата:")
        print(f"1. Левый верхний: {top_left}")
        print(f"2. Правый верхний: {top_right}")
        print(f"3. Правый нижний: {bottom_right}")
        print(f"4. Левый нижний: {bottom_left}")

    # Отображение результата
    cv2.imshow("Red Square Detection", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    output_path = "output.jpg"
    cv2.imwrite(output_path, img)
    print(f"\nРезультат сохранен в {output_path}")

def check_cam():
    robot_is_destroy()

    # ret, frame = cap.read()
    # if not ret:
    #     print("Не удалось считать кадр")
    #     robot_is_destroy(True)

    image_path = "photo.jpg"  # Укажите свой путь
    frame = cv2.imread(image_path)

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Преобразование цветного изображения в черно-белое
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray_frame)
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        for i in range(len(ids)):
            # Получаем координаты углов маркера
            marker_corners = corners[i][0]  # Углы маркера в формате (x, y)
            marker_id = ids[i][0]  # ID маркера

            # Вычисляем центр маркера
            marker_center_x = int(np.mean(marker_corners[:, 0]))  # Среднее по x-координатам углов
            marker_center_y = int(np.mean(marker_corners[:, 1]))  # Среднее по y-координатам углов

            # Вычисляем смещение маркера относительно центра кадра
            offset_x = marker_center_x - center_x
            offset_y = marker_center_y - center_y

            # Вычисляем расстояние до маркера
            marker_width = np.linalg.norm(marker_corners[0] - marker_corners[1])  # Длина верхней стороны маркера
            distance = (marker_size * focal_length) / marker_width  # Расчет расстояния по формуле

            vector_x = marker_corners[1][0] - marker_corners[0][0]
            vector_y = marker_corners[1][1] - marker_corners[0][1]
            angle = np.degrees(np.arctan2(vector_y, vector_x))

            # Отображение информации на изображении
            cv2.putText(frame, f"ID: {marker_id}", (marker_center_x, marker_center_y - 20 * k_string),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5 * k_string, colors, 2 * k_string)
            cv2.putText(frame, f"Offset: (X: {offset_x}, Y: {offset_y})", (marker_center_x, marker_center_y - 40 * k_string),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5 * k_string, colors, 2 * k_string)
            cv2.putText(frame, f"Angle: {angle:.2f} deg", (marker_center_x, marker_center_y - 80 * k_string),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5 * k_string, colors, 2 * k_string)

            # Рисуем центр маркера и центр кадра
            cv2.circle(frame, (marker_center_x, marker_center_y), 5, (0, 255, 0), -1)  # Центр маркера
            # cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)  # Центр кадра

    # cv2.imshow('frame', frame)
    # time.sleep(0.005)
    # cv2.waitKey(1)

    output_path = "output.jpg"
    cv2.imwrite(output_path, frame)
    print(f"\nРезультат сохранен в {output_path}")

if cam:
    print("подключение камеры")
    marker_size = 0.066
    focal_length = 650

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    cap = cv2.VideoCapture(0)

print("запуск")
check_cam()

def main():
    detect_red_square()

    #robot_is_destroy(True)

if __name__ == "__main__":
    main()