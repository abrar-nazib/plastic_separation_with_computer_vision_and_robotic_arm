import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
    QLabel,
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from ultralytics import YOLO
import time
import random
import servocontroller
import pandas as pd

CAM_NUMBER = 1
RECORD = False

classes = {
    0: "Paysaa! $_$",
    2: "Plastic",
    1: "Paper",
}


class PickingApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Picking Application")
        self.setGeometry(100, 100, 800, 600)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        self.camera_label = QLabel()
        layout.addWidget(self.camera_label)

        self.start_button = QPushButton("Start Picking")
        self.start_button.clicked.connect(self.start_picking)
        layout.addWidget(self.start_button)

        self.cap = connect_camera(CAM_NUMBER)
        self.pointresolver = PointResolver("cleaned_data_2.csv")
        self.arduino = servocontroller.connect_arduino()

        # Make sure the arduino is connected
        servocontroller.move_arm(self.arduino, 80, 80, 80)
        servocontroller.move_arm(self.arduino, 90, 90, 90)

        self.model = load_model("final_model.pt")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)  # Update every 30 ms

        self.is_picking = False

    def update_frame(self):
        frame = read_camera(self.cap, 960, 720)
        processed_frame = preprocess_image(
            frame, (236, 54), (833, 49), (236, 464), (862, 435)
        )
        boxes = detect_plastic(processed_frame, self.model)
        frame = draw_boxes(processed_frame, boxes)

        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        q_image = QImage(
            frame.data, width, height, bytes_per_line, QImage.Format_RGB888
        ).rgbSwapped()
        self.camera_label.setPixmap(QPixmap.fromImage(q_image))

        if self.is_picking and len(boxes) > 0:
            self.pick_object(boxes[0])

    def start_picking(self):
        self.is_picking = True
        self.start_button.setEnabled(False)

    def pick_object(self, box):
        cx, cy = get_center(box)
        point_touch, point_hover = self.pointresolver.get_closest_point(cx, cy)
        point_hover = point_hover.to_dict()
        pickup_object(
            self.arduino,
            point_hover["S1"],
            point_hover["S2"],
            point_hover["S3"],
            1 if box[4] == "Paper" else 0,
        )

        # (0, 155, 125)
        if box[4] == "Paper":
            servocontroller.move_arm(self.arduino, 0, 155, 125)
        # (180, 155, 125)
        if box[4] == "Plastic":
            servocontroller.move_arm(self.arduino, 180, 155, 125)
        # 154:45:0
        if box[4] == "Paysaa! $_$":
            servocontroller.move_arm(self.arduino, 0, 45, 154)

        # Release the object
        servocontroller.turn_off_suction_pump(self.arduino)

        # Get back to position
        servocontroller.move_arm(self.arduino, 90, 90, 90)
        self.is_picking = False
        self.start_button.setEnabled(True)


# The rest of the functions remain the same as in the original code
def get_center(box):
    x1, y1, x2, y2, cls = box
    return (x1 + x2) // 2, (y1 + y2) // 2


def load_model(model_path):
    print("[INFO] Loading model...")
    model = YOLO(model_path)
    print("[INFO] Model loaded")
    return model


def connect_camera(cam_number):
    print("[INFO] Connecting to camera...")
    try:
        cap = cv2.VideoCapture(cam_number)
    except:
        print("Error connecting to camera")
        cap = None
    return cap


def read_camera(cap, width, height):
    try:
        ret, frame = cap.read()
    except:
        ret = False
    if ret:
        frame = cv2.resize(frame, (width, height))
        return frame
    else:
        frame = np.zeros((height, width, 3), np.uint8)
    return frame


def preprocess_image(frame, t_l_point, t_r_point, b_l_point, b_r_point):
    points = [t_l_point, t_r_point, b_l_point, b_r_point]
    for point in points:
        cv2.circle(frame, point, 5, (0, 0, 255), -1)

    cv2.polylines(
        frame,
        [np.array([t_l_point, t_r_point, b_r_point, b_l_point], np.int32)],
        True,
        (0, 255, 0),
        2,
    )

    mask = np.zeros(frame.shape, np.uint8)
    roi_corners = np.array([t_l_point, t_r_point, b_r_point, b_l_point], dtype=np.int32)
    white = (255, 255, 255)
    cv2.fillPoly(mask, [roi_corners], white)
    frame = cv2.bitwise_and(frame, mask)
    return frame


def detect_plastic(frame, model):
    results = model.predict(frame, verbose=False)
    boxes = []
    for result in results:
        for box in result.boxes:
            try:
                x1, y1, x2, y2, conf, cls = box.data[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cls = classes[int(cls.item())]
                if conf > 0.6:
                    boxes.append([x1, y1, x2, y2, cls])
            except:
                pass
    return boxes


def draw_boxes(frame, boxes):
    for box in boxes:
        x1, y1, x2, y2, cls = box
        if cls == "Plastic":
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        elif cls == "Paper":
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        else:
            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (
                    random.randint(0, 255),
                    random.randint(0, 255),
                    random.randint(0, 255),
                ),
                2,
            )
        cv2.putText(
            frame,
            cls,
            (x1, y1),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
    return frame


class PointResolver:
    def __init__(self, data_file):
        self.data = pd.read_csv(data_file)

    def get_closest_point(self, x, y):
        self.data["dist"] = np.sqrt(
            (self.data["XX"] - x) ** 2 + (self.data["YY"] - y) ** 2
        )

        min_dist_idx = self.data["dist"].idxmin()
        closest_point = self.data.iloc[min_dist_idx]

        if closest_point["Hover"] == 0:
            closest_point_touch = closest_point
            closest_point_hover = self.data[
                (self.data["Hover"] == 1)
                & (self.data["XX"] == closest_point_touch["XX"])
                & (self.data["YY"] == closest_point_touch["YY"])
            ].iloc[0]
        else:
            closest_point_hover = closest_point
            closest_point_touch = self.data[
                (self.data["Hover"] == 0)
                & (self.data["XX"] == closest_point_hover["XX"])
                & (self.data["YY"] == closest_point_hover["YY"])
            ].iloc[0]
        return closest_point_touch, closest_point_hover


def pickup_object(arduino, s1, s2, s3, height=0):
    if height == 0:
        servocontroller.move_arm(arduino, s1, s2, s3)
        servocontroller.turn_on_suction_pump(arduino)
        time.sleep(2)

        s2 += 15
        s3 -= 15
        servocontroller.move_arm(arduino, s1, s2, s3)
        s1 -= 10

        servocontroller.move_arm(arduino, s1, s2, s3)

        # s1 += 10
        # servocontroller.move_arm(arduino, s1, s2, s3)

        s2 -= 15
        s3 += 15
        servocontroller.move_arm(arduino, s1, s2, s3)
    else:
        servocontroller.move_arm(arduino, s1, s2 - 10, s3 + 10)
        servocontroller.turn_on_suction_pump(arduino)
        time.sleep(2)

        s2 += 8
        s3 -= 8
        servocontroller.move_arm(arduino, s1, s2, s3)
        s1 -= 10

        servocontroller.move_arm(arduino, s1, s2, s3)

        s1 += 10
        servocontroller.move_arm(arduino, s1, s2, s3)

        s2 -= 8
        s3 += 8
        servocontroller.move_arm(arduino, s1, s2, s3)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PickingApp()
    window.show()
    sys.exit(app.exec_())
