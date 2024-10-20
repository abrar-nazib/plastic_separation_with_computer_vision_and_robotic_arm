import cv2
import numpy as np
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
        # Create a black rgb frame
        frame = np.zeros((height, width, 3), np.uint8)

    return frame


def preprocess_image(frame, t_l_point, t_r_point, b_l_point, b_r_point):
    points = [t_l_point, t_r_point, b_l_point, b_r_point]
    for point in points:
        cv2.circle(frame, point, 5, (0, 0, 255), -1)

    # Draw polygon
    cv2.polylines(
        frame,
        [np.array([t_l_point, t_r_point, b_r_point, b_l_point], np.int32)],
        True,
        (0, 255, 0),
        2,
    )

    # # Draw 10 dots on the line joining t_l_point and b_l_point
    # l_points = []
    # for i in range(1, 11):
    #     x = t_l_point[0] + i * (b_l_point[0] - t_l_point[0]) / 11
    #     y = t_l_point[1] + i * (b_l_point[1] - t_l_point[1]) / 11
    #     cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)
    #     l_points.append((int(x), int(y)))

    # # Draw 10 dots on the line joining t_r_point and b_r_point
    # r_points = []
    # for i in range(1, 11):
    #     x = t_r_point[0] + i * (b_r_point[0] - t_r_point[0]) / 11
    #     y = t_r_point[1] + i * (b_r_point[1] - t_r_point[1]) / 11
    #     cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)
    #     r_points.append((int(x), int(y)))

    # # Draw 15 dots on the line joining t_l_point and t_r_point
    # top_points = []
    # for i in range(1, 16):
    #     x = t_l_point[0] + i * (t_r_point[0] - t_l_point[0]) / 16
    #     y = t_l_point[1] + i * (t_r_point[1] - t_l_point[1]) / 16
    #     cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)
    #     top_points.append((int(x), int(y)))

    # # Draw 15 dots on the line joining b_l_point and b_r_point
    # bottom_points = []
    # for i in range(1, 16):
    #     x = b_l_point[0] + i * (b_r_point[0] - b_l_point[0]) / 16
    #     y = b_l_point[1] + i * (b_r_point[1] - b_l_point[1]) / 16
    #     cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)
    #     bottom_points.append((int(x), int(y)))

    # # Draw thin lines between left and right points
    # for i in range(len(l_points)):
    #     cv2.line(frame, l_points[i], r_points[i], (0, 0, 255), 1)

    # # Draw thin lines between top and bottom points
    # for i in range(len(top_points)):
    #     cv2.line(frame, top_points[i], bottom_points[i], (0, 0, 255), 1)

    # Make everything outside the rectangle black
    mask = np.zeros(frame.shape, np.uint8)
    roi_corners = np.array([t_l_point, t_r_point, b_r_point, b_l_point], dtype=np.int32)
    white = (255, 255, 255)
    cv2.fillPoly(mask, [roi_corners], white)
    frame = cv2.bitwise_and(frame, mask)
    return frame


# Mouse click event
def mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(x, y)
        print(x, y)


def detect_plastic(frame, model):
    # Detect plastic in the frame and return the Bounding boxes in an array
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
            # Put a random color for other classes

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
        # df = pd.DataFrame(columns=["Hover", "XX", "YY", "S1", "S2", "S3"])

    def get_closest_point(self, x, y):
        closest_point = None
        self.data["dist"] = np.sqrt(
            (self.data["XX"] - x) ** 2 + (self.data["YY"] - y) ** 2
        )

        # Find the index of the minimum distance
        min_dist_idx = self.data["dist"].idxmin()

        # Select the closest point
        closest_point = self.data.iloc[min_dist_idx]

        hover_points = self.data[self.data["Hover"] == 1]
        touch_points = self.data[self.data["Hover"] == 0]
        # Check whether the Hover is 0 or 1
        if closest_point["Hover"] == 0:
            closest_point_touch = closest_point
            # Find the point with Hover = 1
            closest_point_hover = self.data[
                (self.data["Hover"] == 1)
                & (self.data["XX"] == closest_point_touch["XX"])
                & (self.data["YY"] == closest_point_touch["YY"])
            ].iloc[0]

            return closest_point_touch, closest_point_hover
        else:
            closest_point_hover = closest_point
            # Find the point with Hover = 0
            closest_point_touch = self.data[
                (self.data["Hover"] == 0)
                & (self.data["XX"] == closest_point_hover["XX"])
                & (self.data["YY"] == closest_point_hover["YY"])
            ].iloc[0]
            return closest_point_touch, closest_point_hover


def pickup_object(arduino, s1, s2, s3, height=0):
    if height == 0:
        # Move to the hover position
        servocontroller.move_arm(arduino, s1, s2, s3)
        # Turn on the suction pump
        servocontroller.turn_on_suction_pump(arduino)
        time.sleep(2)

        s2 += 18
        s3 -= 18
        # Move to the touch position
        servocontroller.move_arm(arduino, s1, s2, s3)
        # Move the base servo a bit to the left
        s1 -= 10

        servocontroller.move_arm(arduino, s1, s2, s3)

        s1 += 10
        servocontroller.move_arm(arduino, s1, s2, s3)

        # Get back to the hover position
        s2 -= 18
        s3 += 18
        servocontroller.move_arm(arduino, s1, s2, s3)
    else:
        # Move to the hover position
        servocontroller.move_arm(arduino, s1, s2 - 10, s3 + 10)
        # Turn on the suction pump
        servocontroller.turn_on_suction_pump(arduino)
        time.sleep(2)

        s2 += 12
        s3 -= 12
        # Move to the touch position
        servocontroller.move_arm(arduino, s1, s2, s3)
        # Move the base servo a bit to the left
        s1 -= 10

        servocontroller.move_arm(arduino, s1, s2, s3)

        s1 += 10
        servocontroller.move_arm(arduino, s1, s2, s3)

        # Get back to the hover position
        s2 -= 12
        s3 += 12
        servocontroller.move_arm(arduino, s1, s2, s3)


def main():
    busy = False
    cap = connect_camera(CAM_NUMBER)
    pointresolver = PointResolver("cleaned_data.csv")
    arduino = servocontroller.connect_arduino()
    width = int(640 * 1.5)
    height = int(480 * 1.5)

    if RECORD:
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        out = cv2.VideoWriter("output_pick_place.avi", fourcc, 30.0, (width, height))

    model = load_model("final_model.pt")
    while True:
        frame = read_camera(cap, width, height)
        # (219, 19) (841, 11) (232,395) (852, 378)
        processed_frame = preprocess_image(
            frame, (236, 54), (833, 49), (236, 464), (862, 435)
        )
        boxes = detect_plastic(processed_frame, model)
        if len(boxes) > 0:
            cx, cy = get_center(boxes[0])
            print(cx, cy)
            point_touch, point_hover = pointresolver.get_closest_point(cx, cy)
            point_hover = point_hover.to_dict()
            pickup_object(
                arduino,
                point_hover["S1"],
                point_hover["S2"],
                point_hover["S3"],
                0,
            )
            servocontroller.move_arm(arduino, 90, 90, 90)
            servocontroller.turn_off_suction_pump(arduino)
        frame = draw_boxes(processed_frame, boxes)
        cv2.imshow("frame", frame)
        cv2.imshow("frame", processed_frame)
        if RECORD:
            out.write(frame)

        cv2.setMouseCallback("frame", mouse_click)

        if cv2.waitKey(1) & 0xFF in [ord("q"), 27]:
            break
    cap.release()


if __name__ == "__main__":
    main()
