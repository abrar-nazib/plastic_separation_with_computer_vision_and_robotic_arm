from ultralytics import YOLO
import cv2
import numpy as np
import time
import servocontroller
import pandas as pd

CAM_NUMBER = "mixed_2.mp4"
CAPTURE_VIDEO = False


def load_model(model_path):
    return YOLO(model_path)


def process_frame(frame, model):
    results = model.predict(frame, verbose=False)
    for result in results:
        for box in result.boxes:
            try:
                x1, y1, x2, y2, conf, cls = box.data[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                if cls == 0:
                    cls = "Non-Plastic"
                else:
                    cls = "Plastic"

                if conf > 0.6:
                    if cls == "Plastic":
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    else:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    cv2.putText(
                        frame,
                        cls,
                        (x1, y1),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
            except:
                print("Fail")
                pass
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


def main():
    model_path = "garbage_2.pt"
    model = load_model(model_path)
    pointresolver = PointResolver("cleaned_data.csv")

    cap = cv2.VideoCapture(CAM_NUMBER)
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    width = int(cap.get(3))
    height = int(cap.get(4))
    out = cv2.VideoWriter("output_mixed.avi", fourcc, 30.0, (width, height))

    while True:
        t1 = time.time()
        ret, frame = cap.read()
        if ret:
            processed_frame = process_frame(frame, model)
            t2 = time.time()
            fps = 1 / (t2 - t1)
            cv2.putText(
                processed_frame,
                f"FPS: {round(fps, 2)}",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("frame", processed_frame)
            out.write(processed_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()


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


if __name__ == "__main__":

    main()
