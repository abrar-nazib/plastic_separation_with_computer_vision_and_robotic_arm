import cv2
import numpy as np
from ultralytics import YOLO
import time
CAM_NUMBER = 0
RECORD = False

def load_model(model_path):
    print("[INFO] Loading model...")
    return YOLO(model_path)

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
    cv2.polylines(frame, [np.array([t_l_point, t_r_point, b_r_point, b_l_point], np.int32)], True, (0, 255, 0), 2)
    
    # Draw 10 dots on the line joining t_l_point and b_l_point
    l_points = []
    for i in range(1, 11):
        x = t_l_point[0] + i * (b_l_point[0] - t_l_point[0]) / 11
        y = t_l_point[1] + i * (b_l_point[1] - t_l_point[1]) / 11
        cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)
        l_points.append((int(x), int(y)))
        
    # Draw 10 dots on the line joining t_r_point and b_r_point
    r_points = []
    for i in range(1, 11):
        x = t_r_point[0] + i * (b_r_point[0] - t_r_point[0]) / 11
        y = t_r_point[1] + i * (b_r_point[1] - t_r_point[1]) / 11
        cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)
        r_points.append((int(x), int(y)))
    
    # Draw 15 dots on the line joining t_l_point and t_r_point
    top_points = []
    for i in range(1, 16):
        x = t_l_point[0] + i * (t_r_point[0] - t_l_point[0]) / 16
        y = t_l_point[1] + i * (t_r_point[1] - t_l_point[1]) / 16
        cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)
        top_points.append((int(x), int(y)))
        
    # Draw 15 dots on the line joining b_l_point and b_r_point
    bottom_points = []
    for i in range(1, 16):
        x = b_l_point[0] + i * (b_r_point[0] - b_l_point[0]) / 16
        y = b_l_point[1] + i * (b_r_point[1] - b_l_point[1]) / 16
        cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)
        bottom_points.append((int(x), int(y)))
        
    # Draw thin lines between left and right points
    for i in range(len(l_points)):
        cv2.line(frame, l_points[i], r_points[i], (0, 0, 255), 1)
    
    # Draw thin lines between top and bottom points
    for i in range(len(top_points)):
        cv2.line(frame, top_points[i], bottom_points[i], (0, 0, 255), 1)
        
    
    
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

def detect_plastic(frame, model):
    # Detect plastic in the frame and return the Bounding boxes in an array
    results = model.predict(frame, verbose=False)
    boxes = []
    for result in results:
        for box in result.boxes:
            try:
                x1, y1, x2, y2, conf, cls = box.data[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                if(cls == 0):
                    cls = "Non-Plastic"
                else:
                    cls = "Plastic"
                if(conf > 0.6):
                    boxes.append([x1, y1, x2, y2, cls])
            except:
                pass
    return boxes

def draw_boxes(frame, boxes):
    for box in boxes:
        x1, y1, x2, y2, cls = box
        if(cls == "Plastic"):
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        else:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, cls, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return frame

def main():
    cap = connect_camera(CAM_NUMBER)

 

    width = int(640*1.5)
    height = int(480*1.5)

    if RECORD:
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter('output_pick_place.avi', fourcc, 30.0, (width, height))

    model = load_model('garbage_2.pt')
    while True:
        frame = read_camera(cap, width, height)
        # (219, 19) (841, 11) (232,395) (852, 378)
        processed_frame = preprocess_image(frame, (236, 54), (833, 49), (236,464), (862, 435))
        boxes = detect_plastic(processed_frame, model)
        draw_boxes(frame, boxes)
        cv2.imshow('frame', frame)
        if RECORD:
            out.write(frame)

        cv2.setMouseCallback('frame', mouse_click)

        if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
            break
    cap.release()
if __name__ == "__main__":
    main()