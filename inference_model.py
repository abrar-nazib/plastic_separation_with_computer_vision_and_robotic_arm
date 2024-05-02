from ultralytics import YOLO
import cv2
import numpy as np
import time

CAM_NUMBER = 0
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
                
                if(cls == 0):
                    cls = "Non-Plastic"
                else:
                    cls = "Plastic"
                
                if(conf > 0.6):
                    if(cls == "Plastic"):
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    else:
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
 
                    cv2.putText(frame, cls, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            except:
                print("Fail")
                pass
    return frame

def main():
    model_path = 'garbage_2.pt'
    model = load_model(model_path)
    
    cap = cv2.VideoCapture(CAM_NUMBER)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    width = int(cap.get(3))
    height = int(cap.get(4))
    out = cv2.VideoWriter('output_mixed.avi', fourcc, 30.0, (width, height))

    while True:
        t1 = time.time()
        ret, frame = cap.read()
        if ret:
            processed_frame = process_frame(frame, model)
            t2 = time.time()
            fps = 1/(t2-t1)
            cv2.putText(processed_frame, f"FPS: {round(fps, 2)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)     
            cv2.imshow('frame', processed_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
