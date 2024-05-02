import serial
import time
import math
import tkinter as tk

COM_PORT="COM5"
SUCTION_PUMP_PIN = 9
SUCTION_PUMP_STATE = False
BASE_SERVO_PIN = 3
BASE_SERVO_HOME_ANGLE = 90
BASE_SERVO_ANGLE = 90
SHOULDER_SERVO_PIN = 11
SHOULDER_SERVO_HOME_ANGLE = 90
SHOULDER_SERVO_ANGLE = 90
ELBOW_SERVO_PIN = 5
ELBOW_SERVO_HOME_ANGLE = 90
ELBOW_SERVO_ANGLE  = 90

def connect_arduino():
    try:
        ser = serial.Serial(COM_PORT, 115200)
        time.sleep(2)
        print("Connected to Arduino")
    except:
        print("Error connecting to Arduino")
        ser = None
    return ser

def send_data(arduino):
    global BASE_SERVO_ANGLE, SHOULDER_SERVO_ANGLE, ELBOW_SERVO_ANGLE, SUCTION_PUMP_STATE
    cmd_str = f"{int(BASE_SERVO_ANGLE)}:{int(SHOULDER_SERVO_ANGLE)}:{int(ELBOW_SERVO_ANGLE)}:{1 if SUCTION_PUMP_STATE else 0}\n"
    print(f"[INFO] Sending command: {cmd_str}")
    arduino.write(cmd_str.encode())
    time.sleep(0.05)

# Create a trackbar with tkinter to control servo
def create_servo_control_gui(arduino):
    global SUCTION_PUMP_STATE
    def set_base_angle(val):
        global BASE_SERVO_ANGLE
        BASE_SERVO_ANGLE = int(val)
        send_data(arduino)

    def set_shoulder_angle(val):    
        global SHOULDER_SERVO_ANGLE
        SHOULDER_SERVO_ANGLE = int(val)
        send_data(arduino)

    def set_elbow_angle(val):
        global ELBOW_SERVO_ANGLE
        ELBOW_SERVO_ANGLE = int(val)
        send_data(arduino)

    def toggle_suction_pump():
        global SUCTION_PUMP_STATE
        SUCTION_PUMP_STATE = not SUCTION_PUMP_STATE
        send_data(arduino)

    

    
    
    root = tk.Tk()
    root.title("Servo Control")
    root.geometry("1920x300")
    
    scale1 = tk.Scale(root, from_=0, to=180, orient=tk.HORIZONTAL, command=set_base_angle, length=1000)
    # Set the initial value of the scale
    scale1.set(BASE_SERVO_HOME_ANGLE)
    scale1.pack()

    scale2 = tk.Scale(root, from_=0, to=180, orient=tk.HORIZONTAL, command=set_shoulder_angle, length=1000)
    scale2.set(SHOULDER_SERVO_HOME_ANGLE)
    scale2.pack()

    scale3 = tk.Scale(root, from_=0, to=180, orient=tk.HORIZONTAL, command=set_elbow_angle, length=1000)
    scale3.set(ELBOW_SERVO_HOME_ANGLE)
    scale3.pack()

    # Checkbox
    checkbox =  tk.Checkbutton(root, text="Suction Pump", command=toggle_suction_pump)
    checkbox.pack()

    # Angle textbox 
    angle_textbox = tk.Entry(root)
    angle_textbox.pack()
    
        # Print the angles with button
    def print_angles(angle_entry_box):
        # Clear the current text in the entry box
        angle_entry_box.delete(0, tk.END)

        # Insert the new text
        angle_entry_box.insert(0, f"{BASE_SERVO_ANGLE}:{SHOULDER_SERVO_ANGLE}:{ELBOW_SERVO_ANGLE}")

        print(f"Base Angle: {BASE_SERVO_ANGLE}, Shoulder Angle: {SHOULDER_SERVO_ANGLE}, Elbow Angle: {ELBOW_SERVO_ANGLE}")
        root.clipboard_clear()
        root.clipboard_append(f"{BASE_SERVO_ANGLE}:{SHOULDER_SERVO_ANGLE}:{ELBOW_SERVO_ANGLE}")
    
    # Button
    button = tk.Button(root, text="Print Angles", command=lambda: print_angles(angle_textbox))
    button.pack()

    
    def increase_scale1(event):
        scale1.set(scale1.get() + 1)

    def decrease_scale1(event):
        scale1.set(scale1.get() - 1)

    def increase_scale2(event):
        scale2.set(scale2.get() + 1)

    def decrease_scale2(event):
        scale2.set(scale2.get() - 1)

    def increase_scale3(event):
        scale3.set(scale3.get() + 1)

    def decrease_scale3(event):
        scale3.set(scale3.get() - 1)
    
    root.bind('p', increase_scale1)
    root.bind('q', decrease_scale1)
    root.bind('l', increase_scale2)
    root.bind('a', decrease_scale2)
    root.bind('m', increase_scale3)
    root.bind('z', decrease_scale3)
    
    

    root.mainloop()


def move_arm(arduino, base_angle, shoulder_angle, elbow_angle):
    global BASE_SERVO_ANGLE, SHOULDER_SERVO_ANGLE, ELBOW_SERVO_ANGLE

    # Get the delta for each angle
    base_delta = base_angle - BASE_SERVO_ANGLE
    shoulder_delta = shoulder_angle - SHOULDER_SERVO_ANGLE
    elbow_delta = elbow_angle - ELBOW_SERVO_ANGLE

    # Get the number of steps to move
    steps = int(max(abs(base_delta), abs(shoulder_delta), abs(elbow_delta)))
    if steps == 0:
        return
    # Calculate the step size for each angle
    base_step = base_delta / steps
    shoulder_step = shoulder_delta / steps
    elbow_step = elbow_delta / steps

    # Move the arm to the new position
    for i in range(steps):
        BASE_SERVO_ANGLE += base_step
        SHOULDER_SERVO_ANGLE += shoulder_step
        ELBOW_SERVO_ANGLE += elbow_step
        send_data(arduino)
        time.sleep(0.01)
    BASE_SERVO_ANGLE = base_angle
    SHOULDER_SERVO_ANGLE = shoulder_angle
    ELBOW_SERVO_ANGLE = elbow_angle
    send_data(arduino)

# create_servo_control_gui(arduino)

# # Move the arm to the home position

# [90, 45, 150]
# [53, 135, 74]
# [157, 135, 85]
    
# Connect arduino
# arduino = connect_arduino()

# # Move the arm to the home position
# for i in range(10):
#     send_data(arduino)
#     time.sleep(0.5)

# Move arm angles from one position to another
def move_strategy(arduino):
    global SUCTION_PUMP_STATE
    for i in range(5):
        send_data(arduino)
        time.sleep(0.05)

    # # # # Move the arm to the new position
    move_arm(arduino, 155, 135, 90)
    SUCTION_PUMP_STATE = True
    send_data(arduino)
    move_arm(arduino, 155, 135, 80)
    move_arm(arduino, 155, 140, 75)
    time.sleep(2)
    move_arm(arduino, 155, 130, 100)
    move_arm(arduino, 32, 130, 100)
    move_arm(arduino, 32, 170, 135)
    SUCTION_PUMP_STATE = False
    send_data(arduino)
    move_arm(arduino, 32, 130, 100)

    move_arm(arduino, 90, 90, 90)
    
    move_arm(arduino, 40, 135, 90)
    SUCTION_PUMP_STATE = True
    send_data(arduino)
    move_arm(arduino, 40, 135, 80)
    move_arm(arduino, 40, 140, 75)
    move_arm(arduino, 40, 130, 100)
    time.sleep(2)
    move_arm(arduino, 145, 130, 100)
    move_arm(arduino, 145, 170, 135)
    SUCTION_PUMP_STATE = False
    send_data(arduino)
    move_arm(arduino, 145, 130, 100)
    
    move_arm(arduino, 90, 90, 90)
    time.sleep(2)

    

arduino = connect_arduino()
# move_strategy(arduino)
create_servo_control_gui(arduino)