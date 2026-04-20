# Main.py
import re
import difflib
import serial                               # pyserial: USB serial communication with the Arduino
import numpy as np                          
from IK import compute_ik, check_joint_limits  
from vision import get_vision               

# Serial configuration 
PORT = 'COM9'   # Change to match the port
BAUD = 9600     
ser = serial.Serial(PORT, BAUD, timeout=60) # timeout=60s; max time to wait for Arduino reply

# Pigeon hole coordinates (x, y, z) in mm (?? change to cm)
Pigeon1 = (-26.5, -7.5, 20)
Pigeon2 = (-26.5,  7.5, 20)
Pigeon3 = (-26.5, -7.5,  10)
Pigeon4 = (-26.5,  7.5,  10)

#  Name -> pigeon hole coordinates 
TARGETS = { 
    "George Washington":  Pigeon1,  # top-left pigeon hole
    "Thomas Jefferson":   Pigeon2,  # top-right
    "Theodore Roosevelt": Pigeon3,  # bottom-left
    "Abraham Lincoln":    Pigeon4,  # bottom-right
}

Tray       = (32.5,  0, 15)  # coordinate of the letter when on the tray
VisionPose = (27,  0, 16)  # letter held in front of camera

# Gripper servo angles
GRIPPER_OPEN   = 0    # releases the letter
GRIPPER_CLOSED = 30   # grips the letter

# Backlash compensation (degrees) — tune these experimentally
# Positive = add degrees, Negative = subtract degrees
BACKLASH = {
    'theta_2': 2,   # shoulder
    'theta_3': 10,   # elbow
    'gripper': 0,   # gripper
}


def send_to_arduino(letter_sorted, angles):
    msg = ( #CSV string
        f"{int(letter_sorted)},"        # 1 (done, ready for next) or 0 (still in progress)
        f"{angles[0]:.2f},"             # theta1 — base rotation (degrees)
        f"{angles[1]:.2f},"             # theta2 — shoulder (degrees)
        f"{angles[2]:.2f},"             # theta3 — elbow (degrees)
        f"{angles[3]:.2f}\n"            # gripper angle (degrees)
    )
    ser.write(msg.encode())             # encode string to bytes and send over serial
    print(f"Sent: {msg.strip()}")      


def wait_for_ready(): #wait untill the arduino is finished moving
    print("Waiting for Arduino...")
    while True:                                     # loop until Arduino send "ready"
        line = ser.readline().decode().strip()      # read one line and remove whitespace
        print(f"Arduino: {line}")                   
        if line == "READY":                         # Arduino signals it has finished moving
            return


def move(x, y, z, letter_sorted, gripper_state): # Hub for fonctions
    try:
        angles_joints = compute_ik(x, y, z)         # returns [theta1, theta2, theta3] in degrees
        check_joint_limits(angles_joints)           # ValueError if any angle is out of range
    except ValueError as e:
        print(f"IK/limit Error: {e}")              
        return False                              

    angles_joints[1] += BACKLASH['theta_2']
    angles_joints[2] += BACKLASH['theta_3']
    all_angles = angles_joints + [gripper_state + BACKLASH['gripper']]
    send_to_arduino(letter_sorted, all_angles)      
    wait_for_ready()                               
    return True                                    


def _normalize(s):
    return re.sub(r'\s+', ' ', re.sub(r'[^a-zA-Z ]', '', s)).strip().upper() # Uppercase, remove non-letter characters, collapse whitespace

def get_target(raw_text):
    detected = _normalize(raw_text)

    for name, coords in TARGETS.items():     #Exact match after normalization 
        if _normalize(name) == detected:
            return coords

    
    for name, coords in TARGETS.items(): # Exact name appears somewhere inside the OCR noise
        if _normalize(name) in detected:
            print(f"[Vision] Substring match: '{name}'")
            return coords


    scores = {}
    for name in TARGETS: # partial match
        words = _normalize(name).split()
        tokens = detected.split()
        matched = sum(1 for w in words if any(t in w or w in t for t in tokens))
        scores[name] = matched / len(words)

    best_name = max(scores, key=scores.get)
    if scores[best_name] >= 0.5:                   # 0.5 = at least half the name recognized
        print(f"[Vision] Partial match: '{best_name}' (score={scores[best_name]:.2f})")
        return TARGETS[best_name]

    best_name = max(TARGETS, key=lambda n: difflib.SequenceMatcher( #handles number-letter OCR substitutions (0→O, 1→I, 5→S)
        None, _normalize(n), detected).ratio())
    score = difflib.SequenceMatcher(None, _normalize(best_name), detected).ratio()
    if score >= 0.5:
        print(f"[Vision] Fuzzy match: '{best_name}' (score={score:.2f})")
        return TARGETS[best_name]

    raise ValueError(f"Unknown name: {raw_text!r}")


wait_for_ready()# wait for homing

try:
    while True:

        # PICKUP
        print("[STATE] PICKUP")
        move(*Tray,       letter_sorted=False, gripper_state=GRIPPER_CLOSED)  # go to tray, close gripper

        # VISION 
        print("[STATE] VISION")
        move(*VisionPose, letter_sorted=False, gripper_state=GRIPPER_CLOSED)  # move to camera position

        # SORT 
        print("[STATE] SORT")
        name = get_vision()                         # capture frame and run OCR
        print(f"Detected: '{name}'")
        try:
            target = get_target(name)               # look up the destination pigeon hole
        except ValueError as e:
            print(f"[Vision Error] {e}")             
            continue                                # unknown name -> retry from PICKUP
        move(*target, letter_sorted=True, gripper_state=GRIPPER_OPEN)  # go to pigeon hole, open gripper

except KeyboardInterrupt:
    print("Stopped")
    ser.close()                                     # close the serial port
