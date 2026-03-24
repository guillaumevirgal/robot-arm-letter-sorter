#main Pi
import smbus2
import struct
import time
import RPi.GPIO as GPIO
from InverseKinematics import compute_ik, check_joint_limits
import numpy as np

GPIO_PIN = 17  # Confirm with hardware
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


ADRESSE_ARDUINO = 0x08
bus = smbus2.SMBus(1)

Pigeon1 = (-30,-7.5,15, -30/np.sqrt(-30**2 + -7.5**2), -7.5/np.sqrt(-30**2 + -7.5**2), 0) #(x,y,z,ux,uy,uz)
Pigeon2 = (-30, 7.5,15, -30/np.sqrt(-30**2 + 7.5**2), 7.5/np.sqrt(-30**2 + 7.5**2), 0)
Pigeon3 = (-30,-7.5,7.5, -30/np.sqrt(-30**2 + -7.5**2), -7.5/np.sqrt(-30**2 + -7.5**2), 0)
Pigeon4 = (-30, 7.5,7.5, -30/np.sqrt(-30**2 + 7.5**2), 7.5/np.sqrt(-30**2 + 7.5**2), 0)


# Coordinates for each name (wait for hardware)
TARGETS = {
    "George Washington":  Pigeon1, 
    "Thomas Jefferson":   Pigeon2,
    "Theodore Roosevelt": Pigeon3,
    "Abraham Lincoln":    Pigeon4,
}

Tray       = (30, 0, 15, 1, 0, 0)  # Tray output, where the letters are taken
VisionPose = (10, 0, 20, 1, 0, 0)  # In front of link 1, letter held vertically for OCR — confirm with hardware

# Gripper angles — confirm with hardware
GRIPPER_OPEN   = 0    # degrees — release letter
GRIPPER_CLOSED = 60   # degrees — hold letter

STEP_PICKUP = 0
STEP_VISION = 1
STEP_SORT   = 2
step = STEP_PICKUP
# current_target is no longer needed: vision is read at the start of STEP_SORT,
# once the arm has reached VisionPose and the Arduino has pulsed back

def I2C(letter_sorted, angles, gripper_angle):  # 1 bool + 5 floats = 21 bytes
    octets = struct.pack('?5f', letter_sorted, *angles, gripper_angle)
    bus.write_i2c_block_data(ADRESSE_ARDUINO, 0, list(octets))


def IK(x, y, z, ux, uy, uz, letter_sorted, gripper_angle):

    # Inverse kinematic result
    try:
        angles_joints = compute_ik(x, y, z, ux, uy, uz)  # IK, returns degrees for the 4 joints
        check_joint_limits(angles_joints)
    except ValueError as e:
        print(f"IK/limit Error: {e}")
        return

    I2C(letter_sorted, angles_joints, gripper_angle)
    print(f"Ready? ({letter_sorted}) | angles: {angles_joints} | gripper: {gripper_angle}°")
    return True


def get_vision():
    # Call the Vision script 
    return "George Washington" #return the name

def get_target(vision): # Returns (x, y, z) coordinates for a given name
    if vision not in TARGETS:
        raise ValueError(f"Unknown name : {vision}") # Return an Error, could be changed to put the letter in unlabelled pigeon hole
    return TARGETS[vision]

def on_pulse(channel):  # Every pulse of the GPIO makes it go to the next step of the process
    global step

    if step == STEP_PICKUP:
        print("[STATE] PICKUP")
        if IK(*Tray, letter_sorted=False, gripper_angle=GRIPPER_CLOSED):
            step = STEP_VISION

    elif step == STEP_VISION:
        print("[STATE] VISION")
        # Only send the arm to VisionPose — do NOT call get_vision() here.
        # The Arduino pulses back once the arm has reached VisionPose.
        # OLD (bug): vision was read immediately after IK(), before the arm moved.
        if IK(*VisionPose, letter_sorted=False, gripper_angle=GRIPPER_CLOSED):
            step = STEP_SORT

    elif step == STEP_SORT:
        print("[STATE] SORT")
        # Arm is now at VisionPose — safe to read the letter
        vision = get_vision()
        try:
            target = get_target(vision)
        except ValueError as e:
            print(f"[Vision Error] {e}")
            return  # Stay in STEP_SORT, retry on next pulse
        if IK(*target, letter_sorted=True, gripper_angle=GRIPPER_OPEN):
            step = STEP_PICKUP



GPIO.add_event_detect(GPIO_PIN, GPIO.RISING, callback=on_pulse, bouncetime=300) #The code run every rising edge/pulse on the GPIO
print("En attente des impulsions Arduino...")

try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Arrêt")
    GPIO.cleanup()
    bus.close()