import smbus2
import struct
import time
from InverseKinematics import compute_ik

ADRESSE_ARDUINO = 0x08
bus = smbus2.SMBus(1)


def I2C(letter_sorted, angles: list[float]): # Send state + angles IK : True + 4 servos angles = 1 bool + 4 floats = 17 octets
    octets = struct.pack('?4f', letter_sorted, *angles)
    bus.write_i2c_block_data(ADRESSE_ARDUINO, 0, list(octets))



def pipeline(x_target, y_target, z_target, letter_sorted):
     

    # Inverse kinematic result 
    try :
        angles_joints = compute_ik(x_target, y_target, z_target)  # IK, return degrees of the 4 servos
    except ValueError as e:
        print(f"IK Error : {e}")
        return
    
    I2C(letter_sorted, angles_joints)
    print(f"Ready ? : ({letter_sorted}) | angles: {angles_joints}")

# Coordinates for each name (wait for hardware)
TARGETS = {
    "George Washington":  (20.0, 20.0, 10.0),
    "Thomas Jefferson":   (30.0, 10.0, 10.0),
    "Theodore Roosevelt": (10.0, 30.0, 10.0),
    "Abraham Lincoln":    (25.0, 25.0, 10.0),
}

def get_target(vision):
    # Returns (x, y, z) coordinates for a given name
    if vision not in TARGETS:
        raise ValueError(f"Unknown name : {vision}") # Return an Error, could be changed to put the letter in unlabelled pigeon hole
    return TARGETS[vision]



try:
    while True:
        vision = "George Washington" #vision is the result of the machine vision algorithm
        # letter_sorted says if the letter has been letter_sorted and if we can go to the next one
        letter_sorted = True 

        try:
            x, y, z = get_target(vision)
        except ValueError as e:
            print(e)
            time.sleep(0.05)
            continue  # Skip this cycle, wait for next detection

        pipeline(x, y, z, letter_sorted)
        time.sleep(0.05)  # wait for 50ms, 20 Hz

except KeyboardInterrupt: #stop I2C by ctrl+C in the terminal
    print("Arrêt")
    bus.close()