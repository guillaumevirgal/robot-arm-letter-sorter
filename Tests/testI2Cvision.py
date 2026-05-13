# test_i2c.py — sends 4 known angles, no bool, for basic validation
import smbus2
import struct

ADRESSE_ARDUINO = 0x08
bus = smbus2.SMBus(1)

Pigeon1 = (50) #Gripper closed
Pigeon2 = (100) #Gripper open

TARGETS = {"George Washington":  Pigeon1,
           "Thomas Jefferson":   Pigeon2,
}

def I2C(angles: list):
    """Send gripper angle as 1 float over I2C. Format: '1f' = 4 bytes."""
    octets = struct.pack('1f', *angles)
    print(f"Sending: {angles}")
    print(f"Raw hex: {octets.hex(' ')}")
    bus.write_i2c_block_data(ADRESSE_ARDUINO, 0, list(octets))

def get_vision():
    # Call the Vision 
    return "George Washington" #return the name

def get_target(vision): # Returns (x, y, z) coordinates for a given name
    if vision not in TARGETS:
        raise ValueError(f"Unknown name : {vision}") # Return an Error, could be changed to put the letter in unlabelled pigeon hole
    return TARGETS[vision]
  




I2C([55.0])  # test with known open angle

vision = get_vision()
try:
    target = get_target(vision)
    I2C([float(target)])
except ValueError as e:
    print(f"[Vision Error] {e}")

bus.close()
