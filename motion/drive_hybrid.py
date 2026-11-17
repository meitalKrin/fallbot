##imports
import time                       # sleep, timing pulses
import cv2                        # ArUco detection, grayscale conversion
import numpy as np                # math
from picamera2 import Picamera2   # the camera
from picarx import Picarx         # motors, steer

##constants
## DICT_4X4_50 4×4 bits the “dictionary” used to generate the markers.
TARGET_ID = 0 ## for tests
STOP_SIDE = 160 ##math with claude in the documntion
MAX_STEER = 30 ##max angle from the the actual source code from the library of picar
KP = 0.08##math with claude in the documntion
SPEED = 10 ## after defrent speed tests
OBSTACLE_CM = 25 ## from the ultrasonic analysis
PULSE = 0.4
SETTLE = 0.2 ## time for the camera head to stop swaying after the robot stops

##hardware  init
px = Picarx
## point to center pos
px.set_cam_pan_angle(0)
px.set_cam_tilt_angle(0)
px.set_dir_servo_angle(0)
##
cam = Picamera2()
cam.configure(
    cam.create_preview_configuration(
        main={
            "format": "RGB888",
            "size": (640, 480)
        }
    )
)
picam2.start()
# Allow camera to warm up
time.sleep(2)

## acrumarker
det = cv2.aruco.ArucoDetector(
    cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
    cv2.aruco.DetectorParameters()) ## ArucoDetector to control the behavior and thresholds of ArUco marker

##functions
def find_marker(gray): ## is there a marker?
    corners, marker_ids, _ = det.detectMarkers(gray)  # returns 3 values corners, ids, rejected
    if marker_ids is None:                             # no markers in frame at all
        return None
    for k in range(len(marker_ids)): ## temporery code for TARGET_ID for test 
        i = marker_ids[k][0]
        c = corners[k]
        if i == TARGET_ID:
            
            return c[0]
                                       
    return None  

def measure(m):##where is it and how far?
    cx = m[:, 0].mean()                             
    side = np.mean([np.linalg.norm(m[k] - m[(k + 1) % 4]) for k in range(4)]) ##vector between two adjacent corners. math with claude in the documntion
    return cx, side
    
def steer_towards(cx):
    steer = float(np.clip(KP * (cx - 320), -MAX_STEER, MAX_STEER))##math with claude in the documntion
    px.set_dir_servo_angle(steer)
    return steer

def look_stationary():
    time.sleep(SETTLE)
    cam.capture_array()                      
    gray = cv2.cvtColor(cam.capture_array(), cv2.COLOR_BGR2GRAY)
    return find_marker(gray)

def obstacle_confirmed(): ## run  2 times to chekc for an obstical  (the ultrasonic sometimes gives a “jumping” reading that isn’t real in the decomtion )
    hits = 0
    for _ in range(2):
        d = px.ultrasonic.read()
        hits += (d is not None and 0 < d < OBSTACLE_CM)
        time.sleep(0.02)
    return hits == 2

def soft_start():
    for s in range(0, SPEED + 1, 2):
        px.forward(s); time.sleep(0.05)



LOST_PULSE = 3    # misses allowed in pulse mode 
LOST_CONT  = 15   # misses allowed in continuous mode
mode = "pulse"    
seen = False
lost = 0
try:
    while True:
        
        ## PULSE  MODE
        if

        ## CONTINUOUS MODE
        else:
    

    
finally:
