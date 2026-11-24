##imports
import time                       # sleep, timing pulses
import cv2                        # ArUco detection, grayscale conversion
import numpy as np                # math
from picamera2 import Picamera2   # the camera
from picarx import Picarx         # motors, steer

##constants
## DICT_4X4_50 4×4 bits the “dictionary” used to generate the markers.
STOP_SIDE = 160
TARGET_ID = 0   # marker for the standalone test (patrol will pass ROUTE[i])
MAX_STEER = 30 ##max angle from the the actual source code from the library of picar
KP = 0.08
SPEED = 10 ## after defrent speed tests
OBSTACLE_CM = 25 ## from the ultrasonic analysis
PULSE = 0.4
SETTLE = 0.2 ## time for the camera head to stop swaying after the robot stops
FRAME_CX = 320 ## 640/2, image center (camera runs at 640x480)

##hardware  init
px = Picarx()
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
cam.start()
# Allow camera to warm up
time.sleep(2)

## acrumarker
det = cv2.aruco.ArucoDetector(
    cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
    cv2.aruco.DetectorParameters()) ## ArucoDetector to control the behavior and thresholds of ArUco marker

##functions
def find_marker(gray, target_id):  # return corners (4x2) of the marker with this id, or None
    corners, marker_ids, _ = det.detectMarkers(gray)  # returns 3 values: corners, ids, rejected
    if marker_ids is None:                             # no markers in frame at all
        return None
    for k in range(len(marker_ids)):
        if marker_ids[k][0] == target_id:
            return corners[k][0]
    return None

def measure(m):##where is it and how far?
    cx = m[:, 0].mean()
    side = np.mean([np.linalg.norm(m[k] - m[(k + 1) % 4]) for k in range(4)]) ##mean size or  the side of the marker
    return cx, side

def steer_towards(cx):
    steer = float(np.clip(KP * (cx - FRAME_CX), -MAX_STEER, MAX_STEER))##math with claude in the documntion
    px.set_dir_servo_angle(steer)
    return steer

def look_stationary(target_id):  # robot must already be stopped: wait for settle, drop one stale frame, return marker corners (4x2) or None
    time.sleep(SETTLE)
    cam.capture_array()
    gray = cv2.cvtColor(cam.capture_array(), cv2.COLOR_BGR2GRAY)
    return find_marker(gray, target_id)

def obstacle_confirmed(): ## run  2 times to chekc for an obstical  (the ultrasonic sometimes gives a “jumping” reading that isn’t real in the decomtion )
    hits = 0
    for _ in range(2):
        d = px.ultrasonic.read()
        hits += (0 < d < OBSTACLE_CM)
        time.sleep(0.02)
    return hits == 2

def soft_start():
    for s in range(0, SPEED + 1, 2):
        px.forward(s); time.sleep(0.05)


LOST_PULSE = 3    # misses allowed in pulse mode
LOST_CONT  = 15   # misses allowed in continuous mode
NEAR_SIDE  = 38   # switch pulse → continuous
FAR_SIDE   = 30   # switch continuous → pulse
assert FAR_SIDE < NEAR_SIDE < STOP_SIDE, "thresholds out of order"

def approach(target_id):
    """Drive to marker target_id. Returns "arrived", "lost" or "obstacle"."""
    mode = "pulse"
    seen = False
    lost = 0
    t_prev = None
    while True:

        ## PULSE  MODE
        if mode == "pulse":
            m = look_stationary(target_id)
            if m is None:
                lost += 1
                if lost >= LOST_PULSE:
                    px.stop()
                    print(f"[pulse] marker {target_id} lost after {lost} looks - stopping")
                    return "lost"
                continue
            lost = 0        # miss count starts over
            seen = True     # seen at least once
            cx, side = measure(m)
            steer_towards(cx)
            print(f"[pulse] cx={cx:.0f} side={side:.0f}")

            # ARRIVED
            if side >= STOP_SIDE:
                px.stop()
                print(f"[pulse] ARRIVED side={side:.0f}")
                return "arrived"

            # OBSTACLE (checked before ANY command that moves the motors)
            if obstacle_confirmed():
                px.stop()
                print("[pulse] obstacle - stopping")
                return "obstacle"

            ## pulse → continuous
            if side >= NEAR_SIDE:
                mode = "continuous"
                lost = 0
                print(f"[pulse] -> continuous side={side:.0f}")
                soft_start()
                continue

            # PULSE
            px.forward(SPEED)
            time.sleep(PULSE)
            px.stop()

        ## CONTINUOUS MODE
        else:
            # loop time measurement
            now = time.monotonic()
            dt_ms = (now - t_prev) * 1000 if t_prev is not None else 0.0
            t_prev = now

            #  robot is moving -> no SETTLE, no dropped frame
            gray = cv2.cvtColor(cam.capture_array(), cv2.COLOR_BGR2GRAY)
            m =  find_marker(gray, target_id)

            if m is None:
                lost += 1
                if lost >= LOST_CONT:
                    px.stop()
                    mode = "pulse"
                    lost = 0
                    t_prev = None
                    print(f"[cont] lost {LOST_CONT} frames -> pulse")
                continue

            lost = 0
            cx, side = measure(m)
            steer_towards(cx)
            px.forward(SPEED)
            print(f"[cont] cx={cx:.0f} side={side:.0f} dt={dt_ms:.0f}ms")

            # ARRIVED
            if side >= STOP_SIDE:
                px.stop()
                print(f"[cont] ARRIVED side={side:.0f}")
                return "arrived"

            # OBSTACLE: moving -> ONE reading is enough
            d = px.ultrasonic.read()
            if 0 < d < OBSTACLE_CM:
                px.stop()
                print(f"[cont] obstacle d={d} - stopping")
                return "obstacle"


            if side < FAR_SIDE:
                px.stop()
                mode = "pulse"
                t_prev = None
                print(f"[cont] -> pulse side={side:.0f}")



if __name__ == "__main__":
    try:
        result = approach(TARGET_ID)
        print("result:", result)
    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        cam.stop()