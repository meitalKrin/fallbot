
import sys
import time
import math
from drive_hybrid import look_stationary, measure, px, cam, FRAME_CX, SPEED, MAX_STEER

PAN_ANGLES = [0, -45, 45, -90, 90] 
PAN_SETTLE = 0.3                    
F_PX = 583                       
PAN_SIGN = 1                    
STEP_T = 0.3                             # s per half-move (turn_test.py, 24 Nov)
DEG_PER_PAIR = {"L": 32.0, "R": 26.6}    # measured, L and R differ ~20%
TURN_TOL = 25                            # deg: approach() steers out the rest
MAX_TURN_TRIES = 4
PAUSE = 0.15  
def frame_angle(cx):
    """Angle of a point inside the frame, degrees, + = right of the image centre."""
    return math.degrees(math.atan((cx - FRAME_CX) / F_PX))


def scan_with_pan(target_id):
    """Robot stands still; look around with the camera only.
    Returns the marker's bearing in degrees from the robot's nose (+ = right), or None.
    The camera is always pointed straight ahead again on return (approach() assumes it)."""
    try:
        for pan in PAN_ANGLES:
            px.set_cam_pan_angle(PAN_SIGN * pan)
            time.sleep(PAN_SETTLE)                 # let the servo arrive
            m = look_stationary(target_id)         # settle, drop stale frame, detect
            if m is None:
                print(f"[scan] pan={pan:+d}: no marker {target_id}")
                continue
            cx, side = measure(m)
            bearing = pan + frame_angle(cx)
            print(f"[scan] pan={pan:+d}: marker {target_id} cx={cx:.0f} side={side:.0f} -> bearing {bearing:+.1f} deg")
            return bearing
        return None
    finally:
        px.set_cam_pan_angle(0)                  
        time.sleep(PAN_SETTLE)
def drive(speed, steer, duration):
    px.set_dir_servo_angle(steer)
    if speed > 0: px.forward(speed)
    else: px.backward(-speed)
    time.sleep(duration); px.stop(); time.sleep(PAUSE)    

def n_point_pairs(pairs, step_t, direction):  
    for p in range(pairs):
        drive(+SPEED, direction*MAX_STEER, step_t)
        drive(-SPEED, -direction*MAX_STEER, step_t)
    px.set_dir_servo_angle(0)

def turn_in_place(target_id):
    for attempt  in range(MAX_TURN_TRIES):
        bearing = scan_with_pan(target_id)
        if bearing is None:
            return  "not_found"
        if abs(bearing)  <= TURN_TOL:
            return "aligned"
        if bearing > 0:
            
            side, direction = "R", +1
        else:
           side, direction = "L", -1
        pairs = max(1, round(abs(bearing) / DEG_PER_PAIR[side]))
        n_point_pairs(pairs, STEP_T, direction)
    return  "need_turn"
HALF_TURN = 180    # deg: with the +-117 deg scan, two scans cover 360

def search_360(target_id):
    """Find the marker anywhere around: scan; if not found, turn ~180 deg and scan again.
    Returns the same statuses as turn_in_place."""
    turn = turn_in_place(target_id)
    if turn != "not_found":
        return turn
    pairs = round(HALF_TURN / DEG_PER_PAIR["L"])     # 180/32 = 5.6 -> 6 pairs
    print(f"[search] marker {target_id} not around - turning ~180 deg ({pairs} pairs left)")
    n_point_pairs(pairs, STEP_T, -1)
    return turn_in_place(target_id)

if __name__ == "__main__":
    target = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    try:
        print("result:", turn_in_place(target))
    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        px.set_cam_pan_angle(0)
        cam.stop()