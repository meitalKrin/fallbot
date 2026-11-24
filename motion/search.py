## motion/search.py - find a marker without moving the robot: sweep the camera pan servo
## Test on the robot:  sudo python3 search.py 1      (looks for marker 1 and prints its bearing)
##
## Math (see math doc):
##   camera field of view ≈ 54°  →  each look covers pan ± 27°
##   pan steps of 45° < 54°      →  neighbouring looks overlap, no blind gaps
##   sweep -90..+90 at ±27°      →  covers about ±117° without moving the robot
##   marker angle inside a frame:  theta_frame = atan((cx - 320) / f),  f = k / L = 5834 / 10 ≈ 583 px
##   bearing from the robot's nose:  bearing = pan + theta_frame      (+ = right, like steering)
import sys
import time
import math
from drive_hybrid import look_stationary, measure, px, cam, FRAME_CX

PAN_ANGLES = [0, -45, 45, -90, 90]  # deg; start straight ahead (most likely), then widen
PAN_SETTLE = 0.3                    # s for the pan servo to reach its angle - to measure
F_PX = 583                          # focal length in px = k / L = 5834 / 10
PAN_SIGN = 1                        # +1 if px.set_cam_pan_angle(+45) turns the camera RIGHT, else -1 (check once!)


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
        px.set_cam_pan_angle(0)                    # camera straight ahead, whatever happened
        time.sleep(PAN_SETTLE)


if __name__ == "__main__":
    target = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    try:
        print("bearing:", scan_with_pan(target))
    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        px.set_cam_pan_angle(0)
        cam.stop()