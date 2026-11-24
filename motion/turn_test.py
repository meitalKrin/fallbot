## motion/turn_test.py - measure how many degrees one N-point pair (forward + backward) turns the robot
## The robot measures its own rotation with a marker: bearing before vs. bearing after.
##
## Usage (robot ~1 m in front of marker 0, facing it, free space around):
##   sudo python3 turn_test.py <pairs> <step_t> <L|R>
##   e.g.  sudo python3 turn_test.py 3 0.3 L
## Results are appended to turn_test.csv
##
## Math:
##   the marker does not move, so if the robot turns LEFT by d, the marker appears d further RIGHT:
##       d = bearing_after - bearing_before          (+ = robot turned left, - = turned right)
##   degrees per pair = |d| / pairs
##   one pair: forward with steer one way, then backward with steer the OTHER way
##   -> both halves rotate the robot the same direction (Ackermann N-point turn)
import sys
import os
import csv
import time
from drive_hybrid import px, cam, SPEED, MAX_STEER
from search import scan_with_pan

TARGET = 0          # marker used as the reference
PAUSE = 0.15        # s between moves, so the robot doesn't jerk from forward straight into reverse


def drive(speed, steer, duration):
    """One move: set steering, drive forward (speed > 0) or backward (speed < 0), stop."""
    px.set_dir_servo_angle(steer)
    if speed > 0:
        px.forward(speed)
    else:
        px.backward(-speed)
    time.sleep(duration)
    px.stop()
    time.sleep(PAUSE)


def n_point_pairs(pairs, step_t, direction):
    """direction: -1 = turn left, +1 = turn right (same sign as steering)."""
    for p in range(pairs):
        drive(+SPEED, direction * MAX_STEER, step_t)      # forward, steer toward the turn
        drive(-SPEED, -direction * MAX_STEER, step_t)     # backward, steer the OTHER way
        print(f"[turn] pair {p + 1}/{pairs} done")
    px.set_dir_servo_angle(0)


if __name__ == "__main__":
    pairs = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    step_t = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    side = sys.argv[3].upper() if len(sys.argv) > 3 else "L"
    direction = -1 if side == "L" else +1

    try:
        b0 = scan_with_pan(TARGET)
        if b0 is None:
            print(f"marker {TARGET} not visible at the start - place the robot facing it")
            sys.exit(1)

        n_point_pairs(pairs, step_t, direction)

        b1 = scan_with_pan(TARGET)
        if b1 is None:
            print("marker lost after turning: turned past ~117 deg, use fewer pairs or a shorter step_t")
            sys.exit(1)

        d = b1 - b0
        per_pair = abs(d) / pairs
        print(f"\nbearing before {b0:+.1f}, after {b1:+.1f}  ->  turned {abs(d):.1f} deg "
              f"{'left' if d > 0 else 'right'}  ->  {per_pair:.1f} deg per pair "
              f"(step_t={step_t}s, {pairs} pairs, asked {side})")

        new = not os.path.exists("turn_test.csv")
        with open("turn_test.csv", "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["side", "pairs", "step_t_s", "bearing_before", "bearing_after", "turned_deg", "deg_per_pair"])
            w.writerow([side, pairs, step_t, round(b0, 1), round(b1, 1), round(d, 1), round(per_pair, 1)])
    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        px.set_cam_pan_angle(0)
        cam.stop()
