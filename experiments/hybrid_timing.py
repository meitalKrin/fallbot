## hybrid_timing.py - same behaviour as motion/drive_hybrid.py, plus timing + CSV log.
## Usage (on the robot):  sudo python3 hybrid_timing.py <label> [pulse]
##   e.g.  sudo python3 hybrid_timing.py 2m_a          (normal hybrid)
##         sudo python3 hybrid_timing.py p150_a pulse  (pulse only, never switches to continuous)
## Output: timing_<label>.csv (one row per loop) + a line appended to timing_summary.csv

import sys, csv, os, time, statistics as st
import cv2
import numpy as np
from picamera2 import Picamera2
from picarx import Picarx

##constants (same values as drive_hybrid.py)
TARGET_ID = 0
STOP_SIDE = 160
MAX_STEER = 30
KP = 0.08
SPEED = 10
OBSTACLE_CM = 25
PULSE = 0.4
SETTLE = 0.2
FRAME_CX = 320
LOST_PULSE = 3
LOST_CONT = 15
NEAR_SIDE = 38
FAR_SIDE = 30
K_DIST, B_DIST = 5736, 17.5      # Z = K/s + B (math doc, section 4)
assert FAR_SIDE < NEAR_SIDE < STOP_SIDE, "thresholds out of order"

LABEL = sys.argv[1] if len(sys.argv) > 1 else time.strftime("%H%M%S")
PULSE_ONLY = len(sys.argv) > 2 and sys.argv[2] == "pulse"

##hardware init
px = Picarx()
px.set_cam_pan_angle(0); px.set_cam_tilt_angle(0); px.set_dir_servo_angle(0)
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
cam.start()
time.sleep(2)
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                              cv2.aruco.DetectorParameters())

def ms(a, b):
    return round((b - a) * 1000, 2)

def Z(side):
    return K_DIST / side + B_DIST

def capture():  # one frame + its metadata (exposure, frame duration) from the SAME request
    req = cam.capture_request()
    try:
        frame = req.make_array("main")
        md = req.get_metadata()
    finally:
        req.release()
    return frame, md

def find_marker(gray):
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None:
        return None
    ids = np.asarray(ids).flatten()      # (N,1) in OpenCV 4.x, (N,) in some versions
    for k in range(len(ids)):
        if ids[k] == TARGET_ID:
            return corners[k][0]
    return None

def measure(m):
    cx = m[:, 0].mean()
    side = np.mean([np.linalg.norm(m[k] - m[(k + 1) % 4]) for k in range(4)])
    return cx, side

def steer_towards(cx):
    steer = float(np.clip(KP * (cx - FRAME_CX), -MAX_STEER, MAX_STEER))
    px.set_dir_servo_angle(steer)
    return steer

def obstacle_confirmed():
    hits = 0
    for _ in range(2):
        d = px.ultrasonic.read()
        hits += (0 < d < OBSTACLE_CM)
        time.sleep(0.02)
    return hits == 2

def soft_start():
    for s in range(0, SPEED + 1, 2):
        px.forward(s); time.sleep(0.05)

FIELDS = ["t_ms", "mode", "event", "side", "cx", "z_cm", "settle_ms", "cap_ms", "cv_ms",
          "us_ms", "act_ms", "drive_ms", "loop_ms", "exp_us", "fdur_us", "d_cm"]
rows = []
T0 = time.perf_counter()

def log(**kw):
    r = {f: "" for f in FIELDS}
    r.update(kw)
    r["t_ms"] = ms(T0, time.perf_counter())
    rows.append(r)
    print(" ".join(f"{k}={v}" for k, v in r.items() if v != ""))

mode = "pulse"
lost = 0
t_prev = None
result = "?"
try:
    while True:
        if mode == "pulse":
            ta = time.perf_counter()
            time.sleep(SETTLE)
            tb = time.perf_counter()
            capture()                                   # drop one stale frame
            frame, md = capture()
            tc = time.perf_counter()
            m = find_marker(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            td = time.perf_counter()
            base = dict(mode="pulse", settle_ms=ms(ta, tb), cap_ms=ms(tb, tc), cv_ms=ms(tc, td),
                        exp_us=md.get("ExposureTime", ""), fdur_us=md.get("FrameDuration", ""))
            if m is None:
                lost += 1
                log(event="miss", loop_ms=ms(ta, time.perf_counter()), **base)
                if lost >= LOST_PULSE:
                    result = "lost"; break
                continue
            lost = 0
            cx, side = measure(m)
            steer_towards(cx)
            base.update(side=round(float(side), 1), cx=round(float(cx), 1), z_cm=round(float(Z(side)), 1))
            if side >= STOP_SIDE:
                px.stop(); log(event="arrived", **base); result = "arrived"; break
            te = time.perf_counter()
            obst = obstacle_confirmed()
            tf = time.perf_counter()
            base["us_ms"] = ms(te, tf)
            if obst:
                px.stop(); log(event="obstacle", **base); result = "obstacle"; break
            if side >= NEAR_SIDE and not PULSE_ONLY:
                mode = "continuous"; lost = 0; t_prev = None
                log(event="switch_to_cont", loop_ms=ms(ta, tf), **base)
                soft_start()
                continue
            px.forward(SPEED); time.sleep(PULSE); px.stop()
            tg = time.perf_counter()
            log(event="pulse", drive_ms=ms(tf, tg), loop_ms=ms(ta, tg), **base)

        else:
            t0 = time.perf_counter()
            loop = ms(t_prev, t0) if t_prev is not None else ""
            t_prev = t0
            frame, md = capture()
            t1 = time.perf_counter()
            m = find_marker(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            if m is not None:
                cx, side = measure(m)
            t2 = time.perf_counter()
            base = dict(mode="cont", cap_ms=ms(t0, t1), cv_ms=ms(t1, t2), loop_ms=loop,
                        exp_us=md.get("ExposureTime", ""), fdur_us=md.get("FrameDuration", ""))
            if m is None:
                lost += 1
                log(event="miss", **base)
                if lost >= LOST_CONT:
                    px.stop(); mode = "pulse"; lost = 0; t_prev = None
                    log(event="lost_to_pulse", mode="cont")
                continue
            lost = 0
            steer_towards(cx)
            px.forward(SPEED)
            t3 = time.perf_counter()
            d = px.ultrasonic.read()
            t4 = time.perf_counter()
            base.update(side=round(float(side), 1), cx=round(float(cx), 1), z_cm=round(float(Z(side)), 1),
                        act_ms=ms(t2, t3), us_ms=ms(t3, t4), d_cm=d)
            if side >= STOP_SIDE:
                px.stop(); log(event="arrived", **base); result = "arrived"; break
            if 0 < d < OBSTACLE_CM:
                px.stop(); log(event="obstacle", **base); result = "obstacle"; break
            if side < FAR_SIDE:
                px.stop(); mode = "pulse"; t_prev = None
                log(event="far_to_pulse", **base)
                continue
            log(event="frame", **base)

except KeyboardInterrupt:
    result = "ctrl-c"
finally:
    px.stop(); px.set_dir_servo_angle(0); cam.stop()

## ---------- save + summary ----------
with open(f"timing_{LABEL}.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=FIELDS); w.writeheader(); w.writerows(rows)

def col(rs, key):
    return [r[key] for r in rs if r[key] != ""]

def stat(xs):
    if not xs:
        return "-"
    return f"median {st.median(xs):.1f}  min {min(xs):.1f}  max {max(xs):.1f}  (n={len(xs)})"

steps = []
pulses = [r for r in rows if r["mode"] == "pulse" and r["event"] == "pulse"]
conts = [r for r in rows if r["mode"] == "cont" and r["event"] in ("frame", "arrived", "far_to_pulse", "obstacle", "miss")]
print("\n===== SUMMARY", LABEL, "pulse-only" if PULSE_ONLY else "hybrid", "result:", result, "=====")
print("PULSE cycle (ms)")
for k in ("settle_ms", "cap_ms", "cv_ms", "us_ms", "drive_ms", "loop_ms"):
    print(f"  {k:9s}", stat(col(pulses, k)))
if pulses:
    duty = st.mean(col(pulses, "drive_ms")) / st.mean(col(pulses, "loop_ms"))
    print(f"  driving fraction {duty*100:.0f}%")
    looks = [r for r in rows if r["mode"] == "pulse" and r["z_cm"] != ""]
    steps = [round(looks[i]["z_cm"] - looks[i + 1]["z_cm"], 1) for i in range(len(looks) - 1)
             if looks[i]["event"] == "pulse"]
    print("  step per pulse (cm):", steps, "->", stat(steps))
print("CONTINUOUS frame (ms)")
for k in ("cap_ms", "cv_ms", "act_ms", "us_ms", "loop_ms"):
    print(f"  {k:9s}", stat(col(conts, k)))
print("  misses:", sum(r["event"] == "miss" for r in conts), "of", len(conts), "frames")
print("CAMERA")
print("  exposure_us   ", stat(col(rows, "exp_us")))
print("  frame_dur_us  ", stat(col(rows, "fdur_us")))
sw = [r for r in rows if r["event"] == "switch_to_cont"]
end = rows[-1] if rows else {}
print("SWITCH side/Z:", (sw[0]["side"], sw[0]["z_cm"]) if sw else "-",
      " END:", end.get("event"), end.get("side"), end.get("z_cm"))

try:
    tape = input("\nTape-measured distance camera->marker at stop (cm), Enter to skip: ").strip()
except EOFError:
    tape = ""
new = not os.path.exists("timing_summary.csv")
with open("timing_summary.csv", "a", newline="") as f:
    w = csv.writer(f)
    if new:
        w.writerow(["label", "result", "switch_side", "switch_z", "end_side", "end_z", "tape_cm",
                    "pulse_loop_med_ms", "pulse_step_med_cm", "cont_loop_med_ms", "cont_misses", "cont_frames"])
    w.writerow([LABEL, result,
                sw[0]["side"] if sw else "", sw[0]["z_cm"] if sw else "",
                end.get("side", ""), end.get("z_cm", ""), tape,
                round(st.median(col(pulses, "loop_ms")), 1) if pulses else "",
                round(st.median(steps), 1) if pulses and steps else "",
                round(st.median(col(conts, "loop_ms")), 1) if col(conts, "loop_ms") else "",
                sum(r["event"] == "miss" for r in conts), len(conts)])
print(f"saved timing_{LABEL}.csv and timing_summary.csv")
