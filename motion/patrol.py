
import time
from drive_hybrid import approach, px, cam, SPEED   # the import runs the hardware init once

ROUTE = [0, 1, 2]     
STATION_PAUSE = 2.0     
BACK_OFF_T = 1.0      
OBSTACLE_WAIT = 3.0   
MAX_OBSTACLE_TRIES = 3
MAX_LAPS = 2       


def back_off():
    px.set_dir_servo_angle(0)   
    px.backward(SPEED)
    time.sleep(BACK_OFF_T)
    px.stop()                


def patrol():
    """Drive along ROUTE. Returns "done", "lost" or "stuck". Motors are stopped on return."""
    i = 0
    laps = 0
    obstacle_tries = 0
    while laps < MAX_LAPS:
        target = ROUTE[i]
        print(f"[patrol] -> marker {target}")
        result = approach(target)

        if result == "arrived":
            obstacle_tries = 0
            time.sleep(STATION_PAUSE)
            back_off()
            i = (i + 1) % len(ROUTE)   
            if i == 0:
                laps += 1

        elif result == "obstacle":
            obstacle_tries += 1
            if obstacle_tries >= MAX_OBSTACLE_TRIES:
                print(f"[patrol] blocked {obstacle_tries} times at marker {target} - giving up")
                return "stuck"
            print(f"[patrol] obstacle, waiting {OBSTACLE_WAIT}s (try {obstacle_tries})")
            time.sleep(OBSTACLE_WAIT)   

        else:  # "lost"
            print(f"[patrol] lost marker {target} - stopping (search comes later)")
            return "lost"

    return "done"


if __name__ == "__main__":
    try:
        print("patrol result:", patrol())
    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        cam.stop()