import time
from drive_hybrid import approach, px, cam, SPEED   # the import runs the hardware init once
from search import turn_in_place

ROUTE = [0, 1, 2]           # marker ids in visiting order
STATION_PAUSE = 2.0         # s to wait at each station (people scan goes here later)
BACK_OFF_T = 1.0            # s reversing away from the marker before turning
OBSTACLE_WAIT = 3.0         # s to wait before retrying after an obstacle
MAX_OBSTACLE_TRIES = 3      # obstacles in a row on the same leg -> "stuck"
MAX_LAPS = 2                # full laps of ROUTE before returning "done"


def back_off():
    """Reverse straight for BACK_OFF_T seconds, then stop."""
    px.set_dir_servo_angle(0)   # straight wheels so we reverse in a line
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
        result = approach(target)          # "arrived" / "lost" / "obstacle"

        if result == "arrived":
            obstacle_tries = 0
            time.sleep(STATION_PAUSE)
            back_off()
            i = (i + 1) % len(ROUTE)       # next station (wraps 2 -> 0)
            if i == 0:
                laps += 1
                if laps >= MAX_LAPS:       # finished: no need to turn toward the next one
                    break
            turn = turn_in_place(ROUTE[i]) # "aligned" / "not_found" / "need_turn"
            if turn != "aligned":
                print(f"[patrol] could not face marker {ROUTE[i]} ({turn}) - stopping")
                return "lost"

        elif result == "obstacle":
            obstacle_tries += 1
            if obstacle_tries >= MAX_OBSTACLE_TRIES:
                print(f"[patrol] blocked {obstacle_tries} times at marker {target} - giving up")
                return "stuck"
            print(f"[patrol] obstacle, waiting {OBSTACLE_WAIT}s (try {obstacle_tries})")
            time.sleep(OBSTACLE_WAIT)      # i unchanged -> retry the same marker

        else:  # "lost"
            print(f"[patrol] lost marker {target} - stopping")
            return "lost"