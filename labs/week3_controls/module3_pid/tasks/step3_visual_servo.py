"""
MIT BWSI Autonomous Drone Racing Course - UAV Neo
GNU General Public License v3.0

Week 2/3 Lab — Step 3: Visual Servoing (Vision + PID)
Capstone: use a PID loop on the camera pixel error to keep a glowing
gate centered by yawing. Combines Week 2 vision with Week 3 control.
"""

import drone_core
import drone_utils as uav_utils
import cv2
import numpy as np

import os as _os, sys as _sys
_d = _os.path.dirname(_os.path.abspath(__file__))
while _os.path.basename(_d) != "labs" and _os.path.dirname(_d) != _d:
    _d = _os.path.dirname(_d)
if _d not in _sys.path:
    _sys.path.insert(0, _d)
import neo_lab

# -- Constants --------------------------------------------------------------
V_MIN = 200
MIN_AREA = 500
COL_CENTER = 320
KP = 0.35
KI = 0.0
KD = 0.2
MAX_YAW = 0.25
SEARCH_YAW = 0.2
CENTER_TOL = 0.15    # normalized error considered centered
HOLD_TIME = 1.0

# -- Module-level state -----------------------------------------------------
_err_int = 0.0
_prev_err = 0.0
_target_col = None
_hold = 0.0
_done = False

def pid_control(err, err_int, err_dot, kp, ki, kd):
    """Standard PID law: output = kp*err + ki*err_int + kd*err_dot."""
    ##################################
    #### START PUT CODE HERE #########
    output = 0.0  # YOUR CODE HERE (combine the three gain terms)
    ###### END PUT CODE HERE #########
    ##################################
    return output

def reset():
    global _err_int, _prev_err, _target_col, _hold, _done
    _err_int = 0.0
    _prev_err = 0.0
    _target_col = None
    _hold = 0.0
    _done = False


def update(drone):
    global _err_int, _prev_err, _target_col, _hold, _done
    if _done:
        return True
    ##################################
    #### START PUT CODE HERE #########

    # Vision gives the error, PID turns it into a yaw command. Track ONE gate so the
    # target does not jump between gates as you turn.
    # 1. best = neo_lab.gate_nearest_center(image, V_MIN, MIN_AREA) if _target_col is None
    #    else neo_lab.gate_nearest_to(image, _target_col, V_MIN, MIN_AREA)
    # 2. If best is None: spin to search (send_pcmd(0,0,SEARCH_YAW,0)), set _target_col=None,
    #    reset _err_int and _hold, return False.
    # 3. row, col = uav_utils.get_contour_center(best) ; _target_col = col
    # 4. error = (col - COL_CENTER) / COL_CENTER          # normalized pixel error
    # 5. Update _err_int (clamped), err_dot, _prev_err (see Step 1).
    # 6. yaw = uav_utils.clamp(pid_control(error, _err_int, err_dot, KP, KI, KD),
    #                          -MAX_YAW, MAX_YAW)
    # 7. send_pcmd(0, 0, yaw, 0); finish once centered (abs(error) < CENTER_TOL) for HOLD_TIME

    ###### END PUT CODE HERE #########
    ##################################
    return _done


if __name__ == "__main__":
    _drone = drone_core.create_drone()
    _launcher = neo_lab.Launcher(3.0)

    def start():
        _launcher.reset()
        reset()
        print("Step 3: Visual Servoing (Vision + PID)")

    def _update():
        if not _launcher.done:        # arm + climb to a safe height first
            _launcher.update(_drone)
            return
        if update(_drone):
            _drone.flight.land()

    _drone.set_start_update(start, _update)
    _drone.go()
