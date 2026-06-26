# Simulator Integration Notes

Notes from validating these labs against the real **UAVSim_Linux** build. Read this
before running on the simulator or tuning the control labs.

## How to run a lab against the sim

```bash
# 1. Launch the simulator (native Linux build)
~/projects/bwsi/uav-neo-installer/UAVNeo-Simulator/UAVSim_Linux_v0.0.1/UAVSim_Linux_v0.0.1.x86_64 &

# 2. Run a lab. The course labs are symlinked into the drone tool at labs/course,
#    so once the venv .pth points at the library you can use:
drone sim course/week3_controls/module2_feedback_control/main.py

#    ...or run directly with the venv python:
PYTHONPATH=.../drone-student/library \
  .../drone-venv/bin/python .../module2_feedback_control/main.py -s

# 3. In the UAVSim window: press ENTER to connect + enter user-program mode.
```

The Python script is the server; the simulator connects to it over UDP
(127.0.0.1:5064/5065). Pressing **Enter** in the sim window is required to start the
program — that step is manual.

## Bug found & fixed (committed to uav-neo-installer @ course-labs-integration)

`drone_core_sim.__resolve_sim_ip()` used the default-route **gateway**
unconditionally (a comment said "WSL2" but there was no WSL2 check). On native
Linux this resolved the sim to the LAN router (e.g. `192.168.1.1`) and the lab
hung at *"awaiting connection."* Fixed by guarding the gateway lookup behind an
actual `/proc/version` WSL2 check, falling back to `127.0.0.1`. You can also force
it with `DRONE_SIM_IP=127.0.0.1`.

## Sim flight model (measured)

These facts drive `labs/neo_lab.py` and the control-lab gains:

| Observation | Value / behavior |
|-------------|------------------|
| Spawn altitude (`get_altitude()`) | **~7 m**, not 0 — there is a ground offset |
| `takeoff()` alone | arms the motors but does **not** auto-climb |
| `send_pcmd(0,0,0,throttle)` | throttle is a **vertical velocity** command (~12 m/s per unit) |
| `stop()` (`throttle=0`) | engages hover-hold; altitude stays rock-steady |
| Throttle deadband | ~0.05 — commands below this are absorbed by hover-hold |

Consequences encoded in the labs:
- **Always launch via `neo_lab.Launcher`**: arm with `takeoff()`, then climb with a
  small-gain throttle controller to a height **above the measured ground**.
- **Altitude targets are heights above ground** (`neo_lab.height(drone)`), never
  absolute — the spawn offset varies and the drone keeps its altitude between runs.
- **Keep gains small** (`KP≈0.2`, clamp throttle). The deadband means a pure-P
  controller settles ~0.2–0.4 m short of target (a real proportional-control droop),
  so tolerances are widened and the PID lab's integral term closes the gap.

## The scene (captured live)

The forward camera sees dark gate frames with **cyan** glowing edges and AprilTag
fiducials over a blue wall / grey floor; the downward camera sees the gate frames as
**white** edges. There are **no red props and no colored ground line**, so the vision
labs detect gates by brightness (`neo_lab.bright_mask`) / cyan (`neo_lab.CYAN_LOWER/UPPER`).

## Validation status (all flown to completion in the sim)

- ✅ Connection / handshake (after the IP fix)
- ✅ `neo_lab.Launcher` — arm + climb to a height above the measured ground
- ✅ **Module 2** (P altitude hold) — launch + hold 5 m + setpoint sequence 3→6→2 m
- ✅ **Module 3 Step 1** (PID altitude) — holds 5 m within ~0.05 m
- ✅ **Module 3 Step 2** (fly-a-distance) — flies forward, holds altitude, completes on a
  settle-after-MIN_TRAVEL criterion. NOTE: distance is dead-reckoned from velocity (no
  position feedback) + pitch deadband, so it settles ~0.5–1 m short and reports the
  estimate honestly — a real lesson, not a bug.
- ✅ **Module 3 Step 3** (gate visual-servo) — "Locked onto the gate". Full Module 3 runs
  Step 1 → 2 → 3 → land end to end.
- ✅ **Module 4** (downward gate) — find contours → locate gate → center over it → land.
- ✅ **Module 5** (seek cyan gate) — mask → bounding box → search/center/approach →
  "Reached the gate".
- ✅ Week 2 Modules 2/3 (threshold/morphology, edge regression) complete on timers.
- ✅ All Week 2 vision step solutions also run against the **real captured frames**
  (offline harness) with 0 failures.

### Gate vision in a multi-gate field
Gates are detected by brightness/cyan and filtered to roughly-SQUARE contours
(`neo_lab.largest_gate` / `largest_cyan_gate`) to reject the elongated glowing boundary
lines. For yaw visual-servoing, selecting the *largest* gate flickers between several
similar gates, so Step 3 **tracks one gate** (`gate_nearest_to` seeded at the image
center) and uses a tolerant lock — this is what makes it latch reliably.

### Throttle / pitch deadband
Commands with magnitude below ~0.05 are absorbed by the hover-hold, so pure-proportional
loops settle a little short of target (widened tolerances accommodate this; the PID labs'
integral term closes most of the altitude gap).
