"""Minimal DACH controller: raises the right hand and waves "hello", looking at it.

Run the simulator first, then this script in another terminal:
    ROBOT_TYPE=DACH_TRON2A uv run simulator.py
    uv run examples/dach_hello.py

Ctrl+C lowers the arm back to the start pose before exiting.
It talks to the simulator through limxsdk exactly as a controller would talk to
the real robot (RobotCmd out, RobotState in), so only ROBOT_IP changes on hardware.
"""
import math
import os
import threading
import time

from limxsdk import datatypes
from limxsdk.robot.Robot import Robot
from limxsdk.robot.RobotType import RobotType

# Wire order of the DACH channel (tron2_sim/spec.py: ARM_DA + HEAD_PY).
JOINTS = [
    "proximal_pitch_L_Joint", "proximal_roll_L_Joint", "proximal_yaw_L_Joint", "elbow_L_Joint",
    "wrist_yaw_L_Joint", "wrist_pitch_L_Joint", "wrist_roll_L_Joint",
    "proximal_pitch_R_Joint", "proximal_roll_R_Joint", "proximal_yaw_R_Joint", "elbow_R_Joint",
    "wrist_yaw_R_Joint", "wrist_pitch_R_Joint", "wrist_roll_R_Joint",
    "head_pitch_Joint", "head_yaw_Joint",
]
N = len(JOINTS)
IDX = {name: i for i, name in enumerate(JOINTS)}

# PD gains: shoulders/elbow stiff (the raised arm fights gravity), wrists/head soft.
KP = [150.0] * 4 + [20.0] * 3 + [150.0] * 4 + [20.0] * 3 + [20.0] * 2
KD = [6.0] * 4 + [1.0] * 3 + [6.0] * 4 + [1.0] * 3 + [1.0] * 2

RATE_HZ = 500
RAISE_S = 2.0      # start pose -> raised hand
LOWER_S = 2.0      # raised hand -> start pose (on Ctrl+C)
WAVE_PERIOD_S = 1.0
WAVE_AMP = 0.5     # rad, upper-arm yaw swing that sways the forearm left/right
# DACH's head has only pitch + yaw (no roll), so the head "sways" by swinging yaw
# left/right around the look-at-hand offset, slower than the hand wave.
HEAD_PERIOD_S = 2.0
HEAD_AMP = 0.25    # rad

# Raised-hand pose, as offsets on top of the start pose. Pitch < 0 swings the
# arm forward, elbow < 0 bends it, roll < 0 moves the right arm outward. The
# upper arm is lifted only slightly (pitch ~0.8 rad); the elbow bends ~2.3 rad so
# the forearm ends up at pitch + elbow ~= pi, i.e. pointing straight up.
RAISED = {
    "proximal_pitch_R_Joint": -0.8,
    "proximal_roll_R_Joint": -0.4,
    "elbow_R_Joint": -2.3,
    "head_yaw_Joint": 0.3,      # look toward the right hand
    "head_pitch_Joint": 0.3,    # tilt the face up a little
}


class StateCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.q = None

    def on_state(self, s):
        with self.lock:
            self.q = list(s.q)


def smooth(x):
    """0 -> 1 with zero slope at both ends."""
    x = min(1.0, max(0.0, x))
    return 0.5 * (1.0 - math.cos(math.pi * x))


def target(q0, raise_amount, wave_t):
    """Start pose q0 blended toward the raised pose, plus the wave."""
    q = list(q0)
    for name, dq in RAISED.items():
        q[IDX[name]] += raise_amount * dq
    if wave_t > 0.0:
        # Fade the wave in over one period so it starts without a jerk.
        fade = smooth(wave_t / WAVE_PERIOD_S)
        swing = WAVE_AMP * fade * math.sin(2.0 * math.pi * wave_t / WAVE_PERIOD_S)
        q[IDX["proximal_yaw_R_Joint"]] += raise_amount * swing
        q[IDX["wrist_roll_R_Joint"]] += raise_amount * 0.5 * swing   # flick the hand too
        head_fade = smooth(wave_t / HEAD_PERIOD_S)
        q[IDX["head_yaw_Joint"]] += raise_amount * HEAD_AMP * head_fade * math.sin(
            2.0 * math.pi * wave_t / HEAD_PERIOD_S)
    return q


def main():
    robot = Robot(RobotType.Tron2)
    ip = os.environ.get("ROBOT_IP", "127.0.0.1")
    if not robot.init(ip):
        raise SystemExit(f"limxsdk init failed (robot_ip={ip})")

    cache = StateCache()
    robot.subscribeRobotState(cache.on_state)

    # DACH publishes no state while the viewer is paused (Space), so this blocks then.
    print("waiting for RobotState from the simulator (unpause the viewer if paused) ...")
    while True:
        with cache.lock:
            q0 = cache.q
        if q0 is not None and len(q0) == N:
            break
        time.sleep(0.05)
    print("start pose:", [round(v, 3) for v in q0])

    cmd = datatypes.RobotCmd()
    cmd.motor_names = JOINTS
    cmd.mode = [0] * N
    cmd.dq = [0.0] * N
    cmd.tau = [0.0] * N
    cmd.Kp = KP
    cmd.Kd = KD

    # Open both hands (100 = open), as in a friendly wave.
    grip = datatypes.GripperCmd()
    grip.speed = [50.0, 50.0]
    grip.force = [50.0, 50.0]
    grip.opening = [100.0, 100.0]
    grip.stamp = time.time_ns()
    robot.publishGripperCmd(grip)

    def send(q):
        cmd.q = q
        cmd.stamp = time.time_ns()
        robot.publishRobotCmd(cmd)

    t_start = time.monotonic()
    next_print = 0.0
    q_last = list(q0)
    try:
        while True:
            t = time.monotonic() - t_start
            q_last = target(q0, smooth(t / RAISE_S), t - RAISE_S)
            send(q_last)
            if t >= next_print:
                next_print += 1.0
                with cache.lock:
                    q = cache.q
                print(f"t={t:5.1f}s  elbow_R={q[IDX['elbow_R_Joint']]:+.3f}  "
                      f"yaw_R={q[IDX['proximal_yaw_R_Joint']]:+.3f}")
            time.sleep(1.0 / RATE_HZ)
    except KeyboardInterrupt:
        print("lowering the arm ...")
        try:
            t0 = time.monotonic()
            while (t := time.monotonic() - t0) < LOWER_S:
                a = smooth(t / LOWER_S)
                send([l + a * (s - l) for l, s in zip(q_last, q0)])
                time.sleep(1.0 / RATE_HZ)
            send(list(q0))
        except KeyboardInterrupt:
            pass
        print("stopped")


if __name__ == "__main__":
    main()
