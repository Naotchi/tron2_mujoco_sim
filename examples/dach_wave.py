"""Minimal DACH controller: waves both arms, nods the head, opens/closes the 2F gripper.

Run the simulator first, then this script in another terminal:
    ROBOT_TYPE=DACH_TRON2A uv run simulator.py
    uv run examples/dach_wave.py

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

# PD gains: shoulders/elbow stiff, wrists/head soft.
KP = [100.0] * 4 + [20.0] * 3 + [100.0] * 4 + [20.0] * 3 + [20.0] * 2
KD = [5.0] * 4 + [1.0] * 3 + [5.0] * 4 + [1.0] * 3 + [1.0] * 2

RATE_HZ = 500
RAMP_S = 2.0       # blend from the measured pose to the motion over this time
PERIOD_S = 4.0     # one wave cycle


class StateCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.q = None
        self.gripper = None

    def on_state(self, s):
        with self.lock:
            self.q = list(s.q)

    def on_gripper(self, g):
        with self.lock:
            self.gripper = list(g.q)


def target(q0, t):
    """Motion offsets on top of the start pose q0."""
    q = list(q0)
    phase = 2.0 * math.pi * t / PERIOD_S
    bend = 0.5 * (1.0 - math.cos(phase))          # 0 -> 1 -> 0, never negative
    q[IDX["proximal_roll_L_Joint"]] += 0.6 * bend   # lift arms sideways
    q[IDX["proximal_roll_R_Joint"]] -= 0.6 * bend
    q[IDX["elbow_L_Joint"]] -= 1.0 * bend         # elbow bends toward negative
    q[IDX["elbow_R_Joint"]] -= 1.0 * bend
    q[IDX["head_yaw_Joint"]] += 0.5 * math.sin(phase)
    q[IDX["head_pitch_Joint"]] += 0.2 * math.sin(2.0 * phase)
    return q


def main():
    robot = Robot(RobotType.Tron2)
    ip = os.environ.get("ROBOT_IP", "127.0.0.1")
    if not robot.init(ip):
        raise SystemExit(f"limxsdk init failed (robot_ip={ip})")

    cache = StateCache()
    robot.subscribeRobotState(cache.on_state)
    robot.subscribeGripperState(cache.on_gripper)

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

    grip = datatypes.GripperCmd()
    grip.speed = [50.0, 50.0]
    grip.force = [50.0, 50.0]

    t_start = time.monotonic()
    next_print = 0.0
    last_opening = None
    try:
        while True:
            t = time.monotonic() - t_start
            alpha = min(1.0, t / RAMP_S)
            goal = target(q0, max(0.0, t - RAMP_S))
            cmd.q = [a + alpha * (g - a) for a, g in zip(q0, goal)]
            cmd.stamp = time.time_ns()
            robot.publishRobotCmd(cmd)

            # Gripper: open for half a cycle, close for the other half. Sent only on
            # change, since the SDK logs every publishGripperCmd call.
            opening = 100.0 if int(t / (PERIOD_S / 2)) % 2 == 0 else 0.0
            if opening != last_opening:
                last_opening = opening
                grip.opening = [opening, opening]
                grip.stamp = time.time_ns()
                robot.publishGripperCmd(grip)

            if t >= next_print:
                next_print += 1.0
                with cache.lock:
                    q, g = cache.q, cache.gripper
                print(f"t={t:5.1f}s  elbow_L={q[IDX['elbow_L_Joint']]:+.3f}  "
                      f"head_yaw={q[IDX['head_yaw_Joint']]:+.3f}  gripper_q={g}")
            time.sleep(1.0 / RATE_HZ)
    except KeyboardInterrupt:
        print("stopped")


if __name__ == "__main__":
    main()
