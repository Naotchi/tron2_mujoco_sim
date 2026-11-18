"""DACH motion loop: wave -> bow -> cheer, repeated until Ctrl+C.

  1. wave  : face up, right hand raised and waving
  2. bow   : head lowered, hands held together in front
  3. cheer : both hands raised overhead, bouncing

Run the simulator first, then this script in another terminal:
    ROBOT_TYPE=DACH_TRON2A uv run simulator.py
    uv run examples/dach_motion.py

Ctrl+C returns to the start pose before exiting. Like dach_hello.py, it talks to
the simulator through limxsdk (RobotCmd out, RobotState in), so only ROBOT_IP
changes on hardware.

Each motion is a plain function taking a Controller, built from three moves:
    ctrl.set_grippers(opening)          # 0 = closed, 100 = open
    ctrl.move_to(pose, duration)        # smooth blend to a pose
    ctrl.hold(pose, duration, osc=fn)   # stay at a pose, plus a faded oscillation
A pose is {joint: offset} on top of the start pose. To add a motion, write a
function like wave() below and add it to SEQUENCE.
"""
import math
import os
import threading
import time

from limxsdk import datatypes
from limxsdk.robot.Robot import Robot
from limxsdk.robot.RobotType import RobotType

# Wire order of the DACH channel (tron2_sim/spec.py: ARM_DA + HEAD_PY).
ARM_L = [
    "proximal_pitch_L_Joint", "proximal_roll_L_Joint", "proximal_yaw_L_Joint", "elbow_L_Joint",
    "wrist_yaw_L_Joint", "wrist_pitch_L_Joint", "wrist_roll_L_Joint",
]
ARM_R = [n.replace("_L_", "_R_") for n in ARM_L]
JOINTS = ARM_L + ARM_R + ["head_pitch_Joint", "head_yaw_Joint"]
N = len(JOINTS)
IDX = {name: i for i, name in enumerate(JOINTS)}

# Right arm joint = MIRROR * left arm joint for a mirror-symmetric pose
# (same convention as dach_ippon_jime.py).
MIRROR = [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0]

# PD gains: shoulders/elbow stiff (raised arms fight gravity), wrists/head soft.
KP = [150.0] * 4 + [20.0] * 3 + [150.0] * 4 + [20.0] * 3 + [20.0] * 2
KD = [6.0] * 4 + [1.0] * 3 + [6.0] * 4 + [1.0] * 3 + [1.0] * 2

RATE_HZ = 500
TRANS_S = 1.5      # blend from one motion's pose to the next
RETURN_S = 2.0     # back to the start pose on Ctrl+C
FADE_S = 0.6       # the oscillation fades in/out over this, so poses join smoothly

GRIP_OPEN = 100.0
GRIP_CLOSED = 0.0


def both_arms(pitch=0.0, roll=0.0, yaw=0.0, elbow=0.0, w_yaw=0.0, w_pitch=0.0, w_roll=0.0):
    """Mirror-symmetric offsets for both arms, given as LEFT-arm values.

    pitch < 0 swings the arm forward/up, roll > 0 moves the left arm outward,
    elbow < 0 bends the elbow.
    """
    left = [pitch, roll, yaw, elbow, w_yaw, w_pitch, w_roll]
    out = dict(zip(ARM_L, left))
    out.update({r: m * v for r, m, v in zip(ARM_R, MIRROR, left)})
    return out


def smooth(x):
    """0 -> 1 with zero slope at both ends."""
    x = min(1.0, max(0.0, x))
    return 0.5 * (1.0 - math.cos(math.pi * x))


class StateCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.q = None

    def on_state(self, s):
        with self.lock:
            self.q = list(s.q)


class Controller:
    """Owns the limxsdk link and the last commanded pose; motions call its moves."""

    def __init__(self, ip):
        self.robot = Robot(RobotType.Tron2)
        if not self.robot.init(ip):
            raise SystemExit(f"limxsdk init failed (robot_ip={ip})")
        self.cache = StateCache()
        self.robot.subscribeRobotState(self.cache.on_state)

        self.cmd = datatypes.RobotCmd()
        self.cmd.motor_names = JOINTS
        self.cmd.mode = [0] * N
        self.cmd.dq = [0.0] * N
        self.cmd.tau = [0.0] * N
        self.cmd.Kp = KP
        self.cmd.Kd = KD

        self.grip = datatypes.GripperCmd()
        self.grip.speed = [50.0, 50.0]
        self.grip.force = [50.0, 50.0]

        self.q0 = self._wait_for_state()
        self.q_last = list(self.q0)

    def _wait_for_state(self):
        # DACH publishes no state while the viewer is paused (Space), so this blocks then.
        print("waiting for RobotState from the simulator (unpause the viewer if paused) ...")
        while True:
            with self.cache.lock:
                q = self.cache.q
            if q is not None and len(q) == N:
                print("start pose:", [round(v, 3) for v in q])
                return q
            time.sleep(0.05)

    # -- low level ----------------------------------------------------------

    def send(self, q):
        self.q_last = list(q)
        self.cmd.q = self.q_last
        self.cmd.stamp = time.time_ns()
        self.robot.publishRobotCmd(self.cmd)

    def pose_q(self, pose):
        """Joint targets for a pose given as {joint: offset} from the start pose."""
        q = list(self.q0)
        for name, dq in pose.items():
            q[IDX[name]] += dq
        return q

    def _stream(self, duration, q_at):
        """Send q_at(t) at RATE_HZ for `duration` seconds."""
        t0 = time.monotonic()
        while (t := time.monotonic() - t0) < duration:
            self.send(q_at(t))
            time.sleep(1.0 / RATE_HZ)

    # -- moves used by the motions ------------------------------------------

    def set_grippers(self, opening):
        self.grip.opening = [opening, opening]
        self.grip.stamp = time.time_ns()
        self.robot.publishGripperCmd(self.grip)

    def move_to(self, pose, duration=TRANS_S):
        """Blend smoothly from the last commanded pose to `pose`."""
        q_from, q_to = list(self.q_last), self.pose_q(pose)

        def q_at(t):
            a = smooth(t / duration)
            return [f + a * (g - f) for f, g in zip(q_from, q_to)]

        self._stream(duration, q_at)

    def hold(self, pose, duration, osc=None):
        """Stay at `pose`; osc(t) -> {joint: offset} is faded in and out at the ends."""
        q_pose = self.pose_q(pose)

        def q_at(t):
            q = list(q_pose)
            if osc is not None:
                env = smooth(t / FADE_S) * smooth((duration - t) / FADE_S)
                for name, dq in osc(t).items():
                    q[IDX[name]] += env * dq
            return q

        self._stream(duration, q_at)

    def go_home(self, duration=RETURN_S):
        self.move_to({}, duration)
        self.send(self.q0)


# ---------------------------------------------------------------------------
# Motions. Head: pitch < 0 looks up, pitch > 0 looks down, yaw < 0 turns right.
# ---------------------------------------------------------------------------

def wave(ctrl):
    """Face up, right hand raised and waving."""
    pose = {
        "proximal_pitch_R_Joint": -0.8,   # upper arm lifted a little
        "proximal_roll_R_Joint": -0.4,    # out to the right
        "elbow_R_Joint": -2.3,            # forearm pointing straight up
        "head_pitch_Joint": -0.3,         # face up
        "head_yaw_Joint": -0.3,           # toward the right hand
    }

    def osc(t):
        swing = 0.5 * math.sin(2.0 * math.pi * t / 1.0)        # 1 s per wave
        return {
            "proximal_yaw_R_Joint": swing,
            "wrist_roll_R_Joint": 0.5 * swing,
            "head_yaw_Joint": 0.15 * math.sin(2.0 * math.pi * t / 2.0),
        }

    ctrl.set_grippers(GRIP_OPEN)
    ctrl.move_to(pose)
    ctrl.hold(pose, 5.0, osc)


def bow(ctrl):
    """Head lowered, hands held together (and closed) in front."""
    pose = {
        # Upper arms slightly forward and inward, elbows bent, so the hands
        # meet in front of the belly. (elbow > 0 here: -1.4 bent the arms
        # the wrong way in the simulator.)
        **both_arms(pitch=-1, roll=-0.2, elbow=1.4),
        "head_pitch_Joint": 0.5,          # head down
    }

    def osc(t):
        # One slow, deeper dip of the head in the middle of the hold.
        return {"head_pitch_Joint": 0.25}

    ctrl.set_grippers(GRIP_CLOSED)
    ctrl.move_to(pose)
    ctrl.hold(pose, 3.0, osc)


def cheer(ctrl):
    """Both hands raised overhead, bouncing."""
    pose = {
        **both_arms(pitch=-2.4, roll=0.4, elbow=-0.2),   # both arms up and out
        "head_pitch_Joint": -0.3,         # face up
    }

    def osc(t):
        bounce = 0.5 + 0.5 * math.sin(2.0 * math.pi * t / 0.6)   # quick "yay!" pumps
        return {
            "elbow_L_Joint": -0.35 * bounce,
            "elbow_R_Joint": -0.35 * bounce,
            "head_yaw_Joint": 0.2 * math.sin(2.0 * math.pi * t / 1.2),
        }

    ctrl.set_grippers(GRIP_OPEN)
    ctrl.move_to(pose)
    ctrl.hold(pose, 4.0, osc)


SEQUENCE = [wave, bow, cheer]


def main():
    ctrl = Controller(os.environ.get("ROBOT_IP", "127.0.0.1"))
    ctrl.set_grippers(GRIP_OPEN)
    try:
        i = 0
        while True:
            motion = SEQUENCE[i % len(SEQUENCE)]
            print(f"[{i + 1}] {motion.__name__}")
            motion(ctrl)
            i += 1
    except KeyboardInterrupt:
        print("returning to the start pose ...")
        try:
            ctrl.set_grippers(GRIP_OPEN)
            ctrl.go_home()
        except KeyboardInterrupt:
            pass
        print("stopped")


if __name__ == "__main__":
    main()