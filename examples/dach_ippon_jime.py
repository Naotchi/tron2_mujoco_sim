"""DACH "dynamic loop" ippon-jime: cross both hands in front, sweep each hand in a big
circle (up -> out -> down) like the NTT logo, and clap at the bottom. Loops forever.

Run the simulator first (unpaused), then this script in another terminal:
    ROBOT_TYPE=DACH_TRON2A uv run simulator.py
    ROBOT_TYPE=DACH_TRON2A uv run examples/dach_ippon_jime.py

Unlike dach_wave.py, the motion is defined as hand paths in Cartesian space. The
script loads the same MJCF as the simulator and solves IK offline at startup, then
streams joint targets with velocity and gravity feedforward over limxsdk.
"""
import math
import os
import threading
import time

import mujoco
import numpy as np
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
ARM = {"L": JOINTS[0:7], "R": JOINTS[7:14]}
HAND_BODY = {"L": "grasper_base_L_Link", "R": "grasper_base_R_Link"}
HAND_OFFSET = np.array([0.0, 0.0, -0.08])   # hand point: between the gripper jaws

# Right arm joint = MIRROR * left arm joint for a mirror-symmetric pose.
MIRROR = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
# Rest posture the IK null space pulls toward: upper arms slightly forward and
# out, elbows bent (pitch < 0 swings the arm forward, elbow < 0 bends it).
_Q_NOM_L = np.array([-0.8, 0.3, 0.0, -1.2, 0.0, 0.0, 0.0])
Q_NOMINAL = np.concatenate([_Q_NOM_L, MIRROR * _Q_NOM_L, [0.0, 0.0]])

KP = np.array([150.0] * 4 + [30.0] * 3 + [150.0] * 4 + [30.0] * 3 + [20.0] * 2)
KD = np.array([6.0] * 4 + [1.0] * 3 + [6.0] * 4 + [1.0] * 3 + [1.0] * 2)

# Loop geometry, in the base_Link frame (x forward, y left, z up), metres.
PLANE_X = 0.42      # distance of the drawing plane in front of the robot
CENTER_Y = 0.14     # each hand's circle centre, mirrored for the right hand
CENTER_Z = -0.10    # relative to base_Link (shoulders sit at about +0.05)
RADIUS = 0.26
THETA_START = math.radians(135)    # upper-inner: hands crossed in front
CLAP_Y = 0.035      # hand points this far from the midline at the clap; the closed
                    # grippers are ~8 cm wide, so they press about 1 cm into each other
THETA_CLAP = -math.acos((CLAP_Y - CENTER_Y) / RADIUS)   # lower-inner
CROSS_DX = 0.10     # front/back and up/down offsets while crossed, so the
CROSS_DZ = -0.04    # forearms pass each other instead of colliding

# Timeline of one cycle, seconds.
T_TO_START = 2.0    # first cycle only: current pose -> crossed pose
T_HOLD = 1.0        # "iyo-o!" hold in the crossed pose
T_LOOP = 1.3        # the big circle, ending at the clap
T_CLAP = 0.6        # stay clapped
T_BACK = 1.6        # return to the crossed pose

RATE_HZ = 500
IK_HZ = 200         # trajectory sampling rate; the stream interpolates between samples


def robot_xml():
    robot_type = os.environ.get("ROBOT_TYPE", "DACH_TRON2A")
    family = robot_type.split("_")[-1].lower()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(root, "robot-description", family, robot_type, "xml", "robot_grasper.xml")


class Kinematics:
    def __init__(self, path):
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)
        m = self.m
        self.qadr = np.array([m.jnt_qposadr[m.joint(n).id] for n in JOINTS])
        self.vadr = np.array([m.jnt_dofadr[m.joint(n).id] for n in JOINTS])
        self.lo = np.array([m.jnt_range[m.joint(n).id][0] for n in JOINTS])
        self.hi = np.array([m.jnt_range[m.joint(n).id][1] for n in JOINTS])
        base = m.body("base_Link").id
        self.base_pos = self.d.xpos[base].copy()
        self.base_rot = self.d.xmat[base].reshape(3, 3).copy()
        # Desired hand orientation: the zero-pose hand (pointing down) pitched to
        # point forward, so the jaws open sideways and the grippers meet flat.
        ry = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])  # R_y(-90 deg)
        self.hand_rot = self.base_rot @ ry

    def to_world(self, p_base):
        return self.base_pos + self.base_rot @ p_base

    def set_q(self, q):
        self.d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)

    def hand(self, side):
        b = self.m.body(HAND_BODY[side]).id
        rot = self.d.xmat[b].reshape(3, 3)
        return self.d.xpos[b] + rot @ HAND_OFFSET, rot, b

    def ik(self, q, targets, iters=50, w_rot=0.05, damping=1e-2, w_post=0.1, max_step=0.05):
        """Damped least squares on both arms. targets: {side: world position}.

        The hand position is the main task; orientation is weighted down, and the
        null space pulls toward Q_NOMINAL so the solution never flips elbow/wrist
        configuration between neighbouring samples.
        """
        q = q.copy()
        jp = np.zeros((3, self.m.nv))
        jr = np.zeros((3, self.m.nv))
        for _ in range(iters):
            self.set_q(q)
            for side, p_des in targets.items():
                p, rot, b = self.hand(side)
                mujoco.mj_jac(self.m, self.d, jp, jr, p, b)
                cols = [JOINTS.index(n) for n in ARM[side]]
                J = np.vstack([jp[:, self.vadr[cols]], w_rot * jr[:, self.vadr[cols]]])
                r_err = rot_error(self.hand_rot, rot)
                err = np.concatenate([p_des - p, w_rot * r_err])
                J_pinv = J.T @ np.linalg.inv(J @ J.T + damping * np.eye(6))
                null = np.eye(7) - J_pinv @ J
                dq = J_pinv @ err + null @ (w_post * (Q_NOMINAL[cols] - q[cols]))
                dq = np.clip(dq, -max_step, max_step)
                q[cols] = np.clip(q[cols] + dq, self.lo[cols], self.hi[cols])
        self.set_q(q)
        return q

    def pos_error(self, targets):
        return max(np.linalg.norm(self.hand(s)[0] - p) for s, p in targets.items())

    def gravity(self, q):
        """Joint torques that hold q against gravity (qfrc_bias at zero velocity)."""
        self.d.qpos[self.qadr] = q
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)
        return self.d.qfrc_bias[self.vadr].copy()


def rot_error(r_des, r):
    """Rotation vector taking r to r_des, in the world frame."""
    e = r_des @ r.T
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, e.flatten())
    vel = np.zeros(3)
    mujoco.mju_quat2Vel(vel, quat, 1.0)
    return vel


def hand_point(kin, side, y, z, cross):
    """Hand point from left-side coordinates (y mirrored for the right hand).
    cross in [0,1] adds the front/back and up/down offsets of the crossed pose."""
    s = 1.0 if side == "L" else -1.0
    x = PLANE_X + s * CROSS_DX * cross        # left hand in front and below while crossed
    return kin.to_world(np.array([x, s * y, z + s * CROSS_DZ * cross]))


def hand_target(kin, side, theta, cross):
    """Point on the side's circle."""
    return hand_point(kin, side, CENTER_Y + RADIUS * math.cos(theta),
                      CENTER_Z + RADIUS * math.sin(theta), cross)


def loop_theta(u):
    """Angle along the loop for u in [0,1]: eases in, still moving at the clap."""
    s = u * u * (2.0 - u)
    return THETA_START + (THETA_CLAP - THETA_START) * s


def plan(kin):
    """IK for the crossed pose, the loop and the way back. Returns joint samples."""
    def targets(theta, cross):
        return {side: hand_target(kin, side, theta, cross) for side in "LR"}

    # Converge on the crossed pose gradually from the zero pose (big jumps make
    # damped least squares land in odd elbow configurations).
    q = Q_NOMINAL.copy()
    start = targets(THETA_START, 1.0)
    kin.set_q(q)
    p0 = {s: kin.hand(s)[0].copy() for s in "LR"}
    for k in range(1, 41):
        a = k / 40
        q = kin.ik(q, {s: (1 - a) * p0[s] + a * start[s] for s in "LR"}, iters=20)
    q = kin.ik(q, start, iters=200)
    q_cross = q.copy()
    worst = kin.pos_error(start)

    loop = []
    n = int(T_LOOP * IK_HZ)
    for i in range(n + 1):
        u = i / n
        tgt = targets(loop_theta(u), min(1.0, max(0.0, (0.35 - u) / 0.15)))  # uncross once clear
        q = kin.ik(q, tgt, iters=30)
        worst = max(worst, kin.pos_error(tgt))
        loop.append(q.copy())

    # Way back: open the hands sideways first, set up the front/back offset, then
    # slide into the cross. Interpolating joints directly would push the hands
    # through each other. Waypoints are (y, z, cross) in left-side coordinates.
    z_clap = CENTER_Z + RADIUS * math.sin(THETA_CLAP)
    z_start = CENTER_Z + RADIUS * math.sin(THETA_START)
    ways = np.array([
        (CLAP_Y, z_clap, 0.0),
        (0.17, z_clap + 0.08, 0.0),
        (0.12, z_start, 1.0),
        (CENTER_Y + RADIUS * math.cos(THETA_START), z_start, 1.0),
    ])
    back = []
    n = int(T_BACK * IK_HZ)
    for i in range(n + 1):
        f = smooth(i / n) * (len(ways) - 1)
        k = min(int(f), len(ways) - 2)
        y, z, cross = ways[k] + (f - k) * (ways[k + 1] - ways[k])
        tgt = {side: hand_point(kin, side, y, z, cross) for side in "LR"}
        q = kin.ik(q, tgt, iters=30)
        worst = max(worst, kin.pos_error(tgt))
        back.append(q.copy())
    # Settle onto exactly q_cross so the next cycle starts without a jump.
    back[-1] = q_cross

    if worst > 0.03:   # the crossed pose alone sits ~2 cm off its target
        print(f"WARNING: IK misses the hand path by up to {worst * 100:.1f} cm")
    return q_cross, np.array(loop), np.array(back)


def smooth(a):
    return 0.5 - 0.5 * math.cos(math.pi * min(1.0, max(0.0, a)))


def sample(samples, t):
    """Linear interpolation in a trajectory sampled at IK_HZ."""
    f = t * IK_HZ
    i = min(int(f), len(samples) - 2)
    return samples[i] + min(1.0, f - i) * (samples[i + 1] - samples[i])


def cycle_target(t, q_cross, loop, back):
    """Joint target at time t (seconds) within one cycle after the first approach."""
    if t < T_HOLD:
        return q_cross
    t -= T_HOLD
    if t < T_LOOP:
        return sample(loop, t)
    t -= T_LOOP
    if t < T_CLAP:
        return loop[-1]
    return sample(back, t - T_CLAP)


CYCLE = T_HOLD + T_LOOP + T_CLAP + T_BACK


class StateCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.q = None

    def on_state(self, s):
        with self.lock:
            self.q = list(s.q)


def main():
    path = robot_xml()
    print(f"planning on {path} ...")
    kin = Kinematics(path)
    q_cross, loop, back = plan(kin)

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
    q0 = np.array(q0)

    # Close the grippers so the hands meet as compact blocks.
    grip = datatypes.GripperCmd()
    grip.opening, grip.speed, grip.force = [0.0, 0.0], [80.0, 80.0], [50.0, 50.0]
    grip.stamp = time.time_ns()
    robot.publishGripperCmd(grip)

    cmd = datatypes.RobotCmd()
    cmd.motor_names = JOINTS
    cmd.mode = [0] * N
    cmd.Kp = KP.tolist()
    cmd.Kd = KD.tolist()

    dt = 1.0 / RATE_HZ
    t_start = time.monotonic()
    q_prev = q0
    last_phase = None
    try:
        while True:
            t = time.monotonic() - t_start
            if t < T_TO_START:
                q = q0 + smooth(t / T_TO_START) * (q_cross - q0)
                phase = "approach"
            else:
                tc = (t - T_TO_START) % CYCLE
                q = cycle_target(tc, q_cross, loop, back)
                phase = ("hold" if tc < T_HOLD else
                         "loop" if tc < T_HOLD + T_LOOP else
                         "clap" if tc < T_HOLD + T_LOOP + T_CLAP else "back")
            if phase != last_phase:
                msg = {"hold": "いよーっ", "loop": "(ダイナミックループ)", "clap": "パン!"}
                if phase in msg:
                    print(msg[phase])
                last_phase = phase

            cmd.q = q.tolist()
            cmd.dq = ((q - q_prev) / dt).tolist()
            cmd.tau = kin.gravity(q).tolist()
            cmd.stamp = time.time_ns()
            robot.publishRobotCmd(cmd)
            q_prev = q
            time.sleep(dt)
    except KeyboardInterrupt:
        print("stopped")


if __name__ == "__main__":
    main()
