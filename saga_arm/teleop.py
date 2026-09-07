"""Keyboard teleop for the SO-ARM100 in the MuJoCo passive viewer.

Run with ``uv run mjpython -m saga_arm.teleop`` (macOS needs mjpython for
the passive viewer). Keys are read from the terminal you launched from, not
from the viewer window, because the viewer binds every letter key to its own
visualisation toggles.
"""

from __future__ import annotations

import select
import sys
import termios
import time
import tty

import mujoco
import mujoco.viewer

from saga_arm.sim import ArmSim

STEP_RAD = 0.05  # radians per key press (hold the key to auto-repeat)
HOME_KEY = "0"
QUIT_KEYS = {"\x1b", "\x03", "\x04"}  # Esc, Ctrl-C, Ctrl-D

# key -> (joint, direction)
KEYMAP: dict[str, tuple[str, float]] = {
    "q": ("Rotation", +1), "a": ("Rotation", -1),
    "w": ("Pitch", +1), "s": ("Pitch", -1),
    "e": ("Elbow", +1), "d": ("Elbow", -1),
    "r": ("Wrist_Pitch", +1), "f": ("Wrist_Pitch", -1),
    "t": ("Wrist_Roll", +1), "g": ("Wrist_Roll", -1),
    "y": ("Jaw", +1), "h": ("Jaw", -1),
}

BANNER = """\
saga-arm teleop  (keep this terminal focused; the viewer window just displays)

  base rotation   q / a        wrist pitch   r / f
  shoulder pitch  w / s        wrist roll    t / g
  elbow           e / d        gripper       y / h
  home pose       0            quit          Esc or Ctrl-C
"""


def apply_key(sim: ArmSim, key: str, step: float = STEP_RAD) -> bool:
    """Apply one key press to the sim. Returns False when the key means quit."""
    key = key.lower()
    if key in QUIT_KEYS:
        return False
    if key == HOME_KEY:
        sim.home()
    elif key in KEYMAP:
        joint, direction = KEYMAP[key]
        sim.nudge(joint, direction * step)
    return True


class RawTerminal:
    """Context manager that puts stdin into cbreak mode and restores it."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def read_key(self) -> str | None:
        """Return one pending key, or None if nothing is waiting."""
        ready, _, _ = select.select([self.fd], [], [], 0)
        return sys.stdin.read(1) if ready else None


def frame_camera(cam: mujoco.MjvCamera) -> None:
    """Pull the free camera back so the whole arm is visible."""
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = (0.0, -0.1, 0.15)
    cam.distance = 0.9
    cam.azimuth = 135
    cam.elevation = -20


def main() -> None:
    sim = ArmSim()
    print(BANNER)
    with (
        mujoco.viewer.launch_passive(sim.model, sim.data) as viewer,
        RawTerminal() as term,
    ):
        frame_camera(viewer.cam)
        dt = sim.model.opt.timestep
        while viewer.is_running():
            t0 = time.perf_counter()
            key = term.read_key()
            if key is not None and not apply_key(sim, key):
                break
            sim.step()
            viewer.sync()
            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))


if __name__ == "__main__":
    main()
