"""Headless MuJoCo simulation of the SO-ARM100 with joint-target control."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

MODEL_DIR = Path(__file__).resolve().parent.parent / "models" / "so_arm100"
SCENE_XML = MODEL_DIR / "scene.xml"

# Actuator names in the Menagerie model, in actuator order.
JOINTS: tuple[str, ...] = (
    "Rotation",
    "Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
    "Jaw",
)


class ArmSim:
    """Owns an MjModel/MjData pair and a target angle per actuator.

    Targets are written to ``data.ctrl`` on every step, so the model's
    position actuators drive each joint toward its target.
    """

    def __init__(self, scene_xml: Path | str = SCENE_XML):
        self.model = mujoco.MjModel.from_xml_path(str(scene_xml))
        self.data = mujoco.MjData(self.model)
        self.target = np.zeros(self.model.nu)
        self._index = {name: self.model.actuator(name).id for name in JOINTS}
        self.home()

    def home(self) -> None:
        """Reset the arm to the model's ``home`` keyframe."""
        key = self.model.key("home")
        mujoco.mj_resetDataKeyframe(self.model, self.data, key.id)
        self.target[:] = key.ctrl
        self.data.ctrl[:] = self.target
        mujoco.mj_forward(self.model, self.data)

    def joint_range(self, joint: str) -> tuple[float, float]:
        lo, hi = self.model.actuator_ctrlrange[self._index[joint]]
        return float(lo), float(hi)

    def set_target(self, joint: str, value: float) -> None:
        """Set one joint's target angle in radians, clamped to its range."""
        lo, hi = self.joint_range(joint)
        self.target[self._index[joint]] = float(np.clip(value, lo, hi))

    def nudge(self, joint: str, delta: float) -> None:
        """Move one joint's target by ``delta`` radians, clamped to its range."""
        self.set_target(joint, self.target[self._index[joint]] + delta)

    def step(self, seconds: float | None = None) -> None:
        """Advance physics by ``seconds`` (default: one model timestep)."""
        n = 1 if seconds is None else max(1, round(seconds / self.model.opt.timestep))
        self.data.ctrl[:] = self.target
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)
