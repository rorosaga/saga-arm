import numpy as np
import pytest

from saga_arm.sim import ArmSim, JOINTS


@pytest.fixture
def sim():
    return ArmSim()


def test_model_has_expected_actuators(sim):
    assert sim.model.nu == 6
    assert list(JOINTS) == [
        "Rotation",
        "Pitch",
        "Elbow",
        "Wrist_Pitch",
        "Wrist_Roll",
        "Jaw",
    ]


def test_starts_at_home_keyframe(sim):
    home = sim.model.key("home")
    np.testing.assert_allclose(sim.data.qpos, home.qpos)
    np.testing.assert_allclose(sim.target, home.ctrl)


def test_nudge_moves_target_and_joint_follows(sim):
    idx = JOINTS.index("Elbow")
    before = sim.target[idx]
    sim.nudge("Elbow", +0.3)
    assert sim.target[idx] == pytest.approx(before + 0.3)
    sim.step(seconds=1.0)
    assert sim.data.qpos[idx] == pytest.approx(before + 0.3, abs=0.05)


def test_nudge_clamps_to_joint_range(sim):
    lo, hi = sim.joint_range("Rotation")
    sim.nudge("Rotation", +100.0)
    assert sim.target[JOINTS.index("Rotation")] == pytest.approx(hi)
    sim.nudge("Rotation", -100.0)
    assert sim.target[JOINTS.index("Rotation")] == pytest.approx(lo)


def test_unknown_joint_raises(sim):
    with pytest.raises(KeyError):
        sim.nudge("Shoulder", 0.1)


def test_home_resets_after_moving(sim):
    sim.nudge("Pitch", 0.5)
    sim.step(seconds=0.5)
    sim.home()
    home = sim.model.key("home")
    np.testing.assert_allclose(sim.data.qpos, home.qpos)
    np.testing.assert_allclose(sim.target, home.ctrl)
