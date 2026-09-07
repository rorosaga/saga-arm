import pytest

from saga_arm.sim import ArmSim, JOINTS
from saga_arm.teleop import KEYMAP, STEP_RAD, apply_key


@pytest.fixture
def sim():
    return ArmSim()


def test_every_joint_has_a_plus_and_minus_key():
    seen = {}
    for joint, direction in KEYMAP.values():
        seen.setdefault(joint, set()).add(direction)
    assert set(seen) == set(JOINTS)
    assert all(dirs == {+1, -1} for dirs in seen.values())


def test_key_nudges_mapped_joint(sim):
    idx = JOINTS.index("Elbow")
    before = sim.target[idx]
    assert apply_key(sim, "e") is True
    assert sim.target[idx] == pytest.approx(before + STEP_RAD)
    assert apply_key(sim, "D") is True  # case-insensitive
    assert sim.target[idx] == pytest.approx(before)


def test_home_key_resets(sim):
    apply_key(sim, "w")
    apply_key(sim, "0")
    assert list(sim.target) == pytest.approx(list(sim.model.key("home").ctrl))


def test_quit_keys_return_false(sim):
    assert apply_key(sim, "\x1b") is False
    assert apply_key(sim, "\x03") is False


def test_unmapped_key_is_ignored(sim):
    before = sim.target.copy()
    assert apply_key(sim, "z") is True
    assert list(sim.target) == pytest.approx(list(before))
