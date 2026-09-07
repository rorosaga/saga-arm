# saga-arm

MuJoCo simulation of the [LeRobot](https://github.com/huggingface/lerobot)
SO-ARM100 arm, driven from the keyboard.

The model is the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
`trs_so_arm100` package, vendored under `models/so_arm100/` (Apache-2.0).

## Setup

Requires [uv](https://docs.astral.sh/uv/) and, on macOS, a Homebrew Python 3.13
(`brew install python@3.13`). uv is told to prefer system interpreters because
MuJoCo's `mjpython` launcher cannot load uv's standalone Python builds.

```bash
uv sync
uv run pytest
```

## Teleop

```bash
uv run mjpython -m saga_arm.teleop     # macOS
uv run python   -m saga_arm.teleop     # Linux
```

A viewer window opens with the arm in its home pose. Keep the **terminal**
focused and press keys there; the viewer only displays. (The viewer binds every
letter to its own visualisation toggles, so keys are read from the terminal.)

| Joint          | +   | −   |
| -------------- | --- | --- |
| base rotation  | `q` | `a` |
| shoulder pitch | `w` | `s` |
| elbow          | `e` | `d` |
| wrist pitch    | `r` | `f` |
| wrist roll     | `t` | `g` |
| gripper        | `y` | `h` |

`0` returns to the home pose. `Esc` or `Ctrl-C` quits. Hold a key to keep
moving; each press nudges the joint target by 0.05 rad.

## Layout

```
saga_arm/sim.py      ArmSim: model + data, per-joint targets, clamping, stepping
saga_arm/teleop.py   key map, terminal reader, passive-viewer loop
models/so_arm100/    vendored Menagerie model (see SOURCE.txt)
tests/               headless tests, no viewer needed
```
