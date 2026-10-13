# Deployment executor patch

The deployment executor (`g1_ctrl`) is Unitree's `unitree_rl_mjlab` at commit
`1425b15f73bd4095f0df53709d7c389c3eb9e790` (2026-04-13). Only two executor files
were modified; the training code under `src/` is unmodified.

| File | Change |
|---|---|
| `deploy/include/isaaclab/envs/mdp/terminations.h` | `bad_orientation` was disabled upstream (`return false`); re-enabled with a clamped `acos`, so the executor falls back to damping when the trunk inclination exceeds `limit_angle` (1.0 rad in `deploy.yaml`) |
| `deploy/robots/g1/config/config.yaml` | New FSM state `Mimic_Baileaxe` (id 6) reachable from `Velocity` with `RB + B`; `start` added as an alternative to `RT + A` to return to `Velocity` |

`cmake_aarch64_no_fmt.patch` removes `fmt` from the link line of
`deploy/robots/g1/CMakeLists.txt`; it is needed only when building on the Jetson
(spdlog there ships its bundled fmt). It does not change the executor's behaviour.

## Apply

```bash
git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git
cd unitree_rl_mjlab
git checkout 1425b15
git apply /path/to/release/deploy/unitree_rl_mjlab_1425b15.patch
git apply /path/to/release/deploy/cmake_aarch64_no_fmt.patch   # Jetson only
```

`config.yaml` and `terminations.h` in this folder are the full patched files, for
reference. `deploy.yaml` is the policy-level configuration read by the executor
(joint map, PD gains, action scale, `limit_angle`); it is identical to
`params/deploy.yaml` except for a trailing newline.

## Install the policy

```
deploy/robots/g1/config/policy/mimic/baileaxe/
├── exported/policy.onnx      <- release/policies/baileaxe/policy.onnx
└── params/
    ├── deploy.yaml           <- release/deploy/deploy.yaml
    └── baileaxe57.npz        <- release/references/baileaxe57.npz
```

Build and run `g1_ctrl` as described in the upstream README. On the robot the
binary is started by `launcher/g1_launcher.cpp` (see `launcher/`), which only
launches it after the factory controller has released `rt/lowcmd`.
