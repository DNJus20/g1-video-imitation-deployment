# Reproducible Motion Imitation from Monocular Video on a Commercial Humanoid

Code, configuration, policies and logs for the paper
*Reproducible Motion Imitation from Monocular Video on a Commercial Humanoid: An Open-Source Deployment Pipeline*
(HRFEST 2026, paper 82). ESAN R&D Labs, Universidad ESAN, Lima, Peru.

Release `v1.0-hrfest2026` is the version cited in the camera-ready paper.

## What is here

| Folder | Contents |
|---|---|
| `launcher/` | `g1_launcher.cpp` and the two systemd units: the admission-gated launcher for the Jetson Orin NX (Sec. IV-C) |
| `deploy/` | `unitree_rl_mjlab_1425b15.patch` against the upstream executor (two files: `terminations.h`, inclination-damping termination re-enabled; `config.yaml`, the `baileaxe` motion registered as an FSM state), the full patched files, `cmake_aarch64_no_fmt.patch` for the onboard build, `deploy.yaml`, and `APPLY.md` with the steps. The published `config.yaml` is the 2026-07-31 campaign version and registers `baileaxe` only; the `bailetinkus` state used on 2026-10-12 was added on the robot the same way |
| `policies/` | `baileaxe/policy.onnx` (deployed policy, 6/6 runs) and `bailetinkus/policy.onnx` (second motion) |
| `references/` | Retargeted reference motions (`.npz`, 50 Hz, 29 joints): `baileaxe57`, `bailetinkus30`, the non-training reference `bailemix57` and its variants `bailemix57f` (joint filter) and `bailemix57g` (root-height correction). The source videos are not released. `dance1_subject2.npz` is the public reference shipped with the upstream repository (LAFAN1, CC BY-NC-ND 4.0) and is included as a substitute |
| `params/` | `env.yaml`, `agent.yaml` and `deploy.yaml` of the reported training run (`2026-07-21_19-47-02`); seed 42 |
| `analysis/` | Scripts that produce every number in the paper: hardware RMSE and body-position error by forward kinematics, the deployment-binary bench (sim-to-sim), the intervention study, the reference feasibility checks, the attempt accounting and the inclination-damping bench. `lowstate_logger/` is the native Jetson logger used on 2026-10-12, with `lowcmd_pubs.cpp`, the command-topic publisher counter used for the cold-boot test. `insumos/` holds the robot model (`g1.xml`) and joint constants |
| `data/campaign_2026-07-31/` | Logs of the six protocol runs (`.npz` + `.json`), the excluded records, and `per_run.csv` / `summary.json` |
| `data/bench/` | Deployment-binary bench runs for both motions and the intervention variants |
| `data/lab_2026-10-12/` | Verification run with the replacement motion controller, second motion on hardware, five cold-boot cycles, inclination-damping bench |
| `data/cr/` | Summary JSON files behind Tables III–VI |

## Upstream components (unmodified)

- GVHMR `6ec3ca3` — 3D human pose from monocular video
- GMR `bb1bbe4` — retargeting to the 29-DoF model
- `unitree_rl_mjlab` `1425b15` — motion-tracking task (a reimplementation of BeyondMimic) and the deployment executor; `mjlab` 1.2.0

Training uses the repository defaults; only `--env.scene.num-envs=8192`, `--agent.max-iterations=15000` and `--motion_file` are set on the command line.

## Reproducing

1. Reference checks: `analysis/ref_viabilidad.py <reference.npz>` prints the per-joint continuity bound and the sole-height statistics.
2. Training: `unitree_rl_mjlab@1425b15`, task `Unitree-G1-Tracking-No-State-Estimation`, with the files in `params/`.
3. Deployment-binary bench: `analysis/sim2sim_run.sh <policy> <reference>` runs the unmodified `g1_ctrl` against MuJoCo and reports RMSE, peak pitch and completion.
4. Hardware metrics: `analysis/analyze_runs.py data/campaign_2026-07-31/` reproduces Table II; `analysis/fk_cuerpos.py` reproduces the body-position error.

## Hardware

Unitree G1 EDU Plus (29 DoF), onboard NVIDIA Jetson Orin NX. Training on one RTX 4070 Ti SUPER (16 GB).

## License

Apache-2.0 for our code and data. The `deploy/` patch applies to Unitree's `unitree_rl_mjlab`, which keeps its own license; `references/dance1_subject2.npz` is LAFAN1 data under CC BY-NC-ND 4.0.

`MANIFEST.txt` lists the SHA-256 of every file in this release.

## Citation

G. D. Justiniani Arohuanca, J. G. Ayala Rojas and N. S. Carrión Puelles, "Reproducible Motion Imitation from Monocular Video on a Commercial Humanoid: An Open-Source Deployment Pipeline," in *Proc. HRFEST 2026 (International Conference on Humanoid Robotics, Innovation and Leadership)*, IEEE, 2026.
