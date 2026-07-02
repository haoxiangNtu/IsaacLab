# StiffGIPC × IsaacLab-3.0 lift_franka_soft — integration notes

## Original task (PRESERVED in git HEAD; my edits are uncommitted)
- **Task**: `Isaac-Lift-Soft-Franka-v0` (also `Isaac-Lift-Cloth-Franka-v0`)
  - dir: `source/isaaclab_tasks/.../manager_based/manipulation/lift_franka_soft/`
  - env cfg: `franka_soft_env_cfg.py:FrankaSoftEnvCfg`; RL: `agents/rsl_rl_ppo_cfg.py:FrankaDeformablePPORunnerCfg`
  - Mirrors `newton/examples/softbody/example_softbody_franka.py`: Franka Panda on a table + a
    tetrahedral deformable; lift the deformable's CoM to a randomized target.
  - **Default physics**: `PhysicsCfg.newton_mjwarp_vbd` = MJWarp (rigid) + VBD (soft), coupled
    (`CoupledMJWarpVBDSolverCfg`), num_substeps=10, cuda_graph=True. Also a `physx` preset.
  - **Deformable**: `DeformableCfg.newton_mjwarp_vbd` = `DeformableObjectCfg(spawn=MeshCuboidCfg(
    size=(0.3,0.05,0.05), deformable_props=NewtonDeformableBodyPropertiesCfg, ...))` — **USD-spawned**.
  - To restore original: `git -C /home/ps/Downloads/IsaacLab-3.0 checkout -- <franka_soft_env_cfg.py>`.

## Runtime set up (this session)
- conda env **il3** (Python 3.12.13).
- IsaacLab-3.0 (`release/3.0.0-beta2`, ext v6.1.2) installed editable kitless (`./isaaclab.sh --install`;
  isaaclab_mimic failed on egl-probe build — not needed).
- **newton 1.3.0.dev0** installed editable from `/home/ps/Downloads/newton` (upstream clone + the
  **StiffGIPC overlay** = untracked `newton/_src/solvers/stiffgipc/` with this session's edits:
  GPU Kabsch, get_contacts_device, joint_qd finite-diff, d=0 fix). This is the canonical newton-solver
  source for StiffGIPC. Engine = StiffGIPC **v0.6.5** (`STIFFGIPC_ROOT=/home/ps/Downloads/Stiff-GIPC-v065`,
  py3.12 → build_312). rsl-rl-lib 5.4.1 installed.
- Verified in il3: `from newton.solvers import SolverStiffGIPC`, `NewtonStiffGIPCManager`,
  `StiffGIPCSolverCfg`, `import stiff_physics` all OK.

## What I added (uncommitted)
- `franka_soft_env_cfg.py`: import `StiffGIPCSolverCfg`; added a `PhysicsCfg.stiffgipc` preset
  (`DeformableNewtonCfg(solver_cfg=StiffGIPCSolverCfg(young_modulus=YOUNGS_MODULUS, newton_iter_cap=150,
  relative_dhat=1e-3, friction_rate=0.5), num_substeps=1, use_cuda_graph=False)`).

## The blocker found
Running `./isaaclab.sh train --task=Isaac-Lift-Soft-Franka-v0 ... physics=stiffgipc --headless` (kitless)
→ **`[ERROR] Isaac Sim is not installed`**. The task is **NOT kitless-compatible**: its deformable is
USD-spawned, and `DeformableNewtonCfg` is deliberately named so `_is_kitless_physics` does NOT match it
("ensuring **Kit is launched for USD deformable spawning**"). So the lift task **requires Isaac Sim (Kit)**
for the USD deformable spawn — not for the Newton backend (physics is isaaclab_newton).

## Architecture of the deformable (why "just bypass USD" isn't a one-liner)
- `DeformableObject` (core) is a backend-dispatched **managed asset** → `isaaclab_contrib.deformable.deformable_object`.
- It is registered into the newton `ModelBuilder` via `NewtonManager._deformable_registry` + per-world
  builder hooks (newton_replicate.py). The **USD spawn** (MeshCuboidCfg, needs Kit) produces the tet geometry.
- A procedural (kitless) bypass = generate tet geometry without USD + register through the contrib builder
  hook + make `DeformableObjectData` read it back. Cross-pipeline change, not a bare `add_soft_grid`.

## Options to actually train StiffGIPC soft-grasp
- (A) Build Isaac Sim 6 from source (github isaac-sim/IsaacSim, py3.12) → run lift task as-is (USD deformable + Kit) with StiffGIPC. Big.
- (B) Use installed Isaac Sim 5.1 (`/home/ps/isaacsim`, py3.11) + a **py3.11** IL3 env → Kit for USD spawn + isaaclab_newton physics + StiffGIPC (build_311). No IS6 build. (IL3 beta2 targets IS6; 5.1 support per setup.py pin.)
- (C) Procedural-deformable kitless variant (bypass USD) — the cross-pipeline change above; no Isaac Sim.
- NOTE: standalone proof already exists — SolverStiffGIPC does a Franka soft-cube grasp kitless
  (`scratchpad/run_pick_headless.py`, grip ~6.9 N, cube lifted) via procedural `add_soft_grid`. The gap is
  only IL3's managed USD-deformable asset pipeline.
