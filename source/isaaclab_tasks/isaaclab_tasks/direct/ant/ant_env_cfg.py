# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from isaaclab_newton.physics import KaminoSolverCfg, MJWarpSolverCfg, NewtonCfg, StiffGIPCSolverCfg
from isaaclab_ovphysx.physics import OvPhysxCfg
from isaaclab_physx.physics import PhysxCfg

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils.configclass import configclass

from isaaclab_tasks.utils import PresetCfg

from isaaclab_assets.robots.ant import ANT_CFG


@configclass
class AntPhysicsCfg(PresetCfg):
    default: PhysxCfg = PhysxCfg()
    physx: PhysxCfg = PhysxCfg()
    newton_mjwarp: NewtonCfg = NewtonCfg(
        solver_cfg=MJWarpSolverCfg(
            njmax=45,
            nconmax=25,
            cone="pyramidal",
            integrator="implicitfast",
            impratio=1,
        ),
        num_substeps=1,
        debug_mode=False,
    )
    newton_kamino: NewtonCfg = NewtonCfg(
        solver_cfg=KaminoSolverCfg(
            integrator="moreau",
            use_collision_detector=False,
            sparse_jacobian=True,
            constraints_alpha=0.1,
            padmm_max_iterations=100,
            padmm_primal_tolerance=1e-4,
            padmm_dual_tolerance=1e-4,
            padmm_compl_tolerance=1e-4,
            padmm_rho_0=0.05,
            padmm_eta=1e-5,
            padmm_use_acceleration=True,
            padmm_warmstart_mode="containers",
            padmm_contact_warmstart_method="geom_pair_net_force",
            padmm_use_graph_conditionals=False,
            collision_detector_pipeline="unified",
            collision_detector_max_contacts_per_pair=8,
        ),
        num_substeps=2,
        debug_mode=False,
        use_cuda_graph=True,
    )
    ovphysx: OvPhysxCfg = OvPhysxCfg()
    # StiffGIPC (IPC) — locomotion variant. Unlike cartpole this task is CONTACT-HEAVY
    # (feet must push against the ground), so collision is ENABLED. The floating torso is
    # a Newton FREE joint (skipped by the StiffGIPC joint builder -> a free ABD body that
    # falls/contacts under gravity, i.e. a floating base); the 8 leg joints are REVOLUTE,
    # effort-driven (set_joint_effort_target -> joint_f), so they are NOT position-driven.
    stiffgipc: NewtonCfg = NewtonCfg(
        solver_cfg=StiffGIPCSolverCfg(
            use_effort_control=True,
            skip_all_collision=False,  # feet need ground contact to walk
            newton_iter_cap=150,
            newton_tol=2.0e-1,  # with absolute eff_bbox (=1.0), this matches the 2-env-quality
            absolute_dhat=1.0e-3,  # fixed IPC barrier distance (multi-env stable)
            friction_rate=3.0,  # high foot grip so the feet don't slip when pushing off (locomotion traction)
            joint_strength_ratio=1000.0,
            revolute_driving_strength_ratio=0.0,  # legs are effort-driven, not position-held
            collision_mesh_segments=8,  # simpler feet/legs (~81 verts/capsule) -> faster IPC contact
            torso_skip_ground_collision=True,  # only legs/feet touch ground, not the torso sphere
        ),
        num_substeps=1,
        debug_mode=False,
        use_cuda_graph=False,
    )


@configclass
class AntEnvCfg(DirectRLEnvCfg):
    # env
    episode_length_s = 15.0
    decimation = 2
    action_scale = 0.5
    action_space = 8
    observation_space = 36
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation, physics=AntPhysicsCfg())
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="average",
            restitution_combine_mode="average",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096, env_spacing=4.0, replicate_physics=True, clone_in_fabric=True
    )

    # robot
    robot: ArticulationCfg = ANT_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    joint_gears: list = [15, 15, 15, 15, 15, 15, 15, 15]

    heading_weight: float = 0.5
    up_weight: float = 0.1

    energy_cost_scale: float = 0.05
    actions_cost_scale: float = 0.005
    alive_reward_scale: float = 0.5
    dof_vel_scale: float = 0.2

    death_cost: float = -2.0
    termination_height: float = 0.31

    angular_velocity_scale: float = 1.0
    contact_force_scale: float = 0.1

    # Penalty for being slower than ~0.5 m/s toward the target. Pushes the policy out of
    # the degenerate "stand still and collect alive reward" optimum so it actually walks.
    standing_penalty_scale: float = 1.0
