# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Franka deformable lifting environment.

The scene mirrors ``newton/examples/softbody/example_softbody_franka.py``:
a Franka Panda manipulator on a tabletop with a tetrahedral deformable object simulated
by VBD. The RL task is to lift the deformable object's centre of mass to a randomised target
position sampled in the robot's root frame.
"""

from __future__ import annotations

import os

from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg, StiffGIPCSolverCfg
from isaaclab_newton.sim.schemas import NewtonDeformableBodyPropertiesCfg
from isaaclab_newton.sim.spawners.materials import NewtonDeformableBodyMaterialCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab_physx.sim.schemas import PhysxDeformableBodyPropertiesCfg
from isaaclab_physx.sim.spawners.materials import PhysxDeformableBodyMaterialCfg

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.assets.deformable_object import DeformableObjectCfg
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.configclass import configclass

from isaaclab_contrib.deformable.newton_manager_cfg import CoupledMJWarpVBDSolverCfg, NewtonModelCfg, VBDSolverCfg

from isaaclab_tasks.utils import PresetCfg

from . import mdp

##
# Pre-defined configs
##

from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG  # isort:skip


##
# Helpers
##


# Shared volume material parameters. The Newton config below uses the equivalent Lame parameters.
# 8e4 (jelly) was too soft for IPC: under the gripper it deformed deeply -> contact d->0 ->
# barrier Kappa blew up (220->22335), Newton hit the iter cap, training death-spiralled.
# 5e5 (soft rubber) keeps deformation shallow -> d stays > 0 -> Kappa bounded -> solvable.
YOUNGS_MODULUS = 5e5
POISSONS_RATIO = 0.25


@configclass
class DeformableNewtonCfg(NewtonCfg):
    """NewtonCfg extended with model-level contact parameters for deformable objects.

    Uses a distinct class name so that ``_is_kitless_physics`` does not
    match it, ensuring Kit is launched for USD deformable spawning.
    """

    model_cfg: NewtonModelCfg | None = None
    """Global Newton model parameters applied after builder finalization."""


@configclass
class DeformableCfg(PresetCfg):
    """Preset config for the deformable object, matching the Newton example."""

    newton_mjwarp_vbd: DeformableObjectCfg = DeformableObjectCfg(
        prim_path="/World/envs/env_.*/Deformable",
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.05)),
        spawn=sim_utils.MeshCuboidCfg(
            size=(0.3, 0.05, 0.05),
            deformable_props=NewtonDeformableBodyPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.95, 0.85, 0.1)),
            physics_material=NewtonDeformableBodyMaterialCfg(
                density=300.0,
                k_mu=YOUNGS_MODULUS / (2.0 * (1.0 + POISSONS_RATIO)),
                k_lambda=(YOUNGS_MODULUS * POISSONS_RATIO / ((1.0 + POISSONS_RATIO) * (1.0 - 2.0 * POISSONS_RATIO))),
                particle_radius=0.01,
            ),
        ),
    )

    physx: DeformableObjectCfg = DeformableObjectCfg(
        prim_path="/World/envs/env_.*/Deformable",
        init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.5, 0.0, 0.05)),
        spawn=sim_utils.MeshCuboidCfg(
            size=(0.3, 0.05, 0.05),
            deformable_props=PhysxDeformableBodyPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.95, 0.85, 0.1)),
            physics_material=PhysxDeformableBodyMaterialCfg(
                density=300.0,
                youngs_modulus=YOUNGS_MODULUS,
                poissons_ratio=POISSONS_RATIO,
                static_friction=10.0,
                dynamic_friction=5.0,
            ),
        ),
    )

    default = newton_mjwarp_vbd


@configclass
class PhysicsCfg(PresetCfg):
    # Newton physics: MJWarp rigid + VBD soft, one-way coupled
    # (matches newton/examples/softbody/example_softbody_franka.py)
    newton_mjwarp_vbd: DeformableNewtonCfg = DeformableNewtonCfg(
        solver_cfg=CoupledMJWarpVBDSolverCfg(
            rigid_solver_cfg=MJWarpSolverCfg(
                njmax=40,
                nconmax=20,
                ls_iterations=20,
                cone="pyramidal",
                impratio=1,
                ls_parallel=False,
                integrator="implicitfast",
                ccd_iterations=100,
            ),
            soft_solver_cfg=VBDSolverCfg(
                iterations=10,
                integrate_with_external_rigid_solver=True,
                particle_enable_self_contact=False,
                particle_collision_detection_interval=-1,
            ),
            coupling_mode="two_way",
        ),
        model_cfg=NewtonModelCfg(
            soft_contact_ke=1e4,
            soft_contact_kd=1e-5,
            soft_contact_mu=5.0,
            shape_material_ke=4e4,
            shape_material_kd=1e-5,
            shape_material_mu=5.0,
        ),
        num_substeps=10,
        use_cuda_graph=True,
    )

    # StiffGIPC (IPC): unified rigid (ABD) + FEM soft + penetration-free contact in
    # a single solver (no MJWarp+VBD coupling). num_substeps=1 (IPC self-iterates);
    # CUDA graph off (the IPC engine.step does host<->device syncs).
    stiffgipc: DeformableNewtonCfg = DeformableNewtonCfg(
        solver_cfg=StiffGIPCSolverCfg(
            young_modulus=YOUNGS_MODULUS,
            newton_iter_cap=150,
            relative_dhat=1.0e-3,
            # Fixed 2 mm IPC barrier (absolute), so contact behaves identically at any
            # num_envs. Without this, dHat = relative_dhat * full-scene bbox grows with
            # env count → inconsistent/coupled contact across environments.
            absolute_dhat=3.0e-3,
            # Per-frame joint slew cap = ~6 rad/s @60fps (a real Franka does ~2-3 rad/s).
            # The default 1.0 rad/frame = 60 rad/s lets the untrained policy demand huge
            # one-frame joint jumps the IPC joint solve can't converge (Newton pegs at the
            # iter cap with NO contact). A physical cap keeps each step solvable; measured
            # sin² Newton iters: cap0.1 -> mean ~17 / 0 lockups (cap1.0 -> permanent lockup).
            max_revolute_step_per_frame=0.1,
            max_prismatic_step_per_frame=0.005,
            friction_rate=0.5,
            # Manipulator: arm self-collision is always excluded; also drop arm↔ground
            # IPC contact so links aren't perturbed by the engine ground plane. The FEM
            # block keeps ground contact and still rests on the table.
            disable_articulation_ground_collision=True,
            # Only the gripper (hand+fingers) may touch the soft cube -> forces a real
            # finger grasp instead of shoving with the forearm, and cuts contact pairs.
            # (task specifies WHICH links are the "gripper"; solver just applies the filter)
            object_collision_link_filter=("finger", "hand"),
        ),
        model_cfg=NewtonModelCfg(
            soft_contact_ke=1e4,
            soft_contact_kd=1e-5,
            soft_contact_mu=5.0,
            shape_material_ke=4e4,
            shape_material_kd=1e-5,
            shape_material_mu=5.0,
        ),
        num_substeps=1,
        use_cuda_graph=False,
    )

    physx: PhysxCfg = PhysxCfg()

    default = newton_mjwarp_vbd


##
# Scene definition
##


@configclass
class _FrankaSoftSceneCfg(InteractiveSceneCfg):
    """Scene for the Franka deformable environment."""

    robot: ArticulationCfg = FRANKA_PANDA_CFG.replace(prim_path="/World/envs/env_.*/Robot")

    # end-effector frame for reward shaping
    ee_frame: FrameTransformerCfg = FrameTransformerCfg(
        prim_path="/World/envs/env_.*/Robot/panda_link0",
        debug_vis=False,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="/World/envs/env_.*/Robot/panda_hand",
                name="end_effector",
                offset=OffsetCfg(pos=[0.0, 0.0, 0.1034]),
            ),
        ],
    )

    deformable: DeformableCfg = DeformableCfg()

    # static table matching the Newton example: half-extents (0.4, 0.4, 0.1) → top at z = 0.2
    # NOTE: SeattleLabTable USD has its origin on the top surface, so the deformable object
    # sits directly on it when placed at z = 0.05.
    table: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0.0, 0.0], rot=[0.0, 0.0, 0.707, 0.707]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    )

    # ground plane
    ground: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.0, 0.0, -1.05]),
        spawn=GroundPlaneCfg(),
    )

    # lights
    # dome_light: AssetBaseCfg = AssetBaseCfg(
    #     prim_path="/World/light",
    #     spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2000.0),
    # )
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )

    def __post_init__(self) -> None:
        # disable gravity on the arm so the low-PD actuators do not need to fight gravity sag,
        # which is the dominant source of steady-state IK tracking error.
        self.robot.spawn.rigid_props.disable_gravity = True

        # increase franka gripper stiffness
        self.robot.actuators["panda_hand"].effort_limit_sim = 500.0
        self.robot.actuators["panda_hand"].stiffness = 1000.0
        self.robot.actuators["panda_hand"].damping = 100.0


##
# MDP settings
##


@configclass
class CommandsCfg:
    """Commands for the deformable goal pose (xyz + identity quat in robot root frame)."""

    deformable_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name="panda_hand",
        resampling_time_range=(5.0, 5.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(0.4, 0.6),
            pos_y=(-0.25, 0.25),
            pos_z=(0.25, 0.5),
            roll=(0.0, 0.0),
            pitch=(0.0, 0.0),
            yaw=(0.0, 0.0),
        ),
        # Render the goal as a transparent colored sphere (a point) instead of a coordinate frame.
        goal_pose_visualizer_cfg=VisualizationMarkersCfg(
            prim_path="/Visuals/Command/goal_pose",
            markers={
                "sphere": sim_utils.SphereCfg(
                    radius=0.03,
                    visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(0.1, 0.9, 0.2),
                        opacity=0.4,
                    ),
                ),
            },
        ),
    )


@configclass
class ActionsCfg:
    """7-dim absolute end-effector pose (xyz + quaternion) via differential IK + 1-dim binary gripper."""

    arm_action = DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["panda_joint.*"],
        body_name="panda_hand",
        controller=DifferentialIKControllerCfg(
            command_type="pose",
            use_relative_mode=False,
            ik_method="dls",
            ik_params={"lambda_val": 0.6},
        ),
        body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.107]),
    )
    gripper_action = mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["panda_finger.*"],
        open_command_expr={"panda_finger_.*": 0.05},
        close_command_expr={"panda_finger_.*": 0.0},
    )


@configclass
class ObservationsCfg:
    """Policy observations: joint state, deformable COM in robot frame, target, last action."""

    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        deformable_sampled_points = ObsTerm(
            func=mdp.DeformableSampledPointsInRobotRootFrame,
            params={"asset_cfg": SceneEntityCfg("deformable"), "num_points": 20},
        )
        target_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "deformable_pose"})
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Reset events: robot to default joint config, deformable with small position randomization."""

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={"position_range": (0.9, 1.1), "velocity_range": (0.0, 0.0)},
    )

    reset_deformable = EventTerm(
        func=mdp.reset_nodal_state_uniform,
        mode="reset",
        params={
            "position_range": {"x": (-0.05, 0.05), "y": (-0.05, 0.05), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("deformable"),
        },
    )


@configclass
class RewardsCfg:
    """Lift-to-target reward for a deformable object."""

    # --- Stage 1: reach a PRE-GRASP point ABOVE the cube (makes from-above the path) ---
    reach_above = RewTerm(
        func=mdp.reach_above_cube,
        # std 0.6 (was 0.3): 1-tanh(d/std) saturates (~0 gradient) past ~3*std, so 0.3 died
        # past ~0.9 m; if the policy ever flung the arm far it got stuck with no gradient to
        # recover. 0.6 keeps a usable pull-back gradient out to ~1.8 m.
        params={"height": 0.10, "std": 0.6, "asset_cfg": SceneEntityCfg("deformable")},
        # big weight: getting the EE above the cube FAST is the learnable first step;
        # w3 was too weak vs noise (reach_above peaked 0.2 then collapsed). 10 dominates
        # early so the policy reliably reaches the pre-grasp pose, then the later stages build.
        weight=10.0,
    )
    # --- Stage 2: descend to the cube COM, gated on being horizontally above it ---
    reach_grasp = RewTerm(
        func=mdp.reach_grasp_from_above,
        params={"align_radius": 0.05, "std": 0.05, "asset_cfg": SceneEntityCfg("deformable")},
        weight=5.0,
    )
    # --- Stage 3: reward CLOSING the gripper only when fingers straddle the cube ---
    grasp_close = RewTerm(
        func=mdp.grasp_when_close,
        params={"max_dist": 0.04, "action_name": "gripper_action", "asset_cfg": SceneEntityCfg("deformable")},
        weight=2.0,
    )
    # penalise diving below the table (StiffGIPC has no arm-ground collision)
    ee_below_table = RewTerm(
        func=mdp.ee_below_table_penalty,
        params={"table_height": 0.0},
        weight=-25.0,
    )
    # --- Anti-overpress: penalize slamming the soft block (per-env proxy for the IPC
    # Newton=150 blow-up). The block's nodal speed spikes on a violent press; punishing
    # it teaches a gentle grasp (a gentle press / slow lift stays under the threshold). ---
    block_impact = RewTerm(
        func=mdp.block_impact_penalty,
        params={"vel_threshold": 0.5, "asset_cfg": SceneEntityCfg("deformable")},
        weight=-2.0,
    )
    # --- Stage 4: lift (strengthened 5 -> 15, matching the standard lift-cube) ---
    lifting_deformable = RewTerm(
        func=mdp.deformable_lifted,
        params={"minimal_height": 0.04, "asset_cfg": SceneEntityCfg("deformable")},
        weight=15.0,
    )
    deformable_goal_tracking = RewTerm(
        func=mdp.deformable_com_goal_distance,
        params={
            "std": 0.3,
            "minimal_height": 0.075,
            "command_name": "deformable_pose",
            "asset_cfg": SceneEntityCfg("deformable"),
        },
        weight=16.0,
    )
    deformable_goal_tracking_fine_grained = RewTerm(
        func=mdp.deformable_com_goal_distance,
        params={
            "std": 0.05,
            "minimal_height": 0.075,
            "command_name": "deformable_pose",
            "asset_cfg": SceneEntityCfg("deformable"),
        },
        weight=5.0,
    )

    # Only a LIGHT command-smoothness penalty. The old -1e-2 action_rate/joint_vel +
    # gripper-close -1.0 made moving/grasping net-negative -> the policy stayed still.
    # joint_vel / joint_torque / joint_acc penalties removed (per design: don't penalise
    # joint dynamics); -1e-4 action_rate just keeps the EE command from being jerky.
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)


@configclass
class TerminationsCfg:
    """Time out + table bounds/drop termination."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    deformable_outside_table = DoneTerm(
        func=mdp.deformable_outside_table_bounds,
        params={
            "x_bounds": (0.0, 1.0),
            "y_bounds": (-0.5, 0.5),
            "asset_cfg": SceneEntityCfg("deformable"),
        },
    )

    deformable_dropped = DoneTerm(
        func=mdp.deformable_com_below_minimum,
        params={"minimum_height": -0.1, "asset_cfg": SceneEntityCfg("deformable")},
    )

    ee_below_table = DoneTerm(
        func=mdp.ee_below_minimum,
        # relaxed 0.0 -> -0.1: the graded ee_below_table reward penalty now handles small
        # dips (gives a gradient to climb back); only terminate on a catastrophic dive so
        # episodes don't die before the policy can recover and proceed to grasping.
        params={"minimum_height": -0.1, "ee_frame_cfg": SceneEntityCfg("ee_frame")},
    )


##
# Environment configuration
##


@configclass
class FrankaSoftSceneCfg(PresetCfg):
    newton_mjwarp_vbd: _FrankaSoftSceneCfg = _FrankaSoftSceneCfg(num_envs=128, env_spacing=2.5, replicate_physics=True)

    # PhysX does not support replicating physics for deformable objects
    physx: _FrankaSoftSceneCfg = _FrankaSoftSceneCfg(num_envs=128, env_spacing=2.5, replicate_physics=False)

    default = newton_mjwarp_vbd


@configclass
class FrankaSoftEnvCfg(ManagerBasedRLEnvCfg):
    """Manager-based RL environment: Franka Panda lifting a volume deformable."""

    # Scene settings
    scene: FrankaSoftSceneCfg = FrankaSoftSceneCfg()
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self) -> None:
        # general settings
        # decimation 4 (was 1): hold each policy action for 4 sim steps so the arm has
        # time to track the commanded EE pose smoothly between action switches, instead of
        # chasing a new absolute target every single step (which made it slam/over-press
        # the soft block -> contact blow-ups during exploration).
        self.decimation = 4
        self.episode_length_s = 5.0

        # simulation settings
        self.sim.dt = 1 / 60.0
        self.sim.render_interval = self.decimation
        self.sim.gravity = (0.0, 0.0, -9.81)  # was (0,0,0); enabled per user to see deformable fall under StiffGIPC
        self.sim.physics = PhysicsCfg()

        # viewer settings
        # "world" (not "asset_root"): the camera no longer follows/re-centers on the
        # robot, so it doesn't snap on env reset — free to orbit/pan manually.
        self.viewer.origin_type = "world"
        # pulled back + up to frame ALL 4 envs at once (2x2 grid, env_spacing 2.5 -> ~3 m span,
        # centred ~(0.25, 0)); was (2,-2,1.5) which only showed ~1 env.
        self.viewer.eye = (5.5, -5.5, 4.5)
        self.viewer.lookat = (0.25, 0.0, 0.2)
        self.viewer.resolution = (1920, 1080)

        # [replay-only] REPLAY_NO_RESET=1 disables all episode resets so each env runs
        # continuously — watch sustained / successful behavior without reset stutter.
        # Per-env independent anyway, but this removes the global reset-processing pause.
        # Do NOT set during training (the policy needs resets to learn).
        if os.environ.get("REPLAY_NO_RESET"):
            self.episode_length_s = 1.0e6
            self.terminations.time_out = None
            self.terminations.deformable_outside_table = None
            self.terminations.deformable_dropped = None
            self.terminations.ee_below_table = None
