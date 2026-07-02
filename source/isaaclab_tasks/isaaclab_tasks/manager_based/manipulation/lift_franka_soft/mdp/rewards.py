# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Reward and termination functions for the Franka deformable lifting environment."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
import warp as wp

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import combine_frame_transforms

if TYPE_CHECKING:
    from isaaclab.assets import Articulation, DeformableObject
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.sensors import FrameTransformer


def deformable_lifted(
    env: ManagerBasedRLEnv,
    minimal_height: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("deformable"),
) -> torch.Tensor:
    """Reward if the deformable COM is above a minimum height.

    Args:
        env: The environment instance.
        minimal_height: Minimum COM height [m].
        asset_cfg: The deformable object entity.

    Returns:
        Reward tensor with shape ``(num_envs,)``.
    """
    asset: DeformableObject = env.scene[asset_cfg.name]
    com_z = wp.to_torch(asset.data.root_pos_w)[:, 2]
    return torch.where(com_z > minimal_height, 1.0, 0.0)


def deformable_ee_distance(
    env: ManagerBasedRLEnv,
    std: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("deformable"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward reaching the deformable's nearest nodal point with the end-effector.

    Args:
        env: The environment instance.
        std: The tanh kernel standard deviation [m].
        asset_cfg: The deformable object entity.
        ee_frame_cfg: The end-effector frame entity.

    Returns:
        Reward tensor with shape ``(num_envs,)``.
    """
    asset: DeformableObject = env.scene[asset_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    nodal_pos_w = wp.to_torch(asset.data.nodal_pos_w)
    ee_w = wp.to_torch(ee_frame.data.target_pos_w)[..., 0, :]
    distance = torch.linalg.norm(nodal_pos_w - ee_w.unsqueeze(1), dim=2).min(dim=1).values
    return 1.0 - torch.tanh(distance / std)


def ee_below_table_penalty(
    env: ManagerBasedRLEnv,
    table_height: float = 0.0,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Graded penalty for the end-effector dipping below the table surface.

    StiffGIPC disables arm<->ground IPC contact, so nothing physically stops the
    arm from passing through the table; the untrained policy overshoots downward
    (the dominant ``ee_below_table`` termination). Returns the depth below the
    table (>=0); pair with a NEGATIVE weight so the policy gets a gradient pushing
    the EE back up instead of only a sparse episode-end signal.

    Returns:
        Depth below the table per env [m], clamped >= 0, shape ``(num_envs,)``.
    """
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_z = wp.to_torch(ee_frame.data.target_pos_w)[..., 0, 2]
    return torch.clamp(table_height - ee_z, min=0.0)


def reach_above_cube(
    env: ManagerBasedRLEnv,
    height: float = 0.10,
    std: float = 0.1,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("deformable"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Stage 1: reward the EE reaching a pre-grasp point ABOVE the cube COM.

    Target = cube COM + (0,0,height). Pulling the EE to a point above the cube makes
    the from-above approach the rewarded path (vs diving through the table).
    """
    asset: DeformableObject = env.scene[asset_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    com = wp.to_torch(asset.data.nodal_pos_w).mean(dim=1)
    target = com.clone()
    target[:, 2] = target[:, 2] + height
    ee = wp.to_torch(ee_frame.data.target_pos_w)[..., 0, :]
    distance = torch.linalg.norm(ee - target, dim=1)
    return 1.0 - torch.tanh(distance / std)


def reach_grasp_from_above(
    env: ManagerBasedRLEnv,
    align_radius: float = 0.05,
    std: float = 0.05,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("deformable"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Stage 2: reward descending to the cube COM, ONLY when the EE is horizontally
    aligned over the cube (within ``align_radius``). The horizontal gate enforces a
    from-above descent — the EE cannot earn this from the side or from below.
    """
    asset: DeformableObject = env.scene[asset_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    com = wp.to_torch(asset.data.nodal_pos_w).mean(dim=1)
    ee = wp.to_torch(ee_frame.data.target_pos_w)[..., 0, :]
    horiz = torch.linalg.norm(ee[:, :2] - com[:, :2], dim=1)
    gate = (horiz < align_radius).float()
    distance = torch.linalg.norm(ee - com, dim=1)
    return gate * (1.0 - torch.tanh(distance / std))


def grasp_when_close(
    env: ManagerBasedRLEnv,
    max_dist: float = 0.04,
    action_name: str = "gripper_action",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("deformable"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Stage 3: reward CLOSING the gripper only when the EE (grasp point between the
    fingers) is within ``max_dist`` of the cube — i.e. the fingers straddle the cube.
    Replaces the blanket gripper-close penalty: encourages closing at the right moment
    instead of forbidding it.
    """
    asset: DeformableObject = env.scene[asset_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    com = wp.to_torch(asset.data.nodal_pos_w).mean(dim=1)
    ee = wp.to_torch(ee_frame.data.target_pos_w)[..., 0, :]
    near = (torch.linalg.norm(ee - com, dim=1) < max_dist).float()
    gripper_action = env.action_manager.get_term(action_name).raw_actions
    closing = torch.any(gripper_action < 0.0, dim=1).float()
    return near * closing


def block_impact_penalty(
    env: ManagerBasedRLEnv,
    vel_threshold: float = 0.5,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("deformable"),
) -> torch.Tensor:
    """Penalty for the gripper pressing/slamming the soft block too hard (per-env).

    The barrier-based IPC contact blows up (Newton hits the iteration cap, Kappa
    explodes) when the gripper drives deep/fast into the block during exploration. The
    block's nodal speed spikes on such a violent press, so penalize the max nodal speed
    above ``vel_threshold`` — a gentle grasp / slow lift stays under it. This is the
    per-env, GPU-side equivalent of "punish the Newton=150 blow-up": discourage the
    behaviour that causes it. Pair with a NEGATIVE weight.

    Returns:
        Per-env excess nodal speed above the threshold [m/s], clamped >= 0, shape ``(num_envs,)``.
    """
    asset: DeformableObject = env.scene[asset_cfg.name]
    nodal_vel = wp.to_torch(asset.data.nodal_vel_w)  # (N, nodes, 3)
    max_speed = torch.linalg.norm(nodal_vel, dim=2).max(dim=1).values  # (N,)
    return torch.clamp(max_speed - vel_threshold, min=0.0)


def deformable_com_goal_distance(
    env: ManagerBasedRLEnv,
    std: float,
    minimal_height: float,
    command_name: str,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("deformable"),
) -> torch.Tensor:
    """Reward tracking of the goal position by the deformable's COM (tanh kernel).

    Only credits when the COM is above ``minimal_height`` (i.e. the object is lifted).
    The command is interpreted as ``[x, y, z, qw, qx, qy, qz]`` in the robot's root frame.
    """
    robot: Articulation = env.scene[robot_cfg.name]
    asset: DeformableObject = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(
        wp.to_torch(robot.data.root_pos_w), wp.to_torch(robot.data.root_quat_w), des_pos_b
    )
    com_w = wp.to_torch(asset.data.root_pos_w)
    distance = torch.linalg.norm(des_pos_w - com_w, dim=1)
    return (com_w[:, 2] > minimal_height) * (1.0 - torch.tanh(distance / std))


def gripper_close_action(env: ManagerBasedRLEnv, action_name: str = "gripper_action") -> torch.Tensor:
    """Penalty signal for commanding the gripper to close.

    The binary gripper action uses negative float actions for close commands and
    non-negative actions for open commands.

    Args:
        env: The environment instance.
        action_name: Name of the gripper action term.

    Returns:
        Tensor with shape ``(num_envs,)`` containing ``1`` when the gripper is
        commanded closed and ``0`` otherwise.
    """
    gripper_action = env.action_manager.get_term(action_name).raw_actions
    return torch.any(gripper_action < 0.0, dim=1).float()


def deformable_com_below_minimum(
    env: ManagerBasedRLEnv,
    minimum_height: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("deformable"),
) -> torch.Tensor:
    """Termination signal when the deformable's COM falls below ``minimum_height`` [m]."""
    asset: DeformableObject = env.scene[asset_cfg.name]
    com_z = wp.to_torch(asset.data.root_pos_w)[:, 2]
    return com_z < minimum_height


def deformable_outside_table_bounds(
    env: ManagerBasedRLEnv,
    x_bounds: tuple[float, float],
    y_bounds: tuple[float, float],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("deformable"),
) -> torch.Tensor:
    """Terminate if any deformable nodal point leaves the table footprint.

    Args:
        env: The environment instance.
        x_bounds: Allowed x-position range in the environment frame [m].
        y_bounds: Allowed y-position range in the environment frame [m].
        asset_cfg: The deformable object entity.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    asset: DeformableObject = env.scene[asset_cfg.name]
    nodal_pos = wp.to_torch(asset.data.nodal_pos_w) - env.scene.env_origins.unsqueeze(1)
    outside_x = (nodal_pos[..., 0] < x_bounds[0]) | (nodal_pos[..., 0] > x_bounds[1])
    outside_y = (nodal_pos[..., 1] < y_bounds[0]) | (nodal_pos[..., 1] > y_bounds[1])
    return torch.any(outside_x | outside_y, dim=1)


def ee_below_minimum(
    env: ManagerBasedRLEnv,
    minimum_height: float,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Termination signal when the end-effector falls below ``minimum_height`` [m].

    Height is measured in the environment frame (``z`` of the EE position with the env
    origin subtracted), so the threshold is independent of the environment's xy offset.

    Args:
        env: The environment instance.
        minimum_height: Minimum allowed EE height in the environment frame [m].
        ee_frame_cfg: The end-effector frame entity.

    Returns:
        Boolean tensor with shape ``(num_envs,)``.
    """
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_z = wp.to_torch(ee_frame.data.target_pos_w)[..., 0, 2] - env.scene.env_origins[:, 2]
    return ee_z < minimum_height
