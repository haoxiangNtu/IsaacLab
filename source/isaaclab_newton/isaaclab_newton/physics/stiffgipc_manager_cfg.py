# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the StiffGIPC Newton physics manager."""

from __future__ import annotations

from typing import TYPE_CHECKING

from isaaclab.utils.configclass import configclass

from .newton_manager_cfg import NewtonSolverCfg

if TYPE_CHECKING:
    from isaaclab_newton.physics import NewtonManager


@configclass
class StiffGIPCSolverCfg(NewtonSolverCfg):
    """StiffGIPC: a stiff, GPU Incremental-Potential-Contact (IPC) solver for rigid (ABD),
    deformable (FEM) bodies and articulations with accurate, penetration-free contact and
    contact-force readback (net per-body and per-pair, for the contact sensor)."""

    class_type: type[NewtonManager] | str = "{DIR}.stiffgipc_manager:NewtonStiffGIPCManager"
    """Manager class for the StiffGIPC solver."""

    solver_type: str = "stiffgipc"
    """Solver type."""

    young_modulus: float = 1.0e7
    """Default Young's modulus [Pa] for FEM bodies."""

    density: float = 1000.0
    """Default mass density [kg/m^3] (per-body overridden from Newton body_mass / volume)."""

    newton_iter_cap: int = 150
    """Maximum Newton iterations per IPC solve."""

    newton_tol: float = 1.0e-2
    """Newton convergence threshold. The solve stops when the projected-Newton step
    size drops below ``newton_tol * scene_bbox_diagonal``. Larger = converge in fewer
    iterations (looser). The engine's GUI defaults range 1e-2..5e-2."""

    relative_dhat: float = 1.0e-3
    """IPC barrier activation distance, relative to the scene bounding box."""

    friction_rate: float = 0.4
    """Coulomb friction coefficient."""

    joint_strength_ratio: float = 1000.0
    """Stiffness of the joint CONNECTION constraint (keeps parent/child bodies attached
    at the joint anchor). Higher = links resist being pulled apart."""

    revolute_driving_strength_ratio: float = 1000.0
    """Stiffness driving each revolute joint toward its target angle."""

    prismatic_driving_strength_ratio: float = 1000.0
    """Stiffness driving each prismatic joint toward its target position."""

    max_revolute_step_per_frame: float = 1.0
    """Maximum revolute target slew per IPC step [rad].

    IsaacLab position targets can jump by more than StiffGIPC's conservative
    native default (0.1 rad), so the Newton backend uses a larger default to
    make articulation targets track within one frame. Lower this for scenes
    with delicate FEM attachments that need gentle joint motion.
    """

    max_prismatic_step_per_frame: float = 0.02
    """Maximum prismatic target slew per IPC step [m]."""

    skip_all_collision: bool = False
    """Disable all IPC collision/contact handling.

    Useful for articulation-only tracking diagnostics. Keep this disabled for
    contact-force, cloth, FEM, or IsaacLab sensor validation.
    """

    use_effort_control: bool = False
    """Drive joints by FORCE/TORQUE (control.joint_f via set_revolute_torque /
    set_prismatic_force, with drive strength 0) instead of position targets.

    Needed for effort-controlled tasks with free joints (e.g. cartpole: force on the
    cart + free-swinging pole). Leave False for position/IK-controlled manipulators."""

    disable_articulation_ground_collision: bool = False
    """Skip IPC ground-plane collision for every link of each articulation.

    Self-collision within an articulation is always excluded; this additionally drops
    arm↔ground contact so a manipulator isn't stopped/perturbed by the engine ground
    plane (z=0). FEM/free bodies keep ground contact, so a soft object still rests.
    """

    stiffgipc_root: str | None = None
    """Filesystem path to the StiffGIPC python package (else STIFFGIPC_ROOT env / default)."""
