# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

""".. deprecated:: v1
    Arch B — NOT the supported integration path. Use Arch A: drive
    :class:`newton.solvers.SolverStiffGIPC` directly via ``StiffGIPCSolverCfg``
    (``physics=stiffgipc`` in the direct tasks). Kept for reference only;
    unsupported in the v1 release.

StiffGIPC Newton manager.

Wraps :class:`newton.solvers.SolverStiffGIPC` (a Newton backend over the StiffGIPC
CUDA IPC engine) into Isaac Lab's Newton physics-manager interface. StiffGIPC runs
its own IPC collision detection internally, so Newton's collision pipeline is NOT
required (``_needs_collision_pipeline = False``) — analogous to the MuJoCo backend
when ``use_mujoco_contacts=True``.
"""

from __future__ import annotations

import inspect

import numpy as np
import warp as wp


class _WarpArrayNdCompat:
    """Make Warp 1.12's array2d/3d/4d functions support PEP-585 style annotations."""

    def __init__(self, fn):
        self._fn = fn
        self.__name__ = getattr(fn, "__name__", "arraynd")

    def __call__(self, *args, **kwargs):
        return self._fn(*args, **kwargs)

    def __getitem__(self, dtype):
        return self._fn(dtype=dtype)


def _patch_warp_config_log_level():
    # warp 1.13 (shipped with Isaac Sim 6) dropped ``warp.config.log_level``
    # (now ``verbose`` / ``verbose_warnings``), but newton 1.3's
    # ``ModelBuilder.color()`` still reads/writes ``warp.config.log_level``.
    # Add a settable compat alias so deformable graph-coloring works.
    import warp as _wp

    if not hasattr(_wp.config, "log_level"):
        try:
            _wp.config.log_level = 20  # logging.INFO; settable for newton's save/restore
        except Exception:
            pass
    # warp 1.13 also dropped the module-level LOG_* level constants that newton
    # 1.3's graph_coloring compares against. Restore the standard logging values.
    for _name, _val in (("LOG_DEBUG", 10), ("LOG_INFO", 20), ("LOG_WARNING", 30),
                        ("LOG_ERROR", 40), ("LOG_CRITICAL", 50)):
        if not hasattr(_wp, _name):
            try:
                setattr(_wp, _name, _val)
            except Exception:
                pass


def _patch_warp_array_nd_annotations():
    try:
        array_annotation = wp.array[wp.float32]
        if getattr(array_annotation, "__origin__", None) is wp.array:
            wp.array.__class_getitem__ = classmethod(lambda cls, dtype: cls(dtype=dtype))
    except Exception:
        pass
    for name in ("array2d", "array3d", "array4d"):
        array_nd = getattr(wp, name, None)
        if array_nd is None:
            continue
        try:
            array_nd[wp.float32]
        except TypeError:
            setattr(wp, name, _WarpArrayNdCompat(array_nd))


_patch_warp_config_log_level()
_patch_warp_array_nd_annotations()

from newton import Model  # noqa: E402
from newton.sensors import SensorContact as NewtonContactSensor  # noqa: E402
from newton.solvers import SolverStiffGIPC  # noqa: E402

from .newton_manager import NewtonManager  # noqa: E402
from .stiffgipc_manager_cfg import StiffGIPCSolverCfg  # noqa: E402


class _StiffGIPCContactSensor:
    """Newton SensorContact-compatible view backed by StiffGIPC contact readback."""

    def __init__(
        self,
        model: Model,
        solver: SolverStiffGIPC | None,
        sensing_obj_bodies: str | list[str] | None = None,
        sensing_obj_shapes: str | list[str] | None = None,
        counterpart_bodies: str | list[str] | None = None,
        counterpart_shapes: str | list[str] | None = None,
        measure_total: bool = True,
        verbose: bool = False,
    ):
        self._model = model
        self._solver = solver
        self._base = NewtonContactSensor(
            model,
            sensing_obj_bodies=sensing_obj_bodies,
            sensing_obj_shapes=sensing_obj_shapes,
            counterpart_bodies=counterpart_bodies,
            counterpart_shapes=counterpart_shapes,
            measure_total=measure_total,
            verbose=verbose,
            request_contact_attributes=False,
        )

        self.device = self._base.device
        self.verbose = self._base.verbose
        self.total_force = self._base.total_force
        self.total_force_friction = self._base.total_force_friction
        self.force_matrix = self._base.force_matrix
        self.force_matrix_friction = self._base.force_matrix_friction
        self.sensing_obj_type = self._base.sensing_obj_type
        self.counterpart_type = self._base.counterpart_type
        self.sensing_obj_idx = self._base.sensing_obj_idx
        self.counterpart_indices = self._base.counterpart_indices
        self.sensing_obj_transforms = self._base.sensing_obj_transforms

        shape_body = model.shape_body.numpy() if model.shape_body is not None else np.empty(0, dtype=np.int32)
        self._sensing_body_idx = self._entity_indices_to_body_indices(
            self.sensing_obj_type, self.sensing_obj_idx, shape_body
        )
        self._counterpart_body_idx = [
            self._entity_indices_to_body_indices(self.counterpart_type, row, shape_body)
            for row in self.counterpart_indices
        ]

    @staticmethod
    def _entity_indices_to_body_indices(
        entity_type: str | None, indices: list[int], shape_body: np.ndarray
    ) -> list[int]:
        if entity_type is None:
            return []
        if entity_type == "body":
            return [int(i) for i in indices]
        if entity_type == "shape":
            return [int(shape_body[int(i)]) if 0 <= int(i) < shape_body.size else -1 for i in indices]
        raise RuntimeError(f"Unexpected contact sensor entity type: {entity_type!r}")

    def update(self, state, contacts=None):
        if self.total_force is not None:
            total = self._compute_total_force()
            self.total_force.assign(total)
            if self.total_force_friction is not None:
                self.total_force_friction.zero_()
        else:
            total = None

        if self.force_matrix is not None:
            matrix = self._compute_force_matrix(total)
            self.force_matrix.assign(matrix)
            if self.force_matrix_friction is not None:
                self.force_matrix_friction.zero_()

    def _compute_total_force(self) -> np.ndarray:
        out = np.zeros((len(self._sensing_body_idx), 3), dtype=np.float32)
        solver = self._get_solver()
        if solver is None:
            return out
        for row, body_idx in enumerate(self._sensing_body_idx):
            if body_idx >= 0:
                out[row] = np.asarray(solver.get_body_contact_force_world(body_idx), dtype=np.float32)
        return out

    def _compute_force_matrix(self, total: np.ndarray | None) -> np.ndarray:
        if self.force_matrix is None:
            return np.zeros((0, 0, 3), dtype=np.float32)
        matrix = np.zeros(self.force_matrix.shape + (3,), dtype=np.float32)
        solver = self._get_solver()
        if solver is None:
            return matrix
        all_contact_bodies = tuple(getattr(solver, "_body_vrange", {}).keys())

        for row, sensing_body in enumerate(self._sensing_body_idx):
            if sensing_body < 0:
                continue
            row_total = total[row] if total is not None else np.asarray(
                solver.get_body_contact_force_world(sensing_body), dtype=np.float32
            )
            counterparts = self._counterpart_body_idx[row] if row < len(self._counterpart_body_idx) else []
            for col, counterpart_body in enumerate(counterparts):
                if counterpart_body >= 0:
                    matrix[row, col] = np.asarray(
                        solver.get_pair_contact_force_world(sensing_body, counterpart_body),
                        dtype=np.float32,
                    )
                else:
                    # Global shapes such as the ground plane have no Newton body.
                    # StiffGIPC reports ground in the net body force, so isolate it
                    # by subtracting known body-body partners.
                    body_pair_sum = np.zeros(3, dtype=np.float32)
                    for other_body in all_contact_bodies:
                        if other_body != sensing_body:
                            body_pair_sum += np.asarray(
                                solver.get_pair_contact_force_world(sensing_body, other_body),
                                dtype=np.float32,
                            )
                    matrix[row, col] = row_total - body_pair_sum
        return matrix

    def _get_solver(self) -> SolverStiffGIPC | None:
        if self._solver is None:
            self._solver = NewtonManager._solver
        return self._solver


class NewtonStiffGIPCManager(NewtonManager):
    """:class:`NewtonManager` specialization for the StiffGIPC IPC solver."""

    @classmethod
    def _build_solver(cls, model: Model, solver_cfg: StiffGIPCSolverCfg) -> None:
        valid = set(inspect.signature(SolverStiffGIPC.__init__).parameters) - {"self", "model"}
        kwargs = {k: v for k, v in solver_cfg.to_dict().items() if k in valid}
        NewtonManager._solver = SolverStiffGIPC(model, **kwargs)
        # StiffGIPC steps with separate input/output states and does its own
        # IPC contact detection, so Newton's collision pipeline is not needed.
        NewtonManager._use_single_state = False
        NewtonManager._needs_collision_pipeline = False

    @classmethod
    def initialize(cls, sim_context) -> None:
        """Install the deformable builder hooks (so USD-spawned deformables get
        registered + added to the Newton builder via ``add_soft_mesh``) before the
        base init. StiffGIPC consumes the same Newton soft mesh as the other
        Newton backends, so the generic deformable hooks apply unchanged."""
        from isaaclab_contrib.deformable.deformable_object import install_deformable_builder_hooks

        install_deformable_builder_hooks()
        super().initialize(sim_context)

    @classmethod
    def _solver_specific_clear(cls):
        from isaaclab_contrib.deformable.deformable_object import clear_deformable_builder_hooks

        clear_deformable_builder_hooks()

    @classmethod
    def _get_deformable_ignore_paths(cls) -> list[str]:
        """USD prim paths to skip in ``builder.add_usd`` (sim + visual meshes of
        each registered deformable), so Newton doesn't make redundant static
        colliders alongside the ``add_soft_mesh`` particles."""
        paths: list[str] = []
        for entry in getattr(cls, "_deformable_registry", []):
            paths.append(entry.sim_mesh_prim_path)
            paths.append(entry.vis_mesh_prim_path)
        return paths

    @classmethod
    def start_simulation(cls) -> None:
        super().start_simulation()
        cls._setup_fabric_particle_sync()

    @classmethod
    def _setup_fabric_particle_sync(cls):
        """Author ``newton:particleOffset``/``particleCount`` on each deformable
        visual-mesh prim so :meth:`NewtonManager.sync_particles_to_usd` pushes the
        simulated ``particle_q`` to the Kit viewport. Without this the soft body
        renders FROZEN at spawn (it is simulated, but never synced to Fabric).
        Mirrors the VBD/coupled managers, which do this in their start_simulation."""
        if getattr(cls, "_clone_physics_only", False) or not getattr(cls, "_deformable_registry", None):
            return
        import re

        try:
            import usdrt
            from isaaclab.sim.utils.stage import get_current_stage
        except Exception as exc:  # noqa: BLE001
            print(f"[stiffgipc] fabric particle sync setup skipped: {exc}", flush=True)
            return
        if NewtonManager._usdrt_stage is None:
            NewtonManager._usdrt_stage = get_current_stage(fabric=True)
        stage = get_current_stage()
        for entry in cls._deformable_registry:
            for inst_idx, offset in enumerate(entry.particle_offsets):
                resolved_vis = re.sub(r"(?<=[Ee]nv_)\.\*", str(inst_idx), entry.vis_mesh_prim_path)
                resolved_vis = re.sub(r"\.\*", str(inst_idx), resolved_vis)
                vis_prim = stage.GetPrimAtPath(resolved_vis)
                if not vis_prim or not vis_prim.IsValid():
                    continue
                fab_prim = NewtonManager._usdrt_stage.GetPrimAtPath(vis_prim.GetPath().pathString)
                fab_prim.CreateAttribute(
                    NewtonManager._newton_particle_offset_attr, usdrt.Sdf.ValueTypeNames.UInt, True
                )
                fab_prim.GetAttribute(NewtonManager._newton_particle_offset_attr).Set(offset)
                fab_prim.CreateAttribute(
                    NewtonManager._newton_particle_count_attr, usdrt.Sdf.ValueTypeNames.UInt, True
                )
                fab_prim.GetAttribute(NewtonManager._newton_particle_count_attr).Set(entry.particles_per_body)
        cls._mark_particles_dirty()
        cls.sync_particles_to_usd()

    @classmethod
    def add_contact_sensor(
        cls,
        body_names_expr: str | list[str] | None = None,
        shape_names_expr: str | list[str] | None = None,
        contact_partners_body_expr: str | list[str] | None = None,
        contact_partners_shape_expr: str | list[str] | None = None,
        verbose: bool = False,
    ) -> tuple[str | list[str] | None, str | list[str] | None, str | list[str] | None, str | list[str] | None]:
        """Add a ContactSensor view backed by StiffGIPC's internal IPC contacts."""
        if body_names_expr is None and shape_names_expr is None:
            raise ValueError("At least one of body_names_expr or shape_names_expr must be provided")
        if body_names_expr is not None and shape_names_expr is not None:
            raise ValueError("Only one of body_names_expr or shape_names_expr must be provided")
        if contact_partners_body_expr is not None and contact_partners_shape_expr is not None:
            raise ValueError("Only one of contact_partners_body_expr or contact_partners_shape_expr must be provided")
        def _hashable_key(x):
            return tuple(x) if isinstance(x, list) else x

        def _to_fnmatch(expr: str | list[str] | None) -> str | list[str] | None:
            if expr is None:
                return None
            if isinstance(expr, str):
                return expr.replace(".*", "*")
            return [p.replace(".*", "*") for p in expr]

        def _normalize_for_labels(expr: str | list[str] | None, labels: list[str]) -> str | list[str] | None:
            if expr is None or not labels:
                return expr
            label_has_paths = any("/" in label for label in labels)
            items = [expr] if isinstance(expr, str) else list(expr)
            expr_uses_paths = any("/" in pattern for pattern in items)
            if label_has_paths or not expr_uses_paths:
                return expr
            normalized = [pattern.rsplit("/", 1)[-1] for pattern in items]
            return normalized[0] if isinstance(expr, str) else normalized

        sensor_key = (
            _hashable_key(body_names_expr),
            _hashable_key(shape_names_expr),
            _hashable_key(contact_partners_body_expr),
            _hashable_key(contact_partners_shape_expr),
        )

        body_labels = list(cls._model.body_label)
        shape_labels = list(cls._model.shape_label)
        sensor = _StiffGIPCContactSensor(
            cls._model,
            NewtonManager._solver,
            sensing_obj_bodies=_normalize_for_labels(_to_fnmatch(body_names_expr), body_labels),
            sensing_obj_shapes=_normalize_for_labels(_to_fnmatch(shape_names_expr), shape_labels),
            counterpart_bodies=_normalize_for_labels(_to_fnmatch(contact_partners_body_expr), body_labels),
            counterpart_shapes=_normalize_for_labels(_to_fnmatch(contact_partners_shape_expr), shape_labels),
            measure_total=True,
            verbose=verbose,
        )

        NewtonManager._newton_contact_sensors[sensor_key] = sensor
        NewtonManager._report_contacts = True
        return sensor_key
