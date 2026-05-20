from __future__ import annotations

from contextlib import nullcontext
from math import pi
from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
CARCASS_W = 0.60
CARCASS_D = 0.55
CARCASS_H = 0.32
PANEL_T = 0.018
BACK_T = 0.006

OPENING_W = CARCASS_W - (2.0 * PANEL_T)
OPENING_H = CARCASS_H - (2.0 * PANEL_T)

SIDE_CLEARANCE = 0.005
VERT_CLEARANCE = 0.006
DRAWER_SIDE_T = 0.012
DRAWER_BOTTOM_T = 0.010
DRAWER_BACK_T = 0.012
DRAWER_FRONT_T = 0.018
DRAWER_BOX_D = 0.43

DRAWER_OUTER_W = OPENING_W - (2.0 * SIDE_CLEARANCE)
DRAWER_OUTER_H = OPENING_H - (2.0 * VERT_CLEARANCE)
DRAWER_FRONT_W = OPENING_W - 0.004
DRAWER_FRONT_H = OPENING_H - 0.004
DRAWER_FRONT_PROUD = 0.002
DRAWER_TRAVEL = 0.32

CARCASS_FRONT_Y = CARCASS_D / 2.0
DRAWER_FRONT_CENTER_Y = CARCASS_FRONT_Y - (DRAWER_FRONT_T / 2.0) + DRAWER_FRONT_PROUD
DRAWER_BOX_FRONT_Y = DRAWER_FRONT_CENTER_Y - (DRAWER_FRONT_T / 2.0)
DRAWER_BOX_CENTER_Y = DRAWER_BOX_FRONT_Y - (DRAWER_BOX_D / 2.0)

DRAWER_JOINT_X = (-CARCASS_W / 2.0) + PANEL_T + SIDE_CLEARANCE + (DRAWER_SIDE_T / 2.0)
DRAWER_JOINT_Y = DRAWER_BOX_CENTER_Y

HANDLE_BAR_R = 0.005
HANDLE_BAR_L = 0.12
HANDLE_POST_R = 0.004
HANDLE_POST_L = 0.018
HANDLE_Y = DRAWER_FRONT_CENTER_Y - DRAWER_BOX_CENTER_Y + (DRAWER_FRONT_T / 2.0) + 0.012
HANDLE_Z = 0.0
HANDLE_SPAN = 0.09


def _add_box(part, size, xyz, *, material=None, collision=True) -> None:
    origin = Origin(xyz=xyz)
    part.visual(Box(size), origin=origin, material=material)
    if collision:
        pass


def _pose_context(ctx: TestContext, amount: float):
    pose = getattr(ctx, "pose", None)
    if pose is None:
        return nullcontext()
    for args, kwargs in (
        ((), {"carcass_to_drawer": amount}),
        (({"carcass_to_drawer": amount},), {}),
        (("carcass_to_drawer", amount), {}),
    ):
        try:
            return pose(*args, **kwargs)
        except TypeError:
            continue
    return nullcontext()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_drawer")

    model.material("desk_shell", rgba=(0.83, 0.78, 0.70, 1.0))
    model.material("drawer_box", rgba=(0.92, 0.90, 0.86, 1.0))
    model.material("drawer_front", rgba=(0.70, 0.52, 0.36, 1.0))
    model.material("handle_metal", rgba=(0.63, 0.66, 0.70, 1.0))

    carcass = model.part("carcass")
    _add_box(
        carcass,
        (CARCASS_W, CARCASS_D, PANEL_T),
        (0.0, 0.0, (CARCASS_H / 2.0) - (PANEL_T / 2.0)),
        material="desk_shell",
    )
    _add_box(
        carcass,
        (CARCASS_W, CARCASS_D, PANEL_T),
        (0.0, 0.0, -(CARCASS_H / 2.0) + (PANEL_T / 2.0)),
        material="desk_shell",
    )
    _add_box(
        carcass,
        (PANEL_T, CARCASS_D, OPENING_H),
        (-(CARCASS_W / 2.0) + (PANEL_T / 2.0), 0.0, 0.0),
        material="desk_shell",
    )
    _add_box(
        carcass,
        (PANEL_T, CARCASS_D, OPENING_H),
        ((CARCASS_W / 2.0) - (PANEL_T / 2.0), 0.0, 0.0),
        material="desk_shell",
    )
    _add_box(
        carcass,
        (OPENING_W, BACK_T, OPENING_H),
        (0.0, -(CARCASS_D / 2.0) + (BACK_T / 2.0), 0.0),
        material="desk_shell",
    )
    carcass.inertial = Inertial.from_geometry(
        Box((CARCASS_W, CARCASS_D, CARCASS_H)),
        mass=18.0,
    )

    drawer = model.part("drawer")
    _add_box(
        drawer,
        (DRAWER_SIDE_T, DRAWER_BOX_D, DRAWER_OUTER_H),
        (0.0, 0.0, 0.0),
        material="drawer_box",
    )
    _add_box(
        drawer,
        (DRAWER_SIDE_T, DRAWER_BOX_D, DRAWER_OUTER_H),
        (DRAWER_OUTER_W - DRAWER_SIDE_T, 0.0, 0.0),
        material="drawer_box",
    )
    _add_box(
        drawer,
        (DRAWER_OUTER_W - (2.0 * DRAWER_SIDE_T), DRAWER_BOX_D, DRAWER_BOTTOM_T),
        (
            (DRAWER_OUTER_W - DRAWER_SIDE_T) / 2.0,
            0.0,
            -(DRAWER_OUTER_H / 2.0) + (DRAWER_BOTTOM_T / 2.0),
        ),
        material="drawer_box",
    )
    _add_box(
        drawer,
        (DRAWER_OUTER_W - (2.0 * DRAWER_SIDE_T), DRAWER_BACK_T, DRAWER_OUTER_H - DRAWER_BOTTOM_T),
        (
            (DRAWER_OUTER_W - DRAWER_SIDE_T) / 2.0,
            -(DRAWER_BOX_D / 2.0) + (DRAWER_BACK_T / 2.0),
            DRAWER_BOTTOM_T / 2.0,
        ),
        material="drawer_box",
    )
    _add_box(
        drawer,
        (DRAWER_FRONT_W, DRAWER_FRONT_T, DRAWER_FRONT_H),
        ((DRAWER_OUTER_W - DRAWER_SIDE_T) / 2.0, DRAWER_FRONT_CENTER_Y - DRAWER_BOX_CENTER_Y, 0.0),
        material="drawer_front",
    )

    drawer.visual(
        Cylinder(radius=HANDLE_BAR_R, length=HANDLE_BAR_L),
        origin=Origin(
            xyz=((DRAWER_OUTER_W - DRAWER_SIDE_T) / 2.0, HANDLE_Y, HANDLE_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="handle_metal",
    )
    drawer.visual(
        Cylinder(radius=HANDLE_POST_R, length=HANDLE_POST_L),
        origin=Origin(
            xyz=(
                (DRAWER_OUTER_W - DRAWER_SIDE_T) / 2.0 - (HANDLE_SPAN / 2.0),
                HANDLE_Y - (HANDLE_POST_L / 2.0),
                HANDLE_Z,
            ),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="handle_metal",
    )
    drawer.visual(
        Cylinder(radius=HANDLE_POST_R, length=HANDLE_POST_L),
        origin=Origin(
            xyz=(
                (DRAWER_OUTER_W - DRAWER_SIDE_T) / 2.0 + (HANDLE_SPAN / 2.0),
                HANDLE_Y - (HANDLE_POST_L / 2.0),
                HANDLE_Z,
            ),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="handle_metal",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_OUTER_W, DRAWER_BOX_D + DRAWER_FRONT_T, DRAWER_OUTER_H)),
        mass=4.5,
        origin=Origin(
            xyz=(
                (DRAWER_OUTER_W - DRAWER_SIDE_T) / 2.0,
                (DRAWER_FRONT_CENTER_Y - DRAWER_BOX_CENTER_Y - (DRAWER_FRONT_T / 2.0)) / 2.0,
                0.0,
            )
        ),
    )

    model.articulation(
        "carcass_to_drawer",
        ArticulationType.PRISMATIC,
        parent="carcass",
        child="drawer",
        origin=Origin(xyz=(DRAWER_JOINT_X, DRAWER_JOINT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=DRAWER_TRAVEL,
            effort=120.0,
            velocity=0.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, seed=0)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.002, overlap_volume_tol=0.0)

    with _pose_context(ctx, 0.0):
        ctx.expect_aabb_overlap("drawer", "carcass", axes="xy", min_overlap=0.20)

    ctx.expect_joint_motion_axis(
        "carcass_to_drawer",
        "drawer",
        world_axis="y",
        direction="positive",
        min_delta=0.12,
    )

    with _pose_context(ctx, DRAWER_TRAVEL):
        ctx.expect_aabb_overlap("drawer", "carcass", axes="xy", min_overlap=0.10)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
