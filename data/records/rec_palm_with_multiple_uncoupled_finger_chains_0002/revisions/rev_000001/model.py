from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
PALM_LENGTH = 0.050
PALM_WIDTH = 0.082
PALM_THICKNESS = 0.008

TAB_LENGTH = 0.008
TAB_WIDTH = 0.012
TAB_OVERLAP = 0.0015
HINGE_X = PALM_LENGTH / 2.0 + TAB_LENGTH / 2.0 - TAB_OVERLAP

STANDOFF_RADIUS = 0.0035
STANDOFF_HEIGHT = 0.003

FINGER_THICKNESS = 0.006
FINGER_BASE_RADIUS = 0.006
FINGER_BODY_LENGTH = 0.030
FINGER_BODY_WIDTH = 0.009
FINGER_TIP_RADIUS = 0.0045
FINGER_TIP_X = 0.036
FINGER_BASE_COLLISION_SIZE = 2.0 * FINGER_BASE_RADIUS
FINGER_TIP_COLLISION_SIZE = 2.0 * FINGER_TIP_RADIUS
FINGER_COLLISION_LENGTH = 0.028
FINGER_COLLISION_WIDTH = 0.0065
FINGER_COLLISION_CENTER_X = 0.021
FINGER_VISUAL_MIN_X = -FINGER_BASE_RADIUS
FINGER_VISUAL_MAX_X = FINGER_TIP_X + FINGER_TIP_RADIUS
FINGER_VISUAL_LENGTH = FINGER_VISUAL_MAX_X - FINGER_VISUAL_MIN_X
FINGER_VISUAL_CENTER_X = (FINGER_VISUAL_MAX_X + FINGER_VISUAL_MIN_X) / 2.0
FINGER_VISUAL_WIDTH = max(FINGER_BASE_COLLISION_SIZE, FINGER_BODY_WIDTH, FINGER_TIP_COLLISION_SIZE)

FINGER_STACK_Z = PALM_THICKNESS / 2.0 + FINGER_THICKNESS / 2.0 + 0.0005

PALM_MASS = 0.22
FINGER_MASS = 0.028

FINGER_SPECS = (
    {
        "name": "finger_top_outer",
        "joint": "palm_to_finger_top_outer",
        "y": 0.0285,
        "yaw": 0.32,
        "axis": (0.0, 0.0, -1.0),
        "upper": 0.22,
        "direction": "negative",
        "motion_axis": "y",
        "min_delta": 0.003,
    },
    {
        "name": "finger_top_inner",
        "joint": "palm_to_finger_top_inner",
        "y": 0.010,
        "yaw": 0.15,
        "axis": (0.0, 0.0, -1.0),
        "upper": 0.09,
        "direction": "negative",
        "motion_axis": "y",
        "min_delta": 0.001,
    },
    {
        "name": "finger_bottom_inner",
        "joint": "palm_to_finger_bottom_inner",
        "y": -0.010,
        "yaw": -0.15,
        "axis": (0.0, 0.0, 1.0),
        "upper": 0.09,
        "direction": "positive",
        "motion_axis": "y",
        "min_delta": 0.001,
    },
    {
        "name": "finger_bottom_outer",
        "joint": "palm_to_finger_bottom_outer",
        "y": -0.0285,
        "yaw": -0.32,
        "axis": (0.0, 0.0, 1.0),
        "upper": 0.22,
        "direction": "positive",
        "motion_axis": "y",
        "min_delta": 0.003,
    },
)


def _make_palm_shape() -> cq.Workplane:
    palm = cq.Workplane("XY").box(PALM_LENGTH, PALM_WIDTH, PALM_THICKNESS)
    tab_center_x = PALM_LENGTH / 2.0 + TAB_LENGTH / 2.0 - TAB_OVERLAP

    for spec in FINGER_SPECS:
        tab = (
            cq.Workplane("XY")
            .box(TAB_LENGTH, TAB_WIDTH, PALM_THICKNESS)
            .translate((tab_center_x, spec["y"], 0.0))
        )
        standoff = (
            cq.Workplane("XY")
            .circle(STANDOFF_RADIUS)
            .extrude(STANDOFF_HEIGHT)
            .translate((HINGE_X, spec["y"], PALM_THICKNESS / 2.0))
        )
        palm = palm.union(tab).union(standoff)

    return palm


def _make_finger_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(FINGER_BASE_RADIUS).extrude(FINGER_THICKNESS)
    body = (
        cq.Workplane("XY")
        .box(FINGER_BODY_LENGTH, FINGER_BODY_WIDTH, FINGER_THICKNESS)
        .translate((FINGER_COLLISION_CENTER_X, 0.0, FINGER_THICKNESS / 2.0))
    )
    tip = (
        cq.Workplane("XY")
        .circle(FINGER_TIP_RADIUS)
        .extrude(FINGER_THICKNESS)
        .translate((FINGER_TIP_X, 0.0, 0.0))
    )
    return base.union(body).union(tip).translate((0.0, 0.0, -FINGER_THICKNESS / 2.0))


def _add_palm_collisions(palm_part) -> None:


    tab_collision_length = TAB_LENGTH - 0.001
    tab_collision_width = TAB_WIDTH - 0.002
    tab_collision_center_x = PALM_LENGTH / 2.0 + tab_collision_length / 2.0 - TAB_OVERLAP

    for index, spec in enumerate(FINGER_SPECS):
        pass


def _configure_finger_part(finger_part, finger_mesh) -> None:
    finger_part.visual(finger_mesh, material="finger_plate")



    finger_part.inertial = Inertial.from_geometry(
        Box((FINGER_VISUAL_LENGTH, FINGER_VISUAL_WIDTH, FINGER_THICKNESS)),
        mass=FINGER_MASS,
        origin=Origin(xyz=(FINGER_VISUAL_CENTER_X, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_finger_end_effector", assets=ASSETS)

    model.material("palm_plate", rgba=(0.60, 0.64, 0.68, 1.0))
    model.material("finger_plate", rgba=(0.16, 0.17, 0.20, 1.0))

    palm_mesh = mesh_from_cadquery(_make_palm_shape(), "palm_plate.obj", assets=ASSETS)
    finger_mesh = mesh_from_cadquery(
        _make_finger_shape(),
        "finger_plate.obj",
        assets=ASSETS,
    )

    palm = model.part("palm")
    palm.visual(palm_mesh, material="palm_plate")
    _add_palm_collisions(palm)
    palm.inertial = Inertial.from_geometry(
        Box((PALM_LENGTH, PALM_WIDTH, PALM_THICKNESS)),
        mass=PALM_MASS,
    )

    for spec in FINGER_SPECS:
        finger = model.part(spec["name"])
        _configure_finger_part(finger, finger_mesh)

        model.articulation(
            spec["joint"],
            ArticulationType.REVOLUTE,
            parent=palm,
            child=finger,
            origin=Origin(
                xyz=(HINGE_X, spec["y"], FINGER_STACK_Z),
                rpy=(0.0, 0.0, spec["yaw"]),
            ),
            axis=spec["axis"],
            motion_limits=MotionLimits(
                lower=0.0,
                upper=spec["upper"],
                effort=2.0,
                velocity=3.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    for spec in FINGER_SPECS:
        ctx.expect_origin_gap(spec["name"], "palm", axis="z", min_gap=0.0)
        ctx.expect_aabb_gap(spec["name"], "palm", axis="z", max_gap=0.002, max_penetration=0.0)
        ctx.expect_origin_distance(spec["name"], "palm", axes="xy", max_dist=0.065)
        ctx.expect_joint_motion_axis(
            spec["joint"],
            spec["name"],
            world_axis=spec["motion_axis"],
            direction=spec["direction"],
            min_delta=spec["min_delta"],
        )

    ctx.expect_origin_distance("finger_top_outer", "finger_top_inner", axes="xy", max_dist=0.03)
    ctx.expect_origin_distance("finger_bottom_outer", "finger_bottom_inner", axes="xy", max_dist=0.03)
    ctx.expect_origin_distance("finger_top_inner", "finger_bottom_inner", axes="xy", max_dist=0.03)
    ctx.expect_origin_distance("finger_top_outer", "finger_bottom_outer", axes="xy", max_dist=0.07)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
