from __future__ import annotations

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
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
BASE_WIDTH = 0.46
BASE_DEPTH = 0.34
LOWER_BODY_HEIGHT = 0.18
UPPER_BODY_HEIGHT = 0.10
UPPER_BODY_WIDTH = 0.36
UPPER_BODY_DEPTH = 0.26
PAN_OUTER_RADIUS = 0.16
PAN_INNER_RADIUS = 0.045
PAN_HEIGHT = 0.022
COLLAR_RADIUS = 0.018
COLLAR_HEIGHT = 0.028

SHAFT_RADIUS = 0.014
SHAFT_HEIGHT = 0.022
COUPLING_LUG_SIZE = (0.012, 0.008, 0.010)
COUPLING_LUG_ORIGIN = (0.020, 0.0, 0.007)

WHEELHEAD_RADIUS = 0.14
WHEELHEAD_THICKNESS = 0.020
WHEELHEAD_HUB_RADIUS = 0.030
WHEELHEAD_HUB_HEIGHT = 0.018
GROOVE_DEPTH = 0.0015
GROOVE_RADII = (
    (0.115, 0.111),
    (0.090, 0.086),
    (0.065, 0.061),
)

BASE_TOP_Z = LOWER_BODY_HEIGHT + UPPER_BODY_HEIGHT
COLLAR_TOP_Z = BASE_TOP_Z + COLLAR_HEIGHT


def _make_base_visual_mesh():
    import cadquery as cq

    lower_body = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, LOWER_BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.03)
    )
    upper_body = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_BODY_HEIGHT)
        .box(
            UPPER_BODY_WIDTH,
            UPPER_BODY_DEPTH,
            UPPER_BODY_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.02)
    )
    splash_pan = (
        cq.Workplane("XY")
        .workplane(offset=BASE_TOP_Z)
        .circle(PAN_OUTER_RADIUS)
        .circle(PAN_INNER_RADIUS)
        .extrude(PAN_HEIGHT)
    )
    spindle_collar = (
        cq.Workplane("XY").workplane(offset=BASE_TOP_Z).circle(COLLAR_RADIUS).extrude(COLLAR_HEIGHT)
    )
    base_shape = lower_body.union(upper_body).union(splash_pan).union(spindle_collar)
    return mesh_from_cadquery(base_shape, MESH_DIR / "pottery_wheel_base.obj")


def _make_wheelhead_visual_mesh():
    import cadquery as cq

    wheelhead = cq.Workplane("XY").circle(WHEELHEAD_HUB_RADIUS).extrude(WHEELHEAD_HUB_HEIGHT)
    wheelhead = wheelhead.union(
        cq.Workplane("XY")
        .workplane(offset=WHEELHEAD_HUB_HEIGHT)
        .circle(WHEELHEAD_RADIUS)
        .extrude(WHEELHEAD_THICKNESS)
    )
    for outer_radius, inner_radius in GROOVE_RADII:
        wheelhead = (
            wheelhead.faces(">Z")
            .workplane()
            .circle(outer_radius)
            .circle(inner_radius)
            .cutBlind(-GROOVE_DEPTH)
        )
    return mesh_from_cadquery(wheelhead, MESH_DIR / "pottery_wheel_head.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pottery_wheel", assets=ASSETS)

    model.material("body_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("wheelhead", rgba=(0.80, 0.82, 0.84, 1.0))

    base = model.part("base")
    base.visual(_make_base_visual_mesh(), material="body_gray")
    base.inertial = Inertial.from_geometry(
        Box((0.44, 0.32, 0.31)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT / 2.0)),
        material="steel",
    )
    shaft.visual(
        Box(COUPLING_LUG_SIZE),
        origin=Origin(xyz=COUPLING_LUG_ORIGIN),
        material="steel",
    )


    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT / 2.0)),
    )

    wheelhead = model.part("wheelhead")
    wheelhead.visual(_make_wheelhead_visual_mesh(), material="wheelhead")
    wheelhead.inertial = Inertial.from_geometry(
        Cylinder(
            radius=WHEELHEAD_RADIUS,
            length=WHEELHEAD_HUB_HEIGHT + WHEELHEAD_THICKNESS,
        ),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (WHEELHEAD_HUB_HEIGHT + WHEELHEAD_THICKNESS) / 2.0)),
    )

    model.articulation(
        "base_to_shaft",
        ArticulationType.CONTINUOUS,
        parent="base",
        child="shaft",
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=12.0),
    )
    model.articulation(
        "shaft_to_wheelhead",
        ArticulationType.FIXED,
        parent="shaft",
        child="wheelhead",
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_distance("shaft", "base", axes="xy", max_dist=0.006)
    ctx.expect_aabb_overlap("shaft", "base", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_gap("shaft", "base", axis="z", max_gap=0.002, max_penetration=0.0)

    ctx.expect_origin_distance("wheelhead", "shaft", axes="xy", max_dist=0.006)
    ctx.expect_aabb_overlap("wheelhead", "shaft", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_gap("wheelhead", "shaft", axis="z", max_gap=0.002, max_penetration=0.0)

    ctx.expect_origin_gap("wheelhead", "base", axis="z", min_gap=0.015)
    ctx.expect_origin_distance("wheelhead", "base", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("wheelhead", "base", axes="xy", min_overlap=0.20)
    ctx.expect_aabb_gap("wheelhead", "base", axis="z", max_gap=0.03, max_penetration=0.0)
    ctx.expect_origin_gap("shaft", "base", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("wheelhead", "shaft", axis="z", min_gap=0.0)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
