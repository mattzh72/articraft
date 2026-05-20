from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 1.35
BASE_W = 0.24
BASE_T = 0.035

SUPPORT_SPAN = 0.95
SUPPORT_LEN = 0.13
SUPPORT_FOOT_W = 0.19
SUPPORT_FOOT_T = 0.04
SUPPORT_WEB_W = 0.09
SUPPORT_HOUSING_R = 0.075
SUPPORT_AXIS_RISE = 0.20

SHAFT_R = 0.03
SHAFT_LEN = 1.16
FLANGE_R = 0.075
FLANGE_T = 0.02
COLLAR_R = 0.048
COLLAR_W = 0.03
COLLAR_X = 0.05
BORE_CLEARANCE = 0.0

SHAFT_AXIS_Z = BASE_T + SUPPORT_AXIS_RISE
SUPPORT_X = SUPPORT_SPAN / 2.0


def _build_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LEN, BASE_W, BASE_T)
        .translate((0.0, 0.0, BASE_T / 2.0))
    )
    return plate.edges("|Z").fillet(0.01)


def _build_support_shape() -> cq.Workplane:
    foot_z = -SUPPORT_AXIS_RISE + (SUPPORT_FOOT_T / 2.0)
    web_bottom = -SUPPORT_AXIS_RISE + SUPPORT_FOOT_T - 0.002
    web_top = -SUPPORT_HOUSING_R + 0.01
    web_h = web_top - web_bottom
    web_z = (web_top + web_bottom) / 2.0

    foot = (
        cq.Workplane("XY")
        .box(SUPPORT_LEN, SUPPORT_FOOT_W, SUPPORT_FOOT_T)
        .translate((0.0, 0.0, foot_z))
    )
    web = (
        cq.Workplane("XY")
        .box(SUPPORT_LEN * 0.92, SUPPORT_WEB_W, web_h)
        .translate((0.0, 0.0, web_z))
    )
    housing = (
        cq.Workplane("YZ")
        .circle(SUPPORT_HOUSING_R)
        .extrude(SUPPORT_LEN)
        .translate((-SUPPORT_LEN / 2.0, 0.0, 0.0))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(SHAFT_R + BORE_CLEARANCE)
        .extrude(SUPPORT_LEN + 0.04)
        .translate((-(SUPPORT_LEN + 0.04) / 2.0, 0.0, 0.0))
    )

    return foot.union(web).union(housing).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drive_shaft_fixture")

    model.material("base_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("support_cast", rgba=(0.46, 0.48, 0.50, 1.0))
    model.material("shaft_steel", rgba=(0.72, 0.74, 0.76, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "fixture_base"),
        material="base_steel",
        name="bed_plate",
    )

    left_support = model.part("left_support")
    left_support.visual(
        mesh_from_cadquery(
            _build_support_shape(),
            "left_support",
            tolerance=0.0002,
            angular_tolerance=0.05,
        ),
        material="support_cast",
        name="left_block",
    )

    right_support = model.part("right_support")
    right_support.visual(
        mesh_from_cadquery(
            _build_support_shape(),
            "right_support",
            tolerance=0.0002,
            angular_tolerance=0.05,
        ),
        material="support_cast",
        name="right_block",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_R, length=SHAFT_LEN),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="shaft_steel",
        name="main_shaft",
    )
    shaft.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_T),
        origin=Origin(
            xyz=(-(SHAFT_LEN / 2.0) + (FLANGE_T / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="shaft_steel",
        name="flange",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_W),
        origin=Origin(xyz=(COLLAR_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="shaft_steel",
        name="collar",
    )

    model.articulation(
        "base_to_left_support",
        ArticulationType.FIXED,
        parent=base,
        child=left_support,
        origin=Origin(xyz=(-SUPPORT_X, 0.0, SHAFT_AXIS_Z)),
    )
    model.articulation(
        "base_to_right_support",
        ArticulationType.FIXED,
        parent=base,
        child=right_support,
        origin=Origin(xyz=(SUPPORT_X, 0.0, SHAFT_AXIS_Z)),
    )
    model.articulation(
        "base_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    left_support = object_model.get_part("left_support")
    right_support = object_model.get_part("right_support")
    shaft = object_model.get_part("shaft")
    shaft_joint = object_model.get_articulation("base_to_shaft")

    ctx.allow_overlap(
        left_support,
        shaft,
        elem_a="left_block",
        elem_b="main_shaft",
        reason=(
            "The left support is a close-running journal support around the rotating "
            "shaft; the overlap signal comes from the mesh-backed bore sharing the "
            "same cylindrical surface as the shaft."
        ),
    )
    ctx.allow_overlap(
        right_support,
        shaft,
        elem_a="right_block",
        elem_b="main_shaft",
        reason=(
            "The right support is a close-running journal support around the rotating "
            "shaft; the overlap signal comes from the mesh-backed bore sharing the "
            "same cylindrical surface as the shaft."
        ),
    )

    ctx.expect_contact(left_support, base, name="left support is mounted to the bed plate")
    ctx.expect_contact(right_support, base, name="right support is mounted to the bed plate")
    ctx.expect_origin_gap(
        right_support,
        left_support,
        axis="x",
        min_gap=0.90,
        max_gap=1.00,
        name="end supports are widely spaced along the base",
    )
    ctx.expect_origin_distance(
        left_support,
        shaft,
        axes="yz",
        max_dist=0.001,
        name="left support bore shares the shaft centerline",
    )
    ctx.expect_origin_distance(
        right_support,
        shaft,
        axes="yz",
        max_dist=0.001,
        name="right support bore shares the shaft centerline",
    )

    left_aabb = ctx.part_world_aabb(left_support)
    right_aabb = ctx.part_world_aabb(right_support)
    flange_aabb = ctx.part_element_world_aabb(shaft, elem="flange")
    collar_aabb = ctx.part_element_world_aabb(shaft, elem="collar")

    flange_ok = (
        left_aabb is not None
        and flange_aabb is not None
        and flange_aabb[1][0] < left_aabb[0][0] - 0.01
    )
    ctx.check(
        "flange sits outboard of the left support",
        flange_ok,
        details=f"left_support={left_aabb}, flange={flange_aabb}",
    )

    collar_center_x = None
    if collar_aabb is not None:
        collar_center_x = (collar_aabb[0][0] + collar_aabb[1][0]) / 2.0
    collar_ok = (
        left_aabb is not None
        and right_aabb is not None
        and collar_center_x is not None
        and left_aabb[1][0] + 0.05 < collar_center_x < right_aabb[0][0] - 0.05
    )
    ctx.check(
        "collar remains near the middle of the supported span",
        collar_ok,
        details=(
            f"left_support={left_aabb}, right_support={right_aabb}, "
            f"collar={collar_aabb}"
        ),
    )

    joint_ok = (
        shaft_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(shaft_joint.axis) == (1.0, 0.0, 0.0)
        and shaft_joint.motion_limits is not None
        and shaft_joint.motion_limits.lower is None
        and shaft_joint.motion_limits.upper is None
    )
    ctx.check(
        "shaft uses one continuous revolute joint about the shared support axis",
        joint_ok,
        details=(
            f"type={shaft_joint.articulation_type}, axis={shaft_joint.axis}, "
            f"limits={shaft_joint.motion_limits}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
