from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_LEN = 0.31
PLATE_W = 0.09
PLATE_T = 0.014
PLATE_Z = 0.112
MOUNT_HOLE_D = 0.011
MOUNT_HOLE_X = 0.11
MOUNT_HOLE_Y = 0.025

BEARING_X = 0.082
HOUSING_LEN = 0.034
HOUSING_W = 0.058
HOUSING_H = 0.05
BORE_R = 0.0125
STRAP_X = 0.018
STRAP_Y = 0.01
STRAP_Y_OFFSET = 0.024
SUPPORT_OVERLAP = 0.002

SHAFT_R = 0.011
SHAFT_LEN = 0.24
HUB_R = 0.016
HUB_LEN = 0.046
COLLAR_R = 0.018
COLLAR_LEN = 0.012
COLLAR_X = 0.108
FLAG_LEN = 0.032
FLAG_W = 0.01
FLAG_H = 0.008
FLAG_Z = SHAFT_R + FLAG_H / 2.0 - 0.001
PAD_LEN = 0.022
PAD_W = 0.016
PAD_H = 0.005
PAD_Z = -SHAFT_R - PAD_H / 2.0


def _make_top_plate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(SUPPORT_LEN, PLATE_W, PLATE_T)
        .edges("|Z")
        .fillet(0.004)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-MOUNT_HOLE_X, -MOUNT_HOLE_Y),
                (-MOUNT_HOLE_X, MOUNT_HOLE_Y),
                (MOUNT_HOLE_X, -MOUNT_HOLE_Y),
                (MOUNT_HOLE_X, MOUNT_HOLE_Y),
            ]
        )
        .hole(MOUNT_HOLE_D)
    )


def _make_hanging_bearing(x_center: float) -> cq.Workplane:
    housing = (
        cq.Workplane("YZ", origin=(x_center, 0.0, 0.0))
        .rect(HOUSING_W, HOUSING_H)
        .extrude(HOUSING_LEN / 2.0, both=True)
        .edges("|X")
        .fillet(0.004)
    )
    bore = (
        cq.Workplane("YZ", origin=(x_center, 0.0, 0.0))
        .circle(BORE_R)
        .extrude(HOUSING_LEN / 2.0 + 0.004, both=True)
    )
    strap_bottom = HOUSING_H / 2.0 - SUPPORT_OVERLAP
    strap_top = PLATE_Z - PLATE_T / 2.0 + SUPPORT_OVERLAP
    strap_h = strap_top - strap_bottom
    strap_z = (strap_top + strap_bottom) / 2.0

    assembly = housing.cut(bore)
    for y_center in (-STRAP_Y_OFFSET, STRAP_Y_OFFSET):
        strap = (
            cq.Workplane("XY")
            .box(STRAP_X, STRAP_Y, strap_h)
            .translate((x_center, y_center, strap_z))
        )
        assembly = assembly.union(strap)
    return assembly


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_rotary_shaft_unit")

    support_color = model.material("support_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    shaft_color = model.material("shaft_steel", rgba=(0.78, 0.8, 0.83, 1.0))
    flag_color = model.material("shaft_key", rgba=(0.55, 0.57, 0.6, 1.0))
    pad_color = model.material("bearing_liner", rgba=(0.62, 0.52, 0.28, 1.0))

    support = model.part("support_bracket")
    support.visual(
        mesh_from_cadquery(_make_top_plate(), "top_plate"),
        origin=Origin(xyz=(0.0, 0.0, PLATE_Z)),
        material=support_color,
        name="top_plate",
    )
    support.visual(
        mesh_from_cadquery(_make_hanging_bearing(-BEARING_X), "left_hanger_bearing"),
        material=support_color,
        name="left_hanger_bearing",
    )
    support.visual(
        mesh_from_cadquery(_make_hanging_bearing(BEARING_X), "right_hanger_bearing"),
        material=support_color,
        name="right_hanger_bearing",
    )
    support.visual(
        Box((PAD_LEN, PAD_W, PAD_H)),
        origin=Origin(xyz=(-BEARING_X, 0.0, PAD_Z)),
        material=pad_color,
        name="left_bearing_pad",
    )
    support.visual(
        Box((PAD_LEN, PAD_W, PAD_H)),
        origin=Origin(xyz=(BEARING_X, 0.0, PAD_Z)),
        material=pad_color,
        name="right_bearing_pad",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_R, length=SHAFT_LEN),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_color,
        name="shaft_body",
    )
    shaft.visual(
        Cylinder(radius=HUB_R, length=HUB_LEN),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_color,
        name="center_hub",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(-COLLAR_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_color,
        name="left_collar",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(COLLAR_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_color,
        name="right_collar",
    )
    shaft.visual(
        Box((FLAG_LEN, FLAG_W, FLAG_H)),
        origin=Origin(xyz=(0.0, 0.0, FLAG_Z)),
        material=flag_color,
        name="indicator_flag",
    )

    model.articulation(
        "support_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=12.0),
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
    support = object_model.get_part("support_bracket")
    shaft = object_model.get_part("shaft")
    joint = object_model.get_articulation("support_to_shaft")

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3)) if aabb else None

    ctx.check(
        "shaft joint is continuous about the supported centerline",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and joint.axis == (1.0, 0.0, 0.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower is None
        and joint.motion_limits.upper is None,
        details=(
            f"type={joint.articulation_type}, axis={joint.axis}, "
            f"limits={joint.motion_limits}"
        ),
    )

    with ctx.pose({joint: 0.0}):
        ctx.expect_origin_distance(
            support,
            shaft,
            axes="yz",
            max_dist=1e-6,
            name="shaft axis stays centered in the hanging supports",
        )
        ctx.expect_gap(
            support,
            shaft,
            axis="z",
            positive_elem="top_plate",
            negative_elem="shaft_body",
            min_gap=0.07,
            name="shaft hangs visibly below the top bracket",
        )
        ctx.expect_gap(
            support,
            shaft,
            axis="z",
            positive_elem="top_plate",
            negative_elem="indicator_flag",
            min_gap=0.06,
            name="rotating key clears the top bracket at rest",
        )
        ctx.expect_contact(
            support,
            shaft,
            elem_a="left_bearing_pad",
            elem_b="shaft_body",
            name="left hanging bearing actually supports the shaft",
        )
        ctx.expect_contact(
            support,
            shaft,
            elem_a="right_bearing_pad",
            elem_b="shaft_body",
            name="right hanging bearing actually supports the shaft",
        )

        left_aabb = ctx.part_element_world_aabb(support, elem="left_hanger_bearing")
        right_aabb = ctx.part_element_world_aabb(support, elem="right_hanger_bearing")
        left_center = aabb_center(left_aabb)
        right_center = aabb_center(right_aabb)
        ctx.check(
            "paired hanging bearings straddle the shaft symmetrically",
            left_center is not None
            and right_center is not None
            and left_center[0] < -0.06
            and right_center[0] > 0.06
            and abs(left_center[1] - right_center[1]) < 1e-6
            and abs(left_center[2] - right_center[2]) < 1e-6,
            details=f"left_center={left_center}, right_center={right_center}",
        )

    with ctx.pose({joint: 0.0}):
        rest_flag = aabb_center(ctx.part_element_world_aabb(shaft, elem="indicator_flag"))
    with ctx.pose({joint: math.pi / 2.0}):
        quarter_flag = aabb_center(ctx.part_element_world_aabb(shaft, elem="indicator_flag"))
        ctx.expect_gap(
            support,
            shaft,
            axis="z",
            positive_elem="top_plate",
            negative_elem="indicator_flag",
            min_gap=0.09,
            name="rotating key remains clear of the support at quarter turn",
        )
    ctx.check(
        "shaft flag swings around the x-axis when posed",
        rest_flag is not None
        and quarter_flag is not None
        and rest_flag[2] > 0.013
        and quarter_flag[1] < -0.010
        and abs(quarter_flag[2]) < 0.006,
        details=f"rest_flag={rest_flag}, quarter_flag={quarter_flag}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
