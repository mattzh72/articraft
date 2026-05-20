from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


LOWER_AXIS_HEIGHT = 0.12
LOWER_PLATTER_RADIUS = 0.22
LOWER_PLATTER_THICKNESS = 0.03
LOWER_SLEEVE_OUTER_RADIUS = 0.085
LOWER_SLEEVE_INNER_RADIUS = 0.058
LOWER_SLEEVE_LENGTH = 0.02

UPPER_AXIS_X = 0.155
UPPER_HOUSING_OUTER_RADIUS = 0.04
UPPER_HOUSING_INNER_RADIUS = 0.026
UPPER_HOUSING_HEIGHT = 0.062
UPPER_AXIS_Z = 0.34
UPPER_TURNTABLE_RADIUS = 0.10
UPPER_TURNTABLE_THICKNESS = 0.022
UPPER_SPINDLE_RADIUS = 0.024
UPPER_SPINDLE_LENGTH = 0.075


def _make_base_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.18)
        .extrude(0.035)
        .faces(">Z")
        .workplane()
        .circle(0.16)
        .extrude(0.045)
        .faces(">Z")
        .workplane()
        .circle(0.14)
        .extrude(0.02)
    )


def _make_lower_platter() -> cq.Workplane:
    platter = cq.Workplane("XY").circle(LOWER_PLATTER_RADIUS).extrude(LOWER_PLATTER_THICKNESS)
    platter = (
        platter.faces(">Z")
        .workplane()
        .circle(0.185)
        .circle(0.075)
        .cutBlind(-0.004)
    )
    return platter


def _make_annular_sleeve(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def _make_column_support() -> cq.Workplane:
    side_profile = (
        cq.Workplane("XZ")
        .moveTo(-0.04, 0.0)
        .lineTo(-0.048, 0.018)
        .lineTo(-0.048, 0.075)
        .lineTo(-0.037, 0.16)
        .lineTo(-0.032, 0.248)
        .lineTo(0.032, 0.248)
        .lineTo(0.037, 0.16)
        .lineTo(0.048, 0.075)
        .lineTo(0.048, 0.018)
        .lineTo(0.04, 0.0)
        .close()
        .extrude(0.07)
        .translate((0.0, -0.035, 0.0))
    )
    lightening_window = (
        cq.Workplane("XZ")
        .center(0.0, 0.12)
        .rect(0.03, 0.10)
        .extrude(0.08)
        .translate((0.0, -0.04, 0.0))
    )
    top_pad = (
        cq.Workplane("XY")
        .box(0.056, 0.066, 0.014, centered=(True, True, False))
        .translate((0.0, 0.0, 0.234))
    )
    top_relief = (
        cq.Workplane("XY")
        .circle(0.03)
        .extrude(0.07)
        .translate((0.0, 0.0, 0.19))
    )
    return side_profile.cut(lightening_window).union(top_pad).cut(top_relief)


def _make_upper_turntable() -> cq.Workplane:
    disk = cq.Workplane("XY").circle(UPPER_TURNTABLE_RADIUS).extrude(UPPER_TURNTABLE_THICKNESS)
    disk = (
        disk.faces(">Z")
        .workplane()
        .circle(0.072)
        .circle(0.028)
        .cutBlind(-0.003)
    )
    lug = (
        cq.Workplane("XY")
        .box(0.05, 0.028, 0.014, centered=(False, True, False))
        .translate((0.09, 0.0, 0.004))
    )
    return disk.union(lug)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_offset_rotary_head")

    model.material("pedestal_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("carrier_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("bearing_steel", rgba=(0.76, 0.79, 0.82, 1.0))
    model.material("upper_table_gray", rgba=(0.70, 0.73, 0.76, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        mesh_from_cadquery(_make_base_shell(), "pedestal_base_shell"),
        material="pedestal_dark",
        name="base_shell",
    )
    pedestal_base.visual(
        Cylinder(radius=0.055, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material="bearing_steel",
        name="bearing_post",
    )

    lower_carrier = model.part("lower_carrier")
    lower_carrier.visual(
        mesh_from_cadquery(_make_lower_platter(), "lower_platter"),
        material="carrier_gray",
        name="lower_platter",
    )
    lower_carrier.visual(
        mesh_from_cadquery(
            _make_annular_sleeve(
                LOWER_SLEEVE_OUTER_RADIUS,
                LOWER_SLEEVE_INNER_RADIUS,
                LOWER_SLEEVE_LENGTH,
            ),
            "lower_bearing_sleeve",
        ),
        origin=Origin(xyz=(0.0, 0.0, -LOWER_SLEEVE_LENGTH)),
        material="bearing_steel",
        name="lower_sleeve",
    )
    lower_carrier.visual(
        mesh_from_cadquery(_make_column_support(), "column_support"),
        origin=Origin(xyz=(UPPER_AXIS_X, 0.0, LOWER_PLATTER_THICKNESS)),
        material="pedestal_dark",
        name="column_support",
    )
    lower_carrier.visual(
        mesh_from_cadquery(
            _make_annular_sleeve(
                UPPER_HOUSING_OUTER_RADIUS,
                UPPER_HOUSING_INNER_RADIUS,
                UPPER_HOUSING_HEIGHT,
            ),
            "upper_bearing_housing",
        ),
        origin=Origin(xyz=(UPPER_AXIS_X, 0.0, UPPER_AXIS_Z - UPPER_HOUSING_HEIGHT)),
        material="bearing_steel",
        name="upper_housing",
    )

    upper_turntable = model.part("upper_turntable")
    upper_turntable.visual(
        mesh_from_cadquery(_make_upper_turntable(), "upper_turntable"),
        material="upper_table_gray",
        name="upper_disk",
    )
    upper_turntable.visual(
        Cylinder(radius=UPPER_SPINDLE_RADIUS, length=UPPER_SPINDLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -UPPER_SPINDLE_LENGTH / 2.0)),
        material="bearing_steel",
        name="upper_spindle",
    )

    model.articulation(
        "base_to_lower_platter",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=lower_carrier,
        origin=Origin(xyz=(0.0, 0.0, LOWER_AXIS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-math.pi,
            upper=math.pi,
            effort=90.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "lower_to_upper_turntable",
        ArticulationType.REVOLUTE,
        parent=lower_carrier,
        child=upper_turntable,
        origin=Origin(xyz=(UPPER_AXIS_X, 0.0, UPPER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-math.pi,
            upper=math.pi,
            effort=40.0,
            velocity=2.4,
        ),
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

    pedestal_base = object_model.get_part("pedestal_base")
    lower_carrier = object_model.get_part("lower_carrier")
    upper_turntable = object_model.get_part("upper_turntable")
    lower_joint = object_model.get_articulation("base_to_lower_platter")
    upper_joint = object_model.get_articulation("lower_to_upper_turntable")

    ctx.check(
        "independent offset vertical axes are authored",
        lower_joint.axis == (0.0, 0.0, 1.0)
        and upper_joint.axis == (0.0, 0.0, 1.0)
        and abs(upper_joint.origin.xyz[0] - UPPER_AXIS_X) < 1e-6
        and abs(upper_joint.origin.xyz[2] - UPPER_AXIS_Z) < 1e-6,
        details=(
            f"lower_axis={lower_joint.axis}, "
            f"upper_axis={upper_joint.axis}, "
            f"upper_origin={upper_joint.origin.xyz}"
        ),
    )

    ctx.expect_within(
        pedestal_base,
        lower_carrier,
        axes="xy",
        inner_elem="bearing_post",
        outer_elem="lower_sleeve",
        margin=0.003,
        name="lower bearing post stays inside platter sleeve",
    )
    ctx.expect_overlap(
        pedestal_base,
        lower_carrier,
        axes="z",
        elem_a="bearing_post",
        elem_b="lower_sleeve",
        min_overlap=0.018,
        name="lower sleeve keeps real bearing engagement",
    )
    ctx.expect_within(
        upper_turntable,
        lower_carrier,
        axes="xy",
        inner_elem="upper_spindle",
        outer_elem="upper_housing",
        margin=0.0025,
        name="upper spindle stays centered in the offset housing",
    )
    ctx.expect_overlap(
        upper_turntable,
        lower_carrier,
        axes="z",
        elem_a="upper_spindle",
        elem_b="upper_housing",
        min_overlap=0.055,
        name="upper spindle remains retained in the offset housing",
    )
    ctx.expect_gap(
        upper_turntable,
        pedestal_base,
        axis="z",
        min_gap=0.14,
        name="upper table is clearly elevated above the pedestal",
    )

    rest_upper_axis = ctx.part_world_position(upper_turntable)
    with ctx.pose({lower_joint: math.pi / 2.0}):
        swung_upper_axis = ctx.part_world_position(upper_turntable)
    ctx.check(
        "lower platter rotates the offset column around the pedestal axis",
        rest_upper_axis is not None
        and swung_upper_axis is not None
        and rest_upper_axis[0] > 0.14
        and abs(rest_upper_axis[1]) < 0.01
        and swung_upper_axis[1] > 0.14
        and abs(swung_upper_axis[0]) < 0.02,
        details=f"rest={rest_upper_axis}, swung={swung_upper_axis}",
    )

    rest_upper_aabb = ctx.part_world_aabb(upper_turntable)
    with ctx.pose({upper_joint: math.pi / 2.0}):
        spun_upper_aabb = ctx.part_world_aabb(upper_turntable)
    ctx.check(
        "upper turntable spins independently on its own offset axis",
        rest_upper_aabb is not None
        and spun_upper_aabb is not None
        and rest_upper_aabb[1][0] > spun_upper_aabb[1][0] + 0.02
        and spun_upper_aabb[1][1] > rest_upper_aabb[1][1] + 0.02,
        details=f"rest_aabb={rest_upper_aabb}, spun_aabb={spun_upper_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
