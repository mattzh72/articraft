from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import hypot, isclose

import cadquery as cq

from sdk import (
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


BASE_LENGTH = 0.34
BASE_WIDTH = 0.22
BASE_THICKNESS = 0.018

LOWER_HOUSING_OUTER_RADIUS = 0.078
LOWER_HOUSING_INNER_RADIUS = 0.046
LOWER_HOUSING_HEIGHT = 0.028
LOWER_JOINT_Z = BASE_THICKNESS + LOWER_HOUSING_HEIGHT

LOWER_STAGE_GAP = 0.0
LOWER_PLATTER_RADIUS = 0.069
LOWER_PLATTER_THICKNESS = 0.014
LOWER_SPINDLE_RADIUS = 0.034
LOWER_SPINDLE_LENGTH = 0.030
LOWER_SPINDLE_CENTER_Z = -0.011

UPPER_AXIS_X = 0.170
UPPER_JOINT_Z = 0.086
UPPER_HOUSING_OUTER_RADIUS = 0.042
UPPER_HOUSING_INNER_RADIUS = 0.023
UPPER_SPINDLE_RADIUS = 0.016
UPPER_SPINDLE_LENGTH = 0.036
UPPER_SPINDLE_CENTER_Z = -0.014


def _ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _base_plate_shape() -> cq.Workplane:
    hole_points = [
        (-0.120, -0.072),
        (-0.120, 0.072),
        (0.120, -0.072),
        (0.120, 0.072),
    ]
    return (
        cq.Workplane("XY")
        .box(
            BASE_LENGTH,
            BASE_WIDTH,
            BASE_THICKNESS,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.016)
        .faces(">Z")
        .workplane()
        .pushPoints(hole_points)
        .circle(0.006)
        .cutThruAll()
    )


def _base_center_housing_shape() -> cq.Workplane:
    lower_flange = _ring(0.094, 0.060, 0.006).translate((0.0, 0.0, BASE_THICKNESS))
    main_ring = _ring(
        LOWER_HOUSING_OUTER_RADIUS,
        LOWER_HOUSING_INNER_RADIUS,
        LOWER_HOUSING_HEIGHT - 0.006,
    ).translate((0.0, 0.0, BASE_THICKNESS + 0.006))
    return lower_flange.union(main_ring)


def _lower_carrier_body_shape() -> cq.Workplane:
    platter = (
        cq.Workplane("XY")
        .circle(LOWER_PLATTER_RADIUS)
        .extrude(LOWER_PLATTER_THICKNESS)
        .translate((0.0, 0.0, LOWER_STAGE_GAP))
    )

    center_pedestal = (
        cq.Workplane("XY")
        .box(0.060, 0.060, 0.034, centered=(True, True, False))
        .translate((0.022, 0.0, LOWER_STAGE_GAP + LOWER_PLATTER_THICKNESS))
        .edges("|Z")
        .fillet(0.008)
    )

    beam = (
        cq.Workplane("XY")
        .box(0.152, 0.042, 0.018, centered=(True, True, False))
        .translate((0.096, 0.0, 0.048))
        .edges("|Z")
        .fillet(0.006)
    )

    arm_platform = (
        cq.Workplane("XY")
        .box(0.078, 0.070, 0.018, centered=(True, True, False))
        .translate((UPPER_AXIS_X, 0.0, 0.040))
        .edges("|Z")
        .fillet(0.008)
    )

    rib_profile = [
        (0.020, LOWER_STAGE_GAP + LOWER_PLATTER_THICKNESS),
        (0.050, LOWER_STAGE_GAP + LOWER_PLATTER_THICKNESS),
        (0.147, 0.048),
        (0.147, 0.062),
        (0.070, 0.062),
        (0.020, 0.040),
    ]
    rib_pos = (
        cq.Workplane("XZ")
        .polyline(rib_profile)
        .close()
        .extrude(0.010)
        .translate((0.0, 0.011, 0.0))
    )
    rib_neg = (
        cq.Workplane("XZ")
        .polyline(rib_profile)
        .close()
        .extrude(0.010)
        .translate((0.0, -0.021, 0.0))
    )

    body = platter.union(center_pedestal).union(beam).union(arm_platform).union(rib_pos).union(rib_neg)
    upper_bore = (
        cq.Workplane("XY")
        .circle(0.030)
        .extrude(0.050)
        .translate((UPPER_AXIS_X, 0.0, 0.030))
    )
    return body.cut(upper_bore)


def _upper_support_housing_shape() -> cq.Workplane:
    base_flange = _ring(0.048, 0.030, 0.008).translate((UPPER_AXIS_X, 0.0, 0.050))
    ring = _ring(UPPER_HOUSING_OUTER_RADIUS, UPPER_HOUSING_INNER_RADIUS, 0.028).translate(
        (UPPER_AXIS_X, 0.0, 0.058)
    )
    cheek_pos = (
        cq.Workplane("XY")
        .box(0.038, 0.010, 0.032, centered=(True, True, False))
        .translate((UPPER_AXIS_X, 0.026, 0.054))
    )
    cheek_neg = (
        cq.Workplane("XY")
        .box(0.038, 0.010, 0.032, centered=(True, True, False))
        .translate((UPPER_AXIS_X, -0.026, 0.054))
    )
    top_lip = _ring(0.046, 0.030, 0.004).translate((UPPER_AXIS_X, 0.0, 0.082))
    return base_flange.union(ring).union(cheek_pos).union(cheek_neg).union(top_lip)


def _upper_flange_shape() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .circle(0.048)
        .extrude(0.010)
        .translate((0.0, 0.0, 0.0))
    )
    flange = (
        flange.faces(">Z")
        .workplane()
        .polarArray(0.034, 0.0, 360.0, 6)
        .circle(0.0045)
        .cutBlind(-0.010)
    )
    boss = cq.Workplane("XY").circle(0.022).extrude(0.014).translate((0.0, 0.0, 0.010))
    return flange.union(boss)


def _upper_drive_ear_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(0.020, 0.018, 0.010, centered=(False, True, False))
        .translate((0.040, 0.0, 0.009))
    )
    nose = (
        cq.Workplane("XY")
        .circle(0.009)
        .extrude(0.010)
        .translate((0.060, 0.0, 0.009))
    )
    ear = body.union(nose)
    return (
        ear.faces(">Z")
        .workplane()
        .center(0.060, 0.0)
        .circle(0.0045)
        .cutBlind(-0.010)
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return (
        0.5 * (low[0] + high[0]),
        0.5 * (low[1] + high[1]),
        0.5 * (low[2] + high[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_offset_rotary_stage")

    model.material("base_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("bridge_blue", rgba=(0.24, 0.34, 0.57, 1.0))
    model.material("machined_steel", rgba=(0.73, 0.75, 0.78, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate"),
        material="base_charcoal",
        name="base_plate",
    )
    base_frame.visual(
        mesh_from_cadquery(_base_center_housing_shape(), "base_center_housing"),
        material="base_charcoal",
        name="base_center_housing",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, LOWER_JOINT_Z)),
        mass=9.8,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * LOWER_JOINT_Z)),
    )

    lower_bridge_stage = model.part("lower_bridge_stage")
    lower_bridge_stage.visual(
        mesh_from_cadquery(_lower_carrier_body_shape(), "lower_carrier_body"),
        material="bridge_blue",
        name="lower_carrier_body",
    )
    lower_bridge_stage.visual(
        mesh_from_cadquery(_upper_support_housing_shape(), "upper_support_housing"),
        material="machined_steel",
        name="upper_support_housing",
    )
    lower_bridge_stage.visual(
        Cylinder(radius=LOWER_SPINDLE_RADIUS, length=LOWER_SPINDLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, LOWER_SPINDLE_CENTER_Z)),
        material="machined_steel",
        name="lower_spindle",
    )
    lower_bridge_stage.inertial = Inertial.from_geometry(
        Box((0.255, 0.090, 0.098)),
        mass=4.1,
        origin=Origin(xyz=(0.095, 0.0, 0.049)),
    )

    upper_rotary_flange = model.part("upper_rotary_flange")
    upper_rotary_flange.visual(
        mesh_from_cadquery(_upper_flange_shape(), "upper_flange"),
        material="machined_steel",
        name="upper_flange",
    )
    upper_rotary_flange.visual(
        mesh_from_cadquery(_upper_drive_ear_shape(), "upper_drive_ear"),
        material="bridge_blue",
        name="upper_drive_ear",
    )
    upper_rotary_flange.visual(
        Cylinder(radius=UPPER_SPINDLE_RADIUS, length=UPPER_SPINDLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, UPPER_SPINDLE_CENTER_Z)),
        material="machined_steel",
        name="upper_spindle",
    )
    upper_rotary_flange.inertial = Inertial.from_geometry(
        Cylinder(radius=0.048, length=0.030),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=lower_bridge_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=48.0,
            velocity=1.8,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "lower_to_upper_stage",
        ArticulationType.REVOLUTE,
        parent=lower_bridge_stage,
        child=upper_rotary_flange,
        origin=Origin(xyz=(UPPER_AXIS_X, 0.0, UPPER_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=3.0,
            lower=-3.0,
            upper=3.0,
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

    base_frame = object_model.get_part("base_frame")
    lower_bridge_stage = object_model.get_part("lower_bridge_stage")
    upper_rotary_flange = object_model.get_part("upper_rotary_flange")
    base_to_lower = object_model.get_articulation("base_to_lower_stage")
    lower_to_upper = object_model.get_articulation("lower_to_upper_stage")

    ctx.check(
        "rotary axes stay parallel and vertical",
        tuple(base_to_lower.axis) == (0.0, 0.0, 1.0)
        and tuple(lower_to_upper.axis) == (0.0, 0.0, 1.0),
        details=f"lower_axis={base_to_lower.axis}, upper_axis={lower_to_upper.axis}",
    )

    with ctx.pose({base_to_lower: 0.0, lower_to_upper: 0.0}):
        ctx.expect_within(
            lower_bridge_stage,
            base_frame,
            axes="xy",
            inner_elem="lower_spindle",
            outer_elem="base_center_housing",
            name="lower spindle stays centered inside the grounded housing",
        )
        ctx.expect_overlap(
            lower_bridge_stage,
            base_frame,
            axes="z",
            elem_a="lower_spindle",
            elem_b="base_center_housing",
            min_overlap=0.024,
            name="lower spindle remains retained in the grounded housing",
        )
        ctx.expect_contact(
            lower_bridge_stage,
            base_frame,
            elem_a="lower_carrier_body",
            elem_b="base_center_housing",
            contact_tol=0.0005,
            name="lower stage carrier is seated on the grounded housing",
        )

        ctx.expect_within(
            upper_rotary_flange,
            lower_bridge_stage,
            axes="xy",
            inner_elem="upper_spindle",
            outer_elem="upper_support_housing",
            name="upper spindle stays centered inside the bridge housing",
        )
        ctx.expect_overlap(
            upper_rotary_flange,
            lower_bridge_stage,
            axes="z",
            elem_a="upper_spindle",
            elem_b="upper_support_housing",
            min_overlap=0.024,
            name="upper spindle remains retained in the offset bridge housing",
        )
        ctx.expect_contact(
            upper_rotary_flange,
            lower_bridge_stage,
            elem_a="upper_flange",
            elem_b="upper_support_housing",
            contact_tol=0.0005,
            name="upper flange is seated on the bridge housing",
        )

    rest_upper_axis = ctx.part_world_position(upper_rotary_flange)
    with ctx.pose({base_to_lower: 1.0, lower_to_upper: 0.0}):
        swung_upper_axis = ctx.part_world_position(upper_rotary_flange)
    ctx.check(
        "lower stage swings the bridge and offset upper axis around the base",
        rest_upper_axis is not None
        and swung_upper_axis is not None
        and swung_upper_axis[1] > rest_upper_axis[1] + 0.12
        and swung_upper_axis[0] < rest_upper_axis[0] - 0.06,
        details=f"rest={rest_upper_axis}, swung={swung_upper_axis}",
    )

    with ctx.pose({base_to_lower: 0.55, lower_to_upper: 0.0}):
        ear_rest = _aabb_center(ctx.part_element_world_aabb(upper_rotary_flange, elem="upper_drive_ear"))
        axis_rest = ctx.part_world_position(upper_rotary_flange)
    with ctx.pose({base_to_lower: 0.55, lower_to_upper: 1.10}):
        ear_turned = _aabb_center(ctx.part_element_world_aabb(upper_rotary_flange, elem="upper_drive_ear"))
        axis_turned = ctx.part_world_position(upper_rotary_flange)

    same_axis = (
        axis_rest is not None
        and axis_turned is not None
        and isclose(axis_rest[0], axis_turned[0], abs_tol=1e-6)
        and isclose(axis_rest[1], axis_turned[1], abs_tol=1e-6)
        and isclose(axis_rest[2], axis_turned[2], abs_tol=1e-6)
    )
    moved_ear = (
        ear_rest is not None
        and ear_turned is not None
        and hypot(ear_turned[0] - ear_rest[0], ear_turned[1] - ear_rest[1]) > 0.030
    )
    preserved_radius = (
        ear_rest is not None
        and ear_turned is not None
        and axis_rest is not None
        and axis_turned is not None
        and abs(
            hypot(ear_rest[0] - axis_rest[0], ear_rest[1] - axis_rest[1])
            - hypot(ear_turned[0] - axis_turned[0], ear_turned[1] - axis_turned[1])
        )
        < 0.006
    )
    ctx.check(
        "upper rotary flange spins in place on the offset axis",
        same_axis and moved_ear and preserved_radius,
        details=(
            f"axis_rest={axis_rest}, axis_turned={axis_turned}, "
            f"ear_rest={ear_rest}, ear_turned={ear_turned}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
