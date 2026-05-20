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


PLATE_THICKNESS = 0.014
PLATE_WIDTH = 0.220
PLATE_HEIGHT = 0.280

PLATE_SHAFT_HOLE_RADIUS = 0.014
BEARING_BORE_RADIUS = 0.0135
REAR_RING_OUTER_RADIUS = 0.043
REAR_RING_LENGTH = 0.024
FRONT_RING_OUTER_RADIUS = 0.038
FRONT_RING_LENGTH = 0.022
RING_GAP = 0.050
CARRIER_SIDE_WEB_Y = 0.056

SPINDLE_SHAFT_RADIUS = 0.012
SPINDLE_SHAFT_START = -0.018
SPINDLE_SHAFT_END = 0.126
THRUST_SHOULDER_RADIUS = 0.020
THRUST_SHOULDER_START = PLATE_THICKNESS / 2.0 + REAR_RING_LENGTH + RING_GAP + FRONT_RING_LENGTH
THRUST_SHOULDER_LENGTH = 0.010
OUTPUT_PLATE_RADIUS = 0.052
OUTPUT_PLATE_THICKNESS = 0.010
OUTPUT_PLATE_START = THRUST_SHOULDER_START + THRUST_SHOULDER_LENGTH + 0.003

FRONT_RING_START_LOCAL = REAR_RING_LENGTH + RING_GAP
CARRIER_SPAN_LOCAL = FRONT_RING_START_LOCAL + FRONT_RING_LENGTH
X_AXIS_RPY = (0.0, math.pi / 2.0, 0.0)


def _x_cylinder(radius: float, length: float, start_x: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((start_x, 0.0, 0.0))


def _x_tube(outer_radius: float, inner_radius: float, length: float, start_x: float) -> cq.Workplane:
    outer = _x_cylinder(outer_radius, length, start_x)
    inner = _x_cylinder(inner_radius, length + 0.004, start_x - 0.002)
    return outer.cut(inner)


def _make_side_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT).edges("|X").fillet(0.014)

    shaft_hole = _x_cylinder(
        radius=PLATE_SHAFT_HOLE_RADIUS,
        length=PLATE_THICKNESS + 0.008,
        start_x=-PLATE_THICKNESS / 2.0 - 0.004,
    )
    mounting_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (PLATE_WIDTH * 0.34, PLATE_HEIGHT * 0.36),
                (-PLATE_WIDTH * 0.34, PLATE_HEIGHT * 0.36),
                (PLATE_WIDTH * 0.34, -PLATE_HEIGHT * 0.36),
                (-PLATE_WIDTH * 0.34, -PLATE_HEIGHT * 0.36),
            ]
        )
        .circle(0.009)
        .extrude(PLATE_THICKNESS + 0.008)
        .translate((-PLATE_THICKNESS / 2.0 - 0.004, 0.0, 0.0))
    )
    lower_slot = (
        cq.Workplane("YZ")
        .center(0.0, -0.078)
        .slot2D(0.100, 0.030, 90)
        .extrude(PLATE_THICKNESS + 0.006)
        .translate((-PLATE_THICKNESS / 2.0 - 0.003, 0.0, 0.0))
    )
    return plate.cut(shaft_hole).cut(mounting_holes).cut(lower_slot)


def _make_bearing_carrier_shape() -> cq.Workplane:
    rear_ring = _x_cylinder(
        radius=REAR_RING_OUTER_RADIUS,
        length=REAR_RING_LENGTH,
        start_x=0.0,
    )
    front_ring = _x_cylinder(
        radius=FRONT_RING_OUTER_RADIUS,
        length=FRONT_RING_LENGTH,
        start_x=FRONT_RING_START_LOCAL,
    )
    housing = cq.Workplane("XY").box(CARRIER_SPAN_LOCAL, 0.100, 0.112).translate(
        (CARRIER_SPAN_LOCAL / 2.0, 0.0, 0.0)
    )
    window = cq.Workplane("XY").box(CARRIER_SPAN_LOCAL - 0.018, 0.062, 0.068).translate(
        (CARRIER_SPAN_LOCAL / 2.0 + 0.004, 0.0, 0.0)
    )
    side_relief_left = cq.Workplane("XY").box(CARRIER_SPAN_LOCAL * 0.56, 0.018, 0.080).translate(
        (CARRIER_SPAN_LOCAL * 0.42, CARRIER_SIDE_WEB_Y + 0.006, 0.0)
    )
    side_relief_right = cq.Workplane("XY").box(CARRIER_SPAN_LOCAL * 0.56, 0.018, 0.080).translate(
        (CARRIER_SPAN_LOCAL * 0.42, -CARRIER_SIDE_WEB_Y - 0.006, 0.0)
    )

    carrier = rear_ring.union(front_ring).union(housing)
    carrier = carrier.cut(window).cut(side_relief_left).cut(side_relief_right)
    carrier_bore = _x_cylinder(
        radius=BEARING_BORE_RADIUS,
        length=CARRIER_SPAN_LOCAL + 0.008,
        start_x=-0.004,
    )
    return carrier.cut(carrier_bore)


def _make_output_plate_shape() -> cq.Workplane:
    plate = _x_cylinder(
        radius=OUTPUT_PLATE_RADIUS,
        length=OUTPUT_PLATE_THICKNESS,
        start_x=OUTPUT_PLATE_START,
    )
    bolt_holes = (
        cq.Workplane("YZ")
        .workplane(offset=OUTPUT_PLATE_START - 0.001)
        .pushPoints(
            [
                (0.031, 0.0),
                (-0.031, 0.0),
                (0.0, 0.026),
                (0.0, -0.026),
            ]
        )
        .circle(0.006)
        .extrude(OUTPUT_PLATE_THICKNESS + 0.002)
    )
    return plate.cut(bolt_holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_spindle_assembly")

    model.material("painted_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("machined_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("oxide_plate", rgba=(0.20, 0.22, 0.25, 1.0))

    side_plate = model.part("side_plate")
    bearing_carrier = model.part("bearing_carrier")
    spindle = model.part("spindle")

    side_plate.visual(
        mesh_from_cadquery(_make_side_plate_shape(), "side_plate_panel", tolerance=0.0003, angular_tolerance=0.05),
        material="painted_steel",
        name="side_plate_panel",
    )
    bearing_carrier.visual(
        mesh_from_cadquery(_make_bearing_carrier_shape(), "bearing_carrier", tolerance=0.0003, angular_tolerance=0.05),
        material="machined_steel",
        name="bearing_carrier",
    )

    spindle.visual(
        Cylinder(radius=SPINDLE_SHAFT_RADIUS, length=SPINDLE_SHAFT_END - SPINDLE_SHAFT_START),
        origin=Origin(
            xyz=((SPINDLE_SHAFT_START + SPINDLE_SHAFT_END) / 2.0, 0.0, 0.0),
            rpy=X_AXIS_RPY,
        ),
        material="machined_steel",
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=THRUST_SHOULDER_RADIUS, length=THRUST_SHOULDER_LENGTH),
        origin=Origin(
            xyz=(THRUST_SHOULDER_START + THRUST_SHOULDER_LENGTH / 2.0, 0.0, 0.0),
            rpy=X_AXIS_RPY,
        ),
        material="machined_steel",
        name="thrust_shoulder",
    )
    spindle.visual(
        mesh_from_cadquery(_make_output_plate_shape(), "output_plate"),
        material="oxide_plate",
        name="output_plate",
    )

    model.articulation(
        "side_plate_to_bearing_carrier",
        ArticulationType.FIXED,
        parent=side_plate,
        child=bearing_carrier,
        origin=Origin(xyz=(PLATE_THICKNESS / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "spindle_rotation",
        ArticulationType.CONTINUOUS,
        parent=side_plate,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    bearing_carrier = object_model.get_part("bearing_carrier")
    spindle = object_model.get_part("spindle")
    spindle_rotation = object_model.get_articulation("spindle_rotation")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    limits = spindle_rotation.motion_limits
    ctx.check(
        "spindle_joint_is_continuous_about_x",
        spindle_rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(float(v) for v in spindle_rotation.axis) == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details="The spindle must be one continuous rotary joint aligned to the shaft axis.",
    )
    ctx.expect_contact(
        bearing_carrier,
        side_plate,
        name="bearing_carrier_is_rigidly_seated_on_the_side_plate",
    )
    ctx.expect_contact(
        spindle,
        bearing_carrier,
        name="spindle_thrust_shoulder_bears_on_the_front_support",
    )
    ctx.expect_within(
        spindle,
        bearing_carrier,
        axes="yz",
        inner_elem="spindle_shaft",
        outer_elem="bearing_carrier",
        margin=0.0,
        name="shaft_runs_within_the_bearing_support_axis",
    )
    ctx.expect_gap(
        spindle,
        bearing_carrier,
        axis="x",
        positive_elem="output_plate",
        negative_elem="bearing_carrier",
        min_gap=0.011,
        max_gap=0.016,
        name="output_plate_sits_just_ahead_of_the_front_support_face",
    )

    with ctx.pose({spindle_rotation: 1.20}):
        ctx.expect_contact(
            spindle,
            bearing_carrier,
            name="rotated_spindle_remains_axially_seated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
