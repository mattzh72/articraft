from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    Worm,
    SpurGear,
    mesh_from_cadquery,
)


BASE_X = 0.23
BASE_Y = 0.17
BASE_T = 0.012

WORM_AXIS_Y = -0.045
WORM_AXIS_Z = 0.075
WORM_SUPPORT_X = 0.072
WORM_SHAFT_RADIUS = 0.005
WORM_SHAFT_LENGTH = 0.224
WORM_COLLAR_RADIUS = 0.0105
WORM_COLLAR_LENGTH = 0.004
WORM_KNOB_RADIUS = 0.019
WORM_KNOB_LENGTH = 0.010
WORM_BODY_LENGTH = 0.038
WORM_COLLAR_CENTER_X = 0.087
WORM_KNOB_CENTER_X = 0.104

WHEEL_AXIS_Z = 0.075
WHEEL_SHAFT_RADIUS = 0.005
WHEEL_WHEEL_WIDTH = 0.014
WHEEL_TRANSFER_Z = 0.043

OUTPUT_AXIS_X = 0.031
OUTPUT_AXIS_Z = 0.118
OUTPUT_SHAFT_RADIUS = 0.004
OUTPUT_GEAR_WIDTH = 0.008

WHEEL_LOWER_SUPPORT_TOP = 0.058
UPPER_SUPPORT_UNDERSIDE = 0.134
OUTPUT_LOWER_SUPPORT_TOP = 0.110


def x_axis_cylinder(radius: float, length: float, *, center_x: float, y: float, z: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center_x - length / 2.0, y, z))
    )


def z_axis_cylinder(radius: float, length: float, *, x: float, y: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate((x, y, z0))


def make_frame() -> cq.Workplane:
    frame = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_T, centered=(True, True, False))

    for x_pos in (-WORM_SUPPORT_X, WORM_SUPPORT_X):
        support_block = (
            cq.Workplane("XY")
            .box(0.026, 0.034, 0.054, centered=(True, True, False))
            .translate((x_pos, WORM_AXIS_Y, BASE_T))
        )
        support_boss = x_axis_cylinder(
            0.015,
            0.018,
            center_x=x_pos,
            y=WORM_AXIS_Y,
            z=WORM_AXIS_Z,
        )
        frame = frame.union(support_block).union(support_boss)

    wheel_pedestal = z_axis_cylinder(0.020, 0.046, x=0.0, y=0.0, z0=BASE_T)
    back_column = (
        cq.Workplane("XY")
        .box(0.094, 0.020, 0.136, centered=(True, True, False))
        .translate((0.018, 0.055, BASE_T))
    )
    top_bridge = (
        cq.Workplane("XY")
        .box(0.112, 0.070, 0.012, centered=(True, True, False))
        .translate((0.022, 0.030, 0.146))
    )
    output_shelf = (
        cq.Workplane("XY")
        .box(0.064, 0.060, 0.010, centered=(True, True, False))
        .translate((OUTPUT_AXIS_X, 0.030, 0.100))
    )
    wheel_upper_boss = z_axis_cylinder(0.013, 0.012, x=0.0, y=0.0, z0=UPPER_SUPPORT_UNDERSIDE)
    output_lower_boss = z_axis_cylinder(0.011, 0.010, x=OUTPUT_AXIS_X, y=0.0, z0=0.100)
    output_upper_boss = z_axis_cylinder(0.011, 0.012, x=OUTPUT_AXIS_X, y=0.0, z0=UPPER_SUPPORT_UNDERSIDE)

    frame = (
        frame.union(wheel_pedestal)
        .union(back_column)
        .union(top_bridge)
        .union(output_shelf)
        .union(wheel_upper_boss)
        .union(output_lower_boss)
        .union(output_upper_boss)
    )

    worm_bore = x_axis_cylinder(
        WORM_SHAFT_RADIUS + 0.0007,
        0.028,
        center_x=-WORM_SUPPORT_X,
        y=WORM_AXIS_Y,
        z=WORM_AXIS_Z,
    ).union(
        x_axis_cylinder(
            WORM_SHAFT_RADIUS + 0.0007,
            0.028,
            center_x=WORM_SUPPORT_X,
            y=WORM_AXIS_Y,
            z=WORM_AXIS_Z,
        )
    )
    spindle_bore = z_axis_cylinder(
        WHEEL_SHAFT_RADIUS + 0.0006,
        0.146,
        x=0.0,
        y=0.0,
        z0=BASE_T,
    )
    output_bore = z_axis_cylinder(
        OUTPUT_SHAFT_RADIUS + 0.0006,
        0.058,
        x=OUTPUT_AXIS_X,
        y=0.0,
        z0=0.100,
    )

    window_cut = (
        cq.Workplane("XY")
        .box(0.050, 0.022, 0.060, centered=(True, True, False))
        .translate((0.002, 0.055, 0.060))
    )
    spindle_relief = z_axis_cylinder(0.018, 0.018, x=0.0, y=0.0, z0=0.096)

    return (
        frame.cut(worm_bore)
        .cut(spindle_bore)
        .cut(output_bore)
        .cut(window_cut)
        .cut(spindle_relief)
    )


def make_worm_body() -> object:
    return (
        Worm(
            module=0.0022,
            lead_angle=20.0,
            n_threads=1,
            length=WORM_BODY_LENGTH,
            bore_d=2.0 * WORM_SHAFT_RADIUS,
        ).build()
    )


def make_spoked_wheel() -> cq.Workplane:
    wheel_spec = SpurGear(
        module=0.0022,
        teeth_number=28,
        width=WHEEL_WHEEL_WIDTH,
        bore_d=2.0 * WHEEL_SHAFT_RADIUS,
    )
    wheel = cq.Workplane("XY").gear(wheel_spec).translate((0.0, 0.0, -WHEEL_WHEEL_WIDTH / 2.0))
    spoke_points = [
        (0.016 * cos(angle), 0.016 * sin(angle))
        for angle in (0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)
    ]
    spoke_cutters = (
        cq.Workplane("XY")
        .pushPoints(spoke_points)
        .circle(0.006)
        .extrude(WHEEL_WHEEL_WIDTH * 2.5)
        .translate((0.0, 0.0, -WHEEL_WHEEL_WIDTH * 1.25))
    )
    return wheel.cut(spoke_cutters)


def make_transfer_gear() -> cq.Workplane:
    spec = SpurGear(
        module=0.0020,
        teeth_number=16,
        width=OUTPUT_GEAR_WIDTH,
        bore_d=2.0 * WHEEL_SHAFT_RADIUS,
    )
    return cq.Workplane("XY").gear(spec).translate((0.0, 0.0, -OUTPUT_GEAR_WIDTH / 2.0))


def make_output_gear() -> cq.Workplane:
    spec = SpurGear(
        module=0.0020,
        teeth_number=10,
        width=OUTPUT_GEAR_WIDTH,
        bore_d=2.0 * OUTPUT_SHAFT_RADIUS,
    )
    return cq.Workplane("XY").gear(spec).translate((0.0, 0.0, -OUTPUT_GEAR_WIDTH / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="worm_driven_indexing_unit")

    model.material("frame_paint", color=(0.25, 0.39, 0.33))
    model.material("steel", color=(0.64, 0.66, 0.70))
    model.material("dark_steel", color=(0.32, 0.34, 0.38))
    model.material("bronze", color=(0.72, 0.53, 0.22))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_frame(), "indexing_frame"),
        material="frame_paint",
        name="frame_body",
    )

    worm_shaft = model.part("worm_shaft")
    worm_shaft.visual(
        Cylinder(radius=WORM_SHAFT_RADIUS, length=WORM_SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="steel",
        name="shaft_core",
    )
    worm_shaft.visual(
        Cylinder(radius=WORM_COLLAR_RADIUS, length=WORM_COLLAR_LENGTH),
        origin=Origin(xyz=(-WORM_COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="left_collar",
    )
    worm_shaft.visual(
        Cylinder(radius=WORM_COLLAR_RADIUS, length=WORM_COLLAR_LENGTH),
        origin=Origin(xyz=(WORM_COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="right_collar",
    )
    worm_shaft.visual(
        Cylinder(radius=WORM_KNOB_RADIUS, length=WORM_KNOB_LENGTH),
        origin=Origin(xyz=(WORM_KNOB_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="drive_knob",
    )
    worm_shaft.visual(
        mesh_from_cadquery(make_worm_body(), "worm_body"),
        material="steel",
        name="worm",
    )

    wheel_spindle = model.part("wheel_spindle")
    wheel_spindle.visual(
        Cylinder(radius=WHEEL_SHAFT_RADIUS, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material="steel",
        name="spindle",
    )
    wheel_spindle.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material="dark_steel",
        name="lower_collar",
    )
    wheel_spindle.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material="dark_steel",
        name="upper_collar",
    )
    wheel_spindle.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="bronze",
        name="wheel_hub",
    )
    wheel_spindle.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material="steel",
        name="gear_spacer",
    )
    wheel_spindle.visual(
        mesh_from_cadquery(make_spoked_wheel(), "worm_wheel"),
        material="bronze",
        name="wheel",
    )
    wheel_spindle.visual(
        mesh_from_cadquery(make_transfer_gear(), "transfer_gear"),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_TRANSFER_Z)),
        material="steel",
        name="upper_gear",
    )

    output_shaft = model.part("output_shaft")
    output_shaft.visual(
        Cylinder(radius=OUTPUT_SHAFT_RADIUS, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="steel",
        name="shaft",
    )
    output_shaft.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material="dark_steel",
        name="lower_collar",
    )
    output_shaft.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="dark_steel",
        name="upper_collar",
    )
    output_shaft.visual(
        mesh_from_cadquery(make_output_gear(), "secondary_output_gear"),
        material="steel",
        name="output_gear",
    )

    model.articulation(
        "frame_to_worm_shaft",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=worm_shaft,
        origin=Origin(xyz=(0.0, WORM_AXIS_Y, WORM_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=8.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )
    model.articulation(
        "frame_to_wheel_spindle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=wheel_spindle,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "frame_to_output_shaft",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=output_shaft,
        origin=Origin(xyz=(OUTPUT_AXIS_X, 0.0, OUTPUT_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=5.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    parts_by_name = {part.name: part for part in object_model.parts}
    joints_by_name = {joint.name: joint for joint in object_model.articulations}

    required_parts = {"frame", "worm_shaft", "wheel_spindle", "output_shaft"}
    required_joints = {
        "frame_to_worm_shaft",
        "frame_to_wheel_spindle",
        "frame_to_output_shaft",
    }
    ctx.check(
        "required parts present",
        required_parts.issubset(parts_by_name),
        details=f"present parts: {sorted(parts_by_name)}",
    )
    ctx.check(
        "required joints present",
        required_joints.issubset(joints_by_name),
        details=f"present joints: {sorted(joints_by_name)}",
    )

    if not (required_parts.issubset(parts_by_name) and required_joints.issubset(joints_by_name)):
        return ctx.report()

    frame = parts_by_name["frame"]
    worm_shaft = parts_by_name["worm_shaft"]
    wheel_spindle = parts_by_name["wheel_spindle"]
    output_shaft = parts_by_name["output_shaft"]

    worm_joint = joints_by_name["frame_to_worm_shaft"]
    wheel_joint = joints_by_name["frame_to_wheel_spindle"]
    output_joint = joints_by_name["frame_to_output_shaft"]

    ctx.check(
        "worm axis is horizontal",
        tuple(worm_joint.axis) == (1.0, 0.0, 0.0),
        details=f"worm axis was {worm_joint.axis}",
    )
    ctx.check(
        "wheel and output axes are vertical",
        tuple(wheel_joint.axis) == (0.0, 0.0, 1.0) and tuple(output_joint.axis) == (0.0, 0.0, 1.0),
        details=f"wheel axis={wheel_joint.axis}, output axis={output_joint.axis}",
    )

    ctx.expect_contact(frame, worm_shaft, name="worm shaft is captured by the frame")
    ctx.expect_contact(frame, wheel_spindle, name="wheel spindle is supported by the frame")
    ctx.expect_contact(frame, output_shaft, name="output shaft is supported by the frame")

    ctx.expect_origin_gap(
        wheel_spindle,
        worm_shaft,
        axis="z",
        min_gap=-0.002,
        max_gap=0.002,
        name="worm axis meets the wheel midplane",
    )
    ctx.expect_origin_gap(
        output_shaft,
        wheel_spindle,
        axis="z",
        min_gap=0.035,
        max_gap=0.050,
        name="secondary spur stage sits above the wheel hub",
    )
    ctx.expect_origin_distance(
        output_shaft,
        wheel_spindle,
        axes="xy",
        min_dist=0.026,
        max_dist=0.036,
        name="upper spur gears occupy a compact parallel-axis mesh spacing",
    )

    with ctx.pose({worm_joint: 1.1, wheel_joint: 0.6, output_joint: -0.9}):
        ctx.expect_contact(frame, worm_shaft, name="worm support remains captured in a posed state")
        ctx.expect_contact(frame, wheel_spindle, name="wheel spindle remains seated in a posed state")
        ctx.expect_contact(frame, output_shaft, name="output shaft remains seated in a posed state")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
