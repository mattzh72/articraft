from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.17
BASE_WIDTH = 0.14
BASE_THICKNESS = 0.018
PEDESTAL_RADIUS = 0.042
PEDESTAL_HEIGHT = 0.072
YAW_BEARING_RADIUS = 0.056
YAW_BEARING_HEIGHT = 0.016

YAW_DISC_RADIUS = 0.05
YAW_DISC_HEIGHT = 0.014
PITCH_AXIS_X = 0.105
PITCH_AXIS_Z = 0.082

ROLL_AXIS_X = 0.044
ROLL_AXIS_Y = 0.11


def box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def cylinder_z(radius: float, length: float, z0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z0).circle(radius).extrude(length)


def cylinder_y(
    radius: float,
    length: float,
    y0: float = 0.0,
    center_x: float = 0.0,
    center_z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .workplane(offset=y0)
        .circle(radius)
        .extrude(length)
        .translate((center_x, 0.0, center_z))
    )


def cylinder_x(
    radius: float,
    length: float,
    x0: float = 0.0,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .workplane(offset=x0)
        .circle(radius)
        .extrude(length)
        .translate((0.0, center_y, center_z))
    )


def ring_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    x0: float,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> cq.Workplane:
    outer = cylinder_x(outer_radius, length, x0=x0, center_y=center_y, center_z=center_z)
    inner = cylinder_x(
        inner_radius,
        length + 0.002,
        x0=x0 - 0.001,
        center_y=center_y,
        center_z=center_z,
    )
    return outer.cut(inner)


def make_base_pedestal() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.008)
    )
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.055, -0.04),
                (-0.055, 0.04),
                (0.055, -0.04),
                (0.055, 0.04),
            ]
        )
        .cboreHole(0.009, 0.016, 0.006)
    )

    pedestal = cylinder_z(PEDESTAL_RADIUS, PEDESTAL_HEIGHT, z0=BASE_THICKNESS)
    pedestal = pedestal.edges(">Z").fillet(0.004)

    lower_bearing = cylinder_z(
        YAW_BEARING_RADIUS,
        YAW_BEARING_HEIGHT,
        z0=BASE_THICKNESS + PEDESTAL_HEIGHT,
    )
    lower_bearing = lower_bearing.cut(
        cylinder_z(
            0.035,
            YAW_BEARING_HEIGHT + 0.002,
            z0=BASE_THICKNESS + PEDESTAL_HEIGHT - 0.001,
        )
    )

    motor_box = (
        box_at(
            (0.05, 0.06, 0.036),
            (-0.034, 0.0, BASE_THICKNESS + 0.018),
        )
        .edges("|Z")
        .fillet(0.004)
    )

    return plate.union(pedestal).union(lower_bearing).union(motor_box)


def make_yaw_carrier() -> cq.Workplane:
    turntable = cylinder_z(YAW_DISC_RADIUS, YAW_DISC_HEIGHT, z0=0.0)
    upper_hub = cylinder_z(0.041, 0.018, z0=YAW_DISC_HEIGHT)

    arm = box_at((0.11, 0.026, 0.028), (0.05, 0.012, 0.034))
    shoulder = box_at((0.05, 0.034, 0.05), (0.024, 0.016, 0.055))

    support_plate = box_at((0.022, 0.06, 0.11), (PITCH_AXIS_X, 0.04, PITCH_AXIS_Z))
    support_plate = support_plate.edges("|Y").fillet(0.003)

    bearing_housing = cylinder_y(
        0.032,
        0.012,
        y0=0.07,
        center_x=PITCH_AXIS_X,
        center_z=PITCH_AXIS_Z,
    )

    rear_spine = box_at((0.034, 0.02, 0.054), (0.084, 0.012, PITCH_AXIS_Z))
    lower_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.018, 0.02),
                (0.018, 0.05),
                (PITCH_AXIS_X - 0.015, PITCH_AXIS_Z - 0.018),
                (PITCH_AXIS_X - 0.012, 0.02),
            ]
        )
        .close()
        .extrude(0.008)
        .translate((0.0, 0.008, 0.0))
    )
    upper_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.026, 0.05),
                (0.026, 0.08),
                (PITCH_AXIS_X - 0.015, PITCH_AXIS_Z + 0.028),
                (PITCH_AXIS_X - 0.014, PITCH_AXIS_Z),
            ]
        )
        .close()
        .extrude(0.008)
        .translate((0.0, 0.016, 0.0))
    )

    body = (
        turntable.union(upper_hub)
        .union(arm)
        .union(shoulder)
        .union(support_plate)
        .union(bearing_housing)
        .union(rear_spine)
        .union(lower_web)
        .union(upper_web)
    )

    pitch_bore = cylinder_y(
        0.0216,
        0.084,
        y0=-0.001,
        center_x=PITCH_AXIS_X,
        center_z=PITCH_AXIS_Z,
    )
    return body.cut(pitch_bore)


def make_pitch_frame() -> cq.Workplane:
    trunnion_shaft = cylinder_y(0.021, 0.08, y0=0.0)
    outboard_collar = cylinder_y(0.028, 0.006, y0=0.08)

    upper_post = box_at((0.008, 0.04, 0.008), (0.06, 0.088, 0.03))
    lower_post = box_at((0.008, 0.04, 0.008), (0.06, 0.088, -0.03))
    upper_strut = box_at((0.062, 0.012, 0.008), (0.048, 0.094, 0.03))
    lower_strut = box_at((0.062, 0.012, 0.008), (0.048, 0.094, -0.03))

    nose_rear_ring = ring_x(0.03, 0.0226, 0.006, x0=ROLL_AXIS_X - 0.015, center_y=ROLL_AXIS_Y)
    nose_front_ring = ring_x(0.03, 0.0226, 0.006, x0=ROLL_AXIS_X + 0.015, center_y=ROLL_AXIS_Y)
    top_rib = box_at((0.042, 0.008, 0.008), (ROLL_AXIS_X, ROLL_AXIS_Y, 0.029))
    bottom_rib = box_at((0.042, 0.008, 0.008), (ROLL_AXIS_X, ROLL_AXIS_Y, -0.029))

    return (
        trunnion_shaft.union(outboard_collar)
        .union(upper_post)
        .union(lower_post)
        .union(upper_strut)
        .union(lower_strut)
        .union(nose_rear_ring)
        .union(nose_front_ring)
        .union(top_rib)
        .union(bottom_rib)
    )


def make_roll_nose() -> cq.Workplane:
    journal = cylinder_x(0.022, 0.04, x0=-0.02)
    rear_collar = cylinder_x(0.026, 0.004, x0=-0.024)
    front_collar = cylinder_x(0.026, 0.004, x0=0.02)
    body = cylinder_x(0.018, 0.066, x0=0.024)
    flange = cylinder_x(0.024, 0.008, x0=0.08)
    tip = (
        cq.Workplane("XZ")
        .moveTo(0.088, 0.0)
        .lineTo(0.088, 0.016)
        .lineTo(0.108, 0.011)
        .lineTo(0.12, 0.011)
        .lineTo(0.12, 0.0)
        .close()
        .revolve(360.0, (0.0, 0.0), (1.0, 0.0))
    )
    return journal.union(rear_collar).union(front_collar).union(body).union(flange).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_robotic_wrist")

    base_dark = model.material("base_dark", color=(0.17, 0.18, 0.2))
    carrier_gray = model.material("carrier_gray", color=(0.45, 0.48, 0.52))
    frame_orange = model.material("frame_orange", color=(0.78, 0.42, 0.14))
    steel = model.material("steel", color=(0.71, 0.73, 0.76))

    base = model.part("base_pedestal")
    base.visual(
        mesh_from_cadquery(make_base_pedestal(), "base_pedestal"),
        material=base_dark,
        name="base_shell",
    )

    yaw_carrier = model.part("yaw_carrier")
    yaw_carrier.visual(
        mesh_from_cadquery(make_yaw_carrier(), "yaw_carrier"),
        material=carrier_gray,
        name="carrier_shell",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(make_pitch_frame(), "pitch_frame"),
        material=frame_orange,
        name="frame_shell",
    )

    roll_nose = model.part("roll_nose")
    roll_nose.visual(
        mesh_from_cadquery(make_roll_nose(), "roll_nose"),
        material=steel,
        name="nose_shell",
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_carrier,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT + YAW_BEARING_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.2, lower=-2.4, upper=2.4),
    )

    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=yaw_carrier,
        child=pitch_frame,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.0, upper=1.15),
    )

    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_nose,
        origin=Origin(xyz=(ROLL_AXIS_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_pedestal")
    yaw_carrier = object_model.get_part("yaw_carrier")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_nose = object_model.get_part("roll_nose")

    yaw_joint = object_model.get_articulation("yaw_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")
    roll_joint = object_model.get_articulation("roll_joint")

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
    ctx.allow_overlap(
        yaw_carrier,
        pitch_frame,
        reason="Pitch trunnion is intentionally modeled nested inside the side support bearing housing.",
    )
    ctx.allow_overlap(
        pitch_frame,
        roll_nose,
        reason="Roll nose journal is intentionally modeled nested inside the pitch-frame bearing rings.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "wrist_parts_present",
        all(part is not None for part in (base, yaw_carrier, pitch_frame, roll_nose)),
        "Expected base pedestal, yaw carrier, pitch frame, and roll nose parts.",
    )
    ctx.check(
        "yaw_axis_vertical",
        tuple(yaw_joint.axis) == (0.0, 0.0, 1.0),
        f"Yaw axis should be vertical, got {yaw_joint.axis}.",
    )
    ctx.check(
        "pitch_axis_lateral",
        tuple(pitch_joint.axis) == (0.0, 1.0, 0.0),
        f"Pitch axis should run laterally, got {pitch_joint.axis}.",
    )
    ctx.check(
        "roll_axis_forward",
        tuple(roll_joint.axis) == (1.0, 0.0, 0.0),
        f"Roll axis should run through the nose, got {roll_joint.axis}.",
    )

    ctx.expect_contact(base, yaw_carrier, contact_tol=0.0015, name="yaw_carrier_grounded_on_pedestal")
    ctx.expect_contact(
        yaw_carrier,
        pitch_frame,
        contact_tol=0.0015,
        name="pitch_frame_grounded_on_side_support",
    )
    ctx.expect_contact(
        pitch_frame,
        roll_nose,
        contact_tol=0.0015,
        name="roll_nose_grounded_in_pitch_frame",
    )
    ctx.expect_overlap(base, yaw_carrier, axes="xy", min_overlap=0.08, name="yaw_turntable_has_support_footprint")
    ctx.expect_origin_gap(
        pitch_frame,
        yaw_carrier,
        axis="x",
        min_gap=0.09,
        max_gap=0.12,
        name="pitch_axis_is_offset_from_yaw_pedestal",
    )
    ctx.expect_origin_gap(
        roll_nose,
        pitch_frame,
        axis="x",
        min_gap=0.035,
        max_gap=0.05,
        name="roll_axis_sits_forward_inside_pitch_frame",
    )
    ctx.expect_overlap(
        roll_nose,
        pitch_frame,
        axes="yz",
        min_overlap=0.05,
        name="roll_nose_projects_inside_pitch_frame_window",
    )

    with ctx.pose({pitch_joint: 0.7, roll_joint: 1.3}):
        ctx.expect_contact(
            yaw_carrier,
            pitch_frame,
            contact_tol=0.0015,
            name="pitch_support_remains_grounded_in_tilted_pose",
        )
        ctx.expect_contact(
            pitch_frame,
            roll_nose,
            contact_tol=0.0015,
            name="roll_support_remains_grounded_in_tilted_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
