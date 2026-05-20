from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


PLATE_THICKNESS = 0.018
PLATE_WIDTH = 0.118
PLATE_HEIGHT = 0.170
SHOULDER_X = 0.030
SHOULDER_Z = 0.024

PRIMARY_LENGTH = 0.175
SECONDARY_LENGTH = 0.270
TILT_AXIS_X = 0.056

VERTICAL_PIN_RADIUS = 0.014
VERTICAL_PIN_HEIGHT = 0.032
VERTICAL_THRUST_RADIUS = 0.026
VERTICAL_THRUST_THICKNESS = 0.002
VERTICAL_CAP_RADIUS = 0.019
VERTICAL_CAP_THICKNESS = 0.004
VERTICAL_COLLAR_INNER_RADIUS = 0.016
VERTICAL_COLLAR_OUTER_RADIUS = 0.029
VERTICAL_COLLAR_HEIGHT = 0.026

PAN_PIN_RADIUS = 0.012
PAN_THRUST_RADIUS = 0.022
PAN_COLLAR_INNER_RADIUS = 0.014
PAN_COLLAR_OUTER_RADIUS = 0.025
PAN_COLLAR_HEIGHT = 0.022

TILT_BARREL_RADIUS = 0.010
TILT_BARREL_LENGTH = 0.030
VESA_PLATE_SIZE = 0.118
VESA_PATTERN = 0.100
VESA_HOLE_RADIUS = 0.003


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def _rounded_box(x: float, y: float, z: float, fillet: float) -> cq.Workplane:
    return cq.Workplane("XY").box(x, y, z).edges("|X").fillet(fillet)


def _vertical_ring(
    *,
    inner_radius: float,
    outer_radius: float,
    height: float,
    z_center: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z_center - (height / 2.0)))
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _vertical_pin(radius: float, height: float, x_center: float) -> Cylinder:
    return Cylinder(radius=radius, length=height)


def _wall_plate_body_shape() -> cq.Workplane:
    plate = _rounded_box(PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT, 0.010)

    center_stem = (
        cq.Workplane("XY")
        .box(0.014, 0.020, 0.066)
        .translate((0.002, 0.0, SHOULDER_Z))
    )
    side_ribs = (
        cq.Workplane("XY")
        .box(0.012, 0.018, 0.076)
        .translate((0.0, 0.026, SHOULDER_Z))
        .union(
            cq.Workplane("XY")
            .box(0.012, 0.018, 0.076)
            .translate((0.0, -0.026, SHOULDER_Z))
        )
    )
    top_bridge = (
        cq.Workplane("XY")
        .box(0.010, 0.060, 0.014)
        .translate((0.0, 0.0, SHOULDER_Z + 0.032))
    )
    bottom_bridge = (
        cq.Workplane("XY")
        .box(0.010, 0.060, 0.014)
        .translate((0.0, 0.0, SHOULDER_Z - 0.032))
    )

    body = plate.union(center_stem).union(side_ribs).union(top_bridge).union(bottom_bridge)

    mount_recesses = (
        cq.Workplane("YZ", origin=(0.0, 0.0, 0.0))
        .pushPoints(
            [
                (-0.030, 0.054),
                (0.030, 0.054),
                (-0.030, -0.054),
                (0.030, -0.054),
            ]
        )
        .circle(0.005)
        .extrude(PLATE_THICKNESS + 0.010, both=True)
    )
    return body.cut(mount_recesses)


def _pivot_eye_shape(
    *,
    hole_radius: float,
    outer_radius: float,
    height: float,
    tail_length: float,
    tail_width: float,
    eye_offset_x: float,
) -> cq.Workplane:
    eye = (
        cq.Workplane("XY", origin=(0.0, 0.0, -(height / 2.0)))
        .center(eye_offset_x, 0.0)
        .circle(outer_radius)
        .extrude(height)
    )
    tail = (
        cq.Workplane("XY")
        .box(tail_length, tail_width, height)
        .translate((eye_offset_x + (tail_length / 2.0), 0.0, 0.0))
    )
    upper_belt = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.005))
        .center(eye_offset_x + 0.004, 0.0)
        .circle(outer_radius + 0.0025)
        .extrude(0.010)
    )
    bore = (
        cq.Workplane("XY", origin=(0.0, 0.0, -((height + 0.004) / 2.0)))
        .circle(hole_radius)
        .extrude(height + 0.004)
    )
    rear_opening = (
        cq.Workplane("XY")
        .box(0.024, tail_width + 0.016, height + 0.004)
        .translate((-0.014, 0.0, 0.0))
    )
    return eye.union(tail).union(upper_belt).cut(bore).cut(rear_opening)


def _primary_collar_shape() -> cq.Workplane:
    return _pivot_eye_shape(
        hole_radius=VERTICAL_PIN_RADIUS + 0.0006,
        outer_radius=0.023,
        height=VERTICAL_COLLAR_HEIGHT,
        tail_length=0.028,
        tail_width=0.042,
        eye_offset_x=0.016,
    )


def _primary_arm_body_shape() -> cq.Workplane:
    beam = (
        cq.Workplane("XY")
        .box(0.112, 0.046, 0.018)
        .translate((0.095, 0.0, 0.012))
    )
    shoulder_web = (
        cq.Workplane("XY")
        .box(0.054, 0.050, 0.026)
        .translate((0.050, 0.0, 0.010))
    )
    elbow_neck = (
        cq.Workplane("XY")
        .box(0.024, 0.024, 0.028)
        .translate((0.158, 0.0, 0.002))
    )
    lower_bracket = (
        cq.Workplane("XY")
        .box(0.106, 0.034, 0.008)
        .translate((0.094, 0.0, -0.002))
    )
    return shoulder_web.union(beam).union(lower_bracket).union(elbow_neck)


def _primary_cover_shape() -> cq.Workplane:
    cover = (
        cq.Workplane("XY")
        .box(0.090, 0.024, 0.008)
        .translate((0.102, 0.0, 0.026))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.020, 0.022, 0.010)
        .translate((0.060, 0.0, 0.024))
    )
    tail = (
        cq.Workplane("XY")
        .box(0.020, 0.020, 0.009)
        .translate((0.145, 0.0, 0.025))
    )
    return cover.union(nose).union(tail)


def _secondary_collar_shape() -> cq.Workplane:
    return _pivot_eye_shape(
        hole_radius=VERTICAL_PIN_RADIUS + 0.0006,
        outer_radius=0.024,
        height=VERTICAL_COLLAR_HEIGHT,
        tail_length=0.032,
        tail_width=0.042,
        eye_offset_x=0.016,
    )


def _secondary_arm_body_shape() -> cq.Workplane:
    beam = (
        cq.Workplane("XY")
        .box(0.176, 0.044, 0.018)
        .translate((0.144, 0.0, -0.012))
    )
    elbow_web = (
        cq.Workplane("XY")
        .box(0.060, 0.048, 0.028)
        .translate((0.050, 0.0, -0.008))
    )
    pan_neck = (
        cq.Workplane("XY")
        .box(0.018, 0.024, 0.026)
        .translate((0.247, 0.0, -0.004))
    )
    lower_strap = (
        cq.Workplane("XY")
        .box(0.170, 0.032, 0.008)
        .translate((0.144, 0.0, -0.024))
    )
    return elbow_web.union(beam).union(lower_strap).union(pan_neck)


def _secondary_cover_shape() -> cq.Workplane:
    spine = (
        cq.Workplane("XY")
        .box(0.170, 0.022, 0.010)
        .translate((0.150, 0.0, 0.004))
    )
    front_fairing = (
        cq.Workplane("XY")
        .box(0.030, 0.022, 0.012)
        .translate((0.236, 0.0, 0.006))
    )
    rear_fairing = (
        cq.Workplane("XY")
        .box(0.026, 0.022, 0.012)
        .translate((0.072, 0.0, 0.005))
    )
    return spine.union(front_fairing).union(rear_fairing)


def _pan_collar_shape() -> cq.Workplane:
    return _pivot_eye_shape(
        hole_radius=PAN_PIN_RADIUS + 0.0006,
        outer_radius=0.020,
        height=PAN_COLLAR_HEIGHT,
        tail_length=0.022,
        tail_width=0.028,
        eye_offset_x=0.014,
    )


def _pan_head_body_shape() -> cq.Workplane:
    rear_core = (
        cq.Workplane("XY")
        .box(0.024, 0.024, 0.024)
        .translate((0.016, 0.0, -0.002))
    )
    left_strut = (
        cq.Workplane("XY")
        .box(0.030, 0.008, 0.014)
        .translate((0.032, 0.018, -0.003))
    )
    right_strut = (
        cq.Workplane("XY")
        .box(0.030, 0.008, 0.014)
        .translate((0.032, -0.018, -0.003))
    )
    left_ear = (
        cq.Workplane("XY")
        .box(0.012, 0.010, 0.036)
        .translate((TILT_AXIS_X - 0.004, 0.024, 0.0))
        .union(
            cq.Workplane("XZ", origin=(0.0, 0.019, 0.0))
            .center(TILT_AXIS_X, 0.0)
            .circle(0.013)
            .extrude(0.010)
        )
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.012, 0.010, 0.036)
        .translate((TILT_AXIS_X - 0.004, -0.024, 0.0))
        .union(
            cq.Workplane("XZ", origin=(0.0, -0.029, 0.0))
            .center(TILT_AXIS_X, 0.0)
            .circle(0.013)
            .extrude(0.010)
        )
    )
    left_bore = (
        cq.Workplane("XZ", origin=(0.0, 0.018, 0.0))
        .center(TILT_AXIS_X, 0.0)
        .circle(TILT_BARREL_RADIUS + 0.0012)
        .extrude(0.012)
    )
    right_bore = (
        cq.Workplane("XZ", origin=(0.0, -0.030, 0.0))
        .center(TILT_AXIS_X, 0.0)
        .circle(TILT_BARREL_RADIUS + 0.0012)
        .extrude(0.012)
    )
    return rear_core.union(left_strut).union(right_strut).union(left_ear).union(right_ear).cut(left_bore).cut(right_bore)


def _tilt_cradle_pivot_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("XZ", origin=(0.0, -0.022, 0.0))
        .circle(TILT_BARREL_RADIUS)
        .extrude(0.044)
    )
    hub = (
        cq.Workplane("XY")
        .box(0.018, 0.016, 0.018)
        .translate((0.009, 0.0, 0.0))
    )
    return barrel.union(hub)


def _tilt_cradle_frame_shape() -> cq.Workplane:
    spine = (
        cq.Workplane("XY")
        .box(0.036, 0.018, 0.048)
        .translate((0.036, 0.0, 0.0))
    )
    upper_web = (
        cq.Workplane("XY")
        .box(0.030, 0.014, 0.014)
        .translate((0.058, 0.0, 0.030))
    )
    lower_web = (
        cq.Workplane("XY")
        .box(0.030, 0.014, 0.014)
        .translate((0.058, 0.0, -0.030))
    )
    plate = _rounded_box(0.004, VESA_PLATE_SIZE, VESA_PLATE_SIZE, 0.008).translate((0.088, 0.0, 0.0))
    vesa_holes = (
        cq.Workplane("YZ", origin=(0.088, 0.0, 0.0))
        .pushPoints(
            [
                (-VESA_PATTERN / 2.0, -VESA_PATTERN / 2.0),
                (-VESA_PATTERN / 2.0, VESA_PATTERN / 2.0),
                (VESA_PATTERN / 2.0, -VESA_PATTERN / 2.0),
                (VESA_PATTERN / 2.0, VESA_PATTERN / 2.0),
            ]
        )
        .circle(VESA_HOLE_RADIUS)
        .extrude(0.020, both=True)
    )
    center_relief = (
        cq.Workplane("YZ", origin=(0.088, 0.0, 0.0))
        .rect(0.038, 0.038)
        .extrude(0.008, both=True)
    )
    return spine.union(upper_web).union(lower_web).union(plate).cut(vesa_holes).cut(center_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_monitor_arm")

    model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("aluminum", rgba=(0.67, 0.70, 0.74, 1.0))
    model.material("trim_black", rgba=(0.11, 0.12, 0.14, 1.0))
    model.material("steel", rgba=(0.58, 0.60, 0.64, 1.0))
    model.material("black_oxide", rgba=(0.28, 0.30, 0.33, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        _mesh(_wall_plate_body_shape(), "wall_plate_body"),
        material="graphite",
        name="wall_plate_body",
    )
    wall_plate.visual(
        Cylinder(radius=VERTICAL_PIN_RADIUS, length=VERTICAL_PIN_HEIGHT),
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        material="steel",
        name="shoulder_pin",
    )
    wall_plate.visual(
        Cylinder(radius=VERTICAL_THRUST_RADIUS, length=VERTICAL_THRUST_THICKNESS),
        origin=Origin(
            xyz=(
                SHOULDER_X,
                0.0,
                SHOULDER_Z - ((VERTICAL_COLLAR_HEIGHT + VERTICAL_THRUST_THICKNESS) / 2.0),
            )
        ),
        material="black_oxide",
        name="shoulder_thrust",
    )
    wall_plate.visual(
        Cylinder(radius=VERTICAL_CAP_RADIUS, length=VERTICAL_CAP_THICKNESS),
        origin=Origin(
            xyz=(
                SHOULDER_X,
                0.0,
                SHOULDER_Z + (VERTICAL_COLLAR_HEIGHT / 2.0) + 0.003,
            )
        ),
        material="black_oxide",
        name="shoulder_cap",
    )

    primary_link = model.part("primary_link")
    primary_link.visual(
        _mesh(_primary_collar_shape(), "primary_shoulder_collar"),
        material="graphite",
        name="shoulder_collar",
    )
    primary_link.visual(
        _mesh(_primary_arm_body_shape(), "primary_arm_body"),
        material="aluminum",
        name="primary_arm_body",
    )
    primary_link.visual(
        _mesh(_primary_cover_shape(), "primary_cable_cover"),
        material="trim_black",
        name="primary_cable_cover",
    )
    primary_link.visual(
        Cylinder(radius=VERTICAL_PIN_RADIUS, length=VERTICAL_PIN_HEIGHT),
        origin=Origin(xyz=(PRIMARY_LENGTH, 0.0, 0.0)),
        material="steel",
        name="elbow_pin",
    )
    primary_link.visual(
        Cylinder(radius=VERTICAL_THRUST_RADIUS, length=VERTICAL_THRUST_THICKNESS),
        origin=Origin(
            xyz=(PRIMARY_LENGTH, 0.0, -((VERTICAL_COLLAR_HEIGHT + VERTICAL_THRUST_THICKNESS) / 2.0))
        ),
        material="black_oxide",
        name="elbow_thrust",
    )
    primary_link.visual(
        Cylinder(radius=VERTICAL_CAP_RADIUS, length=VERTICAL_CAP_THICKNESS),
        origin=Origin(xyz=(PRIMARY_LENGTH, 0.0, (VERTICAL_COLLAR_HEIGHT / 2.0) + 0.003)),
        material="black_oxide",
        name="elbow_cap",
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        _mesh(_secondary_collar_shape(), "secondary_elbow_collar"),
        material="graphite",
        name="elbow_collar",
    )
    secondary_link.visual(
        _mesh(_secondary_arm_body_shape(), "secondary_arm_body"),
        material="aluminum",
        name="secondary_arm_body",
    )
    secondary_link.visual(
        _mesh(_secondary_cover_shape(), "secondary_cable_cover"),
        material="trim_black",
        name="secondary_cable_cover",
    )
    secondary_link.visual(
        Cylinder(radius=PAN_PIN_RADIUS, length=VERTICAL_PIN_HEIGHT),
        origin=Origin(xyz=(SECONDARY_LENGTH, 0.0, 0.0)),
        material="steel",
        name="pan_pin",
    )
    secondary_link.visual(
        Cylinder(radius=PAN_THRUST_RADIUS, length=VERTICAL_THRUST_THICKNESS),
        origin=Origin(
            xyz=(SECONDARY_LENGTH, 0.0, -((PAN_COLLAR_HEIGHT + VERTICAL_THRUST_THICKNESS) / 2.0))
        ),
        material="black_oxide",
        name="pan_thrust",
    )
    secondary_link.visual(
        Cylinder(radius=0.017, length=VERTICAL_CAP_THICKNESS),
        origin=Origin(xyz=(SECONDARY_LENGTH, 0.0, (PAN_COLLAR_HEIGHT / 2.0) + 0.003)),
        material="black_oxide",
        name="pan_cap",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        _mesh(_pan_collar_shape(), "pan_head_collar"),
        material="graphite",
        name="pan_collar",
    )
    pan_head.visual(
        _mesh(_pan_head_body_shape(), "pan_head_body"),
        material="graphite",
        name="pan_head_body",
    )

    tilt_cradle = model.part("tilt_cradle")
    tilt_cradle.visual(
        _mesh(_tilt_cradle_pivot_shape(), "tilt_cradle_pivot"),
        material="steel",
        name="tilt_pivot",
    )
    tilt_cradle.visual(
        _mesh(_tilt_cradle_frame_shape(), "tilt_cradle_frame"),
        material="trim_black",
        name="tilt_cradle_frame",
    )

    model.articulation(
        "shoulder_swivel",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=primary_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.55, upper=1.55, effort=30.0, velocity=1.6),
    )
    model.articulation(
        "elbow_swivel",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(PRIMARY_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.50, upper=1.50, effort=22.0, velocity=1.8),
    )
    model.articulation(
        "head_pan",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=pan_head,
        origin=Origin(xyz=(SECONDARY_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.25, upper=1.25, effort=12.0, velocity=2.0),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_cradle,
        origin=Origin(xyz=(TILT_AXIS_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.60, upper=0.45, effort=8.0, velocity=2.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    pan_head = object_model.get_part("pan_head")
    tilt_cradle = object_model.get_part("tilt_cradle")

    shoulder_swivel = object_model.get_articulation("shoulder_swivel")
    elbow_swivel = object_model.get_articulation("elbow_swivel")
    head_pan = object_model.get_articulation("head_pan")
    head_tilt = object_model.get_articulation("head_tilt")

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
        pan_head,
        secondary_link,
        elem_a="pan_head_body",
        elem_b="pan_pin",
        reason="The compact pan spindle is represented as a solid steel axle captured inside the pan housing.",
    )
    ctx.allow_overlap(
        pan_head,
        tilt_cradle,
        elem_a="pan_head_body",
        elem_b="tilt_pivot",
        reason="The tilt hinge uses a captured axle sleeve through the yoke, represented here as a solid pivot body.",
    )

    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "shoulder_axis_vertical",
        shoulder_swivel.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical shoulder axis, got {shoulder_swivel.axis}",
    )
    ctx.check(
        "elbow_axis_vertical",
        elbow_swivel.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical elbow axis, got {elbow_swivel.axis}",
    )
    ctx.check(
        "pan_axis_vertical",
        head_pan.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical head pan axis, got {head_pan.axis}",
    )
    ctx.check(
        "tilt_axis_horizontal",
        head_tilt.axis == (0.0, 1.0, 0.0),
        details=f"expected horizontal head tilt axis, got {head_tilt.axis}",
    )
    ctx.check(
        "link_length_hierarchy",
        SECONDARY_LENGTH > PRIMARY_LENGTH,
        details="secondary link should be visibly longer than primary link",
    )
    ctx.check(
        "shoulder_origin_on_plate",
        isclose(shoulder_swivel.origin.xyz[0], SHOULDER_X, abs_tol=1e-9)
        and isclose(shoulder_swivel.origin.xyz[2], SHOULDER_Z, abs_tol=1e-9),
        details=f"unexpected shoulder origin {shoulder_swivel.origin.xyz}",
    )

    ctx.expect_contact(
        primary_link,
        wall_plate,
        elem_a="shoulder_collar",
        elem_b="shoulder_thrust",
        name="shoulder_joint_supported",
    )
    ctx.expect_contact(
        secondary_link,
        primary_link,
        elem_a="elbow_collar",
        elem_b="elbow_thrust",
        name="elbow_joint_supported",
    )
    ctx.expect_contact(
        pan_head,
        secondary_link,
        elem_a="pan_collar",
        elem_b="pan_thrust",
        name="pan_joint_supported",
    )
    ctx.expect_overlap(
        pan_head,
        tilt_cradle,
        axes="yz",
        elem_a="pan_head_body",
        elem_b="tilt_pivot",
        min_overlap=0.018,
        name="tilt_pivot_nested_in_yoke",
    )
    ctx.expect_gap(
        tilt_cradle,
        pan_head,
        axis="x",
        positive_elem="tilt_cradle_frame",
        negative_elem="pan_head_body",
        min_gap=0.004,
        name="head_frame_clears_yoke_at_rest",
    )

    with ctx.pose({head_pan: 0.80, head_tilt: 0.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="head_pose_up_clear")

    with ctx.pose({head_pan: -1.00, head_tilt: -0.50}):
        ctx.fail_if_parts_overlap_in_current_pose(name="head_pose_down_clear")

    with ctx.pose({shoulder_swivel: 0.95, elbow_swivel: -1.05}):
        ctx.expect_gap(
            secondary_link,
            wall_plate,
            axis="x",
            positive_elem="secondary_arm_body",
            negative_elem="wall_plate_body",
            min_gap=0.030,
            name="folded_arm_keeps_secondary_clear_of_wall_plate",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
