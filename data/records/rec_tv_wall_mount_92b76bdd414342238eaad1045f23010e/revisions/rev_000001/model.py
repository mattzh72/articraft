from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

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


WALL_PLATE_THICKNESS = 0.006
WALL_PLATE_WIDTH = 0.095
WALL_PLATE_HEIGHT = 0.190
WALL_PLATE_FRONT_X = -0.040

PRIMARY_LINK_LENGTH = 0.235
SECONDARY_LINK_LENGTH = 0.215
TILT_AXIS_X = 0.024


def _box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]):
    x, y, z = center
    return (
        cq.Workplane("YZ")
        .center(y, z)
        .circle(radius)
        .extrude(length)
        .translate((x - length / 2.0, 0.0, 0.0))
    )


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]):
    x, y, z = center
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length)
        .translate((0.0, y - length / 2.0, 0.0))
    )


def _wall_bracket_shape():
    plate_center_x = WALL_PLATE_FRONT_X - WALL_PLATE_THICKNESS / 2.0

    side = (
        cq.Workplane("XZ")
        .moveTo(WALL_PLATE_FRONT_X - WALL_PLATE_THICKNESS, -0.095)
        .lineTo(WALL_PLATE_FRONT_X - WALL_PLATE_THICKNESS, 0.095)
        .lineTo(WALL_PLATE_FRONT_X, 0.095)
        .lineTo(WALL_PLATE_FRONT_X, 0.064)
        .lineTo(-0.020, 0.046)
        .lineTo(0.0, 0.046)
        .lineTo(0.0, -0.046)
        .lineTo(-0.020, -0.046)
        .lineTo(WALL_PLATE_FRONT_X, -0.064)
        .lineTo(WALL_PLATE_FRONT_X, -0.095)
        .close()
        .extrude(0.044)
        .translate((0.0, -0.022, 0.0))
    )
    clevis_slot = _box((0.012, 0.024, 0.070), (-0.006, 0.0, 0.0))

    hole_positions = [
        (0.0, 0.060),
        (0.0, -0.060),
        (0.025, 0.0),
        (-0.025, 0.0),
    ]
    wall_holes = (
        cq.Workplane("YZ")
        .pushPoints(hole_positions)
        .circle(0.0042)
        .extrude(WALL_PLATE_THICKNESS + 0.004)
        .translate((plate_center_x - WALL_PLATE_THICKNESS / 2.0 - 0.002, 0.0, 0.0))
    )

    return side.cut(clevis_slot).cut(wall_holes)


def _primary_link_shape():
    arm = (
        cq.Workplane("XZ")
        .moveTo(0.0, -0.020)
        .lineTo(0.016, -0.022)
        .lineTo(0.036, -0.018)
        .lineTo(0.178, -0.012)
        .lineTo(0.214, -0.016)
        .lineTo(PRIMARY_LINK_LENGTH, -0.020)
        .lineTo(PRIMARY_LINK_LENGTH, 0.020)
        .lineTo(0.214, 0.016)
        .lineTo(0.178, 0.012)
        .lineTo(0.036, 0.018)
        .lineTo(0.016, 0.022)
        .lineTo(0.0, 0.020)
        .close()
        .extrude(0.028)
        .translate((0.0, -0.014, 0.0))
    )
    clevis_slot = _box((0.024, 0.022, 0.032), (0.222, 0.0, 0.0))
    window = _box((0.094, 0.010, 0.008), (0.116, 0.0, 0.0))
    return arm.cut(window).cut(clevis_slot)


def _secondary_link_shape():
    arm = (
        cq.Workplane("XZ")
        .moveTo(0.0, -0.018)
        .lineTo(0.016, -0.020)
        .lineTo(0.034, -0.016)
        .lineTo(0.160, -0.011)
        .lineTo(0.190, -0.014)
        .lineTo(SECONDARY_LINK_LENGTH, -0.024)
        .lineTo(SECONDARY_LINK_LENGTH, 0.024)
        .lineTo(0.190, 0.014)
        .lineTo(0.160, 0.011)
        .lineTo(0.034, 0.016)
        .lineTo(0.016, 0.020)
        .lineTo(0.0, 0.018)
        .close()
        .extrude(0.028)
        .translate((0.0, -0.014, 0.0))
    )
    window = _box((0.082, 0.010, 0.007), (0.108, 0.0, 0.0))
    return arm.cut(window)


def _head_swivel_shape():
    body = (
        cq.Workplane("XZ")
        .moveTo(0.0, -0.020)
        .lineTo(0.010, -0.022)
        .lineTo(0.020, -0.024)
        .lineTo(TILT_AXIS_X, -0.024)
        .lineTo(TILT_AXIS_X, 0.024)
        .lineTo(0.020, 0.024)
        .lineTo(0.010, 0.022)
        .lineTo(0.0, 0.020)
        .close()
        .extrude(0.076)
        .translate((0.0, -0.038, 0.0))
    )
    yoke_slot = _box((0.022, 0.024, 0.052), (0.017, 0.0, 0.0))
    swivel_clear = _box((0.010, 0.020, 0.018), (0.006, 0.0, 0.0))
    return body.cut(yoke_slot).cut(swivel_clear)


def _vesa_plate_shape():
    tilt_tab = _box((0.008, 0.024, 0.024), (0.000, 0.0, 0.0))
    stem = _box((0.018, 0.024, 0.040), (0.011, 0.0, 0.0))
    web = _box((0.020, 0.040, 0.052), (0.017, 0.0, 0.0))
    plate = _box((0.006, 0.118, 0.118), (0.028, 0.0, 0.0))
    vesa_holes = (
        cq.Workplane("YZ")
        .pushPoints([(-0.050, -0.050), (-0.050, 0.050), (0.050, -0.050), (0.050, 0.050)])
        .circle(0.0032)
        .extrude(0.012)
        .translate((0.022, 0.0, 0.0))
    )
    center_window = _box((0.008, 0.028, 0.028), (0.028, 0.0, 0.0))

    return tilt_tab.union(stem).union(web).union(plate).cut(vesa_holes).cut(center_window)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_display_arm")

    model.material("powder_black", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("satin_graphite", rgba=(0.33, 0.35, 0.37, 1.0))
    model.material("machined_aluminum", rgba=(0.71, 0.73, 0.75, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        mesh_from_cadquery(_wall_bracket_shape(), "wall_bracket"),
        material="powder_black",
        name="bracket_shell",
    )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.050, 0.095, 0.190)),
        mass=1.35,
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
    )

    primary_link = model.part("primary_link")
    primary_link.visual(
        mesh_from_cadquery(_primary_link_shape(), "primary_link"),
        material="satin_graphite",
        name="primary_shell",
    )
    primary_link.inertial = Inertial.from_geometry(
        Box((0.235, 0.036, 0.040)),
        mass=0.82,
        origin=Origin(xyz=(0.118, 0.0, 0.0)),
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        mesh_from_cadquery(_secondary_link_shape(), "secondary_link"),
        material="satin_graphite",
        name="secondary_shell",
    )
    secondary_link.inertial = Inertial.from_geometry(
        Box((0.215, 0.042, 0.050)),
        mass=0.68,
        origin=Origin(xyz=(0.108, 0.0, 0.0)),
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        Box((0.020, 0.034, 0.028)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material="powder_black",
        name="swivel_body",
    )
    head_swivel.visual(
        Box((0.012, 0.010, 0.040)),
        origin=Origin(xyz=(TILT_AXIS_X, 0.021, 0.0)),
        material="powder_black",
        name="yoke_arm_upper",
    )
    head_swivel.visual(
        Box((0.012, 0.010, 0.040)),
        origin=Origin(xyz=(TILT_AXIS_X, -0.021, 0.0)),
        material="powder_black",
        name="yoke_arm_lower",
    )
    head_swivel.inertial = Inertial.from_geometry(
        Box((0.040, 0.082, 0.050)),
        mass=0.24,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    vesa_plate = model.part("vesa_plate")
    vesa_plate.visual(
        Cylinder(radius=0.004, length=0.032),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_aluminum",
        name="tilt_trunnion",
    )
    vesa_plate.visual(
        Box((0.014, 0.020, 0.034)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material="machined_aluminum",
        name="tilt_stem",
    )
    vesa_plate.visual(
        Box((0.014, 0.042, 0.056)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material="machined_aluminum",
        name="plate_web",
    )
    vesa_plate.visual(
        Box((0.006, 0.118, 0.118)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material="machined_aluminum",
        name="vesa_panel",
    )
    vesa_plate.inertial = Inertial.from_geometry(
        Box((0.031, 0.118, 0.118)),
        mass=0.34,
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=primary_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=1.10, effort=28.0, velocity=1.6),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(PRIMARY_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.65, upper=1.35, effort=24.0, velocity=1.8),
    )
    model.articulation(
        "head_swivel_joint",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=head_swivel,
        origin=Origin(xyz=(SECONDARY_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.70, upper=1.70, effort=10.0, velocity=2.4),
    )
    model.articulation(
        "head_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=vesa_plate,
        origin=Origin(xyz=(TILT_AXIS_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.60, upper=0.80, effort=8.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    head_swivel = object_model.get_part("head_swivel")
    vesa_plate = object_model.get_part("vesa_plate")

    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    head_swivel_joint = object_model.get_articulation("head_swivel_joint")
    head_tilt_joint = object_model.get_articulation("head_tilt_joint")

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

    ctx.check(
        "all_parts_present",
        all(
            part is not None
            for part in (wall_bracket, primary_link, secondary_link, head_swivel, vesa_plate)
        ),
        "expected wall bracket, both links, swivel head, and VESA plate parts",
    )
    ctx.check(
        "all_joints_present",
        all(
            joint is not None
            for joint in (shoulder_pitch, elbow_pitch, head_swivel_joint, head_tilt_joint)
        ),
        "expected shoulder, elbow, swivel, and tilt revolute joints",
    )

    def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
        return all(isclose(a, b, abs_tol=1e-6) for a, b in zip(actual, expected))

    ctx.check(
        "joint_axes_match_mechanism",
        _axis_matches(shoulder_pitch.axis, (0.0, -1.0, 0.0))
        and _axis_matches(elbow_pitch.axis, (0.0, -1.0, 0.0))
        and _axis_matches(head_swivel_joint.axis, (0.0, 0.0, 1.0))
        and _axis_matches(head_tilt_joint.axis, (0.0, 1.0, 0.0)),
        "shoulder/elbow should pitch, head should swivel on Z, and plate should tilt on Y",
    )

    ctx.expect_contact(primary_link, wall_bracket, contact_tol=0.0015, name="shoulder_mount_contact")
    ctx.expect_contact(secondary_link, primary_link, contact_tol=0.0015, name="elbow_mount_contact")
    ctx.expect_contact(head_swivel, secondary_link, contact_tol=0.0015, name="swivel_mount_contact")
    ctx.expect_contact(vesa_plate, head_swivel, contact_tol=0.0015, name="tilt_mount_contact")

    with ctx.pose({shoulder_pitch: 0.65}):
        ctx.expect_origin_gap(
            secondary_link,
            wall_bracket,
            axis="z",
            min_gap=0.12,
            name="shoulder_positive_raises_elbow",
        )

    with ctx.pose({head_swivel_joint: 0.60}):
        ctx.expect_origin_gap(
            vesa_plate,
            secondary_link,
            axis="y",
            min_gap=0.012,
            name="positive_swivel_moves_head_sideways",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
