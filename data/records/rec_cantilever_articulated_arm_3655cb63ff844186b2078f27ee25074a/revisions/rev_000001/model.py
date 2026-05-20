from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BACKPLATE_THICKNESS = 0.014
BACKPLATE_WIDTH = 0.180
BACKPLATE_HEIGHT = 0.280

SHOULDER_GAP = 0.040
SHOULDER_CHEEK_THICKNESS = 0.022
SHOULDER_CHEEK_LENGTH = 0.048
SHOULDER_CHEEK_HEIGHT = 0.132

FIRST_LINK_LENGTH = 0.330
ELBOW_GAP = 0.038
ELBOW_CHEEK_THICKNESS = 0.018
ELBOW_CHEEK_LENGTH = 0.050
ELBOW_CHEEK_HEIGHT = 0.105

FOREARM_LENGTH = 0.260
WRIST_GAP = 0.022
WRIST_CHEEK_THICKNESS = 0.014
WRIST_CHEEK_LENGTH = 0.040
WRIST_CHEEK_HEIGHT = 0.072

END_PLATE_SIZE = 0.090
END_PLATE_THICKNESS = 0.012

SHOULDER_BARREL_RADIUS = 0.018
ELBOW_BARREL_RADIUS = 0.024
WRIST_BARREL_RADIUS = 0.012


def _union_all(*shapes: cq.Workplane) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _y_cylinder(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((x, y - length / 2.0, z))
    )


def _x_cylinder(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((x - length / 2.0, y, z))
    )


def _wall_mount_shape() -> cq.Workplane:
    backplate = (
        cq.Workplane("XY")
        .box(BACKPLATE_THICKNESS, BACKPLATE_WIDTH, BACKPLATE_HEIGHT)
        .translate((-0.102, 0.0, 0.0))
        .edges("|X")
        .fillet(0.008)
    )
    rear_mass = (
        cq.Workplane("YZ")
        .rect(0.154, 0.224)
        .workplane(offset=0.050)
        .rect(0.122, 0.186)
        .loft(combine=True)
        .translate((-0.092, 0.0, 0.0))
    )
    shoulder_wedge = (
        cq.Workplane("YZ")
        .rect(0.110, 0.158)
        .workplane(offset=0.032)
        .rect(0.080, 0.120)
        .loft(combine=True)
        .translate((-0.038, 0.0, 0.0))
    )

    cheek_offset = SHOULDER_GAP / 2.0 + SHOULDER_CHEEK_THICKNESS / 2.0
    shoulder_cheek_left = (
        cq.Workplane("XY")
        .box(0.040, SHOULDER_CHEEK_THICKNESS, SHOULDER_CHEEK_HEIGHT)
        .translate((-0.020, cheek_offset, 0.0))
    )
    shoulder_cheek_right = (
        cq.Workplane("XY")
        .box(0.040, SHOULDER_CHEEK_THICKNESS, SHOULDER_CHEEK_HEIGHT)
        .translate((-0.020, -cheek_offset, 0.0))
    )
    top_bridge = cq.Workplane("XY").box(0.062, 0.118, 0.018).translate((-0.032, 0.0, 0.058))
    bottom_bridge = cq.Workplane("XY").box(0.062, 0.118, 0.018).translate((-0.032, 0.0, -0.058))
    side_web_left = cq.Workplane("XY").box(0.050, 0.012, 0.050).translate((-0.024, 0.028, 0.0))
    side_web_right = cq.Workplane("XY").box(0.050, 0.012, 0.050).translate((-0.024, -0.028, 0.0))

    cap_y = SHOULDER_GAP / 2.0 + SHOULDER_CHEEK_THICKNESS + 0.003
    shoulder_caps = _union_all(
        _y_cylinder(0.030, 0.006, x=-0.030, y=cap_y, z=0.0),
        _y_cylinder(0.030, 0.006, x=-0.030, y=-cap_y, z=0.0),
    )
    mount_bolts = (
        cq.Workplane("YZ")
        .pushPoints([(-0.075, -0.098), (0.075, -0.098), (-0.075, 0.098), (0.075, 0.098)])
        .circle(0.008)
        .extrude(0.006)
        .translate((-0.095, 0.0, 0.0))
    )
    cap_bolts = _union_all(
        cq.Workplane("XZ")
        .pushPoints([(-0.044, -0.032), (-0.022, -0.032), (-0.044, 0.032), (-0.022, 0.032)])
        .circle(0.0038)
        .extrude(0.005)
        .translate((0.0, SHOULDER_GAP / 2.0 + SHOULDER_CHEEK_THICKNESS, 0.0)),
        cq.Workplane("XZ")
        .pushPoints([(-0.044, -0.032), (-0.022, -0.032), (-0.044, 0.032), (-0.022, 0.032)])
        .circle(0.0038)
        .extrude(0.005)
        .translate((0.0, -(SHOULDER_GAP / 2.0 + SHOULDER_CHEEK_THICKNESS + 0.005), 0.0)),
    )

    return _union_all(
        backplate,
        rear_mass,
        shoulder_wedge,
        shoulder_cheek_left,
        shoulder_cheek_right,
        top_bridge,
        bottom_bridge,
        side_web_left,
        side_web_right,
        shoulder_caps,
        mount_bolts,
        cap_bolts,
    ).clean()


def _first_link_shape() -> cq.Workplane:
    shoulder_knuckle = _union_all(
        _y_cylinder(SHOULDER_BARREL_RADIUS, SHOULDER_GAP, x=SHOULDER_BARREL_RADIUS, y=0.0, z=0.0),
        cq.Workplane("XY").box(0.028, 0.022, 0.036).translate((0.028, 0.0, 0.0)),
    )
    beam = (
        cq.Workplane("YZ")
        .rect(0.060, 0.086)
        .workplane(offset=0.210)
        .rect(0.042, 0.064)
        .loft(combine=True)
        .translate((0.055, 0.0, 0.0))
    )
    rib_left = cq.Workplane("XY").box(0.188, 0.005, 0.030).translate((0.166, 0.024, 0.0))
    rib_right = cq.Workplane("XY").box(0.188, 0.005, 0.030).translate((0.166, -0.024, 0.0))

    elbow_center_x = FIRST_LINK_LENGTH - 0.024
    elbow_bridge = cq.Workplane("XY").box(0.040, 0.034, 0.050).translate((FIRST_LINK_LENGTH - 0.050, 0.0, 0.0))
    elbow_top = cq.Workplane("XY").box(0.046, 0.060, 0.014).translate((FIRST_LINK_LENGTH - 0.042, 0.0, 0.048))
    elbow_bottom = cq.Workplane("XY").box(0.046, 0.060, 0.014).translate((FIRST_LINK_LENGTH - 0.042, 0.0, -0.048))
    elbow_cheek_offset = ELBOW_GAP / 2.0 + ELBOW_CHEEK_THICKNESS / 2.0
    elbow_cheek_left = cq.Workplane("XY").box(0.048, ELBOW_CHEEK_THICKNESS, 0.110).translate((elbow_center_x, elbow_cheek_offset, 0.0))
    elbow_cheek_right = cq.Workplane("XY").box(0.048, ELBOW_CHEEK_THICKNESS, 0.110).translate((elbow_center_x, -elbow_cheek_offset, 0.0))
    elbow_caps = _union_all(
        _y_cylinder(0.028, 0.006, x=FIRST_LINK_LENGTH - 0.028, y=ELBOW_GAP / 2.0 + ELBOW_CHEEK_THICKNESS + 0.003, z=0.0),
        _y_cylinder(0.028, 0.006, x=FIRST_LINK_LENGTH - 0.028, y=-(ELBOW_GAP / 2.0 + ELBOW_CHEEK_THICKNESS + 0.003), z=0.0),
    )
    elbow_cap_bolts = _union_all(
        cq.Workplane("XZ")
        .pushPoints(
            [
                (FIRST_LINK_LENGTH - 0.046, -0.034),
                (FIRST_LINK_LENGTH - 0.022, -0.034),
                (FIRST_LINK_LENGTH - 0.046, 0.034),
                (FIRST_LINK_LENGTH - 0.022, 0.034),
            ]
        )
        .circle(0.0035)
        .extrude(0.005)
        .translate((0.0, ELBOW_GAP / 2.0 + ELBOW_CHEEK_THICKNESS, 0.0)),
        cq.Workplane("XZ")
        .pushPoints(
            [
                (FIRST_LINK_LENGTH - 0.046, -0.034),
                (FIRST_LINK_LENGTH - 0.022, -0.034),
                (FIRST_LINK_LENGTH - 0.046, 0.034),
                (FIRST_LINK_LENGTH - 0.022, 0.034),
            ]
        )
        .circle(0.0035)
        .extrude(0.005)
        .translate((0.0, -(ELBOW_GAP / 2.0 + ELBOW_CHEEK_THICKNESS + 0.005), 0.0)),
    )

    return _union_all(
        shoulder_knuckle,
        beam,
        rib_left,
        rib_right,
        elbow_bridge,
        elbow_top,
        elbow_bottom,
        elbow_cheek_left,
        elbow_cheek_right,
        elbow_caps,
        elbow_cap_bolts,
    ).clean()


def _forearm_shape() -> cq.Workplane:
    elbow_knuckle = _union_all(
        _y_cylinder(ELBOW_BARREL_RADIUS, ELBOW_GAP, x=ELBOW_BARREL_RADIUS, y=0.0, z=0.0),
        cq.Workplane("XY").box(0.028, 0.020, 0.048).translate((0.030, 0.0, 0.0)),
        cq.Workplane("XY").box(0.020, 0.018, 0.016).translate((0.018, 0.0, -0.030)),
    )
    beam = (
        cq.Workplane("YZ")
        .rect(0.046, 0.066)
        .workplane(offset=0.172)
        .rect(0.032, 0.048)
        .loft(combine=True)
        .translate((0.052, 0.0, 0.0))
    )
    rib_left = cq.Workplane("XY").box(0.120, 0.004, 0.022).translate((0.138, 0.018, 0.0))
    rib_right = cq.Workplane("XY").box(0.120, 0.004, 0.022).translate((0.138, -0.018, 0.0))

    wrist_center_x = FOREARM_LENGTH - 0.020
    wrist_bridge = cq.Workplane("XY").box(0.030, 0.024, 0.032).translate((FOREARM_LENGTH - 0.046, 0.0, 0.0))
    wrist_top = cq.Workplane("XY").box(0.036, 0.044, 0.012).translate((FOREARM_LENGTH - 0.034, 0.0, 0.032))
    wrist_bottom = cq.Workplane("XY").box(0.036, 0.044, 0.012).translate((FOREARM_LENGTH - 0.034, 0.0, -0.032))
    wrist_cheek_offset = WRIST_GAP / 2.0 + WRIST_CHEEK_THICKNESS / 2.0
    wrist_cheek_left = cq.Workplane("XY").box(0.040, WRIST_CHEEK_THICKNESS, 0.066).translate((wrist_center_x, wrist_cheek_offset, 0.0))
    wrist_cheek_right = cq.Workplane("XY").box(0.040, WRIST_CHEEK_THICKNESS, 0.066).translate((wrist_center_x, -wrist_cheek_offset, 0.0))
    wrist_caps = _union_all(
        _y_cylinder(0.020, 0.005, x=FOREARM_LENGTH - 0.020, y=WRIST_GAP / 2.0 + WRIST_CHEEK_THICKNESS + 0.0025, z=0.0),
        _y_cylinder(0.020, 0.005, x=FOREARM_LENGTH - 0.020, y=-(WRIST_GAP / 2.0 + WRIST_CHEEK_THICKNESS + 0.0025), z=0.0),
    )

    return _union_all(
        elbow_knuckle,
        beam,
        rib_left,
        rib_right,
        wrist_bridge,
        wrist_top,
        wrist_bottom,
        wrist_cheek_left,
        wrist_cheek_right,
        wrist_caps,
    ).clean()


def _end_plate_shape() -> cq.Workplane:
    wrist_hub = _union_all(
        _y_cylinder(WRIST_BARREL_RADIUS, WRIST_GAP, x=WRIST_BARREL_RADIUS, y=0.0, z=0.0),
        cq.Workplane("XY").box(0.014, 0.012, 0.022).translate((0.014, 0.0, 0.0)),
    )
    standoff = (
        cq.Workplane("YZ")
        .rect(0.018, 0.028)
        .workplane(offset=0.050)
        .rect(0.040, 0.064)
        .loft(combine=True)
        .translate((0.032, 0.0, 0.0))
    )
    gusset_top = cq.Workplane("XY").box(0.030, 0.012, 0.014).translate((0.034, 0.0, 0.022))
    gusset_bottom = cq.Workplane("XY").box(0.030, 0.012, 0.014).translate((0.034, 0.0, -0.022))
    plate_body = (
        cq.Workplane("XY")
        .box(END_PLATE_THICKNESS, END_PLATE_SIZE, END_PLATE_SIZE)
        .translate((0.084, 0.0, 0.0))
        .edges("|X")
        .fillet(0.004)
    )
    rim = cq.Workplane("XY").box(0.006, 0.102, 0.102).translate((0.078, 0.0, 0.0))
    plate = _union_all(plate_body, rim)
    recess = cq.Workplane("XY").box(0.006, 0.060, 0.060).translate((0.087, 0.0, 0.0))
    center_hole = _x_cylinder(0.012, 0.036, x=0.084, y=0.0, z=0.0)
    corner_holes = (
        cq.Workplane("YZ")
        .pushPoints([(-0.028, -0.028), (0.028, -0.028), (-0.028, 0.028), (0.028, 0.028)])
        .circle(0.0042)
        .extrude(0.036)
        .translate((0.066, 0.0, 0.0))
    )
    tool_plate = plate.cut(recess).cut(center_hole).cut(corner_holes)

    return _union_all(wrist_hub, standoff, gusset_top, gusset_bottom, tool_plate).clean()


def _axis_is_supported_horizontal(axis: tuple[float, float, float]) -> bool:
    return isclose(axis[0], 0.0, abs_tol=1e-6) and isclose(abs(axis[1]), 1.0, abs_tol=1e-6) and isclose(axis[2], 0.0, abs_tol=1e-6)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_transfer_arm")

    model.material("powder_steel", rgba=(0.21, 0.23, 0.26, 1.0))
    model.material("machine_gray", rgba=(0.47, 0.50, 0.54, 1.0))
    model.material("cast_aluminum", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("light_alloy", rgba=(0.76, 0.79, 0.82, 1.0))

    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        mesh_from_cadquery(_wall_mount_shape(), "wall_mount"),
        material="powder_steel",
        name="wall_mount_shell",
    )
    wall_mount.inertial = Inertial.from_geometry(
        Box((0.160, 0.180, 0.300)),
        mass=18.0,
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
    )

    first_link = model.part("first_link")
    first_link.visual(
        mesh_from_cadquery(_first_link_shape(), "first_link"),
        material="machine_gray",
        name="first_link_shell",
    )
    first_link.inertial = Inertial.from_geometry(
        Box((FIRST_LINK_LENGTH + 0.050, 0.080, 0.120)),
        mass=9.0,
        origin=Origin(xyz=(FIRST_LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_shape(), "forearm"),
        material="cast_aluminum",
        name="forearm_shell",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH + 0.050, 0.060, 0.120)),
        mass=5.8,
        origin=Origin(xyz=(FOREARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        mesh_from_cadquery(_end_plate_shape(), "end_plate"),
        material="light_alloy",
        name="end_plate_shell",
    )
    end_plate.inertial = Inertial.from_geometry(
        Box((0.110, 0.102, 0.102)),
        mass=2.2,
        origin=Origin(xyz=(0.062, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=wall_mount,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.30, upper=0.32, effort=140.0, velocity=1.2),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=forearm,
        origin=Origin(xyz=(FIRST_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=0.05, effort=95.0, velocity=1.5),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=end_plate,
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.40, upper=0.35, effort=36.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_mount = object_model.get_part("wall_mount")
    first_link = object_model.get_part("first_link")
    forearm = object_model.get_part("forearm")
    end_plate = object_model.get_part("end_plate")

    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist = object_model.get_articulation("wrist_pitch")

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
        "primary_parts_present",
        True,
        details="wall_mount, first_link, forearm, and end_plate should all resolve",
    )
    ctx.check(
        "horizontal_supported_pivot_axes",
        _axis_is_supported_horizontal(shoulder.axis)
        and _axis_is_supported_horizontal(elbow.axis)
        and _axis_is_supported_horizontal(wrist.axis),
        details=f"axes={shoulder.axis}, {elbow.axis}, {wrist.axis}",
    )

    ctx.expect_contact(
        wall_mount,
        first_link,
        contact_tol=0.0008,
        name="shoulder_knuckle_is_supported",
    )
    ctx.expect_contact(
        first_link,
        forearm,
        contact_tol=0.0008,
        name="elbow_knuckle_is_supported",
    )
    ctx.expect_contact(
        forearm,
        end_plate,
        contact_tol=0.0008,
        name="wrist_knuckle_is_supported",
    )
    ctx.expect_origin_gap(
        end_plate,
        wall_mount,
        axis="x",
        min_gap=0.45,
        name="arm_reaches_out_from_wall",
    )

    with ctx.pose(shoulder_pitch=-0.28, elbow_pitch=-0.65, wrist_pitch=0.35):
        ctx.expect_origin_gap(
            end_plate,
            wall_mount,
            axis="x",
            min_gap=0.45,
            name="folded_service_tip_stays_forward_of_mount",
        )
        ctx.expect_origin_gap(
            forearm,
            wall_mount,
            axis="x",
            min_gap=0.30,
            name="folded_service_forearm_clears_mount",
        )
        ctx.expect_origin_distance(
            end_plate,
            first_link,
            axes="xz",
            min_dist=0.44,
            name="folded_service_tip_clears_upper_arm",
        )

    with ctx.pose(shoulder_pitch=0.28, elbow_pitch=0.0, wrist_pitch=-0.35):
        ctx.expect_origin_gap(
            end_plate,
            first_link,
            axis="x",
            min_gap=0.22,
            name="downreach_tip_stays_ahead_of_upper_arm",
        )
        ctx.expect_origin_gap(
            forearm,
            wall_mount,
            axis="x",
            min_gap=0.31,
            name="downreach_forearm_stays_off_mount",
        )
        ctx.expect_origin_distance(
            end_plate,
            wall_mount,
            axes="xz",
            min_dist=0.58,
            name="downreach_tip_keeps_clear_working_envelope",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
