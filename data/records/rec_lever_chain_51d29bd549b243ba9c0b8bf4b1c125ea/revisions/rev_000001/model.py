from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, degrees, hypot

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


PLATE_T = 0.005
EYE_T = 0.006
EAR_T = 0.003
INNER_COLLAR_T = 0.0015
EAR_CENTER_Y = 0.006
COLLAR_T = 0.004
OUTER_COLLAR_T = 0.002
BAR_H = 0.0095
JOINT_R = 0.0086
HOLE_R = 0.0041

LINK1_VEC = (0.062, -0.018)
LINK2_VEC = (0.058, 0.026)
LINK3_VEC = (0.070, -0.014)


def _bar_angle(dx: float, dz: float) -> float:
    return degrees(atan2(dz, dx))


def _bar_length(dx: float, dz: float) -> float:
    return hypot(dx, dz)


def _bar_dir(dx: float, dz: float) -> tuple[float, float, float]:
    length = _bar_length(dx, dz)
    return dx / length, dz / length, length


def _along(dx: float, dz: float, distance: float) -> tuple[float, float]:
    ux, uz, _ = _bar_dir(dx, dz)
    return ux * distance, uz * distance


def _box_xz(
    length_x: float,
    thickness_y: float,
    height_z: float,
    *,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    angle_deg: float = 0.0,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(length_x, thickness_y, height_z)
    if angle_deg:
        shape = shape.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)
    return shape.translate(center)


def _y_cylinder(
    radius: float,
    length_y: float,
    *,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    shape = cq.Workplane("XY").circle(radius).extrude(length_y)
    shape = shape.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    return shape.translate((center[0], center[1] - 0.5 * length_y, center[2]))


def _slot_y(
    slot_length: float,
    slot_diameter: float,
    length_y: float,
    *,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    angle_deg: float = 0.0,
) -> cq.Workplane:
    shape = cq.Workplane("XY").slot2D(slot_length, slot_diameter).extrude(length_y)
    shape = shape.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    if angle_deg:
        shape = shape.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)
    return shape.translate((center[0], center[1] - 0.5 * length_y, center[2]))


def _mount_hole(
    radius: float,
    depth_z: float,
    *,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(depth_z).translate(center)


def _make_root_bracket() -> cq.Workplane:
    base = _box_xz(0.050, 0.022, 0.008, center=(-0.030, 0.0, -0.030))
    upright = _box_xz(0.010, 0.010, 0.026, center=(-0.022, 0.0, -0.015))
    stop_pad = _box_xz(0.010, 0.006, 0.006, center=(-0.008, 0.0, 0.012), angle_deg=-10.0)

    left_cheek = (
        _y_cylinder(JOINT_R, EAR_T, center=(0.0, EAR_CENTER_Y, 0.0))
        .union(_y_cylinder(0.0070, INNER_COLLAR_T, center=(0.0, 0.00375, 0.0)))
        .union(_box_xz(0.020, EAR_T, 0.017, center=(-0.010, EAR_CENTER_Y, -0.004)))
        .union(_box_xz(0.018, 0.004, 0.008, center=(-0.020, 0.0075, -0.016), angle_deg=24.0))
    )
    right_cheek = (
        _y_cylinder(JOINT_R, EAR_T, center=(0.0, -EAR_CENTER_Y, 0.0))
        .union(_y_cylinder(0.0070, INNER_COLLAR_T, center=(0.0, -0.00375, 0.0)))
        .union(_box_xz(0.020, EAR_T, 0.017, center=(-0.010, -EAR_CENTER_Y, -0.004)))
        .union(_box_xz(0.018, 0.004, 0.008, center=(-0.020, -0.0075, -0.016), angle_deg=24.0))
    )

    body = base.union(upright).union(stop_pad).union(left_cheek).union(right_cheek)

    pivot_hole_left = _y_cylinder(HOLE_R, EAR_T + 0.001, center=(0.0, EAR_CENTER_Y, 0.0))
    pivot_hole_right = _y_cylinder(HOLE_R, EAR_T + 0.001, center=(0.0, -EAR_CENTER_Y, 0.0))
    mount_hole_a = _mount_hole(0.0036, 0.012, center=(-0.028, 0.0, -0.035))
    mount_hole_b = _mount_hole(0.0036, 0.012, center=(0.000, 0.0, -0.035))

    return body.cut(pivot_hole_left).cut(pivot_hole_right).cut(mount_hole_a).cut(mount_hole_b)


def _make_link_one() -> cq.Workplane:
    dx, dz = LINK1_VEC
    angle = _bar_angle(dx, dz)
    ux, uz, length = _bar_dir(dx, dz)
    eye_lug_len = 0.014
    bar_start = eye_lug_len
    bar_end = length - 0.016
    bar_len = bar_end - bar_start
    bar_center = bar_start + 0.5 * bar_len
    ear_center_d = length - 0.006
    web_center_d = length - 0.015
    rib_x, rib_z = _along(dx, dz, 0.028)

    shape = (
        _y_cylinder(JOINT_R, EYE_T, center=(0.0, 0.0, 0.0))
        .union(_box_xz(eye_lug_len, EYE_T, BAR_H, center=(0.5 * eye_lug_len * ux, 0.0, 0.5 * eye_lug_len * uz), angle_deg=angle))
        .union(_box_xz(bar_len, PLATE_T, BAR_H, center=(ux * bar_center, 0.0, uz * bar_center), angle_deg=angle))
        .union(_box_xz(0.020, OUTER_COLLAR_T, 0.010, center=(rib_x, 0.0038, rib_z - 0.002), angle_deg=angle - 8.0))
        .union(_box_xz(0.010, OUTER_COLLAR_T, 0.005, center=(rib_x - 0.006, -0.0038, rib_z - 0.004), angle_deg=angle - 18.0))
        .union(_y_cylinder(JOINT_R, EAR_T, center=(dx, EAR_CENTER_Y, dz)))
        .union(_y_cylinder(JOINT_R, EAR_T, center=(dx, -EAR_CENTER_Y, dz)))
        .union(_y_cylinder(0.0070, INNER_COLLAR_T, center=(dx, 0.00375, dz)))
        .union(_y_cylinder(0.0070, INNER_COLLAR_T, center=(dx, -0.00375, dz)))
        .union(_box_xz(0.012, EAR_T, 0.008, center=(ux * ear_center_d, EAR_CENTER_Y, uz * ear_center_d), angle_deg=angle))
        .union(_box_xz(0.012, EAR_T, 0.008, center=(ux * ear_center_d, -EAR_CENTER_Y, uz * ear_center_d), angle_deg=angle))
        .union(_box_xz(0.018, 0.003, 0.007, center=(ux * web_center_d, 0.0070, uz * web_center_d), angle_deg=angle))
        .union(_box_xz(0.018, 0.003, 0.007, center=(ux * web_center_d, -0.0070, uz * web_center_d), angle_deg=angle))
    )

    return shape.cut(_y_cylinder(HOLE_R, EYE_T + 0.001, center=(0.0, 0.0, 0.0))).cut(
        _y_cylinder(HOLE_R, EAR_T + 0.001, center=(dx, EAR_CENTER_Y, dz))
    ).cut(_y_cylinder(HOLE_R, EAR_T + 0.001, center=(dx, -EAR_CENTER_Y, dz)))


def _make_link_two() -> cq.Workplane:
    dx, dz = LINK2_VEC
    angle = _bar_angle(dx, dz)
    ux, uz, length = _bar_dir(dx, dz)
    eye_lug_len = 0.014
    bar_start = eye_lug_len
    bar_end = length - 0.016
    bar_len = bar_end - bar_start
    bar_center = bar_start + 0.5 * bar_len
    ear_center_d = length - 0.006
    web_center_d = length - 0.015
    rib_x, rib_z = _along(dx, dz, 0.024)

    shape = (
        _y_cylinder(JOINT_R, EYE_T, center=(0.0, 0.0, 0.0))
        .union(_box_xz(eye_lug_len, EYE_T, BAR_H, center=(0.5 * eye_lug_len * ux, 0.0, 0.5 * eye_lug_len * uz), angle_deg=angle))
        .union(_box_xz(bar_len, PLATE_T, BAR_H, center=(ux * bar_center, 0.0, uz * bar_center), angle_deg=angle))
        .union(_box_xz(0.018, OUTER_COLLAR_T, 0.010, center=(rib_x, -0.0038, rib_z + 0.002), angle_deg=angle + 10.0))
        .union(_box_xz(0.010, OUTER_COLLAR_T, 0.005, center=(rib_x - 0.004, 0.0038, rib_z - 0.002), angle_deg=angle + 20.0))
        .union(_y_cylinder(JOINT_R, EAR_T, center=(dx, EAR_CENTER_Y, dz)))
        .union(_y_cylinder(JOINT_R, EAR_T, center=(dx, -EAR_CENTER_Y, dz)))
        .union(_y_cylinder(0.0070, INNER_COLLAR_T, center=(dx, 0.00375, dz)))
        .union(_y_cylinder(0.0070, INNER_COLLAR_T, center=(dx, -0.00375, dz)))
        .union(_box_xz(0.012, EAR_T, 0.008, center=(ux * ear_center_d, EAR_CENTER_Y, uz * ear_center_d), angle_deg=angle))
        .union(_box_xz(0.012, EAR_T, 0.008, center=(ux * ear_center_d, -EAR_CENTER_Y, uz * ear_center_d), angle_deg=angle))
        .union(_box_xz(0.018, 0.003, 0.007, center=(ux * web_center_d, 0.0070, uz * web_center_d), angle_deg=angle))
        .union(_box_xz(0.018, 0.003, 0.007, center=(ux * web_center_d, -0.0070, uz * web_center_d), angle_deg=angle))
    )

    return shape.cut(_y_cylinder(HOLE_R, EYE_T + 0.001, center=(0.0, 0.0, 0.0))).cut(
        _y_cylinder(HOLE_R, EAR_T + 0.001, center=(dx, EAR_CENTER_Y, dz))
    ).cut(_y_cylinder(HOLE_R, EAR_T + 0.001, center=(dx, -EAR_CENTER_Y, dz)))


def _make_link_three() -> cq.Workplane:
    dx, dz = LINK3_VEC
    angle = _bar_angle(dx, dz)
    ux, uz, length = _bar_dir(dx, dz)
    tab_length = 0.034
    tab_width = 0.016
    eye_lug_len = 0.014
    bar_len = length - eye_lug_len
    tab_center = (ux * (length - 0.5 * tab_length), 0.0, uz * (length - 0.5 * tab_length))
    slot_center = (ux * (length - 0.014), 0.0, uz * (length - 0.014))
    rib_x, rib_z = _along(dx, dz, 0.025)

    shape = (
        _y_cylinder(JOINT_R, EYE_T, center=(0.0, 0.0, 0.0))
        .union(_box_xz(eye_lug_len, EYE_T, BAR_H, center=(0.5 * eye_lug_len * ux, 0.0, 0.5 * eye_lug_len * uz), angle_deg=angle))
        .union(_box_xz(length - eye_lug_len, PLATE_T, BAR_H, center=(ux * (eye_lug_len + 0.5 * (length - eye_lug_len)), 0.0, uz * (eye_lug_len + 0.5 * (length - eye_lug_len))), angle_deg=angle))
        .union(_box_xz(tab_length, PLATE_T, tab_width, center=tab_center, angle_deg=angle))
        .union(_box_xz(0.020, OUTER_COLLAR_T, 0.010, center=(rib_x, 0.0038, rib_z - 0.003), angle_deg=angle - 14.0))
        .union(_box_xz(0.010, OUTER_COLLAR_T, 0.005, center=(rib_x - 0.006, -0.0038, rib_z - 0.002), angle_deg=angle - 24.0))
    )

    return shape.cut(_y_cylinder(HOLE_R, EYE_T + 0.001, center=(0.0, 0.0, 0.0))).cut(
        _slot_y(0.019, 0.0075, PLATE_T + 0.003, center=slot_center, angle_deg=angle)
    )


def _add_mesh_visual(part, shape: cq.Workplane, mesh_name: str, material: str, visual_name: str) -> None:
    part.visual(mesh_from_cadquery(shape, mesh_name), material=material, name=visual_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="over_center_lever_train")

    model.material("bracket_steel", rgba=(0.28, 0.31, 0.34, 1.0))
    model.material("lever_steel", rgba=(0.54, 0.57, 0.60, 1.0))
    model.material("dark_hardware", rgba=(0.16, 0.17, 0.18, 1.0))

    root_bracket = model.part("root_bracket")
    lever_1 = model.part("lever_1")
    lever_2 = model.part("lever_2")
    lever_3 = model.part("lever_3")

    _add_mesh_visual(root_bracket, _make_root_bracket(), "root_bracket", "bracket_steel", "bracket_shell")
    _add_mesh_visual(lever_1, _make_link_one(), "lever_1", "lever_steel", "lever_1_shell")
    _add_mesh_visual(lever_2, _make_link_two(), "lever_2", "lever_steel", "lever_2_shell")
    _add_mesh_visual(lever_3, _make_link_three(), "lever_3", "dark_hardware", "lever_3_shell")

    model.articulation(
        "root_to_lever_1",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=lever_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=-0.65, upper=0.22),
    )
    model.articulation(
        "lever_1_to_lever_2",
        ArticulationType.REVOLUTE,
        parent=lever_1,
        child=lever_2,
        origin=Origin(xyz=(LINK1_VEC[0], 0.0, LINK1_VEC[1])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.2, lower=-0.82, upper=0.32),
    )
    model.articulation(
        "lever_2_to_lever_3",
        ArticulationType.REVOLUTE,
        parent=lever_2,
        child=lever_3,
        origin=Origin(xyz=(LINK2_VEC[0], 0.0, LINK2_VEC[1])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.4, lower=-0.55, upper=0.58),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    lever_1 = object_model.get_part("lever_1")
    lever_2 = object_model.get_part("lever_2")
    lever_3 = object_model.get_part("lever_3")
    joint_1 = object_model.get_articulation("root_to_lever_1")
    joint_2 = object_model.get_articulation("lever_1_to_lever_2")
    joint_3 = object_model.get_articulation("lever_2_to_lever_3")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        root_bracket,
        lever_1,
        reason="root pivot uses simplified nested sleeve geometry at the coaxial hinge stack instead of a separate explicit pin",
    )
    ctx.allow_overlap(
        lever_1,
        lever_2,
        reason="middle pivot block is modeled as interleaved fabricated lugs with a shared hinge sleeve volume around the second revolute axis",
    )
    ctx.allow_overlap(
        lever_2,
        lever_3,
        reason="terminal pivot stack uses a simplified shared collar volume to keep the slotted end link mechanically grounded at the third revolute axis",
    )

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
        "serial revolute axes stay planar",
        joint_1.axis == (0.0, 1.0, 0.0)
        and joint_2.axis == (0.0, 1.0, 0.0)
        and joint_3.axis == (0.0, 1.0, 0.0),
        details=f"axes were {joint_1.axis}, {joint_2.axis}, {joint_3.axis}",
    )
    ctx.check(
        "three joint blocks articulate in order",
        joint_1.child == "lever_1" and joint_2.child == "lever_2" and joint_3.child == "lever_3",
        details="serial linkage chain was not rooted bracket -> lever_1 -> lever_2 -> lever_3",
    )

    ctx.expect_contact(root_bracket, lever_1, name="root bracket and first lever share the root pivot stack")
    ctx.expect_contact(lever_1, lever_2, name="first and second levers stay mechanically linked at joint two")
    ctx.expect_contact(lever_2, lever_3, name="second lever and slotted terminal lever share the third pivot")
    ctx.expect_gap(
        lever_2,
        root_bracket,
        axis="x",
        min_gap=0.035,
        name="second lever stays forward of the grounded bracket outside the root pivot stack",
    )
    ctx.expect_gap(
        lever_3,
        lever_1,
        axis="x",
        min_gap=0.030,
        name="terminal link stays forward of the first lever outside the shared pivot stack",
    )
    ctx.expect_gap(
        lever_3,
        root_bracket,
        axis="x",
        min_gap=0.085,
        name="terminal slotted tab stays forward of the grounded bracket in rest pose",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=True,
        name="nonadjacent members stay clear through sampled motion",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
