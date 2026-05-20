from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.158
LID_RADIUS = 0.147
HINGE_Y = -0.166
HINGE_Z = 0.066
LID_CENTER_Y = 0.166
PLATE_RADIUS = 0.124
BUTTON_Y = 0.115
BUTTON_XS = (0.118, 0.145)
BUTTON_POCKET_FLOOR_Z = 0.062


def _revolve_profile(points: list[tuple[float, float]]) -> cq.Workplane:
    profile = cq.Workplane("XZ").moveTo(*points[0])
    for radius, height in points[1:]:
        profile = profile.lineTo(radius, height)
    return profile.close().revolve(360.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))


def _x_cylinder(
    radius: float,
    length: float,
    center_xyz: tuple[float, float, float],
) -> cq.Workplane:
    cx, cy, cz = center_xyz
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((cx - (length * 0.5), cy, cz))
    )


def _cut_crosshatch(
    solid: cq.Workplane,
    *,
    center_xy: tuple[float, float],
    plane_z: float,
    span: float,
    pitch: float,
    groove_width: float,
    groove_depth: float,
) -> cq.Workplane:
    cx, cy = center_xy
    offsets: list[float] = []
    offset = -span * 0.55
    while offset <= span * 0.55 + 1e-9:
        offsets.append(offset)
        offset += pitch

    cutter_height = groove_depth * 2.4
    for angle_deg in (45.0, -45.0):
        for offset in offsets:
            cutter = (
                cq.Workplane("XY")
                .box(span * 2.8, groove_width, cutter_height)
                .translate((0.0, offset, plane_z + (groove_depth * 0.4)))
                .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
                .translate((cx, cy, 0.0))
            )
            solid = solid.cut(cutter)

    for angle_deg in (0.0, 90.0):
        channel = (
            cq.Workplane("XY")
            .box(span * 2.2, groove_width * 1.35, cutter_height * 1.1)
            .translate((0.0, 0.0, plane_z + (groove_depth * 0.45)))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
            .translate((cx, cy, 0.0))
        )
        solid = solid.cut(channel)

    return solid


def _build_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(0.050)
    base = base.union(
        cq.Workplane("XY").circle(0.150).extrude(0.006).translate((0.0, 0.0, 0.050))
    )

    cooking_recess = (
        cq.Workplane("XY")
        .circle(PLATE_RADIUS)
        .extrude(0.013)
        .translate((0.0, 0.0, 0.043))
    )
    base = base.cut(cooking_recess)
    base = _cut_crosshatch(
        base,
        center_xy=(0.0, 0.0),
        plane_z=0.043,
        span=PLATE_RADIUS,
        pitch=0.030,
        groove_width=0.008,
        groove_depth=0.0026,
    )

    control_pod = (
        cq.Workplane("XY")
        .box(0.074, 0.044, 0.012)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.132, 0.116, 0.060))
    )
    base = base.union(control_pod)

    for button_x in BUTTON_XS:
        button_pocket = (
            cq.Workplane("XY")
            .box(0.015, 0.013, 0.004)
            .translate((button_x, BUTTON_Y, 0.064))
        )
        base = base.cut(button_pocket)

    return base


def _build_lid_shell_shape() -> cq.Workplane:
    rim = (
        cq.Workplane("XY")
        .circle(LID_RADIUS)
        .extrude(0.018)
        .translate((0.0, LID_CENTER_Y, -0.002))
    )
    dome_cap = (
        cq.Workplane("XY")
        .sphere(0.190)
        .translate((0.0, LID_CENTER_Y, -0.110))
        .intersect(
            cq.Workplane("XY")
            .circle(LID_RADIUS)
            .extrude(0.075)
            .translate((0.0, LID_CENTER_Y, 0.010))
        )
    )
    lid = rim.union(dome_cap)

    upper_recess = (
        cq.Workplane("XY")
        .circle(PLATE_RADIUS)
        .extrude(0.011)
        .translate((0.0, LID_CENTER_Y, -0.002))
    )
    lid = lid.cut(upper_recess)
    lid = _cut_crosshatch(
        lid,
        center_xy=(0.0, LID_CENTER_Y),
        plane_z=0.009,
        span=PLATE_RADIUS,
        pitch=0.030,
        groove_width=0.008,
        groove_depth=0.0024,
    )

    return lid


def _build_base_hinge_shape() -> cq.Workplane:
    hinge = (
        cq.Workplane("XY")
        .box(0.136, 0.020, 0.016)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, HINGE_Y, 0.034))
    )
    for x_pos in (-0.050, 0.050):
        riser = (
            cq.Workplane("XY")
            .box(0.028, 0.024, 0.026)
            .edges("|Z")
            .fillet(0.003)
            .translate((x_pos, HINGE_Y, 0.047))
        )
        hinge = hinge.union(riser)
    hinge = hinge.union(_x_cylinder(0.009, 0.036, (-0.050, HINGE_Y, HINGE_Z)))
    hinge = hinge.union(_x_cylinder(0.009, 0.036, (0.050, HINGE_Y, HINGE_Z)))
    return hinge


def _build_lid_hinge_shape() -> cq.Workplane:
    hinge = (
        cq.Workplane("XY")
        .box(0.046, 0.024, 0.012)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.012, 0.006))
    )
    hinge = hinge.union(_x_cylinder(0.007, 0.050, (0.0, 0.000, 0.000)))
    return hinge


def _build_handle_shape() -> cq.Workplane:
    handle = (
        cq.Workplane("XY")
        .box(0.090, 0.024, 0.014)
        .translate((0.0, 0.315, 0.011))
    )
    finger_pull = (
        cq.Workplane("XY")
        .box(0.060, 0.016, 0.008)
        .translate((0.0, 0.324, 0.006))
    )
    handle = handle.union(finger_pull)
    return handle.edges("|Z").fillet(0.0035)


def _build_button_shape() -> cq.Workplane:
    skirt = cq.Workplane("XY").box(0.015, 0.013, 0.004).translate((0.0, 0.0, 0.004))
    cap = cq.Workplane("XY").box(0.017, 0.015, 0.004).translate((0.0, 0.0, 0.008))
    return skirt.union(cap).edges("|Z").fillet(0.0018)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="belgian_waffle_maker")

    base_black = model.material("base_black", rgba=(0.12, 0.12, 0.13, 1.0))
    lid_silver = model.material("lid_silver", rgba=(0.73, 0.74, 0.76, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))
    button_gray = model.material("button_gray", rgba=(0.30, 0.31, 0.33, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "waffle_maker_base"),
        material=base_black,
        name="base_housing",
    )
    base.visual(
        mesh_from_cadquery(_build_base_hinge_shape(), "waffle_maker_base_hinge"),
        material=base_black,
        name="rear_hinge",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell_shape(), "waffle_maker_lid"),
        material=lid_silver,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_build_lid_hinge_shape(), "waffle_maker_lid_hinge"),
        material=handle_black,
        name="rear_hinge",
    )
    lid.visual(
        mesh_from_cadquery(_build_handle_shape(), "waffle_maker_handle"),
        material=handle_black,
        name="front_handle",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    button_mesh = mesh_from_cadquery(_build_button_shape(), "waffle_maker_program_button")
    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"program_button_{index}")
        button.visual(
            button_mesh,
            material=button_gray,
            name="button",
        )
        model.articulation(
            f"base_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(button_x, BUTTON_Y, BUTTON_POCKET_FLOOR_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.04,
                lower=0.0,
                upper=0.0015,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("base_to_lid")
    button_0 = object_model.get_part("program_button_0")
    button_1 = object_model.get_part("program_button_1")
    button_joint_0 = object_model.get_articulation("base_to_program_button_0")
    button_joint_1 = object_model.get_articulation("base_to_program_button_1")

    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="lid_shell",
        elem_b="base_housing",
        min_overlap=0.26,
        name="lid stays centered over the lower housing",
    )
    lid_shell_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "closed lid keeps a tight clamshell seam",
        lid_shell_aabb is not None and 0.057 <= lid_shell_aabb[0][2] <= 0.064,
        details=f"lid_shell_aabb={lid_shell_aabb}",
    )

    closed_aabb = ctx.part_world_aabb(lid)
    opened_aabb = None
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
            opened_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward on the rear hinge",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.11
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.03,
        details=f"closed_aabb={closed_aabb}, opened_aabb={opened_aabb}",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    pressed_button_0 = None
    pressed_button_1 = None
    stationary_button_1 = None
    stationary_button_0 = None

    if button_joint_0.motion_limits is not None and button_joint_0.motion_limits.upper is not None:
        with ctx.pose({button_joint_0: button_joint_0.motion_limits.upper}):
            pressed_button_0 = ctx.part_world_position(button_0)
            stationary_button_1 = ctx.part_world_position(button_1)
    if button_joint_1.motion_limits is not None and button_joint_1.motion_limits.upper is not None:
        with ctx.pose({button_joint_1: button_joint_1.motion_limits.upper}):
            pressed_button_1 = ctx.part_world_position(button_1)
            stationary_button_0 = ctx.part_world_position(button_0)

    ctx.check(
        "program_button_0 presses downward",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and pressed_button_0[2] < rest_button_0[2] - 0.001,
        details=f"rest={rest_button_0}, pressed={pressed_button_0}",
    )
    ctx.check(
        "program_button_1 presses downward",
        rest_button_1 is not None
        and pressed_button_1 is not None
        and pressed_button_1[2] < rest_button_1[2] - 0.001,
        details=f"rest={rest_button_1}, pressed={pressed_button_1}",
    )
    ctx.check(
        "program buttons articulate independently",
        rest_button_0 is not None
        and rest_button_1 is not None
        and stationary_button_0 is not None
        and stationary_button_1 is not None
        and abs(stationary_button_0[2] - rest_button_0[2]) < 1e-6
        and abs(stationary_button_1[2] - rest_button_1[2]) < 1e-6,
        details=(
            f"rest_button_0={rest_button_0}, rest_button_1={rest_button_1}, "
            f"stationary_button_0={stationary_button_0}, stationary_button_1={stationary_button_1}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
