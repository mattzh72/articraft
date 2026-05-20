from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tackle_box")

    shell_green = model.material("shell_green", rgba=(0.30, 0.37, 0.22, 1.0))
    shell_green_dark = model.material("shell_green_dark", rgba=(0.21, 0.25, 0.15, 1.0))
    tray_tan = model.material("tray_tan", rgba=(0.76, 0.70, 0.58, 1.0))
    tray_tan_dark = model.material("tray_tan_dark", rgba=(0.63, 0.57, 0.46, 1.0))

    body_length = 0.44
    body_depth = 0.24
    bottom_thickness = 0.008
    wall_thickness = 0.006
    wall_height = 0.167
    body_top_z = bottom_thickness + wall_height

    hinge_radius = 0.0065
    hinge_y = -body_depth * 0.5 - hinge_radius * 0.35
    hinge_z = body_top_z - 0.002

    lid_length = body_length + 0.012
    lid_depth = body_depth + 0.010
    lid_wall_thickness = 0.005
    lid_panel_thickness = 0.004
    lid_skirt_height = 0.036

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_depth, body_top_z)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, body_top_z * 0.5)),
    )
    body.visual(
        Box((body_length, body_depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness * 0.5)),
        material=shell_green_dark,
        name="floor",
    )
    body.visual(
        Box((body_length, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, body_depth * 0.5 - wall_thickness * 0.5, bottom_thickness + wall_height * 0.5)),
        material=shell_green,
        name="front_wall",
    )
    body.visual(
        Box((body_length, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, -body_depth * 0.5 + wall_thickness * 0.5, bottom_thickness + wall_height * 0.5)),
        material=shell_green,
        name="rear_wall",
    )
    body.visual(
        Box((wall_thickness, body_depth - 2.0 * wall_thickness, wall_height)),
        origin=Origin(
            xyz=(body_length * 0.5 - wall_thickness * 0.5, 0.0, bottom_thickness + wall_height * 0.5)
        ),
        material=shell_green,
        name="right_wall",
    )
    body.visual(
        Box((wall_thickness, body_depth - 2.0 * wall_thickness, wall_height)),
        origin=Origin(
            xyz=(-body_length * 0.5 + wall_thickness * 0.5, 0.0, bottom_thickness + wall_height * 0.5)
        ),
        material=shell_green,
        name="left_wall",
    )
    body.visual(
        Box((body_length - 0.036, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, body_depth * 0.5 - wall_thickness - 0.007, body_top_z - 0.004)),
        material=shell_green_dark,
        name="front_seat",
    )
    body.visual(
        Box((0.014, body_depth - 0.038, 0.008)),
        origin=Origin(xyz=(body_length * 0.5 - wall_thickness - 0.007, 0.0, body_top_z - 0.004)),
        material=shell_green_dark,
        name="right_seat",
    )
    body.visual(
        Box((0.014, body_depth - 0.038, 0.008)),
        origin=Origin(xyz=(-body_length * 0.5 + wall_thickness + 0.007, 0.0, body_top_z - 0.004)),
        material=shell_green_dark,
        name="left_seat",
    )
    body.visual(
        Box((0.090, body_depth * 0.72, 0.008)),
        origin=Origin(xyz=(0.130, 0.0, 0.004)),
        material=shell_green_dark,
        name="right_runner",
    )
    body.visual(
        Box((0.090, body_depth * 0.72, 0.008)),
        origin=Origin(xyz=(-0.130, 0.0, 0.004)),
        material=shell_green_dark,
        name="left_runner",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.082),
        origin=Origin(xyz=(-0.136, hinge_y, hinge_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=shell_green_dark,
        name="left_hinge_knuckle",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.082),
        origin=Origin(xyz=(0.136, hinge_y, hinge_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=shell_green_dark,
        name="right_hinge_knuckle",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_depth, lid_skirt_height)),
        mass=0.7,
        origin=Origin(xyz=(0.0, lid_depth * 0.5, lid_skirt_height * 0.5)),
    )
    lid.visual(
        Box((lid_length, lid_depth, lid_panel_thickness)),
        origin=Origin(xyz=(0.0, lid_depth * 0.5, lid_skirt_height - lid_panel_thickness * 0.5)),
        material=shell_green,
        name="top_panel",
    )
    lid.visual(
        Box((lid_length, lid_wall_thickness, lid_skirt_height)),
        origin=Origin(xyz=(0.0, lid_depth - lid_wall_thickness * 0.5, lid_skirt_height * 0.5)),
        material=shell_green_dark,
        name="front_skirt",
    )
    lid.visual(
        Box((lid_wall_thickness, lid_depth, lid_skirt_height)),
        origin=Origin(xyz=(-lid_length * 0.5 + lid_wall_thickness * 0.5, lid_depth * 0.5, lid_skirt_height * 0.5)),
        material=shell_green_dark,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_wall_thickness, lid_depth, lid_skirt_height)),
        origin=Origin(xyz=(lid_length * 0.5 - lid_wall_thickness * 0.5, lid_depth * 0.5, lid_skirt_height * 0.5)),
        material=shell_green_dark,
        name="right_skirt",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.168),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=shell_green_dark,
        name="center_hinge_knuckle",
    )
    lid.visual(
        Box((0.030, 0.010, lid_skirt_height - 0.002)),
        origin=Origin(xyz=(-0.058, 0.005, (lid_skirt_height - 0.002) * 0.5)),
        material=shell_green_dark,
        name="left_hinge_bridge",
    )
    lid.visual(
        Box((0.030, 0.010, lid_skirt_height - 0.002)),
        origin=Origin(xyz=(0.058, 0.005, (lid_skirt_height - 0.002) * 0.5)),
        material=shell_green_dark,
        name="right_hinge_bridge",
    )

    tray = model.part("tray_insert")
    tray_floor_z = 0.104
    tray_floor_thickness = 0.004
    tray_wall_height = 0.028
    tray_length = body_length - 0.044
    tray_depth = body_depth - 0.076
    support_height = tray_floor_z - tray_floor_thickness * 0.5 - bottom_thickness

    tray.inertial = Inertial.from_geometry(
        Box((tray_length, tray_depth, tray_floor_z + tray_wall_height)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, (tray_floor_z + tray_wall_height) * 0.5)),
    )
    tray.visual(
        Box((tray_length, tray_depth, tray_floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, tray_floor_z)),
        material=tray_tan,
        name="tray_floor",
    )
    tray.visual(
        Box((tray_length, 0.004, tray_wall_height)),
        origin=Origin(xyz=(0.0, tray_depth * 0.5 - 0.002, tray_floor_z + tray_wall_height * 0.5)),
        material=tray_tan_dark,
        name="front_tray_wall",
    )
    tray.visual(
        Box((tray_length, 0.004, tray_wall_height)),
        origin=Origin(xyz=(0.0, -tray_depth * 0.5 + 0.002, tray_floor_z + tray_wall_height * 0.5)),
        material=tray_tan_dark,
        name="rear_tray_wall",
    )
    tray.visual(
        Box((0.004, tray_depth - 0.008, tray_wall_height)),
        origin=Origin(xyz=(-tray_length * 0.5 + 0.002, 0.0, tray_floor_z + tray_wall_height * 0.5)),
        material=tray_tan_dark,
        name="left_tray_wall",
    )
    tray.visual(
        Box((0.004, tray_depth - 0.008, tray_wall_height)),
        origin=Origin(xyz=(tray_length * 0.5 - 0.002, 0.0, tray_floor_z + tray_wall_height * 0.5)),
        material=tray_tan_dark,
        name="right_tray_wall",
    )
    tray.visual(
        Box((0.004, tray_depth - 0.014, tray_wall_height - 0.004)),
        origin=Origin(xyz=(-0.072, 0.0, tray_floor_z + (tray_wall_height - 0.004) * 0.5)),
        material=tray_tan_dark,
        name="left_divider",
    )
    tray.visual(
        Box((0.004, tray_depth - 0.014, tray_wall_height - 0.004)),
        origin=Origin(xyz=(0.072, 0.0, tray_floor_z + (tray_wall_height - 0.004) * 0.5)),
        material=tray_tan_dark,
        name="right_divider",
    )
    tray.visual(
        Box((0.018, 0.018, support_height)),
        origin=Origin(xyz=(-0.150, -0.048, bottom_thickness + support_height * 0.5)),
        material=tray_tan_dark,
        name="rear_left_support",
    )
    tray.visual(
        Box((0.018, 0.018, support_height)),
        origin=Origin(xyz=(0.150, -0.048, bottom_thickness + support_height * 0.5)),
        material=tray_tan_dark,
        name="rear_right_support",
    )
    tray.visual(
        Box((0.018, 0.018, support_height)),
        origin=Origin(xyz=(-0.150, 0.048, bottom_thickness + support_height * 0.5)),
        material=tray_tan_dark,
        name="front_left_support",
    )
    tray.visual(
        Box((0.018, 0.018, support_height)),
        origin=Origin(xyz=(0.150, 0.048, bottom_thickness + support_height * 0.5)),
        material=tray_tan_dark,
        name="front_right_support",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.95),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.FIXED,
        parent=body,
        child=tray,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray_insert")
    lid_hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="top_panel",
        min_overlap=0.22,
        name="lid panel covers the box opening",
    )

    rest_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({lid_hinge: 1.2}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    ctx.check(
        "lid front edge lifts upward when opened",
        rest_front is not None
        and opened_front is not None
        and opened_front[0][2] > rest_front[1][2] + 0.08,
        details=f"rest_front={rest_front}, opened_front={opened_front}",
    )
    ctx.expect_contact(
        tray,
        body,
        elem_a="front_left_support",
        elem_b="floor",
        name="fixed tray insert is supported by the body floor",
    )
    ctx.expect_gap(
        lid,
        tray,
        axis="z",
        positive_elem="top_panel",
        negative_elem="left_divider",
        min_gap=0.05,
        name="closed lid clears the organizer dividers",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
