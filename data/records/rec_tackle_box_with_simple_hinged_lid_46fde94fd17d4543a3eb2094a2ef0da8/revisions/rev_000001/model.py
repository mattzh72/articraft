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

    body_plastic = model.material("body_plastic", rgba=(0.16, 0.34, 0.17, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.19, 0.39, 0.21, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.63, 0.66, 0.69, 1.0))
    tray_plastic = model.material("tray_plastic", rgba=(0.76, 0.77, 0.71, 1.0))

    body_width = 0.42
    body_depth = 0.23
    body_height = 0.17
    wall = 0.008
    floor_thickness = 0.008
    hinge_axis_y = -(body_depth * 0.5) - 0.009
    hinge_axis_z = body_height

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=body_plastic,
        name="floor",
    )
    body.visual(
        Box((body_width, wall, body_height - floor_thickness)),
        origin=Origin(
            xyz=(0.0, (body_depth * 0.5) - (wall * 0.5), floor_thickness + (body_height - floor_thickness) * 0.5)
        ),
        material=body_plastic,
        name="front_wall",
    )
    body.visual(
        Box((body_width, wall, body_height - floor_thickness)),
        origin=Origin(
            xyz=(0.0, -(body_depth * 0.5) + (wall * 0.5), floor_thickness + (body_height - floor_thickness) * 0.5)
        ),
        material=body_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((wall, body_depth - (2.0 * wall), body_height - floor_thickness)),
        origin=Origin(
            xyz=((body_width * 0.5) - (wall * 0.5), 0.0, floor_thickness + (body_height - floor_thickness) * 0.5)
        ),
        material=body_plastic,
        name="right_wall",
    )
    body.visual(
        Box((wall, body_depth - (2.0 * wall), body_height - floor_thickness)),
        origin=Origin(
            xyz=(-(body_width * 0.5) + (wall * 0.5), 0.0, floor_thickness + (body_height - floor_thickness) * 0.5)
        ),
        material=body_plastic,
        name="left_wall",
    )

    tray_z = 0.102
    tray_depth = 0.152
    tray_width = body_width - (2.0 * wall) + 0.002
    tray_wall_height = 0.022
    body.visual(
        Box((tray_width, tray_depth, 0.004)),
        origin=Origin(xyz=(0.0, 0.002, tray_z)),
        material=tray_plastic,
        name="tray_floor",
    )
    body.visual(
        Box((tray_width, 0.004, tray_wall_height)),
        origin=Origin(xyz=(0.0, 0.076, tray_z + (tray_wall_height * 0.5))),
        material=tray_plastic,
        name="tray_front_wall",
    )
    body.visual(
        Box((0.004, tray_depth, tray_wall_height)),
        origin=Origin(xyz=(-0.068, 0.002, tray_z + (tray_wall_height * 0.5))),
        material=tray_plastic,
        name="tray_divider_left",
    )
    body.visual(
        Box((0.004, tray_depth, tray_wall_height)),
        origin=Origin(xyz=(0.068, 0.002, tray_z + (tray_wall_height * 0.5))),
        material=tray_plastic,
        name="tray_divider_right",
    )
    body.visual(
        Box((body_width - (2.0 * wall), 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.087, 0.030)),
        material=tray_plastic,
        name="front_bin_rib",
    )

    knuckle_radius = 0.009
    body.visual(
        Cylinder(radius=knuckle_radius, length=0.075),
        origin=Origin(xyz=(-0.1675, hinge_axis_y, hinge_axis_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=hinge_metal,
        name="left_body_knuckle",
    )
    body.visual(
        Cylinder(radius=knuckle_radius, length=0.075),
        origin=Origin(xyz=(0.1675, hinge_axis_y, hinge_axis_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=hinge_metal,
        name="right_body_knuckle",
    )
    body.visual(
        Box((0.075, 0.020, 0.014)),
        origin=Origin(xyz=(-0.1675, hinge_axis_y + 0.006, hinge_axis_z - 0.007)),
        material=body_plastic,
        name="left_hinge_bridge",
    )
    body.visual(
        Box((0.075, 0.020, 0.014)),
        origin=Origin(xyz=(0.1675, hinge_axis_y + 0.006, hinge_axis_z - 0.007)),
        material=body_plastic,
        name="right_hinge_bridge",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((body_width, 0.228, 0.005)),
        origin=Origin(xyz=(0.0, 0.124, 0.0025)),
        material=lid_plastic,
        name="top_panel",
    )
    lid.visual(
        Box((body_width, 0.006, 0.026)),
        origin=Origin(xyz=(0.0, 0.241, -0.013)),
        material=lid_plastic,
        name="front_skirt",
    )
    lid.visual(
        Box((0.006, 0.228, 0.024)),
        origin=Origin(xyz=(0.213, 0.124, -0.012)),
        material=lid_plastic,
        name="right_skirt",
    )
    lid.visual(
        Box((0.006, 0.228, 0.024)),
        origin=Origin(xyz=(-0.213, 0.124, -0.012)),
        material=lid_plastic,
        name="left_skirt",
    )
    lid.visual(
        Box((0.048, 0.022, 0.004)),
        origin=Origin(xyz=(-0.075, 0.008, 0.004)),
        material=lid_plastic,
        name="left_hinge_strap",
    )
    lid.visual(
        Box((0.048, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, 0.008, 0.004)),
        material=lid_plastic,
        name="center_hinge_strap",
    )
    lid.visual(
        Box((0.048, 0.022, 0.004)),
        origin=Origin(xyz=(0.075, 0.008, 0.004)),
        material=lid_plastic,
        name="right_hinge_strap",
    )
    lid.visual(
        Cylinder(radius=0.0075, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=hinge_metal,
        name="lid_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, 0.030)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.120, -0.004)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="top_panel",
        min_overlap=0.20,
        name="closed lid covers the box opening",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="top_panel",
        negative_elem="front_wall",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid sits on the body rim height",
    )
    ctx.expect_gap(
        body,
        lid,
        axis="y",
        positive_elem="rear_wall",
        negative_elem="lid_knuckle",
        min_gap=0.001,
        max_gap=0.003,
        name="hinge barrel sits just outside the rear wall",
    )

    closed_top = ctx.part_element_world_aabb(lid, elem="top_panel")
    with ctx.pose({hinge: 1.4}):
        opened_top = ctx.part_element_world_aabb(lid, elem="top_panel")
    ctx.check(
        "lid opens upward about the rear hinge",
        closed_top is not None
        and opened_top is not None
        and opened_top[1][2] > closed_top[1][2] + 0.18
        and opened_top[1][1] < closed_top[1][1] - 0.05,
        details=f"closed_top={closed_top}, opened_top={opened_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
