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

    body_color = model.material("body_plastic", rgba=(0.28, 0.33, 0.20, 1.0))
    lid_color = model.material("lid_plastic", rgba=(0.32, 0.37, 0.23, 1.0))
    hinge_color = model.material("hinge_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    tray_color = model.material("tray_insert", rgba=(0.78, 0.74, 0.62, 1.0))

    body_depth = 0.24
    body_width = 0.42
    body_height = 0.14
    wall = 0.004
    lid_depth = body_depth + 0.002
    lid_width = body_width + 0.004
    lid_shell_height = 0.032
    lid_top_thickness = 0.0035
    hinge_radius = 0.0045
    hinge_axis_x = -(body_depth / 2.0) - 0.003
    hinge_axis_z = body_height + hinge_radius

    body = model.part("body")
    body.visual(
        Box((body_depth, body_width, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=body_color,
        name="bottom_panel",
    )
    body.visual(
        Box((body_depth - 2.0 * wall, wall, body_height)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - wall / 2.0, body_height / 2.0)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((body_depth - 2.0 * wall, wall, body_height)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + wall / 2.0, body_height / 2.0)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((wall, body_width - 2.0 * wall, body_height)),
        origin=Origin(xyz=(body_depth / 2.0 - wall / 2.0, 0.0, body_height / 2.0)),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((wall, body_width - 2.0 * wall, body_height)),
        origin=Origin(xyz=(-body_depth / 2.0 + wall / 2.0, 0.0, body_height / 2.0)),
        material=body_color,
        name="rear_wall",
    )
    for name, y_center in (("hinge_barrel_left", 0.135), ("hinge_barrel_right", -0.135)):
        body.visual(
            Box((0.008, 0.052, 0.009)),
            origin=Origin(xyz=(hinge_axis_x + 0.001, y_center, body_height + 0.002)),
            material=body_color,
            name=f"{name}_leaf",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=0.052),
            origin=Origin(
                xyz=(hinge_axis_x, y_center, hinge_axis_z),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_color,
            name=name,
        )
    body.inertial = Inertial.from_geometry(
        Box((body_depth, body_width, body_height)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_depth - 0.008, lid_width, lid_top_thickness)),
        origin=Origin(xyz=((lid_depth - 0.008) / 2.0 + 0.004, 0.0, 0.017)),
        material=lid_color,
        name="top_panel",
    )
    lid.visual(
        Box((lid_depth - 0.010, wall, lid_shell_height)),
        origin=Origin(
            xyz=((lid_depth - 0.010) / 2.0 + 0.005, lid_width / 2.0 - wall / 2.0, 0.001),
        ),
        material=lid_color,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_depth - 0.010, wall, lid_shell_height)),
        origin=Origin(
            xyz=((lid_depth - 0.010) / 2.0 + 0.005, -lid_width / 2.0 + wall / 2.0, 0.001),
        ),
        material=lid_color,
        name="right_skirt",
    )
    lid.visual(
        Box((wall, lid_width, lid_shell_height)),
        origin=Origin(xyz=(lid_depth - wall / 2.0, 0.0, 0.001)),
        material=lid_color,
        name="front_skirt",
    )
    lid.visual(
        Box((0.010, 0.110, 0.020)),
        origin=Origin(xyz=(0.005, 0.0, 0.010)),
        material=lid_color,
        name="rear_leaf",
    )
    lid.visual(
        Cylinder(radius=hinge_radius * 0.92, length=0.114),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hinge_color,
        name="hinge_barrel_center",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_depth, lid_width, lid_shell_height)),
        mass=0.65,
        origin=Origin(xyz=(lid_depth / 2.0, 0.0, 0.008)),
    )

    tray = model.part("tray_insert")
    tray_floor_z = 0.074
    tray_depth = 0.186
    tray_width = 0.348
    tray_floor_thickness = 0.003
    tray_wall_height = 0.022
    tray.visual(
        Box((tray_depth, tray_width, tray_floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, tray_floor_z)),
        material=tray_color,
        name="tray_floor",
    )
    tray.visual(
        Box((0.003, tray_width, tray_wall_height)),
        origin=Origin(
            xyz=(
                tray_depth / 2.0 - 0.0015,
                0.0,
                tray_floor_z + tray_floor_thickness / 2.0 + tray_wall_height / 2.0,
            ),
        ),
        material=tray_color,
        name="tray_front_wall",
    )
    tray.visual(
        Box((0.003, tray_width, tray_wall_height)),
        origin=Origin(
            xyz=(
                -tray_depth / 2.0 + 0.0015,
                0.0,
                tray_floor_z + tray_floor_thickness / 2.0 + tray_wall_height / 2.0,
            ),
        ),
        material=tray_color,
        name="tray_rear_wall",
    )
    tray.visual(
        Box((tray_depth - 0.006, 0.003, tray_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                tray_width / 2.0 - 0.0015,
                tray_floor_z + tray_floor_thickness / 2.0 + tray_wall_height / 2.0,
            ),
        ),
        material=tray_color,
        name="tray_left_wall",
    )
    tray.visual(
        Box((tray_depth - 0.006, 0.003, tray_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                -tray_width / 2.0 + 0.0015,
                tray_floor_z + tray_floor_thickness / 2.0 + tray_wall_height / 2.0,
            ),
        ),
        material=tray_color,
        name="tray_right_wall",
    )
    tray.visual(
        Box((tray_depth - 0.006, 0.003, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, tray_floor_z + tray_floor_thickness / 2.0 + 0.009)),
        material=tray_color,
        name="center_divider",
    )
    tray.visual(
        Box((0.003, tray_width / 2.0 - 0.003, 0.016)),
        origin=Origin(xyz=(-0.028, tray_width / 4.0, tray_floor_z + tray_floor_thickness / 2.0 + 0.008)),
        material=tray_color,
        name="left_cross_divider",
    )
    tray.visual(
        Box((0.003, tray_width / 2.0 - 0.003, 0.016)),
        origin=Origin(xyz=(0.038, -tray_width / 4.0, tray_floor_z + tray_floor_thickness / 2.0 + 0.008)),
        material=tray_color,
        name="right_cross_divider",
    )
    post_height = tray_floor_z - tray_floor_thickness / 2.0 - wall
    post_z = wall + post_height / 2.0
    for name, x_pos, y_pos in (
        ("post_front_left", 0.064, 0.128),
        ("post_front_right", 0.064, -0.128),
        ("post_rear_left", -0.064, 0.128),
        ("post_rear_right", -0.064, -0.128),
    ):
        tray.visual(
            Box((0.010, 0.010, post_height)),
            origin=Origin(xyz=(x_pos, y_pos, post_z)),
            material=tray_color,
            name=name,
        )
    tray.inertial = Inertial.from_geometry(
        Box((tray_depth, tray_width, 0.10)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=1.90,
        ),
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
    hinge = object_model.get_articulation("body_to_lid")
    open_angle = 1.75
    if hinge.motion_limits is not None and hinge.motion_limits.upper is not None:
        open_angle = hinge.motion_limits.upper

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.20,
            name="closed lid covers the box opening",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="top_panel",
            negative_elem="front_wall",
            min_gap=0.012,
            max_gap=0.030,
            name="closed lid stays close to the body rim",
        )
        ctx.expect_within(
            tray,
            body,
            axes="xy",
            margin=0.0,
            name="tray insert stays inside the box walls",
        )
        ctx.expect_gap(
            lid,
            tray,
            axis="z",
            positive_elem="top_panel",
            negative_elem="center_divider",
            min_gap=0.050,
            name="closed lid clears the fixed tray dividers",
        )

    rest_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({hinge: open_angle}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="hinge_barrel_center",
            negative_elem="rear_wall",
            min_gap=0.0002,
            name="opened lid hinge barrel clears the rear wall",
        )
        open_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    rest_front_center_z = None if rest_front is None else 0.5 * (rest_front[0][2] + rest_front[1][2])
    open_front_center_z = None if open_front is None else 0.5 * (open_front[0][2] + open_front[1][2])
    ctx.check(
        "lid front edge lifts upward when opened",
        rest_front_center_z is not None
        and open_front_center_z is not None
        and open_front_center_z > rest_front_center_z + 0.11,
        details=f"rest_z={rest_front_center_z}, open_z={open_front_center_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
