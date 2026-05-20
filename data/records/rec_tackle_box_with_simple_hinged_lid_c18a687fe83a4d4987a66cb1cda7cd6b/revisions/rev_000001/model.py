from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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

    body_color = model.material("body_color", rgba=(0.22, 0.31, 0.16, 1.0))
    lid_color = model.material("lid_color", rgba=(0.25, 0.36, 0.18, 1.0))
    hinge_color = model.material("hinge_color", rgba=(0.18, 0.18, 0.19, 1.0))
    tray_color = model.material("tray_color", rgba=(0.45, 0.47, 0.43, 1.0))

    outer_length = 0.44
    outer_width = 0.24
    body_height = 0.18
    wall_thickness = 0.014
    bottom_thickness = 0.014

    lid_depth = 0.028
    lid_frame_width = 0.032
    lid_center_thickness = 0.020

    hinge_radius = 0.008
    hinge_axis_z = body_height + 0.011
    hinge_x = -outer_length / 2.0

    inner_length = outer_length - 2.0 * wall_thickness
    inner_width = outer_width - 2.0 * wall_thickness

    body = model.part("body")
    body.visual(
        Box((outer_length, outer_width, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness / 2.0)),
        material=body_color,
        name="bottom_panel",
    )
    body.visual(
        Box((wall_thickness, outer_width, body_height)),
        origin=Origin(
            xyz=(outer_length / 2.0 - wall_thickness / 2.0, 0.0, body_height / 2.0),
        ),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((wall_thickness, outer_width, body_height)),
        origin=Origin(
            xyz=(-outer_length / 2.0 + wall_thickness / 2.0, 0.0, body_height / 2.0),
        ),
        material=body_color,
        name="rear_wall",
    )
    side_wall_length = outer_length - 2.0 * wall_thickness
    for side, y_sign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((side_wall_length, wall_thickness, body_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    y_sign * (outer_width / 2.0 - wall_thickness / 2.0),
                    body_height / 2.0,
                )
            ),
            material=body_color,
            name=f"{side}_wall",
        )

    for index, y_pos in enumerate((-0.075, 0.0, 0.075)):
        body.visual(
            Box((0.014, 0.026, 0.024)),
            origin=Origin(xyz=(hinge_x - 0.007, y_pos, body_height + 0.002)),
            material=body_color,
            name=f"rear_hinge_boss_{index}",
        )

    tray_rail_length = 0.30
    tray_rail_thickness = 0.010
    tray_rail_z = 0.109
    tray_floor_thickness = 0.006
    tray_floor_length = 0.30
    tray_floor_width = inner_width - 0.010
    tray_floor_center_x = 0.020
    tray_floor_center_z = tray_rail_z + 0.007

    for side, y_sign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((tray_rail_length, tray_rail_thickness, tray_rail_thickness)),
            origin=Origin(
                xyz=(
                    tray_floor_center_x,
                    y_sign * (inner_width / 2.0 - tray_rail_thickness / 2.0),
                    tray_rail_z,
                )
            ),
            material=tray_color,
            name=f"tray_support_{side}",
        )

    body.visual(
        Box((tray_floor_length, tray_floor_width, tray_floor_thickness)),
        origin=Origin(
            xyz=(tray_floor_center_x, 0.0, tray_floor_center_z),
        ),
        material=tray_color,
        name="tray_floor",
    )
    body.visual(
        Box((0.24, 0.010, tray_floor_center_z - bottom_thickness / 2.0)),
        origin=Origin(
            xyz=(
                tray_floor_center_x,
                0.0,
                (tray_floor_center_z + bottom_thickness / 2.0) / 2.0,
            ),
        ),
        material=tray_color,
        name="lower_center_divider",
    )
    for index, x_pos in enumerate((-0.075, 0.115)):
        body.visual(
            Box((0.010, tray_floor_width, 0.030)),
            origin=Origin(xyz=(x_pos, 0.0, tray_floor_center_z + 0.015)),
            material=tray_color,
            name=f"tray_cross_divider_{index}",
        )
    body.visual(
        Box((tray_floor_length, 0.008, 0.030)),
        origin=Origin(
            xyz=(tray_floor_center_x, 0.0, tray_floor_center_z + 0.015),
        ),
        material=tray_color,
        name="tray_long_divider",
    )

    barrel_positions = (-0.090, 0.0, 0.090)
    barrel_lengths = (0.060, 0.016, 0.060)
    for index, (y_pos, barrel_length) in enumerate(zip(barrel_positions, barrel_lengths)):
        body.visual(
            Cylinder(radius=hinge_radius, length=barrel_length),
            origin=Origin(
                xyz=(hinge_x, y_pos, hinge_axis_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_color,
            name=f"body_hinge_knuckle_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((outer_length, outer_width, body_height)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    lid = model.part("lid")
    for side, y_pos in (("left", -0.032), ("right", 0.032)):
        lid.visual(
            Box((0.020, 0.038, 0.020)),
            origin=Origin(xyz=(0.010, y_pos, -0.002)),
            material=lid_color,
            name=f"rear_leaf_{side}",
        )
    lid.visual(
        Box((lid_frame_width, outer_width, lid_depth)),
        origin=Origin(
            xyz=(lid_frame_width / 2.0 + 0.010, 0.0, 0.004),
        ),
        material=lid_color,
        name="rear_frame",
    )
    lid.visual(
        Box((lid_frame_width, outer_width, lid_depth)),
        origin=Origin(
            xyz=(outer_length - lid_frame_width / 2.0 - 0.010, 0.0, 0.004),
        ),
        material=lid_color,
        name="front_frame",
    )
    side_frame_length = outer_length - 2.0 * lid_frame_width
    side_frame_center_x = outer_length / 2.0
    for side, y_sign in (("left", 1.0), ("right", -1.0)):
        lid.visual(
            Box((side_frame_length, lid_frame_width, lid_depth)),
            origin=Origin(
                xyz=(
                    side_frame_center_x,
                    y_sign * (outer_width / 2.0 - lid_frame_width / 2.0),
                    0.004,
                )
            ),
            material=lid_color,
            name=f"{side}_frame",
        )
    lid.visual(
        Box(
            (
                outer_length - 2.0 * lid_frame_width,
                outer_width - 2.0 * lid_frame_width,
                lid_center_thickness,
            )
        ),
        origin=Origin(
            xyz=(outer_length / 2.0, 0.0, 0.0),
        ),
        material=lid_color,
        name="center_panel",
    )

    lid_barrel_positions = (-0.032, 0.032)
    lid_barrel_lengths = (0.040, 0.040)
    for index, (y_pos, barrel_length) in enumerate(zip(lid_barrel_positions, lid_barrel_lengths)):
        lid.visual(
            Cylinder(radius=hinge_radius, length=barrel_length),
            origin=Origin(
                xyz=(0.0, y_pos, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_color,
            name=f"lid_hinge_knuckle_{index}",
        )

    lid.inertial = Inertial.from_geometry(
        Box((outer_length, outer_width, lid_depth)),
        mass=1.0,
        origin=Origin(xyz=(outer_length / 2.0, 0.0, 0.004)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(118.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="center_panel",
        negative_elem="front_wall",
        min_gap=0.0005,
        max_gap=0.003,
        name="closed lid sits just above the box rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="center_panel",
        elem_b="bottom_panel",
        min_overlap=0.17,
        name="lid covers the tackle box footprint",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_frame")
    with ctx.pose({lid_hinge: math.radians(105.0)}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_frame")

    closed_front_top = None if closed_front is None else closed_front[1][2]
    opened_front_top = None if opened_front is None else opened_front[1][2]
    ctx.check(
        "lid front edge rises when opened",
        closed_front_top is not None
        and opened_front_top is not None
        and opened_front_top > closed_front_top + 0.13,
        details=f"closed_top={closed_front_top}, opened_top={opened_front_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
