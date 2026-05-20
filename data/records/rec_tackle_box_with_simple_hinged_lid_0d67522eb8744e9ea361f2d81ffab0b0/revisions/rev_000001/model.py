from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tackle_box")

    body_plastic = model.material("body_plastic", rgba=(0.16, 0.27, 0.30, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.19, 0.32, 0.35, 1.0))
    latch_metal = model.material("latch_metal", rgba=(0.73, 0.76, 0.79, 1.0))

    body_length = 0.38
    body_width = 0.23
    body_height = 0.19
    floor_thickness = 0.010
    left_wall = 0.014
    right_wall = 0.040
    rear_wall = 0.014
    front_wall = 0.018
    wall_height = body_height - floor_thickness

    opening_length = body_length - rear_wall - front_wall
    opening_width = body_width - left_wall - right_wall
    opening_center_y = ((-body_width / 2.0 + left_wall) + (body_width / 2.0 - right_wall)) / 2.0

    body = model.part("body")
    body.visual(
        Box((body_length, body_width, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=body_plastic,
        name="floor_panel",
    )
    body.visual(
        Box((body_length, left_wall, wall_height)),
        origin=Origin(
            xyz=(0.0, -body_width / 2.0 + left_wall / 2.0, floor_thickness + wall_height / 2.0)
        ),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((body_length, right_wall, wall_height)),
        origin=Origin(
            xyz=(0.0, body_width / 2.0 - right_wall / 2.0, floor_thickness + wall_height / 2.0)
        ),
        material=body_plastic,
        name="right_wall",
    )
    body.visual(
        Box((front_wall, opening_width, wall_height)),
        origin=Origin(
            xyz=(body_length / 2.0 - front_wall / 2.0, opening_center_y, floor_thickness + wall_height / 2.0)
        ),
        material=body_plastic,
        name="front_wall",
    )
    body.visual(
        Box((rear_wall, opening_width, wall_height)),
        origin=Origin(
            xyz=(-body_length / 2.0 + rear_wall / 2.0, opening_center_y, floor_thickness + wall_height / 2.0)
        ),
        material=body_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((0.004, 0.042, 0.020)),
        origin=Origin(xyz=(body_length / 2.0 + 0.002, opening_center_y, 0.124)),
        material=latch_metal,
        name="latch_strike",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_height)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    lid = model.part("lid")
    hinge_back_offset = 0.004
    hinge_raise = 0.004
    lid_thickness = 0.006
    lid_length = opening_length - 0.004
    lid_width = opening_width - 0.004
    lid.visual(
        Box((lid_length, lid_width, lid_thickness)),
        origin=Origin(
            xyz=(
                lid_length / 2.0 + hinge_back_offset,
                0.0,
                lid_thickness / 2.0 - hinge_raise,
            )
        ),
        material=lid_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.022, 0.072, 0.010)),
        origin=Origin(xyz=(lid_length - 0.007, 0.0, 0.007)),
        material=lid_plastic,
        name="finger_lift",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_width, 0.028)),
        mass=0.45,
        origin=Origin(xyz=(lid_length / 2.0 + hinge_back_offset, 0.0, 0.010)),
    )

    tray_insert = model.part("tray_insert")
    tray_length = 0.252
    tray_width = 0.150
    tray_floor_thickness = 0.004
    tray_wall = 0.004
    tray_height = 0.024
    tray_floor_z = 0.078
    tray_center_x = 0.018
    tray_center_y = opening_center_y - 0.003
    tray_insert.visual(
        Box((tray_length, tray_width, tray_floor_thickness)),
        origin=Origin(xyz=(tray_center_x, tray_center_y, tray_floor_z)),
        material=lid_plastic,
        name="tray_floor",
    )
    tray_insert.visual(
        Box((tray_length, tray_wall, tray_height)),
        origin=Origin(
            xyz=(
                tray_center_x,
                tray_center_y - tray_width / 2.0 + tray_wall / 2.0,
                tray_floor_z + tray_floor_thickness / 2.0 + tray_height / 2.0,
            )
        ),
        material=lid_plastic,
        name="tray_left_wall",
    )
    tray_insert.visual(
        Box((tray_length, tray_wall, tray_height)),
        origin=Origin(
            xyz=(
                tray_center_x,
                tray_center_y + tray_width / 2.0 - tray_wall / 2.0,
                tray_floor_z + tray_floor_thickness / 2.0 + tray_height / 2.0,
            )
        ),
        material=lid_plastic,
        name="tray_right_wall",
    )
    tray_insert.visual(
        Box((tray_wall, tray_width - 2.0 * tray_wall, tray_height)),
        origin=Origin(
            xyz=(
                tray_center_x - tray_length / 2.0 + tray_wall / 2.0,
                tray_center_y,
                tray_floor_z + tray_floor_thickness / 2.0 + tray_height / 2.0,
            )
        ),
        material=lid_plastic,
        name="tray_rear_wall",
    )
    tray_insert.visual(
        Box((tray_wall, tray_width - 2.0 * tray_wall, tray_height)),
        origin=Origin(
            xyz=(
                tray_center_x + tray_length / 2.0 - tray_wall / 2.0,
                tray_center_y,
                tray_floor_z + tray_floor_thickness / 2.0 + tray_height / 2.0,
            )
        ),
        material=lid_plastic,
        name="tray_front_wall",
    )
    tray_insert.visual(
        Box((0.150, tray_wall, 0.020)),
        origin=Origin(xyz=(tray_center_x - 0.026, tray_center_y - 0.018, tray_floor_z + 0.012)),
        material=lid_plastic,
        name="tray_divider_long",
    )
    tray_insert.visual(
        Box((tray_wall, 0.052, 0.020)),
        origin=Origin(xyz=(tray_center_x + 0.040, tray_center_y + 0.030, tray_floor_z + 0.012)),
        material=lid_plastic,
        name="tray_divider_cross",
    )
    post_height = tray_floor_z - tray_floor_thickness / 2.0 - floor_thickness
    for post_name, post_x, post_y in (
        ("support_post_rear_left", tray_center_x - 0.090, tray_center_y - 0.048),
        ("support_post_rear_right", tray_center_x - 0.090, tray_center_y + 0.048),
        ("support_post_front_left", tray_center_x + 0.090, tray_center_y - 0.048),
        ("support_post_front_right", tray_center_x + 0.090, tray_center_y + 0.048),
    ):
        tray_insert.visual(
            Box((0.012, 0.012, post_height)),
            origin=Origin(xyz=(post_x, post_y, floor_thickness + post_height / 2.0)),
            material=lid_plastic,
            name=post_name,
        )
    tray_insert.inertial = Inertial.from_geometry(
        Box((tray_length, tray_width, tray_floor_z + tray_height)),
        mass=0.35,
        origin=Origin(xyz=(tray_center_x, tray_center_y, (tray_floor_z + tray_height) / 2.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(
            xyz=(
                -body_length / 2.0 + rear_wall - hinge_back_offset,
                opening_center_y,
                body_height + hinge_raise,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    model.articulation(
        "body_to_tray_insert",
        ArticulationType.FIXED,
        parent=body,
        child=tray_insert,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray_insert = object_model.get_part("tray_insert")
    lid_hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem="lid_panel",
            name="closed lid sits just above the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.16,
            elem_a="lid_panel",
            name="lid covers the tackle-box opening footprint",
        )
        ctx.expect_within(
            tray_insert,
            body,
            axes="xy",
            margin=0.0,
            name="tray insert remains inside the body cavity",
        )
        ctx.expect_contact(
            tray_insert,
            body,
            name="tray insert is physically supported by the body",
        )

    left_wall_aabb = ctx.part_element_world_aabb(body, elem="left_wall")
    right_wall_aabb = ctx.part_element_world_aabb(body, elem="right_wall")
    left_wall_width = None if left_wall_aabb is None else left_wall_aabb[1][1] - left_wall_aabb[0][1]
    right_wall_width = None if right_wall_aabb is None else right_wall_aabb[1][1] - right_wall_aabb[0][1]
    ctx.check(
        "opening is offset so the right body side is heavier",
        left_wall_width is not None
        and right_wall_width is not None
        and right_wall_width > left_wall_width + 0.02,
        details=f"left_wall_width={left_wall_width}, right_wall_width={right_wall_width}",
    )

    closed_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.check(
            "lid opens upward around the rear hinge",
            closed_lid_panel is not None
            and open_lid_panel is not None
            and open_lid_panel[1][2] > closed_lid_panel[1][2] + 0.12
            and open_lid_panel[1][0] < closed_lid_panel[1][0] - 0.08,
            details=f"closed={closed_lid_panel}, open={open_lid_panel}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
