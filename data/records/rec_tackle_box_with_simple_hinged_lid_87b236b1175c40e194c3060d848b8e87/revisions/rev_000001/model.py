from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

    body_green = model.material("body_green", rgba=(0.22, 0.39, 0.22, 1.0))
    lid_green = model.material("lid_green", rgba=(0.25, 0.44, 0.24, 1.0))
    tray_tan = model.material("tray_tan", rgba=(0.77, 0.73, 0.62, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.14, 0.15, 0.15, 1.0))

    body_outer_x = 0.36
    body_outer_y = 0.20
    body_height = 0.18
    wall_t = 0.008
    floor_t = 0.008

    lid_outer_x = 0.372
    lid_outer_y = 0.206
    lid_height = 0.024
    lid_top_t = 0.004
    lid_skirt_t = 0.006

    tray_floor_z = 0.112
    tray_floor_t = 0.004
    tray_divider_h = 0.022

    inner_x = body_outer_x - (2.0 * wall_t)
    inner_y = body_outer_y - (2.0 * wall_t)
    wall_h = body_height - floor_t

    body = model.part("body")
    body.visual(
        Box((body_outer_x, body_outer_y, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t * 0.5)),
        material=body_green,
        name="floor",
    )
    body.visual(
        Box((wall_t, body_outer_y, wall_h)),
        origin=Origin(xyz=((body_outer_x - wall_t) * 0.5, 0.0, floor_t + wall_h * 0.5)),
        material=body_green,
        name="right_wall",
    )
    body.visual(
        Box((wall_t, body_outer_y, wall_h)),
        origin=Origin(xyz=(-(body_outer_x - wall_t) * 0.5, 0.0, floor_t + wall_h * 0.5)),
        material=body_green,
        name="left_wall",
    )
    body.visual(
        Box((inner_x, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -((body_outer_y - wall_t) * 0.5), floor_t + wall_h * 0.5)),
        material=body_green,
        name="front_wall",
    )
    body.visual(
        Box((inner_x, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, ((body_outer_y - wall_t) * 0.5), floor_t + wall_h * 0.5)),
        material=body_green,
        name="rear_wall",
    )
    body.visual(
        Box((inner_x, inner_y, tray_floor_t)),
        origin=Origin(xyz=(0.0, 0.0, tray_floor_z + tray_floor_t * 0.5)),
        material=tray_tan,
        name="tray_floor",
    )
    body.visual(
        Box((0.004, inner_y - 0.008, tray_divider_h)),
        origin=Origin(xyz=(0.0, 0.0, tray_floor_z + tray_floor_t + tray_divider_h * 0.5)),
        material=tray_tan,
        name="tray_center_divider",
    )
    body.visual(
        Box((0.166, 0.004, tray_divider_h)),
        origin=Origin(xyz=(-0.085, -0.045, tray_floor_z + tray_floor_t + tray_divider_h * 0.5)),
        material=tray_tan,
        name="tray_left_front_divider",
    )
    body.visual(
        Box((0.166, 0.004, tray_divider_h)),
        origin=Origin(xyz=(0.085, 0.040, tray_floor_z + tray_floor_t + tray_divider_h * 0.5)),
        material=tray_tan,
        name="tray_right_rear_divider",
    )
    body.visual(
        Box((inner_x, 0.004, lid_height)),
        origin=Origin(xyz=(0.0, body_outer_y * 0.5 - 0.002, body_height + lid_height * 0.5)),
        material=hinge_dark,
        name="rear_hinge_flange",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_outer_x, body_outer_y, body_height + lid_height)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, (body_height + lid_height) * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_outer_x, lid_outer_y, lid_top_t)),
        origin=Origin(xyz=(0.0, -(lid_outer_y * 0.5), -(lid_top_t * 0.5))),
        material=lid_green,
        name="top_panel",
    )
    lid.visual(
        Box((lid_outer_x, lid_skirt_t, lid_height - lid_top_t)),
        origin=Origin(
            xyz=(
                0.0,
                -(lid_skirt_t * 0.5),
                -(lid_top_t + (lid_height - lid_top_t) * 0.5),
            )
        ),
        material=hinge_dark,
        name="rear_skirt",
    )
    lid.visual(
        Box((lid_outer_x, lid_skirt_t, lid_height - lid_top_t)),
        origin=Origin(
            xyz=(
                0.0,
                -(lid_outer_y - lid_skirt_t * 0.5),
                -(lid_top_t + (lid_height - lid_top_t) * 0.5),
            )
        ),
        material=hinge_dark,
        name="front_skirt",
    )
    lid.visual(
        Box((lid_skirt_t, lid_outer_y, lid_height - lid_top_t)),
        origin=Origin(
            xyz=(
                (lid_outer_x - lid_skirt_t) * 0.5,
                -(lid_outer_y * 0.5),
                -(lid_top_t + (lid_height - lid_top_t) * 0.5),
            )
        ),
        material=hinge_dark,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_skirt_t, lid_outer_y, lid_height - lid_top_t)),
        origin=Origin(
            xyz=(
                -((lid_outer_x - lid_skirt_t) * 0.5),
                -(lid_outer_y * 0.5),
                -(lid_top_t + (lid_height - lid_top_t) * 0.5),
            )
        ),
        material=hinge_dark,
        name="left_skirt",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_outer_x, lid_outer_y, lid_height)),
        mass=0.45,
        origin=Origin(xyz=(0.0, -(lid_outer_y * 0.5), -(lid_height * 0.5))),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, body_outer_y * 0.5 + lid_skirt_t, body_height + lid_height)),
        # Closed lid geometry extends forward along local -Y from the rear hinge.
        # Using -X makes positive joint values lift the front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.19,
            name="closed lid covers the tackle box body",
        )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({hinge: 2.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_skirt",
            min_gap=0.10,
            name="opened lid front edge clears the box body",
        )
        open_front = ctx.part_element_world_aabb(lid, elem="front_skirt")

    def center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    closed_front_z = center_z(closed_front)
    open_front_z = center_z(open_front)
    ctx.check(
        "lid rotates upward about the rear hinge",
        closed_front_z is not None and open_front_z is not None and open_front_z > closed_front_z + 0.16,
        details=f"closed_front_z={closed_front_z}, open_front_z={open_front_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
