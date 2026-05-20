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

    outer_w = 0.42
    outer_d = 0.23
    body_h = 0.19
    wall_t = 0.008
    floor_t = 0.010

    hinge_r = 0.006
    body_knuckle_len = 0.065
    lid_knuckle_len = 0.250
    knuckle_gap = 0.005

    lid_skin_t = 0.004
    lid_frame_h = 0.012

    tray_z = 0.114
    tray_t = 0.004
    tray_depth = 0.122
    tray_width = outer_w - 2.0 * wall_t + 0.004

    shell_mat = model.material("olive_shell", rgba=(0.29, 0.35, 0.20, 1.0))
    tray_mat = model.material("tray_plastic", rgba=(0.53, 0.50, 0.39, 1.0))
    hinge_mat = model.material("hinge_pin", rgba=(0.60, 0.60, 0.58, 1.0))

    body = model.part("body")
    body.visual(
        Box((outer_w, outer_d, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material=shell_mat,
        name="floor",
    )
    body.visual(
        Box((outer_w, wall_t, body_h)),
        origin=Origin(xyz=(0.0, outer_d / 2.0 - wall_t / 2.0, body_h / 2.0)),
        material=shell_mat,
        name="front_wall",
    )
    body.visual(
        Box((outer_w, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -outer_d / 2.0 + wall_t / 2.0, body_h / 2.0)),
        material=shell_mat,
        name="rear_wall",
    )
    body.visual(
        Box((wall_t, outer_d - 2.0 * wall_t, body_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + wall_t / 2.0, 0.0, body_h / 2.0)),
        material=shell_mat,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_d - 2.0 * wall_t, body_h)),
        origin=Origin(xyz=(outer_w / 2.0 - wall_t / 2.0, 0.0, body_h / 2.0)),
        material=shell_mat,
        name="right_wall",
    )

    body.visual(
        Box((tray_width, tray_depth, tray_t)),
        origin=Origin(xyz=(0.0, -0.018, tray_z)),
        material=tray_mat,
        name="tray_floor",
    )
    body.visual(
        Box((tray_width, 0.008, 0.022)),
        origin=Origin(xyz=(0.0, -0.018 + tray_depth / 2.0 - 0.004, tray_z + 0.011)),
        material=tray_mat,
        name="tray_front_lip",
    )
    body.visual(
        Box((tray_width, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, -0.018 - tray_depth / 2.0 + 0.004, tray_z + 0.009)),
        material=tray_mat,
        name="tray_back_lip",
    )
    body.visual(
        Box((0.006, tray_depth - 0.016, 0.024)),
        origin=Origin(xyz=(0.0, -0.018, tray_z + 0.012)),
        material=tray_mat,
        name="tray_divider",
    )
    body.visual(
        Box((0.006, outer_d - 0.070, 0.050)),
        origin=Origin(xyz=(0.0, 0.022, 0.025)),
        material=tray_mat,
        name="lower_center_divider",
    )
    body.visual(
        Box((outer_w - 0.100, 0.006, 0.034)),
        origin=Origin(xyz=(0.0, 0.060, 0.017)),
        material=tray_mat,
        name="lower_cross_divider",
    )

    knuckle_x = lid_knuckle_len / 2.0 + knuckle_gap + body_knuckle_len / 2.0
    for name, x_pos in (("left_hinge_knuckle", -knuckle_x), ("right_hinge_knuckle", knuckle_x)):
        body.visual(
            Cylinder(radius=hinge_r, length=body_knuckle_len),
            origin=Origin(
                xyz=(x_pos, -outer_d / 2.0, body_h + hinge_r),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=hinge_mat,
            name=name,
        )

    body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, body_h)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((outer_w, outer_d, lid_skin_t)),
        origin=Origin(
            xyz=(0.0, outer_d / 2.0, -hinge_r + lid_skin_t / 2.0),
        ),
        material=shell_mat,
        name="lid_skin",
    )
    lid.visual(
        Box((0.220, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, 0.010, 0.002)),
        material=shell_mat,
        name="rear_beam",
    )
    lid.visual(
        Box((outer_w - 0.050, 0.018, lid_frame_h)),
        origin=Origin(
            xyz=(0.0, outer_d - 0.028, -hinge_r + lid_skin_t + lid_frame_h / 2.0),
        ),
        material=shell_mat,
        name="front_frame",
    )
    side_rail_y = outer_d / 2.0
    side_rail_len = outer_d - 0.056
    side_rail_x = outer_w / 2.0 - 0.026
    for name, x_pos in (("left_frame", -side_rail_x), ("right_frame", side_rail_x)):
        lid.visual(
            Box((0.018, side_rail_len, lid_frame_h)),
            origin=Origin(
                xyz=(x_pos, side_rail_y, -hinge_r + lid_skin_t + lid_frame_h / 2.0),
            ),
            material=shell_mat,
            name=name,
        )
    lid.visual(
        Box((0.014, outer_d - 0.084, 0.010)),
        origin=Origin(xyz=(0.0, outer_d / 2.0 + 0.010, 0.001)),
        material=shell_mat,
        name="center_rib",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=lid_knuckle_len),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_mat,
        name="lid_hinge_knuckle",
    )

    lid.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, 0.028)),
        mass=0.45,
        origin=Origin(xyz=(0.0, outer_d / 2.0, 0.008)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -outer_d / 2.0, body_h + hinge_r)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_skin",
            negative_elem="front_wall",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid rests on the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.19,
            elem_a="lid_skin",
            elem_b="floor",
            name="closed lid covers the tackle box footprint",
        )

        closed_panel = ctx.part_element_world_aabb(lid, elem="lid_skin")

    with ctx.pose({hinge: 1.45}):
        open_panel = ctx.part_element_world_aabb(lid, elem="lid_skin")

    ctx.check(
        "lid swings upward on the rear hinge",
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][2] > closed_panel[1][2] + 0.14
        and open_panel[0][1] < closed_panel[0][1] + 0.03,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
