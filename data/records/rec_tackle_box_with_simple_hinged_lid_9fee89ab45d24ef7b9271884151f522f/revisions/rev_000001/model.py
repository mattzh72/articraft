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

    body_color = model.material("body_green", color=(0.23, 0.36, 0.22))
    lid_color = model.material("lid_green", color=(0.28, 0.42, 0.26))
    tray_color = model.material("tray_tan", color=(0.66, 0.61, 0.49))
    hinge_color = model.material("hinge_dark", color=(0.14, 0.14, 0.14))

    body_w = 0.39
    body_d = 0.19
    body_h = 0.18
    wall_t = 0.008
    bottom_t = 0.008

    hinge_frame_d = 0.009
    hinge_frame_h = 0.216
    hinge_axis_y = 0.078
    hinge_axis_z = 0.187
    hinge_body_r = 0.0075
    hinge_lid_r = 0.0068

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=body_color,
        name="body_bottom",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(-(body_w - wall_t) / 2.0, 0.0, body_h / 2.0)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=((body_w - wall_t) / 2.0, 0.0, body_h / 2.0)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -(body_d - wall_t) / 2.0, body_h / 2.0)),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, 0.014, body_h)),
        origin=Origin(xyz=(0.0, 0.074, body_h / 2.0)),
        material=body_color,
        name="rear_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -(body_d / 2.0) + 0.011, body_h - 0.006)),
        material=body_color,
        name="front_lip",
    )
    body.visual(
        Box((0.014, body_d - hinge_frame_d - 0.022, 0.012)),
        origin=Origin(
            xyz=(
                -(body_w / 2.0) + 0.011,
                -0.006,
                body_h - 0.006,
            )
        ),
        material=body_color,
        name="left_lip",
    )
    body.visual(
        Box((0.014, body_d - hinge_frame_d - 0.022, 0.012)),
        origin=Origin(
            xyz=(
                (body_w / 2.0) - 0.011,
                -0.006,
                body_h - 0.006,
            )
        ),
        material=body_color,
        name="right_lip",
    )
    body.visual(
        Box((body_w, hinge_frame_d, hinge_frame_h)),
        origin=Origin(
            xyz=(
                0.0,
                body_d / 2.0 - hinge_frame_d / 2.0,
                hinge_frame_h / 2.0,
            )
        ),
        material=body_color,
        name="hinge_frame",
    )
    body.visual(
        Box((0.024, 0.024, hinge_frame_h)),
        origin=Origin(xyz=(-(body_w / 2.0) + 0.012, 0.086, hinge_frame_h / 2.0)),
        material=body_color,
        name="left_hinge_post",
    )
    body.visual(
        Box((0.024, 0.024, hinge_frame_h)),
        origin=Origin(xyz=((body_w / 2.0) - 0.012, 0.086, hinge_frame_h / 2.0)),
        material=body_color,
        name="right_hinge_post",
    )

    for idx, x_center in enumerate((-0.156, 0.0, 0.156), start=1):
        body.visual(
            Cylinder(radius=hinge_body_r, length=0.072),
            origin=Origin(xyz=(x_center, hinge_axis_y, hinge_axis_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_color,
            name=f"body_hinge_knuckle_{idx}",
        )

    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, hinge_frame_h)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, hinge_frame_h / 2.0)),
    )

    tray = model.part("tray")
    tray_outer_w = body_w - 2.0 * wall_t
    tray_outer_d = 0.120
    tray_wall = 0.004
    tray_h = 0.028

    tray.visual(
        Box((tray_outer_w, tray_outer_d, tray_wall)),
        origin=Origin(xyz=(0.0, -0.010, tray_wall / 2.0)),
        material=tray_color,
        name="tray_floor",
    )
    tray.visual(
        Box((tray_wall, tray_outer_d, tray_h)),
        origin=Origin(xyz=(-(tray_outer_w - tray_wall) / 2.0, -0.010, tray_h / 2.0)),
        material=tray_color,
        name="tray_left_wall",
    )
    tray.visual(
        Box((tray_wall, tray_outer_d, tray_h)),
        origin=Origin(xyz=((tray_outer_w - tray_wall) / 2.0, -0.010, tray_h / 2.0)),
        material=tray_color,
        name="tray_right_wall",
    )
    tray.visual(
        Box((tray_outer_w - 2.0 * tray_wall, tray_wall, tray_h)),
        origin=Origin(xyz=(0.0, -0.010 - (tray_outer_d - tray_wall) / 2.0, tray_h / 2.0)),
        material=tray_color,
        name="tray_front_wall",
    )
    tray.visual(
        Box((tray_outer_w - 2.0 * tray_wall, tray_wall, tray_h)),
        origin=Origin(xyz=(0.0, -0.010 + (tray_outer_d - tray_wall) / 2.0, tray_h / 2.0)),
        material=tray_color,
        name="tray_rear_wall",
    )
    tray.visual(
        Box((tray_wall, tray_outer_d - 2.0 * tray_wall, tray_h - 0.006)),
        origin=Origin(xyz=(-0.070, -0.010, (tray_h - 0.006) / 2.0)),
        material=tray_color,
        name="tray_divider_left",
    )
    tray.visual(
        Box((tray_wall, tray_outer_d - 2.0 * tray_wall, tray_h - 0.006)),
        origin=Origin(xyz=(0.070, -0.010, (tray_h - 0.006) / 2.0)),
        material=tray_color,
        name="tray_divider_right",
    )
    tray.inertial = Inertial.from_geometry(
        Box((tray_outer_w, tray_outer_d, tray_h)),
        mass=0.25,
        origin=Origin(xyz=(0.0, -0.010, tray_h / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.402, 0.172, 0.014)),
        origin=Origin(xyz=(0.0, -0.096, 0.0)),
        material=lid_color,
        name="lid_panel",
    )
    lid.visual(
        Box((0.060, 0.010, 0.018)),
        origin=Origin(xyz=(-0.087, -0.005, -0.002)),
        material=lid_color,
        name="lid_hinge_web_left",
    )
    lid.visual(
        Box((0.060, 0.010, 0.018)),
        origin=Origin(xyz=(0.087, -0.005, -0.002)),
        material=lid_color,
        name="lid_hinge_web_right",
    )
    lid.visual(
        Box((0.390, 0.006, 0.028)),
        origin=Origin(xyz=(0.0, -0.175, -0.016)),
        material=lid_color,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((0.006, 0.164, 0.028)),
        origin=Origin(xyz=(-0.198, -0.082, -0.016)),
        material=lid_color,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((0.006, 0.164, 0.028)),
        origin=Origin(xyz=(0.198, -0.082, -0.016)),
        material=lid_color,
        name="lid_right_skirt",
    )
    for idx, x_center in enumerate((-0.087, 0.087), start=1):
        lid.visual(
            Cylinder(radius=hinge_lid_r, length=0.054),
            origin=Origin(xyz=(x_center, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_color,
            name=f"lid_hinge_knuckle_{idx}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((0.402, 0.172, 0.040)),
        mass=0.6,
        origin=Origin(xyz=(0.0, -0.096, -0.006)),
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.FIXED,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=2.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    tray = object_model.get_part("tray")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_contact(
        lid,
        body,
        elem_a="lid_panel",
        elem_b="front_lip",
        name="closed lid seats on the front lip",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        min_overlap=0.15,
        name="closed lid covers the box opening footprint",
    )
    ctx.expect_gap(
        tray,
        body,
        axis="z",
        positive_elem="tray_floor",
        negative_elem="body_bottom",
        min_gap=0.08,
        max_gap=0.09,
        name="fixed tray sits above the box floor",
    )
    ctx.expect_within(
        tray,
        body,
        axes="xy",
        margin=0.0,
        name="fixed tray remains within the body footprint",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    with ctx.pose({lid_hinge: 1.2}):
        open_front = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_front_skirt",
            negative_elem="front_wall",
            min_gap=0.05,
            name="opened lid front edge rises clear of the body",
        )

    if closed_front is not None and open_front is not None:
        ctx.check(
            "lid front edge swings upward",
            open_front[0][2] > closed_front[0][2] + 0.08,
            details=f"closed_min_z={closed_front[0][2]:.4f}, open_min_z={open_front[0][2]:.4f}",
        )
    else:
        ctx.fail("lid front edge swings upward", "Could not resolve lid front skirt AABBs.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
