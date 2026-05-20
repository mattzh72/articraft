from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tackle_box")

    body_w = 0.40
    body_d = 0.24
    body_h = 0.16
    wall_t = 0.005
    lid_overhang = 0.006
    lid_shell_h = 0.035
    lid_t = 0.004

    plastic = model.material("tackle_box_plastic", rgba=(0.18, 0.35, 0.15, 1.0))
    trim = model.material("tackle_box_trim", rgba=(0.06, 0.06, 0.06, 1.0))
    tray_color = model.material("tray_insert", rgba=(0.73, 0.72, 0.66, 1.0))

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, wall_t / 2.0)),
        material=plastic,
        name="body_floor",
    )
    body.visual(
        Box((body_w, wall_t, body_h)),
        origin=Origin(xyz=(0.0, body_d / 2.0 - wall_t / 2.0, body_h / 2.0)),
        material=plastic,
        name="front_wall",
    )
    body.visual(
        Box((body_w, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 + wall_t / 2.0, body_h / 2.0)),
        material=plastic,
        name="rear_wall",
    )
    body.visual(
        Box((wall_t, body_d - 2.0 * wall_t, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - wall_t / 2.0, 0.0, body_h / 2.0)),
        material=plastic,
        name="right_wall",
    )
    body.visual(
        Box((wall_t, body_d - 2.0 * wall_t, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + wall_t / 2.0, 0.0, body_h / 2.0)),
        material=plastic,
        name="left_wall",
    )
    body.visual(
        Box((body_w * 0.26, wall_t, 0.020)),
        origin=Origin(xyz=(0.0, body_d / 2.0 + wall_t / 2.0, body_h * 0.72)),
        material=trim,
        name="front_grip_strip",
    )

    lid = model.part("lid")
    lid_w = body_w + 2.0 * lid_overhang
    lid_d = body_d + 2.0 * lid_overhang
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(xyz=(0.0, lid_d / 2.0, lid_shell_h + lid_t / 2.0)),
        material=plastic,
        name="lid_top",
    )
    lid.visual(
        Box((lid_w, lid_t, lid_shell_h)),
        origin=Origin(xyz=(0.0, lid_t / 2.0, lid_shell_h / 2.0)),
        material=plastic,
        name="lid_rear_skirt",
    )
    lid.visual(
        Box((lid_w, lid_t, lid_shell_h)),
        origin=Origin(xyz=(0.0, lid_d - lid_t / 2.0, lid_shell_h / 2.0)),
        material=plastic,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((lid_t, lid_d, lid_shell_h)),
        origin=Origin(xyz=(lid_w / 2.0 - lid_t / 2.0, lid_d / 2.0, lid_shell_h / 2.0)),
        material=plastic,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((lid_t, lid_d, lid_shell_h)),
        origin=Origin(xyz=(-lid_w / 2.0 + lid_t / 2.0, lid_d / 2.0, lid_shell_h / 2.0)),
        material=plastic,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((body_w * 0.20, lid_t, 0.016)),
        origin=Origin(xyz=(0.0, lid_d - lid_t / 2.0, lid_shell_h * 0.55)),
        material=trim,
        name="lid_pull",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -body_d / 2.0, body_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=0.72 * pi,
        ),
    )

    tray = model.part("tray_insert")
    tray_w = body_w - 2.0 * (wall_t + 0.010)
    tray_d = body_d - 2.0 * (wall_t + 0.012)
    tray_floor_t = 0.003
    tray_wall_t = 0.003
    tray_h = 0.045
    tray_floor_z = wall_t + tray_floor_t / 2.0
    tray_wall_z = wall_t + tray_h / 2.0
    tray.visual(
        Box((tray_w, tray_d, tray_floor_t)),
        origin=Origin(xyz=(0.0, 0.0, tray_floor_z)),
        material=tray_color,
        name="tray_floor",
    )
    tray.visual(
        Box((tray_w, tray_wall_t, tray_h)),
        origin=Origin(xyz=(0.0, tray_d / 2.0 - tray_wall_t / 2.0, tray_wall_z)),
        material=tray_color,
        name="tray_front_wall",
    )
    tray.visual(
        Box((tray_w, tray_wall_t, tray_h)),
        origin=Origin(xyz=(0.0, -tray_d / 2.0 + tray_wall_t / 2.0, tray_wall_z)),
        material=tray_color,
        name="tray_rear_wall",
    )
    tray.visual(
        Box((tray_wall_t, tray_d - 2.0 * tray_wall_t, tray_h)),
        origin=Origin(xyz=(tray_w / 2.0 - tray_wall_t / 2.0, 0.0, tray_wall_z)),
        material=tray_color,
        name="tray_right_wall",
    )
    tray.visual(
        Box((tray_wall_t, tray_d - 2.0 * tray_wall_t, tray_h)),
        origin=Origin(xyz=(-tray_w / 2.0 + tray_wall_t / 2.0, 0.0, tray_wall_z)),
        material=tray_color,
        name="tray_left_wall",
    )
    tray.visual(
        Box((tray_wall_t, tray_d - 2.0 * tray_wall_t, tray_h - 0.010)),
        origin=Origin(xyz=(0.0, 0.0, wall_t + (tray_h - 0.010) / 2.0)),
        material=tray_color,
        name="center_divider",
    )
    tray.visual(
        Box((tray_w * 0.44, tray_wall_t, tray_h - 0.015)),
        origin=Origin(
            xyz=(-tray_w * 0.28, -tray_d * 0.14, wall_t + (tray_h - 0.015) / 2.0)
        ),
        material=tray_color,
        name="left_cross_divider",
    )
    tray.visual(
        Box((tray_w * 0.44, tray_wall_t, tray_h - 0.015)),
        origin=Origin(
            xyz=(tray_w * 0.28, tray_d * 0.14, wall_t + (tray_h - 0.015) / 2.0)
        ),
        material=tray_color,
        name="right_cross_divider",
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
    lid_hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.003,
            max_penetration=0.001,
            name="closed lid seats on the box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.18,
            name="closed lid covers the box opening footprint",
        )
        ctx.expect_within(
            "tray_insert",
            body,
            axes="xy",
            margin=0.001,
            name="tray insert stays inside the body footprint",
        )

    body_aabb = ctx.part_world_aabb(body)
    tray_aabb = ctx.part_world_aabb("tray_insert")
    ctx.check(
        "tray sits below the body rim",
        body_aabb is not None
        and tray_aabb is not None
        and tray_aabb[1][2] < body_aabb[1][2] - 0.08,
        details=f"body={body_aabb}, tray={tray_aabb}",
    )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.35}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward from the rear hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.10,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
