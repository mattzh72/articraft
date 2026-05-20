from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

    outer_w = 0.32
    outer_d = 0.185
    body_h = 0.095
    wall_t = 0.004
    floor_t = 0.004

    lid_top_t = 0.004
    lid_overhang_side = 0.003
    lid_overhang_front = 0.004
    skirt_t = 0.003
    skirt_drop = 0.020

    inner_w = outer_w - 2.0 * wall_t
    inner_d = outer_d - 2.0 * wall_t

    tray_w = inner_w - 0.022
    tray_d = 0.110
    tray_floor_t = 0.003
    tray_bottom_z = 0.046
    tray_lip_t = 0.003
    tray_lip_h = 0.014
    post_s = 0.008

    body_color = model.material("body_color", color=(0.20, 0.27, 0.17, 1.0))
    lid_color = model.material("lid_color", color=(0.17, 0.23, 0.14, 1.0))
    tray_color = model.material("tray_color", color=(0.82, 0.79, 0.70, 1.0))

    body = model.part("body")
    body.visual(
        Box((outer_w, outer_d, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material=body_color,
        name="body_floor",
    )
    wall_h = body_h - floor_t
    wall_z = floor_t + wall_h / 2.0
    body.visual(
        Box((wall_t, outer_d, wall_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + wall_t / 2.0, 0.0, wall_z)),
        material=body_color,
        name="body_left_wall",
    )
    body.visual(
        Box((wall_t, outer_d, wall_h)),
        origin=Origin(xyz=(outer_w / 2.0 - wall_t / 2.0, 0.0, wall_z)),
        material=body_color,
        name="body_right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -outer_d / 2.0 + wall_t / 2.0, wall_z)),
        material=body_color,
        name="body_rear_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, outer_d / 2.0 - wall_t / 2.0, wall_z)),
        material=body_color,
        name="body_front_wall",
    )

    lid = model.part("lid")
    lid_w = outer_w + 2.0 * lid_overhang_side
    lid_d = outer_d + lid_overhang_front
    lid.visual(
        Box((lid_w, lid_d, lid_top_t)),
        origin=Origin(xyz=(0.0, lid_d / 2.0, lid_top_t / 2.0)),
        material=lid_color,
        name="lid_panel",
    )
    skirt_span_d = outer_d - wall_t
    lid.visual(
        Box((skirt_t, skirt_span_d, skirt_drop)),
        origin=Origin(
            xyz=(-lid_w / 2.0 + skirt_t / 2.0, skirt_span_d / 2.0, -skirt_drop / 2.0)
        ),
        material=lid_color,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((skirt_t, skirt_span_d, skirt_drop)),
        origin=Origin(
            xyz=(lid_w / 2.0 - skirt_t / 2.0, skirt_span_d / 2.0, -skirt_drop / 2.0)
        ),
        material=lid_color,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((lid_w - 2.0 * skirt_t, skirt_t, skirt_drop)),
        origin=Origin(
            xyz=(0.0, lid_d - skirt_t / 2.0, -skirt_drop / 2.0)
        ),
        material=lid_color,
        name="lid_front_skirt",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -outer_d / 2.0, body_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.9,
        ),
    )

    tray = model.part("tray_insert")
    tray_floor_z = tray_bottom_z + tray_floor_t / 2.0
    tray.visual(
        Box((tray_w, tray_d, tray_floor_t)),
        origin=Origin(xyz=(0.0, 0.0, tray_floor_z)),
        material=tray_color,
        name="tray_floor",
    )
    tray_lip_z = tray_bottom_z + tray_floor_t + tray_lip_h / 2.0
    tray.visual(
        Box((tray_lip_t, tray_d, tray_lip_h)),
        origin=Origin(xyz=(-tray_w / 2.0 + tray_lip_t / 2.0, 0.0, tray_lip_z)),
        material=tray_color,
        name="tray_left_lip",
    )
    tray.visual(
        Box((tray_lip_t, tray_d, tray_lip_h)),
        origin=Origin(xyz=(tray_w / 2.0 - tray_lip_t / 2.0, 0.0, tray_lip_z)),
        material=tray_color,
        name="tray_right_lip",
    )
    tray.visual(
        Box((tray_w - 2.0 * tray_lip_t, tray_lip_t, tray_lip_h)),
        origin=Origin(xyz=(0.0, -tray_d / 2.0 + tray_lip_t / 2.0, tray_lip_z)),
        material=tray_color,
        name="tray_rear_lip",
    )
    tray.visual(
        Box((tray_w - 2.0 * tray_lip_t, tray_lip_t, tray_lip_h)),
        origin=Origin(xyz=(0.0, tray_d / 2.0 - tray_lip_t / 2.0, tray_lip_z)),
        material=tray_color,
        name="tray_front_lip",
    )
    tray.visual(
        Box((tray_lip_t, tray_d - 2.0 * tray_lip_t, tray_lip_h)),
        origin=Origin(xyz=(0.0, 0.0, tray_lip_z)),
        material=tray_color,
        name="tray_center_divider",
    )
    divider_y = tray_d * 0.22
    tray.visual(
        Box((tray_w * 0.44, tray_lip_t, tray_lip_h)),
        origin=Origin(xyz=(-tray_w * 0.14, -divider_y, tray_lip_z)),
        material=tray_color,
        name="tray_left_cross_divider",
    )
    tray.visual(
        Box((tray_w * 0.44, tray_lip_t, tray_lip_h)),
        origin=Origin(xyz=(tray_w * 0.14, divider_y, tray_lip_z)),
        material=tray_color,
        name="tray_right_cross_divider",
    )

    post_h = tray_bottom_z - floor_t
    post_z = floor_t + post_h / 2.0
    post_x = tray_w / 2.0 - post_s / 2.0 - 0.012
    post_y = tray_d / 2.0 - post_s / 2.0 - 0.012
    tray.visual(
        Box((post_s, post_s, post_h)),
        origin=Origin(xyz=(-post_x, -post_y, post_z)),
        material=tray_color,
        name="post_rear_left",
    )
    tray.visual(
        Box((post_s, post_s, post_h)),
        origin=Origin(xyz=(post_x, -post_y, post_z)),
        material=tray_color,
        name="post_rear_right",
    )
    tray.visual(
        Box((post_s, post_s, post_h)),
        origin=Origin(xyz=(-post_x, post_y, post_z)),
        material=tray_color,
        name="post_front_left",
    )
    tray.visual(
        Box((post_s, post_s, post_h)),
        origin=Origin(xyz=(post_x, post_y, post_z)),
        material=tray_color,
        name="post_front_right",
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
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray_insert")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="body_front_wall",
            max_gap=0.001,
            max_penetration=0.0,
            name="lid panel seats on body rim height",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="body_floor",
            min_overlap=0.16,
            name="lid panel covers the body opening footprint",
        )
        ctx.expect_within(
            tray,
            body,
            axes="xy",
            inner_elem="tray_floor",
            outer_elem="body_floor",
            margin=0.0,
            name="tray footprint stays inside the body",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="post_front_left",
            elem_b="body_floor",
            name="tray support post lands on the body floor",
        )

    rest_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.25}):
        open_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")

    if rest_panel is not None and open_panel is not None:
        rest_center_z = (rest_panel[0][2] + rest_panel[1][2]) / 2.0
        open_center_z = (open_panel[0][2] + open_panel[1][2]) / 2.0
        ctx.check(
            "lid opens upward from the rear hinge",
            open_center_z > rest_center_z + 0.05,
            details=f"rest_center_z={rest_center_z}, open_center_z={open_center_z}",
        )
    else:
        ctx.fail("lid panel aabb available", "Could not resolve lid panel AABBs.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
