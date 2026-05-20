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

    body_color = model.material("body_plastic", rgba=(0.31, 0.41, 0.22, 1.0))
    lid_color = model.material("lid_plastic", rgba=(0.41, 0.49, 0.28, 1.0))
    inner_color = model.material("inner_plastic", rgba=(0.37, 0.46, 0.25, 1.0))

    body_width = 0.42
    body_depth = 0.22
    body_height = 0.18
    wall_t = 0.004
    floor_t = 0.004
    fit_eps = 0.0005

    inner_width = body_width - 2.0 * wall_t
    inner_depth = body_depth - 2.0 * wall_t
    wall_height = body_height - floor_t

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t / 2.0)),
        material=body_color,
        name="floor",
    )
    body.visual(
        Box((body_width, wall_t, wall_height)),
        origin=Origin(
            xyz=(0.0, body_depth / 2.0 - wall_t / 2.0, floor_t + wall_height / 2.0)
        ),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((body_width, wall_t, wall_height)),
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0 + wall_t / 2.0, floor_t + wall_height / 2.0)
        ),
        material=body_color,
        name="rear_wall",
    )
    side_depth = body_depth - 2.0 * wall_t + 2.0 * fit_eps
    body.visual(
        Box((wall_t, side_depth, wall_height)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + wall_t / 2.0,
                0.0,
                floor_t + wall_height / 2.0,
            )
        ),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, side_depth, wall_height)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - wall_t / 2.0,
                0.0,
                floor_t + wall_height / 2.0,
            )
        ),
        material=body_color,
        name="right_wall",
    )

    rail_height = 0.008
    rail_depth = inner_depth - 0.020
    rail_width = 0.012
    rail_z = 0.106
    rail_x = body_width / 2.0 - wall_t - rail_width / 2.0 + fit_eps
    body.visual(
        Box((rail_width, rail_depth, rail_height)),
        origin=Origin(xyz=(-rail_x, 0.0, rail_z)),
        material=inner_color,
        name="left_tray_rail",
    )
    body.visual(
        Box((rail_width, rail_depth, rail_height)),
        origin=Origin(xyz=(rail_x, 0.0, rail_z)),
        material=inner_color,
        name="right_tray_rail",
    )

    divider_t = 0.004
    center_divider_height = 0.072
    center_divider_depth = inner_depth - 0.024
    body.visual(
        Box((divider_t, center_divider_depth, center_divider_height)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                floor_t + center_divider_height / 2.0 - fit_eps,
            )
        ),
        material=inner_color,
        name="center_divider",
    )

    side_bin_width = inner_width / 2.0 - divider_t / 2.0 + fit_eps
    side_bin_center_x = inner_width / 4.0 - divider_t / 4.0
    cross_divider_height = 0.048
    cross_divider_y = 0.032
    body.visual(
        Box((side_bin_width, divider_t, cross_divider_height)),
        origin=Origin(
            xyz=(
                -side_bin_center_x,
                cross_divider_y,
                floor_t + cross_divider_height / 2.0 - fit_eps,
            )
        ),
        material=inner_color,
        name="left_cross_divider",
    )
    body.visual(
        Box((side_bin_width, divider_t, cross_divider_height)),
        origin=Origin(
            xyz=(
                side_bin_center_x,
                -cross_divider_y,
                floor_t + cross_divider_height / 2.0 - fit_eps,
            )
        ),
        material=inner_color,
        name="right_cross_divider",
    )

    lid = model.part("lid")
    lid_width = body_width + 0.008
    lid_depth = body_depth + 0.004
    lid_t = 0.012
    skirt_t = 0.003
    skirt_height = 0.018
    rear_clear = 0.012

    lid.visual(
        Box((lid_width, lid_depth, lid_t)),
        origin=Origin(xyz=(0.0, lid_depth / 2.0, lid_t / 2.0)),
        material=lid_color,
        name="lid_panel",
    )
    side_skirt_depth = lid_depth - rear_clear
    side_skirt_center_y = rear_clear + side_skirt_depth / 2.0
    side_skirt_z = -skirt_height / 2.0 + fit_eps
    lid.visual(
        Box((skirt_t, side_skirt_depth, skirt_height + 2.0 * fit_eps)),
        origin=Origin(
            xyz=(-lid_width / 2.0 + skirt_t / 2.0, side_skirt_center_y, side_skirt_z)
        ),
        material=lid_color,
        name="left_skirt",
    )
    lid.visual(
        Box((skirt_t, side_skirt_depth, skirt_height + 2.0 * fit_eps)),
        origin=Origin(
            xyz=(lid_width / 2.0 - skirt_t / 2.0, side_skirt_center_y, side_skirt_z)
        ),
        material=lid_color,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_width - 2.0 * skirt_t, skirt_t, skirt_height + 2.0 * fit_eps)),
        origin=Origin(
            xyz=(0.0, lid_depth - skirt_t / 2.0, side_skirt_z)
        ),
        material=lid_color,
        name="front_skirt",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -body_depth / 2.0, body_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=2.2,
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
            positive_elem="lid_panel",
            max_gap=0.001,
            max_penetration=0.0,
            name="lid panel seats on the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.20,
            name="lid panel covers the tackle box opening",
        )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({hinge: 1.7}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_skirt")

    ctx.check(
        "lid front edge rises when opened",
        closed_front is not None
        and opened_front is not None
        and opened_front[1][2] > closed_front[1][2] + 0.10,
        details=f"closed={closed_front}, opened={opened_front}",
    )
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
