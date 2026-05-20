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
    model = ArticulatedObject(name="architect_flat_file_cabinet")

    body_color = model.material("body_enamel", rgba=(0.71, 0.70, 0.64, 1.0))
    drawer_color = model.material("drawer_face", rgba=(0.77, 0.76, 0.70, 1.0))
    slide_color = model.material("slide_zinc", rgba=(0.56, 0.58, 0.60, 1.0))
    pull_color = model.material("pull_dark", rgba=(0.18, 0.20, 0.22, 1.0))

    cabinet = model.part("cabinet_body")

    width = 1.20
    depth = 0.90
    height = 0.74
    drawer_count = 8

    side_t = 0.020
    top_t = 0.022
    bottom_t = 0.022
    back_t = 0.012
    divider_t = 0.010

    cavity_height = height - top_t - bottom_t
    opening_h = (cavity_height - (drawer_count - 1) * divider_t) / drawer_count

    drawer_face_t = 0.018
    drawer_front_inset = 0.002
    drawer_face_bottom_gap = 0.002
    drawer_face_top_gap = 0.002
    drawer_face_h = opening_h - drawer_face_bottom_gap - drawer_face_top_gap

    drawer_side_gap = 0.006
    drawer_face_side_gap = 0.004
    inner_w = width - 2.0 * side_t
    drawer_outer_w = inner_w - 2.0 * drawer_side_gap
    drawer_face_w = inner_w - 2.0 * drawer_face_side_gap

    tray_depth = 0.79
    drawer_wall_t = 0.010
    drawer_bottom_t = 0.006
    drawer_side_h = 0.056

    rail_h = 0.002
    runner_t = 0.002
    rail_w = 0.012
    runner_w = 0.012
    rail_len = 0.72
    runner_len = 0.72
    runner_front_setback = 0.040
    runner_inset_from_side = 0.014

    drawer_travel = 0.32

    cabinet_front_y = depth / 2.0
    drawer_face_outer_y = cabinet_front_y - drawer_front_inset
    shelf_front_y = drawer_face_outer_y - drawer_face_t - 0.002
    shelf_back_y = -depth / 2.0 + back_t
    shelf_depth = shelf_front_y - shelf_back_y
    shelf_center_y = (shelf_front_y + shelf_back_y) / 2.0

    shell_side_h = height - top_t - bottom_t
    shell_side_z = bottom_t + shell_side_h / 2.0

    rail_center_y = drawer_face_outer_y - drawer_face_t - runner_front_setback - rail_len / 2.0
    runner_center_y = -drawer_face_t - runner_front_setback - runner_len / 2.0
    runner_x = drawer_outer_w / 2.0 - runner_w / 2.0 - runner_inset_from_side

    cabinet.visual(
        Box((width, depth, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=body_color,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, height - top_t / 2.0)),
        material=body_color,
        name="top_panel",
    )
    cabinet.visual(
        Box((side_t, depth, shell_side_h)),
        origin=Origin(xyz=(-width / 2.0 + side_t / 2.0, 0.0, shell_side_z)),
        material=body_color,
        name="left_side",
    )
    cabinet.visual(
        Box((side_t, depth, shell_side_h)),
        origin=Origin(xyz=(width / 2.0 - side_t / 2.0, 0.0, shell_side_z)),
        material=body_color,
        name="right_side",
    )
    cabinet.visual(
        Box((inner_w, back_t, shell_side_h)),
        origin=Origin(
            xyz=(0.0, -depth / 2.0 + back_t / 2.0, shell_side_z),
        ),
        material=body_color,
        name="back_panel",
    )

    for drawer_index in range(drawer_count - 1):
        divider_z = bottom_t + (drawer_index + 1) * opening_h + drawer_index * divider_t + divider_t / 2.0
        cabinet.visual(
            Box((inner_w, shelf_depth, divider_t)),
            origin=Origin(xyz=(0.0, shelf_center_y, divider_z)),
            material=body_color,
            name=f"divider_{drawer_index + 1}",
        )

    for drawer_index in range(drawer_count):
        opening_floor_z = bottom_t + drawer_index * (opening_h + divider_t)
        left_rail_name = f"drawer_{drawer_index + 1}_left_rail"
        right_rail_name = f"drawer_{drawer_index + 1}_right_rail"

        cabinet.visual(
            Box((rail_w, rail_len, rail_h)),
            origin=Origin(xyz=(-runner_x, rail_center_y, opening_floor_z + rail_h / 2.0)),
            material=slide_color,
            name=left_rail_name,
        )
        cabinet.visual(
            Box((rail_w, rail_len, rail_h)),
            origin=Origin(xyz=(runner_x, rail_center_y, opening_floor_z + rail_h / 2.0)),
            material=slide_color,
            name=right_rail_name,
        )

        drawer = model.part(f"drawer_{drawer_index + 1}")

        drawer.visual(
            Box((drawer_face_w, drawer_face_t, drawer_face_h)),
            origin=Origin(xyz=(0.0, -drawer_face_t / 2.0, drawer_face_h / 2.0)),
            material=drawer_color,
            name="drawer_face",
        )
        drawer.visual(
            Box((0.26, 0.016, 0.010)),
            origin=Origin(xyz=(0.0, 0.008, drawer_face_h * 0.56)),
            material=pull_color,
            name="pull_bar",
        )
        drawer.visual(
            Box((drawer_outer_w - 2.0 * drawer_wall_t, tray_depth, drawer_bottom_t)),
            origin=Origin(
                xyz=(0.0, -drawer_face_t - tray_depth / 2.0, runner_t + drawer_bottom_t / 2.0),
            ),
            material=body_color,
            name="drawer_bottom",
        )
        drawer.visual(
            Box((drawer_wall_t, tray_depth, drawer_side_h)),
            origin=Origin(
                xyz=(
                    -drawer_outer_w / 2.0 + drawer_wall_t / 2.0,
                    -drawer_face_t - tray_depth / 2.0,
                    runner_t + drawer_bottom_t + drawer_side_h / 2.0,
                ),
            ),
            material=body_color,
            name="left_wall",
        )
        drawer.visual(
            Box((drawer_wall_t, tray_depth, drawer_side_h)),
            origin=Origin(
                xyz=(
                    drawer_outer_w / 2.0 - drawer_wall_t / 2.0,
                    -drawer_face_t - tray_depth / 2.0,
                    runner_t + drawer_bottom_t + drawer_side_h / 2.0,
                ),
            ),
            material=body_color,
            name="right_wall",
        )
        drawer.visual(
            Box((drawer_outer_w - 2.0 * drawer_wall_t, drawer_wall_t, drawer_side_h)),
            origin=Origin(
                xyz=(
                    0.0,
                    -drawer_face_t - tray_depth + drawer_wall_t / 2.0,
                    runner_t + drawer_bottom_t + drawer_side_h / 2.0,
                ),
            ),
            material=body_color,
            name="back_wall",
        )
        drawer.visual(
            Box((runner_w, runner_len, runner_t)),
            origin=Origin(xyz=(-runner_x, runner_center_y, runner_t / 2.0)),
            material=slide_color,
            name="left_runner",
        )
        drawer.visual(
            Box((runner_w, runner_len, runner_t)),
            origin=Origin(xyz=(runner_x, runner_center_y, runner_t / 2.0)),
            material=slide_color,
            name="right_runner",
        )

        model.articulation(
            f"cabinet_to_drawer_{drawer_index + 1}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(
                xyz=(
                    0.0,
                    drawer_face_outer_y,
                    opening_floor_z + drawer_face_bottom_gap,
                ),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=90.0,
                velocity=0.35,
                lower=0.0,
                upper=drawer_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet_body")

    for drawer_index in range(8):
        drawer = object_model.get_part(f"drawer_{drawer_index + 1}")
        slide = object_model.get_articulation(f"cabinet_to_drawer_{drawer_index + 1}")
        upper = slide.motion_limits.upper if slide.motion_limits is not None else None

        ctx.expect_within(
            drawer,
            cabinet,
            axes="xz",
            margin=0.012,
            name=f"drawer {drawer_index + 1} stays aligned in the cabinet opening",
        )
        ctx.expect_contact(
            drawer,
            cabinet,
            elem_a="left_runner",
            elem_b=f"drawer_{drawer_index + 1}_left_rail",
            name=f"drawer {drawer_index + 1} left guide rides on its rail",
        )
        ctx.expect_contact(
            drawer,
            cabinet,
            elem_a="right_runner",
            elem_b=f"drawer_{drawer_index + 1}_right_rail",
            name=f"drawer {drawer_index + 1} right guide rides on its rail",
        )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: upper}):
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="y",
                elem_a="left_runner",
                elem_b=f"drawer_{drawer_index + 1}_left_rail",
                min_overlap=0.36,
                name=f"drawer {drawer_index + 1} left guide retains insertion when open",
            )
            extended_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer {drawer_index + 1} extends forward along +Y",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] > rest_pos[1] + 0.20,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
