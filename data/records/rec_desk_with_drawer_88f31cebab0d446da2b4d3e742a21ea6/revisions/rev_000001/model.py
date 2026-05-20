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


def _add_rect_bar_pull(
    part,
    *,
    bar_length: float,
    handle_z: float,
    material,
    name_prefix: str,
    post_offset: float | None = None,
    post_size: tuple[float, float, float] = (0.014, 0.024, 0.014),
    bar_size: tuple[float, float] = (0.012, 0.012),
) -> None:
    offset = post_offset if post_offset is not None else bar_length * 0.33
    for side_name, x_pos in (("left", -offset), ("right", offset)):
        part.visual(
            Box(post_size),
            origin=Origin(xyz=(x_pos, -0.022, handle_z)),
            material=material,
            name=f"{name_prefix}_{side_name}_post",
        )
    part.visual(
        Box((bar_length, bar_size[0], bar_size[1])),
        origin=Origin(xyz=(0.0, -0.034, handle_z)),
        material=material,
        name=f"{name_prefix}_bar",
    )


def _add_drawer(
    part,
    *,
    front_width: float,
    front_height: float,
    shell_width: float,
    shell_height: float,
    shell_depth: float,
    runner_width: float,
    runner_height: float,
    runner_length: float,
    runner_z: float,
    wood_material,
    runner_material,
    handle_material,
    handle_bar_length: float,
    handle_z: float,
    handle_post_offset: float | None = None,
) -> None:
    front_thickness = 0.020
    panel_thickness = 0.012
    bottom_thickness = 0.012

    part.visual(
        Box((front_width, front_thickness, front_height)),
        material=wood_material,
        name="front_panel",
    )

    side_y = (front_thickness * 0.5) + (shell_depth * 0.5)
    side_x = (shell_width * 0.5) - (panel_thickness * 0.5)
    for side_name, x_pos in (("left", -side_x), ("right", side_x)):
        part.visual(
            Box((panel_thickness, shell_depth, shell_height)),
            origin=Origin(xyz=(x_pos, side_y, 0.0)),
            material=wood_material,
            name=f"{side_name}_side",
        )

    inner_width = shell_width - (2.0 * panel_thickness)
    part.visual(
        Box((inner_width, panel_thickness, shell_height)),
        origin=Origin(
            xyz=(
                0.0,
                (front_thickness * 0.5) + shell_depth - (panel_thickness * 0.5),
                0.0,
            )
        ),
        material=wood_material,
        name="back_panel",
    )
    part.visual(
        Box((inner_width, shell_depth, bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                side_y,
                -(shell_height * 0.5) + (bottom_thickness * 0.5),
            )
        ),
        material=wood_material,
        name="bottom_panel",
    )

    runner_x = (shell_width * 0.5) + (runner_width * 0.5)
    runner_y = (front_thickness * 0.5) + (runner_length * 0.5)
    for side_name, x_pos in (("left", -runner_x), ("right", runner_x)):
        part.visual(
            Box((runner_width, runner_length, runner_height)),
            origin=Origin(xyz=(x_pos, runner_y, runner_z)),
            material=runner_material,
            name=f"{side_name}_runner",
        )

    _add_rect_bar_pull(
        part,
        bar_length=handle_bar_length,
        handle_z=handle_z,
        material=handle_material,
        name_prefix="pull",
        post_offset=handle_post_offset,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="library_writing_table")

    body_wood = model.material("body_wood", rgba=(0.30, 0.20, 0.12, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.34, 0.23, 0.14, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.22, 0.23, 0.24, 1.0))
    brass = model.material("aged_brass", rgba=(0.76, 0.66, 0.34, 1.0))

    desk_width = 1.62
    desk_depth = 0.82
    body_width = 1.54
    body_depth = 0.72
    pedestal_width = 0.40
    knee_width = body_width - (2.0 * pedestal_width)
    desk_height = 0.77

    top_upper_thickness = 0.030
    top_lower_thickness = 0.015
    body_top_z = desk_height - top_upper_thickness - top_lower_thickness

    side_panel_t = 0.020
    back_panel_t = 0.014
    plinth_h = 0.055
    upper_rail_h = 0.115

    left_pedestal_x = -((knee_width * 0.5) + (pedestal_width * 0.5))
    right_pedestal_x = -left_pedestal_x
    body_front_y = -(body_depth * 0.5)
    body_back_y = body_depth * 0.5

    desk_body = model.part("desk_body")
    desk_body.visual(
        Box((desk_width, desk_depth, top_upper_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                body_top_z + top_lower_thickness + (top_upper_thickness * 0.5),
            )
        ),
        material=body_wood,
        name="top_cap",
    )
    desk_body.visual(
        Box((1.56, 0.76, top_lower_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_top_z + (top_lower_thickness * 0.5))),
        material=body_wood,
        name="top_subframe",
    )

    for side_name, x_pos in (
        ("left_outer_side", left_pedestal_x - (pedestal_width * 0.5) + (side_panel_t * 0.5)),
        ("left_inner_side", left_pedestal_x + (pedestal_width * 0.5) - (side_panel_t * 0.5)),
        ("right_inner_side", right_pedestal_x - (pedestal_width * 0.5) + (side_panel_t * 0.5)),
        ("right_outer_side", right_pedestal_x + (pedestal_width * 0.5) - (side_panel_t * 0.5)),
    ):
        desk_body.visual(
            Box((side_panel_t, body_depth, body_top_z)),
            origin=Origin(xyz=(x_pos, 0.0, body_top_z * 0.5)),
            material=body_wood,
            name=side_name,
        )

    for side_name, x_pos in (("left", left_pedestal_x), ("right", right_pedestal_x)):
        desk_body.visual(
            Box((pedestal_width - (2.0 * side_panel_t), back_panel_t, body_top_z)),
            origin=Origin(
                xyz=(x_pos, body_back_y - (back_panel_t * 0.5), body_top_z * 0.5)
            ),
            material=body_wood,
            name=f"{side_name}_back_panel",
        )
        desk_body.visual(
            Box((pedestal_width, 0.64, plinth_h)),
            origin=Origin(xyz=(x_pos, 0.0, plinth_h * 0.5)),
            material=body_wood,
            name=f"{side_name}_plinth",
        )
        desk_body.visual(
            Box((pedestal_width - (2.0 * side_panel_t), 0.020, upper_rail_h)),
            origin=Origin(
                xyz=(
                    x_pos,
                    body_front_y + 0.010,
                    body_top_z - (upper_rail_h * 0.5),
                )
            ),
            material=body_wood,
            name=f"{side_name}_top_rail",
        )

    desk_body.visual(
        Box((knee_width, 0.020, upper_rail_h)),
        origin=Origin(
            xyz=(
                0.0,
                body_back_y - 0.010,
                body_top_z - (upper_rail_h * 0.5),
            )
        ),
        material=body_wood,
        name="rear_knee_apron",
    )

    file_rail_length = 0.50
    file_rail_height = 0.040
    file_rail_z = 0.184
    file_rail_y = -0.09
    rail_thickness = 0.015

    desk_body.visual(
        Box((rail_thickness, file_rail_length, file_rail_height)),
        origin=Origin(xyz=(-0.7425, file_rail_y, file_rail_z)),
        material=rail_metal,
        name="left_file_outer_rail",
    )
    desk_body.visual(
        Box((rail_thickness, file_rail_length, file_rail_height)),
        origin=Origin(xyz=(-0.3975, file_rail_y, file_rail_z)),
        material=rail_metal,
        name="left_file_inner_rail",
    )
    desk_body.visual(
        Box((rail_thickness, file_rail_length, file_rail_height)),
        origin=Origin(xyz=(0.3975, file_rail_y, file_rail_z)),
        material=rail_metal,
        name="right_file_inner_rail",
    )
    desk_body.visual(
        Box((rail_thickness, file_rail_length, file_rail_height)),
        origin=Origin(xyz=(0.7425, file_rail_y, file_rail_z)),
        material=rail_metal,
        name="right_file_outer_rail",
    )

    desk_body.visual(
        Box((0.030, 0.460, 0.030)),
        origin=Origin(xyz=(-0.355, -0.110, 0.645)),
        material=rail_metal,
        name="center_left_rail",
    )
    desk_body.visual(
        Box((0.030, 0.460, 0.030)),
        origin=Origin(xyz=(0.355, -0.110, 0.645)),
        material=rail_metal,
        name="center_right_rail",
    )
    desk_body.inertial = Inertial.from_geometry(
        Box((desk_width, desk_depth, desk_height)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, desk_height * 0.5)),
    )

    center_drawer = model.part("center_drawer")
    _add_drawer(
        center_drawer,
        front_width=0.724,
        front_height=0.092,
        shell_width=0.640,
        shell_height=0.070,
        shell_depth=0.500,
        runner_width=0.020,
        runner_height=0.030,
        runner_length=0.440,
        runner_z=-0.015,
        wood_material=drawer_wood,
        runner_material=rail_metal,
        handle_material=brass,
        handle_bar_length=0.120,
        handle_z=0.000,
        handle_post_offset=0.038,
    )
    center_drawer.inertial = Inertial.from_geometry(
        Box((0.724, 0.520, 0.092)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.250, 0.0)),
    )

    left_file_drawer = model.part("left_file_drawer")
    _add_drawer(
        left_file_drawer,
        front_width=0.344,
        front_height=0.538,
        shell_width=0.300,
        shell_height=0.480,
        shell_depth=0.580,
        runner_width=0.015,
        runner_height=0.040,
        runner_length=0.480,
        runner_z=-0.140,
        wood_material=drawer_wood,
        runner_material=rail_metal,
        handle_material=brass,
        handle_bar_length=0.180,
        handle_z=0.020,
        handle_post_offset=0.060,
    )
    left_file_drawer.inertial = Inertial.from_geometry(
        Box((0.344, 0.600, 0.538)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.290, 0.0)),
    )

    right_file_drawer = model.part("right_file_drawer")
    _add_drawer(
        right_file_drawer,
        front_width=0.344,
        front_height=0.538,
        shell_width=0.300,
        shell_height=0.480,
        shell_depth=0.580,
        runner_width=0.015,
        runner_height=0.040,
        runner_length=0.480,
        runner_z=-0.140,
        wood_material=drawer_wood,
        runner_material=rail_metal,
        handle_material=brass,
        handle_bar_length=0.180,
        handle_z=0.020,
        handle_post_offset=0.060,
    )
    right_file_drawer.inertial = Inertial.from_geometry(
        Box((0.344, 0.600, 0.538)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.290, 0.0)),
    )

    model.articulation(
        "desk_to_center_drawer",
        ArticulationType.PRISMATIC,
        parent=desk_body,
        child=center_drawer,
        origin=Origin(xyz=(0.0, -0.370, 0.661)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=0.340,
        ),
    )
    model.articulation(
        "desk_to_left_file_drawer",
        ArticulationType.PRISMATIC,
        parent=desk_body,
        child=left_file_drawer,
        origin=Origin(xyz=(left_pedestal_x, -0.370, 0.324)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.25,
            lower=0.0,
            upper=0.340,
        ),
    )
    model.articulation(
        "desk_to_right_file_drawer",
        ArticulationType.PRISMATIC,
        parent=desk_body,
        child=right_file_drawer,
        origin=Origin(xyz=(right_pedestal_x, -0.370, 0.324)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.25,
            lower=0.0,
            upper=0.340,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    desk_body = object_model.get_part("desk_body")
    center_drawer = object_model.get_part("center_drawer")
    left_file_drawer = object_model.get_part("left_file_drawer")
    right_file_drawer = object_model.get_part("right_file_drawer")

    center_slide = object_model.get_articulation("desk_to_center_drawer")
    left_slide = object_model.get_articulation("desk_to_left_file_drawer")
    right_slide = object_model.get_articulation("desk_to_right_file_drawer")

    center_left_rail = desk_body.get_visual("center_left_rail")
    center_right_rail = desk_body.get_visual("center_right_rail")
    left_outer_rail = desk_body.get_visual("left_file_outer_rail")
    left_inner_rail = desk_body.get_visual("left_file_inner_rail")
    right_inner_rail = desk_body.get_visual("right_file_inner_rail")
    right_outer_rail = desk_body.get_visual("right_file_outer_rail")

    center_left_runner = center_drawer.get_visual("left_runner")
    center_right_runner = center_drawer.get_visual("right_runner")
    left_left_runner = left_file_drawer.get_visual("left_runner")
    left_right_runner = left_file_drawer.get_visual("right_runner")
    right_left_runner = right_file_drawer.get_visual("left_runner")
    right_right_runner = right_file_drawer.get_visual("right_runner")

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        desk_body,
        center_drawer,
        elem_a=center_left_rail,
        elem_b=center_left_runner,
        name="center_drawer_left_guide_contact_rest",
    )
    ctx.expect_contact(
        desk_body,
        center_drawer,
        elem_a=center_right_rail,
        elem_b=center_right_runner,
        name="center_drawer_right_guide_contact_rest",
    )
    ctx.expect_contact(
        desk_body,
        left_file_drawer,
        elem_a=left_outer_rail,
        elem_b=left_left_runner,
        name="left_file_drawer_outer_guide_contact_rest",
    )
    ctx.expect_contact(
        desk_body,
        left_file_drawer,
        elem_a=left_inner_rail,
        elem_b=left_right_runner,
        name="left_file_drawer_inner_guide_contact_rest",
    )
    ctx.expect_contact(
        desk_body,
        right_file_drawer,
        elem_a=right_inner_rail,
        elem_b=right_left_runner,
        name="right_file_drawer_inner_guide_contact_rest",
    )
    ctx.expect_contact(
        desk_body,
        right_file_drawer,
        elem_a=right_outer_rail,
        elem_b=right_right_runner,
        name="right_file_drawer_outer_guide_contact_rest",
    )

    center_closed = ctx.part_world_position(center_drawer)
    left_closed = ctx.part_world_position(left_file_drawer)
    right_closed = ctx.part_world_position(right_file_drawer)

    with ctx.pose({center_slide: 0.260}):
        center_open = ctx.part_world_position(center_drawer)
        ctx.check(
            "center_drawer_opens_forward",
            center_open is not None
            and center_closed is not None
            and center_open[1] < center_closed[1] - 0.250,
            details=f"closed={center_closed}, open={center_open}",
        )
        ctx.expect_contact(
            desk_body,
            center_drawer,
            elem_a=center_left_rail,
            elem_b=center_left_runner,
            name="center_drawer_left_guide_contact_open",
        )
        ctx.expect_contact(
            desk_body,
            center_drawer,
            elem_a=center_right_rail,
            elem_b=center_right_runner,
            name="center_drawer_right_guide_contact_open",
        )

    with ctx.pose({left_slide: 0.280}):
        left_open = ctx.part_world_position(left_file_drawer)
        ctx.check(
            "left_file_drawer_opens_forward",
            left_open is not None
            and left_closed is not None
            and left_open[1] < left_closed[1] - 0.270,
            details=f"closed={left_closed}, open={left_open}",
        )
        ctx.expect_contact(
            desk_body,
            left_file_drawer,
            elem_a=left_outer_rail,
            elem_b=left_left_runner,
            name="left_file_drawer_outer_guide_contact_open",
        )
        ctx.expect_contact(
            desk_body,
            left_file_drawer,
            elem_a=left_inner_rail,
            elem_b=left_right_runner,
            name="left_file_drawer_inner_guide_contact_open",
        )

    with ctx.pose({right_slide: 0.280}):
        right_open = ctx.part_world_position(right_file_drawer)
        ctx.check(
            "right_file_drawer_opens_forward",
            right_open is not None
            and right_closed is not None
            and right_open[1] < right_closed[1] - 0.270,
            details=f"closed={right_closed}, open={right_open}",
        )
        ctx.expect_contact(
            desk_body,
            right_file_drawer,
            elem_a=right_inner_rail,
            elem_b=right_left_runner,
            name="right_file_drawer_inner_guide_contact_open",
        )
        ctx.expect_contact(
            desk_body,
            right_file_drawer,
            elem_a=right_outer_rail,
            elem_b=right_right_runner,
            name="right_file_drawer_outer_guide_contact_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
