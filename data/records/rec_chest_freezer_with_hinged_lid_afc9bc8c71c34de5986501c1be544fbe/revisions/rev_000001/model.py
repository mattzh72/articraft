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
    model = ArticulatedObject(name="commercial_display_chest_freezer")

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.96, 1.0))
    liner_white = model.material("liner_white", rgba=(0.88, 0.90, 0.92, 1.0))
    plinth_charcoal = model.material("plinth_charcoal", rgba=(0.18, 0.20, 0.22, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.74, 0.77, 0.80, 1.0))
    frame_aluminum = model.material("frame_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.24, 0.26, 0.28, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.66, 0.83, 0.91, 0.34))

    outer_len = 1.86
    outer_depth = 0.78
    plinth_len = 1.82
    plinth_depth = 0.74
    plinth_height = 0.08
    shell_height = 0.74
    wall_thickness = 0.045
    rim_height = 0.06

    inner_floor_len = 1.70
    inner_floor_depth = 0.57
    inner_floor_thickness = 0.028
    liner_height = 0.64
    liner_thickness = 0.012
    inner_floor_top = 0.128

    rim_z = plinth_height + shell_height + rim_height / 2.0
    shell_z = plinth_height + shell_height / 2.0
    liner_z = inner_floor_top + liner_height / 2.0

    body = model.part("body")
    body.visual(
        Box((plinth_len, plinth_depth, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height / 2.0)),
        material=plinth_charcoal,
        name="base_plinth",
    )
    body.visual(
        Box((outer_len, wall_thickness, shell_height)),
        origin=Origin(xyz=(0.0, outer_depth / 2.0 - wall_thickness / 2.0, shell_z)),
        material=body_white,
        name="front_shell",
    )
    body.visual(
        Box((outer_len, wall_thickness, shell_height)),
        origin=Origin(xyz=(0.0, -outer_depth / 2.0 + wall_thickness / 2.0, shell_z)),
        material=body_white,
        name="rear_shell",
    )
    body.visual(
        Box((wall_thickness, outer_depth - 2.0 * wall_thickness, shell_height)),
        origin=Origin(xyz=(-outer_len / 2.0 + wall_thickness / 2.0, 0.0, shell_z)),
        material=body_white,
        name="left_shell",
    )
    body.visual(
        Box((wall_thickness, outer_depth - 2.0 * wall_thickness, shell_height)),
        origin=Origin(xyz=(outer_len / 2.0 - wall_thickness / 2.0, 0.0, shell_z)),
        material=body_white,
        name="right_shell",
    )

    body.visual(
        Box((outer_len, 0.07, rim_height)),
        origin=Origin(xyz=(0.0, outer_depth / 2.0 - 0.035, rim_z)),
        material=rail_aluminum,
        name="front_rim",
    )
    body.visual(
        Box((outer_len, 0.07, rim_height)),
        origin=Origin(xyz=(0.0, -outer_depth / 2.0 + 0.035, rim_z)),
        material=rail_aluminum,
        name="rear_rim",
    )
    body.visual(
        Box((0.08, outer_depth, rim_height)),
        origin=Origin(xyz=(-outer_len / 2.0 + 0.04, 0.0, rim_z)),
        material=rail_aluminum,
        name="left_rim",
    )
    body.visual(
        Box((0.08, outer_depth, rim_height)),
        origin=Origin(xyz=(outer_len / 2.0 - 0.04, 0.0, rim_z)),
        material=rail_aluminum,
        name="right_rim",
    )

    body.visual(
        Box((inner_floor_len, inner_floor_depth, inner_floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, inner_floor_top - inner_floor_thickness / 2.0)),
        material=liner_white,
        name="liner_floor",
    )
    body.visual(
        Box((inner_floor_len, liner_thickness, liner_height)),
        origin=Origin(xyz=(0.0, inner_floor_depth / 2.0 - liner_thickness / 2.0, liner_z)),
        material=liner_white,
        name="liner_front",
    )
    body.visual(
        Box((inner_floor_len, liner_thickness, liner_height)),
        origin=Origin(xyz=(0.0, -inner_floor_depth / 2.0 + liner_thickness / 2.0, liner_z)),
        material=liner_white,
        name="liner_rear",
    )
    body.visual(
        Box((liner_thickness, inner_floor_depth, liner_height)),
        origin=Origin(xyz=(-inner_floor_len / 2.0 + liner_thickness / 2.0, 0.0, liner_z)),
        material=liner_white,
        name="liner_left",
    )
    body.visual(
        Box((liner_thickness, inner_floor_depth, liner_height)),
        origin=Origin(xyz=(inner_floor_len / 2.0 - liner_thickness / 2.0, 0.0, liner_z)),
        material=liner_white,
        name="liner_right",
    )

    rail_length = 1.76
    rail_depth = 0.018
    rail_y = 0.306
    lower_rail_top = 0.888
    upper_rail_top = 0.888
    lower_rail_height = 0.010
    upper_rail_height = 0.010

    body.visual(
        Box((rail_length, rail_depth, upper_rail_height)),
        origin=Origin(xyz=(0.0, rail_y, upper_rail_top - upper_rail_height / 2.0)),
        material=rail_aluminum,
        name="left_front_rail",
    )
    body.visual(
        Box((rail_length, rail_depth, upper_rail_height)),
        origin=Origin(xyz=(0.0, -rail_y, upper_rail_top - upper_rail_height / 2.0)),
        material=rail_aluminum,
        name="left_rear_rail",
    )
    body.visual(
        Box((rail_length, rail_depth, lower_rail_height)),
        origin=Origin(xyz=(0.0, rail_y, lower_rail_top - lower_rail_height / 2.0)),
        material=trim_dark,
        name="right_front_rail",
    )
    body.visual(
        Box((rail_length, rail_depth, lower_rail_height)),
        origin=Origin(xyz=(0.0, -rail_y, lower_rail_top - lower_rail_height / 2.0)),
        material=trim_dark,
        name="right_rear_rail",
    )

    body.visual(
        Box((0.12, 0.065, 0.085)),
        origin=Origin(xyz=(0.79, 0.272, 0.78)),
        material=trim_dark,
        name="controller_housing",
    )
    body.visual(
        Box((0.06, 0.004, 0.03)),
        origin=Origin(xyz=(0.79, 0.303, 0.79)),
        material=glass_tint,
        name="controller_window",
    )
    body.inertial = Inertial.from_geometry(
        Box((outer_len, outer_depth, plinth_height + shell_height + rim_height)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, (plinth_height + shell_height + rim_height) / 2.0)),
    )

    panel_len = 0.86
    panel_depth = 0.66
    frame_bar = 0.032
    outer_stile = 0.028
    inner_stile = 0.040
    frame_thickness = 0.018
    runner_len = 0.80
    runner_depth = 0.018
    runner_height = 0.006
    runner_y = 0.306
    glass_len = panel_len - outer_stile - inner_stile
    glass_depth = panel_depth - 2.0 * frame_bar
    glass_thickness = 0.006

    def add_lid_panel(part_name: str, inner_sign: float) -> object:
        panel = model.part(part_name)
        panel.visual(
            Box((panel_len, frame_bar, frame_thickness)),
            origin=Origin(xyz=(0.0, panel_depth / 2.0 - frame_bar / 2.0, frame_thickness / 2.0)),
            material=frame_aluminum,
            name="front_frame",
        )
        panel.visual(
            Box((panel_len, frame_bar, frame_thickness)),
            origin=Origin(xyz=(0.0, -panel_depth / 2.0 + frame_bar / 2.0, frame_thickness / 2.0)),
            material=frame_aluminum,
            name="rear_frame",
        )
        panel.visual(
            Box((outer_stile, panel_depth - 2.0 * frame_bar, frame_thickness)),
            origin=Origin(
                xyz=(
                    -inner_sign * (panel_len / 2.0 - outer_stile / 2.0),
                    0.0,
                    frame_thickness / 2.0,
                )
            ),
            material=frame_aluminum,
            name="outer_stile",
        )
        panel.visual(
            Box((inner_stile, panel_depth, frame_thickness + 0.004)),
            origin=Origin(
                xyz=(
                    inner_sign * (panel_len / 2.0 - inner_stile / 2.0),
                    0.0,
                    (frame_thickness + 0.004) / 2.0,
                )
            ),
            material=frame_aluminum,
            name="meeting_stile",
        )
        panel.visual(
            Box((glass_len, glass_depth, glass_thickness)),
            origin=Origin(
                xyz=(-inner_sign * 0.006, 0.0, frame_thickness / 2.0 + 0.002),
            ),
            material=glass_tint,
            name="glass_panel",
        )
        panel.visual(
            Box((0.018, 0.24, 0.010)),
            origin=Origin(
                xyz=(
                    inner_sign * (panel_len / 2.0 - 0.012),
                    0.0,
                    frame_thickness + 0.005,
                )
            ),
            material=trim_dark,
            name="handle_grip",
        )
        panel.visual(
            Box((runner_len, runner_depth, runner_height)),
            origin=Origin(xyz=(0.0, runner_y, -runner_height / 2.0)),
            material=trim_dark,
            name="front_runner",
        )
        panel.visual(
            Box((runner_len, runner_depth, runner_height)),
            origin=Origin(xyz=(0.0, -runner_y, -runner_height / 2.0)),
            material=trim_dark,
            name="rear_runner",
        )
        panel.inertial = Inertial.from_geometry(
            Box((panel_len, panel_depth, frame_thickness + 0.02)),
            mass=6.5,
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
        )
        return panel

    left_lid = add_lid_panel("left_lid", inner_sign=1.0)
    right_lid = add_lid_panel("right_lid", inner_sign=-1.0)

    model.articulation(
        "body_to_left_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_lid,
        origin=Origin(xyz=(-0.435, 0.0, upper_rail_top + runner_height)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.35,
            lower=0.0,
            upper=0.12,
        ),
    )
    model.articulation(
        "body_to_right_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_lid,
        origin=Origin(xyz=(0.435, 0.0, lower_rail_top + runner_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.35,
            lower=0.0,
            upper=0.12,
        ),
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
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")
    left_slide = object_model.get_articulation("body_to_left_lid")
    right_slide = object_model.get_articulation("body_to_right_lid")

    left_front_runner = left_lid.get_visual("front_runner")
    left_rear_runner = left_lid.get_visual("rear_runner")
    right_front_runner = right_lid.get_visual("front_runner")
    right_rear_runner = right_lid.get_visual("rear_runner")

    left_front_rail = body.get_visual("left_front_rail")
    left_rear_rail = body.get_visual("left_rear_rail")
    right_front_rail = body.get_visual("right_front_rail")
    right_rear_rail = body.get_visual("right_rear_rail")

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_contact(
            left_lid,
            body,
            elem_a=left_front_runner,
            elem_b=left_front_rail,
            name="left lid front runner sits on its rail",
        )
        ctx.expect_contact(
            left_lid,
            body,
            elem_a=left_rear_runner,
            elem_b=left_rear_rail,
            name="left lid rear runner sits on its rail",
        )
        ctx.expect_contact(
            right_lid,
            body,
            elem_a=right_front_runner,
            elem_b=right_front_rail,
            name="right lid front runner sits on its rail",
        )
        ctx.expect_contact(
            right_lid,
            body,
            elem_a=right_rear_runner,
            elem_b=right_rear_rail,
            name="right lid rear runner sits on its rail",
        )
        ctx.expect_gap(
            right_lid,
            left_lid,
            axis="x",
            min_gap=0.0,
            max_gap=0.015,
            name="closed lids meet with only a narrow center seam",
        )
        ctx.expect_overlap(
            left_lid,
            right_lid,
            axes="y",
            min_overlap=0.60,
            name="closed lids cover the full freezer opening depth",
        )

    left_rest = ctx.part_world_position(left_lid)
    right_rest = ctx.part_world_position(right_lid)
    with ctx.pose({left_slide: 0.12, right_slide: 0.12}):
        ctx.expect_overlap(
            left_lid,
            body,
            axes="x",
            elem_a=left_front_runner,
            elem_b=left_front_rail,
            min_overlap=0.70,
            name="left lid retains substantial rail engagement when open",
        )
        ctx.expect_overlap(
            right_lid,
            body,
            axes="x",
            elem_a=right_front_runner,
            elem_b=right_front_rail,
            min_overlap=0.70,
            name="right lid retains substantial rail engagement when open",
        )
        left_open = ctx.part_world_position(left_lid)
        right_open = ctx.part_world_position(right_lid)

    ctx.check(
        "left lid slides leftward",
        left_rest is not None and left_open is not None and left_open[0] < left_rest[0] - 0.10,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "right lid slides rightward",
        right_rest is not None and right_open is not None and right_open[0] > right_rest[0] + 0.10,
        details=f"rest={right_rest}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
