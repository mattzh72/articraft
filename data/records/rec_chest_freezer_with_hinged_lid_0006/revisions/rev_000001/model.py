from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
try:
    os.chdir("/")
except OSError:
    pass

__file__ = "/model.py"

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_chest_freezer", assets=ASSETS)

    cabinet_white = model.material("cabinet_white", rgba=(0.91, 0.93, 0.95, 1.0))
    liner_white = model.material("liner_white", rgba=(0.97, 0.98, 0.99, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.43, 0.47, 0.50, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.84, 0.92, 0.33))
    handle_dark = model.material("handle_dark", rgba=(0.18, 0.20, 0.22, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((1.38, 0.64, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=trim_grey,
        name="plinth",
    )
    cabinet.visual(
        Box((1.34, 0.58, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=cabinet_white,
        name="insulation_base",
    )
    cabinet.visual(
        Box((1.46, 0.04, 0.66)),
        origin=Origin(xyz=(0.0, 0.34, 0.41)),
        material=cabinet_white,
        name="front_shell",
    )
    cabinet.visual(
        Box((1.46, 0.04, 0.66)),
        origin=Origin(xyz=(0.0, -0.34, 0.41)),
        material=cabinet_white,
        name="rear_shell",
    )
    cabinet.visual(
        Box((0.04, 0.64, 0.66)),
        origin=Origin(xyz=(-0.71, 0.0, 0.41)),
        material=cabinet_white,
        name="left_shell",
    )
    cabinet.visual(
        Box((0.04, 0.64, 0.66)),
        origin=Origin(xyz=(0.71, 0.0, 0.41)),
        material=cabinet_white,
        name="right_shell",
    )
    cabinet.visual(
        Box((1.28, 0.52, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=liner_white,
        name="liner_floor",
    )
    cabinet.visual(
        Box((1.28, 0.03, 0.56)),
        origin=Origin(xyz=(0.0, 0.245, 0.46)),
        material=liner_white,
        name="liner_front",
    )
    cabinet.visual(
        Box((1.28, 0.03, 0.56)),
        origin=Origin(xyz=(0.0, -0.245, 0.46)),
        material=liner_white,
        name="liner_rear",
    )
    cabinet.visual(
        Box((0.03, 0.52, 0.56)),
        origin=Origin(xyz=(-0.625, 0.0, 0.46)),
        material=liner_white,
        name="liner_left",
    )
    cabinet.visual(
        Box((0.03, 0.52, 0.56)),
        origin=Origin(xyz=(0.625, 0.0, 0.46)),
        material=liner_white,
        name="liner_right",
    )
    cabinet.visual(
        Box((1.46, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, 0.31, 0.76)),
        material=trim_grey,
        name="top_rim_front",
    )
    cabinet.visual(
        Box((1.46, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, -0.31, 0.76)),
        material=trim_grey,
        name="top_rim_rear",
    )
    cabinet.visual(
        Box((0.10, 0.52, 0.04)),
        origin=Origin(xyz=(-0.68, 0.0, 0.76)),
        material=trim_grey,
        name="top_rim_left",
    )
    cabinet.visual(
        Box((0.10, 0.52, 0.04)),
        origin=Origin(xyz=(0.68, 0.0, 0.76)),
        material=trim_grey,
        name="top_rim_right",
    )
    cabinet.visual(
        Box((1.30, 0.035, 0.014)),
        origin=Origin(xyz=(0.0, 0.287, 0.787)),
        material=rail_aluminum,
        name="front_rail",
    )
    cabinet.visual(
        Box((1.30, 0.035, 0.014)),
        origin=Origin(xyz=(0.0, -0.287, 0.787)),
        material=rail_aluminum,
        name="rear_rail",
    )
    cabinet.visual(
        Box((1.30, 0.018, 0.10)),
        origin=Origin(xyz=(0.0, 0.348, 0.24)),
        material=trim_grey,
        name="front_bumper",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((1.46, 0.72, 0.80)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
    )

    lid_panel = model.part("lid_panel")
    lid_panel.visual(
        Box((0.80, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.287, 0.004)),
        material=rail_aluminum,
        name="front_runner",
    )
    lid_panel.visual(
        Box((0.80, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, -0.287, 0.004)),
        material=rail_aluminum,
        name="rear_runner",
    )
    lid_panel.visual(
        Box((0.84, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, 0.287, 0.016)),
        material=rail_aluminum,
        name="front_frame",
    )
    lid_panel.visual(
        Box((0.84, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, -0.287, 0.016)),
        material=rail_aluminum,
        name="rear_frame",
    )
    lid_panel.visual(
        Box((0.030, 0.572, 0.016)),
        origin=Origin(xyz=(-0.405, 0.0, 0.016)),
        material=rail_aluminum,
        name="left_frame",
    )
    lid_panel.visual(
        Box((0.030, 0.572, 0.016)),
        origin=Origin(xyz=(0.405, 0.0, 0.016)),
        material=rail_aluminum,
        name="right_frame",
    )
    lid_panel.visual(
        Box((0.78, 0.54, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=glass,
        name="glass_panel",
    )
    lid_panel.visual(
        Box((0.20, 0.016, 0.016)),
        origin=Origin(xyz=(0.18, 0.307, 0.018)),
        material=handle_dark,
        name="handle",
    )
    lid_panel.inertial = Inertial.from_geometry(
        Box((0.84, 0.60, 0.03)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_panel,
        origin=Origin(xyz=(-0.19, 0.0, 0.794)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.45, lower=0.0, upper=0.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid_panel = object_model.get_part("lid_panel")
    lid_slide = object_model.get_articulation("lid_slide")
    front_shell = cabinet.get_visual("front_shell")
    right_shell = cabinet.get_visual("right_shell")
    liner_front = cabinet.get_visual("liner_front")
    liner_right = cabinet.get_visual("liner_right")
    liner_floor = cabinet.get_visual("liner_floor")
    top_rim_front = cabinet.get_visual("top_rim_front")
    front_rail = cabinet.get_visual("front_rail")
    rear_rail = cabinet.get_visual("rear_rail")
    front_runner = lid_panel.get_visual("front_runner")
    rear_runner = lid_panel.get_visual("rear_runner")
    glass_panel = lid_panel.get_visual("glass_panel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="y",
        positive_elem=front_shell,
        negative_elem=liner_front,
        min_gap=0.05,
        max_gap=0.07,
        name="insulated_front_wall_thickness",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="x",
        positive_elem=right_shell,
        negative_elem=liner_right,
        min_gap=0.049,
        max_gap=0.07,
        name="insulated_side_wall_thickness",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        positive_elem=top_rim_front,
        negative_elem=liner_floor,
        min_gap=0.55,
        name="deep_merchandising_cavity",
    )
    ctx.expect_contact(
        lid_panel,
        cabinet,
        elem_a=front_runner,
        elem_b=front_rail,
        name="front_runner_seated_on_rail",
    )
    ctx.expect_contact(
        lid_panel,
        cabinet,
        elem_a=rear_runner,
        elem_b=rear_rail,
        name="rear_runner_seated_on_rail",
    )
    ctx.expect_overlap(
        lid_panel,
        cabinet,
        axes="x",
        elem_a=front_runner,
        elem_b=front_rail,
        min_overlap=0.79,
        name="front_runner_guided_by_front_rail",
    )
    ctx.expect_overlap(
        lid_panel,
        cabinet,
        axes="x",
        elem_a=rear_runner,
        elem_b=rear_rail,
        min_overlap=0.79,
        name="rear_runner_guided_by_rear_rail",
    )
    ctx.expect_overlap(
        lid_panel,
        cabinet,
        axes="xy",
        elem_a=glass_panel,
        min_overlap=0.30,
        name="glass_reads_as_top_cover",
    )

    rest_position = ctx.part_world_position(lid_panel)
    ctx.check("lid_panel_rest_position_resolved", rest_position is not None, "lid_panel world pose unavailable")

    limits = lid_slide.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({lid_slide: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="lid_slide_lower_no_floating")

        with ctx.pose({lid_slide: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="lid_slide_upper_no_floating")
            ctx.expect_contact(
                lid_panel,
                cabinet,
                elem_a=front_runner,
                elem_b=front_rail,
                name="front_runner_stays_seated_open",
            )
            ctx.expect_contact(
                lid_panel,
                cabinet,
                elem_a=rear_runner,
                elem_b=rear_rail,
                name="rear_runner_stays_seated_open",
            )
            ctx.expect_overlap(
                lid_panel,
                cabinet,
                axes="x",
                elem_a=front_runner,
                elem_b=front_rail,
                min_overlap=0.79,
                name="front_runner_keeps_rail_overlap_open",
            )
            ctx.expect_overlap(
                lid_panel,
                cabinet,
                axes="x",
                elem_a=rear_runner,
                elem_b=rear_rail,
                min_overlap=0.79,
                name="rear_runner_keeps_rail_overlap_open",
            )

            open_position = ctx.part_world_position(lid_panel)
            moved = open_position is not None and rest_position is not None
            ctx.check(
                "lid_panel_open_position_resolved",
                moved,
                "lid_panel world pose unavailable at open position",
            )
            if moved:
                dx = open_position[0] - rest_position[0]
                dy = abs(open_position[1] - rest_position[1])
                dz = abs(open_position[2] - rest_position[2])
                ctx.check(
                    "lid_panel_moves_along_prismatic_x",
                    0.39 <= dx <= 0.41 and dy <= 1e-6 and dz <= 1e-6,
                    f"expected ~0.40 m x-only travel, got dx={dx:.4f}, dy={dy:.6f}, dz={dz:.6f}",
                )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
