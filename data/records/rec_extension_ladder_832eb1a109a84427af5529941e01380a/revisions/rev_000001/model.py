from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_library_ladder")

    wall_paint = model.material("wall_paint", color=(0.90, 0.89, 0.85))
    floor_oak = model.material("floor_oak", color=(0.55, 0.42, 0.28))
    ladder_oak = model.material("ladder_oak", color=(0.62, 0.45, 0.24))
    dark_steel = model.material("dark_steel", color=(0.22, 0.23, 0.25))
    brass = model.material("brass", color=(0.72, 0.58, 0.28))
    black_rubber = model.material("black_rubber", color=(0.10, 0.10, 0.11))

    wall = model.part("wall_system")
    wall.visual(
        Box((2.60, 0.04, 2.55)),
        origin=Origin(xyz=(0.0, -0.02, 1.275)),
        material=wall_paint,
        name="wall_panel",
    )
    wall.visual(
        Box((2.60, 0.18, 0.24)),
        origin=Origin(xyz=(0.0, 0.09, 0.12)),
        material=floor_oak,
        name="bookcase_plinth",
    )
    wall.visual(
        Box((2.60, 0.72, 0.02)),
        origin=Origin(xyz=(0.0, 0.36, 0.01)),
        material=floor_oak,
        name="floor_platform",
    )
    wall.visual(
        Cylinder(radius=0.018, length=2.25),
        origin=Origin(xyz=(0.0, 0.055, 2.16), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="upper_rail",
    )
    for idx, x_pos in enumerate((-0.82, 0.0, 0.82), start=1):
        wall.visual(
            Box((0.06, 0.014, 0.16)),
            origin=Origin(xyz=(x_pos, 0.007, 2.16)),
            material=dark_steel,
            name=f"rail_back_plate_{idx}",
        )
        wall.visual(
            Box((0.06, 0.042, 0.03)),
            origin=Origin(xyz=(x_pos, 0.021, 2.138)),
            material=dark_steel,
            name=f"rail_support_arm_{idx}",
        )
    wall.visual(
        Box((2.25, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, 0.52, 0.006)),
        material=dark_steel,
        name="floor_track_base",
    )
    wall.visual(
        Box((2.25, 0.07, 0.022)),
        origin=Origin(xyz=(0.0, 0.52, 0.031)),
        material=dark_steel,
        name="floor_track_head",
    )

    ladder = model.part("ladder_frame")

    stile_center_y = 0.30
    stile_center_z = 1.12
    stile_roll = math.atan2(0.465, 2.136)
    stile_half_span_x = 0.19
    top_wheel_x = 0.2525
    top_wheel_y = 0.055
    top_wheel_z = 2.218
    bottom_wheel_y = 0.52
    bottom_wheel_z = 0.082

    def ladder_y_at_z(z_pos: float) -> float:
        z0 = top_wheel_z
        z1 = bottom_wheel_z
        y0 = top_wheel_y
        y1 = bottom_wheel_y
        return y0 + ((z0 - z_pos) / (z0 - z1)) * (y1 - y0)

    for side_name, x_pos in (("left", -stile_half_span_x), ("right", stile_half_span_x)):
        ladder.visual(
            Box((0.045, 0.055, 2.00)),
            origin=Origin(xyz=(x_pos, stile_center_y, stile_center_z), rpy=(stile_roll, 0.0, 0.0)),
            material=ladder_oak,
            name=f"{side_name}_stile",
        )

    for idx, z_pos in enumerate((0.36, 0.64, 0.92, 1.20, 1.48, 1.76, 2.00), start=1):
        ladder.visual(
            Box((0.40, 0.10, 0.03)),
            origin=Origin(xyz=(0.0, ladder_y_at_z(z_pos), z_pos)),
            material=ladder_oak,
            name=f"tread_{idx}",
        )

    ladder.visual(
        Box((0.44, 0.06, 0.035)),
        origin=Origin(xyz=(0.0, 0.49, 0.19)),
        material=ladder_oak,
        name="base_spreader",
    )
    ladder.visual(
        Box((0.44, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.14, 2.05)),
        material=ladder_oak,
        name="top_crossbar",
    )

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        plate_x = sign * 0.225

        ladder.visual(
            Box((0.025, 0.03, 0.27)),
            origin=Origin(xyz=(plate_x, 0.084, 2.165)),
            material=dark_steel,
            name=f"top_side_plate_{side_name}",
        )
        ladder.visual(
            Box((0.025, 0.09, 0.016)),
            origin=Origin(xyz=(plate_x, 0.042, 2.278)),
            material=dark_steel,
            name=f"top_hook_cap_{side_name}",
        )
        ladder.visual(
            Box((0.025, 0.016, 0.22)),
            origin=Origin(xyz=(plate_x, 0.005, 2.170)),
            material=dark_steel,
            name=f"top_hook_keeper_{side_name}",
        )
        ladder.visual(
            Box((0.025, 0.03, 0.18)),
            origin=Origin(xyz=(plate_x, 0.491, 0.170)),
            material=dark_steel,
            name=f"bottom_side_plate_{side_name}",
        )

    top_left_wheel = model.part("top_left_wheel")
    top_left_wheel.visual(
        Cylinder(radius=0.04, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="rim",
    )

    top_right_wheel = model.part("top_right_wheel")
    top_right_wheel.visual(
        Cylinder(radius=0.04, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="rim",
    )

    bottom_left_wheel = model.part("bottom_left_wheel")
    bottom_left_wheel.visual(
        Cylinder(radius=0.04, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="rim",
    )

    bottom_right_wheel = model.part("bottom_right_wheel")
    bottom_right_wheel.visual(
        Cylinder(radius=0.04, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="rim",
    )

    model.articulation(
        "ladder_travel",
        ArticulationType.PRISMATIC,
        parent=wall,
        child=ladder,
        origin=Origin(xyz=(-0.55, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=120.0, velocity=0.45),
    )

    wheel_limits = MotionLimits(effort=15.0, velocity=8.0)

    model.articulation(
        "ladder_to_top_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=ladder,
        child=top_left_wheel,
        origin=Origin(xyz=(-top_wheel_x, top_wheel_y, top_wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=wheel_limits,
    )
    model.articulation(
        "ladder_to_top_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=ladder,
        child=top_right_wheel,
        origin=Origin(xyz=(top_wheel_x, top_wheel_y, top_wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=wheel_limits,
    )
    model.articulation(
        "ladder_to_bottom_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=ladder,
        child=bottom_left_wheel,
        origin=Origin(xyz=(-top_wheel_x, bottom_wheel_y, bottom_wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=wheel_limits,
    )
    model.articulation(
        "ladder_to_bottom_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=ladder,
        child=bottom_right_wheel,
        origin=Origin(xyz=(top_wheel_x, bottom_wheel_y, bottom_wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=wheel_limits,
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

    wall = object_model.get_part("wall_system")
    ladder = object_model.get_part("ladder_frame")
    top_left_wheel = object_model.get_part("top_left_wheel")
    top_right_wheel = object_model.get_part("top_right_wheel")
    bottom_left_wheel = object_model.get_part("bottom_left_wheel")
    bottom_right_wheel = object_model.get_part("bottom_right_wheel")
    ladder_travel = object_model.get_articulation("ladder_travel")

    ctx.expect_gap(
        top_left_wheel,
        wall,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        negative_elem="upper_rail",
        name="top left wheel bears on the upper rail",
    )
    ctx.expect_gap(
        top_right_wheel,
        wall,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        negative_elem="upper_rail",
        name="top right wheel bears on the upper rail",
    )
    ctx.expect_overlap(
        top_left_wheel,
        wall,
        axes="xy",
        elem_b="upper_rail",
        min_overlap=0.02,
        name="top left wheel aligns over the rail footprint",
    )
    ctx.expect_overlap(
        top_right_wheel,
        wall,
        axes="xy",
        elem_b="upper_rail",
        min_overlap=0.02,
        name="top right wheel aligns over the rail footprint",
    )
    ctx.expect_gap(
        bottom_left_wheel,
        wall,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem="floor_track_head",
        name="bottom left wheel rides on the floor guide track",
    )
    ctx.expect_gap(
        bottom_right_wheel,
        wall,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem="floor_track_head",
        name="bottom right wheel rides on the floor guide track",
    )
    ctx.expect_overlap(
        bottom_left_wheel,
        wall,
        axes="xy",
        elem_b="floor_track_head",
        min_overlap=0.02,
        name="bottom left wheel stays over the floor track",
    )
    ctx.expect_overlap(
        bottom_right_wheel,
        wall,
        axes="xy",
        elem_b="floor_track_head",
        min_overlap=0.02,
        name="bottom right wheel stays over the floor track",
    )
    ctx.expect_origin_gap(
        bottom_left_wheel,
        top_left_wheel,
        axis="y",
        min_gap=0.40,
        name="ladder wheel contacts hold the stile frame at a lean",
    )

    rest_pos = ctx.part_world_position(ladder)
    with ctx.pose({ladder_travel: 1.10}):
        moved_pos = ctx.part_world_position(ladder)
        ctx.expect_gap(
            top_left_wheel,
            wall,
            axis="z",
            max_gap=0.001,
            max_penetration=0.001,
            negative_elem="upper_rail",
            name="top wheel stays seated on the rail at full travel",
        )
        ctx.expect_gap(
            bottom_left_wheel,
            wall,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            negative_elem="floor_track_head",
            name="bottom wheel stays seated on the floor track at full travel",
        )

    ctx.check(
        "ladder assembly travels sideways along the rail",
        rest_pos is not None and moved_pos is not None and moved_pos[0] > rest_pos[0] + 1.0,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
