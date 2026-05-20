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
    model = ArticulatedObject(name="single_sash_sliding_window")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.60, 0.62, 0.66, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.84, 0.92, 0.35))
    gasket = model.material("gasket", rgba=(0.10, 0.10, 0.10, 1.0))

    window_width = 1.20
    window_height = 1.00
    frame_depth = 0.10
    outer_rail = 0.06
    clear_width = window_width - 2.0 * outer_rail
    clear_height = window_height - 2.0 * outer_rail

    fixed_panel_width = 0.62
    sliding_panel_width = 0.62
    panel_height = 0.82
    sash_depth = 0.024
    sash_rail = 0.045
    glass_capture = 0.012

    fixed_track_y = -0.024
    sliding_track_y = 0.024
    travel = 0.39

    def add_sash(
        part,
        *,
        width: float,
        height: float,
        depth: float,
        rail: float,
        frame_material,
        glass_material,
        prefix: str,
    ) -> None:
        part.visual(
            Box((rail, depth, height)),
            origin=Origin(xyz=(-(width - rail) / 2.0, 0.0, 0.0)),
            material=frame_material,
            name=f"{prefix}_left_stile",
        )
        part.visual(
            Box((rail, depth, height)),
            origin=Origin(xyz=((width - rail) / 2.0, 0.0, 0.0)),
            material=frame_material,
            name=f"{prefix}_right_stile",
        )
        part.visual(
            Box((width, depth, rail)),
            origin=Origin(xyz=(0.0, 0.0, (height - rail) / 2.0)),
            material=frame_material,
            name=f"{prefix}_top_rail",
        )
        part.visual(
            Box((width, depth, rail)),
            origin=Origin(xyz=(0.0, 0.0, -(height - rail) / 2.0)),
            material=frame_material,
            name=f"{prefix}_bottom_rail",
        )
        part.visual(
            Box(
                (
                    width - 2.0 * (rail - glass_capture),
                    0.006,
                    height - 2.0 * (rail - glass_capture),
                )
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=glass_material,
            name=f"{prefix}_glass",
        )

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((outer_rail, frame_depth, window_height)),
        origin=Origin(xyz=(-(window_width - outer_rail) / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((outer_rail, frame_depth, window_height)),
        origin=Origin(xyz=((window_width - outer_rail) / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((window_width, frame_depth, outer_rail)),
        origin=Origin(xyz=(0.0, 0.0, (window_height - outer_rail) / 2.0)),
        material=aluminum,
        name="head",
    )
    outer_frame.visual(
        Box((window_width, frame_depth, outer_rail)),
        origin=Origin(xyz=(0.0, 0.0, -(window_height - outer_rail) / 2.0)),
        material=aluminum,
        name="sill",
    )

    for z_sign, name_prefix in ((1.0, "top"), (-1.0, "bottom")):
        outer_frame.visual(
            Box((clear_width, 0.008, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, z_sign * (clear_height / 2.0 - 0.010))),
            material=dark_aluminum,
            name=f"{name_prefix}_track_divider",
        )
        outer_frame.visual(
            Box((clear_width, 0.012, 0.020)),
            origin=Origin(
                xyz=(0.0, fixed_track_y - sash_depth / 2.0 - 0.006, z_sign * (clear_height / 2.0 - 0.010))
            ),
            material=dark_aluminum,
            name=f"{name_prefix}_outer_retaining_lip",
        )
        outer_frame.visual(
            Box((clear_width, 0.012, 0.020)),
            origin=Origin(
                xyz=(0.0, sliding_track_y + sash_depth / 2.0 + 0.006, z_sign * (clear_height / 2.0 - 0.010))
            ),
            material=dark_aluminum,
            name=f"{name_prefix}_inner_retaining_lip",
        )

    outer_frame.visual(
        Box((clear_width, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -(clear_height / 2.0 - 0.010))),
        material=dark_aluminum,
        name="sill_separator_bridge",
    )
    outer_frame.inertial = Inertial.from_geometry(
        Box((window_width, frame_depth, window_height)),
        mass=18.0,
    )

    fixed_segment = model.part("fixed_segment")
    add_sash(
        fixed_segment,
        width=fixed_panel_width,
        height=panel_height,
        depth=sash_depth,
        rail=sash_rail,
        frame_material=dark_aluminum,
        glass_material=glass,
        prefix="fixed",
    )
    fixed_segment.visual(
        Box((0.008, sash_depth, panel_height)),
        origin=Origin(xyz=((fixed_panel_width - 0.008) / 2.0, 0.0, 0.0)),
        material=gasket,
        name="meeting_gasket",
    )
    fixed_segment.inertial = Inertial.from_geometry(
        Box((fixed_panel_width, sash_depth, panel_height)),
        mass=8.0,
    )

    sliding_sash = model.part("sliding_sash")
    add_sash(
        sliding_sash,
        width=sliding_panel_width,
        height=panel_height,
        depth=sash_depth,
        rail=sash_rail,
        frame_material=dark_aluminum,
        glass_material=glass,
        prefix="sash",
    )
    sliding_sash.visual(
        Box((0.030, 0.020, 0.160)),
        origin=Origin(xyz=((sliding_panel_width - 0.030) / 2.0, 0.014, 0.0)),
        material=dark_aluminum,
        name="pull_rail",
    )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((sliding_panel_width, 0.030, panel_height)),
        mass=9.0,
    )

    travel_stop = model.part("travel_stop")
    travel_stop.visual(
        Box((0.018, 0.032, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=gasket,
        name="travel_stop_block",
    )
    travel_stop.inertial = Inertial.from_geometry(
        Box((0.018, 0.032, 0.050)),
        mass=0.2,
    )

    fixed_center_x = -0.23
    sliding_closed_x = 0.23

    model.articulation(
        "frame_to_fixed_segment",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_segment,
        origin=Origin(xyz=(fixed_center_x, fixed_track_y, 0.0)),
    )
    sash_slide = model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(sliding_closed_x, sliding_track_y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.40,
            lower=0.0,
            upper=travel,
        ),
    )
    model.articulation(
        "frame_to_travel_stop",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=travel_stop,
        origin=Origin(
            xyz=(
                sliding_closed_x - travel - sliding_panel_width / 2.0 + 0.009,
                sliding_track_y,
                clear_height / 2.0 - 0.035,
            )
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_segment = object_model.get_part("fixed_segment")
    sliding_sash = object_model.get_part("sliding_sash")
    travel_stop = object_model.get_part("travel_stop")
    sash_slide = object_model.get_articulation("frame_to_sliding_sash")

    ctx.expect_gap(
        sliding_sash,
        fixed_segment,
        axis="y",
        min_gap=0.020,
        max_gap=0.028,
        name="sliding sash runs in a separate parallel track from the fixed segment",
    )
    ctx.expect_overlap(
        sliding_sash,
        fixed_segment,
        axes="x",
        min_overlap=0.14,
        name="closed sash overlaps the fixed segment in plan like a real slider",
    )

    closed_pos = ctx.part_world_position(sliding_sash)
    with ctx.pose({sash_slide: sash_slide.motion_limits.upper}):
        ctx.expect_contact(
            sliding_sash,
            travel_stop,
            elem_a="sash_top_rail",
            elem_b="travel_stop_block",
            name="opened sash meets the fixed travel stop",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_segment,
            axes="x",
            min_overlap=0.50,
            name="opened sash still remains substantially over the fixed segment",
        )
        open_pos = ctx.part_world_position(sliding_sash)

    ctx.check(
        "sash opens left along the guide channel",
        closed_pos is not None and open_pos is not None and open_pos[0] < closed_pos[0] - 0.30,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
