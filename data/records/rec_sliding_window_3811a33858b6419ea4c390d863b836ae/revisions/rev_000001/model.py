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


FRAME_W = 1.20
FRAME_H = 1.00
FRAME_D = 0.12
PERIM_T = 0.055
TRACK_W = 0.026
TRACK_H = 0.025
HEAD_GUIDE_H = 0.020

PANEL_D = 0.028
PANEL_H = 0.84
FIXED_PANEL_W = 0.545
SASH_W = 0.575
STILE_W = 0.050
RAIL_H = 0.050
GLASS_T = 0.008

FIXED_X = -0.2725
SASH_CLOSED_X = 0.2575
PANEL_CENTER_Z = 0.50
FIXED_Y = -0.018
SASH_Y = 0.018
SASH_TRAVEL = 0.475


def _add_glazed_panel(
    part,
    *,
    width: float,
    height: float,
    depth: float,
    frame_material,
    glass_material,
    accent_material=None,
    add_handle: bool = False,
) -> None:
    half_w = width * 0.5
    half_h = height * 0.5

    part.visual(
        Box((STILE_W, depth, height)),
        origin=Origin(xyz=(-half_w + STILE_W * 0.5, 0.0, 0.0)),
        material=frame_material,
        name="left_stile",
    )
    part.visual(
        Box((STILE_W, depth, height)),
        origin=Origin(xyz=(half_w - STILE_W * 0.5, 0.0, 0.0)),
        material=frame_material,
        name="right_stile",
    )
    part.visual(
        Box((width, depth, RAIL_H)),
        origin=Origin(xyz=(0.0, 0.0, half_h - RAIL_H * 0.5)),
        material=frame_material,
        name="top_rail",
    )
    part.visual(
        Box((width, depth, RAIL_H)),
        origin=Origin(xyz=(0.0, 0.0, -half_h + RAIL_H * 0.5)),
        material=frame_material,
        name="bottom_rail",
    )
    part.visual(
        Box((width - 0.075, GLASS_T, height - 0.090)),
        origin=Origin(),
        material=glass_material,
        name="glass",
    )

    if add_handle and accent_material is not None:
        handle_x = -half_w + STILE_W * 0.92
        part.visual(
            Box((0.020, 0.012, 0.140)),
            origin=Origin(xyz=(handle_x, depth * 0.55, 0.0)),
            material=accent_material,
            name="pull_rail",
        )
        part.visual(
            Box((0.008, 0.018, 0.095)),
            origin=Origin(xyz=(handle_x + 0.013, depth * 0.70, 0.0)),
            material=accent_material,
            name="finger_pull",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    frame_white = model.material("frame_white", rgba=(0.93, 0.94, 0.95, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.72, 0.84, 0.90, 0.35))

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((PERIM_T, FRAME_D, FRAME_H)),
        origin=Origin(xyz=(-FRAME_W * 0.5 + PERIM_T * 0.5, 0.0, FRAME_H * 0.5)),
        material=frame_white,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((PERIM_T, FRAME_D, FRAME_H)),
        origin=Origin(xyz=(FRAME_W * 0.5 - PERIM_T * 0.5, 0.0, FRAME_H * 0.5)),
        material=frame_white,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((FRAME_W, FRAME_D, PERIM_T)),
        origin=Origin(xyz=(0.0, 0.0, PERIM_T * 0.5)),
        material=frame_white,
        name="sill",
    )
    outer_frame.visual(
        Box((FRAME_W, FRAME_D, PERIM_T)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_H - PERIM_T * 0.5)),
        material=frame_white,
        name="head",
    )
    outer_frame.visual(
        Box((FRAME_W - 2.0 * PERIM_T, TRACK_W, TRACK_H)),
        origin=Origin(xyz=(0.0, FIXED_Y, PERIM_T + TRACK_H * 0.5)),
        material=frame_white,
        name="rear_sill_track",
    )
    outer_frame.visual(
        Box((FRAME_W - 2.0 * PERIM_T, TRACK_W, TRACK_H)),
        origin=Origin(xyz=(0.0, SASH_Y, PERIM_T + TRACK_H * 0.5)),
        material=frame_white,
        name="front_sill_track",
    )
    outer_frame.visual(
        Box((FRAME_W - 2.0 * PERIM_T, TRACK_W, HEAD_GUIDE_H)),
        origin=Origin(xyz=(0.0, FIXED_Y, FRAME_H - PERIM_T - HEAD_GUIDE_H * 0.5)),
        material=frame_white,
        name="rear_head_guide",
    )
    outer_frame.visual(
        Box((FRAME_W - 2.0 * PERIM_T, TRACK_W, HEAD_GUIDE_H)),
        origin=Origin(xyz=(0.0, SASH_Y, FRAME_H - PERIM_T - HEAD_GUIDE_H * 0.5)),
        material=frame_white,
        name="front_head_guide",
    )
    outer_frame.inertial = Inertial.from_geometry(
        Box((FRAME_W, FRAME_D, FRAME_H)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_H * 0.5)),
    )

    fixed_panel = model.part("fixed_panel")
    _add_glazed_panel(
        fixed_panel,
        width=FIXED_PANEL_W,
        height=PANEL_H,
        depth=PANEL_D,
        frame_material=frame_white,
        glass_material=glass_tint,
    )
    fixed_panel.inertial = Inertial.from_geometry(
        Box((FIXED_PANEL_W, PANEL_D, PANEL_H)),
        mass=11.0,
    )

    sliding_sash = model.part("sliding_sash")
    _add_glazed_panel(
        sliding_sash,
        width=SASH_W,
        height=PANEL_H,
        depth=PANEL_D,
        frame_material=frame_white,
        glass_material=glass_tint,
        accent_material=gasket_dark,
        add_handle=True,
    )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((SASH_W, 0.040, PANEL_H)),
        mass=12.0,
    )

    model.articulation(
        "frame_to_fixed_panel",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_panel,
        origin=Origin(xyz=(FIXED_X, FIXED_Y, PANEL_CENTER_Z)),
    )
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(SASH_CLOSED_X, SASH_Y, PANEL_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=SASH_TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    fixed_panel = object_model.get_part("fixed_panel")
    sliding_sash = object_model.get_part("sliding_sash")
    sash_joint = object_model.get_articulation("frame_to_sliding_sash")

    ctx.expect_gap(
        fixed_panel,
        outer_frame,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="rear_sill_track",
        max_gap=0.001,
        max_penetration=0.0,
        name="fixed panel sits on rear sill track",
    )
    ctx.expect_gap(
        sliding_sash,
        outer_frame,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="front_sill_track",
        max_gap=0.001,
        max_penetration=0.0,
        name="sliding sash sits on front sill track",
    )
    ctx.expect_gap(
        sliding_sash,
        fixed_panel,
        axis="y",
        min_gap=0.006,
        name="sliding sash stays on a separate front track",
    )
    ctx.expect_gap(
        outer_frame,
        sliding_sash,
        axis="x",
        positive_elem="right_jamb",
        negative_elem="right_stile",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed sash meets the right jamb",
    )

    sash_rest = ctx.part_world_position(sliding_sash)
    with ctx.pose({sash_joint: SASH_TRAVEL}):
        ctx.expect_gap(
            outer_frame,
            sliding_sash,
            axis="x",
            positive_elem="right_jamb",
            negative_elem="right_stile",
            min_gap=0.45,
            name="open sash clears the right side opening",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_panel,
            axes="x",
            min_overlap=0.40,
            name="open sash overlaps the fixed segment in plan",
        )
        ctx.expect_gap(
            sliding_sash,
            fixed_panel,
            axis="y",
            min_gap=0.006,
            name="tracks stay separated when the sash is open",
        )
        sash_open = ctx.part_world_position(sliding_sash)

    ctx.check(
        "sash translates left to open",
        sash_rest is not None and sash_open is not None and sash_open[0] < sash_rest[0] - 0.30,
        details=f"rest={sash_rest}, open={sash_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
