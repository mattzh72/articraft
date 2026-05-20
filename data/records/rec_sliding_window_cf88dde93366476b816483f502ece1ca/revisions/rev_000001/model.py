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


FRAME_WIDTH = 1.20
FRAME_HEIGHT = 1.00
FRAME_DEPTH = 0.12
FRAME_BORDER = 0.06

OPENING_WIDTH = FRAME_WIDTH - 2.0 * FRAME_BORDER
PANEL_WIDTH = 0.53
PANEL_HEIGHT = 0.81
PANEL_DEPTH = 0.036
PANEL_CENTER_Z = 0.50

TRACK_STOP_THICKNESS = 0.008
TRACK_STOP_Y = 0.044
TRACK_SEPARATOR_Y = 0.0

EXTERIOR_TRACK_Y = -0.022
INTERIOR_TRACK_Y = 0.022

PANEL_TRAVEL = 0.53


def _add_window_panel(
    model: ArticulatedObject,
    name: str,
    *,
    frame_material,
    glass_material,
    seal_material,
    meeting_side: str = "left",
    handle: bool = False,
):
    panel = model.part(name)

    stile = 0.036
    rail = 0.036
    half_w = PANEL_WIDTH * 0.5
    half_h = PANEL_HEIGHT * 0.5

    panel.visual(
        Box((stile, PANEL_DEPTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(-half_w + stile * 0.5, 0.0, 0.0)),
        material=frame_material,
        name="left_stile",
    )
    panel.visual(
        Box((stile, PANEL_DEPTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(half_w - stile * 0.5, 0.0, 0.0)),
        material=frame_material,
        name="right_stile",
    )
    panel.visual(
        Box((PANEL_WIDTH, PANEL_DEPTH, rail)),
        origin=Origin(xyz=(0.0, 0.0, half_h - rail * 0.5)),
        material=frame_material,
        name="top_rail",
    )
    panel.visual(
        Box((PANEL_WIDTH, PANEL_DEPTH, rail)),
        origin=Origin(xyz=(0.0, 0.0, -half_h + rail * 0.5)),
        material=frame_material,
        name="bottom_rail",
    )

    panel.visual(
        Box((PANEL_WIDTH - 2.0 * stile + 0.006, 0.006, PANEL_HEIGHT - 2.0 * rail + 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass_material,
        name="glass",
    )

    strip_sign = -1.0 if meeting_side == "left" else 1.0
    strip_x = strip_sign * (half_w - stile + 0.007)
    panel.visual(
        Box((0.014, PANEL_DEPTH + 0.002, 0.74)),
        origin=Origin(xyz=(strip_x, 0.0, 0.0)),
        material=seal_material,
        name="meeting_strip",
    )

    if handle:
        panel.visual(
            Box((0.018, 0.008, 0.10)),
            origin=Origin(
                xyz=(
                    strip_sign * (half_w - stile + 0.003),
                    PANEL_DEPTH * 0.5 + 0.004,
                    0.0,
                )
            ),
            material=seal_material,
            name="pull_handle",
        )

    panel.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, PANEL_DEPTH, PANEL_HEIGHT)),
        mass=8.0,
    )
    return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    vinyl = model.material("vinyl", rgba=(0.94, 0.94, 0.92, 1.0))
    gasket = model.material("gasket", rgba=(0.18, 0.18, 0.19, 1.0))
    glass = model.material("glass", rgba=(0.68, 0.83, 0.90, 0.32))

    outer_frame = model.part("outer_frame")

    outer_frame.visual(
        Box((FRAME_BORDER, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-(FRAME_WIDTH - FRAME_BORDER) * 0.5, 0.0, FRAME_HEIGHT * 0.5)),
        material=vinyl,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((FRAME_BORDER, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=((FRAME_WIDTH - FRAME_BORDER) * 0.5, 0.0, FRAME_HEIGHT * 0.5)),
        material=vinyl,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_BORDER)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT - FRAME_BORDER * 0.5)),
        material=vinyl,
        name="head",
    )
    outer_frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_BORDER)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_BORDER * 0.5)),
        material=vinyl,
        name="sill",
    )

    guide_height = 0.04
    guide_depth = 0.84
    jamb_guide_width = 0.04

    for visual_name, y_pos in (
        ("exterior_stop", -TRACK_STOP_Y),
        ("track_separator", TRACK_SEPARATOR_Y),
        ("interior_stop", TRACK_STOP_Y),
    ):
        outer_frame.visual(
            Box((OPENING_WIDTH, TRACK_STOP_THICKNESS, guide_height)),
            origin=Origin(xyz=(0.0, y_pos, FRAME_HEIGHT - FRAME_BORDER - guide_height * 0.5)),
            material=vinyl,
            name=f"head_{visual_name}",
        )
        outer_frame.visual(
            Box((OPENING_WIDTH, TRACK_STOP_THICKNESS, guide_height)),
            origin=Origin(xyz=(0.0, y_pos, FRAME_BORDER + guide_height * 0.5)),
            material=vinyl,
            name=f"sill_{visual_name}",
        )
        outer_frame.visual(
            Box((jamb_guide_width, TRACK_STOP_THICKNESS, guide_depth)),
            origin=Origin(
                xyz=(
                    -OPENING_WIDTH * 0.5 + jamb_guide_width * 0.5,
                    y_pos,
                    PANEL_CENTER_Z,
                )
            ),
            material=vinyl,
            name=f"left_{visual_name}",
        )
        outer_frame.visual(
            Box((jamb_guide_width, TRACK_STOP_THICKNESS, guide_depth)),
            origin=Origin(
                xyz=(
                    OPENING_WIDTH * 0.5 - jamb_guide_width * 0.5,
                    y_pos,
                    PANEL_CENTER_Z,
                )
            ),
            material=vinyl,
            name=f"right_{visual_name}",
        )

    outer_frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT * 0.5)),
    )

    fixed_panel = _add_window_panel(
        model,
        "fixed_panel",
        frame_material=vinyl,
        glass_material=glass,
        seal_material=gasket,
        meeting_side="right",
    )

    model.articulation(
        "frame_to_fixed_panel",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_panel,
        origin=Origin(xyz=(-PANEL_WIDTH * 0.5, EXTERIOR_TRACK_Y, PANEL_CENTER_Z)),
    )

    sliding_sash = _add_window_panel(
        model,
        "sliding_sash",
        frame_material=vinyl,
        glass_material=glass,
        seal_material=gasket,
        meeting_side="left",
        handle=True,
    )

    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(PANEL_WIDTH * 0.5, INTERIOR_TRACK_Y, PANEL_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.30,
            lower=0.0,
            upper=PANEL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("outer_frame")
    fixed_panel = object_model.get_part("fixed_panel")
    sliding_sash = object_model.get_part("sliding_sash")
    sash_slide = object_model.get_articulation("frame_to_sliding_sash")

    upper = sash_slide.motion_limits.upper if sash_slide.motion_limits else PANEL_TRAVEL

    with ctx.pose({sash_slide: 0.0}):
        ctx.expect_gap(
            sliding_sash,
            frame,
            axis="y",
            positive_elem="top_rail",
            negative_elem="head_track_separator",
            max_gap=0.0005,
            max_penetration=0.0,
            name="sliding sash bears against the separator rail",
        )
        ctx.expect_gap(
            frame,
            sliding_sash,
            axis="y",
            positive_elem="head_interior_stop",
            negative_elem="top_rail",
            max_gap=0.0005,
            max_penetration=0.0,
            name="sliding sash stays captured by the interior stop",
        )
        ctx.expect_gap(
            fixed_panel,
            frame,
            axis="y",
            positive_elem="top_rail",
            negative_elem="head_exterior_stop",
            max_gap=0.0005,
            max_penetration=0.0,
            name="fixed panel bears against the exterior stop",
        )
        ctx.expect_gap(
            frame,
            fixed_panel,
            axis="y",
            positive_elem="head_track_separator",
            negative_elem="top_rail",
            max_gap=0.0005,
            max_penetration=0.0,
            name="fixed panel is clamped to the separator",
        )
        ctx.expect_gap(
            sliding_sash,
            fixed_panel,
            axis="x",
            positive_elem="left_stile",
            negative_elem="right_stile",
            max_gap=0.001,
            max_penetration=1e-6,
            name="closed sash meets the fixed panel at the centerline",
        )
        ctx.expect_overlap(
            sliding_sash,
            frame,
            axes="x",
            elem_a="top_rail",
            elem_b="head_track_separator",
            min_overlap=0.50,
            name="closed sash top rail remains inserted in the head guide",
        )

    rest_pos = ctx.part_world_position(sliding_sash)
    with ctx.pose({sash_slide: upper}):
        ctx.expect_overlap(
            sliding_sash,
            frame,
            axes="x",
            elem_a="top_rail",
            elem_b="head_track_separator",
            min_overlap=0.50,
            name="open sash top rail remains inserted in the head guide",
        )
        ctx.expect_overlap(
            sliding_sash,
            frame,
            axes="x",
            elem_a="bottom_rail",
            elem_b="sill_track_separator",
            min_overlap=0.50,
            name="open sash bottom rail remains inserted in the sill guide",
        )
        open_pos = ctx.part_world_position(sliding_sash)

    ctx.check(
        "positive sash travel opens leftward",
        rest_pos is not None and open_pos is not None and open_pos[0] < rest_pos[0] - 0.45,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
