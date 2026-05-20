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

    frame_aluminum = model.material("frame_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_gasket = model.material("dark_gasket", rgba=(0.12, 0.12, 0.13, 1.0))
    latch_black = model.material("latch_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.83, 0.92, 0.35))

    outer_width = 1.20
    outer_height = 1.00
    frame_depth = 0.09
    perimeter = 0.06
    inner_width = outer_width - 2.0 * perimeter
    inner_height = outer_height - 2.0 * perimeter

    sash_width = 0.548
    sash_height = 0.84
    sash_depth = 0.022
    sash_stile = 0.05
    sash_rail = 0.05

    fixed_width = 0.548
    fixed_height = 0.84
    fixed_depth = 0.018
    fixed_stile = 0.048
    fixed_rail = 0.048

    moving_closed_x = 0.265
    fixed_x = -0.266
    fixed_y = -0.029
    max_open_travel = 0.49

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((perimeter, frame_depth, outer_height)),
        origin=Origin(xyz=(-outer_width / 2.0 + perimeter / 2.0, 0.0, 0.0)),
        material=frame_aluminum,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((perimeter, frame_depth, outer_height)),
        origin=Origin(xyz=(outer_width / 2.0 - perimeter / 2.0, 0.0, 0.0)),
        material=frame_aluminum,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((inner_width, frame_depth, perimeter)),
        origin=Origin(xyz=(0.0, 0.0, outer_height / 2.0 - perimeter / 2.0)),
        material=frame_aluminum,
        name="head",
    )
    outer_frame.visual(
        Box((inner_width, frame_depth, perimeter)),
        origin=Origin(xyz=(0.0, 0.0, -outer_height / 2.0 + perimeter / 2.0)),
        material=frame_aluminum,
        name="sill",
    )

    guide_lip_depth = 0.008
    guide_lip_height = 0.036
    guide_slot_half_depth = 0.011
    guide_lip_center_y = 0.015
    top_lip_center_z = inner_height / 2.0 - guide_lip_height / 2.0 + 0.001
    bottom_lip_center_z = -inner_height / 2.0 + guide_lip_height / 2.0 - 0.001

    outer_frame.visual(
        Box((inner_width, guide_lip_depth, guide_lip_height)),
        origin=Origin(xyz=(0.0, -guide_lip_center_y, top_lip_center_z)),
        material=dark_gasket,
        name="top_rear_guide_lip",
    )
    outer_frame.visual(
        Box((inner_width, guide_lip_depth, guide_lip_height)),
        origin=Origin(xyz=(0.0, guide_lip_center_y, top_lip_center_z)),
        material=dark_gasket,
        name="top_front_guide_lip",
    )
    outer_frame.visual(
        Box((inner_width, guide_lip_depth, guide_lip_height)),
        origin=Origin(xyz=(0.0, -guide_lip_center_y, bottom_lip_center_z)),
        material=dark_gasket,
        name="bottom_rear_guide_lip",
    )
    outer_frame.visual(
        Box((inner_width, guide_lip_depth, guide_lip_height)),
        origin=Origin(xyz=(0.0, guide_lip_center_y, bottom_lip_center_z)),
        material=dark_gasket,
        name="bottom_front_guide_lip",
    )
    outer_frame.inertial = Inertial.from_geometry(
        Box((outer_width, frame_depth, outer_height)),
        mass=18.0,
    )

    fixed_panel = model.part("fixed_panel")
    fixed_panel.visual(
        Box((fixed_width, fixed_depth, fixed_rail)),
        origin=Origin(xyz=(0.0, 0.0, fixed_height / 2.0 - fixed_rail / 2.0)),
        material=frame_aluminum,
        name="fixed_top_rail",
    )
    fixed_panel.visual(
        Box((fixed_width, fixed_depth, fixed_rail)),
        origin=Origin(xyz=(0.0, 0.0, -fixed_height / 2.0 + fixed_rail / 2.0)),
        material=frame_aluminum,
        name="fixed_bottom_rail",
    )
    fixed_panel.visual(
        Box((fixed_stile, fixed_depth, fixed_height - 2.0 * fixed_rail + 0.01)),
        origin=Origin(xyz=(-fixed_width / 2.0 + fixed_stile / 2.0, 0.0, 0.0)),
        material=frame_aluminum,
        name="fixed_left_stile",
    )
    fixed_panel.visual(
        Box((fixed_stile, fixed_depth, fixed_height - 2.0 * fixed_rail + 0.01)),
        origin=Origin(xyz=(fixed_width / 2.0 - fixed_stile / 2.0, 0.0, 0.0)),
        material=frame_aluminum,
        name="fixed_right_stile",
    )
    fixed_panel.visual(
        Box((fixed_width - 2.0 * fixed_stile + 0.012, 0.006, fixed_height - 2.0 * fixed_rail + 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="fixed_glass",
    )
    fixed_panel.inertial = Inertial.from_geometry(
        Box((fixed_width, fixed_depth, fixed_height)),
        mass=6.5,
    )

    sliding_sash = model.part("sliding_sash")
    sliding_sash.visual(
        Box((sash_width, sash_depth, sash_rail)),
        origin=Origin(xyz=(0.0, 0.0, sash_height / 2.0 - sash_rail / 2.0)),
        material=frame_aluminum,
        name="sash_top_rail",
    )
    sliding_sash.visual(
        Box((sash_width, sash_depth, sash_rail)),
        origin=Origin(xyz=(0.0, 0.0, -sash_height / 2.0 + sash_rail / 2.0)),
        material=frame_aluminum,
        name="sash_bottom_rail",
    )
    sliding_sash.visual(
        Box((sash_stile, sash_depth, sash_height - 2.0 * sash_rail + 0.01)),
        origin=Origin(xyz=(-sash_width / 2.0 + sash_stile / 2.0, 0.0, 0.0)),
        material=frame_aluminum,
        name="meeting_stile",
    )
    sliding_sash.visual(
        Box((sash_stile, sash_depth, sash_height - 2.0 * sash_rail + 0.01)),
        origin=Origin(xyz=(sash_width / 2.0 - sash_stile / 2.0, 0.0, 0.0)),
        material=frame_aluminum,
        name="lock_stile",
    )
    sliding_sash.visual(
        Box((sash_width - 2.0 * sash_stile + 0.012, 0.006, sash_height - 2.0 * sash_rail + 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="sash_glass",
    )
    sliding_sash.visual(
        Box((0.498, guide_slot_half_depth * 2.0 - 0.004, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, inner_height / 2.0 - 0.015)),
        material=dark_gasket,
        name="top_guide_shoe",
    )
    sliding_sash.visual(
        Box((0.498, guide_slot_half_depth * 2.0 - 0.004, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -inner_height / 2.0 + 0.015)),
        material=dark_gasket,
        name="bottom_guide_shoe",
    )
    sliding_sash.visual(
        Box((0.018, 0.007, 0.11)),
        origin=Origin(xyz=(-sash_width / 2.0 + 0.030, sash_depth / 2.0 + 0.0035, 0.0)),
        material=latch_black,
        name="pull_handle",
    )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((sash_width, 0.03, 0.87)),
        mass=7.2,
    )

    model.articulation(
        "frame_to_fixed_panel",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_panel,
        origin=Origin(xyz=(fixed_x, fixed_y, 0.0)),
    )

    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(moving_closed_x, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.45,
            lower=0.0,
            upper=max_open_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    fixed_panel = object_model.get_part("fixed_panel")
    sliding_sash = object_model.get_part("sliding_sash")
    sash_slide = object_model.get_articulation("frame_to_sliding_sash")

    ctx.expect_origin_distance(
        sliding_sash,
        outer_frame,
        axes="yz",
        max_dist=1e-6,
        name="sliding sash stays centered on the frame body axis",
    )
    ctx.expect_within(
        sliding_sash,
        outer_frame,
        axes="yz",
        margin=0.0,
        name="sliding sash remains within the frame depth and height envelope",
    )
    ctx.expect_gap(
        sliding_sash,
        fixed_panel,
        axis="y",
        min_gap=0.008,
        max_gap=0.010,
        name="sliding sash clears the rear fixed segment in depth",
    )

    closed_x = ctx.part_world_position(sliding_sash)
    with ctx.pose({sash_slide: 0.49}):
        ctx.expect_within(
            sliding_sash,
            outer_frame,
            axes="yz",
            margin=0.0,
            name="opened sash stays captured by the frame guides",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_panel,
            axes="x",
            min_overlap=0.45,
            name="opened sash tracks horizontally across the fixed light",
        )
        open_x = ctx.part_world_position(sliding_sash)

    ctx.check(
        "sash opens leftward along the guide channels",
        closed_x is not None and open_x is not None and open_x[0] < closed_x[0] - 0.40,
        details=f"closed={closed_x}, open={open_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
