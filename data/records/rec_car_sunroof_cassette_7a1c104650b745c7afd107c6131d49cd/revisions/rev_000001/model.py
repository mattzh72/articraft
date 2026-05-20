from __future__ import annotations

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
    model = ArticulatedObject(name="sliding_panoramic_moonroof_cassette")

    aluminium = model.material("satin_aluminium", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_anodized = model.material("dark_anodized_rail", rgba=(0.08, 0.085, 0.09, 1.0))
    black_rubber = model.material("black_rubber_seal", rgba=(0.004, 0.004, 0.005, 1.0))
    tinted_glass = model.material("smoked_blue_glass", rgba=(0.05, 0.12, 0.17, 0.42))
    frit = model.material("black_ceramic_frit", rgba=(0.0, 0.0, 0.0, 0.86))
    fabric = model.material("warm_grey_sunblind_fabric", rgba=(0.66, 0.62, 0.54, 1.0))
    blind_edge = model.material("dark_blind_edge", rgba=(0.12, 0.11, 0.10, 1.0))

    # Vehicle coordinates: X is fore/aft, positive X is rearward, Y is cross-car,
    # Z is upward toward the roof exterior.
    frame = model.part("frame")

    # Low aluminium cassette perimeter, sized like a real panoramic roof module.
    frame.visual(
        Box((1.70, 0.17, 0.040)),
        origin=Origin(xyz=(0.0, 0.440, 0.020)),
        material=aluminium,
        name="side_rail_0",
    )
    frame.visual(
        Box((1.70, 0.17, 0.040)),
        origin=Origin(xyz=(0.0, -0.440, 0.020)),
        material=aluminium,
        name="side_rail_1",
    )
    frame.visual(
        Box((0.18, 1.05, 0.040)),
        origin=Origin(xyz=(-0.760, 0.0, 0.020)),
        material=aluminium,
        name="front_crossmember",
    )
    frame.visual(
        Box((0.18, 1.05, 0.040)),
        origin=Origin(xyz=(0.760, 0.0, 0.020)),
        material=aluminium,
        name="rear_crossmember",
    )
    frame.visual(
        Cylinder(radius=0.032, length=0.620),
        origin=Origin(xyz=(-0.655, 0.0, 0.068), rpy=(-1.5708, 0.0, 0.0)),
        material=blind_edge,
        name="roller_tube",
    )

    # Two raised U-channel guides for the moving glass panel.  The lips sit proud
    # of the base extrusion and define the visible prismatic rail pair.
    frame.visual(
        Box((1.46, 0.092, 0.012)),
        origin=Origin(xyz=(0.0, 0.390, 0.046)),
        material=dark_anodized,
        name="guide_floor_0",
    )
    frame.visual(
        Box((1.46, 0.014, 0.055)),
        origin=Origin(xyz=(0.0, 0.344, 0.0675)),
        material=aluminium,
        name="guide_inner_lip_0",
    )
    frame.visual(
        Box((1.46, 0.014, 0.055)),
        origin=Origin(xyz=(0.0, 0.436, 0.0675)),
        material=aluminium,
        name="guide_outer_lip_0",
    )
    frame.visual(
        Box((1.38, 0.080, 0.012)),
        origin=Origin(xyz=(0.0, 0.320, 0.052)),
        material=dark_anodized,
        name="blind_track_0",
    )
    frame.visual(
        Box((0.060, 0.160, 0.025)),
        origin=Origin(xyz=(-0.705, 0.372, 0.0525)),
        material=black_rubber,
        name="front_stop_0",
    )
    frame.visual(
        Box((0.060, 0.160, 0.025)),
        origin=Origin(xyz=(0.705, 0.372, 0.0525)),
        material=black_rubber,
        name="rear_stop_0",
    )
    frame.visual(
        Box((1.46, 0.092, 0.012)),
        origin=Origin(xyz=(0.0, -0.390, 0.046)),
        material=dark_anodized,
        name="guide_floor_1",
    )
    frame.visual(
        Box((1.46, 0.014, 0.055)),
        origin=Origin(xyz=(0.0, -0.344, 0.0675)),
        material=aluminium,
        name="guide_inner_lip_1",
    )
    frame.visual(
        Box((1.46, 0.014, 0.055)),
        origin=Origin(xyz=(0.0, -0.436, 0.0675)),
        material=aluminium,
        name="guide_outer_lip_1",
    )
    frame.visual(
        Box((1.38, 0.080, 0.012)),
        origin=Origin(xyz=(0.0, -0.320, 0.052)),
        material=dark_anodized,
        name="blind_track_1",
    )
    frame.visual(
        Box((0.060, 0.160, 0.025)),
        origin=Origin(xyz=(-0.705, -0.372, 0.0525)),
        material=black_rubber,
        name="front_stop_1",
    )
    frame.visual(
        Box((0.060, 0.160, 0.025)),
        origin=Origin(xyz=(0.705, -0.372, 0.0525)),
        material=black_rubber,
        name="rear_stop_1",
    )

    # A continuous gasket around the roof opening emphasizes that this is a
    # cassette with a sealed aperture, not a simple rectangular tray.
    frame.visual(
        Box((1.36, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.285, 0.101)),
        material=black_rubber,
        name="gasket_side_0",
    )
    frame.visual(
        Box((1.36, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.285, 0.101)),
        material=black_rubber,
        name="gasket_side_1",
    )
    frame.visual(
        Box((1.36, 0.075, 0.008)),
        origin=Origin(xyz=(0.0, 0.3135, 0.092)),
        material=black_rubber,
        name="gasket_shelf_0",
    )
    frame.visual(
        Box((1.36, 0.075, 0.008)),
        origin=Origin(xyz=(0.0, -0.3135, 0.092)),
        material=black_rubber,
        name="gasket_shelf_1",
    )
    frame.visual(
        Box((0.018, 0.588, 0.018)),
        origin=Origin(xyz=(-0.670, 0.0, 0.101)),
        material=black_rubber,
        name="gasket_front",
    )
    frame.visual(
        Box((0.018, 0.588, 0.018)),
        origin=Origin(xyz=(0.670, 0.0, 0.101)),
        material=black_rubber,
        name="gasket_rear",
    )

    glass_panel = model.part("glass_panel")
    glass_panel.visual(
        Box((1.05, 0.640, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tinted_glass,
        name="glass_pane",
    )
    # Black ceramic frit printed onto the glass edge.
    glass_panel.visual(
        Box((1.05, 0.055, 0.004)),
        origin=Origin(xyz=(0.0, 0.292, 0.009)),
        material=frit,
        name="frit_side_0",
    )
    glass_panel.visual(
        Box((1.05, 0.055, 0.004)),
        origin=Origin(xyz=(0.0, -0.292, 0.009)),
        material=frit,
        name="frit_side_1",
    )
    glass_panel.visual(
        Box((0.075, 0.640, 0.004)),
        origin=Origin(xyz=(-0.4875, 0.0, 0.009)),
        material=frit,
        name="frit_front",
    )
    glass_panel.visual(
        Box((0.075, 0.640, 0.004)),
        origin=Origin(xyz=(0.4875, 0.0, 0.009)),
        material=frit,
        name="frit_rear",
    )
    # Side carriers are bonded under the glass and bridge outward to the two
    # aluminium guide channels.  They sit just above the rail lips.
    glass_panel.visual(
        Box((1.05, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, 0.390, -0.012)),
        material=dark_anodized,
        name="side_carrier_0",
    )
    glass_panel.visual(
        Box((1.05, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, -0.390, -0.012)),
        material=dark_anodized,
        name="side_carrier_1",
    )
    glass_panel.visual(
        Box((1.05, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.335, -0.008)),
        material=frit,
        name="carrier_bond_0",
    )
    glass_panel.visual(
        Box((1.05, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, -0.335, -0.008)),
        material=frit,
        name="carrier_bond_1",
    )
    glass_panel.visual(
        Box((0.090, 0.620, 0.006)),
        origin=Origin(xyz=(-0.480, 0.0, 0.010)),
        material=black_rubber,
        name="front_seal",
    )
    glass_panel.visual(
        Box((0.090, 0.620, 0.006)),
        origin=Origin(xyz=(0.480, 0.0, 0.010)),
        material=black_rubber,
        name="rear_seal",
    )

    sunblind = model.part("sunblind")
    sunblind.visual(
        Box((1.04, 0.590, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=fabric,
        name="fabric_panel",
    )
    sunblind.visual(
        Box((0.050, 0.610, 0.020)),
        origin=Origin(xyz=(-0.495, 0.0, 0.002)),
        material=blind_edge,
        name="pull_slat",
    )
    sunblind.visual(
        Box((1.04, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.302, -0.002)),
        material=blind_edge,
        name="blind_runner_0",
    )
    sunblind.visual(
        Box((1.04, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.302, -0.002)),
        material=blind_edge,
        name="blind_runner_1",
    )

    model.articulation(
        "frame_to_glass",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=glass_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.30, lower=0.0, upper=0.48),
    )
    model.articulation(
        "frame_to_sunblind",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sunblind,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.45, lower=0.0, upper=0.58),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    glass = object_model.get_part("glass_panel")
    blind = object_model.get_part("sunblind")
    glass_slide = object_model.get_articulation("frame_to_glass")
    blind_slide = object_model.get_articulation("frame_to_sunblind")

    ctx.check(
        "glass panel uses a rearward prismatic slide",
        glass_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(glass_slide.axis) == (1.0, 0.0, 0.0)
        and glass_slide.motion_limits.lower == 0.0
        and glass_slide.motion_limits.upper >= 0.45,
        details=f"type={glass_slide.articulation_type}, axis={glass_slide.axis}, limits={glass_slide.motion_limits}",
    )
    ctx.check(
        "sunblind uses an independent rearward prismatic slide",
        blind_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(blind_slide.axis) == (1.0, 0.0, 0.0)
        and blind_slide.motion_limits.upper > glass_slide.motion_limits.upper,
        details=f"type={blind_slide.articulation_type}, axis={blind_slide.axis}, limits={blind_slide.motion_limits}",
    )

    with ctx.pose({glass_slide: 0.0, blind_slide: 0.0}):
        ctx.expect_overlap(
            glass,
            frame,
            axes="xy",
            elem_a="side_carrier_0",
            elem_b="guide_floor_0",
            min_overlap=0.045,
            name="glass carrier overlaps the upper guide rail at rest",
        )
        ctx.expect_gap(
            glass,
            frame,
            axis="z",
            positive_elem="side_carrier_0",
            negative_elem="guide_inner_lip_0",
            min_gap=0.0,
            max_gap=0.003,
            name="glass carrier rides just above the rail lip",
        )
        ctx.expect_overlap(
            blind,
            frame,
            axes="xy",
            elem_a="blind_runner_0",
            elem_b="blind_track_0",
            min_overlap=0.020,
            name="sunblind runner overlaps its interior track at rest",
        )
        ctx.expect_gap(
            blind,
            frame,
            axis="z",
            positive_elem="blind_runner_0",
            negative_elem="blind_track_0",
            min_gap=0.0,
            max_gap=0.002,
            name="sunblind runner sits on the interior track",
        )
        rest_glass = ctx.part_world_position(glass)
        rest_blind = ctx.part_world_position(blind)

    with ctx.pose({glass_slide: glass_slide.motion_limits.upper, blind_slide: 0.0}):
        ctx.expect_overlap(
            glass,
            frame,
            axes="xy",
            elem_a="side_carrier_0",
            elem_b="guide_floor_0",
            min_overlap=0.045,
            name="glass carrier stays captured on the guide rail when rearward",
        )
        open_glass = ctx.part_world_position(glass)

    with ctx.pose({glass_slide: 0.0, blind_slide: blind_slide.motion_limits.upper}):
        ctx.expect_overlap(
            blind,
            frame,
            axes="xy",
            elem_a="blind_runner_0",
            elem_b="blind_track_0",
            min_overlap=0.020,
            name="sunblind runner stays captured on its track when rearward",
        )
        open_blind = ctx.part_world_position(blind)

    ctx.check(
        "glass panel slides rearward",
        rest_glass is not None and open_glass is not None and open_glass[0] > rest_glass[0] + 0.40,
        details=f"rest={rest_glass}, open={open_glass}",
    )
    ctx.check(
        "sunblind slides rearward independently",
        rest_blind is not None and open_blind is not None and open_blind[0] > rest_blind[0] + 0.50,
        details=f"rest={rest_blind}, open={open_blind}",
    )

    return ctx.report()


object_model = build_object_model()
