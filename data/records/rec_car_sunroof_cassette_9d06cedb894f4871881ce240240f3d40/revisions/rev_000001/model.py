from __future__ import annotations

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
    model = ArticulatedObject(name="panoramic_sunroof_cassette")

    aluminium = model.material("brushed_aluminium", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_anodized = model.material("dark_anodized_rail", rgba=(0.055, 0.060, 0.065, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.005, 0.006, 0.006, 1.0))
    tinted_glass = model.material("blue_tinted_glass", rgba=(0.10, 0.22, 0.30, 0.43))
    black_ceramic = model.material("black_ceramic_frit", rgba=(0.0, 0.0, 0.0, 1.0))
    nylon = model.material("black_nylon_shoe", rgba=(0.025, 0.025, 0.023, 1.0))
    fabric = model.material("warm_grey_fabric", rgba=(0.46, 0.43, 0.39, 1.0))
    fabric_seam = model.material("raised_fabric_seam", rgba=(0.34, 0.32, 0.30, 1.0))

    frame = model.part("cassette_frame")

    # A full roof-width cassette frame: long side extrusions plus deeper front
    # and rear cross members form a large central aperture.
    frame.visual(
        Box((2.60, 0.16, 0.040)),
        origin=Origin(xyz=(0.0, 0.520, 0.0)),
        material=aluminium,
        name="side_frame_0",
    )
    frame.visual(
        Box((2.60, 0.16, 0.040)),
        origin=Origin(xyz=(0.0, -0.520, 0.0)),
        material=aluminium,
        name="side_frame_1",
    )
    frame.visual(
        Box((0.45, 0.88, 0.040)),
        origin=Origin(xyz=(1.075, 0.0, 0.0)),
        material=aluminium,
        name="end_frame_0",
    )
    frame.visual(
        Box((0.45, 0.88, 0.040)),
        origin=Origin(xyz=(-1.075, 0.0, 0.0)),
        material=aluminium,
        name="end_frame_1",
    )

    # Rubber seal around the roof opening, proud of the aluminium frame.
    frame.visual(
        Box((1.78, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.455, 0.026)),
        material=black_rubber,
        name="side_gasket_0",
    )
    frame.visual(
        Box((1.78, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.455, 0.026)),
        material=black_rubber,
        name="side_gasket_1",
    )
    frame.visual(
        Box((0.030, 0.88, 0.012)),
        origin=Origin(xyz=(0.875, 0.0, 0.026)),
        material=black_rubber,
        name="end_gasket_0",
    )
    frame.visual(
        Box((0.030, 0.88, 0.012)),
        origin=Origin(xyz=(-0.875, 0.0, 0.026)),
        material=black_rubber,
        name="end_gasket_1",
    )

    # Twin upper prismatic rails for the glass panel.  The inward side is open
    # so the guide-shoe brackets can be seen entering the channel.
    for y, base_name, flange_name, strip_name in (
        (0.500, "glass_rail_base_0", "glass_rail_flange_0", "glass_wear_strip_0"),
        (-0.500, "glass_rail_base_1", "glass_rail_flange_1", "glass_wear_strip_1"),
    ):
        frame.visual(
            Box((2.46, 0.085, 0.014)),
            origin=Origin(xyz=(0.0, y, 0.027)),
            material=dark_anodized,
            name=base_name,
        )
        frame.visual(
            Box((2.46, 0.012, 0.034)),
            origin=Origin(xyz=(0.0, y + (0.047 if y > 0 else -0.047), 0.051)),
            material=dark_anodized,
            name=flange_name,
        )
        # Machined silver wear strip down the centre of each rail.
        frame.visual(
            Box((2.34, 0.014, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.036)),
            material=aluminium,
            name=strip_name,
        )

    # Lower independent sun-shade rails tucked below the opening and supported
    # by continuous vertical webs from the underside of the frame.
    for y, web_name, rail_name in (
        (0.440, "shade_rail_web_0", "shade_rail_0"),
        (-0.440, "shade_rail_web_1", "shade_rail_1"),
    ):
        frame.visual(
            Box((2.42, 0.014, 0.032)),
            origin=Origin(xyz=(0.0, y, -0.027)),
            material=dark_anodized,
            name=web_name,
        )
        frame.visual(
            Box((2.42, 0.026, 0.016)),
            origin=Origin(xyz=(0.0, y, -0.050)),
            material=dark_anodized,
            name=rail_name,
        )

    glass = model.part("glass_panel")
    glass.visual(
        Box((1.62, 0.94, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=tinted_glass,
        name="glass_lite",
    )
    # Black ceramic frit border, bonded to the top of the glass.
    glass.visual(
        Box((1.62, 0.055, 0.003)),
        origin=Origin(xyz=(0.0, 0.442, 0.104)),
        material=black_ceramic,
        name="frit_side_0",
    )
    glass.visual(
        Box((1.62, 0.055, 0.003)),
        origin=Origin(xyz=(0.0, -0.442, 0.104)),
        material=black_ceramic,
        name="frit_side_1",
    )
    glass.visual(
        Box((0.060, 0.94, 0.003)),
        origin=Origin(xyz=(0.780, 0.0, 0.104)),
        material=black_ceramic,
        name="frit_end_0",
    )
    glass.visual(
        Box((0.060, 0.94, 0.003)),
        origin=Origin(xyz=(-0.780, 0.0, 0.104)),
        material=black_ceramic,
        name="frit_end_1",
    )

    # Four glass guide-shoe carriages, visible at the panel corners and tied to
    # the glass by short saddles and vertical webs.
    for x, y_sign, shoe_name, saddle_name, web_name, clamp_name in (
        (-0.680, 1.0, "guide_shoe_0", "shoe_saddle_0", "shoe_web_0", "glass_clamp_0"),
        (-0.680, -1.0, "guide_shoe_1", "shoe_saddle_1", "shoe_web_1", "glass_clamp_1"),
        (0.680, 1.0, "guide_shoe_2", "shoe_saddle_2", "shoe_web_2", "glass_clamp_2"),
        (0.680, -1.0, "guide_shoe_3", "shoe_saddle_3", "shoe_web_3", "glass_clamp_3"),
    ):
        y = 0.500 * y_sign
        glass.visual(
            Box((0.120, 0.050, 0.022)),
            origin=Origin(xyz=(x, y, 0.049)),
            material=nylon,
            name=shoe_name,
        )
        glass.visual(
            Box((0.110, 0.060, 0.009)),
            origin=Origin(xyz=(x, 0.458 * y_sign, 0.064)),
            material=nylon,
            name=saddle_name,
        )
        glass.visual(
            Box((0.090, 0.014, 0.048)),
            origin=Origin(xyz=(x, 0.429 * y_sign, 0.079)),
            material=nylon,
            name=web_name,
        )
        glass.visual(
            Box((0.115, 0.035, 0.010)),
            origin=Origin(xyz=(x, 0.438 * y_sign, 0.088)),
            material=black_ceramic,
            name=clamp_name,
        )

    shade = model.part("sun_shade")
    shade.visual(
        Box((1.52, 0.780, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.066)),
        material=fabric,
        name="fabric_panel",
    )
    shade.visual(
        Box((0.050, 0.830, 0.024)),
        origin=Origin(xyz=(-0.785, 0.0, -0.054)),
        material=dark_anodized,
        name="pull_bar",
    )
    shade.visual(
        Box((1.54, 0.020, 0.009)),
        origin=Origin(xyz=(0.0, 0.394, -0.060)),
        material=fabric_seam,
        name="edge_binding_0",
    )
    shade.visual(
        Box((1.54, 0.020, 0.009)),
        origin=Origin(xyz=(0.0, -0.394, -0.060)),
        material=fabric_seam,
        name="edge_binding_1",
    )
    for i, x in enumerate((-0.48, 0.0, 0.48)):
        shade.visual(
            Box((0.018, 0.760, 0.004)),
            origin=Origin(xyz=(x, 0.0, -0.061)),
            material=fabric_seam,
            name=f"fabric_bow_{i}",
        )
    for x, y, slider_name in (
        (-0.700, 0.414, "shade_slider_0_0"),
        (-0.700, -0.414, "shade_slider_0_1"),
        (0.700, 0.414, "shade_slider_1_0"),
        (0.700, -0.414, "shade_slider_1_1"),
    ):
        shade.visual(
            Box((0.260, 0.026, 0.014)),
            origin=Origin(xyz=(x, y, -0.050)),
            material=nylon,
            name=slider_name,
        )

    model.articulation(
        "glass_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=glass,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.50),
    )
    model.articulation(
        "shade_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=shade,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.45, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("cassette_frame")
    glass = object_model.get_part("glass_panel")
    shade = object_model.get_part("sun_shade")
    glass_slide = object_model.get_articulation("glass_slide")
    shade_slide = object_model.get_articulation("shade_slide")

    ctx.expect_within(
        glass,
        frame,
        axes="y",
        inner_elem="glass_lite",
        margin=0.03,
        name="glass spans between side frame extrusions",
    )
    ctx.expect_overlap(
        glass,
        frame,
        axes="x",
        elem_a="guide_shoe_0",
        elem_b="glass_rail_base_0",
        min_overlap=0.10,
        name="front guide shoe sits on upper rail",
    )
    ctx.expect_gap(
        glass,
        frame,
        axis="z",
        positive_elem="glass_lite",
        negative_elem="side_frame_0",
        min_gap=0.060,
        max_gap=0.090,
        name="glass rides above aluminium frame",
    )
    ctx.expect_gap(
        frame,
        shade,
        axis="z",
        positive_elem="side_frame_0",
        negative_elem="fabric_panel",
        min_gap=0.040,
        max_gap=0.060,
        name="shade lies below the cassette frame",
    )

    rest_glass_pos = ctx.part_world_position(glass)
    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({glass_slide: 0.50, shade_slide: 0.55}):
        ctx.expect_overlap(
            glass,
            frame,
            axes="x",
            elem_a="guide_shoe_3",
            elem_b="glass_rail_base_1",
            min_overlap=0.08,
            name="opened rear guide shoe remains captured on rail",
        )
        ctx.expect_overlap(
            shade,
            frame,
            axes="x",
            elem_a="shade_slider_1_1",
            elem_b="shade_rail_1",
            min_overlap=0.08,
            name="retracted shade slider remains on lower rail",
        )
        open_glass_pos = ctx.part_world_position(glass)
        open_shade_pos = ctx.part_world_position(shade)

    ctx.check(
        "glass slide moves rearward",
        rest_glass_pos is not None
        and open_glass_pos is not None
        and open_glass_pos[0] > rest_glass_pos[0] + 0.45,
        details=f"rest={rest_glass_pos}, open={open_glass_pos}",
    )
    ctx.check(
        "sun shade retracts on its own rail",
        rest_shade_pos is not None
        and open_shade_pos is not None
        and open_shade_pos[0] > rest_shade_pos[0] + 0.50,
        details=f"rest={rest_shade_pos}, open={open_shade_pos}",
    )

    return ctx.report()


object_model = build_object_model()
