from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="flush_sunroof_cassette")

    aluminium = model.material("brushed_aluminium", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_anodized = model.material("dark_anodized_track", rgba=(0.08, 0.09, 0.10, 1.0))
    black_rubber = model.material("black_rubber_seal", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("smoked_blue_glass", rgba=(0.12, 0.24, 0.32, 0.44))
    ceramic = model.material("black_ceramic_frit", rgba=(0.01, 0.012, 0.014, 1.0))
    hinge_metal = model.material("hinge_stainless", rgba=(0.55, 0.57, 0.58, 1.0))

    # Object frame: +X is rearward, +Y is across the roof opening, +Z is up.
    outer_x = 1.35
    outer_y = 0.86
    frame_h = 0.062
    opening_x = 0.95
    opening_y = 0.58
    rear_extension = 0.36
    cross_w = (outer_x - opening_x) * 0.5
    side_w = (outer_y - opening_y) * 0.5
    rear_frame_x = outer_x * 0.5 - cross_w * 0.5 + rear_extension

    cassette = model.part("cassette")
    # A four-piece rectangular aluminium frame with overlapping, welded-looking
    # corners leaves a real open aperture for the glass panel.
    cassette.visual(
        Box((cross_w, outer_y, frame_h)),
        origin=Origin(xyz=(-outer_x * 0.5 + cross_w * 0.5, 0.0, 0.0)),
        material=aluminium,
        name="front_frame",
    )
    cassette.visual(
        Box((cross_w, outer_y, frame_h)),
        origin=Origin(xyz=(rear_frame_x, 0.0, 0.0)),
        material=aluminium,
        name="rear_frame",
    )
    cassette.visual(
        Box((opening_x + rear_extension + 0.012, side_w, frame_h)),
        origin=Origin(xyz=(rear_extension * 0.5, outer_y * 0.5 - side_w * 0.5, 0.0)),
        material=aluminium,
        name="side_frame_0",
    )
    cassette.visual(
        Box((opening_x + rear_extension + 0.012, side_w, frame_h)),
        origin=Origin(xyz=(rear_extension * 0.5, -outer_y * 0.5 + side_w * 0.5, 0.0)),
        material=aluminium,
        name="side_frame_1",
    )

    # Seals sit on the inner lip; the dark guide rails are slightly below the
    # flush top face where the moving shoe carriage rides.
    seal_z = 0.034
    cassette.visual(
        Box((opening_x + 0.020, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, opening_y * 0.5 + 0.010, seal_z)),
        material=black_rubber,
        name="side_seal_0",
    )
    cassette.visual(
        Box((opening_x + 0.020, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, -opening_y * 0.5 - 0.010, seal_z)),
        material=black_rubber,
        name="side_seal_1",
    )
    cassette.visual(
        Box((0.028, opening_y + 0.052, 0.006)),
        origin=Origin(xyz=(-opening_x * 0.5 - 0.010, 0.0, seal_z)),
        material=black_rubber,
        name="front_seal",
    )
    cassette.visual(
        Box((0.028, opening_y + 0.052, 0.006)),
        origin=Origin(xyz=(opening_x * 0.5 + 0.010, 0.0, seal_z)),
        material=black_rubber,
        name="rear_seal",
    )
    rail_z = -0.021
    cassette.visual(
        Box((opening_x * 0.96 + rear_extension, 0.032, 0.018)),
        origin=Origin(xyz=(0.010 + rear_extension * 0.5, opening_y * 0.5 - 0.016, rail_z)),
        material=dark_anodized,
        name="guide_rail_0",
    )
    cassette.visual(
        Box((opening_x * 0.96 + rear_extension, 0.032, 0.018)),
        origin=Origin(xyz=(0.010 + rear_extension * 0.5, -opening_y * 0.5 + 0.016, rail_z)),
        material=dark_anodized,
        name="guide_rail_1",
    )
    cassette.visual(
        Box((opening_x + rear_extension, 0.018, 0.014)),
        origin=Origin(xyz=(-0.015 + rear_extension * 0.5, 0.0, -0.021)),
        material=dark_anodized,
        name="center_drain_trough",
    )

    slide_carriage = model.part("slide_carriage")
    # The carriage frame is located on the glass front hinge line.  It carries
    # two shoes riding on the fixed guide rails plus cross-ties and a hinge pin.
    slide_carriage.visual(
        Cylinder(radius=0.010, length=0.540),
        origin=Origin(xyz=(-0.010, 0.0, 0.017), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="hinge_pin",
    )
    slide_carriage.visual(
        Box((0.055, 0.490, 0.018)),
        origin=Origin(xyz=(0.026, 0.0, -0.022)),
        material=dark_anodized,
        name="front_cross_tie",
    )
    slide_carriage.visual(
        Box((0.055, 0.490, 0.016)),
        origin=Origin(xyz=(0.760, 0.0, -0.022)),
        material=dark_anodized,
        name="rear_cross_tie",
    )
    slide_carriage.visual(
        Box((0.720, 0.030, 0.014)),
        origin=Origin(xyz=(0.390, opening_y * 0.5 - 0.045, -0.022)),
        material=dark_anodized,
        name="guide_shoe_0",
    )
    slide_carriage.visual(
        Box((0.720, 0.030, 0.014)),
        origin=Origin(xyz=(0.390, -opening_y * 0.5 + 0.045, -0.022)),
        material=dark_anodized,
        name="guide_shoe_1",
    )
    slide_carriage.visual(
        Box((0.060, 0.040, 0.034)),
        origin=Origin(xyz=(0.018, opening_y * 0.5 - 0.030, -0.004)),
        material=hinge_metal,
        name="hinge_block_0",
    )
    slide_carriage.visual(
        Box((0.060, 0.040, 0.034)),
        origin=Origin(xyz=(0.018, -opening_y * 0.5 + 0.030, -0.004)),
        material=hinge_metal,
        name="hinge_block_1",
    )

    glass_panel = model.part("glass_panel")
    panel_x = 0.86
    panel_y = 0.50
    # Panel geometry extends in +X from its front hinge axis.  The black ceramic
    # frit is on the upper surface and makes the panel top flush with the frame.
    glass_panel.visual(
        Box((panel_x - 0.052, panel_y - 0.052, 0.014)),
        origin=Origin(xyz=(panel_x * 0.5 + 0.010, 0.0, -0.008)),
        material=glass,
        name="glass_sheet",
    )
    glass_panel.visual(
        Box((panel_x, 0.040, 0.004)),
        origin=Origin(xyz=(panel_x * 0.5 + 0.010, panel_y * 0.5 - 0.020, 0.001)),
        material=ceramic,
        name="side_frit_0",
    )
    glass_panel.visual(
        Box((panel_x, 0.040, 0.004)),
        origin=Origin(xyz=(panel_x * 0.5 + 0.010, -panel_y * 0.5 + 0.020, 0.001)),
        material=ceramic,
        name="side_frit_1",
    )
    glass_panel.visual(
        Box((0.026, panel_y, 0.004)),
        origin=Origin(xyz=(0.023, 0.0, 0.001)),
        material=ceramic,
        name="front_frit",
    )
    glass_panel.visual(
        Box((0.026, panel_y, 0.004)),
        origin=Origin(xyz=(panel_x - 0.003, 0.0, 0.001)),
        material=ceramic,
        name="rear_frit",
    )
    glass_panel.visual(
        Box((0.035, 0.470, 0.010)),
        origin=Origin(xyz=(0.028, 0.0, -0.020)),
        material=hinge_metal,
        name="glass_hinge_leaf",
    )

    hinge_x = -panel_x * 0.5
    model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=slide_carriage,
        origin=Origin(xyz=(hinge_x, 0.0, 0.017)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.36),
    )
    model.articulation(
        "panel_tilt",
        ArticulationType.REVOLUTE,
        parent=slide_carriage,
        child=glass_panel,
        origin=Origin(xyz=(-0.010, 0.0, 0.017)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette = object_model.get_part("cassette")
    carriage = object_model.get_part("slide_carriage")
    panel = object_model.get_part("glass_panel")
    rail_slide = object_model.get_articulation("rail_slide")
    panel_tilt = object_model.get_articulation("panel_tilt")

    ctx.expect_contact(
        carriage,
        cassette,
        elem_a="guide_shoe_0",
        elem_b="guide_rail_0",
        contact_tol=0.002,
        name="upper guide shoe rides on rail",
    )
    ctx.expect_contact(
        carriage,
        cassette,
        elem_a="guide_shoe_1",
        elem_b="guide_rail_1",
        contact_tol=0.002,
        name="lower guide shoe rides on rail",
    )

    panel_aabb = ctx.part_world_aabb(panel)
    cassette_aabb = ctx.part_world_aabb(cassette)
    if panel_aabb is not None and cassette_aabb is not None:
        panel_min, panel_max = panel_aabb
        cassette_min, cassette_max = cassette_aabb
        ctx.check(
            "glass panel nests inside frame opening at rest",
            panel_min[0] > -0.475 and panel_max[0] < 0.475 and panel_min[1] > -0.290 and panel_max[1] < 0.290,
            details=f"panel_aabb={panel_aabb!r}",
        )
        ctx.check(
            "glass top is nearly flush with aluminium frame top",
            abs(panel_max[2] - cassette_max[2]) <= 0.006,
            details=f"panel_top={panel_max[2]:.4f}, frame_top={cassette_max[2]:.4f}",
        )

    rest_panel = ctx.part_world_aabb(panel)
    with ctx.pose({panel_tilt: 0.16}):
        tilted_panel = ctx.part_world_aabb(panel)
    if rest_panel is not None and tilted_panel is not None:
        ctx.check(
            "rear edge tilts upward",
            tilted_panel[1][2] > rest_panel[1][2] + 0.07,
            details=f"rest={rest_panel!r}, tilted={tilted_panel!r}",
        )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({rail_slide: 0.36}):
        slid_carriage_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage slides rearward along guide rails",
        rest_carriage_pos is not None
        and slid_carriage_pos is not None
        and slid_carriage_pos[0] > rest_carriage_pos[0] + 0.30,
        details=f"rest={rest_carriage_pos!r}, slid={slid_carriage_pos!r}",
    )

    return ctx.report()


object_model = build_object_model()
