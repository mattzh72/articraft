from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _offset_profile(profile, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_hard_panel_sunroof_cassette")

    roof_paint = Material("dark_tinted_roof_paint", rgba=(0.025, 0.028, 0.032, 1.0))
    black_plastic = Material("black_plastic_and_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    satin_aluminum = Material("satin_brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    anodized_rail = Material("dark_anodized_aluminum", rgba=(0.11, 0.115, 0.12, 1.0))

    cassette = model.part("cassette")

    # Coordinates: X across the vehicle, +Y rearward, +Z upward.  The fixed
    # roof sheet is a thin panel with a large forward aperture and a solid rear
    # roof section above the storage cavity.
    roof_outer = rounded_rect_profile(1.24, 1.70, 0.060, corner_segments=10)
    opening = _offset_profile(
        rounded_rect_profile(0.84, 0.72, 0.045, corner_segments=8),
        dy=-0.30,
    )
    roof_skin = ExtrudeWithHolesGeometry(roof_outer, [opening], 0.018, center=True)
    cassette.visual(
        mesh_from_geometry(roof_skin, "roof_skin"),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=roof_paint,
        name="roof_skin",
    )

    rail_y = 0.050
    rail_len = 1.55
    cassette.visual(
        Box((0.040, rail_len, 0.066)),
        origin=Origin(xyz=(-0.495, rail_y, 0.053)),
        material=anodized_rail,
        name="guide_rail_0_web",
    )
    cassette.visual(
        Box((0.085, rail_len, 0.012)),
        origin=Origin(xyz=(-0.4325, rail_y, 0.064)),
        material=anodized_rail,
        name="guide_rail_0_upper",
    )
    cassette.visual(
        Box((0.085, rail_len, 0.013)),
        origin=Origin(xyz=(-0.4325, rail_y, 0.0325)),
        material=anodized_rail,
        name="guide_rail_0_lower",
    )
    cassette.visual(
        Box((0.040, rail_len, 0.066)),
        origin=Origin(xyz=(0.495, rail_y, 0.053)),
        material=anodized_rail,
        name="guide_rail_1_web",
    )
    cassette.visual(
        Box((0.085, rail_len, 0.012)),
        origin=Origin(xyz=(0.4325, rail_y, 0.064)),
        material=anodized_rail,
        name="guide_rail_1_upper",
    )
    cassette.visual(
        Box((0.085, rail_len, 0.013)),
        origin=Origin(xyz=(0.4325, rail_y, 0.0325)),
        material=anodized_rail,
        name="guide_rail_1_lower",
    )

    cassette.visual(
        Box((0.92, 0.75, 0.016)),
        origin=Origin(xyz=(0.0, 0.45, 0.018)),
        material=black_plastic,
        name="rear_cavity_floor",
    )
    cassette.visual(
        Box((0.92, 0.050, 0.060)),
        origin=Origin(xyz=(0.0, 0.825, 0.056)),
        material=black_plastic,
        name="rear_cavity_stop",
    )
    cassette.visual(
        Box((0.92, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, -0.735, 0.041)),
        material=black_plastic,
        name="front_gutter",
    )

    panel = model.part("panel")

    panel_plate = ExtrudeGeometry(
        rounded_rect_profile(0.76, 0.68, 0.035, corner_segments=8),
        0.026,
        center=True,
    )
    panel.visual(
        mesh_from_geometry(panel_plate, "aluminum_panel"),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=satin_aluminum,
        name="aluminum_panel",
    )

    seal_ring = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.80, 0.70, 0.040, corner_segments=8),
        [rounded_rect_profile(0.68, 0.58, 0.030, corner_segments=8)],
        0.004,
        center=True,
    )
    panel.visual(
        mesh_from_geometry(seal_ring, "weatherstrip"),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=black_plastic,
        name="weatherstrip",
    )

    panel.visual(
        Box((0.064, 0.60, 0.014)),
        origin=Origin(xyz=(-0.410, 0.0, 0.0)),
        material=black_plastic,
        name="guide_shoe_0",
    )
    panel.visual(
        Box((0.064, 0.60, 0.014)),
        origin=Origin(xyz=(0.410, 0.0, 0.0)),
        material=black_plastic,
        name="guide_shoe_1",
    )

    for suffix, y in (("0", -0.22), ("1", 0.22)):
        panel.visual(
            Box((0.64, 0.030, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.008)),
            material=satin_aluminum,
            name=f"underside_rib_{suffix}",
        )

    model.articulation(
        "cassette_to_panel",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=panel,
        # The child frame lies on the shoe centerline in the closed position.
        origin=Origin(xyz=(0.0, -0.30, 0.046)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.72),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette = object_model.get_part("cassette")
    panel = object_model.get_part("panel")
    slide = object_model.get_articulation("cassette_to_panel")

    ctx.expect_gap(
        cassette,
        panel,
        positive_elem="roof_skin",
        negative_elem="weatherstrip",
        axis="z",
        min_gap=0.001,
        max_gap=0.008,
        name="closed weatherstrip clears underside of roof skin",
    )
    ctx.expect_gap(
        panel,
        cassette,
        positive_elem="guide_shoe_0",
        negative_elem="guide_rail_0_lower",
        axis="z",
        min_gap=0.0,
        max_gap=0.003,
        name="closed left shoe rides on lower guide rail",
    )
    ctx.expect_gap(
        panel,
        cassette,
        positive_elem="guide_shoe_1",
        negative_elem="guide_rail_1_lower",
        axis="z",
        min_gap=0.0,
        max_gap=0.003,
        name="closed right shoe rides on lower guide rail",
    )
    ctx.expect_overlap(
        panel,
        cassette,
        elem_a="guide_shoe_0",
        elem_b="guide_rail_0_lower",
        axes="y",
        min_overlap=0.55,
        name="closed left shoe is retained in rail length",
    )
    ctx.expect_overlap(
        panel,
        cassette,
        elem_a="guide_shoe_1",
        elem_b="guide_rail_1_lower",
        axes="y",
        min_overlap=0.55,
        name="closed right shoe is retained in rail length",
    )

    rest_pos = ctx.part_world_position(panel)
    with ctx.pose({slide: 0.72}):
        ctx.expect_within(
            panel,
            cassette,
            inner_elem="aluminum_panel",
            outer_elem="rear_cavity_floor",
            axes="xy",
            margin=0.012,
            name="open panel fits in rear roof cavity footprint",
        )
        ctx.expect_gap(
            cassette,
            panel,
            positive_elem="roof_skin",
            negative_elem="weatherstrip",
            axis="z",
            min_gap=0.001,
            max_gap=0.008,
            name="open weatherstrip clears underside of rear roof",
        )
        ctx.expect_overlap(
            panel,
            cassette,
            elem_a="guide_shoe_0",
            elem_b="guide_rail_0_lower",
            axes="y",
            min_overlap=0.55,
            name="open left shoe remains captured in rail",
        )
        ctx.expect_overlap(
            panel,
            cassette,
            elem_a="guide_shoe_1",
            elem_b="guide_rail_1_lower",
            axes="y",
            min_overlap=0.55,
            name="open right shoe remains captured in rail",
        )
        open_pos = ctx.part_world_position(panel)

    ctx.check(
        "panel translates rearward into cassette",
        rest_pos is not None and open_pos is not None and open_pos[1] > rest_pos[1] + 0.65,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
