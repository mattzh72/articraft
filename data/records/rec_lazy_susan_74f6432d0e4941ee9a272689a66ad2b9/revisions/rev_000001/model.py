from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _lathe(profile: list[tuple[float, float]], name: str):
    """Build a managed annular mesh from a radius/z cross-section."""
    return mesh_from_geometry(LatheGeometry(profile, segments=128), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_display_lazy_susan")

    satin_graphite = model.material("satin_graphite", rgba=(0.055, 0.058, 0.060, 1.0))
    rim_graphite = model.material("knurled_dark_rim", rgba=(0.030, 0.032, 0.034, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.63, 0.60, 1.0))
    bearing_steel = model.material("shadowed_bearing_steel", rgba=(0.12, 0.13, 0.135, 1.0))

    base_ring = model.part("base_ring")
    base_ring.visual(
        _lathe(
            [
                (0.060, 0.000),
                (0.162, 0.000),
                (0.162, 0.010),
                (0.060, 0.010),
            ],
            "base_low_ring",
        ),
        material=brushed_aluminum,
        name="low_ring",
    )
    base_ring.visual(
        _lathe(
            [
                (0.148, 0.009),
                (0.162, 0.009),
                (0.162, 0.018),
                (0.148, 0.018),
            ],
            "base_outer_wall",
        ),
        material=brushed_aluminum,
        name="outer_wall",
    )
    base_ring.visual(
        _lathe(
            [
                (0.139, 0.016),
                (0.162, 0.016),
                (0.162, 0.018),
                (0.139, 0.018),
            ],
            "base_clip_ledge",
        ),
        material=brushed_aluminum,
        name="clip_ledge",
    )
    base_ring.visual(
        _lathe(
            [
                (0.060, 0.009),
                (0.094, 0.009),
                (0.094, 0.017),
                (0.060, 0.017),
            ],
            "base_inner_race",
        ),
        material=brushed_aluminum,
        name="inner_race",
    )
    base_ring.visual(
        _lathe(
            [
                (0.101, 0.0095),
                (0.127, 0.0095),
                (0.127, 0.0165),
                (0.101, 0.0165),
            ],
            "base_bearing_band",
        ),
        material=bearing_steel,
        name="bearing_band",
    )

    upper_disc = model.part("upper_disc")
    upper_disc.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.340,
                0.024,
                body_style="cylindrical",
                edge_radius=0.0015,
                grip=KnobGrip(
                    style="knurled",
                    count=96,
                    depth=0.0022,
                    helix_angle_deg=24.0,
                ),
                center=False,
            ),
            "upper_disc_knurled_body",
        ),
        material=rim_graphite,
        name="knurled_body",
    )
    upper_disc.visual(
        _lathe(
            [
                (0.000, 0.0226),
                (0.153, 0.0226),
                (0.153, 0.0246),
                (0.000, 0.0246),
            ],
            "upper_disc_smooth_top",
        ),
        material=satin_graphite,
        name="smooth_top",
    )
    upper_disc.visual(
        _lathe(
            [
                (0.132, -0.0065),
                (0.137, -0.0065),
                (0.137, 0.0030),
                (0.132, 0.0030),
            ],
            "upper_disc_retainer_stem",
        ),
        material=bearing_steel,
        name="retainer_stem",
    )
    upper_disc.visual(
        _lathe(
            [
                (0.134, -0.0080),
                (0.145, -0.0080),
                (0.145, -0.0060),
                (0.134, -0.0060),
            ],
            "upper_disc_retainer_lip",
        ),
        material=bearing_steel,
        name="retainer_lip",
    )
    upper_disc.visual(
        _lathe(
            [
                (0.088, -0.0035),
                (0.126, -0.0035),
                (0.126, 0.0005),
                (0.088, 0.0005),
            ],
            "upper_disc_upper_race",
        ),
        material=bearing_steel,
        name="upper_race",
    )

    model.articulation(
        "base_to_disc",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=upper_disc,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_ring")
    disc = object_model.get_part("upper_disc")
    turntable = object_model.get_articulation("base_to_disc")

    ctx.check(
        "upper disc has continuous vertical rotation",
        turntable.articulation_type == ArticulationType.CONTINUOUS
        and tuple(turntable.axis) == (0.0, 0.0, 1.0),
        details=f"type={turntable.articulation_type}, axis={turntable.axis}",
    )

    ctx.expect_gap(
        disc,
        base,
        axis="z",
        positive_elem="knurled_body",
        negative_elem="clip_ledge",
        min_gap=0.001,
        max_gap=0.004,
        name="upper disc sits just above the fixed base ring",
    )
    ctx.expect_gap(
        base,
        disc,
        axis="z",
        positive_elem="clip_ledge",
        negative_elem="retainer_lip",
        min_gap=0.001,
        max_gap=0.004,
        name="fixed ledge captures the underside retaining lip",
    )
    ctx.expect_overlap(
        base,
        disc,
        axes="xy",
        elem_a="clip_ledge",
        elem_b="retainer_lip",
        min_overlap=0.006,
        name="retaining lip tucks under the base ledge",
    )
    ctx.expect_gap(
        disc,
        base,
        axis="z",
        positive_elem="upper_race",
        negative_elem="bearing_band",
        min_gap=0.0,
        max_gap=0.001,
        name="compact bearing is hidden between the races",
    )

    with ctx.pose({turntable: 3.14159}):
        ctx.expect_gap(
            disc,
            base,
            axis="z",
            positive_elem="knurled_body",
            negative_elem="clip_ledge",
            min_gap=0.001,
            max_gap=0.004,
            name="disc clearance remains constant while rotating",
        )

    return ctx.report()


object_model = build_object_model()
