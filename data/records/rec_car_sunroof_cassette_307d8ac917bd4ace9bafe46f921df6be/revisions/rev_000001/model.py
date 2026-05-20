from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_plate(length: float, width: float, thickness: float, radius: float):
    """CadQuery rounded rectangle plate centered on the local origin."""
    sketch = cq.Sketch().rect(length, width).vertices().fillet(radius)
    return cq.Workplane("XY").placeSketch(sketch).extrude(thickness).translate(
        (0.0, 0.0, -thickness / 2.0)
    )


def _rounded_ring(
    outer_length: float,
    outer_width: float,
    inner_length: float,
    inner_width: float,
    thickness: float,
    outer_radius: float,
    inner_radius: float,
):
    """CadQuery rectangular ring with rounded outside and aperture corners."""
    sketch = (
        cq.Sketch()
        .rect(outer_length, outer_width, tag="outer")
        .vertices(tag="outer")
        .fillet(outer_radius)
        .reset()
        .rect(inner_length, inner_width, mode="s", tag="inner")
        .vertices(tag="inner")
        .fillet(inner_radius)
    )
    return cq.Workplane("XY").placeSketch(sketch).extrude(thickness).translate(
        (0.0, 0.0, -thickness / 2.0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_sunroof_cassette")

    e_coat = model.material("satin_black_e_coat", rgba=(0.015, 0.017, 0.018, 1.0))
    rubber = model.material("molded_black_rubber", rgba=(0.002, 0.002, 0.002, 1.0))
    rail_metal = model.material("brushed_anodized_aluminum", rgba=(0.55, 0.57, 0.56, 1.0))
    drain_plastic = model.material("black_drain_plastic", rgba=(0.018, 0.018, 0.016, 1.0))
    fastener = model.material("zinc_fastener_heads", rgba=(0.64, 0.62, 0.55, 1.0))
    tinted_glass = model.material("smoked_tempered_glass", rgba=(0.02, 0.07, 0.10, 0.45))
    frit = model.material("black_ceramic_frit", rgba=(0.0, 0.0, 0.0, 0.92))

    cassette = model.part("cassette")

    roof_frame = _rounded_ring(
        outer_length=1.90,
        outer_width=1.10,
        inner_length=1.24,
        inner_width=0.80,
        thickness=0.028,
        outer_radius=0.080,
        inner_radius=0.055,
    )
    cassette.visual(
        mesh_from_cadquery(roof_frame, "roof_frame", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=e_coat,
        name="roof_frame",
    )

    seal_ring = _rounded_ring(
        outer_length=1.24,
        outer_width=0.80,
        inner_length=1.08,
        inner_width=0.71,
        thickness=0.018,
        outer_radius=0.050,
        inner_radius=0.040,
    )
    cassette.visual(
        mesh_from_cadquery(seal_ring, "rubber_seal", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=rubber,
        name="rubber_seal",
    )

    cassette.visual(
        Box((0.54, 0.70, 0.018)),
        origin=Origin(xyz=(0.725, 0.0, 0.035)),
        material=e_coat,
        name="rear_trough",
    )
    cassette.visual(
        Box((0.44, 0.28, 0.030)),
        origin=Origin(xyz=(-0.775, 0.0, 0.046)),
        material=drain_plastic,
        name="motor_housing",
    )
    cassette.visual(
        Cylinder(radius=0.017, length=0.88),
        origin=Origin(xyz=(-0.775, 0.0, 0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drain_plastic,
        name="drive_cable_tube",
    )

    for side_index, side in enumerate((1.0, -1.0)):
        y = side * 0.440
        cassette.visual(
            Box((1.70, 0.130, 0.024)),
            origin=Origin(xyz=(0.100, y, 0.038)),
            material=rail_metal,
            name=f"rail_base_{side_index}",
        )
        cassette.visual(
            Box((1.66, 0.070, 0.006)),
            origin=Origin(xyz=(0.110, y, 0.053)),
            material=rubber,
            name=f"guide_lane_{side_index}",
        )
        cassette.visual(
            Box((1.62, 0.016, 0.042)),
            origin=Origin(xyz=(0.110, side * 0.380, 0.069)),
            material=rail_metal,
            name=f"guide_lip_inner_{side_index}",
        )
        cassette.visual(
            Box((1.62, 0.016, 0.042)),
            origin=Origin(xyz=(0.110, side * 0.500, 0.069)),
            material=rail_metal,
            name=f"guide_lip_outer_{side_index}",
        )

        for screw_index, x in enumerate((-0.55, -0.12, 0.35, 0.78)):
            cassette.visual(
                Cylinder(radius=0.018, length=0.008),
                origin=Origin(xyz=(x, y, 0.054)),
                material=fastener,
                name=f"screw_head_{side_index}_{screw_index}",
            )

    for drain_index, (x, y) in enumerate(
        ((-0.780, 0.455), (0.770, 0.455), (-0.780, -0.455), (0.770, -0.455))
    ):
        side = 1.0 if y > 0.0 else -1.0
        cassette.visual(
            Cylinder(radius=0.033, length=0.042),
            origin=Origin(xyz=(x, y, 0.047)),
            material=drain_plastic,
            name=f"drain_pot_{drain_index}",
        )
        cassette.visual(
            Cylinder(radius=0.011, length=0.190),
            origin=Origin(xyz=(x, side * 0.575, 0.044), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=drain_plastic,
            name=f"drain_nipple_{drain_index}",
        )

    glass = model.part("glass_panel")
    glass.visual(
        mesh_from_cadquery(_rounded_plate(1.05, 0.68, 0.018, 0.055), "glass_pane"),
        material=tinted_glass,
        name="glass_pane",
    )
    glass.visual(
        mesh_from_cadquery(
            _rounded_ring(1.05, 0.68, 0.87, 0.50, 0.006, 0.055, 0.040),
            "ceramic_border",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=frit,
        name="ceramic_border",
    )
    glass.visual(
        Box((0.30, 0.045, 0.004)),
        origin=Origin(xyz=(-0.485, 0.0, 0.011)),
        material=frit,
        name="front_frit_band",
    )

    for side_index, side in enumerate((1.0, -1.0)):
        for station_name, x in (("front", -0.340), ("rear", 0.340)):
            glass.visual(
                Box((0.190, 0.055, 0.014)),
                origin=Origin(xyz=(x, side * 0.440, 0.041)),
                material=rail_metal,
                name=f"shoe_{station_name}_{side_index}",
            )
            glass.visual(
                Box((0.205, 0.110, 0.010)),
                origin=Origin(xyz=(x, side * 0.395, 0.031)),
                material=rail_metal,
                name=f"support_arm_{station_name}_{side_index}",
            )
            glass.visual(
                Box((0.120, 0.030, 0.052)),
                origin=Origin(xyz=(x, side * 0.335, 0.016)),
                material=frit,
                name=f"edge_tab_{station_name}_{side_index}",
            )

    model.articulation(
        "cassette_to_glass",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=glass,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.420),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette = object_model.get_part("cassette")
    glass = object_model.get_part("glass_panel")
    slide = object_model.get_articulation("cassette_to_glass")

    ctx.expect_within(
        glass,
        cassette,
        axes="xy",
        inner_elem="glass_pane",
        outer_elem="rubber_seal",
        margin=0.001,
        name="closed glass is centered inside the roof aperture seal",
    )
    ctx.expect_gap(
        glass,
        cassette,
        axis="z",
        positive_elem="glass_pane",
        negative_elem="rubber_seal",
        min_gap=0.004,
        max_gap=0.014,
        name="closed glass sits just proud of the rubber seal",
    )
    for side_index in (0, 1):
        ctx.expect_within(
            glass,
            cassette,
            axes="y",
            inner_elem=f"shoe_front_{side_index}",
            outer_elem=f"rail_base_{side_index}",
            margin=0.002,
            name=f"front carrier shoe {side_index} stays in its guide lane",
        )
        ctx.expect_gap(
            glass,
            cassette,
            axis="z",
            positive_elem=f"shoe_front_{side_index}",
            negative_elem=f"guide_lip_inner_{side_index}",
            min_gap=0.004,
            max_gap=0.018,
            name=f"front carrier shoe {side_index} rides above rail lip",
        )

    rest_pos = ctx.part_world_position(glass)
    with ctx.pose({slide: 0.420}):
        for side_index in (0, 1):
            ctx.expect_within(
                glass,
                cassette,
                axes="y",
                inner_elem=f"shoe_rear_{side_index}",
                outer_elem=f"rail_base_{side_index}",
                margin=0.002,
                name=f"extended rear carrier shoe {side_index} remains laterally guided",
            )
            ctx.expect_overlap(
                glass,
                cassette,
                axes="x",
                elem_a=f"shoe_rear_{side_index}",
                elem_b=f"rail_base_{side_index}",
                min_overlap=0.080,
                name=f"extended rear carrier shoe {side_index} remains on the rail",
            )
        extended_pos = ctx.part_world_position(glass)

    ctx.check(
        "glass panel slides rearward along cassette rails",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.35,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
