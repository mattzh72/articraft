from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_monitor_stand_lazy_susan")

    satin_black = model.material("satin_black", rgba=(0.025, 0.026, 0.028, 1.0))
    charcoal_edge = model.material("charcoal_edge", rgba=(0.055, 0.058, 0.062, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.62, 0.43, 0.24, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.73, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.010, 0.011, 0.012, 1.0))

    top_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.62, 0.34, radius=0.025, corner_segments=8),
            0.035,
        ),
        "rounded_rectangular_top_plate",
    )
    lower_race_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.112, tube=0.008, radial_segments=16, tubular_segments=72),
        "lower_bearing_race",
    )
    upper_race_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.112, tube=0.008, radial_segments=16, tubular_segments=72),
        "upper_bearing_race",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.160, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=satin_black,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.150, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_rubber,
        name="rubber_foot_ring",
    )
    base.visual(
        lower_race_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=brushed_steel,
        name="lower_bearing_race",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=brushed_steel,
        name="center_bearing_boss",
    )

    platform = model.part("platform")
    platform.visual(
        upper_race_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=brushed_steel,
        name="upper_bearing_race",
    )
    platform.visual(
        Cylinder(radius=0.136, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=satin_black,
        name="rotating_carrier_disc",
    )
    for index, (x, y) in enumerate(
        (
            (-0.082, -0.046),
            (-0.082, 0.046),
            (0.082, -0.046),
            (0.082, 0.046),
        )
    ):
        platform.visual(
            Box((0.040, 0.034, 0.014)),
            origin=Origin(xyz=(x, y, 0.032)),
            material=satin_black,
            name=f"standoff_{index}",
        )
    platform.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0565)),
        material=warm_wood,
        name="top_plate",
    )
    platform.visual(
        Box((0.620, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.170, 0.0565)),
        material=charcoal_edge,
        name="long_edge_band_0",
    )
    platform.visual(
        Box((0.620, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.170, 0.0565)),
        material=charcoal_edge,
        name="long_edge_band_1",
    )
    platform.visual(
        Box((0.018, 0.304, 0.030)),
        origin=Origin(xyz=(0.310, 0.0, 0.0565)),
        material=charcoal_edge,
        name="short_edge_band_0",
    )
    platform.visual(
        Box((0.018, 0.304, 0.030)),
        origin=Origin(xyz=(-0.310, 0.0, 0.0565)),
        material=charcoal_edge,
        name="short_edge_band_1",
    )
    for index, (x, y) in enumerate(
        (
            (-0.082, -0.046),
            (-0.082, 0.046),
            (0.082, -0.046),
            (0.082, 0.046),
        )
    ):
        platform.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(x, y, 0.076)),
            material=charcoal_edge,
            name=f"recessed_screw_{index}",
        )

    model.articulation(
        "center_bearing",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platform = object_model.get_part("platform")
    bearing = object_model.get_articulation("center_bearing")

    ctx.expect_within(
        base,
        platform,
        axes="xy",
        inner_elem="base_disc",
        outer_elem="top_plate",
        margin=0.003,
        name="round base is centered below the rectangular top",
    )
    ctx.expect_gap(
        platform,
        base,
        axis="z",
        positive_elem="top_plate",
        negative_elem="base_disc",
        min_gap=0.030,
        max_gap=0.090,
        name="boxy top platform stands over the low base disc",
    )
    ctx.expect_contact(
        platform,
        base,
        elem_a="upper_bearing_race",
        elem_b="lower_bearing_race",
        contact_tol=0.002,
        name="opposed bearing races meet at the center turntable",
    )

    rest_aabb = ctx.part_element_world_aabb(platform, elem="top_plate")
    with ctx.pose({bearing: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(platform, elem="top_plate")

    def _extent(aabb, axis_index):
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.check(
        "rectangular platform rotates continuously about the vertical bearing",
        rest_aabb is not None
        and turned_aabb is not None
        and abs(_extent(rest_aabb, 0) - _extent(turned_aabb, 1)) < 0.020
        and abs(_extent(rest_aabb, 1) - _extent(turned_aabb, 0)) < 0.020,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
