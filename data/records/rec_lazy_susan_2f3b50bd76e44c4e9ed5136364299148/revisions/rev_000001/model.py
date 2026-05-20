from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _octagon_points(radius: float) -> list[tuple[float, float]]:
    # Rotate by 22.5 degrees so the +X side is a flat edge for the lock tab.
    return [
        (
            radius * math.cos(math.pi / 8.0 + i * math.pi / 4.0),
            radius * math.sin(math.pi / 8.0 + i * math.pi / 4.0),
        )
        for i in range(8)
    ]


def _octagonal_prism(radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .polyline(_octagon_points(radius))
        .close()
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _annular_prism(
    outer_radius: float, inner_radius: float, height: float, z0: float = 0.0
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _retainer_channel() -> cq.Workplane:
    """A shallow C-channel fixed under the rotating top and flanking the ring."""
    cap = _annular_prism(0.315, 0.205, 0.006, 0.0)
    outer_skirt = _annular_prism(0.315, 0.300, 0.030, -0.024)
    inner_skirt = _annular_prism(0.210, 0.195, 0.030, -0.024)
    return cap.union(outer_skirt).union(inner_skirt)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_octagonal_lazy_susan")

    oiled_wood = model.material("oiled_wood", rgba=(0.55, 0.32, 0.14, 1.0))
    dark_base = model.material("dark_powder_coated_base", rgba=(0.035, 0.033, 0.030, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.70, 0.66, 1.0))
    brushed_brass = model.material("brushed_brass", rgba=(0.86, 0.62, 0.25, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.010, 0.010, 0.009, 1.0))

    base = model.part("base_disc")
    base.visual(
        Cylinder(radius=0.460, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_base,
        name="lower_plate",
    )
    base.visual(
        mesh_from_cadquery(_annular_prism(0.285, 0.225, 0.006, 0.029), "ring_support"),
        material=dark_base,
        name="ring_support",
    )
    base.visual(
        mesh_from_cadquery(_annular_prism(0.285, 0.225, 0.026, 0.034), "bearing_ring"),
        material=satin_steel,
        name="bearing_ring",
    )
    base.visual(
        Box((0.050, 0.018, 0.056)),
        origin=Origin(xyz=(0.470, 0.031, 0.057)),
        material=dark_base,
        name="keeper_lug_0",
    )
    base.visual(
        Box((0.050, 0.018, 0.056)),
        origin=Origin(xyz=(0.470, -0.031, 0.057)),
        material=dark_base,
        name="keeper_lug_1",
    )
    base.visual(
        Box((0.105, 0.082, 0.006)),
        origin=Origin(xyz=(0.450, 0.0, 0.032)),
        material=black_rubber,
        name="lock_socket_pad",
    )

    top = model.part("top_plate")
    top.visual(
        mesh_from_cadquery(_octagonal_prism(0.420, 0.036, 0.005), "octagonal_top"),
        material=oiled_wood,
        name="octagonal_slab",
    )
    top.visual(
        mesh_from_cadquery(_annular_prism(0.315, 0.205, 0.006, 0.0), "upper_race"),
        material=satin_steel,
        name="upper_race",
    )
    top.visual(
        mesh_from_cadquery(_annular_prism(0.315, 0.300, 0.030, -0.024), "outer_clip_skirt"),
        material=satin_steel,
        name="outer_clip_skirt",
    )
    top.visual(
        mesh_from_cadquery(_annular_prism(0.210, 0.195, 0.030, -0.024), "inner_clip_skirt"),
        material=satin_steel,
        name="inner_clip_skirt",
    )
    top.visual(
        Cylinder(radius=0.055, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=brushed_brass,
        name="center_cap",
    )

    locking_tab = model.part("locking_tab")
    locking_tab.visual(
        Box((0.150, 0.045, 0.008)),
        origin=Origin(xyz=(0.075, 0.0, 0.006)),
        material=brushed_brass,
        name="tab_leaf",
    )
    locking_tab.visual(
        Cylinder(radius=0.019, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_steel,
        name="pivot_boss",
    )
    locking_tab.visual(
        Box((0.019, 0.016, 0.058)),
        origin=Origin(xyz=(0.135, 0.0, -0.026)),
        material=brushed_brass,
        name="lock_tooth",
    )
    locking_tab.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.132, 0.0, 0.016)),
        material=satin_steel,
        name="tooth_rivet",
    )

    model.articulation(
        "top_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2),
    )
    model.articulation(
        "tab_pivot",
        ArticulationType.REVOLUTE,
        parent=top,
        child=locking_tab,
        origin=Origin(xyz=(0.335, 0.0, 0.041)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_disc")
    top = object_model.get_part("top_plate")
    tab = object_model.get_part("locking_tab")
    top_spin = object_model.get_articulation("top_spin")
    tab_pivot = object_model.get_articulation("tab_pivot")

    ctx.check(
        "top is a continuous turntable joint",
        top_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"top_spin type={top_spin.articulation_type}",
    )
    ctx.check(
        "lock tab has limited pivot travel",
        tab_pivot.motion_limits is not None
        and tab_pivot.motion_limits.lower == 0.0
        and tab_pivot.motion_limits.upper is not None
        and 1.0 <= tab_pivot.motion_limits.upper <= 1.4,
        details=f"tab_pivot limits={tab_pivot.motion_limits}",
    )

    ctx.expect_gap(
        top,
        base,
        axis="z",
        positive_elem="upper_race",
        negative_elem="bearing_ring",
        min_gap=0.0,
        max_gap=0.0005,
        name="upper retainer is seated just above bearing ring",
    )
    ctx.expect_overlap(
        top,
        base,
        axes="xy",
        elem_a="upper_race",
        elem_b="bearing_ring",
        min_overlap=0.50,
        name="upper race covers bearing ring footprint",
    )
    ctx.expect_within(
        base,
        top,
        axes="xy",
        inner_elem="bearing_ring",
        outer_elem="outer_clip_skirt",
        margin=0.002,
        name="bearing ring stays inside outer clip skirt",
    )
    ctx.expect_within(
        top,
        base,
        axes="xy",
        inner_elem="inner_clip_skirt",
        outer_elem="bearing_ring",
        margin=0.002,
        name="inner clip skirt sits inside bearing ring bore",
    )

    ctx.expect_overlap(
        tab,
        base,
        axes="xz",
        elem_a="lock_tooth",
        elem_b="keeper_lug_0",
        min_overlap=0.015,
        name="lock tooth reaches the keeper slot height",
    )
    ctx.expect_gap(
        base,
        tab,
        axis="y",
        positive_elem="keeper_lug_0",
        negative_elem="lock_tooth",
        min_gap=0.006,
        max_gap=0.020,
        name="lock tooth clears the upper side of keeper slot",
    )
    ctx.expect_gap(
        tab,
        base,
        axis="y",
        positive_elem="lock_tooth",
        negative_elem="keeper_lug_1",
        min_gap=0.006,
        max_gap=0.020,
        name="lock tooth clears the lower side of keeper slot",
    )

    with ctx.pose({tab_pivot: 1.15}):
        tooth_aabb = ctx.part_element_world_aabb(tab, elem="lock_tooth")
        ctx.check(
            "lock tab swings clear of keeper",
            tooth_aabb is not None and tooth_aabb[0][1] > 0.10,
            details=f"lock_tooth_aabb={tooth_aabb}",
        )

    with ctx.pose({top_spin: math.pi / 2.0, tab_pivot: 1.15}):
        ctx.expect_gap(
            top,
            base,
            axis="z",
            positive_elem="upper_race",
            negative_elem="bearing_ring",
            min_gap=0.0,
            max_gap=0.0005,
            name="rotated top remains seated on bearing ring",
        )

    return ctx.report()


object_model = build_object_model()
