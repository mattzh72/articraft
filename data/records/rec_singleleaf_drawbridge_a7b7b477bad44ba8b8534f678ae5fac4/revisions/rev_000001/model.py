from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("weathered_concrete", rgba=(0.50, 0.49, 0.45, 1.0))
    road = model.material("dark_asphalt", rgba=(0.06, 0.065, 0.07, 1.0))
    steel = model.material("painted_steel", rgba=(0.13, 0.16, 0.18, 1.0))
    worn_steel = model.material("worn_bearing_steel", rgba=(0.34, 0.35, 0.34, 1.0))
    safety = model.material("safety_yellow", rgba=(0.95, 0.72, 0.08, 1.0))

    hinge_z = 0.82

    bearing_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.165, segments=64),
            [_circle_profile(0.082, segments=48)],
            height=0.17,
            center=True,
        ),
        "bearing_ring",
    )

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((1.30, 2.90, 0.10)),
        origin=Origin(xyz=(-0.45, 0.0, 0.05)),
        material=concrete,
        name="base_slab",
    )
    shore_frame.visual(
        Box((0.24, 2.54, 0.70)),
        origin=Origin(xyz=(-0.19, 0.0, 0.40)),
        material=concrete,
        name="abutment_wall",
    )
    shore_frame.visual(
        Box((0.76, 2.02, 0.16)),
        origin=Origin(xyz=(-0.54, 0.0, 0.63)),
        material=road,
        name="approach_deck",
    )
    shore_frame.visual(
        Box((0.18, 2.18, 0.07)),
        origin=Origin(xyz=(-0.04, 0.0, 0.615)),
        material=worn_steel,
        name="hinge_sill",
    )

    for side_index, y in enumerate((-1.22, 1.22)):
        shore_frame.visual(
            Box((0.46, 0.20, 0.90)),
            origin=Origin(xyz=(-0.02, y, 0.50)),
            material=concrete,
            name=f"bearing_tower_{side_index}",
        )
        shore_frame.visual(
            Box((0.38, 0.18, 0.16)),
            origin=Origin(xyz=(-0.02, y, hinge_z)),
            material=steel,
            name=f"bearing_plinth_{side_index}",
        )
        shore_frame.visual(
            bearing_mesh,
            origin=Origin(xyz=(0.0, -1.045 if y < 0 else 1.045, hinge_z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"bearing_ring_{side_index}",
        )

    shore_frame.visual(
        Box((0.20, 2.46, 0.18)),
        origin=Origin(xyz=(-0.05, 0.0, 1.04)),
        material=concrete,
        name="overhead_tie",
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Cylinder(radius=0.064, length=2.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="hinge_pin",
    )
    bridge_leaf.visual(
        Box((0.86, 1.80, 0.12)),
        origin=Origin(xyz=(0.60, 0.0, -0.06)),
        material=steel,
        name="deck_panel",
    )
    bridge_leaf.visual(
        Box((0.80, 1.56, 0.014)),
        origin=Origin(xyz=(0.62, 0.0, 0.007)),
        material=road,
        name="road_surface",
    )
    bridge_leaf.visual(
        Box((0.065, 1.80, 0.18)),
        origin=Origin(xyz=(1.00, 0.0, -0.09)),
        material=safety,
        name="front_lip",
    )
    bridge_leaf.visual(
        Box((0.96, 0.07, 0.22)),
        origin=Origin(xyz=(0.59, -0.88, -0.10)),
        material=steel,
        name="side_girder_0",
    )
    bridge_leaf.visual(
        Box((0.96, 0.07, 0.22)),
        origin=Origin(xyz=(0.59, 0.88, -0.10)),
        material=steel,
        name="side_girder_1",
    )
    for rib_index, x in enumerate((0.30, 0.56, 0.82)):
        bridge_leaf.visual(
            Box((0.055, 1.62, 0.10)),
            origin=Origin(xyz=(x, 0.0, -0.165)),
            material=steel,
            name=f"cross_rib_{rib_index}",
        )
    for side_index, y in enumerate((-0.74, 0.74)):
        bridge_leaf.visual(
            Box((0.20, 0.09, 0.22)),
            origin=Origin(xyz=(0.085, y, -0.055)),
            material=steel,
            name=f"hinge_lug_{side_index}",
        )
        bridge_leaf.visual(
            Box((0.78, 0.035, 0.04)),
            origin=Origin(xyz=(0.61, y, 0.34)),
            material=steel,
            name=f"top_rail_{side_index}",
        )
        for post_index, x in enumerate((0.28, 0.56, 0.86)):
            bridge_leaf.visual(
                Box((0.035, 0.035, 0.35)),
                origin=Origin(xyz=(x, y, 0.165)),
                material=steel,
                name=f"rail_post_{side_index}_{post_index}",
            )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20000.0, velocity=0.45, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("shore_to_leaf")

    ctx.check(
        "single revolute bridge leaf",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.parent == "shore_frame"
        and hinge.child == "bridge_leaf",
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "horizontal shore-side hinge axis",
        tuple(round(v, 6) for v in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    ctx.allow_overlap(
        bridge_leaf,
        shore_frame,
        elem_a="hinge_pin",
        elem_b="bearing_ring_0",
        reason="The leaf trunnion is intentionally captured inside the shore-side bearing sleeve.",
    )
    ctx.allow_overlap(
        bridge_leaf,
        shore_frame,
        elem_a="hinge_pin",
        elem_b="bearing_ring_1",
        reason="The leaf trunnion is intentionally captured inside the shore-side bearing sleeve.",
    )
    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="y",
        elem_a="hinge_pin",
        elem_b="bearing_ring_0",
        min_overlap=0.045,
        name="hinge pin enters one side bearing",
    )
    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="y",
        elem_a="hinge_pin",
        elem_b="bearing_ring_1",
        min_overlap=0.045,
        name="hinge pin enters other side bearing",
    )
    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="z",
        positive_elem="deck_panel",
        negative_elem="approach_deck",
        min_gap=-0.012,
        max_gap=0.13,
        name="closed leaf deck sits at approach height",
    )

    closed_aabb = ctx.part_world_aabb(bridge_leaf)
    with ctx.pose({hinge: 1.15}):
        raised_aabb = ctx.part_world_aabb(bridge_leaf)
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="z",
            positive_elem="front_lip",
            negative_elem="approach_deck",
            min_gap=0.45,
            name="raised leaf clears upward",
        )
    ctx.check(
        "positive hinge angle lifts free end",
        closed_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > closed_aabb[1][2] + 0.45,
        details=f"closed={closed_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
