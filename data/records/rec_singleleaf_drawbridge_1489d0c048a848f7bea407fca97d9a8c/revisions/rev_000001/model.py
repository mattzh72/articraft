from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="singleleaf_drawbridge")

    galvanized = model.material("galvanized_steel", rgba=(0.56, 0.60, 0.62, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    concrete = model.material("precast_concrete", rgba=(0.50, 0.49, 0.45, 1.0))
    asphalt = model.material("asphalt_wearing_surface", rgba=(0.035, 0.037, 0.035, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.72, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    # One reusable annular extrusion for the two pillow-block side bearings.
    # It is intentionally hollow: the bridge leaf's trunnion shaft passes
    # through the bore instead of intersecting a solid proxy.
    bearing_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.250),
            [_circle_profile(0.135)],
            0.220,
            center=True,
        ),
        "hollow_side_bearing_ring",
    )

    support = model.part("support_frame")
    support.visual(
        Box((5.80, 2.45, 0.18)),
        origin=Origin(xyz=(1.70, 0.0, 0.09)),
        material=concrete,
        name="base_slab",
    )
    support.visual(
        Box((1.20, 1.55, 0.67)),
        origin=Origin(xyz=(-0.70, 0.0, 0.515)),
        material=concrete,
        name="shore_abutment",
    )
    support.visual(
        Box((1.00, 1.34, 0.16)),
        origin=Origin(xyz=(-0.72, 0.0, 0.920)),
        material=asphalt,
        name="fixed_approach",
    )
    support.visual(
        Box((0.48, 1.18, 0.63)),
        origin=Origin(xyz=(4.08, 0.0, 0.495)),
        material=concrete,
        name="receiving_abutment",
    )
    support.visual(
        Box((0.34, 1.22, 0.030)),
        origin=Origin(xyz=(4.03, 0.0, 0.825)),
        material=rubber,
        name="rest_pad",
    )

    # Two broad pedestals carry the prefabricated split bearing rings.  A low
    # cross tie keeps the frame visually and physically one bolted assembly.
    support.visual(
        Box((0.60, 2.02, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=galvanized,
        name="bearing_cross_tie",
    )
    for suffix, y in (("0", 1.05), ("1", -1.05)):
        support.visual(
            Box((0.56, 0.28, 0.64)),
            origin=Origin(xyz=(0.0, y, 0.500)),
            material=galvanized,
            name=f"bearing_pedestal_{suffix}",
        )
        support.visual(
            bearing_ring_mesh,
            origin=Origin(xyz=(0.0, y, 1.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=("bearing_ring_0" if suffix == "0" else "bearing_ring_1"),
        )

        face_y = y + (0.120 if y > 0.0 else -0.120)
        for bx in (-0.120, 0.120):
            for bz in (0.820, 1.180):
                support.visual(
                    Cylinder(radius=0.032, length=0.020),
                    origin=Origin(xyz=(bx, face_y, bz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                    material=galvanized,
                    name=f"cap_bolt_{suffix}_{'p' if bx > 0.0 else 'n'}_{'t' if bz > 1.0 else 'b'}",
                )

    leaf = model.part("bridge_leaf")
    leaf.visual(
        Cylinder(radius=0.105, length=2.36),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    for suffix, y in (("0", 0.88), ("1", -0.88)):
        leaf.visual(
            Cylinder(radius=0.160, length=0.050),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"trunnion_collar_{suffix}",
        )

    leaf.visual(
        Box((4.20, 1.40, 0.14)),
        origin=Origin(xyz=(2.10, 0.0, -0.090)),
        material=galvanized,
        name="deck_pan",
    )
    leaf.visual(
        Box((4.02, 1.02, 0.018)),
        origin=Origin(xyz=(2.20, 0.0, -0.011)),
        material=asphalt,
        name="road_surface",
    )
    for suffix, y in (("0", 0.76), ("1", -0.76)):
        leaf.visual(
            Box((4.20, 0.18, 0.34)),
            origin=Origin(xyz=(2.10, y, -0.060)),
            material=galvanized,
            name=f"side_girder_{suffix}",
        )
        leaf.visual(
            Box((4.00, 0.050, 0.035)),
            origin=Origin(xyz=(2.16, y, 0.123)),
            material=safety_yellow,
            name=f"curb_stripe_{suffix}",
        )

    for idx, x in enumerate((0.62, 1.80, 2.98, 3.78)):
        leaf.visual(
            Box((0.11, 1.42, 0.10)),
            origin=Origin(xyz=(x, 0.0, -0.200)),
            material=galvanized,
            name=f"stamped_rib_{idx}",
        )

    leaf.visual(
        Box((0.055, 1.34, 0.17)),
        origin=Origin(xyz=(4.22, 0.0, -0.050)),
        material=safety_yellow,
        name="nose_marker",
    )

    model.articulation(
        "leaf_pivot",
        ArticulationType.REVOLUTE,
        parent=support,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=85000.0, velocity=0.35, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    leaf = object_model.get_part("bridge_leaf")
    pivot = object_model.get_articulation("leaf_pivot")

    ctx.allow_overlap(
        leaf,
        support,
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_0",
        reason=(
            "The heavy trunnion is intentionally captured through the side bearing bore; "
            "the mesh ring is used as a simplified bearing sleeve for exact QC."
        ),
    )
    ctx.allow_overlap(
        leaf,
        support,
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_1",
        reason=(
            "The heavy trunnion is intentionally captured through the opposite side bearing bore; "
            "the mesh ring is used as a simplified bearing sleeve for exact QC."
        ),
    )

    ctx.expect_gap(
        leaf,
        support,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="deck_pan",
        negative_elem="rest_pad",
        name="closed leaf sits on the receiving pad",
    )
    ctx.expect_overlap(
        leaf,
        support,
        axes="xy",
        min_overlap=0.18,
        elem_a="deck_pan",
        elem_b="rest_pad",
        name="receiving pad is under the free end",
    )
    ctx.expect_overlap(
        leaf,
        support,
        axes="y",
        min_overlap=0.18,
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_0",
        name="trunnion passes through first side bearing",
    )
    ctx.expect_overlap(
        leaf,
        support,
        axes="y",
        min_overlap=0.18,
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_1",
        name="trunnion passes through second side bearing",
    )

    closed_aabb = ctx.part_element_world_aabb(leaf, elem="nose_marker")
    with ctx.pose({pivot: 1.25}):
        open_aabb = ctx.part_element_world_aabb(leaf, elem="nose_marker")
        ctx.expect_gap(
            leaf,
            support,
            axis="z",
            min_gap=0.70,
            positive_elem="nose_marker",
            negative_elem="rest_pad",
            name="raised leaf clears the receiving pad",
        )

    ctx.check(
        "positive pivot raises the bridge leaf",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] > closed_aabb[0][2] + 2.0,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
