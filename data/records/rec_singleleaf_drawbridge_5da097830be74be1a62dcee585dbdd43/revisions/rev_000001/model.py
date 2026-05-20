from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="singleleaf_utility_drawbridge")

    concrete = model.material("aged_concrete", rgba=(0.46, 0.45, 0.40, 1.0))
    utility_green = model.material("painted_olive_steel", rgba=(0.18, 0.28, 0.18, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.035, 0.040, 0.038, 1.0))
    deck_black = model.material("gritty_black_deck", rgba=(0.020, 0.022, 0.020, 1.0))
    hazard_yellow = model.material("safety_yellow", rgba=(0.94, 0.68, 0.06, 1.0))
    worn_bolt = model.material("worn_bolt_heads", rgba=(0.30, 0.31, 0.29, 1.0))

    abutment = model.part("abutment")
    leaf = model.part("leaf")

    # Fixed rugged abutment: concrete mass, rear block, and tied side bearing
    # pedestals.  The single root part makes the support path visually obvious.
    abutment.visual(
        Box((1.25, 2.70, 0.28)),
        origin=Origin(xyz=(-0.18, 0.0, 0.14)),
        material=concrete,
        name="foundation",
    )
    abutment.visual(
        Box((0.48, 2.28, 0.44)),
        origin=Origin(xyz=(-0.48, 0.0, 0.50)),
        material=concrete,
        name="rear_abutment",
    )
    abutment.visual(
        Box((0.52, 1.92, 0.10)),
        origin=Origin(xyz=(-0.39, 0.0, 0.77)),
        material=dark_steel,
        name="abutment_cap",
    )
    abutment.visual(
        Box((0.18, 1.92, 0.18)),
        origin=Origin(xyz=(-0.22, 0.0, 0.65)),
        material=dark_steel,
        name="heel_stop",
    )

    bearing_ring_mesh = mesh_from_geometry(
        TorusGeometry(0.160, 0.034, radial_segments=36, tubular_segments=16),
        "bearing_ring",
    )
    for side, y in enumerate((-1.13, 1.13)):
        suffix = f"_{side}"
        abutment.visual(
            Box((0.36, 0.24, 0.52)),
            origin=Origin(xyz=(0.0, y, 0.54)),
            material=dark_steel,
            name=f"bearing_pedestal{suffix}",
        )
        abutment.visual(
            bearing_ring_mesh,
            origin=Origin(xyz=(0.0, y, 0.90), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"bearing_ring{suffix}",
        )
        abutment.visual(
            Box((0.11, 0.16, 0.06)),
            origin=Origin(xyz=(0.0, y, 0.800)),
            material=worn_bolt,
            name=f"bearing_saddle{suffix}",
        )
        abutment.visual(
            Box((0.58, 0.12, 0.10)),
            origin=Origin(xyz=(-0.39, y, 0.80)),
            material=dark_steel,
            name=f"bearing_tie{suffix}",
        )
        abutment.visual(
            Box((0.55, 0.08, 0.12)),
            origin=Origin(xyz=(-0.30, y, 0.58), rpy=(0.0, -0.58, 0.0)),
            material=utility_green,
            name=f"diagonal_gusset{suffix}",
        )
        for x in (-0.15, 0.15):
            for z in (0.68, 0.82):
                abutment.visual(
                    Cylinder(radius=0.026, length=0.050),
                    origin=Origin(
                        xyz=(x, y - math.copysign(0.120, y), z),
                        rpy=(-math.copysign(math.pi / 2.0, y), 0.0, 0.0),
                    ),
                    material=worn_bolt,
                    name=f"bearing_bolt{suffix}_{x:+.2f}_{z:.2f}",
                )

    for x in (-0.46, -0.22):
        for y in (-0.78, 0.78):
            abutment.visual(
                Cylinder(radius=0.034, length=0.030),
                origin=Origin(xyz=(x, y, 0.295)),
                material=worn_bolt,
                name=f"anchor_bolt_{x:+.2f}_{y:+.2f}",
            )

    # Moving bridge leaf: thick road plate, deep side girders, underside beams,
    # trunnion shaft and service rails.  The part frame is on the hinge axis.
    leaf.visual(
        Box((4.30, 1.55, 0.18)),
        origin=Origin(xyz=(2.18, 0.0, -0.18)),
        material=utility_green,
        name="deck_plate",
    )
    leaf.visual(
        Box((4.14, 1.39, 0.035)),
        origin=Origin(xyz=(2.26, 0.0, -0.072)),
        material=deck_black,
        name="wear_surface",
    )
    leaf.visual(
        Box((0.16, 1.74, 0.32)),
        origin=Origin(xyz=(4.38, 0.0, -0.20)),
        material=utility_green,
        name="front_lip",
    )

    for side, y in enumerate((-0.82, 0.82)):
        suffix = f"_{side}"
        leaf.visual(
            Box((4.46, 0.15, 0.46)),
            origin=Origin(xyz=(2.18, y, -0.20)),
            material=utility_green,
            name=f"side_girder{suffix}",
        )
        leaf.visual(
            Box((4.05, 0.06, 0.08)),
            origin=Origin(xyz=(2.35, y * 1.08, 0.42)),
            material=hazard_yellow,
            name=f"guard_rail{suffix}",
        )
        for x in (0.45, 1.35, 2.25, 3.15, 4.05):
            leaf.visual(
                Box((0.08, 0.08, 0.40)),
                origin=Origin(xyz=(x, y * 1.08, 0.22)),
                material=dark_steel,
                name=f"rail_post{suffix}_{x:.2f}",
            )
        for x in (0.36, 1.18, 2.00, 2.82, 3.64):
            leaf.visual(
                Cylinder(radius=0.026, length=0.020),
                origin=Origin(xyz=(x, y, 0.035)),
                material=worn_bolt,
                name=f"side_girder_bolt{suffix}_{x:.2f}",
            )

    for x in (0.55, 1.35, 2.15, 2.95, 3.75):
        leaf.visual(
            Box((0.13, 1.74, 0.16)),
            origin=Origin(xyz=(x, 0.0, -0.35)),
            material=dark_steel,
            name=f"crossbeam_{x:.2f}",
        )
    for y in (-0.38, 0.38):
        leaf.visual(
            Box((4.05, 0.12, 0.18)),
            origin=Origin(xyz=(2.22, y, -0.36)),
            material=dark_steel,
            name=f"longitudinal_beam_{y:+.2f}",
        )

    for x in (0.72, 1.22, 1.72, 2.22, 2.72, 3.22, 3.72):
        leaf.visual(
            Box((0.045, 1.30, 0.024)),
            origin=Origin(xyz=(x, 0.0, -0.048)),
            material=dark_steel,
            name=f"tread_bar_{x:.2f}",
        )

    leaf.visual(
        Cylinder(radius=0.070, length=2.44),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_bolt,
        name="hinge_shaft",
    )
    for side, y in enumerate((-0.86, 0.86)):
        suffix = f"_{side}"
        leaf.visual(
            Cylinder(radius=0.120, length=0.20),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"leaf_trunnion{suffix}",
        )
        leaf.visual(
            Box((0.22, 0.12, 0.26)),
            origin=Origin(xyz=(0.08, y, -0.14)),
            material=dark_steel,
            name=f"hinge_lug{suffix}",
        )
        leaf.visual(
            Box((0.36, 0.08, 0.10)),
            origin=Origin(xyz=(0.16, y, -0.30), rpy=(0.0, 0.45, 0.0)),
            material=utility_green,
            name=f"lug_web{suffix}",
        )

    for y in (-0.43, 0.43):
        for x in (4.36,):
            leaf.visual(
                Cylinder(radius=0.036, length=0.022),
                origin=Origin(xyz=(x, y, -0.035)),
                material=hazard_yellow,
                name=f"front_service_bolt_{y:+.2f}",
            )

    model.articulation(
        "abutment_to_leaf",
        ArticulationType.REVOLUTE,
        parent=abutment,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.35, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    abutment = object_model.get_part("abutment")
    leaf = object_model.get_part("leaf")
    hinge = object_model.get_articulation("abutment_to_leaf")

    ctx.expect_gap(
        leaf,
        abutment,
        axis="x",
        min_gap=0.08,
        positive_elem="deck_plate",
        negative_elem="abutment_cap",
        name="closed leaf clears the abutment cap",
    )
    for ring_name in ("bearing_ring_0", "bearing_ring_1"):
        ctx.expect_within(
            leaf,
            abutment,
            axes="xz",
            inner_elem="hinge_shaft",
            outer_elem=ring_name,
            name=f"hinge shaft is centered in {ring_name}",
        )
        ctx.expect_overlap(
            leaf,
            abutment,
            axes="y",
            elem_a="hinge_shaft",
            elem_b=ring_name,
            min_overlap=0.040,
            name=f"hinge shaft passes through {ring_name}",
        )

    rest_front = ctx.part_element_world_aabb(leaf, elem="front_lip")
    with ctx.pose({hinge: 1.10}):
        raised_front = ctx.part_element_world_aabb(leaf, elem="front_lip")
    ctx.check(
        "bridge leaf raises upward",
        rest_front is not None
        and raised_front is not None
        and raised_front[1][2] > rest_front[1][2] + 2.0,
        details=f"rest_front={rest_front}, raised_front={raised_front}",
    )

    return ctx.report()


object_model = build_object_model()
