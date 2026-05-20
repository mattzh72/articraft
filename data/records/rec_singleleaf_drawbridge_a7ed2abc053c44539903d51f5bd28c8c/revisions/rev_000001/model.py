from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_singleleaf_drawbridge")

    steel = model.material("painted_dark_steel", rgba=(0.08, 0.09, 0.095, 1.0))
    plate = model.material("weathered_bridge_plate", rgba=(0.22, 0.24, 0.23, 1.0))
    concrete = model.material("sealed_concrete", rgba=(0.48, 0.48, 0.44, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.78, 0.05, 1.0))
    red = model.material("lockout_red", rgba=(0.85, 0.05, 0.03, 1.0))
    bolt = model.material("galvanized_fasteners", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = model.material("black_rubber_buffer", rgba=(0.015, 0.014, 0.012, 1.0))

    support = model.part("support_frame")

    # Fixed concrete sill and welded steel bearing frame.  All separate-looking
    # guards and brackets are seated into, or welded onto, this support part.
    support.visual(
        Box((1.90, 4.70, 0.30)),
        origin=Origin(xyz=(0.05, 0.0, 0.15)),
        material=concrete,
        name="foundation_slab",
    )
    support.visual(
        Box((0.34, 4.34, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=steel,
        name="base_tie_plate",
    )
    support.visual(
        Box((0.30, 4.05, 0.18)),
        origin=Origin(xyz=(-0.55, 0.0, 0.46)),
        material=steel,
        name="rear_tie_beam",
    )

    for side, y, bearing_housing_name in (
        ("guard_0", -1.86, "guard_0_bearing_housing"),
        ("guard_1", 1.86, "guard_1_bearing_housing"),
    ):
        sign = -1.0 if y < 0.0 else 1.0
        support.visual(
            Box((0.44, 0.42, 1.32)),
            origin=Origin(xyz=(0.0, y, 0.96)),
            material=steel,
            name=f"{side}_bearing_tower",
        )
        support.visual(
            Box((0.78, 0.66, 0.08)),
            origin=Origin(xyz=(0.0, y, 0.355)),
            material=steel,
            name=f"{side}_anchor_plate",
        )
        support.visual(
            Cylinder(radius=0.38, length=0.26),
            origin=Origin(xyz=(0.0, sign * 1.67, 1.00), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=bearing_housing_name,
        )
        support.visual(
            Cylinder(radius=0.23, length=0.26),
            origin=Origin(xyz=(0.0, sign * 1.67, 1.00), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bolt,
            name=f"{side}_bearing_bushing",
        )
        support.visual(
            Box((0.28, 0.16, 0.78)),
            origin=Origin(xyz=(-0.30, y, 1.08), rpy=(0.0, math.radians(-20.0), 0.0)),
            material=steel,
            name=f"{side}_rear_gusset",
        )
        support.visual(
            Box((0.18, 0.12, 0.40)),
            origin=Origin(xyz=(-0.30, sign * 1.34, 1.35), rpy=(0.0, math.radians(-30.0), 0.0)),
            material=yellow,
            name=f"{side}_overtravel_stop",
        )
        support.visual(
            Box((0.22, 0.14, 0.10)),
            origin=Origin(xyz=(-0.16, sign * 1.34, 1.45), rpy=(0.0, math.radians(-30.0), 0.0)),
            material=rubber,
            name=f"{side}_stop_buffer",
        )
        support.visual(
            Box((0.18, 0.08, 0.52)),
            origin=Origin(xyz=(-0.63, sign * 1.41, 0.74)),
            material=red,
            name=f"{side}_lockout_pin",
        )
        support.visual(
            Box((0.45, 0.34, 0.12)),
            origin=Origin(xyz=(-0.42, sign * 1.54, 0.72)),
            material=steel,
            name=f"{side}_lockout_mount",
        )
        support.visual(
            Box((0.28, 0.10, 0.18)),
            origin=Origin(xyz=(-0.63, sign * 1.41, 0.49)),
            material=steel,
            name=f"{side}_lockout_clip",
        )
        support.visual(
            Box((0.05, 0.05, 0.95)),
            origin=Origin(xyz=(0.43, sign * 2.10, 0.775)),
            material=yellow,
            name=f"{side}_guard_post_0",
        )
        support.visual(
            Box((0.05, 0.05, 0.95)),
            origin=Origin(xyz=(-0.70, sign * 2.10, 0.775)),
            material=yellow,
            name=f"{side}_guard_post_1",
        )
        support.visual(
            Box((1.18, 0.05, 0.07)),
            origin=Origin(xyz=(-0.135, sign * 2.10, 1.245)),
            material=yellow,
            name=f"{side}_guard_top_rail",
        )
        support.visual(
            Box((1.18, 0.045, 0.055)),
            origin=Origin(xyz=(-0.135, sign * 2.10, 0.91)),
            material=yellow,
            name=f"{side}_guard_mid_rail",
        )

        for i, (dx, dz) in enumerate(
            (
                (-0.31, -0.15),
                (0.0, -0.31),
                (0.31, -0.15),
                (0.31, 0.15),
                (0.0, 0.31),
                (-0.31, 0.15),
            )
        ):
            support.visual(
                Cylinder(radius=0.032, length=0.050),
                origin=Origin(
                    xyz=(dx, sign * 1.525, 1.00 + dz),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=bolt,
                name=f"{side}_bearing_bolt_{i}",
            )

        for i, x in enumerate((-0.30, 0.30)):
            for j, yy in enumerate((y - sign * 0.20, y + sign * 0.20)):
                support.visual(
                    Cylinder(radius=0.030, length=0.028),
                    origin=Origin(xyz=(x, yy, 0.385)),
                    material=bolt,
                    name=f"{side}_anchor_bolt_{i}_{j}",
                )

    support.visual(
        Box((0.24, 4.04, 0.18)),
        origin=Origin(xyz=(-0.02, 0.0, 1.62)),
        material=steel,
        name="upper_tie_beam",
    )
    support.visual(
        Box((0.52, 4.05, 0.16)),
        origin=Origin(xyz=(-0.35, 0.0, 0.42)),
        material=steel,
        name="rear_beam_saddle",
    )

    leaf = model.part("bridge_leaf")

    # The leaf frame is at the trunnion center line.  In the closed pose the
    # roadway extends in +X; positive joint motion raises the free end.
    leaf.visual(
        Box((6.10, 2.70, 0.18)),
        origin=Origin(xyz=(3.05, 0.0, 0.0)),
        material=plate,
        name="deck_plate",
    )
    leaf.visual(
        Box((6.05, 0.22, 0.42)),
        origin=Origin(xyz=(3.05, -1.34, -0.17)),
        material=steel,
        name="edge_girder_0",
    )
    leaf.visual(
        Box((6.05, 0.22, 0.42)),
        origin=Origin(xyz=(3.05, 1.34, -0.17)),
        material=steel,
        name="edge_girder_1",
    )
    leaf.visual(
        Box((0.30, 2.88, 0.38)),
        origin=Origin(xyz=(0.18, 0.0, -0.12)),
        material=steel,
        name="hinge_box_beam",
    )
    leaf.visual(
        Cylinder(radius=0.155, length=2.80),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion_shaft",
    )

    for side, y, trunnion_collar_name in (
        ("side_0", -1.47, "side_0_trunnion_collar"),
        ("side_1", 1.47, "side_1_trunnion_collar"),
    ):
        sign = -1.0 if y < 0.0 else 1.0
        leaf.visual(
            Cylinder(radius=0.245, length=0.14),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bolt,
            name=trunnion_collar_name,
        )
        leaf.visual(
            Box((0.86, 0.20, 0.40)),
            origin=Origin(xyz=(0.35, sign * 1.25, -0.12), rpy=(0.0, math.radians(23.0), 0.0)),
            material=steel,
            name=f"{side}_hinge_knee_brace",
        )
        leaf.visual(
            Box((0.36, 0.15, 0.46)),
            origin=Origin(xyz=(0.22, sign * 1.18, 0.19), rpy=(0.0, math.radians(-18.0), 0.0)),
            material=steel,
            name=f"{side}_stop_striker",
        )
        leaf.visual(
            Box((0.30, 0.13, 0.32)),
            origin=Origin(xyz=(0.75, sign * 1.16, 0.17)),
            material=red,
            name=f"{side}_lockout_receiver",
        )
        leaf.visual(
            Box((5.45, 0.09, 0.20)),
            origin=Origin(xyz=(3.28, sign * 1.43, 0.13)),
            material=yellow,
            name=f"{side}_toe_board",
        )
        leaf.visual(
            Box((0.055, 0.055, 0.70)),
            origin=Origin(xyz=(0.90, sign * 1.43, 0.42)),
            material=yellow,
            name=f"{side}_rail_post_0",
        )
        leaf.visual(
            Box((0.055, 0.055, 0.70)),
            origin=Origin(xyz=(2.60, sign * 1.43, 0.42)),
            material=yellow,
            name=f"{side}_rail_post_1",
        )
        leaf.visual(
            Box((0.055, 0.055, 0.70)),
            origin=Origin(xyz=(4.30, sign * 1.43, 0.42)),
            material=yellow,
            name=f"{side}_rail_post_2",
        )
        leaf.visual(
            Box((0.055, 0.055, 0.70)),
            origin=Origin(xyz=(5.80, sign * 1.43, 0.42)),
            material=yellow,
            name=f"{side}_rail_post_3",
        )
        leaf.visual(
            Box((5.00, 0.065, 0.075)),
            origin=Origin(xyz=(3.35, sign * 1.43, 0.76)),
            material=yellow,
            name=f"{side}_top_rail",
        )
        leaf.visual(
            Box((5.00, 0.055, 0.055)),
            origin=Origin(xyz=(3.35, sign * 1.43, 0.49)),
            material=yellow,
            name=f"{side}_mid_rail",
        )
        for i, x in enumerate((0.45, 0.72, 1.10)):
            leaf.visual(
                Cylinder(radius=0.030, length=0.050),
                origin=Origin(xyz=(x, sign * 1.235, 0.105)),
                material=bolt,
                name=f"{side}_hinge_plate_bolt_{i}",
            )

    for i, x in enumerate((1.20, 2.35, 3.50, 4.65, 5.75)):
        leaf.visual(
            Box((0.18, 2.80, 0.22)),
            origin=Origin(xyz=(x, 0.0, -0.19)),
            material=steel,
            name=f"crossbeam_{i}",
        )
        leaf.visual(
            Box((1.00, 0.14, 0.16)),
            origin=Origin(xyz=(x - 0.08, -0.68, -0.23), rpy=(0.0, 0.0, math.radians(23.0))),
            material=steel,
            name=f"diagonal_brace_{i}_0",
        )
        leaf.visual(
            Box((1.00, 0.14, 0.16)),
            origin=Origin(xyz=(x - 0.08, 0.68, -0.23), rpy=(0.0, 0.0, math.radians(-23.0))),
            material=steel,
            name=f"diagonal_brace_{i}_1",
        )

    pivot = model.articulation(
        "support_to_leaf",
        ArticulationType.REVOLUTE,
        parent=support,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90000.0, velocity=0.28, lower=0.0, upper=1.22),
        motion_properties=MotionProperties(damping=1800.0, friction=420.0),
    )
    pivot.meta["description"] = "Heavy side trunnion pivot; positive rotation raises the bridge leaf."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    leaf = object_model.get_part("bridge_leaf")
    pivot = object_model.get_articulation("support_to_leaf")

    ctx.expect_contact(
        leaf,
        support,
        elem_a="side_1_trunnion_collar",
        elem_b="guard_1_bearing_housing",
        contact_tol=0.002,
        name="positive side trunnion is seated in bearing",
    )
    ctx.expect_contact(
        leaf,
        support,
        elem_a="side_0_trunnion_collar",
        elem_b="guard_0_bearing_housing",
        contact_tol=0.002,
        name="negative side trunnion is seated in bearing",
    )
    ctx.expect_overlap(
        leaf,
        support,
        axes="z",
        elem_a="side_1_trunnion_collar",
        elem_b="guard_1_bearing_housing",
        min_overlap=0.35,
        name="bearing and collar share pivot height",
    )

    closed_aabb = ctx.part_element_world_aabb(leaf, elem="deck_plate")
    closed_free_end_z = closed_aabb[1][2] if closed_aabb is not None else None
    with ctx.pose({pivot: 1.22}):
        raised_aabb = ctx.part_element_world_aabb(leaf, elem="deck_plate")
        raised_free_end_z = raised_aabb[1][2] if raised_aabb is not None else None
        ctx.expect_gap(
            leaf,
            support,
            axis="z",
            positive_elem="deck_plate",
            negative_elem="foundation_slab",
            min_gap=0.45,
            name="raised leaf clears fixed foundation",
        )

    ctx.check(
        "leaf rotates upward from side bearings",
        closed_free_end_z is not None
        and raised_free_end_z is not None
        and raised_free_end_z > closed_free_end_z + 3.2,
        details=f"closed_z={closed_free_end_z}, raised_z={raised_free_end_z}",
    )

    return ctx.report()


object_model = build_object_model()
