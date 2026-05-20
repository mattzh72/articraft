from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = Material("weathered_concrete", color=(0.50, 0.49, 0.45, 1.0))
    dark_steel = Material("dark_blued_steel", color=(0.08, 0.11, 0.14, 1.0))
    deck_steel = Material("painted_steel_deck", color=(0.20, 0.29, 0.34, 1.0))
    asphalt = Material("dark_asphalt", color=(0.025, 0.025, 0.022, 1.0))
    safety_yellow = Material("safety_yellow", color=(0.95, 0.68, 0.06, 1.0))

    hinge_z = 1.20
    bridge_width = 4.60
    leaf_length = 12.0

    shore = model.part("shore_frame")
    shore.visual(
        Box((4.10, 6.40, 0.45)),
        origin=Origin(xyz=(-1.25, 0.0, 0.225)),
        material=concrete,
        name="foundation_slab",
    )
    shore.visual(
        Box((2.40, 5.25, 0.55)),
        origin=Origin(xyz=(-1.40, 0.0, 0.925)),
        material=concrete,
        name="approach_slab",
    )
    shore.visual(
        Box((0.55, 5.75, 1.10)),
        origin=Origin(xyz=(-0.46, 0.0, 0.72)),
        material=concrete,
        name="abutment_wall",
    )
    shore.visual(
        Box((1.20, 0.62, 0.34)),
        origin=Origin(xyz=(0.0, 2.80, 0.60)),
        material=concrete,
        name="bearing_plinth_0",
    )
    shore.visual(
        Box((1.20, 0.62, 0.34)),
        origin=Origin(xyz=(0.0, -2.80, 0.60)),
        material=concrete,
        name="bearing_plinth_1",
    )
    shore.visual(
        Box((0.42, 6.00, 0.18)),
        origin=Origin(xyz=(-0.08, 0.0, 0.62)),
        material=dark_steel,
        name="tie_crossbeam",
    )

    for idx, y in enumerate((2.80, -2.80)):
        cap_y = y + (0.205 if y > 0 else -0.205)
        shore.visual(
            Box((0.82, 0.34, 0.23)),
            origin=Origin(xyz=(0.0, y, hinge_z + 0.345)),
            material=dark_steel,
            name=f"bearing_top_{idx}",
        )
        shore.visual(
            Box((0.82, 0.34, 0.23)),
            origin=Origin(xyz=(0.0, y, hinge_z - 0.345)),
            material=dark_steel,
            name=f"bearing_bottom_{idx}",
        )
        shore.visual(
            Box((0.18, 0.34, 0.48)),
            origin=Origin(xyz=(0.32, y, hinge_z)),
            material=dark_steel,
            name=f"bearing_cheek_{idx}_0",
        )
        shore.visual(
            Box((0.18, 0.34, 0.48)),
            origin=Origin(xyz=(-0.32, y, hinge_z)),
            material=dark_steel,
            name=f"bearing_cheek_{idx}_1",
        )
        shore.visual(
            Box((0.64, 0.08, 0.08)),
            origin=Origin(xyz=(0.0, cap_y, hinge_z + 0.255)),
            material=dark_steel,
            name=f"bearing_cap_top_{idx}",
        )
        shore.visual(
            Box((0.64, 0.08, 0.08)),
            origin=Origin(xyz=(0.0, cap_y, hinge_z - 0.255)),
            material=dark_steel,
            name=f"bearing_cap_bottom_{idx}",
        )
        shore.visual(
            Box((0.08, 0.08, 0.44)),
            origin=Origin(xyz=(0.255, cap_y, hinge_z)),
            material=dark_steel,
            name=f"bearing_cap_side_{idx}_0",
        )
        shore.visual(
            Box((0.08, 0.08, 0.44)),
            origin=Origin(xyz=(-0.255, cap_y, hinge_z)),
            material=dark_steel,
            name=f"bearing_cap_side_{idx}_1",
        )

    leaf = model.part("bridge_leaf")
    leaf.visual(
        Box((leaf_length - 0.10, bridge_width, 0.36)),
        origin=Origin(xyz=(leaf_length * 0.5 + 0.05, 0.0, -0.175)),
        material=deck_steel,
        name="leaf_panel",
    )
    leaf.visual(
        Box((leaf_length - 0.35, bridge_width - 0.45, 0.040)),
        origin=Origin(xyz=(leaf_length * 0.5 + 0.10, 0.0, 0.027)),
        material=asphalt,
        name="road_surface",
    )
    leaf.visual(
        Box((leaf_length - 0.55, 0.20, 0.22)),
        origin=Origin(xyz=(leaf_length * 0.5 + 0.15, bridge_width * 0.5 - 0.16, 0.105)),
        material=deck_steel,
        name="edge_curb_0",
    )
    leaf.visual(
        Box((leaf_length - 0.55, 0.20, 0.22)),
        origin=Origin(xyz=(leaf_length * 0.5 + 0.15, -bridge_width * 0.5 + 0.16, 0.105)),
        material=deck_steel,
        name="edge_curb_1",
    )
    leaf.visual(
        Box((leaf_length - 0.55, 0.055, 0.020)),
        origin=Origin(xyz=(leaf_length * 0.5 + 0.15, bridge_width * 0.5 - 0.42, 0.052)),
        material=safety_yellow,
        name="edge_stripe_0",
    )
    leaf.visual(
        Box((leaf_length - 0.55, 0.055, 0.020)),
        origin=Origin(xyz=(leaf_length * 0.5 + 0.15, -bridge_width * 0.5 + 0.42, 0.052)),
        material=safety_yellow,
        name="edge_stripe_1",
    )
    leaf.visual(
        Box((0.56, bridge_width, 0.42)),
        origin=Origin(xyz=(0.22, 0.0, -0.13)),
        material=deck_steel,
        name="hinge_edge_box",
    )
    leaf.visual(
        Cylinder(radius=0.125, length=5.65),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_axle",
    )
    leaf.visual(
        Cylinder(radius=0.185, length=0.12),
        origin=Origin(xyz=(0.0, 2.43, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_collar_0",
    )
    leaf.visual(
        Cylinder(radius=0.185, length=0.12),
        origin=Origin(xyz=(0.0, -2.43, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_collar_1",
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.25, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    shore = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("shore_to_leaf")

    ctx.check(
        "single revolute bridge leaf",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            leaf,
            shore,
            axis="x",
            positive_elem="leaf_panel",
            negative_elem="approach_slab",
            min_gap=0.08,
            max_gap=0.35,
            name="closed leaf sits just beyond the shore approach",
        )
        ctx.expect_overlap(
            leaf,
            shore,
            axes="y",
            elem_a="leaf_panel",
            elem_b="approach_slab",
            min_overlap=4.0,
            name="leaf width aligns with approach roadway",
        )
        closed_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_panel")

    with ctx.pose({hinge: 1.25}):
        open_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_panel")

    ctx.check(
        "leaf raises at upper limit",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 5.0,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
