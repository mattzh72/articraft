from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("concrete", rgba=(0.62, 0.63, 0.64, 1.0))
    structural_steel = model.material("structural_steel", rgba=(0.31, 0.34, 0.37, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    deck_surface = model.material("deck_surface", rgba=(0.24, 0.24, 0.25, 1.0))
    warning_paint = model.material("warning_paint", rgba=(0.78, 0.66, 0.18, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.inertial = Inertial.from_geometry(
        Box((2.8, 6.8, 2.2)),
        mass=42000.0,
        origin=Origin(xyz=(-1.4, 0.0, 1.1)),
    )
    shore_frame.visual(
        Box((2.8, 5.4, 1.2)),
        origin=Origin(xyz=(-1.4, 0.0, 0.6)),
        material=concrete,
        name="abutment",
    )
    shore_frame.visual(
        Box((1.75, 5.0, 0.18)),
        origin=Origin(xyz=(-0.875, 0.0, 1.29)),
        material=deck_surface,
        name="shore_apron",
    )
    shore_frame.visual(
        Box((0.75, 0.64, 2.2)),
        origin=Origin(xyz=(-0.375, 3.63, 1.1)),
        material=structural_steel,
        name="left_support_frame",
    )
    shore_frame.visual(
        Box((0.75, 0.64, 2.2)),
        origin=Origin(xyz=(-0.375, -3.63, 1.1)),
        material=structural_steel,
        name="right_support_frame",
    )
    shore_frame.visual(
        Box((1.10, 0.90, 1.40)),
        origin=Origin(xyz=(-1.00, 3.15, 0.70)),
        material=concrete,
        name="left_side_pedestal",
    )
    shore_frame.visual(
        Box((1.10, 0.90, 1.40)),
        origin=Origin(xyz=(-1.00, -3.15, 0.70)),
        material=concrete,
        name="right_side_pedestal",
    )
    shore_frame.visual(
        Cylinder(radius=0.30, length=0.28),
        origin=Origin(xyz=(0.0, 3.40, 1.20), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_bearing",
    )
    shore_frame.visual(
        Cylinder(radius=0.30, length=0.28),
        origin=Origin(xyz=(0.0, -3.40, 1.20), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_bearing",
    )
    shore_frame.visual(
        Box((0.22, 5.6, 0.12)),
        origin=Origin(xyz=(-0.11, 0.0, 1.26)),
        material=warning_paint,
        name="hinge_sill",
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((8.0, 5.0, 0.8)),
        mass=18000.0,
        origin=Origin(xyz=(3.9, 0.0, -0.05)),
    )
    bridge_leaf.visual(
        Box((8.0, 5.0, 0.18)),
        origin=Origin(xyz=(4.0, 0.0, 0.09)),
        material=deck_surface,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((8.0, 0.18, 0.08)),
        origin=Origin(xyz=(4.0, 2.41, 0.22)),
        material=warning_paint,
        name="left_curb",
    )
    bridge_leaf.visual(
        Box((8.0, 0.18, 0.08)),
        origin=Origin(xyz=(4.0, -2.41, 0.22)),
        material=warning_paint,
        name="right_curb",
    )
    bridge_leaf.visual(
        Box((7.6, 0.26, 0.62)),
        origin=Origin(xyz=(3.8, 2.13, -0.13)),
        material=structural_steel,
        name="left_girder",
    )
    bridge_leaf.visual(
        Box((7.6, 0.26, 0.62)),
        origin=Origin(xyz=(3.8, -2.13, -0.13)),
        material=structural_steel,
        name="right_girder",
    )
    bridge_leaf.visual(
        Box((0.34, 5.0, 0.48)),
        origin=Origin(xyz=(0.17, 0.0, -0.06)),
        material=structural_steel,
        name="rear_cross_girder",
    )
    bridge_leaf.visual(
        Box((0.24, 5.0, 0.50)),
        origin=Origin(xyz=(7.88, 0.0, -0.07)),
        material=structural_steel,
        name="nose_beam",
    )
    bridge_leaf.visual(
        Box((0.42, 4.50, 0.22)),
        origin=Origin(xyz=(2.10, 0.0, -0.10)),
        material=structural_steel,
        name="mid_cross_girder_a",
    )
    bridge_leaf.visual(
        Box((0.42, 4.50, 0.22)),
        origin=Origin(xyz=(5.20, 0.0, -0.10)),
        material=structural_steel,
        name="mid_cross_girder_b",
    )
    bridge_leaf.visual(
        Box((0.46, 0.58, 0.38)),
        origin=Origin(xyz=(0.23, 2.54, -0.06)),
        material=dark_steel,
        name="left_trunnion_arm",
    )
    bridge_leaf.visual(
        Box((0.46, 0.58, 0.38)),
        origin=Origin(xyz=(0.23, -2.54, -0.06)),
        material=dark_steel,
        name="right_trunnion_arm",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.18, length=0.40),
        origin=Origin(xyz=(0.0, 3.02, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.18, length=0.40),
        origin=Origin(xyz=(0.0, -3.02, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400000.0,
            velocity=0.35,
            lower=0.0,
            upper=1.20,
        ),
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
    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    leaf_hinge = object_model.get_articulation("leaf_hinge")

    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="x",
        positive_elem="deck_plate",
        negative_elem="shore_apron",
        min_gap=0.0,
        max_gap=0.01,
        name="leaf seats directly off the shore apron",
    )
    ctx.expect_within(
        bridge_leaf,
        shore_frame,
        axes="xz",
        inner_elem="left_trunnion",
        outer_elem="left_bearing",
        margin=0.02,
        name="left trunnion aligns inside the left side bearing footprint",
    )
    ctx.expect_within(
        bridge_leaf,
        shore_frame,
        axes="xz",
        inner_elem="right_trunnion",
        outer_elem="right_bearing",
        margin=0.02,
        name="right trunnion aligns inside the right side bearing footprint",
    )

    closed_nose = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")
    with ctx.pose({leaf_hinge: 1.05}):
        opened_nose = ctx.part_element_world_aabb(bridge_leaf, elem="nose_beam")
    ctx.check(
        "leaf nose rises when the hinge opens",
        closed_nose is not None
        and opened_nose is not None
        and opened_nose[0][2] > closed_nose[1][2] + 4.0,
        details=f"closed_nose={closed_nose}, opened_nose={opened_nose}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
