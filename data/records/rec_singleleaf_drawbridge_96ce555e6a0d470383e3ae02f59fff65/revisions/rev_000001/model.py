from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

    concrete = model.material("concrete", rgba=(0.67, 0.68, 0.70, 1.0))
    asphalt = model.material("asphalt", rgba=(0.12, 0.12, 0.13, 1.0))
    bridge_steel = model.material("bridge_steel", rgba=(0.31, 0.37, 0.42, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.21, 0.23, 0.26, 1.0))
    stripe = model.material("stripe", rgba=(0.87, 0.83, 0.24, 1.0))
    curb = model.material("curb", rgba=(0.76, 0.78, 0.80, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((3.7, 11.6, 2.2)),
        origin=Origin(xyz=(-2.55, 0.0, 1.10)),
        material=concrete,
        name="abutment_block",
    )
    shore_frame.visual(
        Box((3.5, 8.6, 0.32)),
        origin=Origin(xyz=(-2.45, 0.0, 2.36)),
        material=asphalt,
        name="shore_roadway",
    )
    shore_frame.visual(
        Box((1.00, 0.78, 1.55)),
        origin=Origin(xyz=(-0.95, 5.05, 2.915)),
        material=bearing_steel,
        name="left_bearing_pedestal",
    )
    shore_frame.visual(
        Box((1.00, 0.78, 1.55)),
        origin=Origin(xyz=(-0.95, -5.05, 2.915)),
        material=bearing_steel,
        name="right_bearing_pedestal",
    )
    shore_frame.visual(
        Cylinder(radius=0.44, length=0.32),
        origin=Origin(xyz=(-0.14, 4.58, 2.12), rpy=(1.57079632679, 0.0, 0.0)),
        material=bearing_steel,
        name="left_side_bearing",
    )
    shore_frame.visual(
        Cylinder(radius=0.44, length=0.32),
        origin=Origin(xyz=(-0.14, -4.58, 2.12), rpy=(1.57079632679, 0.0, 0.0)),
        material=bearing_steel,
        name="right_side_bearing",
    )
    shore_frame.visual(
        Box((0.55, 10.2, 0.22)),
        origin=Origin(xyz=(-0.60, 0.0, 3.35)),
        material=bearing_steel,
        name="pedestal_tie_beam",
    )
    shore_frame.inertial = Inertial.from_geometry(
        Box((4.2, 11.6, 4.0)),
        mass=46000.0,
        origin=Origin(xyz=(-2.40, 0.0, 2.05)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((13.0, 8.8, 0.12)),
        origin=Origin(xyz=(5.9, 0.0, 0.34)),
        material=bridge_steel,
        name="main_shell",
    )
    bridge_leaf.visual(
        Box((12.8, 0.22, 0.82)),
        origin=Origin(xyz=(5.85, 4.29, -0.02)),
        material=bridge_steel,
        name="left_girder",
    )
    bridge_leaf.visual(
        Box((12.8, 0.22, 0.82)),
        origin=Origin(xyz=(5.85, -4.29, -0.02)),
        material=bridge_steel,
        name="right_girder",
    )
    bridge_leaf.visual(
        Box((11.0, 8.38, 0.11)),
        origin=Origin(xyz=(5.1, 0.0, -0.35)),
        material=bridge_steel,
        name="bottom_plate",
    )
    bridge_leaf.visual(
        Box((0.28, 8.8, 0.82)),
        origin=Origin(xyz=(-0.46, 0.0, -0.02)),
        material=bridge_steel,
        name="heel_bulkhead",
    )
    bridge_leaf.visual(
        Box((0.18, 8.38, 0.28)),
        origin=Origin(xyz=(12.31, 0.0, -0.22)),
        material=bridge_steel,
        name="nose_bulkhead",
    )
    bridge_leaf.visual(
        Box((1.75, 8.5, 0.16)),
        origin=Origin(xyz=(0.35, 0.0, 0.48)),
        material=bearing_steel,
        name="reinforcement_band",
    )
    bridge_leaf.visual(
        Box((11.4, 7.9, 0.05)),
        origin=Origin(xyz=(6.1, 0.0, 0.425)),
        material=asphalt,
        name="deck_surface",
    )
    bridge_leaf.visual(
        Box((11.2, 0.18, 0.20)),
        origin=Origin(xyz=(6.2, 4.0, 0.50)),
        material=curb,
        name="left_curb",
    )
    bridge_leaf.visual(
        Box((11.2, 0.18, 0.20)),
        origin=Origin(xyz=(6.2, -4.0, 0.50)),
        material=curb,
        name="right_curb",
    )
    bridge_leaf.visual(
        Box((0.32, 1.2, 0.04)),
        origin=Origin(xyz=(6.1, 0.0, 0.445)),
        material=stripe,
        name="centerline_marking",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.24, length=0.60),
        origin=Origin(xyz=(0.02, 4.12, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=bearing_steel,
        name="left_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.24, length=0.60),
        origin=Origin(xyz=(0.02, -4.12, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=bearing_steel,
        name="right_trunnion",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((13.0, 8.8, 0.95)),
        mass=24000.0,
        origin=Origin(xyz=(5.9, 0.0, 0.0)),
    )

    model.articulation(
        "shore_to_bridge_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 2.12)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160000.0,
            velocity=0.30,
            lower=0.0,
            upper=1.15,
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
    leaf_hinge = object_model.get_articulation("shore_to_bridge_leaf")

    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="z",
        positive_elem="deck_surface",
        negative_elem="shore_roadway",
        min_gap=0.0,
        max_gap=0.02,
        name="closed roadway sits near the shore deck height",
    )
    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="y",
        elem_a="deck_surface",
        elem_b="shore_roadway",
        min_overlap=7.2,
        name="leaf roadway aligns laterally with the shore deck",
    )

    rest_tip = ctx.part_element_world_aabb(bridge_leaf, elem="nose_bulkhead")
    with ctx.pose({leaf_hinge: 1.0}):
        open_tip = ctx.part_element_world_aabb(bridge_leaf, elem="nose_bulkhead")
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="z",
            positive_elem="nose_bulkhead",
            negative_elem="shore_roadway",
            min_gap=8.0,
            name="opened leaf lifts well above the roadway",
        )

    ctx.check(
        "tip rises when the leaf opens",
        rest_tip is not None
        and open_tip is not None
        and open_tip[0][2] > rest_tip[0][2] + 7.0,
        details=f"rest_tip={rest_tip}, open_tip={open_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
