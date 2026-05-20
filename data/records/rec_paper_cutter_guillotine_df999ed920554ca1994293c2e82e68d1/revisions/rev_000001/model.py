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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_WIDTH = 0.300
BASE_LENGTH = 0.460
BASE_THICKNESS = 0.022

ARM_LENGTH = 0.430
ARM_WIDTH = 0.072
ARM_BEAM_THICKNESS = 0.010
ARM_HINGE_RADIUS = 0.009

WING_WIDTH = 0.180
WING_LENGTH = 0.200
WING_THICKNESS = 0.014
WING_HINGE_RADIUS = 0.005


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="school_paper_cutter")

    model.material("laminate_board", color=(0.73, 0.68, 0.56))
    model.material("cut_strip", color=(0.30, 0.24, 0.16))
    model.material("painted_steel", color=(0.42, 0.45, 0.49))
    model.material("blade_steel", color=(0.78, 0.80, 0.82))
    model.material("grip", color=(0.12, 0.12, 0.12))

    base = model.part("base")
    base.visual(
        Box((BASE_WIDTH, BASE_LENGTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="laminate_board",
        name="deck",
    )
    base.visual(
        Box((0.018, BASE_LENGTH, BASE_THICKNESS)),
        origin=Origin(
            xyz=(-BASE_WIDTH / 2.0 + 0.009, 0.0, BASE_THICKNESS / 2.0),
        ),
        material="painted_steel",
        name="hinge_rail",
    )
    base.visual(
        Box((0.016, BASE_LENGTH, 0.002)),
        origin=Origin(xyz=(-BASE_WIDTH / 2.0 + 0.040, 0.0, BASE_THICKNESS - 0.001)),
        material="cut_strip",
        name="cut_strip",
    )
    base.visual(
        Box((0.012, 0.220, 0.010)),
        origin=Origin(xyz=(BASE_WIDTH / 2.0 - 0.006, 0.0, 0.003)),
        material="painted_steel",
        name="wing_mount",
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Cylinder(radius=ARM_HINGE_RADIUS, length=ARM_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="painted_steel",
        name="arm_hinge_barrel",
    )
    blade_arm.visual(
        Box((ARM_WIDTH, ARM_LENGTH, ARM_BEAM_THICKNESS)),
        origin=Origin(xyz=(ARM_WIDTH / 2.0 - 0.004, 0.0, 0.0)),
        material="painted_steel",
        name="arm_beam",
    )
    blade_arm.visual(
        Box((0.008, ARM_LENGTH * 0.98, 0.004)),
        origin=Origin(xyz=(ARM_WIDTH - 0.006, 0.0, -0.007)),
        material="blade_steel",
        name="blade_edge",
    )
    blade_arm.visual(
        Cylinder(radius=0.010, length=0.220),
        origin=Origin(xyz=(ARM_WIDTH * 0.56, 0.0, 0.014), rpy=(pi / 2.0, 0.0, 0.0)),
        material="grip",
        name="handle_grip",
    )

    extension_wing = model.part("extension_wing")
    extension_wing.visual(
        Cylinder(radius=WING_HINGE_RADIUS, length=WING_LENGTH - 0.020),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="painted_steel",
        name="wing_hinge",
    )
    extension_wing.visual(
        Box((0.018, WING_LENGTH, 0.034)),
        origin=Origin(xyz=(-0.009, 0.0, -0.017)),
        material="painted_steel",
        name="wing_bracket",
    )
    extension_wing.visual(
        Box((WING_WIDTH, WING_LENGTH, WING_THICKNESS)),
        origin=Origin(xyz=(-WING_WIDTH / 2.0, 0.0, -0.017)),
        material="laminate_board",
        name="wing_panel",
    )

    model.articulation(
        "base_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(-BASE_WIDTH / 2.0, 0.0, BASE_THICKNESS + ARM_HINGE_RADIUS)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "base_to_extension_wing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=extension_wing,
        origin=Origin(xyz=(BASE_WIDTH / 2.0 + WING_HINGE_RADIUS, 0.0, -0.002)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=pi,
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

    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    extension_wing = object_model.get_part("extension_wing")
    blade_joint = object_model.get_articulation("base_to_blade_arm")
    wing_joint = object_model.get_articulation("base_to_extension_wing")

    ctx.check(
        "all named parts are present",
        all(part is not None for part in (base, blade_arm, extension_wing)),
    )

    with ctx.pose({blade_joint: 0.0}):
        ctx.expect_gap(
            blade_arm,
            base,
            axis="z",
            positive_elem="blade_edge",
            negative_elem="cut_strip",
            max_gap=0.001,
            max_penetration=0.0,
            name="blade edge seats on the cutter strip when closed",
        )
        ctx.expect_overlap(
            blade_arm,
            base,
            axes="xy",
            elem_a="arm_beam",
            elem_b="deck",
            min_overlap=0.050,
            name="closed arm covers the board",
        )

    blade_rest_aabb = ctx.part_world_aabb(blade_arm)
    with ctx.pose({blade_joint: blade_joint.motion_limits.upper}):
        blade_open_aabb = ctx.part_world_aabb(blade_arm)
    ctx.check(
        "blade arm lifts upward on its side hinge",
        blade_rest_aabb is not None
        and blade_open_aabb is not None
        and blade_open_aabb[1][2] > blade_rest_aabb[1][2] + 0.030,
        details=f"rest={blade_rest_aabb}, open={blade_open_aabb}",
    )

    with ctx.pose({wing_joint: 0.0}):
        ctx.expect_gap(
            base,
            extension_wing,
            axis="z",
            positive_elem="deck",
            negative_elem="wing_panel",
            min_gap=0.008,
            max_gap=0.014,
            name="wing stores folded beneath the base",
        )
        ctx.expect_overlap(
            extension_wing,
            base,
            axes="xy",
            elem_a="wing_panel",
            elem_b="deck",
            min_overlap=0.100,
            name="stored wing tucks under the board footprint",
        )

    with ctx.pose({wing_joint: wing_joint.motion_limits.upper}):
        ctx.expect_gap(
            extension_wing,
            base,
            axis="x",
            positive_elem="wing_panel",
            negative_elem="deck",
            max_gap=0.010,
            max_penetration=0.0,
            name="wing opens outward from the far edge",
        )
        ctx.expect_overlap(
            extension_wing,
            base,
            axes="y",
            elem_a="wing_panel",
            elem_b="deck",
            min_overlap=0.180,
            name="opened wing stays aligned with the board length",
        )
        deck_aabb = ctx.part_element_world_aabb(base, elem="deck")
        wing_aabb = ctx.part_element_world_aabb(extension_wing, elem="wing_panel")
    ctx.check(
        "opened wing top sits level with the main board",
        deck_aabb is not None
        and wing_aabb is not None
        and abs(deck_aabb[1][2] - wing_aabb[1][2]) <= 0.002,
        details=f"deck={deck_aabb}, wing={wing_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
