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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="photo_paper_cutter")

    model.material("base_gray", rgba=(0.76, 0.77, 0.79, 1.0))
    model.material("accent_red", rgba=(0.74, 0.12, 0.08, 1.0))
    model.material("arm_black", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("dark_trim", rgba=(0.07, 0.08, 0.09, 1.0))
    model.material("smoke_clear", rgba=(0.74, 0.84, 0.90, 0.35))

    base_length = 0.46
    base_width = 0.31
    deck_thickness = 0.012
    arm_length = 0.44
    hinge_y = -(base_width / 2.0) + 0.012
    arm_hinge_z = deck_thickness + 0.006

    base = model.part("base")
    base.visual(
        Box((base_length, base_width, deck_thickness)),
        origin=Origin(xyz=(0.0, 0.0, deck_thickness / 2.0)),
        material="base_gray",
        name="deck_panel",
    )
    base.visual(
        Box((arm_length, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, hinge_y, deck_thickness + 0.005)),
        material="dark_trim",
        name="hinge_rail",
    )
    base.visual(
        Box((arm_length, 0.018, 0.010)),
        origin=Origin(
            xyz=(0.0, (base_width / 2.0) - 0.009, deck_thickness + 0.005)
        ),
        material="base_gray",
        name="alignment_fence",
    )
    base.visual(
        Box((arm_length * 0.95, 0.004, 0.001)),
        origin=Origin(xyz=(0.0, hinge_y + 0.062, deck_thickness + 0.0005)),
        material="accent_red",
        name="cut_line",
    )

    arm = model.part("cutting_arm")
    arm.visual(
        Box((arm_length, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, 0.031, 0.013)),
        material="arm_black",
        name="arm_body",
    )
    arm.visual(
        Box((arm_length, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.051, 0.005)),
        material="dark_trim",
        name="blade_channel",
    )
    arm.visual(
        Cylinder(radius=0.011, length=0.13),
        origin=Origin(xyz=(0.12, 0.031, 0.031), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_trim",
        name="handle_grip",
    )
    arm.visual(
        Box((0.050, 0.026, 0.032)),
        origin=Origin(xyz=(0.190, 0.031, 0.020)),
        material="arm_black",
        name="handle_block",
    )

    guard = model.part("safety_guard")
    guard.visual(
        Box((0.42, 0.042, 0.003)),
        origin=Origin(xyz=(0.0, -0.021, -0.0015)),
        material="smoke_clear",
        name="guard_panel",
    )
    guard.visual(
        Cylinder(radius=0.004, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_trim",
        name="guard_hinge_tube",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, hinge_y, arm_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "arm_to_guard",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=guard,
        origin=Origin(xyz=(0.0, 0.051, 0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
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

    base = object_model.get_part("base")
    arm = object_model.get_part("cutting_arm")
    guard = object_model.get_part("safety_guard")
    arm_hinge = object_model.get_articulation("base_to_arm")
    guard_hinge = object_model.get_articulation("arm_to_guard")

    ctx.expect_gap(
        guard,
        base,
        axis="z",
        positive_elem="guard_panel",
        negative_elem="deck_panel",
        min_gap=0.001,
        max_gap=0.008,
        name="closed guard floats just above the deck",
    )
    ctx.expect_gap(
        arm,
        guard,
        axis="z",
        positive_elem="arm_body",
        negative_elem="guard_panel",
        min_gap=0.0005,
        max_gap=0.006,
        name="guard tucks directly beneath the cutter arm",
    )
    ctx.expect_overlap(
        guard,
        base,
        axes="xy",
        elem_a="guard_panel",
        elem_b="deck_panel",
        min_overlap=0.03,
        name="guard sits over the paper deck",
    )
    ctx.expect_overlap(
        guard,
        arm,
        axes="xy",
        elem_a="guard_panel",
        elem_b="arm_body",
        min_overlap=0.03,
        name="guard remains aligned under the cutter arm",
    )

    with ctx.pose({arm_hinge: 1.05}):
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem="arm_body",
            negative_elem="hinge_rail",
            min_gap=0.0,
            max_penetration=0.0,
            name="raised arm clears the hinge rail instead of intersecting it",
        )
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem="blade_channel",
            negative_elem="deck_panel",
            min_gap=0.045,
            name="raised arm lifts the blade channel above the deck",
        )

    with ctx.pose({arm_hinge: 0.90, guard_hinge: 0.0}):
        guard_down_aabb = ctx.part_element_world_aabb(guard, elem="guard_panel")

    with ctx.pose({arm_hinge: 0.90, guard_hinge: 0.85}):
        guard_up_aabb = ctx.part_element_world_aabb(guard, elem="guard_panel")

    down_min_z = None if guard_down_aabb is None else guard_down_aabb[0][2]
    up_min_z = None if guard_up_aabb is None else guard_up_aabb[0][2]
    ctx.check(
        "guard swings away from the cutter arm on its own hinge",
        down_min_z is not None
        and up_min_z is not None
        and up_min_z < down_min_z - 0.005,
        details=f"guard_rest_min_z={down_min_z}, guard_swung_min_z={up_min_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
