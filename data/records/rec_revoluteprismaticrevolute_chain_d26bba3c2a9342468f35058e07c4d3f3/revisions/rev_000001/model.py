from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((a + b) * 0.5 for a, b in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolute_prismatic_revolute_chain")

    model.material("base_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("arm_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    model.material("slider_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("fork_orange", rgba=(0.84, 0.37, 0.16, 1.0))

    ground = model.part("ground_support")
    ground.visual(
        Box((0.18, 0.12, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="base_gray",
        name="base_plate",
    )
    ground.visual(
        Box((0.06, 0.04, 0.05)),
        origin=Origin(xyz=(-0.04, 0.0, 0.037)),
        material="base_gray",
        name="pedestal",
    )
    ground.visual(
        Box((0.012, 0.008, 0.05)),
        origin=Origin(xyz=(0.0, 0.012, 0.071)),
        material="base_gray",
        name="left_cheek",
    )
    ground.visual(
        Box((0.012, 0.008, 0.05)),
        origin=Origin(xyz=(0.0, -0.012, 0.071)),
        material="base_gray",
        name="right_cheek",
    )
    ground.visual(
        Box((0.024, 0.024, 0.03)),
        origin=Origin(xyz=(-0.018, 0.0, 0.073)),
        material="base_gray",
        name="rear_web",
    )

    arm = model.part("rotating_arm")
    arm.visual(
        Box((0.020, 0.016, 0.016)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material="arm_gray",
        name="hinge_tongue",
    )
    arm.visual(
        Box((0.094, 0.006, 0.014)),
        origin=Origin(xyz=(0.057, 0.008, 0.0)),
        material="arm_gray",
        name="left_rail",
    )
    arm.visual(
        Box((0.094, 0.006, 0.014)),
        origin=Origin(xyz=(0.057, -0.008, 0.0)),
        material="arm_gray",
        name="right_rail",
    )
    arm.visual(
        Box((0.088, 0.010, 0.002)),
        origin=Origin(xyz=(0.056, 0.0, -0.007)),
        material="arm_gray",
        name="floor_rail",
    )
    arm.visual(
        Box((0.010, 0.022, 0.004)),
        origin=Origin(xyz=(0.099, 0.0, 0.009)),
        material="arm_gray",
        name="front_bridge",
    )

    slider = model.part("sliding_link")
    slider.visual(
        Box((0.084, 0.010, 0.010)),
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        material="slider_steel",
        name="main_bar",
    )
    slider.visual(
        Box((0.070, 0.006, 0.002)),
        origin=Origin(xyz=(0.035, 0.0, -0.005)),
        material="slider_steel",
        name="runner",
    )
    slider.visual(
        Box((0.012, 0.010, 0.010)),
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        material="slider_steel",
        name="clevis_base",
    )
    slider.visual(
        Box((0.012, 0.006, 0.016)),
        origin=Origin(xyz=(0.090, 0.008, 0.0)),
        material="slider_steel",
        name="left_lug",
    )
    slider.visual(
        Box((0.012, 0.006, 0.016)),
        origin=Origin(xyz=(0.090, -0.008, 0.0)),
        material="slider_steel",
        name="right_lug",
    )

    fork = model.part("distal_fork")
    fork.visual(
        Box((0.006, 0.010, 0.016)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material="fork_orange",
        name="pivot_tab",
    )
    fork.visual(
        Box((0.030, 0.008, 0.004)),
        origin=Origin(xyz=(0.021, 0.0, 0.006)),
        material="fork_orange",
        name="upper_tine",
    )
    fork.visual(
        Box((0.030, 0.008, 0.004)),
        origin=Origin(xyz=(0.021, 0.0, -0.006)),
        material="fork_orange",
        name="lower_tine",
    )
    fork.visual(
        Box((0.006, 0.008, 0.016)),
        origin=Origin(xyz=(0.039, 0.0, 0.0)),
        material="fork_orange",
        name="tip_bridge",
    )

    model.articulation(
        "root_hinge",
        ArticulationType.REVOLUTE,
        parent=ground,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    model.articulation(
        "arm_slider",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=slider,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=0.0, upper=0.060),
    )

    model.articulation(
        "slider_fork",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=fork,
        origin=Origin(xyz=(0.096, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.80, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground = object_model.get_part("ground_support")
    arm = object_model.get_part("rotating_arm")
    slider = object_model.get_part("sliding_link")
    fork = object_model.get_part("distal_fork")

    root = object_model.get_articulation("root_hinge")
    slide = object_model.get_articulation("arm_slider")
    wrist = object_model.get_articulation("slider_fork")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "joint types match R-P-R chain",
        root.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC
        and wrist.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"got {[root.articulation_type, slide.articulation_type, wrist.articulation_type]}"
        ),
    )
    ctx.check(
        "joint axes match intended mechanism",
        tuple(root.axis) == (0.0, -1.0, 0.0)
        and tuple(slide.axis) == (1.0, 0.0, 0.0)
        and tuple(wrist.axis) == (0.0, -1.0, 0.0),
        details=f"axes are root={root.axis}, slide={slide.axis}, wrist={wrist.axis}",
    )

    with ctx.pose(root_hinge=0.0, arm_slider=0.0, slider_fork=0.0):
        ctx.expect_contact(arm, ground, name="root hinge hardware remains seated")
        ctx.expect_contact(slider, arm, name="slider is supported by arm guide")
        ctx.expect_contact(fork, slider, name="distal fork sits on wrist pin support")
        ctx.expect_overlap(
            slider,
            arm,
            axes="x",
            min_overlap=0.08,
            name="slider remains substantially engaged in guide at rest",
        )

    with ctx.pose(root_hinge=0.90, arm_slider=0.060, slider_fork=0.55):
        ctx.fail_if_parts_overlap_in_current_pose(name="deployed pose stays clear")
        ctx.expect_overlap(
            slider,
            arm,
            axes="x",
            min_overlap=0.015,
            name="extended slider still remains partially engaged in guide",
        )

    with ctx.pose(root_hinge=0.0, arm_slider=0.0, slider_fork=0.0):
        retracted_slider = ctx.part_world_position(slider)
        neutral_fork_center = _aabb_center(ctx.part_element_world_aabb(fork, elem="tip_bridge"))
    with ctx.pose(root_hinge=0.0, arm_slider=0.060, slider_fork=0.0):
        extended_slider = ctx.part_world_position(slider)
    with ctx.pose(root_hinge=0.90, arm_slider=0.0, slider_fork=0.0):
        lifted_fork_center = _aabb_center(ctx.part_element_world_aabb(fork, elem="tip_bridge"))
    with ctx.pose(root_hinge=0.40, arm_slider=0.050, slider_fork=0.0):
        neutral_pitch_center = _aabb_center(ctx.part_element_world_aabb(fork, elem="tip_bridge"))
    with ctx.pose(root_hinge=0.40, arm_slider=0.050, slider_fork=0.70):
        pitched_fork_center = _aabb_center(ctx.part_element_world_aabb(fork, elem="tip_bridge"))

    ctx.check(
        "prismatic stage extends outward",
        retracted_slider is not None
        and extended_slider is not None
        and extended_slider[0] > retracted_slider[0] + 0.05,
        details=f"retracted={retracted_slider}, extended={extended_slider}",
    )
    ctx.check(
        "root hinge lifts distal end upward",
        neutral_fork_center is not None
        and lifted_fork_center is not None
        and lifted_fork_center[2] > neutral_fork_center[2] + 0.08,
        details=f"neutral={neutral_fork_center}, lifted={lifted_fork_center}",
    )
    ctx.check(
        "distal revolute joint pitches the fork",
        neutral_pitch_center is not None
        and pitched_fork_center is not None
        and pitched_fork_center[2] > neutral_pitch_center[2] + 0.008,
        details=f"neutral={neutral_pitch_center}, pitched={pitched_fork_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
