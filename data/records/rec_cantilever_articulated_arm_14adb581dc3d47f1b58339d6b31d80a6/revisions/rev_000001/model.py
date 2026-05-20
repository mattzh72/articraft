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


def _cyl_along_y(x: float, y: float, z: float) -> Origin:
    """URDF cylinders are along local Z; rotate them so the pin axis is Y."""

    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _cyl_along_x(x: float, y: float, z: float) -> Origin:
    """URDF cylinders are along local Z; rotate them so the pin axis is X."""

    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_cantilever_arm")

    painted = Material("warm_safety_orange", color=(0.95, 0.34, 0.08, 1.0))
    dark = Material("dark_powder_coat", color=(0.04, 0.045, 0.05, 1.0))
    steel = Material("brushed_steel", color=(0.62, 0.64, 0.63, 1.0))
    rubber = Material("black_bushing", color=(0.01, 0.01, 0.012, 1.0))

    # Intrinsic frame: +X is the cantilever reach direction, +Z is up, and the
    # revolute pins run laterally along Y.  The shoulder joint is deliberately
    # below the overhead support, giving the arm its under-slung silhouette.
    shoulder_z = 0.0
    upper_len = 0.52
    upper_drop = -0.055
    fore_len = 0.42
    fore_drop = -0.045

    support = model.part("top_support")
    support.visual(
        Box((0.78, 0.16, 0.060)),
        origin=Origin(xyz=(0.28, 0.0, 0.170)),
        material=dark,
        name="top_beam",
    )
    support.visual(
        Box((0.44, 0.24, 0.025)),
        origin=Origin(xyz=(0.06, 0.0, 0.2125)),
        material=steel,
        name="ceiling_plate",
    )
    support.visual(
        Box((0.12, 0.025, 0.22)),
        origin=Origin(xyz=(0.0, 0.0555, 0.050)),
        material=dark,
        name="shoulder_cheek_0",
    )
    support.visual(
        Box((0.12, 0.025, 0.22)),
        origin=Origin(xyz=(0.0, -0.0555, 0.050)),
        material=dark,
        name="shoulder_cheek_1",
    )
    support.visual(
        Box((0.13, 0.155, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=dark,
        name="shoulder_cap",
    )
    support.visual(
        Box((0.09, 0.11, 0.070)),
        origin=Origin(xyz=(-0.018, 0.0, 0.105)),
        material=dark,
        name="drop_web",
    )
    support.visual(
        Cylinder(radius=0.035, length=0.018),
        origin=_cyl_along_y(0.0, 0.077, shoulder_z),
        material=steel,
        name="outer_pivot_washer_0",
    )
    support.visual(
        Cylinder(radius=0.035, length=0.018),
        origin=_cyl_along_y(0.0, -0.077, shoulder_z),
        material=steel,
        name="outer_pivot_washer_1",
    )

    upper = model.part("upper_arm")
    upper.visual(
        Cylinder(radius=0.052, length=0.086),
        origin=_cyl_along_y(0.0, 0.0, 0.0),
        material=rubber,
        name="shoulder_hub",
    )
    for y in (-0.030, 0.030):
        upper.visual(
            Box((0.44, 0.020, 0.050)),
            origin=Origin(xyz=(0.26, y, upper_drop)),
            material=painted,
            name=f"upper_side_rail_{0 if y < 0 else 1}",
        )
    upper.visual(
        Box((0.030, 0.124, 0.024)),
        origin=Origin(xyz=(0.18, 0.0, upper_drop)),
        material=painted,
        name="upper_cross_tie_0",
    )
    upper.visual(
        Box((0.050, 0.124, 0.024)),
        origin=Origin(xyz=(0.457, 0.0, upper_drop)),
        material=painted,
        name="upper_cross_tie_1",
    )
    # Fork cheeks at the distal end carry the elbow pin while leaving a clear
    # open gap for the next rigid stage's hub.
    for y in (-0.050, 0.050):
        upper.visual(
            Box((0.090, 0.024, 0.110)),
            origin=Origin(xyz=(upper_len, y, upper_drop)),
            material=painted,
            name=f"elbow_fork_cheek_{0 if y < 0 else 1}",
        )
    upper.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=_cyl_along_y(upper_len, 0.072, upper_drop),
        material=steel,
        name="elbow_washer_0",
    )
    upper.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=_cyl_along_y(upper_len, -0.072, upper_drop),
        material=steel,
        name="elbow_washer_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.036, length=0.076),
        origin=_cyl_along_y(0.0, 0.0, 0.0),
        material=rubber,
        name="elbow_hub",
    )
    for y in (-0.0285, 0.0285):
        forearm.visual(
            Box((0.342, 0.018, 0.044)),
            origin=Origin(xyz=(0.193, y, fore_drop)),
            material=painted,
            name=f"forearm_side_rail_{0 if y < 0 else 1}",
        )
    forearm.visual(
        Box((0.026, 0.085, 0.020)),
        origin=Origin(xyz=(0.165, 0.0, fore_drop)),
        material=painted,
        name="forearm_cross_tie",
    )
    forearm.visual(
        Box((0.026, 0.102, 0.020)),
        origin=Origin(xyz=(0.372, 0.0, fore_drop)),
        material=painted,
        name="wrist_rear_bridge",
    )
    for y in (-0.039, 0.039):
        forearm.visual(
            Box((0.080, 0.020, 0.095)),
            origin=Origin(xyz=(fore_len, y, fore_drop)),
            material=painted,
            name=f"wrist_fork_cheek_{0 if y < 0 else 1}",
        )
    forearm.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=_cyl_along_y(fore_len, 0.058, fore_drop),
        material=steel,
        name="wrist_washer_0",
    )
    forearm.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=_cyl_along_y(fore_len, -0.058, fore_drop),
        material=steel,
        name="wrist_washer_1",
    )

    wrist = model.part("wrist_stage")
    wrist.visual(
        Cylinder(radius=0.030, length=0.058),
        origin=_cyl_along_y(0.0, 0.0, 0.0),
        material=rubber,
        name="wrist_hub",
    )
    wrist.visual(
        Box((0.205, 0.055, 0.040)),
        origin=Origin(xyz=(0.120, 0.0, -0.035)),
        material=painted,
        name="wrist_link",
    )
    wrist.visual(
        Cylinder(radius=0.048, length=0.026),
        origin=_cyl_along_x(0.235, 0.0, -0.035),
        material=steel,
        name="tool_flange",
    )
    for y, z in ((0.020, -0.015), (-0.020, -0.015), (0.020, -0.055), (-0.020, -0.055)):
        wrist.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=_cyl_along_x(0.252, y, z),
            material=dark,
            name=f"flange_bolt_{len(wrist.visuals) - 3}",
        )

    shoulder = model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=support,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-0.85, upper=1.05),
    )
    elbow = model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(upper_len, 0.0, upper_drop)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=2.2, lower=-1.8, upper=1.45),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(fore_len, 0.0, fore_drop)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.0, lower=-1.7, upper=1.7),
    )

    # Keep handles to make intentional sequencing clear for readers and tests.
    model.meta["chain_order"] = [shoulder.name, elbow.name, "wrist"]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    ctx.check(
        "three sequential revolute joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (shoulder, elbow, wrist))
        and object_model.meta.get("chain_order") == ["shoulder", "elbow", "wrist"],
        details="Arm should be a shoulder-elbow-wrist revolute chain.",
    )

    top_beam_aabb = ctx.part_element_world_aabb("top_support", elem="top_beam")
    upper_aabb = ctx.part_world_aabb("upper_arm")
    forearm_aabb = ctx.part_world_aabb("forearm")
    wrist_aabb = ctx.part_world_aabb("wrist_stage")
    if top_beam_aabb and upper_aabb and forearm_aabb and wrist_aabb:
        top_beam_bottom = top_beam_aabb[0][2]
        highest_arm_surface = max(upper_aabb[1][2], forearm_aabb[1][2], wrist_aabb[1][2])
        ctx.check(
            "arm stages hang below top beam",
            highest_arm_surface < top_beam_bottom - 0.035,
            details=f"top_beam_bottom={top_beam_bottom:.3f}, highest_arm_surface={highest_arm_surface:.3f}",
        )
    else:
        ctx.fail("arm stages hang below top beam", "Could not measure support and arm AABBs.")

    ctx.expect_within(
        "upper_arm",
        "top_support",
        axes="y",
        inner_elem="shoulder_hub",
        outer_elem="shoulder_cap",
        margin=0.0,
        name="shoulder hub is captured between support cheeks",
    )
    ctx.expect_within(
        "forearm",
        "upper_arm",
        axes="y",
        inner_elem="elbow_hub",
        outer_elem="upper_cross_tie_1",
        margin=0.004,
        name="elbow hub fits within upper fork span",
    )
    ctx.expect_within(
        "wrist_stage",
        "forearm",
        axes="y",
        inner_elem="wrist_hub",
        outer_elem="forearm_cross_tie",
        margin=0.004,
        name="wrist hub fits within forearm fork span",
    )

    rest_wrist_position = ctx.part_world_position("wrist_stage")
    with ctx.pose({elbow: 0.75}):
        bent_wrist_position = ctx.part_world_position("wrist_stage")
    ctx.check(
        "elbow bend moves distal stage",
        rest_wrist_position is not None
        and bent_wrist_position is not None
        and bent_wrist_position[2] < rest_wrist_position[2] - 0.08,
        details=f"rest={rest_wrist_position}, bent={bent_wrist_position}",
    )

    return ctx.report()


object_model = build_object_model()
