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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_branch_fixture_tree")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.21, 0.23, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    bearing_gray = model.material("bearing_gray", rgba=(0.42, 0.45, 0.46, 1.0))
    branch_blue = model.material("branch_blue", rgba=(0.10, 0.23, 0.42, 1.0))
    branch_orange = model.material("branch_orange", rgba=(0.78, 0.36, 0.08, 1.0))

    spine = model.part("spine")
    spine.visual(
        Box((0.30, 0.28, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="base_foot",
    )
    spine.visual(
        Box((0.16, 0.12, 1.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        material=painted_steel,
        name="rectangular_spine",
    )

    # Upper bracket: two side cheeks capture the rotating fork hub around a
    # horizontal cross-shaft on the +Y side of the tall spine.
    spine.visual(
        Box((0.14, 0.055, 0.18)),
        origin=Origin(xyz=(0.0, 0.0875, 1.045)),
        material=painted_steel,
        name="upper_hub_backplate",
    )
    for x, suffix in ((-0.092, "0"), (0.092, "1")):
        spine.visual(
            Box((0.030, 0.225, 0.24)),
            origin=Origin(xyz=(x, 0.1525, 1.045)),
            material=painted_steel,
            name=f"upper_cheek_{suffix}",
        )
    spine.visual(
        Cylinder(radius=0.018, length=0.260),
        origin=Origin(xyz=(0.0, 0.18, 1.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_gray,
        name="upper_cross_shaft",
    )
    for x, suffix in ((-0.136, "0"), (0.136, "1")):
        spine.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(xyz=(x, 0.18, 1.045), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"upper_shaft_cap_{suffix}",
        )

    # Lower swivel pedestal on the opposite (-Y) side of the spine.
    spine.visual(
        Box((0.19, 0.25, 0.15)),
        origin=Origin(xyz=(0.0, -0.155, 0.300)),
        material=painted_steel,
        name="lower_pedestal",
    )
    spine.visual(
        Cylinder(radius=0.065, length=0.055),
        origin=Origin(xyz=(0.0, -0.185, 0.3975)),
        material=dark_steel,
        name="lower_hub_plinth",
    )
    spine.visual(
        Cylinder(radius=0.023, length=0.145),
        origin=Origin(xyz=(0.0, -0.185, 0.4665)),
        material=bearing_gray,
        name="lower_stub_pin",
    )

    upper_fork = model.part("upper_fork")
    upper_fork.visual(
        Cylinder(radius=0.040, length=0.130),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=branch_blue,
        name="upper_barrel",
    )
    upper_fork.visual(
        Box((0.105, 0.090, 0.055)),
        origin=Origin(xyz=(0.0, 0.080, 0.0)),
        material=branch_blue,
        name="fork_neck",
    )
    for x, suffix in ((-0.040, "0"), (0.040, "1")):
        upper_fork.visual(
            Box((0.028, 0.270, 0.034)),
            origin=Origin(xyz=(x, 0.215, 0.0)),
            material=branch_blue,
            name=f"fork_tine_{suffix}",
        )
        upper_fork.visual(
            Cylinder(radius=0.017, length=0.030),
            origin=Origin(xyz=(x, 0.350, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=branch_blue,
            name=f"fork_round_tip_{suffix}",
        )
        upper_fork.visual(
            Cylinder(radius=0.010, length=0.032),
            origin=Origin(xyz=(x, 0.335, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"fork_tip_bushing_{suffix}",
        )

    lower_plate = model.part("lower_plate")
    lower_plate.visual(
        Cylinder(radius=0.055, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=branch_orange,
        name="lower_hub",
    )
    lower_plate.visual(
        Box((0.145, 0.370, 0.034)),
        origin=Origin(xyz=(0.0, -0.235, 0.048)),
        material=branch_orange,
        name="plate_arm",
    )
    lower_plate.visual(
        Box((0.045, 0.285, 0.018)),
        origin=Origin(xyz=(0.0, -0.250, 0.074)),
        material=dark_steel,
        name="plate_center_rib",
    )
    for x, suffix in ((-0.045, "0"), (0.045, "1")):
        lower_plate.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, -0.370, 0.069)),
            material=dark_steel,
            name=f"plate_bolt_boss_{suffix}",
        )

    model.articulation(
        "upper_hinge",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_fork,
        origin=Origin(xyz=(0.0, 0.18, 1.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-0.65, upper=0.95),
    )
    model.articulation(
        "lower_swivel",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_plate,
        origin=Origin(xyz=(0.0, -0.185, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.2, lower=-1.40, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    upper_fork = object_model.get_part("upper_fork")
    lower_plate = object_model.get_part("lower_plate")
    upper_hinge = object_model.get_articulation("upper_hinge")
    lower_swivel = object_model.get_articulation("lower_swivel")

    ctx.allow_overlap(
        spine,
        upper_fork,
        elem_a="upper_cross_shaft",
        elem_b="upper_barrel",
        reason="The fixed cross-shaft is intentionally captured inside the rotating upper hub proxy.",
    )
    ctx.expect_within(
        spine,
        upper_fork,
        axes="yz",
        inner_elem="upper_cross_shaft",
        outer_elem="upper_barrel",
        margin=0.0,
        name="upper shaft is centered inside barrel",
    )
    ctx.expect_overlap(
        spine,
        upper_fork,
        axes="x",
        elem_a="upper_cross_shaft",
        elem_b="upper_barrel",
        min_overlap=0.12,
        name="upper cross-shaft spans the rotating barrel",
    )

    ctx.allow_overlap(
        spine,
        lower_plate,
        elem_a="lower_stub_pin",
        elem_b="lower_hub",
        reason="The fixed vertical stub is intentionally nested inside the lower swivel hub proxy.",
    )
    ctx.expect_within(
        spine,
        lower_plate,
        axes="xy",
        inner_elem="lower_stub_pin",
        outer_elem="lower_hub",
        margin=0.0,
        name="lower stub is centered inside hub",
    )
    ctx.expect_overlap(
        spine,
        lower_plate,
        axes="z",
        elem_a="lower_stub_pin",
        elem_b="lower_hub",
        min_overlap=0.075,
        name="lower stub retains vertical insertion",
    )

    fork_rest = ctx.part_element_world_aabb(upper_fork, elem="fork_round_tip_0")
    with ctx.pose({upper_hinge: 0.80}):
        fork_lifted = ctx.part_element_world_aabb(upper_fork, elem="fork_round_tip_0")
    ctx.check(
        "upper fork lifts about horizontal shaft",
        fork_rest is not None
        and fork_lifted is not None
        and fork_lifted[0][2] > fork_rest[0][2] + 0.20,
        details=f"rest={fork_rest}, lifted={fork_lifted}",
    )

    plate_rest = ctx.part_element_world_aabb(lower_plate, elem="plate_bolt_boss_1")
    with ctx.pose({lower_swivel: 1.00}):
        plate_swung = ctx.part_element_world_aabb(lower_plate, elem="plate_bolt_boss_1")
    ctx.check(
        "lower plate swings about vertical stub",
        plate_rest is not None
        and plate_swung is not None
        and plate_swung[0][0] > plate_rest[0][0] + 0.20,
        details=f"rest={plate_rest}, swung={plate_swung}",
    )

    return ctx.report()


object_model = build_object_model()
