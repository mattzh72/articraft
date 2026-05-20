from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="craft_guillotine_cutter")

    board = model.material("warm_gray_board", rgba=(0.72, 0.70, 0.64, 1.0))
    mat = model.material("green_cutting_mat", rgba=(0.18, 0.42, 0.32, 1.0))
    dark = model.material("dark_alignment_markings", rgba=(0.06, 0.07, 0.07, 1.0))
    fence_mat = model.material("black_anodized_fence", rgba=(0.04, 0.045, 0.045, 1.0))
    steel = model.material("brushed_steel", rgba=(0.74, 0.76, 0.76, 1.0))
    red = model.material("red_blade_arm", rgba=(0.72, 0.04, 0.03, 1.0))
    rubber = model.material("black_rubber_grip", rgba=(0.015, 0.015, 0.014, 1.0))
    clear = model.material("clear_blue_polycarbonate", rgba=(0.45, 0.78, 1.0, 0.32))

    bed = model.part("bed")
    bed.visual(
        Box((0.60, 0.36, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=board,
        name="base_board",
    )
    bed.visual(
        Box((0.54, 0.28, 0.004)),
        origin=Origin(xyz=(0.018, 0.015, 0.034)),
        material=mat,
        name="cutting_mat",
    )
    bed.visual(
        Box((0.56, 0.014, 0.004)),
        origin=Origin(xyz=(0.020, -0.128, 0.039)),
        material=steel,
        name="cutting_strip",
    )
    bed.visual(
        Box((0.55, 0.018, 0.026)),
        origin=Origin(xyz=(0.010, 0.162, 0.049)),
        material=fence_mat,
        name="alignment_fence",
    )
    bed.visual(
        Box((0.018, 0.30, 0.018)),
        origin=Origin(xyz=(-0.268, 0.006, 0.045)),
        material=fence_mat,
        name="rear_square_fence",
    )

    # Shallow alignment/ruler marks are part of the fixed bed surface.
    for i, x in enumerate((-0.20, -0.10, 0.0, 0.10, 0.20)):
        bed.visual(
            Box((0.003, 0.255, 0.0012)),
            origin=Origin(xyz=(x, 0.015, 0.0366)),
            material=dark,
            name=f"grid_line_x_{i}",
        )
    for i, y in enumerate((-0.060, 0.020, 0.100)):
        bed.visual(
            Box((0.52, 0.0025, 0.0012)),
            origin=Origin(xyz=(0.018, y, 0.0368)),
            material=dark,
            name=f"grid_line_y_{i}",
        )

    # Corner yoke supporting the guillotine arm pivot.
    pivot_x = -0.260
    pivot_y = -0.135
    pivot_z = 0.077
    for name, y in (("pivot_cheek_0", -0.159), ("pivot_cheek_1", -0.111)):
        bed.visual(
            Box((0.050, 0.012, 0.076)),
            origin=Origin(xyz=(pivot_x, y, 0.070)),
            material=steel,
            name=name,
        )
    bed.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(pivot_x, -0.170, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin_head_0",
    )
    bed.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(pivot_x, -0.100, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin_head_1",
    )

    # Alternating fixed knuckles for the clear guard hinge near the cut line.
    guard_hinge_y = -0.095
    guard_hinge_z = 0.064
    for i, x in enumerate((-0.105, 0.105)):
        bed.visual(
            Box((0.130, 0.018, 0.038)),
            origin=Origin(xyz=(x, guard_hinge_y, 0.046)),
            material=steel,
            name=f"guard_hinge_stand_{i}",
        )
        bed.visual(
            Cylinder(radius=0.009, length=0.130),
            origin=Origin(xyz=(x, guard_hinge_y, guard_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"guard_hinge_knuckle_{i}",
        )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Cylinder(radius=0.023, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="pivot_hub",
    )
    blade_arm.visual(
        Box((0.505, 0.030, 0.030)),
        origin=Origin(xyz=(0.265, 0.000, 0.022)),
        material=red,
        name="arm_bar",
    )
    blade_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.030, 0.004),
                (0.500, 0.004),
                (0.500, -0.022),
                (0.060, -0.036),
            ]
        )
        .close()
        .extrude(0.006)
    )
    blade_arm.visual(
        mesh_from_cadquery(blade_profile, "sloped_steel_blade"),
        origin=Origin(xyz=(0.0, 0.004, 0.003)),
        material=steel,
        name="sloped_blade",
    )
    blade_arm.visual(
        Cylinder(radius=0.018, length=0.096),
        origin=Origin(xyz=(0.525, 0.0, 0.046), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_grip",
    )
    blade_arm.visual(
        Box((0.035, 0.026, 0.050)),
        origin=Origin(xyz=(0.505, 0.0, 0.034)),
        material=red,
        name="grip_neck",
    )

    finger_guard = model.part("finger_guard")
    finger_guard.visual(
        Box((0.490, 0.135, 0.004)),
        origin=Origin(xyz=(0.000, 0.080, -0.009)),
        material=clear,
        name="clear_panel",
    )
    for i, x in enumerate((-0.210, 0.000, 0.210)):
        finger_guard.visual(
            Cylinder(radius=0.010, length=0.070),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=clear,
            name=f"clip_loop_{i}",
        )
        finger_guard.visual(
            Box((0.052, 0.040, 0.006)),
            origin=Origin(xyz=(x, 0.023, -0.008)),
            material=clear,
            name=f"clip_tab_{i}",
        )

    model.articulation(
        "bed_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=blade_arm,
        origin=Origin(xyz=(pivot_x, pivot_y, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "bed_to_finger_guard",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=finger_guard,
        origin=Origin(xyz=(0.0, guard_hinge_y, guard_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.2, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed")
    blade_arm = object_model.get_part("blade_arm")
    finger_guard = object_model.get_part("finger_guard")
    blade_joint = object_model.get_articulation("bed_to_blade_arm")
    guard_joint = object_model.get_articulation("bed_to_finger_guard")

    ctx.expect_overlap(
        blade_arm,
        bed,
        axes="x",
        elem_a="sloped_blade",
        elem_b="cutting_strip",
        min_overlap=0.42,
        name="blade spans the bed cutting strip",
    )
    ctx.expect_overlap(
        finger_guard,
        bed,
        axes="x",
        elem_a="clear_panel",
        elem_b="cutting_strip",
        min_overlap=0.45,
        name="guard covers the cut-line length",
    )
    ctx.expect_gap(
        finger_guard,
        bed,
        axis="z",
        positive_elem="clear_panel",
        negative_elem="cutting_mat",
        min_gap=0.015,
        max_gap=0.030,
        name="clear guard hovers above the bed",
    )
    ctx.expect_gap(
        finger_guard,
        bed,
        axis="x",
        positive_elem="clip_loop_1",
        negative_elem="guard_hinge_knuckle_0",
        min_gap=0.0,
        max_gap=0.008,
        name="center guard clip is retained beside fixed hinge knuckle",
    )
    ctx.expect_gap(
        bed,
        finger_guard,
        axis="x",
        positive_elem="guard_hinge_knuckle_1",
        negative_elem="clip_loop_1",
        min_gap=0.0,
        max_gap=0.008,
        name="center guard clip is captured between hinge knuckles",
    )

    closed_grip = ctx.part_element_world_aabb(blade_arm, elem="front_grip")
    with ctx.pose({blade_joint: 1.15}):
        raised_grip = ctx.part_element_world_aabb(blade_arm, elem="front_grip")
    ctx.check(
        "blade arm opens upward before cutting downward",
        closed_grip is not None
        and raised_grip is not None
        and raised_grip[0][2] > closed_grip[0][2] + 0.30,
        details=f"closed={closed_grip}, raised={raised_grip}",
    )

    closed_panel = ctx.part_element_world_aabb(finger_guard, elem="clear_panel")
    with ctx.pose({guard_joint: 1.25}):
        raised_panel = ctx.part_element_world_aabb(finger_guard, elem="clear_panel")
    ctx.check(
        "finger guard flips up while remaining on its hinge",
        closed_panel is not None
        and raised_panel is not None
        and raised_panel[1][2] > closed_panel[1][2] + 0.12,
        details=f"closed={closed_panel}, raised={raised_panel}",
    )

    return ctx.report()


object_model = build_object_model()
