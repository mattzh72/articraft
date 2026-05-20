from __future__ import annotations

import math

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

BASE_LENGTH = 0.430
BASE_WIDTH = 0.062
BASE_THICKNESS = 0.010
HINGE_HEIGHT = 0.072
ARM_REST_PITCH = 0.10
ARM_OPEN_ANGLE = 1.02


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_desktop_stapler")

    body_black = model.material("body_black", rgba=(0.14, 0.14, 0.15, 1.0))
    shell_black = model.material("shell_black", rgba=(0.19, 0.20, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.45, 0.47, 0.50, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(BASE_LENGTH * 0.5, 0.0, -HINGE_HEIGHT + BASE_THICKNESS * 0.5)),
        material=body_black,
        name="base_plate",
    )
    base.visual(
        Box((0.340, 0.034, 0.006)),
        origin=Origin(xyz=(0.220, 0.0, -HINGE_HEIGHT + 0.011)),
        material=shell_black,
        name="reach_beam",
    )
    base.visual(
        Box((0.060, 0.038, 0.018)),
        origin=Origin(xyz=(0.396, 0.0, -0.060)),
        material=body_black,
        name="nose_body",
    )
    base.visual(
        Box((0.046, 0.024, 0.008)),
        origin=Origin(xyz=(0.402, 0.0, -0.049)),
        material=dark_steel,
        name="jaw_block",
    )
    base.visual(
        Box((0.032, 0.020, 0.003)),
        origin=Origin(xyz=(0.388, 0.0, -0.0505)),
        material=steel,
        name="anvil_seat",
    )
    base.visual(
        Box((0.040, 0.032, 0.024)),
        origin=Origin(xyz=(0.014, 0.0, -0.058)),
        material=body_black,
        name="rear_bridge",
    )
    for index, side in enumerate((-1.0, 1.0)):
        base.visual(
            Box((0.032, 0.008, HINGE_HEIGHT)),
            origin=Origin(xyz=(0.010, side * 0.026, -HINGE_HEIGHT * 0.5)),
            material=body_black,
            name=f"tower_cheek_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, HINGE_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(BASE_LENGTH * 0.5, 0.0, -HINGE_HEIGHT * 0.5)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.009, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    upper_arm.visual(
        Box((0.032, 0.040, 0.012)),
        origin=Origin(xyz=(0.016, 0.0, -0.010)),
        material=dark_steel,
        name="rear_knuckle",
    )
    for index, side in enumerate((-1.0, 1.0)):
        upper_arm.visual(
            Box((0.326, 0.014, 0.008)),
            origin=Origin(xyz=(0.193, side * 0.015, -0.009)),
            material=shell_black,
            name=f"top_rail_{index}",
        )
        upper_arm.visual(
            Box((0.346, 0.003, 0.025)),
            origin=Origin(xyz=(0.203, side * 0.0235, -0.0215)),
            material=shell_black,
            name=f"side_wall_{index}",
        )
    upper_arm.visual(
        Box((0.054, 0.050, 0.020)),
        origin=Origin(xyz=(0.373, 0.0, 0.005)),
        material=shell_black,
        name="nose_shell",
    )
    upper_arm.visual(
        Box((0.072, 0.020, 0.004)),
        origin=Origin(xyz=(0.332, 0.0, -0.006)),
        material=steel,
        name="top_cap",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.390, 0.050, 0.040)),
        mass=0.9,
        origin=Origin(xyz=(0.195, 0.0, -0.018)),
    )

    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(rpy=(0.0, ARM_REST_PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.4,
            lower=0.0,
            upper=ARM_OPEN_ANGLE,
        ),
    )

    staple_tray = model.part("staple_tray")
    staple_tray.visual(
        Box((0.316, 0.026, 0.002)),
        origin=Origin(xyz=(0.158, 0.0, 0.0)),
        material=steel,
        name="tray_floor",
    )
    for index, side in enumerate((-1.0, 1.0)):
        staple_tray.visual(
            Box((0.312, 0.0025, 0.010)),
            origin=Origin(xyz=(0.158, side * 0.0118, 0.006)),
            material=steel,
            name=f"tray_wall_{index}",
        )
    staple_tray.visual(
        Box((0.010, 0.026, 0.010)),
        origin=Origin(xyz=(0.311, 0.0, 0.006)),
        material=steel,
        name="tray_stop",
    )
    staple_tray.visual(
        Box((0.018, 0.032, 0.012)),
        origin=Origin(xyz=(-0.003, 0.0, 0.006)),
        material=dark_steel,
        name="tray_handle",
    )
    staple_tray.inertial = Inertial.from_geometry(
        Box((0.334, 0.032, 0.014)),
        mass=0.18,
        origin=Origin(xyz=(0.150, 0.0, 0.006)),
    )

    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=upper_arm,
        child=staple_tray,
        origin=Origin(xyz=(0.030, 0.0, -0.026)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.18,
            lower=0.0,
            upper=0.110,
        ),
    )

    follower_tab = model.part("follower_tab")
    follower_tab.visual(
        Box((0.018, 0.0132, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=dark_steel,
        name="shoe",
    )
    for index, side in enumerate((-1.0, 1.0)):
        follower_tab.visual(
            Box((0.018, 0.004, 0.006)),
            origin=Origin(xyz=(0.0, side * 0.00855, 0.005)),
            material=dark_steel,
            name=f"guide_{index}",
        )
    follower_tab.visual(
        Box((0.008, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=shell_black,
        name="tab",
    )
    follower_tab.inertial = Inertial.from_geometry(
        Box((0.020, 0.018, 0.028)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    model.articulation(
        "follower_slide",
        ArticulationType.PRISMATIC,
        parent=staple_tray,
        child=follower_tab,
        origin=Origin(xyz=(0.248, 0.0, 0.001)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.12,
            lower=0.0,
            upper=0.180,
        ),
    )

    anvil = model.part("anvil")
    anvil.visual(
        Box((0.024, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=steel,
        name="plate",
    )
    anvil.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_steel,
        name="pivot",
    )
    anvil.inertial = Inertial.from_geometry(
        Box((0.024, 0.014, 0.006)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    model.articulation(
        "anvil_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=anvil,
        origin=Origin(xyz=(0.388, 0.0, -0.049)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    staple_tray = object_model.get_part("staple_tray")
    follower_tab = object_model.get_part("follower_tab")
    anvil = object_model.get_part("anvil")
    arm_hinge = object_model.get_articulation("arm_hinge")
    tray_slide = object_model.get_articulation("tray_slide")
    follower_slide = object_model.get_articulation("follower_slide")
    anvil_pivot = object_model.get_articulation("anvil_pivot")

    with ctx.pose({arm_hinge: 0.0}):
        ctx.expect_gap(
            upper_arm,
            base,
            axis="z",
            positive_elem="nose_shell",
            negative_elem="jaw_block",
            min_gap=0.0,
            max_gap=0.010,
            name="closed nose sits just above the lower jaw",
        )
        ctx.expect_overlap(
            upper_arm,
            base,
            axes="xy",
            elem_a="nose_shell",
            elem_b="nose_body",
            min_overlap=0.020,
            name="upper nose stays aligned over the front nose housing",
        )
        ctx.expect_within(
            staple_tray,
            upper_arm,
            axes="yz",
            margin=0.0015,
            name="inserted staple tray stays centered inside the upper rail",
        )
        ctx.expect_overlap(
            staple_tray,
            upper_arm,
            axes="x",
            min_overlap=0.260,
            name="inserted tray remains deeply seated in the upper rail",
        )
        ctx.expect_within(
            follower_tab,
            staple_tray,
            axes="yz",
            margin=0.001,
            inner_elem="shoe",
            name="follower stays within the tray channel",
        )
        ctx.expect_overlap(
            follower_tab,
            staple_tray,
            axes="x",
            min_overlap=0.015,
            elem_a="shoe",
            name="follower remains captured in the tray channel",
        )
        ctx.expect_overlap(
            anvil,
            base,
            axes="xy",
            elem_a="plate",
            elem_b="anvil_seat",
            min_overlap=0.010,
            name="anvil plate sits on the front seat beneath the nose",
        )
        ctx.expect_gap(
            upper_arm,
            anvil,
            axis="z",
            positive_elem="nose_shell",
            negative_elem="plate",
            min_gap=0.001,
            max_gap=0.010,
            name="closed nose leaves a realistic gap above the anvil plate",
        )

    closed_nose_aabb = ctx.part_element_world_aabb(upper_arm, elem="nose_shell")
    tray_rest = ctx.part_world_position(staple_tray)
    follower_rest = ctx.part_world_position(follower_tab)
    with ctx.pose({arm_hinge: ARM_OPEN_ANGLE}):
        open_nose_aabb = ctx.part_element_world_aabb(upper_arm, elem="nose_shell")
        ctx.expect_gap(
            upper_arm,
            base,
            axis="z",
            positive_elem="nose_shell",
            negative_elem="nose_body",
            min_gap=0.080,
            name="opened arm lifts clear of the base",
        )

    ctx.check(
        "upper arm rises when opened",
        closed_nose_aabb is not None
        and open_nose_aabb is not None
        and open_nose_aabb[0][2] > closed_nose_aabb[0][2] + 0.05,
        details=f"closed={closed_nose_aabb}, open={open_nose_aabb}",
    )

    tray_limits = tray_slide.motion_limits
    if tray_limits is not None and tray_limits.upper is not None:
        with ctx.pose({tray_slide: tray_limits.upper}):
            tray_extended = ctx.part_world_position(staple_tray)
            ctx.expect_within(
                staple_tray,
                upper_arm,
                axes="yz",
                margin=0.0015,
                name="extended staple tray stays guided by the upper rail",
            )
            ctx.expect_overlap(
                staple_tray,
                upper_arm,
                axes="x",
                min_overlap=0.140,
                name="extended tray still retains insertion in the upper rail",
            )
        ctx.check(
            "tray slides rearward for loading",
            tray_rest is not None and tray_extended is not None and tray_extended[0] < tray_rest[0] - 0.05,
            details=f"rest={tray_rest}, extended={tray_extended}",
        )

    follower_limits = follower_slide.motion_limits
    if follower_limits is not None and follower_limits.upper is not None:
        with ctx.pose({follower_slide: follower_limits.upper}):
            follower_retracted = ctx.part_world_position(follower_tab)
            ctx.expect_within(
                follower_tab,
                staple_tray,
                axes="yz",
                margin=0.001,
                inner_elem="shoe",
                name="retracted follower stays inside the tray channel",
            )
            ctx.expect_overlap(
                follower_tab,
                staple_tray,
                axes="x",
                min_overlap=0.015,
                elem_a="shoe",
                name="retracted follower remains captured by the tray channel",
            )
        ctx.check(
            "follower retracts toward the rear",
            follower_rest is not None
            and follower_retracted is not None
            and follower_retracted[0] < follower_rest[0] - 0.08,
            details=f"rest={follower_rest}, retracted={follower_retracted}",
        )

    anvil_flat = ctx.part_world_aabb(anvil)
    with ctx.pose({anvil_pivot: math.pi * 0.5}):
        anvil_rotated = ctx.part_world_aabb(anvil)
        ctx.expect_overlap(
            anvil,
            base,
            axes="xy",
            elem_a="plate",
            elem_b="anvil_seat",
            min_overlap=0.010,
            name="rotated anvil plate stays on its seat",
        )
    ctx.check(
        "anvil plate rotates in plane",
        anvil_flat is not None
        and anvil_rotated is not None
        and abs((anvil_flat[1][0] - anvil_flat[0][0]) - (anvil_rotated[1][1] - anvil_rotated[0][1])) < 0.004
        and abs((anvil_flat[1][1] - anvil_flat[0][1]) - (anvil_rotated[1][0] - anvil_rotated[0][0])) < 0.004,
        details=f"flat={anvil_flat}, rotated={anvil_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
