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
    model = ArticulatedObject(name="desktop_stapler")

    body_black = model.material("body_black", rgba=(0.11, 0.12, 0.13, 1.0))
    body_shell = model.material("body_shell", rgba=(0.19, 0.22, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.13, 0.13, 0.13, 1.0))
    trim = model.material("trim", rgba=(0.52, 0.54, 0.58, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.160, 0.044, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=rubber,
        name="sole",
    )
    base.visual(
        Box((0.142, 0.036, 0.011)),
        origin=Origin(xyz=(0.004, 0.000, 0.0095)),
        material=body_black,
        name="body",
    )
    base.visual(
        Box((0.042, 0.034, 0.010)),
        origin=Origin(xyz=(0.058, 0.000, 0.0150)),
        material=body_shell,
        name="nose",
    )
    base.visual(
        Box((0.022, 0.030, 0.013)),
        origin=Origin(xyz=(-0.068, 0.000, 0.0125)),
        material=body_shell,
        name="tail",
    )
    base.visual(
        Box((0.090, 0.020, 0.003)),
        origin=Origin(xyz=(0.014, 0.000, 0.0175)),
        material=steel,
        name="deck",
    )
    base.visual(
        Box((0.018, 0.006, 0.028)),
        origin=Origin(xyz=(-0.066, 0.011, 0.019)),
        material=body_shell,
        name="tower_0",
    )
    base.visual(
        Box((0.018, 0.006, 0.028)),
        origin=Origin(xyz=(-0.066, -0.011, 0.019)),
        material=body_shell,
        name="tower_1",
    )
    base.visual(
        Cylinder(radius=0.0036, length=0.006),
        origin=Origin(xyz=(0.064, 0.000, 0.0002)),
        material=trim,
        name="anvil_pivot",
    )

    magazine = model.part("magazine")
    magazine.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_sleeve",
    )
    magazine.visual(
        Box((0.010, 0.016, 0.008)),
        origin=Origin(xyz=(0.005, 0.000, -0.002)),
        material=steel,
        name="rear_block",
    )
    magazine.visual(
        Box((0.126, 0.014, 0.0025)),
        origin=Origin(xyz=(0.069, 0.000, -0.00725)),
        material=steel,
        name="track",
    )
    magazine.visual(
        Box((0.120, 0.0025, 0.010)),
        origin=Origin(xyz=(0.072, 0.00575, -0.001)),
        material=steel,
        name="rail_0",
    )
    magazine.visual(
        Box((0.120, 0.0025, 0.010)),
        origin=Origin(xyz=(0.072, -0.00575, -0.001)),
        material=steel,
        name="rail_1",
    )
    magazine.visual(
        Box((0.012, 0.004, 0.012)),
        origin=Origin(xyz=(0.006, 0.009, 0.001)),
        material=steel,
        name="arm_lug_0",
    )
    magazine.visual(
        Box((0.012, 0.004, 0.012)),
        origin=Origin(xyz=(0.006, -0.009, 0.001)),
        material=steel,
        name="arm_lug_1",
    )
    magazine.visual(
        Box((0.012, 0.014, 0.010)),
        origin=Origin(xyz=(0.132, 0.000, -0.001)),
        material=steel,
        name="nose_block",
    )
    magazine.visual(
        Box((0.016, 0.010, 0.004)),
        origin=Origin(xyz=(0.128, 0.000, -0.006)),
        material=trim,
        name="driver_nose",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.0033, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    arm.visual(
        Box((0.018, 0.028, 0.012)),
        origin=Origin(xyz=(0.006, 0.000, 0.005)),
        material=body_shell,
        name="rear_bridge",
    )
    arm.visual(
        Box((0.118, 0.030, 0.004)),
        origin=Origin(xyz=(0.071, 0.000, 0.013)),
        material=body_shell,
        name="cover",
    )
    arm.visual(
        Box((0.110, 0.003, 0.008)),
        origin=Origin(xyz=(0.075, 0.0135, 0.009)),
        material=body_black,
        name="skirt_0",
    )
    arm.visual(
        Box((0.110, 0.003, 0.008)),
        origin=Origin(xyz=(0.075, -0.0135, 0.009)),
        material=body_black,
        name="skirt_1",
    )
    arm.visual(
        Box((0.018, 0.026, 0.010)),
        origin=Origin(xyz=(0.128, 0.000, 0.011)),
        material=body_shell,
        name="front_nose",
    )
    arm.visual(
        Box((0.036, 0.022, 0.002)),
        origin=Origin(xyz=(0.096, 0.000, 0.016)),
        material=rubber,
        name="press_pad",
    )

    anvil = model.part("anvil")
    anvil.visual(
        Cylinder(radius=0.0032, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, -0.002)),
        material=steel,
        name="pivot",
    )
    anvil.visual(
        Box((0.015, 0.010, 0.003)),
        origin=Origin(xyz=(0.0065, 0.000, -0.0015)),
        material=steel,
        name="plate",
    )
    anvil.visual(
        Box((0.009, 0.004, 0.003)),
        origin=Origin(xyz=(-0.0045, 0.000, -0.0015)),
        material=trim,
        name="tab",
    )

    model.articulation(
        "base_to_magazine",
        ArticulationType.REVOLUTE,
        parent=base,
        child=magazine,
        origin=Origin(xyz=(-0.060, 0.000, 0.034)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "magazine_to_arm",
        ArticulationType.REVOLUTE,
        parent=magazine,
        child=arm,
        origin=Origin(xyz=(0.004, 0.000, 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.2,
            lower=0.0,
            upper=0.078,
        ),
    )
    model.articulation(
        "base_to_anvil",
        ArticulationType.REVOLUTE,
        parent=base,
        child=anvil,
        origin=Origin(xyz=(0.064, 0.000, 0.0000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return (
        (low[0] + high[0]) * 0.5,
        (low[1] + high[1]) * 0.5,
        (low[2] + high[2]) * 0.5,
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    magazine = object_model.get_part("magazine")
    arm = object_model.get_part("arm")
    anvil = object_model.get_part("anvil")

    magazine_hinge = object_model.get_articulation("base_to_magazine")
    arm_hinge = object_model.get_articulation("magazine_to_arm")
    anvil_hinge = object_model.get_articulation("base_to_anvil")

    magazine_limits = magazine_hinge.motion_limits
    arm_limits = arm_hinge.motion_limits
    anvil_limits = anvil_hinge.motion_limits

    with ctx.pose({magazine_hinge: 0.0, arm_hinge: 0.0}):
        ctx.expect_gap(
            magazine,
            base,
            axis="z",
            positive_elem="track",
            negative_elem="deck",
            min_gap=0.004,
            max_gap=0.012,
            name="magazine sits just above the base deck",
        )
        ctx.expect_gap(
            arm,
            magazine,
            axis="z",
            positive_elem="front_nose",
            negative_elem="nose_block",
            min_gap=0.008,
            max_gap=0.020,
            name="arm hovers above the magazine nose at rest",
        )
        ctx.expect_overlap(
            arm,
            magazine,
            axes="x",
            elem_a="cover",
            elem_b="track",
            min_overlap=0.090,
            name="arm remains carried over the magazine length",
        )

    rest_mag_pos = _aabb_center(ctx.part_element_world_aabb(magazine, elem="nose_block"))
    if magazine_limits is not None and magazine_limits.upper is not None:
        with ctx.pose({magazine_hinge: magazine_limits.upper, arm_hinge: 0.0}):
            open_mag_pos = _aabb_center(ctx.part_element_world_aabb(magazine, elem="nose_block"))
        ctx.check(
            "magazine opens upward for reloading",
            rest_mag_pos is not None
            and open_mag_pos is not None
            and open_mag_pos[2] > rest_mag_pos[2] + 0.05
            and open_mag_pos[0] < rest_mag_pos[0] - 0.04,
            details=f"rest={rest_mag_pos}, open={open_mag_pos}",
        )

    if arm_limits is not None and arm_limits.upper is not None:
        rest_arm_nose = _aabb_center(ctx.part_element_world_aabb(arm, elem="front_nose"))
        with ctx.pose({arm_hinge: arm_limits.upper}):
            pressed_arm_nose = _aabb_center(ctx.part_element_world_aabb(arm, elem="front_nose"))
            ctx.expect_gap(
                arm,
                magazine,
                axis="z",
                positive_elem="front_nose",
                negative_elem="nose_block",
                min_gap=0.0005,
                max_gap=0.012,
                name="arm can depress without cutting through the magazine",
            )
        ctx.check(
            "arm rotates downward at the nose when pressed",
            rest_arm_nose is not None
            and pressed_arm_nose is not None
            and pressed_arm_nose[2] < rest_arm_nose[2] - 0.004,
            details=f"rest={rest_arm_nose}, pressed={pressed_arm_nose}",
        )

    if anvil_limits is not None and anvil_limits.upper is not None:
        rest_tab = _aabb_center(ctx.part_element_world_aabb(anvil, elem="tab"))
        with ctx.pose({anvil_hinge: anvil_limits.upper}):
            turned_tab = _aabb_center(ctx.part_element_world_aabb(anvil, elem="tab"))
        ctx.check(
            "anvil rotates under the stapler nose",
            rest_tab is not None
            and turned_tab is not None
            and abs(turned_tab[0] - rest_tab[0]) > 0.008,
            details=f"rest={rest_tab}, turned={turned_tab}",
        )
        ctx.expect_gap(
            base,
            anvil,
            axis="z",
            positive_elem="sole",
            negative_elem="plate",
            min_gap=0.0,
            max_gap=0.004,
            name="anvil remains just under the base sole",
        )

    return ctx.report()


object_model = build_object_model()
