from __future__ import annotations

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
    model = ArticulatedObject(name="shuttle_hinge_chain")

    dark_steel = model.material("dark_steel", rgba=(0.14, 0.15, 0.16, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.66, 0.68, 0.68, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.05, 0.23, 0.55, 1.0))
    orange_arm = model.material("orange_arm", rgba=(0.95, 0.46, 0.12, 1.0))
    tab_black = model.material("tab_black", rgba=(0.03, 0.03, 0.035, 1.0))

    slide_body = model.part("slide_body")
    slide_body.visual(
        Box((0.56, 0.20, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_steel,
        name="base_plate",
    )
    for rail_name, rail_y in (("guide_rail_0", -0.068), ("guide_rail_1", 0.068)):
        slide_body.visual(
            Box((0.50, 0.022, 0.035)),
            origin=Origin(xyz=(0.0, rail_y, 0.0425)),
            material=bright_steel,
            name=rail_name,
        )
    for stop_index, stop_x in enumerate((-0.255, 0.255)):
        slide_body.visual(
            Box((0.030, 0.20, 0.060)),
            origin=Origin(xyz=(stop_x, 0.0, 0.055)),
            material=dark_steel,
            name=f"end_stop_{stop_index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.13, 0.19, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=carriage_blue,
        name="saddle_plate",
    )
    for skirt_index, skirt_y in enumerate((-0.096, 0.096)):
        carriage.visual(
            Box((0.12, 0.012, 0.034)),
            origin=Origin(xyz=(0.0, skirt_y, -0.017)),
            material=carriage_blue,
            name=f"side_skirt_{skirt_index}",
        )
    for lug_index, lug_y in enumerate((-0.043, 0.043)):
        carriage.visual(
            Box((0.026, 0.014, 0.060)),
            origin=Origin(xyz=(0.045, lug_y, 0.048)),
            material=carriage_blue,
            name=f"hinge_cheek_{lug_index}",
        )
        carriage.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(0.045, lug_y, 0.052), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=bright_steel,
            name=f"hinge_knuckle_{lug_index}",
        )
    carriage.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(0.045, 0.0, 0.052), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="hinge_pin",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="hinge_barrel",
    )
    arm.visual(
        Box((0.175, 0.030, 0.016)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=orange_arm,
        name="arm_link",
    )
    arm.visual(
        Box((0.050, 0.064, 0.010)),
        origin=Origin(xyz=(0.203, 0.0, 0.0)),
        material=tab_black,
        name="output_tab",
    )

    model.articulation(
        "slide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=slide_body,
        child=carriage,
        origin=Origin(xyz=(-0.12, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.24),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.045, 0.0, 0.052)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slide_body = object_model.get_part("slide_body")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("arm")
    slide_joint = object_model.get_articulation("slide_to_carriage")
    hinge_joint = object_model.get_articulation("carriage_to_arm")

    ctx.allow_overlap(
        carriage,
        arm,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The carried hinge uses a captured pin represented inside the arm barrel.",
    )

    ctx.check(
        "carriage uses prismatic slide",
        slide_joint.articulation_type == ArticulationType.PRISMATIC,
        details=str(slide_joint.articulation_type),
    )
    ctx.check(
        "arm uses carried revolute hinge",
        hinge_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(hinge_joint.articulation_type),
    )
    ctx.expect_gap(
        carriage,
        slide_body,
        axis="z",
        positive_elem="saddle_plate",
        negative_elem="guide_rail_0",
        max_gap=0.0008,
        max_penetration=0.0005,
        name="carriage bearing face sits on the guide rail",
    )
    ctx.expect_within(
        carriage,
        slide_body,
        axes="y",
        inner_elem="saddle_plate",
        outer_elem="base_plate",
        margin=0.001,
        name="carriage stays centered across the slide body",
    )
    ctx.expect_overlap(
        carriage,
        slide_body,
        axes="x",
        elem_a="saddle_plate",
        elem_b="guide_rail_0",
        min_overlap=0.10,
        name="collapsed carriage remains supported by the rail",
    )
    ctx.expect_within(
        carriage,
        arm,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.002,
        name="hinge pin is centered in the arm barrel",
    )
    ctx.expect_overlap(
        carriage,
        arm,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.055,
        name="hinge pin passes through the arm barrel",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide_joint: 0.24}):
        ctx.expect_overlap(
            carriage,
            slide_body,
            axes="x",
            elem_a="saddle_plate",
            elem_b="guide_rail_0",
            min_overlap=0.10,
            name="extended carriage remains supported by the rail",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage extends along positive x",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return 0.5 * (lower[2] + upper[2])

    tab_rest_z = _aabb_center_z(ctx.part_element_world_aabb(arm, elem="output_tab"))
    with ctx.pose({hinge_joint: 1.0}):
        tab_raised_z = _aabb_center_z(ctx.part_element_world_aabb(arm, elem="output_tab"))

    ctx.check(
        "hinged arm raises the output tab",
        tab_rest_z is not None and tab_raised_z is not None and tab_raised_z > tab_rest_z + 0.12,
        details=f"rest_z={tab_rest_z}, raised_z={tab_raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
