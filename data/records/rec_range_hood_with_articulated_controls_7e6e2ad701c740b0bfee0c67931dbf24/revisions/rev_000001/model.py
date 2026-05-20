from __future__ import annotations

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
    model = ArticulatedObject(name="telescopic_range_hood")

    painted_steel = model.material("painted_steel", rgba=(0.82, 0.83, 0.84, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    galvanized = model.material("galvanized", rgba=(0.66, 0.68, 0.69, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    filter_black = model.material("filter_black", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.07, 0.07, 0.08, 1.0))
    button_grey = model.material("button_grey", rgba=(0.62, 0.64, 0.66, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.60, 0.30, 0.012)),
        origin=Origin(xyz=(0.0, -0.15, 0.174)),
        material=galvanized,
        name="top_cover",
    )
    body.visual(
        Box((0.012, 0.30, 0.156)),
        origin=Origin(xyz=(-0.294, -0.15, 0.090)),
        material=galvanized,
        name="left_wall",
    )
    body.visual(
        Box((0.012, 0.30, 0.156)),
        origin=Origin(xyz=(0.294, -0.15, 0.090)),
        material=galvanized,
        name="right_wall",
    )
    body.visual(
        Box((0.576, 0.012, 0.156)),
        origin=Origin(xyz=(0.0, -0.294, 0.090)),
        material=galvanized,
        name="back_wall",
    )
    body.visual(
        Box((0.576, 0.278, 0.012)),
        origin=Origin(xyz=(0.0, -0.151, 0.044)),
        material=painted_steel,
        name="slot_roof",
    )
    body.visual(
        Box((0.576, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, -0.012, 0.048)),
        material=painted_steel,
        name="front_header",
    )
    body.visual(
        Box((0.004, 0.220, 0.012)),
        origin=Origin(xyz=(-0.286, -0.122, 0.030)),
        material=charcoal,
        name="runner_0",
    )
    body.visual(
        Box((0.004, 0.220, 0.012)),
        origin=Origin(xyz=(0.286, -0.122, 0.030)),
        material=charcoal,
        name="runner_1",
    )
    body.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.0, -0.225, 0.198)),
        material=galvanized,
        name="exhaust_collar",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.60, 0.018, 0.056)),
        origin=Origin(xyz=(0.0, 0.009, 0.028)),
        material=brushed_steel,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.57, 0.270, 0.012)),
        origin=Origin(xyz=(0.0, -0.133, 0.020)),
        material=painted_steel,
        name="drawer_tray",
    )
    drawer.visual(
        Box((0.008, 0.220, 0.026)),
        origin=Origin(xyz=(-0.277, -0.112, 0.021)),
        material=brushed_steel,
        name="side_flange_0",
    )
    drawer.visual(
        Box((0.008, 0.220, 0.026)),
        origin=Origin(xyz=(0.277, -0.112, 0.021)),
        material=brushed_steel,
        name="side_flange_1",
    )
    drawer.visual(
        Box((0.230, 0.180, 0.004)),
        origin=Origin(xyz=(-0.122, -0.145, 0.012)),
        material=filter_black,
        name="filter_0",
    )
    drawer.visual(
        Box((0.230, 0.180, 0.004)),
        origin=Origin(xyz=(0.122, -0.145, 0.012)),
        material=filter_black,
        name="filter_1",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.20,
            lower=0.0,
            upper=0.180,
        ),
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((0.120, 0.052, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=charcoal,
        name="top_shell",
    )
    control_pod.visual(
        Box((0.010, 0.052, 0.016)),
        origin=Origin(xyz=(-0.055, 0.0, -0.003)),
        material=charcoal,
        name="side_0",
    )
    control_pod.visual(
        Box((0.010, 0.052, 0.016)),
        origin=Origin(xyz=(0.055, 0.0, -0.003)),
        material=charcoal,
        name="side_1",
    )
    control_pod.visual(
        Box((0.120, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.022, -0.004)),
        material=charcoal,
        name="front_lip",
    )
    control_pod.visual(
        Box((0.120, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.022, -0.004)),
        material=charcoal,
        name="rear_lip",
    )
    control_pod.visual(
        Box((0.008, 0.036, 0.012)),
        origin=Origin(xyz=(0.002, 0.0, -0.005)),
        material=charcoal,
        name="center_bridge",
    )
    control_pod.visual(
        Box((0.004, 0.012, 0.012)),
        origin=Origin(xyz=(-0.044, -0.010, -0.005)),
        material=charcoal,
        name="rocker_bracket_0",
    )
    control_pod.visual(
        Box((0.004, 0.012, 0.012)),
        origin=Origin(xyz=(-0.004, -0.010, -0.005)),
        material=charcoal,
        name="rocker_bracket_1",
    )
    control_pod.visual(
        Box((0.012, 0.012, 0.008)),
        origin=Origin(xyz=(0.018, 0.004, 0.001)),
        material=charcoal,
        name="button_guide_0",
    )
    control_pod.visual(
        Box((0.012, 0.012, 0.008)),
        origin=Origin(xyz=(0.040, 0.004, 0.001)),
        material=charcoal,
        name="button_guide_1",
    )

    rocker = model.part("rocker")
    rocker.visual(
        Cylinder(radius=0.0025, length=0.036),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=satin_black,
        name="rocker_axle",
    )
    rocker.visual(
        Box((0.034, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.009, -0.002)),
        material=satin_black,
        name="rocker_cap",
    )

    button_0 = model.part("button_0")
    button_0.visual(
        Cylinder(radius=0.0055, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=button_grey,
        name="button_cap",
    )
    button_0.visual(
        Cylinder(radius=0.0025, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=button_grey,
        name="button_stem",
    )

    button_1 = model.part("button_1")
    button_1.visual(
        Cylinder(radius=0.0055, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=button_grey,
        name="button_cap",
    )
    button_1.visual(
        Cylinder(radius=0.0025, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=button_grey,
        name="button_stem",
    )

    model.articulation(
        "drawer_to_control_pod",
        ArticulationType.FIXED,
        parent=drawer,
        child=control_pod,
        origin=Origin(xyz=(0.190, -0.032, 0.005)),
    )
    model.articulation(
        "pod_to_rocker",
        ArticulationType.REVOLUTE,
        parent=control_pod,
        child=rocker,
        origin=Origin(xyz=(-0.024, -0.010, -0.011)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.18,
            upper=0.25,
        ),
    )
    model.articulation(
        "pod_to_button_0",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=button_0,
        origin=Origin(xyz=(0.018, 0.004, -0.011)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.003,
        ),
    )
    model.articulation(
        "pod_to_button_1",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=button_1,
        origin=Origin(xyz=(0.040, 0.004, -0.011)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.003,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    control_pod = object_model.get_part("control_pod")
    rocker = object_model.get_part("rocker")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    rocker_joint = object_model.get_articulation("pod_to_rocker")
    button_0_joint = object_model.get_articulation("pod_to_button_0")
    button_1_joint = object_model.get_articulation("pod_to_button_1")

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_gap(
            drawer,
            body,
            axis="y",
            positive_elem="drawer_front",
            negative_elem="front_header",
            max_gap=0.001,
            max_penetration=0.0,
            name="drawer front seats flush with hood header",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="x",
            inner_elem="drawer_tray",
            outer_elem="slot_roof",
            margin=0.004,
            name="drawer tray stays centered between the hood walls",
        )
        ctx.expect_gap(
            drawer,
            control_pod,
            axis="z",
            positive_elem="drawer_tray",
            negative_elem="top_shell",
            max_gap=0.001,
            max_penetration=0.00001,
            name="control pod mounts directly under the sliding tray",
        )

    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.180}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="drawer_tray",
            elem_b="slot_roof",
            min_overlap=0.070,
            name="drawer keeps retained insertion at full extension",
        )
        extended_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends outward",
        rest_pos is not None and extended_pos is not None and extended_pos[1] > rest_pos[1] + 0.15,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rocker_rest = ctx.part_element_world_aabb(rocker, elem="rocker_cap")
    with ctx.pose({rocker_joint: 0.22}):
        rocker_pressed = ctx.part_element_world_aabb(rocker, elem="rocker_cap")
    ctx.check(
        "rocker tips on its hinge",
        rocker_rest is not None
        and rocker_pressed is not None
        and rocker_pressed[0][2] < rocker_rest[0][2] - 0.002,
        details=f"rest={rocker_rest}, pressed={rocker_pressed}",
    )

    with ctx.pose({button_0_joint: 0.003}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_idle = ctx.part_world_position(button_1)
    ctx.check(
        "button_0 depresses independently",
        button_0_pressed is not None
        and button_1_idle is not None
        and button_0_pressed[2] > button_1_idle[2] + 0.002,
        details=f"button_0={button_0_pressed}, button_1={button_1_idle}",
    )

    with ctx.pose({button_1_joint: 0.003}):
        button_0_idle = ctx.part_world_position(button_0)
        button_1_pressed = ctx.part_world_position(button_1)
    ctx.check(
        "button_1 depresses independently",
        button_1_pressed is not None
        and button_0_idle is not None
        and button_1_pressed[2] > button_0_idle[2] + 0.002,
        details=f"button_0={button_0_idle}, button_1={button_1_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
