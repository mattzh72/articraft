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

BODY_DEPTH = 0.39
BODY_WIDTH = 0.405
BODY_HEIGHT = 0.33
BODY_FRONT_X = BODY_DEPTH * 0.5

CHAMBER_DEPTH = 0.335
CHAMBER_WIDTH = 0.35
CHAMBER_HEIGHT = 0.158
CHAMBER_BOTTOM_Z = 0.048
CHAMBER_FRONT_X = 0.165

DRAWER_DEPTH = 0.33
DRAWER_WIDTH = 0.334
DRAWER_HEIGHT = 0.118
DRAWER_FRONT_THICKNESS = 0.03
DRAWER_FRONT_WIDTH = 0.356
DRAWER_FRONT_HEIGHT = 0.152
DRAWER_BOTTOM_Z = 0.052

BUTTON_Y = (-0.126, -0.084, -0.042, 0.0, 0.042)
BUTTON_Z = 0.258
KNOB_Y = 0.124
KNOB_Z = 0.257
CONTROL_PANEL_FACE_X = 0.185
BUTTON_PANEL_X = 0.189


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_air_fryer")

    housing_mat = model.material("housing_graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    drawer_mat = model.material("drawer_black", rgba=(0.10, 0.11, 0.12, 1.0))
    basket_mat = model.material("basket_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    control_mat = model.material("control_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=housing_mat,
        name="base_plinth",
    )
    body.visual(
        Box((0.35, 0.02, 0.245)),
        origin=Origin(xyz=(-0.01, -0.1925, 0.1425)),
        material=housing_mat,
        name="left_wall",
    )
    body.visual(
        Box((0.35, 0.02, 0.245)),
        origin=Origin(xyz=(-0.01, 0.1925, 0.1425)),
        material=housing_mat,
        name="right_wall",
    )
    body.visual(
        Box((0.02, 0.365, 0.245)),
        origin=Origin(xyz=(-0.185, 0.0, 0.1425)),
        material=housing_mat,
        name="rear_wall",
    )
    body.visual(
        Box((0.35, 0.365, 0.02)),
        origin=Origin(xyz=(-0.01, 0.0, 0.32)),
        material=housing_mat,
        name="roof",
    )
    body.visual(
        Box((0.068, 0.365, 0.11)),
        origin=Origin(xyz=(0.149, 0.0, 0.255)),
        material=housing_mat,
        name="upper_housing",
    )
    body.visual(
        Box((0.01, 0.32, 0.075)),
        origin=Origin(xyz=(0.18, 0.0, 0.255)),
        material=drawer_mat,
        name="control_panel_face",
    )
    body.visual(
        Box((0.005, 0.35, 0.02)),
        origin=Origin(xyz=(0.1625, 0.0, 0.2)),
        material=housing_mat,
        name="opening_header",
    )
    body.visual(
        Box((0.005, 0.35, 0.02)),
        origin=Origin(xyz=(0.1625, 0.0, 0.038)),
        material=housing_mat,
        name="opening_sill",
    )
    body.visual(
        Box((0.005, 0.03, 0.142)),
        origin=Origin(xyz=(0.1625, -0.16, 0.119)),
        material=housing_mat,
        name="left_stile",
    )
    body.visual(
        Box((0.005, 0.03, 0.142)),
        origin=Origin(xyz=(0.1625, 0.16, 0.119)),
        material=housing_mat,
        name="right_stile",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_DEPTH, 0.326, 0.004)),
        origin=Origin(xyz=(-(DRAWER_DEPTH * 0.5), 0.0, 0.002)),
        material=drawer_mat,
        name="drawer_floor",
    )
    drawer.visual(
        Box((DRAWER_DEPTH, 0.004, DRAWER_HEIGHT)),
        origin=Origin(xyz=(-(DRAWER_DEPTH * 0.5), -0.165, DRAWER_HEIGHT * 0.5)),
        material=drawer_mat,
        name="left_drawer_wall",
    )
    drawer.visual(
        Box((DRAWER_DEPTH, 0.004, DRAWER_HEIGHT)),
        origin=Origin(xyz=(-(DRAWER_DEPTH * 0.5), 0.165, DRAWER_HEIGHT * 0.5)),
        material=drawer_mat,
        name="right_drawer_wall",
    )
    drawer.visual(
        Box((0.004, 0.326, DRAWER_HEIGHT)),
        origin=Origin(xyz=(-0.328, 0.0, DRAWER_HEIGHT * 0.5)),
        material=drawer_mat,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.004, 0.326, DRAWER_HEIGHT)),
        origin=Origin(xyz=(-0.002, 0.0, DRAWER_HEIGHT * 0.5)),
        material=drawer_mat,
        name="drawer_front_wall",
    )
    drawer.visual(
        Box((DRAWER_FRONT_THICKNESS, DRAWER_FRONT_WIDTH, DRAWER_FRONT_HEIGHT)),
        origin=Origin(xyz=(DRAWER_FRONT_THICKNESS * 0.5, 0.0, DRAWER_FRONT_HEIGHT * 0.5)),
        material=drawer_mat,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.028, 0.18, 0.028)),
        origin=Origin(xyz=(0.043, 0.0, 0.084)),
        material=control_mat,
        name="handle_base",
    )
    drawer.visual(
        Box((0.028, 0.154, 0.02)),
        origin=Origin(xyz=(0.071, 0.0, 0.069)),
        material=control_mat,
        name="handle_grip",
    )
    drawer.visual(
        Box((0.022, 0.032, 0.004)),
        origin=Origin(xyz=(0.04, 0.0, 0.095)),
        material=control_mat,
        name="latch_floor",
    )
    drawer.visual(
        Box((0.03, 0.004, 0.018)),
        origin=Origin(xyz=(0.046, -0.016, 0.103)),
        material=control_mat,
        name="left_latch_guide",
    )
    drawer.visual(
        Box((0.03, 0.004, 0.018)),
        origin=Origin(xyz=(0.046, 0.016, 0.103)),
        material=control_mat,
        name="right_latch_guide",
    )

    basket = model.part("basket")
    basket.visual(
        Box((0.288, 0.306, 0.003)),
        origin=Origin(xyz=(-0.144, 0.0, 0.0015)),
        material=basket_mat,
        name="basket_floor",
    )
    basket.visual(
        Box((0.288, 0.003, 0.094)),
        origin=Origin(xyz=(-0.144, -0.1515, 0.047)),
        material=basket_mat,
        name="left_basket_wall",
    )
    basket.visual(
        Box((0.288, 0.003, 0.094)),
        origin=Origin(xyz=(-0.144, 0.1515, 0.047)),
        material=basket_mat,
        name="right_basket_wall",
    )
    basket.visual(
        Box((0.003, 0.306, 0.094)),
        origin=Origin(xyz=(-0.2865, 0.0, 0.047)),
        material=basket_mat,
        name="basket_back",
    )
    basket.visual(
        Box((0.003, 0.306, 0.08)),
        origin=Origin(xyz=(-0.0015, 0.0, 0.04)),
        material=basket_mat,
        name="basket_front",
    )
    basket.visual(
        Box((0.24, 0.003, 0.074)),
        origin=Origin(xyz=(-0.14, 0.0, 0.039)),
        material=basket_mat,
        name="divider_plate",
    )
    for x in (-0.24, -0.08):
        for y in (-0.12, 0.12):
            basket.visual(
                Box((0.02, 0.02, 0.008)),
                origin=Origin(xyz=(x, y, -0.004)),
                material=basket_mat,
                name=f"basket_foot_{int((x + 0.24) * 1000)}_{int((y + 0.12) * 1000)}",
            )

    basket.visual(
        Box((0.18, 0.014, 0.004)),
        origin=Origin(xyz=(-0.16, -0.09, 0.005)),
        material=basket_mat,
        name="left_slot_bridge",
    )
    basket.visual(
        Box((0.18, 0.014, 0.004)),
        origin=Origin(xyz=(-0.16, 0.09, 0.005)),
        material=basket_mat,
        name="right_slot_bridge",
    )

    latch = model.part("latch")
    latch.visual(
        Box((0.046, 0.028, 0.01)),
        origin=Origin(xyz=(0.023, 0.0, 0.005)),
        material=control_mat,
        name="latch_cap",
    )
    latch.visual(
        Box((0.02, 0.012, 0.004)),
        origin=Origin(xyz=(0.01, 0.0, -0.002)),
        material=control_mat,
        name="latch_foot",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=control_mat,
        name="knob_skirt",
    )
    knob.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=control_mat,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=control_mat,
        name="knob_nose",
    )
    knob.visual(
        Box((0.003, 0.004, 0.014)),
        origin=Origin(xyz=(0.021, 0.0, 0.013)),
        material=housing_mat,
        name="knob_indicator",
    )

    button_parts = []
    for index in range(5):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.012, 0.032, 0.016)),
            origin=Origin(xyz=(0.006, 0.0, 0.0)),
            material=control_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.004, 0.014, 0.01)),
            origin=Origin(xyz=(-0.002, 0.0, 0.0)),
            material=control_mat,
            name="button_stem",
        )
        button_parts.append(button)

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(CHAMBER_FRONT_X, 0.0, DRAWER_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.28, lower=0.0, upper=0.145),
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(xyz=(-0.023, 0.0, 0.012)),
    )
    model.articulation(
        "drawer_to_latch",
        ArticulationType.PRISMATIC,
        parent=drawer,
        child=latch,
        origin=Origin(xyz=(0.03, 0.0, 0.097)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=0.0, upper=0.012),
    )
    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(CONTROL_PANEL_FACE_X, KNOB_Y, KNOB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )

    for index, (button, y) in enumerate(zip(button_parts, BUTTON_Y)):
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(BUTTON_PANEL_X, y, BUTTON_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.0045),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    latch = object_model.get_part("latch")
    knob_joint = object_model.get_articulation("body_to_knob")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    latch_joint = object_model.get_articulation("drawer_to_latch")

    drawer_limits = drawer_joint.motion_limits
    latch_limits = latch_joint.motion_limits
    knob_limits = knob_joint.motion_limits

    ctx.check(
        "drawer_joint_is_prismatic",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={drawer_joint.articulation_type!r}",
    )
    ctx.check(
        "knob_joint_is_continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_limits is not None
        and knob_limits.lower is None
        and knob_limits.upper is None,
        details=f"type={knob_joint.articulation_type!r}, limits={knob_limits!r}",
    )
    ctx.expect_within(
        basket,
        drawer,
        axes="yz",
        margin=0.002,
        name="basket stays nested within drawer width and height",
    )

    drawer_aabb = ctx.part_world_aabb(drawer)
    basket_aabb = ctx.part_world_aabb(basket)
    ctx.check(
        "basket_sits_behind_handle_front",
        drawer_aabb is not None
        and basket_aabb is not None
        and basket_aabb[1][0] < drawer_aabb[1][0] - 0.05,
        details=f"drawer_aabb={drawer_aabb!r}, basket_aabb={basket_aabb!r}",
    )

    if drawer_limits is not None and drawer_limits.upper is not None:
        rest_pos = ctx.part_world_position(drawer)
        body_aabb = ctx.part_world_aabb(body)
        with ctx.pose({drawer_joint: drawer_limits.upper}):
            extended_pos = ctx.part_world_position(drawer)
            extended_aabb = ctx.part_world_aabb(drawer)
        overlap_x = None
        if body_aabb is not None and extended_aabb is not None:
            overlap_x = min(body_aabb[1][0], extended_aabb[1][0]) - max(body_aabb[0][0], extended_aabb[0][0])
        ctx.check(
            "drawer_extends_forward",
            rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.12,
            details=f"rest={rest_pos!r}, extended={extended_pos!r}",
        )
        ctx.check(
            "drawer_remains_retained_when_extended",
            overlap_x is not None and overlap_x > 0.08,
            details=f"body_aabb={body_aabb!r}, extended_aabb={extended_aabb!r}, overlap_x={overlap_x!r}",
        )

    if latch_limits is not None and latch_limits.upper is not None:
        latch_rest = ctx.part_world_position(latch)
        with ctx.pose({latch_joint: latch_limits.upper}):
            latch_raised = ctx.part_world_position(latch)
        ctx.check(
            "latch_lifts_upward",
            latch_rest is not None and latch_raised is not None and latch_raised[2] > latch_rest[2] + 0.008,
            details=f"rest={latch_rest!r}, raised={latch_raised!r}",
        )

    for index in range(5):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        button_limits = button_joint.motion_limits
        ctx.check(
            f"button_{index}_joint_is_prismatic",
            button_joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={button_joint.articulation_type!r}",
        )
        if button_limits is None or button_limits.upper is None:
            continue
        button_rest = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_limits.upper}):
            button_pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{index}_presses_inward",
            button_rest is not None and button_pressed is not None and button_pressed[0] < button_rest[0] - 0.002,
            details=f"rest={button_rest!r}, pressed={button_pressed!r}",
        )

    return ctx.report()


object_model = build_object_model()
