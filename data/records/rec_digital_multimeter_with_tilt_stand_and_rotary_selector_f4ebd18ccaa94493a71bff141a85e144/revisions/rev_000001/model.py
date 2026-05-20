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


BODY_WIDTH = 0.074
BODY_DEPTH = 0.038
BODY_HEIGHT = 0.166

BUMPER_WIDTH = 0.086
SIDE_RAIL_WIDTH = 0.006
LIP_DEPTH = 0.004
FRONT_PANEL_DEPTH = 0.002

DISPLAY_WIDTH = 0.050
DISPLAY_HEIGHT = 0.030
DISPLAY_Z = 0.130

KEY_WIDTH = 0.011
KEY_DEPTH = 0.003
KEY_HEIGHT = 0.006
KEY_Z = 0.102
KEY_XS = (-0.026, -0.013, 0.0, 0.013, 0.026)

SELECTOR_RADIUS = 0.019
SELECTOR_Z = 0.069
SELECTOR_FRONT_Y = 0.002

JACK_PANEL_Z = 0.018
STAND_HINGE_Z = 0.014


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_multimeter")

    housing = model.material("housing", rgba=(0.18, 0.19, 0.20, 1.0))
    bumper = model.material("bumper", rgba=(0.90, 0.69, 0.18, 1.0))
    glass = model.material("glass", rgba=(0.22, 0.39, 0.44, 0.60))
    trim = model.material("trim", rgba=(0.10, 0.10, 0.11, 1.0))
    control = model.material("control", rgba=(0.26, 0.27, 0.29, 1.0))
    key_cap = model.material("key_cap", rgba=(0.52, 0.55, 0.58, 1.0))
    label_grey = model.material("label_grey", rgba=(0.72, 0.74, 0.76, 1.0))
    jack_red = model.material("jack_red", rgba=(0.76, 0.12, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
        material=housing,
        name="housing_shell",
    )
    body.visual(
        Box((SIDE_RAIL_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=((BUMPER_WIDTH - SIDE_RAIL_WIDTH) / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=bumper,
        name="side_rail_1",
    )
    body.visual(
        Box((SIDE_RAIL_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-(BUMPER_WIDTH - SIDE_RAIL_WIDTH) / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=bumper,
        name="side_rail_0",
    )
    body.visual(
        Box((BUMPER_WIDTH, LIP_DEPTH, 0.012)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + LIP_DEPTH / 2.0, BODY_HEIGHT - 0.008)),
        material=bumper,
        name="top_lip",
    )
    body.visual(
        Box((BUMPER_WIDTH, LIP_DEPTH, 0.020)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + LIP_DEPTH / 2.0, 0.010)),
        material=bumper,
        name="bottom_lip",
    )
    body.visual(
        Box((LIP_DEPTH, LIP_DEPTH, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(BUMPER_WIDTH / 2.0 - LIP_DEPTH / 2.0, BODY_DEPTH / 2.0 + LIP_DEPTH / 2.0, 0.085)),
        material=bumper,
        name="front_side_lip_1",
    )
    body.visual(
        Box((LIP_DEPTH, LIP_DEPTH, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(-(BUMPER_WIDTH / 2.0 - LIP_DEPTH / 2.0), BODY_DEPTH / 2.0 + LIP_DEPTH / 2.0, 0.085)),
        material=bumper,
        name="front_side_lip_0",
    )
    body.visual(
        Box((0.066, FRONT_PANEL_DEPTH, 0.134)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + FRONT_PANEL_DEPTH / 2.0, 0.086)),
        material=trim,
        name="front_panel",
    )
    body.visual(
        Box((DISPLAY_WIDTH, 0.002, DISPLAY_HEIGHT)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.003, DISPLAY_Z)),
        material=glass,
        name="display_window",
    )
    body.visual(
        Box((0.058, 0.0015, 0.020)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.00225, JACK_PANEL_Z)),
        material=label_grey,
        name="jack_panel",
    )
    for index, x_pos in enumerate((-0.020, 0.0, 0.020)):
        body.visual(
            Cylinder(radius=0.0055, length=0.003),
            origin=Origin(
                xyz=(x_pos, BODY_DEPTH / 2.0 + 0.0035, JACK_PANEL_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=jack_red if index == 2 else trim,
            name=f"jack_{index}",
        )
    body.visual(
        Box((0.020, 0.002, 0.006)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.003, 0.051)),
        material=label_grey,
        name="dial_mark",
    )
    body.visual(
        Box((0.050, 0.0015, 0.004)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.0025, 0.112)),
        material=label_grey,
        name="key_legend_strip",
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.005, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="selector_shaft",
    )
    selector.visual(
        Cylinder(radius=SELECTOR_RADIUS, length=0.012),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=control,
        name="selector_knob",
    )
    selector.visual(
        Box((0.006, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.015, 0.010)),
        material=label_grey,
        name="selector_pointer",
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + SELECTOR_FRONT_Y, SELECTOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    for index, x_pos in enumerate(KEY_XS):
        key = model.part(f"soft_key_{index}")
        key.visual(
            Box((KEY_WIDTH, KEY_DEPTH, KEY_HEIGHT)),
            origin=Origin(xyz=(0.0, KEY_DEPTH / 2.0, 0.0)),
            material=key_cap,
            name="key_cap",
        )
        key.visual(
            Box((KEY_WIDTH * 0.55, 0.0015, KEY_HEIGHT * 0.60)),
            origin=Origin(xyz=(0.0, -0.00075, 0.0)),
            material=control,
            name="key_stem",
        )
        key.visual(
            Box((KEY_WIDTH * 0.55, 0.0015, KEY_HEIGHT * 0.55)),
            origin=Origin(xyz=(0.0, 0.00375, 0.0)),
            material=label_grey,
            name="key_label",
        )
        model.articulation(
            f"body_to_soft_key_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(x_pos, BODY_DEPTH / 2.0 + 0.0035, KEY_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0015,
            ),
        )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.004, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    stand.visual(
        Box((0.028, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.001, 0.006)),
        material=trim,
        name="stand_bridge",
    )
    stand.visual(
        Box((0.055, 0.004, 0.080)),
        origin=Origin(xyz=(0.0, 0.002, 0.044)),
        material=control,
        name="stand_pad",
    )
    stand.visual(
        Box((0.040, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, 0.084)),
        material=control,
        name="stand_foot",
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, -(BODY_DEPTH / 2.0 + 0.004), STAND_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector = object_model.get_part("selector")
    stand = object_model.get_part("stand")
    selector_joint = object_model.get_articulation("body_to_selector")
    stand_joint = object_model.get_articulation("body_to_stand")
    key_joint_2 = object_model.get_articulation("body_to_soft_key_2")
    key_joint_1 = object_model.get_articulation("body_to_soft_key_1")
    soft_key_1 = object_model.get_part("soft_key_1")
    soft_key_2 = object_model.get_part("soft_key_2")

    ctx.expect_gap(
        body,
        stand,
        axis="y",
        positive_elem="housing_shell",
        negative_elem="stand_pad",
        max_gap=0.0015,
        max_penetration=1e-6,
        name="stand nests close to the rear shell when folded",
    )
    ctx.expect_gap(
        soft_key_2,
        body,
        axis="y",
        positive_elem="key_cap",
        negative_elem="front_panel",
        min_gap=0.0012,
        max_gap=0.0018,
        name="soft keys stand slightly proud of the front panel at rest",
    )

    selector_limits = selector_joint.motion_limits
    ctx.check(
        "selector is continuous and unbounded",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and selector_limits is not None
        and selector_limits.lower is None
        and selector_limits.upper is None,
        details=f"type={selector_joint.articulation_type}, limits={selector_limits}",
    )

    display_aabb = ctx.part_element_world_aabb(body, elem="display_window")
    selector_aabb = ctx.part_world_aabb(selector)
    key_aabbs = [ctx.part_world_aabb(object_model.get_part(f"soft_key_{index}")) for index in range(5)]
    jack_aabb = ctx.part_element_world_aabb(body, elem="jack_panel")
    row_positions = [ctx.part_world_position(object_model.get_part(f"soft_key_{index}")) for index in range(5)]

    layout_ok = (
        display_aabb is not None
        and selector_aabb is not None
        and jack_aabb is not None
        and all(aabb is not None for aabb in key_aabbs)
        and all(pos is not None for pos in row_positions)
        and min(aabb[0][2] for aabb in key_aabbs) < display_aabb[0][2]
        and max(aabb[1][2] for aabb in key_aabbs) > selector_aabb[1][2]
        and selector_aabb[0][2] > jack_aabb[1][2]
        and max(abs(pos[2] - row_positions[0][2]) for pos in row_positions) < 1e-6
        and all(row_positions[index][0] < row_positions[index + 1][0] for index in range(4))
    )
    ctx.check(
        "front controls follow display keys dial jack stack",
        layout_ok,
        details=(
            f"display={display_aabb}, selector={selector_aabb}, "
            f"jack={jack_aabb}, key_aabbs={key_aabbs}, row_positions={row_positions}"
        ),
    )

    stand_upper = stand_joint.motion_limits.upper if stand_joint.motion_limits is not None else None
    closed_stand = ctx.part_element_world_aabb(stand, elem="stand_pad")
    with ctx.pose({stand_joint: stand_upper} if stand_upper is not None else {}):
        open_stand = ctx.part_element_world_aabb(stand, elem="stand_pad")
    ctx.check(
        "stand swings outward behind the case",
        closed_stand is not None
        and open_stand is not None
        and stand_upper is not None
        and open_stand[0][1] < closed_stand[0][1] - 0.020,
        details=f"closed={closed_stand}, open={open_stand}, upper={stand_upper}",
    )

    key_rest = ctx.part_world_position(soft_key_2)
    neighbor_rest = ctx.part_world_position(soft_key_1)
    key_upper = key_joint_2.motion_limits.upper if key_joint_2.motion_limits is not None else None
    with ctx.pose({key_joint_2: key_upper} if key_upper is not None else {}):
        key_pressed = ctx.part_world_position(soft_key_2)
        neighbor_pressed = ctx.part_world_position(soft_key_1)
        ctx.expect_gap(
            soft_key_2,
            body,
            axis="y",
            positive_elem="key_cap",
            negative_elem="front_panel",
            max_gap=0.0001,
            max_penetration=1e-6,
            name="pressed soft key returns nearly flush to the panel",
        )
    ctx.check(
        "soft keys articulate independently inward",
        key_rest is not None
        and key_pressed is not None
        and neighbor_rest is not None
        and neighbor_pressed is not None
        and key_upper is not None
        and key_pressed[1] < key_rest[1] - 0.001
        and abs(neighbor_pressed[1] - neighbor_rest[1]) < 1e-7,
        details=(
            f"key_rest={key_rest}, key_pressed={key_pressed}, "
            f"neighbor_rest={neighbor_rest}, neighbor_pressed={neighbor_pressed}"
        ),
    )

    key_joints = [object_model.get_articulation(f"body_to_soft_key_{index}") for index in range(5)]
    ctx.check(
        "five independently driven prismatic soft keys are present",
        len(key_joints) == 5
        and all(joint.articulation_type == ArticulationType.PRISMATIC for joint in key_joints),
        details=f"key_joints={[joint.name for joint in key_joints]}",
    )

    return ctx.report()


object_model = build_object_model()
