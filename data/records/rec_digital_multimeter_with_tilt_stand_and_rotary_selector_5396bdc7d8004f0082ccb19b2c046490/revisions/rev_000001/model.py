from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.090
BODY_D = 0.045
BODY_H = 0.178

OVERMOLD_W = 0.102
OVERMOLD_D = 0.054
OVERMOLD_H = 0.190
OVERMOLD_CORNER_R = 0.014
OVERMOLD_OPEN_W = 0.082
OVERMOLD_OPEN_H = 0.158

DISPLAY_W = 0.056
DISPLAY_H = 0.036
DISPLAY_Z = 0.045
DISPLAY_RECESS = 0.0016
DISPLAY_THICKNESS = 0.0014

KEY_COUNT = 5
KEY_CAP_W = 0.0125
KEY_CAP_H = 0.0082
KEY_CAP_D = 0.0030
KEY_STEM_W = 0.0068
KEY_STEM_H = 0.0042
KEY_STEM_D = 0.0060
KEY_ROW_Z = 0.012
KEY_PITCH = 0.014
KEY_TRAVEL = 0.0014

KNOB_DIAMETER = 0.040
KNOB_HEIGHT = 0.018
KNOB_Z = -0.030
KNOB_BEZEL_DIAMETER = 0.049
KNOB_BEZEL_THICKNESS = 0.0018

STAND_W = 0.072
STAND_H = 0.122
STAND_T = 0.0030
STAND_PIVOT_SPAN = 0.068
STAND_PIVOT_R = 0.0042
STAND_OPEN_ANGLE = 1.05


def _body_core_shape():
    return (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .edges("|Z")
        .fillet(0.0075)
        .val()
    )


def _overmold_shape():
    outer = (
        cq.Workplane("XY")
        .box(OVERMOLD_W, OVERMOLD_D, OVERMOLD_H)
        .edges("|Y")
        .fillet(OVERMOLD_CORNER_R)
    )
    opening = cq.Workplane("XY").box(OVERMOLD_OPEN_W, OVERMOLD_D + 0.004, OVERMOLD_OPEN_H)
    return outer.cut(opening).val()


def _soft_key_shape():
    return (
        cq.Workplane("XY")
        .box(KEY_CAP_W, KEY_CAP_D, KEY_CAP_H)
        .translate((0.0, KEY_CAP_D * 0.5, 0.0))
        .edges("|Y")
        .fillet(0.0016)
        .val()
    )


def _rear_support_shape():
    frame = (
        cq.Workplane("XY")
        .box(STAND_W, STAND_T, STAND_H)
        .translate((0.0, -STAND_T * 0.5, STAND_H * 0.5))
        .edges("|Y")
        .fillet(0.006)
    )
    opening = (
        cq.Workplane("XY")
        .box(STAND_W - 0.018, STAND_T + 0.002, STAND_H - 0.034)
        .translate((0.0, -STAND_T * 0.5, STAND_H * 0.53))
    )
    top_bridge = (
        cq.Workplane("XY")
        .box(STAND_W * 0.34, STAND_T, 0.008)
        .translate((0.0, -STAND_T * 0.5, STAND_H - 0.010))
        .edges("|Y")
        .fillet(0.002)
    )

    pivot_half = STAND_PIVOT_SPAN * 0.5
    left_pivot = (
        cq.Workplane("YZ")
        .circle(STAND_PIVOT_R)
        .extrude(0.010)
        .translate((-pivot_half - 0.005, -STAND_T * 0.5, STAND_PIVOT_R))
    )
    right_pivot = (
        cq.Workplane("YZ")
        .circle(STAND_PIVOT_R)
        .extrude(0.010)
        .translate((pivot_half - 0.005, -STAND_T * 0.5, STAND_PIVOT_R))
    )

    return frame.cut(opening).union(top_bridge).union(left_pivot).union(right_pivot).val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_multimeter")

    housing_dark = model.material("housing_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    overmold_yellow = model.material("overmold_yellow", rgba=(0.93, 0.71, 0.12, 1.0))
    key_gray = model.material("key_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.12, 0.13, 1.0))
    support_black = model.material("support_black", rgba=(0.12, 0.12, 0.13, 1.0))
    display_glass = model.material("display_glass", rgba=(0.23, 0.36, 0.39, 0.58))
    trim_gray = model.material("trim_gray", rgba=(0.52, 0.55, 0.58, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_core_shape(), "body_core"),
        material=housing_dark,
        name="body_core",
    )
    body.visual(
        mesh_from_cadquery(_overmold_shape(), "overmold"),
        material=overmold_yellow,
        name="overmold",
    )
    body.visual(
        Cylinder(radius=KNOB_BEZEL_DIAMETER * 0.5, length=KNOB_BEZEL_THICKNESS),
        origin=Origin(
            xyz=(
                0.0,
                BODY_D * 0.5 + KNOB_BEZEL_THICKNESS * 0.5,
                KNOB_Z,
            ),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=trim_gray,
        name="selector_bezel",
    )
    body.inertial = Inertial.from_geometry(Box((OVERMOLD_W, OVERMOLD_D, OVERMOLD_H)), mass=0.68)

    display = model.part("display")
    display.visual(
        Box((DISPLAY_W, DISPLAY_THICKNESS, DISPLAY_H)),
        material=display_glass,
        name="window",
    )
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(
            xyz=(
                0.0,
                BODY_D * 0.5 - DISPLAY_THICKNESS * 0.5,
                DISPLAY_Z,
            )
        ),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            KNOB_DIAMETER,
            KNOB_HEIGHT,
            body_style="cylindrical",
            grip=KnobGrip(style="knurled", count=34, depth=0.0011, helix_angle_deg=22.0),
            center=False,
        ),
        "selector_knob",
    )
    knob = model.part("selector_knob")
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=knob_black,
        name="knob",
    )
    knob.visual(
        Box((0.003, 0.0008, 0.013)),
        origin=Origin(
            xyz=(0.0, KNOB_HEIGHT + 0.0004, KNOB_DIAMETER * 0.18),
        ),
        material=trim_gray,
        name="pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=KNOB_DIAMETER * 0.5, length=KNOB_HEIGHT),
        mass=0.05,
        origin=Origin(xyz=(0.0, KNOB_HEIGHT * 0.5, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, BODY_D * 0.5 + 0.0003, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    key_mesh = mesh_from_cadquery(_soft_key_shape(), "soft_key_cap")
    key_x0 = -0.5 * KEY_PITCH * (KEY_COUNT - 1)
    for index in range(KEY_COUNT):
        key = model.part(f"key_{index}")
        key.visual(key_mesh, material=key_gray, name="cap")
        key.visual(
            Box((KEY_STEM_W, KEY_STEM_D, KEY_STEM_H)),
            origin=Origin(xyz=(0.0, -KEY_STEM_D * 0.5, 0.0)),
            material=key_gray,
            name="stem",
        )
        key.inertial = Inertial.from_geometry(
            Box((KEY_CAP_W, KEY_STEM_D + KEY_CAP_D, KEY_CAP_H)),
            mass=0.006,
        )
        model.articulation(
            f"body_to_key_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(key_x0 + index * KEY_PITCH, BODY_D * 0.5 + 0.0001, KEY_ROW_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.04,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )

    support = model.part("rear_support")
    support.visual(
        mesh_from_cadquery(_rear_support_shape(), "rear_support"),
        material=support_black,
        name="support",
    )
    support.inertial = Inertial.from_geometry(Box((STAND_W, STAND_T, STAND_H)), mass=0.08)
    model.articulation(
        "body_to_rear_support",
        ArticulationType.REVOLUTE,
        parent=body,
        child=support,
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 - 0.0004, -BODY_H * 0.5 + 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=0.0,
            upper=STAND_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    knob = object_model.get_part("selector_knob")
    rear_support = object_model.get_part("rear_support")
    knob_joint = object_model.get_articulation("body_to_selector_knob")
    stand_joint = object_model.get_articulation("body_to_rear_support")

    key_parts = [object_model.get_part(f"key_{index}") for index in range(KEY_COUNT)]
    key_joints = [object_model.get_articulation(f"body_to_key_{index}") for index in range(KEY_COUNT)]

    ctx.expect_overlap(knob, body, axes="xz", min_overlap=0.030, elem_b="body_core", name="selector knob sits on the front body footprint")
    ctx.expect_overlap(
        rear_support,
        body,
        axes="x",
        min_overlap=0.060,
        elem_a="support",
        elem_b="body_core",
        name="rear support spans most of the back width",
    )

    rest_knob_pos = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: math.pi * 0.5}):
        quarter_turn_knob_pos = ctx.part_world_position(knob)
    ctx.check(
        "selector knob spins in place",
        rest_knob_pos is not None
        and quarter_turn_knob_pos is not None
        and max(abs(a - b) for a, b in zip(rest_knob_pos, quarter_turn_knob_pos)) < 1e-6,
        details=f"rest={rest_knob_pos}, quarter_turn={quarter_turn_knob_pos}",
    )

    rest_key_positions = {index: ctx.part_world_position(key) for index, key in enumerate(key_parts)}
    for index, (key, joint) in enumerate(zip(key_parts, key_joints)):
        ctx.allow_overlap(
            body,
            key,
            elem_a="body_core",
            elem_b="stem",
            reason="Each soft key uses a rear plunger stem that intentionally enters the front housing.",
        )
        limits = joint.motion_limits
        upper = 0.0 if limits is None or limits.upper is None else limits.upper
        with ctx.pose({joint: upper}):
            pressed_pos = ctx.part_world_position(key)
            neighbor_index = (index + 1) % KEY_COUNT
            neighbor_pressed_pos = ctx.part_world_position(key_parts[neighbor_index])

        rest_pos = rest_key_positions[index]
        neighbor_rest_pos = rest_key_positions[neighbor_index]
        ctx.check(
            f"key_{index} presses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] < rest_pos[1] - 0.0010,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )
        ctx.check(
            f"key_{index} moves independently",
            neighbor_rest_pos is not None
            and neighbor_pressed_pos is not None
            and max(abs(a - b) for a, b in zip(neighbor_rest_pos, neighbor_pressed_pos)) < 1e-6,
            details=f"neighbor_rest={neighbor_rest_pos}, neighbor_pressed={neighbor_pressed_pos}",
        )
        ctx.expect_overlap(
            key,
            body,
            axes="xz",
            min_overlap=0.008,
            elem_a="cap",
            elem_b="body_core",
            name=f"key_{index} stays aligned with the front panel",
        )

    rest_support_aabb = ctx.part_world_aabb(rear_support)
    with ctx.pose({stand_joint: STAND_OPEN_ANGLE}):
        open_support_aabb = ctx.part_world_aabb(rear_support)
    ctx.check(
        "rear support swings backward from the back panel",
        rest_support_aabb is not None
        and open_support_aabb is not None
        and open_support_aabb[0][1] < rest_support_aabb[0][1] - 0.030,
        details=f"rest={rest_support_aabb}, open={open_support_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
