from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_DEPTH = 0.050
BODY_FRONT_Y = BODY_DEPTH / 2.0
BODY_REAR_Y = -BODY_DEPTH / 2.0
BODY_HEIGHT = 0.178
BODY_BOTTOM_Z = -BODY_HEIGHT / 2.0

DIAL_Z = -0.010
KEY_Z = -0.056
DISPLAY_Z = 0.050

KEY_WIDTH = 0.012
KEY_DEPTH = 0.0046
KEY_HEIGHT = 0.010
KEY_TRAVEL = 0.0024
KEY_XS = (-0.032, -0.016, 0.0, 0.016, 0.032)
KEY_JOINT_Y = 0.0200

STAND_THICKNESS = 0.008
STAND_HEIGHT = 0.098
HINGE_RADIUS = 0.0045
HINGE_Y = BODY_REAR_Y - HINGE_RADIUS + 0.0005
HINGE_Z = BODY_BOTTOM_Z + 0.0045


def build_body_shape() -> cq.Workplane:
    upper = (
        cq.Workplane("XY")
        .box(0.092, BODY_DEPTH, 0.110)
        .edges("|Z")
        .fillet(0.0105)
        .translate((0.0, 0.0, 0.035))
    )
    lower = (
        cq.Workplane("XY")
        .box(0.104, BODY_DEPTH, 0.092)
        .edges("|Z")
        .fillet(0.0125)
        .translate((0.0, 0.0, -0.044))
    )
    body = upper.union(lower)

    display_recess = (
        cq.Workplane("XY")
        .box(0.070, 0.008, 0.042)
        .translate((0.0, BODY_FRONT_Y - 0.003, DISPLAY_Z))
    )
    body = body.cut(display_recess)
    for x_pos in KEY_XS:
        key_pocket = (
            cq.Workplane("XY")
            .box(0.0105, 0.008, 0.0085)
            .translate((x_pos, BODY_FRONT_Y - 0.004, KEY_Z))
        )
        body = body.cut(key_pocket)
    return body


def build_stand_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.038, 0.0),
                (0.038, 0.0),
                (0.032, STAND_HEIGHT),
                (-0.032, STAND_HEIGHT),
            ]
        )
        .close()
        .extrude(-STAND_THICKNESS)
    )
    cutout = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.016, 0.018),
                (0.016, 0.018),
                (0.012, 0.073),
                (-0.012, 0.073),
            ]
        )
        .close()
        .extrude(-(STAND_THICKNESS + 0.002))
    )
    return plate.cut(cutout)


def build_bumper_shape(width: float, depth: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges()
        .fillet(min(width, depth, height) * 0.22)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_digital_multimeter")

    housing_dark = model.material("housing_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    bumper_yellow = model.material("bumper_yellow", rgba=(0.90, 0.73, 0.10, 1.0))
    dial_black = model.material("dial_black", rgba=(0.07, 0.07, 0.08, 1.0))
    stand_dark = model.material("stand_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    key_grey = model.material("key_grey", rgba=(0.74, 0.77, 0.80, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.17, 0.24, 0.20, 0.95))
    lcd_green = model.material("lcd_green", rgba=(0.52, 0.63, 0.44, 1.0))
    accent_ring = model.material("accent_ring", rgba=(0.60, 0.62, 0.64, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(build_body_shape(), "multimeter_body"),
        material=housing_dark,
        name="housing_shell",
    )
    body.visual(
        Box((0.062, 0.0015, 0.034)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.00625, DISPLAY_Z)),
        material=glass_tint,
        name="display_glass",
    )
    body.visual(
        Box((0.056, 0.001, 0.028)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.0055, DISPLAY_Z - 0.0005)),
        material=lcd_green,
        name="display_lcd",
    )
    body.visual(
        Cylinder(radius=0.033, length=0.0012),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.0006, DIAL_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent_ring,
        name="dial_ring",
    )

    upper_bumper_mesh = mesh_from_cadquery(build_bumper_shape(0.018, 0.014, 0.034), "upper_bumper")
    lower_bumper_mesh = mesh_from_cadquery(build_bumper_shape(0.022, 0.016, 0.040), "lower_bumper")
    bumper_positions = (
        ("upper", upper_bumper_mesh, -0.047, 0.0, 0.073),
        ("upper", upper_bumper_mesh, 0.047, 0.0, 0.073),
        ("lower", lower_bumper_mesh, -0.050, 0.0, -0.075),
        ("lower", lower_bumper_mesh, 0.050, 0.0, -0.075),
    )
    for idx, (_, bumper_mesh, x_pos, y_pos, z_pos) in enumerate(bumper_positions):
        body.visual(
            bumper_mesh,
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            material=bumper_yellow,
            name=f"bumper_{idx}",
        )

    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.022),
        origin=Origin(xyz=(-0.022, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_dark,
        name="hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.022),
        origin=Origin(xyz=(0.022, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_dark,
        name="hinge_barrel_1",
    )

    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.018,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.054, 0.004, flare=0.06),
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            center=False,
        ),
        "range_dial",
    )
    dial = model.part("dial")
    dial.visual(
        dial_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="dial_knob",
    )

    for idx, x_pos in enumerate(KEY_XS):
        key = model.part(f"key_{idx}")
        key.visual(
            Box((0.0092, 0.0050, 0.0082)),
            origin=Origin(xyz=(0.0, 0.0025, 0.0)),
            material=key_grey,
            name="key_stem",
        )
        key.visual(
            Box((0.0128, 0.0020, 0.0102)),
            origin=Origin(xyz=(0.0, 0.0060, 0.0)),
            material=key_grey,
            name="key_cap",
        )
        model.articulation(
            f"body_to_key_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(x_pos, KEY_JOINT_Y, KEY_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.08,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(build_stand_shape(), "rear_stand"),
        origin=Origin(xyz=(0.0, -STAND_THICKNESS, 0.0)),
        material=stand_dark,
        name="stand_panel",
    )
    stand.visual(
        Cylinder(radius=0.0040, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_dark,
        name="stand_barrel",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    stand = object_model.get_part("stand")
    stand_joint = object_model.get_articulation("body_to_stand")
    dial_joint = object_model.get_articulation("body_to_dial")

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "dial_is_continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_limits is not None
        and dial_limits.lower is None
        and dial_limits.upper is None,
        details=f"type={dial_joint.articulation_type!r}, limits={dial_limits!r}",
    )

    ctx.expect_gap(
        dial,
        body,
        axis="y",
        positive_elem="dial_knob",
        negative_elem="dial_ring",
        min_gap=-0.0001,
        max_gap=0.0004,
        name="dial seats close to the front housing",
    )

    ctx.expect_gap(
        body,
        stand,
        axis="y",
        positive_elem="housing_shell",
        negative_elem="stand_panel",
        min_gap=0.0035,
        max_gap=0.0045,
        name="stand folds near the rear housing without overlap",
    )

    closed_stand_aabb = ctx.part_element_world_aabb(stand, elem="stand_panel")
    with ctx.pose({stand_joint: 1.0}):
        open_stand_aabb = ctx.part_element_world_aabb(stand, elem="stand_panel")
    ctx.check(
        "stand opens rearward",
        closed_stand_aabb is not None
        and open_stand_aabb is not None
        and open_stand_aabb[0][1] < closed_stand_aabb[0][1] - 0.025,
        details=f"closed={closed_stand_aabb!r}, open={open_stand_aabb!r}",
    )

    for idx in range(5):
        key = object_model.get_part(f"key_{idx}")
        key_joint = object_model.get_articulation(f"body_to_key_{idx}")
        limits = key_joint.motion_limits
        ctx.expect_gap(
            key,
            body,
            axis="y",
            positive_elem="key_cap",
            negative_elem="housing_shell",
            min_gap=-0.0002,
            max_gap=0.0006,
            name=f"key_{idx} rests on the front face",
        )
        rest_pos = ctx.part_world_position(key)
        pressed_pos = None
        if limits is not None and limits.upper is not None:
            with ctx.pose({key_joint: limits.upper}):
                pressed_pos = ctx.part_world_position(key)
        ctx.check(
            f"key_{idx} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.0015,
            details=f"rest={rest_pos!r}, pressed={pressed_pos!r}",
        )

    return ctx.report()


object_model = build_object_model()
