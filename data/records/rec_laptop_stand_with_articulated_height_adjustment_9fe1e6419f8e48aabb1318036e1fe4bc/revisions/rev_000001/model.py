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


BASE_LENGTH = 0.300
BASE_WIDTH = 0.230
BASE_THICKNESS = 0.012

GUIDE_X = -0.048
GUIDE_THICKNESS = 0.022
GUIDE_WIDTH = 0.088
GUIDE_BOTTOM = 0.120
GUIDE_HEIGHT = 0.360
GUIDE_TOP = GUIDE_BOTTOM + GUIDE_HEIGHT

SLEEVE_ENTRY_Z = 0.190
SLEEVE_LENGTH = 0.088
SLEEVE_TRAVEL = 0.150
SLEEVE_OUTER_X = 0.054
SLEEVE_OUTER_Y = 0.104
SLEEVE_CLEARANCE = 0.0025

HINGE_X = 0.058
HINGE_Z = 0.070
PIN_RADIUS = 0.0055
PIN_LENGTH = 0.132
PARENT_KNUCKLE_RADIUS = 0.010
PARENT_KNUCKLE_LENGTH = 0.024
PARENT_KNUCKLE_Y = 0.050

TRAY_DEPTH = 0.255
TRAY_WIDTH = 0.290
TRAY_THICKNESS = 0.008
TRAY_REAR_X = 0.012
TRAY_BOTTOM_Z = 0.008
TRAY_BARREL_RADIUS = 0.0102
TRAY_BARREL_HOLE_RADIUS = PIN_RADIUS
TRAY_BARREL_LENGTH = 0.072
VENT_DEPTH = 0.150
VENT_WIDTH = 0.160
VENT_CENTER_X = 0.130
LIP_THICKNESS = 0.010
LIP_WIDTH = 0.220
LIP_HEIGHT = 0.019


def _base_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .ellipse(BASE_LENGTH / 2.0, BASE_WIDTH / 2.0)
        .extrude(BASE_THICKNESS)
    )


def _spine_support_shape() -> cq.Workplane:
    support = (
        cq.Workplane("XZ")
        .moveTo(-0.112, BASE_THICKNESS)
        .lineTo(-0.080, BASE_THICKNESS)
        .threePointArc((-0.060, 0.058), (-0.054, 0.165))
        .lineTo(-0.074, 0.165)
        .threePointArc((-0.097, 0.062), (-0.112, BASE_THICKNESS))
        .close()
        .extrude(GUIDE_WIDTH / 2.0, both=True)
    )
    return support


def _sleeve_body_shape() -> cq.Workplane:
    back_wall = (
        cq.Workplane("XY")
        .box(
            0.010,
            SLEEVE_OUTER_Y,
            SLEEVE_LENGTH,
            centered=(True, True, False),
        )
        .translate((-(GUIDE_THICKNESS / 2.0 + SLEEVE_CLEARANCE + 0.005), 0.0, 0.0))
    )
    front_wall = (
        cq.Workplane("XY")
        .box(
            0.013,
            SLEEVE_OUTER_Y,
            SLEEVE_LENGTH,
            centered=(True, True, False),
        )
        .translate(((GUIDE_THICKNESS / 2.0 + SLEEVE_CLEARANCE + 0.0065), 0.0, 0.0))
    )
    side_wall_left = (
        cq.Workplane("XY")
        .box(
            GUIDE_THICKNESS + 2.0 * SLEEVE_CLEARANCE,
            0.008,
            SLEEVE_LENGTH,
            centered=(True, True, False),
        )
        .translate((0.0, -(GUIDE_WIDTH / 2.0 + SLEEVE_CLEARANCE + 0.004), 0.0))
    )
    side_wall_right = (
        cq.Workplane("XY")
        .box(
            GUIDE_THICKNESS + 2.0 * SLEEVE_CLEARANCE,
            0.008,
            SLEEVE_LENGTH,
            centered=(True, True, False),
        )
        .translate((0.0, GUIDE_WIDTH / 2.0 + SLEEVE_CLEARANCE + 0.004, 0.0))
    )

    collar = back_wall.union(front_wall).union(side_wall_left).union(side_wall_right)

    head = (
        cq.Workplane("XY")
        .box(0.040, 0.100, 0.032, centered=(True, True, False))
        .translate((0.024, 0.0, 0.050))
    )

    cheek_left = (
        cq.Workplane("XY")
        .box(0.022, 0.022, 0.024, centered=(True, True, False))
        .translate((0.047, -PARENT_KNUCKLE_Y, 0.058))
    )
    cheek_right = (
        cq.Workplane("XY")
        .box(0.022, 0.022, 0.024, centered=(True, True, False))
        .translate((0.047, PARENT_KNUCKLE_Y, 0.058))
    )

    knuckle_left = (
        cq.Workplane("XZ")
        .circle(PARENT_KNUCKLE_RADIUS)
        .extrude(PARENT_KNUCKLE_LENGTH / 2.0, both=True)
        .translate((HINGE_X, -PARENT_KNUCKLE_Y, HINGE_Z))
    )
    knuckle_right = (
        cq.Workplane("XZ")
        .circle(PARENT_KNUCKLE_RADIUS)
        .extrude(PARENT_KNUCKLE_LENGTH / 2.0, both=True)
        .translate((HINGE_X, PARENT_KNUCKLE_Y, HINGE_Z))
    )

    return collar.union(head).union(cheek_left).union(cheek_right).union(knuckle_left).union(knuckle_right)


def _tray_body_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(TRAY_DEPTH, TRAY_WIDTH, TRAY_THICKNESS, centered=(True, True, False))
        .translate((TRAY_REAR_X + TRAY_DEPTH / 2.0, 0.0, TRAY_BOTTOM_Z))
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.050, 0.068, 0.018, centered=(True, True, False))
        .translate((0.028, 0.0, -0.008))
    )
    barrel = (
        cq.Workplane("XZ")
        .circle(TRAY_BARREL_RADIUS)
        .extrude(TRAY_BARREL_LENGTH / 2.0, both=True)
    )

    tray = plate.union(bridge).union(barrel)

    vent = (
        cq.Workplane("XY")
        .box(VENT_DEPTH, VENT_WIDTH, 0.040, centered=(True, True, False))
        .translate((VENT_CENTER_X, 0.0, TRAY_BOTTOM_Z - 0.010))
    )
    pin_bore = (
        cq.Workplane("XZ")
        .circle(TRAY_BARREL_HOLE_RADIUS)
        .extrude((PIN_LENGTH + 0.020) / 2.0, both=True)
    )
    return tray.cut(vent).cut(pin_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_laptop_stand")

    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.26, 0.27, 0.29, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.69, 0.72, 0.75, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_base_plate_shape(), "laptop_stand_base_plate"),
        material=charcoal,
        name="base_plate",
    )
    frame.visual(
        mesh_from_cadquery(_spine_support_shape(), "laptop_stand_spine_support"),
        material=charcoal,
        name="spine_support",
    )
    frame.visual(
        Box((GUIDE_THICKNESS, GUIDE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(GUIDE_X, 0.0, GUIDE_BOTTOM + GUIDE_HEIGHT / 2.0),
        ),
        material=graphite,
        name="guide",
    )

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_sleeve_body_shape(), "laptop_stand_sleeve_body"),
        material=satin_aluminum,
        name="sleeve_body",
    )
    sleeve.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(
            xyz=(HINGE_X, 0.0, HINGE_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="hinge_pin",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_body_shape(), "laptop_stand_tray_body"),
        material=satin_aluminum,
        name="tray_body",
    )
    tray.visual(
        Box((LIP_THICKNESS, LIP_WIDTH, LIP_HEIGHT)),
        origin=Origin(
            xyz=(
                TRAY_REAR_X + TRAY_DEPTH - LIP_THICKNESS / 2.0,
                0.0,
                TRAY_BOTTOM_Z + TRAY_THICKNESS + LIP_HEIGHT / 2.0,
            ),
        ),
        material=dark_rubber,
        name="front_lip",
    )

    sleeve_slide = model.articulation(
        "sleeve_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sleeve,
        origin=Origin(xyz=(GUIDE_X, 0.0, SLEEVE_ENTRY_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=0.0,
            upper=SLEEVE_TRAVEL,
        ),
    )
    tray_tilt = model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=tray,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=math.radians(-15.0),
            upper=math.radians(28.0),
        ),
    )

    frame.meta["primary_articulation"] = sleeve_slide.name
    sleeve.meta["secondary_articulation"] = tray_tilt.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sleeve = object_model.get_part("sleeve")
    tray = object_model.get_part("tray")
    sleeve_slide = object_model.get_articulation("sleeve_slide")
    tray_tilt = object_model.get_articulation("tray_tilt")

    ctx.allow_overlap(
        frame,
        sleeve,
        elem_a="guide",
        elem_b="sleeve_body",
        reason="The sliding sleeve is intentionally authored as a close-fitting collar around the central guide.",
    )
    ctx.allow_overlap(
        sleeve,
        tray,
        elem_a="hinge_pin",
        elem_b="tray_body",
        reason="The tray body is intentionally represented as rotating around the hinge pin through its central barrel.",
    )

    ctx.expect_overlap(
        sleeve,
        frame,
        axes="xy",
        elem_a="sleeve_body",
        elem_b="guide",
        min_overlap=0.020,
        name="sleeve stays centered over the guide at rest",
    )
    ctx.expect_overlap(
        sleeve,
        frame,
        axes="z",
        elem_a="sleeve_body",
        elem_b="guide",
        min_overlap=0.080,
        name="sleeve remains engaged on the guide at rest",
    )
    ctx.expect_gap(
        tray,
        sleeve,
        axis="x",
        positive_elem="front_lip",
        negative_elem="sleeve_body",
        min_gap=0.150,
        name="front lip projects well ahead of the sliding sleeve",
    )

    rest_sleeve_pos = ctx.part_world_position(sleeve)
    with ctx.pose({sleeve_slide: SLEEVE_TRAVEL}):
        ctx.expect_overlap(
            sleeve,
            frame,
            axes="xy",
            elem_a="sleeve_body",
            elem_b="guide",
            min_overlap=0.020,
            name="sleeve stays centered over the guide at full height",
        )
        ctx.expect_overlap(
            sleeve,
            frame,
            axes="z",
            elem_a="sleeve_body",
            elem_b="guide",
            min_overlap=0.080,
            name="sleeve remains engaged on the guide at full height",
        )
        high_sleeve_pos = ctx.part_world_position(sleeve)

    ctx.check(
        "sleeve raises upward along the spine",
        rest_sleeve_pos is not None
        and high_sleeve_pos is not None
        and high_sleeve_pos[2] > rest_sleeve_pos[2] + 0.10,
        details=f"rest={rest_sleeve_pos}, high={high_sleeve_pos}",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(tray, elem="front_lip")
    with ctx.pose({tray_tilt: math.radians(28.0)}):
        raised_lip_aabb = ctx.part_element_world_aabb(tray, elem="front_lip")

    rest_lip_center_z = None
    raised_lip_center_z = None
    if rest_lip_aabb is not None:
        rest_lip_center_z = 0.5 * (rest_lip_aabb[0][2] + rest_lip_aabb[1][2])
    if raised_lip_aabb is not None:
        raised_lip_center_z = 0.5 * (raised_lip_aabb[0][2] + raised_lip_aabb[1][2])

    ctx.check(
        "tray front lip rises when the hinge tilts upward",
        rest_lip_center_z is not None
        and raised_lip_center_z is not None
        and raised_lip_center_z > rest_lip_center_z + 0.08,
        details=f"rest_z={rest_lip_center_z}, raised_z={raised_lip_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
