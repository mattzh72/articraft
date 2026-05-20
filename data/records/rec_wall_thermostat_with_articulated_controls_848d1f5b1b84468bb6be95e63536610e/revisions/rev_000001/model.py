from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_RADIUS = 0.040
TOTAL_DEPTH = 0.028
FRONT_DISC_RADIUS = 0.0268
FRONT_DISC_DEPTH = 0.004

RING_OUTER_RADIUS = 0.043
RING_SLEEVE_INNER_RADIUS = BODY_RADIUS + 0.0006
RING_FACE_INNER_RADIUS = 0.0285
RING_SLEEVE_DEPTH = 0.006
RING_FACE_DEPTH = 0.0028
RING_REAR_OFFSET = 0.0068

BUTTON_CAP_RADIUS = 0.0241
BUTTON_CAP_DEPTH = 0.0048
BUTTON_STEM_RADIUS = 0.0200
BUTTON_STEM_DEPTH = 0.0075
BUTTON_PROUD = 0.0010
BUTTON_TRAVEL = 0.0028
BUTTON_SEAT_RADIUS = 0.0242
BUTTON_SEAT_DEPTH = 0.0058
BUTTON_WELL_RADIUS = 0.0207
BUTTON_WELL_DEPTH = 0.0155

DRAWER_WIDTH = 0.032
DRAWER_BODY_HEIGHT = 0.028
DRAWER_DEPTH = 0.016
DRAWER_CENTER_Y = 0.010
DRAWER_CAP_WIDTH = 0.033
DRAWER_CAP_HEIGHT = 0.008
DRAWER_CAP_DEPTH = 0.0034
DRAWER_CAP_CENTER_Y = -0.0005
DRAWER_FRONT_RECESS = 0.0008
DRAWER_TRAVEL = 0.012
DRAWER_SLOT_WIDTH = 0.0344
DRAWER_SLOT_HEIGHT = 0.033
DRAWER_SLOT_DEPTH = 0.0182
DRAWER_SLOT_CENTER_Y = -0.028
DRAWER_SLOT_FRONT_RECESS = 0.0006


def _build_housing_shape() -> cq.Workplane:
    back_body = cq.Workplane("XY").circle(BODY_RADIUS).extrude(TOTAL_DEPTH - FRONT_DISC_DEPTH)
    front_disc = (
        cq.Workplane("XY")
        .circle(FRONT_DISC_RADIUS)
        .extrude(FRONT_DISC_DEPTH)
        .translate((0.0, 0.0, TOTAL_DEPTH - FRONT_DISC_DEPTH))
    )
    housing = back_body.union(front_disc)

    button_seat = (
        cq.Workplane("XY")
        .circle(BUTTON_SEAT_RADIUS)
        .extrude(BUTTON_SEAT_DEPTH)
        .translate((0.0, 0.0, TOTAL_DEPTH - BUTTON_SEAT_DEPTH))
    )
    button_well = (
        cq.Workplane("XY")
        .circle(BUTTON_WELL_RADIUS)
        .extrude(BUTTON_WELL_DEPTH)
        .translate((0.0, 0.0, TOTAL_DEPTH - BUTTON_WELL_DEPTH))
    )
    drawer_slot = (
        cq.Workplane("XY")
        .box(DRAWER_SLOT_WIDTH, DRAWER_SLOT_HEIGHT, DRAWER_SLOT_DEPTH)
        .translate(
            (
                0.0,
                DRAWER_SLOT_CENTER_Y,
                TOTAL_DEPTH - DRAWER_SLOT_FRONT_RECESS - DRAWER_SLOT_DEPTH * 0.5,
            )
        )
    )
    return housing.cut(button_seat).cut(button_well).cut(drawer_slot)


def _build_ring_shape() -> cq.Workplane:
    rear_sleeve = cq.Workplane("XY").circle(RING_OUTER_RADIUS).extrude(RING_SLEEVE_DEPTH)
    rear_sleeve = rear_sleeve.cut(
        cq.Workplane("XY").circle(RING_SLEEVE_INNER_RADIUS).extrude(RING_SLEEVE_DEPTH)
    )
    front_bezel = cq.Workplane("XY").circle(RING_OUTER_RADIUS).extrude(RING_FACE_DEPTH)
    front_bezel = front_bezel.cut(
        cq.Workplane("XY").circle(RING_FACE_INNER_RADIUS).extrude(RING_FACE_DEPTH)
    ).translate((0.0, 0.0, RING_SLEEVE_DEPTH))
    ring = rear_sleeve.union(front_bezel)
    drawer_notch = (
        cq.Workplane("XY")
        .box(0.040, 0.032, RING_SLEEVE_DEPTH + RING_FACE_DEPTH + 0.001)
        .translate((0.0, -0.028, (RING_SLEEVE_DEPTH + RING_FACE_DEPTH) * 0.5))
    )
    return ring.cut(drawer_notch)


def _build_drawer_shape() -> cq.Workplane:
    tray = (
        cq.Workplane("XY")
        .box(DRAWER_WIDTH, DRAWER_BODY_HEIGHT, DRAWER_DEPTH)
        .translate((0.0, DRAWER_CENTER_Y, 0.0))
    )
    cap = (
        cq.Workplane("XY")
        .box(DRAWER_CAP_WIDTH, DRAWER_CAP_HEIGHT, DRAWER_CAP_DEPTH)
        .translate(
            (
                0.0,
                DRAWER_CAP_CENTER_Y,
                DRAWER_DEPTH * 0.5 - DRAWER_CAP_DEPTH * 0.5 + 0.0002,
            )
        )
    )
    return tray.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_wall_thermostat")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    ring_metal = model.material("ring_metal", rgba=(0.79, 0.80, 0.82, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shape(), "housing_shell"),
        origin=Origin(xyz=(0.0, 0.0, -TOTAL_DEPTH)),
        material=shell_white,
        name="housing_shell",
    )

    ring = model.part("outer_ring")
    ring.visual(
        mesh_from_cadquery(_build_ring_shape(), "outer_ring"),
        origin=Origin(xyz=(0.0, 0.0, -RING_REAR_OFFSET)),
        material=ring_metal,
        name="outer_ring_shell",
    )

    button = model.part("center_button")
    button.visual(
        Cylinder(radius=BUTTON_CAP_RADIUS, length=BUTTON_CAP_DEPTH),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_PROUD - BUTTON_CAP_DEPTH * 0.5)),
        material=graphite,
        name="button_cap",
    )
    button.visual(
        Cylinder(radius=BUTTON_STEM_RADIUS, length=BUTTON_STEM_DEPTH),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BUTTON_PROUD - BUTTON_CAP_DEPTH - BUTTON_STEM_DEPTH * 0.5,
            )
        ),
        material=graphite,
        name="button_stem",
    )

    drawer = model.part("battery_drawer")
    drawer.visual(
        mesh_from_cadquery(_build_drawer_shape(), "battery_drawer"),
        origin=Origin(),
        material=shell_white,
        name="battery_drawer_shell",
    )

    model.articulation(
        "ring_rotation",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    model.articulation(
        "button_press",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=button,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(
            xyz=(
                0.0,
                -BODY_RADIUS,
                -(DRAWER_FRONT_RECESS + DRAWER_DEPTH * 0.5),
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.05,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    ring = object_model.get_part("outer_ring")
    button = object_model.get_part("center_button")
    drawer = object_model.get_part("battery_drawer")

    ring_rotation = object_model.get_articulation("ring_rotation")
    button_press = object_model.get_articulation("button_press")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.allow_isolated_part(
        ring,
        reason="The control ring rides on a concealed bearing gap around the housing shoulder that is not explicitly meshed.",
    )
    ctx.allow_isolated_part(
        button,
        reason="The center button rides in a concealed plunger guide with small running clearance that is not explicitly meshed.",
    )
    ctx.allow_isolated_part(
        drawer,
        reason="The battery drawer rides on concealed internal guide rails with running clearance that are not explicitly meshed.",
    )

    ctx.check(
        "outer ring uses continuous rotation",
        ring_rotation.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={ring_rotation.articulation_type}",
    )
    ctx.check(
        "center button uses prismatic motion",
        button_press.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={button_press.articulation_type}",
    )
    ctx.check(
        "battery drawer uses prismatic motion",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={drawer_slide.articulation_type}",
    )

    ctx.expect_overlap(
        ring,
        housing,
        axes="xy",
        min_overlap=0.075,
        name="outer ring stays concentric with the thermostat body",
    )
    ctx.expect_within(
        drawer,
        housing,
        axes="xz",
        margin=0.003,
        name="battery drawer stays centered inside the housing opening",
    )

    housing_aabb = ctx.part_element_world_aabb(housing, elem="housing_shell")
    button_cap_aabb = ctx.part_element_world_aabb(button, elem="button_cap")
    button_proud = None
    if housing_aabb is not None and button_cap_aabb is not None:
        button_proud = button_cap_aabb[1][2] - housing_aabb[1][2]
    ctx.check(
        "button sits slightly proud at rest",
        button_proud is not None and 0.0004 <= button_proud <= 0.0016,
        details=f"button_proud={button_proud}",
    )

    button_rest = ctx.part_world_position(button)
    with ctx.pose({button_press: BUTTON_TRAVEL}):
        button_pressed = ctx.part_world_position(button)
    ctx.check(
        "button depresses into the housing",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[2] < button_rest[2] - 0.002,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_within(
            drawer,
            housing,
            axes="xz",
            margin=0.003,
            name="extended battery drawer stays aligned with the opening",
        )
        ctx.expect_overlap(
            drawer,
            housing,
            axes="y",
            min_overlap=0.010,
            name="extended battery drawer remains retained in the housing",
        )
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "battery drawer slides downward from the bottom edge",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[1] < drawer_rest[1] - 0.008,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    return ctx.report()


object_model = build_object_model()
