from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_RADIUS = 0.043
BODY_HEIGHT = 0.022
TOP_Z = BODY_HEIGHT / 2.0
BOTTOM_Z = -BODY_HEIGHT / 2.0


def _rounded_cylinder(radius: float, height: float) -> cq.Workplane:
    """Soft-edged puck shell centered on the local origin."""
    return (
        cq.Workplane("XY")
        .cylinder(height, radius)
        .edges("%Circle")
        .fillet(0.0022)
    )


def _selector_ring() -> cq.Workplane:
    """A low annular rotating ring with tactile raised radial ticks."""
    outer_radius = 0.0365
    inner_radius = 0.0252
    ring_height = 0.0032

    ring = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(ring_height)
        .edges("%Circle")
        .fillet(0.00055)
    )

    tick_radius = (outer_radius + inner_radius) / 2.0
    for index in range(24):
        angle = index * 360.0 / 24.0
        tick = (
            cq.Workplane("XY")
            .box(0.0052, 0.00105, 0.00085)
            .translate((tick_radius, 0.0, ring_height + 0.00025))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        ring = ring.union(tick)
    return ring


def _battery_door() -> cq.Workplane:
    """Underside sliding rectangular battery cover with a thumb-grip detail."""
    door_length = 0.050
    door_width = 0.029
    door_thickness = 0.0024
    ridge_height = 0.0008

    door = (
        cq.Workplane("XY")
        .rect(door_length, door_width)
        .extrude(-door_thickness)
        .edges("|Z")
        .fillet(0.004)
    )

    for x in (-0.016, -0.011, -0.006):
        ridge = (
            cq.Workplane("XY")
            .box(0.0021, 0.018, ridge_height)
            .translate((x, 0.0, -door_thickness - ridge_height / 2.0 + 0.00018))
        )
        door = door.union(ridge)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_home_puck_remote")

    body_mat = model.material("warm_matte_white", rgba=(0.86, 0.84, 0.79, 1.0))
    ring_mat = model.material("graphite_selector", rgba=(0.03, 0.035, 0.04, 1.0))
    center_mat = model.material("soft_touch_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    door_mat = model.material("underside_battery_door", rgba=(0.62, 0.61, 0.57, 1.0))
    led_mat = model.material("tiny_status_led", rgba=(0.25, 0.95, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_cylinder(BODY_RADIUS, BODY_HEIGHT), "rounded_puck_body"),
        material=body_mat,
        name="rounded_shell",
    )
    body.visual(
        Cylinder(radius=0.0205, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z + 0.0005)),
        material=center_mat,
        name="center_touch_disc",
    )
    body.visual(
        Cylinder(radius=0.0021, length=0.00045),
        origin=Origin(xyz=(0.0, 0.012, TOP_Z + 0.00125)),
        material=led_mat,
        name="status_led",
    )

    selector_ring = model.part("selector_ring")
    selector_ring.visual(
        mesh_from_cadquery(_selector_ring(), "selector_ring_with_ticks"),
        material=ring_mat,
        name="selector_ring",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        mesh_from_cadquery(_battery_door(), "sliding_battery_door"),
        material=door_mat,
        name="door_panel",
    )

    model.articulation(
        "body_to_selector_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_ring,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.030, effort=8.0, velocity=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector_ring = object_model.get_part("selector_ring")
    battery_door = object_model.get_part("battery_door")
    ring_joint = object_model.get_articulation("body_to_selector_ring")
    door_joint = object_model.get_articulation("body_to_battery_door")

    ctx.check(
        "selector ring is continuous",
        ring_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={ring_joint.articulation_type}",
    )
    ctx.check(
        "selector ring rotates about vertical axis",
        tuple(round(v, 6) for v in ring_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={ring_joint.axis}",
    )
    ctx.check(
        "battery door is a prismatic slider",
        door_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={door_joint.articulation_type}",
    )
    ctx.check(
        "battery door slides along disc diameter",
        tuple(round(v, 6) for v in door_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={door_joint.axis}",
    )

    ctx.expect_gap(
        selector_ring,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="selector_ring",
        negative_elem="rounded_shell",
        name="selector ring rests on top face",
    )
    ctx.expect_overlap(
        selector_ring,
        body,
        axes="xy",
        min_overlap=0.045,
        elem_a="selector_ring",
        elem_b="rounded_shell",
        name="selector ring is centered on puck",
    )

    ctx.expect_gap(
        body,
        battery_door,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="rounded_shell",
        negative_elem="door_panel",
        name="battery door sits just below underside",
    )
    ctx.expect_overlap(
        battery_door,
        body,
        axes="xy",
        min_overlap=0.025,
        elem_a="door_panel",
        elem_b="rounded_shell",
        name="battery door is within the round underside footprint",
    )

    rest_pos = ctx.part_world_position(battery_door)
    with ctx.pose({door_joint: 0.030}):
        extended_pos = ctx.part_world_position(battery_door)
        ctx.expect_overlap(
            battery_door,
            body,
            axes="y",
            min_overlap=0.025,
            elem_a="door_panel",
            elem_b="rounded_shell",
            name="sliding door remains guided across its width",
        )
        ctx.expect_overlap(
            battery_door,
            body,
            axes="x",
            min_overlap=0.025,
            elem_a="door_panel",
            elem_b="rounded_shell",
            name="sliding door remains partially captured",
        )
    ctx.check(
        "battery door moves outward along diameter",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.025
        and abs(extended_pos[1] - rest_pos[1]) < 0.001,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ring_rest = ctx.part_world_position(selector_ring)
    with ctx.pose({ring_joint: math.pi / 2.0}):
        ring_rotated = ctx.part_world_position(selector_ring)
    ctx.check(
        "selector ring spins about its own center",
        ring_rest is not None
        and ring_rotated is not None
        and all(abs(a - b) < 0.0005 for a, b in zip(ring_rest, ring_rotated)),
        details=f"rest={ring_rest}, rotated={ring_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
