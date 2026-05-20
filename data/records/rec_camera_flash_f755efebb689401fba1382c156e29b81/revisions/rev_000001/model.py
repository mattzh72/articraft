from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    """Small CadQuery helper for camera-plastic rounded rectangular housings."""
    x, y, z = size
    r = min(radius, x * 0.18, y * 0.18, z * 0.18)
    solid = cq.Workplane("XY").box(x, y, z).edges().fillet(r)
    return mesh_from_cadquery(solid, name, tolerance=0.0006, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_camera_flash")

    black = model.material("satin_black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    charcoal = model.material("charcoal_rubber", rgba=(0.055, 0.055, 0.052, 1.0))
    seam = model.material("shadow_seam", rgba=(0.004, 0.004, 0.004, 1.0))
    diffuser = model.material("milky_diffuser", rgba=(0.86, 0.90, 0.88, 0.72))
    glass = model.material("smoked_lcd", rgba=(0.02, 0.05, 0.075, 1.0))
    metal = model.material("brushed_hotshoe_metal", rgba=(0.48, 0.46, 0.42, 1.0))
    white = model.material("white_marking", rgba=(0.85, 0.86, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_box_mesh((0.055, 0.080, 0.145), 0.006, "body_shell"),
        origin=Origin(xyz=(-0.005, 0.0, 0.085)),
        material=black,
        name="body_shell",
    )
    # Broad DSLR hot-shoe foot with side rails and a riser that ties it into the shell.
    body.visual(
        Box((0.070, 0.052, 0.010)),
        origin=Origin(xyz=(-0.004, 0.0, 0.006)),
        material=metal,
        name="mounting_foot",
    )
    body.visual(
        Box((0.076, 0.008, 0.005)),
        origin=Origin(xyz=(-0.004, 0.030, 0.004)),
        material=metal,
        name="foot_rail_0",
    )
    body.visual(
        Box((0.076, 0.008, 0.005)),
        origin=Origin(xyz=(-0.004, -0.030, 0.004)),
        material=metal,
        name="foot_rail_1",
    )
    body.visual(
        Box((0.045, 0.038, 0.010)),
        origin=Origin(xyz=(-0.004, 0.0, 0.014)),
        material=black,
        name="foot_riser",
    )
    # Rear control face: a display and a subtle molded panel, with the main dial
    # articulated as its own movable part below.
    body.visual(
        Box((0.002, 0.050, 0.034)),
        origin=Origin(xyz=(-0.0327, 0.0, 0.123)),
        material=seam,
        name="rear_control_recess",
    )
    body.visual(
        Box((0.003, 0.039, 0.022)),
        origin=Origin(xyz=(-0.0336, 0.0, 0.124)),
        material=glass,
        name="rear_lcd",
    )
    body.visual(
        Box((0.0015, 0.028, 0.003)),
        origin=Origin(xyz=(-0.0324, 0.0, 0.148)),
        material=white,
        name="brand_mark",
    )
    # Top turntable socket for the swivel joint.
    body.visual(
        Cylinder(radius=0.021, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.1625)),
        material=charcoal,
        name="swivel_plinth",
    )
    # Battery-door seam and fixed side hinge leaf on one side of the body.
    body.visual(
        Box((0.052, 0.0015, 0.100)),
        origin=Origin(xyz=(-0.004, 0.0398, 0.085)),
        material=seam,
        name="door_shadow_gap",
    )
    body.visual(
        Box((0.006, 0.004, 0.104)),
        origin=Origin(xyz=(-0.032, 0.0418, 0.085)),
        material=charcoal,
        name="hinge_socket",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=charcoal,
        name="turntable",
    )
    yoke.visual(
        Box((0.032, 0.032, 0.030)),
        origin=Origin(xyz=(0.016, 0.0, 0.021)),
        material=charcoal,
        name="neck_block",
    )
    yoke.visual(
        Box((0.052, 0.114, 0.012)),
        origin=Origin(xyz=(0.036, 0.0, 0.031)),
        material=charcoal,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.024, 0.009, 0.076)),
        origin=Origin(xyz=(0.052, 0.056, 0.056)),
        material=charcoal,
        name="side_arm_0",
    )
    yoke.visual(
        Box((0.024, 0.009, 0.076)),
        origin=Origin(xyz=(0.052, -0.056, 0.056)),
        material=charcoal,
        name="side_arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.052, 0.0633, 0.062), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="outer_pivot_0",
    )
    yoke.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.052, -0.0633, 0.062), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="outer_pivot_1",
    )

    head = model.part("head")
    head.visual(
        _rounded_box_mesh((0.082, 0.090, 0.058), 0.005, "head_shell"),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=black,
        name="head_shell",
    )
    head.visual(
        Box((0.005, 0.081, 0.048)),
        origin=Origin(xyz=(0.088, 0.0, 0.0)),
        material=diffuser,
        name="front_diffuser",
    )
    head.visual(
        Box((0.006, 0.089, 0.006)),
        origin=Origin(xyz=(0.090, 0.0, 0.027)),
        material=charcoal,
        name="front_bezel_top",
    )
    head.visual(
        Box((0.006, 0.089, 0.006)),
        origin=Origin(xyz=(0.090, 0.0, -0.027)),
        material=charcoal,
        name="front_bezel_bottom",
    )
    head.visual(
        Box((0.006, 0.006, 0.052)),
        origin=Origin(xyz=(0.090, 0.044, 0.0)),
        material=charcoal,
        name="front_bezel_0",
    )
    head.visual(
        Box((0.006, 0.006, 0.052)),
        origin=Origin(xyz=(0.090, -0.044, 0.0)),
        material=charcoal,
        name="front_bezel_1",
    )
    # The pitch axle is readable on both sides through the yoke openings.
    head.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.0468, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="pivot_cap_0",
    )
    head.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, -0.0468, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="pivot_cap_1",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.050, 0.003, 0.095)),
        origin=Origin(xyz=(0.0275, 0.0, 0.0)),
        material=charcoal,
        name="door_panel",
    )
    for i, z in enumerate((-0.032, 0.0, 0.032)):
        battery_door.visual(
            Cylinder(radius=0.003, length=0.022),
            origin=Origin(xyz=(0.0, 0.003, z)),
            material=charcoal,
            name=f"hinge_knuckle_{i}",
        )
    battery_door.visual(
        Box((0.018, 0.001, 0.004)),
        origin=Origin(xyz=(0.037, 0.0019, -0.039)),
        material=seam,
        name="pull_notch",
    )

    control_dial = model.part("control_dial")
    control_dial.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="dial_disk",
    )
    control_dial.visual(
        Cylinder(radius=0.004, length=0.0044),
        origin=Origin(xyz=(0.0022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="dial_stem",
    )
    control_dial.visual(
        Box((0.001, 0.003, 0.013)),
        origin=Origin(xyz=(-0.0018, 0.0, 0.004)),
        material=white,
        name="dial_index",
    )

    swivel = model.articulation(
        "body_to_yoke",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.1685)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=-2.1, upper=2.1),
    )
    pitch = model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.052, 0.0, 0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.7, lower=-0.20, upper=1.25),
    )
    door_hinge = model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(-0.030, 0.0428, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=1.2, lower=0.0, upper=1.65),
    )
    model.articulation(
        "body_to_control_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=control_dial,
        origin=Origin(xyz=(-0.0365, 0.0, 0.091)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    # Keep named references alive for type-checkers/readability while avoiding
    # state names in part identifiers.
    assert swivel is not None and pitch is not None and door_hinge is not None
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("head")
    door = object_model.get_part("battery_door")
    dial = object_model.get_part("control_dial")
    swivel = object_model.get_articulation("body_to_yoke")
    pitch = object_model.get_articulation("yoke_to_head")
    door_hinge = object_model.get_articulation("body_to_battery_door")

    ctx.allow_overlap(
        body,
        dial,
        elem_a="body_shell",
        elem_b="dial_stem",
        reason="The small rear control dial stem is intentionally captured a fraction of a millimeter inside the plastic control face.",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="x",
        positive_elem="body_shell",
        negative_elem="dial_stem",
        max_gap=0.0003,
        max_penetration=0.0008,
        name="dial stem is shallowly seated in the rear face",
    )
    for knuckle in ("hinge_knuckle_0", "hinge_knuckle_1", "hinge_knuckle_2"):
        ctx.allow_overlap(
            body,
            door,
            elem_a="hinge_socket",
            elem_b=knuckle,
            reason="The battery-door hinge knuckle is intentionally captured against the fixed side hinge socket.",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="xy",
            elem_a="hinge_socket",
            elem_b=knuckle,
            min_overlap=0.0005,
            name=f"{knuckle} is retained by the side hinge socket",
        )
    ctx.expect_gap(
        yoke,
        body,
        axis="z",
        positive_elem="turntable",
        negative_elem="swivel_plinth",
        max_gap=0.0008,
        max_penetration=0.0002,
        name="swivel turntable seats on body plinth",
    )
    ctx.expect_gap(
        door,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="body_shell",
        min_gap=0.0003,
        max_gap=0.0025,
        name="battery door is a distinct side panel in the shell seam",
    )
    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_stem",
        elem_b="body_shell",
        contact_tol=0.0008,
        name="rear control dial stem reaches the body face",
    )
    ctx.expect_gap(
        yoke,
        head,
        axis="y",
        positive_elem="side_arm_0",
        negative_elem="head_shell",
        min_gap=0.004,
        max_gap=0.010,
        name="head clears one yoke arm",
    )
    ctx.expect_gap(
        head,
        yoke,
        axis="y",
        positive_elem="head_shell",
        negative_elem="side_arm_1",
        min_gap=0.004,
        max_gap=0.010,
        name="head clears opposite yoke arm",
    )

    rest_diffuser = ctx.part_element_world_aabb(head, elem="front_diffuser")
    rest_bridge = ctx.part_element_world_aabb(yoke, elem="lower_bridge")
    rest_door = ctx.part_world_aabb(door)

    with ctx.pose({pitch: 0.9}):
        raised_diffuser = ctx.part_element_world_aabb(head, elem="front_diffuser")
    with ctx.pose({swivel: 0.8}):
        swiveled_bridge = ctx.part_element_world_aabb(yoke, elem="lower_bridge")
    with ctx.pose({door_hinge: 1.1}):
        opened_door = ctx.part_world_aabb(door)

    def _center_z(aabb):
        return None if aabb is None else 0.5 * (aabb[0][2] + aabb[1][2])

    def _center_y(aabb):
        return None if aabb is None else 0.5 * (aabb[0][1] + aabb[1][1])

    ctx.check(
        "head pitches upward about the visible yoke axis",
        rest_diffuser is not None
        and raised_diffuser is not None
        and _center_z(raised_diffuser) > _center_z(rest_diffuser) + 0.025,
        details=f"rest={rest_diffuser}, raised={raised_diffuser}",
    )
    ctx.check(
        "head assembly swivels around the vertical turntable",
        rest_bridge is not None
        and swiveled_bridge is not None
        and abs(_center_y(swiveled_bridge) - _center_y(rest_bridge)) > 0.015,
        details=f"rest={rest_bridge}, swiveled={swiveled_bridge}",
    )
    ctx.check(
        "battery door swings outward from side hinge",
        rest_door is not None and opened_door is not None and opened_door[1][1] > rest_door[1][1] + 0.018,
        details=f"rest={rest_door}, opened={opened_door}",
    )

    return ctx.report()


object_model = build_object_model()
