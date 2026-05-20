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


def _rounded_box_mesh(sx: float, sy: float, sz: float, radius: float, name: str):
    """Small filleted plastic housing mesh, authored in meters."""
    shape = cq.Workplane("XY").box(sx, sy, sz)
    try:
        shape = shape.edges("|Z").fillet(min(radius, sx * 0.20, sy * 0.20))
    except Exception:
        # Very thin panels can be below the robust OpenCascade fillet limit.
        pass
    return mesh_from_cadquery(shape, name, tolerance=0.0005, angular_tolerance=0.08)


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_travel_flash")

    satin_black = model.material("satin_black", rgba=(0.015, 0.015, 0.014, 1.0))
    soft_black = model.material("soft_black", rgba=(0.045, 0.045, 0.043, 1.0))
    seam_black = model.material("seam_black", rgba=(0.002, 0.002, 0.002, 1.0))
    charcoal = model.material("charcoal_detail", rgba=(0.10, 0.10, 0.095, 1.0))
    smoked = model.material("smoked_front", rgba=(0.035, 0.040, 0.045, 0.82))
    diffuser = model.material("milky_diffuser", rgba=(0.86, 0.90, 0.88, 0.72))
    warm_lens = model.material("af_assist_red", rgba=(0.55, 0.035, 0.02, 0.86))
    metal = model.material("brushed_shoe_metal", rgba=(0.62, 0.62, 0.58, 1.0))

    body = model.part("body_shell")
    body.visual(
        _rounded_box_mesh(0.030, 0.044, 0.074, 0.0035, "body_case_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=satin_black,
        name="case_shell",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0855)),
        material=soft_black,
        name="top_collar",
    )
    body.visual(
        Box((0.0014, 0.026, 0.018)),
        origin=Origin(xyz=(0.0157, 0.0, 0.056)),
        material=smoked,
        name="front_window",
    )
    body.visual(
        Box((0.0016, 0.010, 0.008)),
        origin=Origin(xyz=(0.0158, 0.0, 0.031)),
        material=warm_lens,
        name="af_assist_window",
    )
    body.visual(
        Box((0.030, 0.038, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.020, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=soft_black,
        name="shoe_foot",
    )
    body.visual(
        Box((0.030, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, 0.0205, 0.0045)),
        material=metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.030, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, -0.0205, 0.0045)),
        material=metal,
        name="shoe_rail_1",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        _rounded_box_mesh(0.025, 0.0024, 0.050, 0.0012, "battery_door_panel"),
        origin=Origin(xyz=(0.0125, 0.0012, 0.0)),
        material=soft_black,
        name="door_panel",
    )
    battery_door.visual(
        Cylinder(radius=0.0020, length=0.046),
        origin=Origin(xyz=(0.0, 0.0043, 0.0)),
        material=charcoal,
        name="door_hinge_barrel",
    )
    battery_door.visual(
        Box((0.022, 0.0005, 0.0010)),
        origin=Origin(xyz=(0.0125, 0.00265, 0.0240)),
        material=seam_black,
        name="door_seam_top",
    )
    battery_door.visual(
        Box((0.022, 0.0005, 0.0010)),
        origin=Origin(xyz=(0.0125, 0.00265, -0.0240)),
        material=seam_black,
        name="door_seam_bottom",
    )
    battery_door.visual(
        Box((0.0010, 0.0005, 0.044)),
        origin=Origin(xyz=(0.0010, 0.00265, 0.0)),
        material=seam_black,
        name="door_seam_hinge",
    )
    battery_door.visual(
        Box((0.0010, 0.0005, 0.044)),
        origin=Origin(xyz=(0.0240, 0.00265, 0.0)),
        material=seam_black,
        name="door_seam_latch",
    )
    battery_door.visual(
        Box((0.0030, 0.0009, 0.014)),
        origin=Origin(xyz=(0.0222, 0.0030, -0.006)),
        material=charcoal,
        name="finger_latch",
    )

    swivel = model.part("swivel_cradle")
    swivel.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=soft_black,
        name="turntable",
    )
    swivel.visual(
        Box((0.016, 0.008, 0.031)),
        origin=Origin(xyz=(0.0, 0.036, 0.0210)),
        material=soft_black,
        name="cradle_cheek_0",
    )
    swivel.visual(
        Box((0.012, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.026, 0.0070)),
        material=soft_black,
        name="cradle_shoulder_0",
    )
    swivel.visual(
        Box((0.016, 0.008, 0.031)),
        origin=Origin(xyz=(0.0, -0.036, 0.0210)),
        material=soft_black,
        name="cradle_cheek_1",
    )
    swivel.visual(
        Box((0.012, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -0.026, 0.0070)),
        material=soft_black,
        name="cradle_shoulder_1",
    )
    swivel.visual(
        Cylinder(radius=0.0032, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="tilt_pin",
    )

    head = model.part("flash_head")
    head.visual(
        _rounded_box_mesh(0.050, 0.060, 0.032, 0.0040, "flash_head_shell"),
        origin=Origin(xyz=(0.025, 0.0, 0.004)),
        material=satin_black,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.0052, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="tilt_barrel",
    )
    head.visual(
        Box((0.0030, 0.052, 0.023)),
        origin=Origin(xyz=(0.0515, 0.0, 0.005)),
        material=diffuser,
        name="diffuser_window",
    )
    head.visual(
        Box((0.0010, 0.046, 0.0010)),
        origin=Origin(xyz=(0.0531, 0.0, -0.002)),
        material=smoked,
        name="fresnel_line_0",
    )
    head.visual(
        Box((0.0010, 0.046, 0.0010)),
        origin=Origin(xyz=(0.0531, 0.0, 0.005)),
        material=smoked,
        name="fresnel_line_1",
    )
    head.visual(
        Box((0.0010, 0.046, 0.0010)),
        origin=Origin(xyz=(0.0531, 0.0, 0.012)),
        material=smoked,
        name="fresnel_line_2",
    )
    head.visual(
        Box((0.032, 0.050, 0.0020)),
        origin=Origin(xyz=(0.030, 0.0, 0.0208)),
        material=charcoal,
        name="bounce_card_slot",
    )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(-0.013, 0.022, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.18, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body_shell")
    door = object_model.get_part("battery_door")
    swivel = object_model.get_part("swivel_cradle")
    head = object_model.get_part("flash_head")
    door_hinge = object_model.get_articulation("body_to_battery_door")
    swivel_joint = object_model.get_articulation("body_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")

    ctx.allow_overlap(
        head,
        swivel,
        elem_a="tilt_barrel",
        elem_b="tilt_pin",
        reason="The tilt pin is intentionally captured inside the head hinge barrel.",
    )
    ctx.expect_gap(
        swivel,
        body,
        axis="z",
        max_gap=0.0008,
        max_penetration=0.000001,
        positive_elem="turntable",
        negative_elem="top_collar",
        name="swivel turntable sits on body collar",
    )
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.0010,
        max_penetration=0.0,
        positive_elem="door_panel",
        negative_elem="case_shell",
        name="battery door panel sits on side shell",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.012,
        positive_elem="head_shell",
        negative_elem="case_shell",
        name="head clears narrow body top",
    )
    ctx.expect_overlap(
        head,
        swivel,
        axes="z",
        min_overlap=0.005,
        elem_a="tilt_barrel",
        elem_b="cradle_cheek_0",
        name="tilt barrel aligns with cradle cheeks",
    )
    ctx.expect_overlap(
        head,
        swivel,
        axes="y",
        min_overlap=0.050,
        elem_a="tilt_barrel",
        elem_b="tilt_pin",
        name="captured tilt pin runs through barrel",
    )

    rest_head = _center_from_aabb(ctx.part_element_world_aabb(head, elem="head_shell"))
    with ctx.pose({tilt_joint: 1.10}):
        tilted_head = _center_from_aabb(ctx.part_element_world_aabb(head, elem="head_shell"))
    ctx.check(
        "tilt hinge raises the bounce head",
        rest_head is not None
        and tilted_head is not None
        and tilted_head[2] > rest_head[2] + 0.016,
        details=f"rest={rest_head}, tilted={tilted_head}",
    )

    with ctx.pose({swivel_joint: 0.90}):
        swiveled_head = _center_from_aabb(ctx.part_element_world_aabb(head, elem="head_shell"))
    ctx.check(
        "vertical swivel slews head sideways",
        rest_head is not None
        and swiveled_head is not None
        and swiveled_head[1] > rest_head[1] + 0.015,
        details=f"rest={rest_head}, swiveled={swiveled_head}",
    )

    rest_door = _center_from_aabb(ctx.part_element_world_aabb(door, elem="door_panel"))
    with ctx.pose({door_hinge: 1.05}):
        opened_door = _center_from_aabb(ctx.part_element_world_aabb(door, elem="door_panel"))
    ctx.check(
        "side battery door swings outward",
        rest_door is not None
        and opened_door is not None
        and opened_door[1] > rest_door[1] + 0.007,
        details=f"rest={rest_door}, opened={opened_door}",
    )

    return ctx.report()


object_model = build_object_model()
