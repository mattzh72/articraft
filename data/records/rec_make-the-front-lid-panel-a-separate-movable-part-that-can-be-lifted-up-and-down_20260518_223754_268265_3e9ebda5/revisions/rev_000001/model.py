from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LENGTH = 0.170
BODY_WIDTH = 0.120
BODY_HEIGHT = 0.250


def _box_part(part, name: str, size: tuple[float, float, float], xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _ring_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    ring = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    return mesh_from_cadquery(ring, name, tolerance=0.0007, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_water_filter_jug")

    clear_blue = model.material("clear_blue_plastic", rgba=(0.56, 0.83, 1.00, 0.48))
    darker_blue = model.material("soft_blue_handle", rgba=(0.32, 0.62, 0.86, 0.82))
    white = model.material("white_filter_plastic", rgba=(0.94, 0.94, 0.90, 1.0))
    charcoal = model.material("charcoal_cap_and_indicator", rgba=(0.08, 0.09, 0.10, 1.0))
    gasket = model.material("pale_socket_gasket", rgba=(0.80, 0.92, 0.96, 0.72))

    body = model.part("body")
    wall = 0.006
    _box_part(body, "base_floor", (BODY_LENGTH, BODY_WIDTH, 0.010), (0.0, 0.0, 0.005), clear_blue)
    _box_part(body, "front_wall", (wall, BODY_WIDTH, BODY_HEIGHT), (BODY_LENGTH / 2 - wall / 2, 0.0, BODY_HEIGHT / 2), clear_blue)
    _box_part(body, "rear_wall", (wall, BODY_WIDTH, BODY_HEIGHT), (-BODY_LENGTH / 2 + wall / 2, 0.0, BODY_HEIGHT / 2), clear_blue)
    _box_part(body, "left_wall", (BODY_LENGTH, wall, BODY_HEIGHT), (0.0, BODY_WIDTH / 2 - wall / 2, BODY_HEIGHT / 2), clear_blue)
    _box_part(body, "right_wall", (BODY_LENGTH, wall, BODY_HEIGHT), (0.0, -BODY_WIDTH / 2 + wall / 2, BODY_HEIGHT / 2), clear_blue)
    _box_part(body, "front_pour_spout", (0.035, 0.055, 0.020), (0.102, 0.0, 0.238), clear_blue)
    _box_part(body, "spout_lip", (0.050, 0.070, 0.005), (0.108, 0.0, 0.252), clear_blue)
    _box_part(body, "stable_foot", (0.140, 0.094, 0.008), (0.0, 0.0, 0.004), darker_blue)

    handle = model.part("handle")
    _box_part(handle, "upper_mount", (0.065, 0.020, 0.016), (-0.033, 0.0, 0.050), darker_blue)
    _box_part(handle, "lower_mount", (0.065, 0.020, 0.016), (-0.033, 0.0, -0.050), darker_blue)
    _box_part(handle, "rear_grip", (0.020, 0.022, 0.120), (-0.070, 0.0, 0.0), darker_blue)
    _box_part(handle, "top_rounding", (0.033, 0.022, 0.020), (-0.054, 0.0, 0.058), darker_blue)
    _box_part(handle, "bottom_rounding", (0.033, 0.022, 0.020), (-0.054, 0.0, -0.058), darker_blue)

    upper_reservoir = model.part("upper_reservoir")
    res_len = 0.132
    res_wid = 0.084
    res_h = 0.100
    res_h = 0.106
    _box_part(upper_reservoir, "reservoir_floor_front_rail", (res_len, 0.014, 0.006), (0.0, 0.035, 0.003), clear_blue)
    _box_part(upper_reservoir, "reservoir_floor_rear_rail", (res_len, 0.014, 0.006), (0.0, -0.035, 0.003), clear_blue)
    _box_part(upper_reservoir, "reservoir_floor_right_rail", (0.028, 0.056, 0.006), (0.052, 0.0, 0.003), clear_blue)
    _box_part(upper_reservoir, "reservoir_floor_left_rail", (0.028, 0.056, 0.006), (-0.052, 0.0, 0.003), clear_blue)
    _box_part(upper_reservoir, "reservoir_front_wall", (0.005, res_wid, res_h), (res_len / 2 - 0.0025, 0.0, res_h / 2), clear_blue)
    _box_part(upper_reservoir, "reservoir_rear_wall", (0.005, res_wid, res_h), (-res_len / 2 + 0.0025, 0.0, res_h / 2), clear_blue)
    _box_part(upper_reservoir, "reservoir_left_wall", (res_len, 0.005, res_h), (0.0, res_wid / 2 - 0.0025, res_h / 2), clear_blue)
    _box_part(upper_reservoir, "reservoir_right_wall", (res_len, 0.005, res_h), (0.0, -res_wid / 2 + 0.0025, res_h / 2), clear_blue)
    _box_part(upper_reservoir, "front_seating_flange", (0.030, 0.112, 0.006), (0.077, 0.0, 0.109), clear_blue)
    _box_part(upper_reservoir, "rear_seating_flange", (0.030, 0.112, 0.006), (-0.077, 0.0, 0.109), clear_blue)
    _box_part(upper_reservoir, "left_seating_flange", (0.132, 0.020, 0.006), (0.0, 0.052, 0.109), clear_blue)
    _box_part(upper_reservoir, "right_seating_flange", (0.132, 0.020, 0.006), (0.0, -0.052, 0.109), clear_blue)
    upper_reservoir.visual(_ring_mesh(0.040, 0.031, 0.006, "filter_socket_ring"), origin=Origin(xyz=(0.0, 0.0, 0.006)), material=gasket, name="filter_socket_ring")

    filter_cartridge = model.part("filter_cartridge")
    filter_cartridge.visual(Cylinder(radius=0.026, length=0.105), origin=Origin(xyz=(0.0, 0.0, -0.037)), material=white, name="white_cartridge")
    filter_cartridge.visual(Cylinder(radius=0.034, length=0.010), origin=Origin(xyz=(0.0, 0.0, 0.007)), material=charcoal, name="dark_top_cap")
    filter_cartridge.visual(Box((0.038, 0.006, 0.045)), origin=Origin(xyz=(0.0, 0.023, -0.030)), material=gasket, name="vertical_filter_rib")
    filter_cartridge.visual(Box((0.038, 0.006, 0.045)), origin=Origin(xyz=(0.0, -0.023, -0.030)), material=gasket, name="opposite_filter_rib")

    lid = model.part("lid")
    _box_part(lid, "front_downturned_rim", (0.006, 0.108, 0.010), (0.084, 0.0, -0.003), clear_blue)
    _box_part(lid, "side_lid_rim_left", (0.158, 0.004, 0.010), (0.005, 0.055, -0.003), clear_blue)
    _box_part(lid, "side_lid_rim_right", (0.158, 0.004, 0.010), (0.005, -0.055, -0.003), clear_blue)
    _box_part(lid, "rear_hinge_bridge", (0.010, 0.108, 0.004), (-0.074, 0.0, 0.002), clear_blue)
    lid.visual(Cylinder(radius=0.004, length=0.104), origin=Origin(xyz=(-0.074, 0.0, 0.008), rpy=(math.pi / 2, 0.0, 0.0)), material=charcoal, name="rear_hinge_pin")

    front_lid_panel = model.part("front_lid_panel")
    _box_part(front_lid_panel, "lift_panel", (0.076, 0.096, 0.008), (0.0, 0.0, 0.0), clear_blue)
    _box_part(front_lid_panel, "left_slide_lip", (0.060, 0.005, 0.004), (0.000, 0.0505, -0.006), clear_blue)
    _box_part(front_lid_panel, "right_slide_lip", (0.060, 0.005, 0.004), (0.000, -0.0505, -0.006), clear_blue)
    _box_part(front_lid_panel, "status_indicator", (0.038, 0.022, 0.003), (0.006, 0.0, 0.004), charcoal)

    refill_flap = model.part("refill_flap")
    _box_part(refill_flap, "flap_panel", (0.070, 0.102, 0.007), (0.041, 0.0, 0.000), clear_blue)
    _box_part(refill_flap, "finger_tab", (0.020, 0.035, 0.006), (0.070, 0.0, 0.0065), charcoal)
    refill_flap.visual(Cylinder(radius=0.003, length=0.096), origin=Origin(xyz=(0.014, 0.0, 0.003), rpy=(math.pi / 2, 0.0, 0.0)), material=charcoal, name="flap_knuckle")

    model.articulation("body_to_handle", ArticulationType.FIXED, parent=body, child=handle, origin=Origin(xyz=(-0.0845, 0.0, 0.135)))
    model.articulation(
        "body_to_upper_reservoir",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_reservoir,
        origin=Origin(xyz=(0.0, 0.0, 0.144)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.100, effort=12.0, velocity=0.20),
        motion_properties=MotionProperties(damping=0.1, friction=0.05),
    )
    model.articulation(
        "upper_reservoir_to_filter_cartridge",
        ArticulationType.PRISMATIC,
        parent=upper_reservoir,
        child=filter_cartridge,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.120, effort=8.0, velocity=0.15),
        motion_properties=MotionProperties(damping=0.1, friction=0.05),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.264)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.060, effort=6.0, velocity=0.20),
        motion_properties=MotionProperties(damping=0.08, friction=0.04),
    )
    model.articulation(
        "lid_to_refill_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=refill_flap,
        origin=Origin(xyz=(-0.075, 0.0, 0.008)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.radians(105.0), effort=2.0, velocity=2.0),
        motion_properties=MotionProperties(damping=0.04, friction=0.02),
    )
    model.articulation(
        "lid_to_front_lid_panel",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=front_lid_panel,
        origin=Origin(xyz=(0.039, 0.0, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.045, effort=3.0, velocity=0.18),
        motion_properties=MotionProperties(damping=0.05, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    upper_reservoir = object_model.get_part("upper_reservoir")
    filter_cartridge = object_model.get_part("filter_cartridge")
    lid = object_model.get_part("lid")
    front_lid_panel = object_model.get_part("front_lid_panel")
    refill_flap = object_model.get_part("refill_flap")
    handle = object_model.get_part("handle")

    reservoir_slide = object_model.get_articulation("body_to_upper_reservoir")
    filter_slide = object_model.get_articulation("upper_reservoir_to_filter_cartridge")
    lid_slide = object_model.get_articulation("body_to_lid")
    flap_hinge = object_model.get_articulation("lid_to_refill_flap")
    front_panel_slide = object_model.get_articulation("lid_to_front_lid_panel")

    ctx.expect_within(upper_reservoir, body, axes="xy", margin=0.008, name="upper reservoir basin nests inside body footprint with a small seating flange")
    ctx.expect_within(filter_cartridge, upper_reservoir, axes="xy", margin=0.0, name="filter remains centered in reservoir socket")
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.085, name="lid covers the pitcher opening")
    ctx.expect_overlap(front_lid_panel, body, axes="xy", min_overlap=0.070, name="liftable front lid panel covers front opening")
    ctx.expect_origin_distance(handle, body, axes="x", min_dist=0.075, max_dist=0.100, name="handle sits at rear of jug")
    ctx.expect_origin_gap(refill_flap, body, axis="z", min_gap=0.005, name="refill flap rests above the open body")

    ctx.check("seven semantic links", len(object_model.parts) == 7, details=f"parts={[p.name for p in object_model.parts]}")
    ctx.check("reservoir lift travel is ten centimeters", reservoir_slide.motion_limits.upper == 0.100)
    ctx.check("filter cartridge lift travel is twelve centimeters", filter_slide.motion_limits.upper == 0.120)
    ctx.check("lid lift travel is six centimeters", lid_slide.motion_limits.upper == 0.060)
    ctx.check("refill flap opens past a right angle", flap_hinge.motion_limits.upper > math.radians(100.0))
    ctx.check("front lid panel lifts vertically", front_panel_slide.motion_limits.upper == 0.045)

    rest_flap_pos = ctx.part_world_position(refill_flap)
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        open_flap_pos = ctx.part_world_position(refill_flap)
    ctx.check(
        "refill flap hinge motion is available",
        rest_flap_pos is not None and open_flap_pos is not None,
        details=f"rest={rest_flap_pos}, open={open_flap_pos}",
    )

    rest_panel_pos = ctx.part_world_position(front_lid_panel)
    with ctx.pose({front_panel_slide: front_panel_slide.motion_limits.upper}):
        lifted_panel_pos = ctx.part_world_position(front_lid_panel)
    ctx.check(
        "front lid panel moves upward independently of rims",
        rest_panel_pos is not None
        and lifted_panel_pos is not None
        and lifted_panel_pos[2] > rest_panel_pos[2] + 0.040,
        details=f"rest={rest_panel_pos}, lifted={lifted_panel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
