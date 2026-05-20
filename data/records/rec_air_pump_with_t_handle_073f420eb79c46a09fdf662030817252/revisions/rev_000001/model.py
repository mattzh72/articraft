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
    mesh_from_geometry,
    mesh_from_cadquery,
    tube_from_spline_points,
)


def _barrel_shell_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    shell = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )
    return mesh_from_cadquery(shell, name)


def _guide_ring_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    ring = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )
    return mesh_from_cadquery(ring, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_head_floor_pump")

    body_red = model.material("body_red", rgba=(0.78, 0.17, 0.12, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    dial_white = model.material("dial_white", rgba=(0.95, 0.95, 0.93, 1.0))
    bezel = model.material("bezel", rgba=(0.84, 0.85, 0.87, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.300, 0.065, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_plastic,
        name="foot_bar",
    )
    body.visual(
        Box((0.090, 0.095, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=body_red,
        name="spine",
    )
    body.visual(
        Box((0.088, 0.040, 0.005)),
        origin=Origin(xyz=(-0.088, 0.0, 0.019)),
        material=rubber,
        name="pad_0",
    )
    body.visual(
        Box((0.088, 0.040, 0.005)),
        origin=Origin(xyz=(0.088, 0.0, 0.019)),
        material=rubber,
        name="pad_1",
    )
    body.visual(
        Cylinder(radius=0.046, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=body_red,
        name="barrel_collar",
    )
    body.visual(
        _barrel_shell_mesh(
            "pump_barrel_shell",
            outer_radius=0.0375,
            inner_radius=0.0325,
            length=0.440,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=body_red,
        name="barrel_shell",
    )
    body.visual(
        _guide_ring_mesh(
            "pump_top_guide",
            outer_radius=0.0415,
            inner_radius=0.0105,
            length=0.024,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.482)),
        material=dark_plastic,
        name="top_guide",
    )
    body.visual(
        Box((0.040, 0.050, 0.060)),
        origin=Origin(xyz=(0.0, 0.030, 0.060)),
        material=body_red,
        name="gauge_mount",
    )
    body.visual(
        Cylinder(radius=0.046, length=0.028),
        origin=Origin(xyz=(0.0, 0.057, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="gauge_body",
    )
    body.visual(
        Cylinder(radius=0.050, length=0.006),
        origin=Origin(xyz=(0.0, 0.069, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bezel,
        name="gauge_bezel",
    )
    body.visual(
        Cylinder(radius=0.041, length=0.004),
        origin=Origin(xyz=(0.0, 0.067, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_white,
        name="gauge_face",
    )
    body.visual(
        Cylinder(radius=0.0095, length=0.022),
        origin=Origin(xyz=(0.044, 0.018, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="hose_port",
    )
    body.visual(
        _guide_ring_mesh(
            "gauge_button_collar",
            outer_radius=0.0085,
            inner_radius=0.0055,
            length=0.003,
        ),
        origin=Origin(xyz=(0.018, 0.056, 0.160)),
        material=bezel,
        name="button_collar",
    )

    pump_rod = model.part("pump_rod")
    pump_rod.visual(
        Cylinder(radius=0.008, length=0.255),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=steel,
        name="rod_shaft",
    )
    pump_rod.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_plastic,
        name="guide_nut",
    )
    pump_rod.visual(
        Cylinder(radius=0.015, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.248)),
        material=dark_plastic,
        name="handle_hub",
    )
    pump_rod.visual(
        Cylinder(radius=0.011, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.276), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="handle_bar",
    )
    pump_rod.visual(
        Cylinder(radius=0.016, length=0.070),
        origin=Origin(xyz=(-0.102, 0.0, 0.276), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="grip_0",
    )
    pump_rod.visual(
        Cylinder(radius=0.016, length=0.070),
        origin=Origin(xyz=(0.102, 0.0, 0.276), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="grip_1",
    )

    model.articulation(
        "body_to_pump_rod",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pump_rod,
        origin=Origin(xyz=(0.0, 0.0, 0.506)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.8, lower=0.0, upper=0.160),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Cylinder(radius=0.0075, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=body_red,
        name="button_cap",
    )

    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(0.018, 0.056, 0.163)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=0.0015),
    )

    hose = model.part("hose")
    hose.visual(
        Cylinder(radius=0.0095, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="hose_cuff",
    )
    hose.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.015, 0.0, 0.0),
                    (0.070, 0.008, 0.015),
                    (0.128, 0.020, 0.115),
                    (0.112, 0.016, 0.255),
                    (0.076, 0.012, 0.364),
                    (0.056, 0.010, 0.420),
                ],
                radius=0.0065,
                samples_per_segment=18,
                radial_segments=18,
            ),
            "floor_pump_hose",
        ),
        material=rubber,
        name="hose_tube",
    )
    hose.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(xyz=(0.073, 0.010, 0.420), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="head_body",
    )
    hose.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.050, 0.010, 0.420), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="head_port_large",
    )
    hose.visual(
        Cylinder(radius=0.0095, length=0.010),
        origin=Origin(xyz=(0.095, 0.010, 0.420), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="head_port_small",
    )
    hose.visual(
        Box((0.005, 0.008, 0.014)),
        origin=Origin(xyz=(0.063, 0.018, 0.427)),
        material=dark_plastic,
        name="lever_ear_0",
    )
    hose.visual(
        Box((0.005, 0.008, 0.014)),
        origin=Origin(xyz=(0.083, 0.018, 0.427)),
        material=dark_plastic,
        name="lever_ear_1",
    )

    model.articulation(
        "body_to_hose",
        ArticulationType.FIXED,
        parent=body,
        child=hose,
        origin=Origin(xyz=(0.055, 0.018, 0.080)),
    )

    clamp_lever = model.part("clamp_lever")
    clamp_lever.visual(
        Cylinder(radius=0.0035, length=0.015),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lever_barrel",
    )
    clamp_lever.visual(
        Box((0.010, 0.036, 0.006)),
        origin=Origin(xyz=(0.0, 0.019, 0.002)),
        material=bezel,
        name="lever_arm",
    )
    clamp_lever.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.0, 0.040, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bezel,
        name="lever_tip",
    )

    model.articulation(
        "hose_to_clamp_lever",
        ArticulationType.REVOLUTE,
        parent=hose,
        child=clamp_lever,
        origin=Origin(xyz=(0.073, 0.018, 0.433)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    hose = object_model.get_part("hose")
    clamp_lever = object_model.get_part("clamp_lever")
    pump_rod = object_model.get_part("pump_rod")
    release_button = object_model.get_part("release_button")
    button_joint = object_model.get_articulation("body_to_release_button")
    lever_joint = object_model.get_articulation("hose_to_clamp_lever")
    rod_joint = object_model.get_articulation("body_to_pump_rod")

    ctx.expect_overlap(
        pump_rod,
        body,
        axes="xy",
        elem_a="guide_nut",
        elem_b="top_guide",
        min_overlap=0.015,
        name="rod stays centered over the guide",
    )
    ctx.expect_contact(
        pump_rod,
        body,
        elem_a="guide_nut",
        elem_b="top_guide",
        name="rod guide nut rests on the barrel guide at rest",
    )
    ctx.expect_contact(
        hose,
        body,
        elem_a="hose_cuff",
        elem_b="hose_port",
        name="hose is mounted to the pump body at the port",
    )
    ctx.expect_contact(
        release_button,
        body,
        elem_a="button_cap",
        elem_b="button_collar",
        name="release button sits on its gauge collar at rest",
    )
    ctx.expect_contact(
        clamp_lever,
        hose,
        elem_a="lever_barrel",
        elem_b="lever_ear_0",
        name="clamp lever is supported by one hinge ear at rest",
    )
    ctx.expect_contact(
        clamp_lever,
        hose,
        elem_a="lever_barrel",
        elem_b="lever_ear_1",
        name="clamp lever is supported by the other hinge ear at rest",
    )

    rod_limits = rod_joint.motion_limits
    if rod_limits is not None and rod_limits.upper is not None:
        rest_pos = ctx.part_world_position(pump_rod)
        with ctx.pose({rod_joint: rod_limits.upper}):
            extended_pos = ctx.part_world_position(pump_rod)
        ctx.check(
            "pump rod extends upward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.10,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    button_limits = button_joint.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        rest_pos = ctx.part_world_position(release_button)
        with ctx.pose({button_joint: button_limits.upper}):
            pressed_pos = ctx.part_world_position(release_button)
        ctx.check(
            "release button presses downward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    lever_limits = lever_joint.motion_limits
    if lever_limits is not None and lever_limits.upper is not None:
        rest_aabb = ctx.part_world_aabb(clamp_lever)
        with ctx.pose({lever_joint: lever_limits.upper}):
            open_aabb = ctx.part_world_aabb(clamp_lever)
        ctx.check(
            "clamp lever flips upward",
            rest_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > rest_aabb[1][2] + 0.018,
            details=f"rest={rest_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
