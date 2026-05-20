from __future__ import annotations

from math import pi, sin, cos

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_plate(outer_radius: float, inner_radius: float, thickness: float):
    """A flat annular plate lying in the YZ plane, with thickness along X."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness / 2.0, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_front_load_washer")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    darker_steel = model.material("dark_stainless", rgba=(0.38, 0.40, 0.39, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    dark_panel = model.material("dark_control_panel", rgba=(0.05, 0.055, 0.06, 1.0))
    blue_glass = model.material("blue_tinted_glass", rgba=(0.40, 0.62, 0.75, 0.42))
    white_print = model.material("white_print", rgba=(0.88, 0.90, 0.88, 1.0))

    body = model.part("body")

    # Stainless cabinet built from real sheet panels rather than a solid block,
    # leaving a true hollow volume for the rotating drum.
    body.visual(Box((0.68, 0.70, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.02)), material=stainless, name="base_pan")
    body.visual(Box((0.68, 0.70, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.9375)), material=stainless, name="top_panel")
    body.visual(Box((0.025, 0.70, 0.95)), origin=Origin(xyz=(-0.3275, 0.0, 0.475)), material=stainless, name="rear_panel")
    body.visual(Box((0.68, 0.025, 0.95)), origin=Origin(xyz=(0.0, 0.3375, 0.475)), material=stainless, name="side_panel_0")
    body.visual(Box((0.68, 0.025, 0.95)), origin=Origin(xyz=(0.0, -0.3375, 0.475)), material=stainless, name="side_panel_1")

    # Front sheet-metal frame around the porthole and the commercial control strip.
    body.visual(Box((0.03, 0.70, 0.20)), origin=Origin(xyz=(0.327, 0.0, 0.85)), material=stainless, name="control_band")
    body.visual(Box((0.03, 0.70, 0.18)), origin=Origin(xyz=(0.327, 0.0, 0.09)), material=stainless, name="toe_panel")
    body.visual(Box((0.03, 0.085, 0.57)), origin=Origin(xyz=(0.327, 0.31, 0.465)), material=stainless, name="front_stile_0")
    body.visual(Box((0.03, 0.085, 0.57)), origin=Origin(xyz=(0.327, -0.31, 0.465)), material=stainless, name="front_stile_1")
    body.visual(
        mesh_from_cadquery(_annular_plate(0.315, 0.252, 0.030), "body_port_ring"),
        origin=Origin(xyz=(0.350, 0.0, 0.49)),
        material=black_rubber,
        name="body_port_ring",
    )
    body.visual(Box((0.012, 0.18, 0.055)), origin=Origin(xyz=(0.348, 0.0, 0.785)), material=dark_panel, name="display_window")
    body.visual(Box((0.014, 0.13, 0.018)), origin=Origin(xyz=(0.348, 0.275, 0.842)), material=white_print, name="status_labels")

    # Fixed hinge mounting plates on the left side of the porthole.
    for idx, z in enumerate((0.33, 0.65)):
        body.visual(
            Box((0.030, 0.040, 0.160)),
            origin=Origin(xyz=(0.350, 0.345, z)),
            material=darker_steel,
            name=f"hinge_plate_{idx}",
        )
    body.visual(
        Cylinder(radius=0.078, length=0.060),
        origin=Origin(xyz=(-0.302, 0.0, 0.49), rpy=(0.0, pi / 2.0, 0.0)),
        material=darker_steel,
        name="rear_bearing_boss",
    )

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=0.245, length=0.50),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=darker_steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.215, length=0.012),
        origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="perforated_face",
    )
    drum.visual(
        Cylinder(radius=0.060, length=0.045),
        origin=Origin(xyz=(0.262, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=darker_steel,
        name="axle_hub",
    )
    drum.visual(
        Cylinder(radius=0.038, length=0.150),
        origin=Origin(xyz=(-0.275, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=darker_steel,
        name="drum_axle",
    )
    for idx, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        y = 0.095 * sin(angle)
        z = 0.095 * cos(angle)
        drum.visual(
            Box((0.040, 0.032, 0.220)),
            origin=Origin(xyz=(0.235, y, z), rpy=(angle, 0.0, 0.0)),
            material=stainless,
            name=f"drum_lifter_{idx}",
        )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.040, 0.0, 0.49)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=25.0),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_annular_plate(0.305, 0.192, 0.080), "door_ring"),
        origin=Origin(xyz=(0.065, -0.335, 0.0)),
        material=darker_steel,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.203, length=0.026),
        origin=Origin(xyz=(0.071, -0.335, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue_glass,
        name="glass_bowl",
    )
    door.visual(
        mesh_from_cadquery(_annular_plate(0.318, 0.292, 0.045), "door_outer_bezel"),
        origin=Origin(xyz=(0.105, -0.335, 0.0)),
        material=stainless,
        name="outer_bezel",
    )
    door.visual(Box((0.052, 0.042, 0.150)), origin=Origin(xyz=(0.117, -0.638, 0.0)), material=black_rubber, name="pull_handle")
    for idx, z in enumerate((-0.16, 0.16)):
        door.visual(
            Cylinder(radius=0.025, length=0.120),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=darker_steel,
            name=f"hinge_barrel_{idx}",
        )
        door.visual(
            Box((0.038, 0.095, 0.050)),
            origin=Origin(xyz=(0.020, -0.045, z)),
            material=darker_steel,
            name=f"hinge_leaf_{idx}",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.390, 0.335, 0.49)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.95),
        motion_properties=MotionProperties(damping=0.35, friction=0.08),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=darker_steel,
        name="knob_cap",
    )
    selector_knob.visual(Box((0.010, 0.008, 0.040)), origin=Origin(xyz=(0.044, 0.0, 0.026)), material=black_rubber, name="pointer_mark")
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.3415, -0.205, 0.850)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.03),
    )

    for idx, y in enumerate((0.095, 0.175)):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.020, 0.060, 0.034)),
            origin=Origin(xyz=(0.010, 0.0, 0.0)),
            material=dark_panel,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.3415, y, 0.850)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.15, lower=-0.006, upper=0.0),
            motion_properties=MotionProperties(damping=0.15, friction=0.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    knob = object_model.get_part("selector_knob")
    door_hinge = object_model.get_articulation("body_to_door")
    drum_spin = object_model.get_articulation("body_to_drum")
    knob_spin = object_model.get_articulation("body_to_selector_knob")

    ctx.allow_overlap(
        body,
        drum,
        elem_a="rear_bearing_boss",
        elem_b="drum_axle",
        reason="The drum axle is intentionally captured inside the rear bearing boss.",
    )
    ctx.expect_within(
        drum,
        body,
        axes="yz",
        inner_elem="drum_axle",
        outer_elem="rear_bearing_boss",
        margin=0.001,
        name="drum axle is centered in rear bearing",
    )
    ctx.expect_overlap(
        drum,
        body,
        axes="x",
        elem_a="drum_axle",
        elem_b="rear_bearing_boss",
        min_overlap=0.030,
        name="rear bearing captures drum axle",
    )
    ctx.expect_within(
        drum,
        body,
        axes="yz",
        inner_elem="drum_shell",
        outer_elem="body_port_ring",
        margin=0.010,
        name="drum centered behind porthole",
    )
    ctx.expect_gap(
        door,
        body,
        axis="x",
        positive_elem="door_ring",
        negative_elem="body_port_ring",
        min_gap=0.010,
        max_gap=0.080,
        name="closed door sits proud of gasket",
    )
    ctx.expect_overlap(door, body, axes="yz", elem_a="glass_bowl", elem_b="body_port_ring", min_overlap=0.30, name="porthole glass covers opening")
    ctx.expect_gap(knob, body, axis="x", positive_elem="knob_cap", negative_elem="control_band", max_penetration=0.001, max_gap=0.030, name="selector knob mounted on panel")

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    with ctx.pose({door_hinge: 1.20}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem="door_ring",
            negative_elem="body_port_ring",
            min_gap=0.015,
            name="door swings outward on left hinges",
        )
    ctx.check(
        "door hinge opens toward front",
        closed_aabb is not None and opened_aabb is not None and opened_aabb[1][0] > closed_aabb[1][0] + 0.05,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    before = ctx.part_world_position(drum)
    with ctx.pose({drum_spin: pi / 2.0, knob_spin: pi / 3.0}):
        after = ctx.part_world_position(drum)
        ctx.expect_overlap(drum, body, axes="yz", elem_a="drum_shell", elem_b="body_port_ring", min_overlap=0.45, name="spinning drum stays coaxial with door")
    ctx.check("drum axle is fixed while spinning", before == after, details=f"before={before}, after={after}")

    return ctx.report()


object_model = build_object_model()
