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


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A softly radiused appliance-casting box, centered at the origin."""
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate(center)
    )


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate(center)
    )


def _base_body_shape() -> cq.Workplane:
    base = _rounded_box((0.64, 0.34, 0.075), 0.035).translate((0.05, 0.0, 0.0375))

    rear_neck = _rounded_box((0.155, 0.205, 0.310), 0.030).translate((-0.215, 0.0, 0.230))
    forward_shoulder = _rounded_box((0.105, 0.190, 0.205), 0.025).translate((-0.125, 0.0, 0.228))
    crown = _rounded_box((0.100, 0.270, 0.050), 0.020).translate((-0.190, 0.0, 0.360))

    hinge_cheek_a = _rounded_box((0.090, 0.030, 0.082), 0.008).translate((-0.120, 0.140, 0.420))
    hinge_cheek_b = _rounded_box((0.090, 0.030, 0.082), 0.008).translate((-0.120, -0.140, 0.420))
    pin_cap_a = _cylinder_y(0.030, 0.024, (-0.120, 0.160, 0.420))
    pin_cap_b = _cylinder_y(0.030, 0.024, (-0.120, -0.160, 0.420))

    return (
        base.union(rear_neck)
        .union(forward_shoulder)
        .union(crown)
        .union(hinge_cheek_a)
        .union(hinge_cheek_b)
        .union(pin_cap_a)
        .union(pin_cap_b)
    )


def _slide_rail_shape() -> cq.Workplane:
    rail_a = cq.Workplane("XY").box(0.250, 0.024, 0.012).translate((0.090, 0.075, 0.081))
    rail_b = cq.Workplane("XY").box(0.250, 0.024, 0.012).translate((0.090, -0.075, 0.081))
    rear_bridge = cq.Workplane("XY").box(0.030, 0.174, 0.012).translate((-0.020, 0.0, 0.081))
    return rail_a.union(rail_b).union(rear_bridge)


def _bowl_shape() -> cq.Workplane:
    # Revolved metal cross-section: open at the top, real wall thickness, and an
    # integral foot ring that seats into the dark sliding carriage.
    profile = [
        (0.045, 0.000),
        (0.082, 0.000),
        (0.082, 0.018),
        (0.064, 0.030),
        (0.160, 0.210),
        (0.162, 0.222),
        (0.146, 0.222),
        (0.057, 0.043),
        (0.045, 0.020),
    ]
    return cq.Workplane("XZ").polyline(profile).close().revolve(
        360, axisStart=(0, 0, 0), axisEnd=(0, 0, 1)
    )


def _head_body_shape() -> cq.Workplane:
    housing = _rounded_box((0.340, 0.220, 0.145), 0.032).translate((0.180, 0.0, 0.016))
    front_motor_nose = _cylinder_x(0.073, 0.070, (0.350, 0.0, 0.016))
    hinge_barrel = _cylinder_y(0.027, 0.120, (0.0, 0.0, 0.0))
    top_brow = _rounded_box((0.240, 0.205, 0.028), 0.016).translate((0.165, 0.0, 0.100))
    return housing.union(front_motor_nose).union(hinge_barrel).union(top_brow)


def _beater_shape() -> cq.Workplane:
    stem = cq.Workplane("XY").cylinder(0.045, 0.011).translate((0.0, 0.0, -0.0225))
    spine = cq.Workplane("XY").box(0.018, 0.018, 0.165).translate((0.0, 0.0, -0.105))

    outer = [
        (-0.054, -0.165),
        (-0.040, -0.072),
        (0.000, -0.040),
        (0.040, -0.072),
        (0.054, -0.165),
        (0.000, -0.190),
    ]
    inner = [
        (-0.030, -0.151),
        (-0.021, -0.087),
        (0.000, -0.069),
        (0.021, -0.087),
        (0.030, -0.151),
        (0.000, -0.166),
    ]
    paddle = (
        cq.Workplane("XZ")
        .polyline(outer)
        .close()
        .extrude(0.018)
        .translate((0.0, -0.009, 0.0))
        .cut(
            cq.Workplane("XZ")
            .polyline(inner)
            .close()
            .extrude(0.024)
            .translate((0.0, -0.012, 0.0))
        )
    )
    bottom_blade = cq.Workplane("XY").box(0.076, 0.018, 0.014).translate((0.0, 0.0, -0.178))
    return stem.union(spine).union(paddle).union(bottom_blade)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dark_tilt_head_stand_mixer")

    dark = model.material("dark_graphite_enamel", rgba=(0.025, 0.023, 0.022, 1.0))
    black = model.material("satin_black_control", rgba=(0.006, 0.006, 0.006, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.74, 0.72, 0.66, 1.0))
    chrome = model.material("polished_drive_metal", rgba=(0.88, 0.86, 0.80, 1.0))
    red = model.material("red_lock_marker", rgba=(0.55, 0.020, 0.015, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "base_body", tolerance=0.0012),
        material=dark,
        name="body",
    )
    base.visual(
        mesh_from_cadquery(_slide_rail_shape(), "slide_rails", tolerance=0.0010),
        material=chrome,
        name="slide_rails",
    )
    base.visual(
        Box((0.060, 0.006, 0.030)),
        origin=Origin(xyz=(0.225, -0.173, 0.070)),
        material=black,
        name="speed_panel",
    )
    base.visual(
        Box((0.006, 0.090, 0.024)),
        origin=Origin(xyz=(0.367, 0.083, 0.070)),
        material=black,
        name="lock_slot",
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.240, 0.190, 0.016)),
        origin=Origin(xyz=(0.080, 0.0, 0.008)),
        material=dark,
        name="carriage_plate",
    )
    bowl_carriage.visual(
        mesh_from_cadquery(_bowl_shape(), "stainless_bowl", tolerance=0.0009),
        origin=Origin(xyz=(0.080, 0.0, 0.014)),
        material=steel,
        name="bowl_shell",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_body_shape(), "mixer_head", tolerance=0.0010),
        material=dark,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.036, length=0.050),
        origin=Origin(xyz=(0.230, 0.0, -0.075)),
        material=chrome,
        name="drive_socket",
    )
    head.visual(
        Box((0.012, 0.230, 0.018)),
        origin=Origin(xyz=(0.305, 0.0, 0.080)),
        material=chrome,
        name="front_trim",
    )

    beater = model.part("beater")
    beater.visual(
        mesh_from_cadquery(_beater_shape(), "flat_beater", tolerance=0.0008),
        material=chrome,
        name="beater_frame",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.058,
                0.030,
                body_style="skirted",
                top_diameter=0.042,
                edge_radius=0.0015,
                skirt=KnobSkirt(0.066, 0.006, flare=0.07, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=25.0),
                center=False,
            ),
            "speed_knob_cap",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_cap",
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.030, 0.040, 0.020)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=red,
        name="lock_tab",
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.030, 0.0, 0.087)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.060),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.120, 0.0, 0.420)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=0.65),
    )

    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.230, 0.0, -0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )

    model.articulation(
        "base_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(0.225, -0.176, 0.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=5.50),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(0.370, 0.083, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.025),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    knob = object_model.get_part("speed_knob")
    lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    beater_drive = object_model.get_articulation("head_to_beater")
    speed_joint = object_model.get_articulation("base_to_speed_knob")
    lock_slide = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "canonical articulations are present",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and beater_drive.articulation_type == ArticulationType.CONTINUOUS
        and speed_joint.articulation_type == ArticulationType.REVOLUTE
        and lock_slide.articulation_type == ArticulationType.PRISMATIC,
        details="Mixer should expose bowl slide, head tilt, beater drive, speed knob, and lock slider.",
    )

    ctx.check(
        "joint axes match prompt",
        bowl_slide.axis == (1.0, 0.0, 0.0)
        and head_tilt.axis == (0.0, -1.0, 0.0)
        and beater_drive.axis == (0.0, 0.0, 1.0)
        and speed_joint.axis == (0.0, -1.0, 0.0)
        and lock_slide.axis == (1.0, 0.0, 0.0),
        details=f"axes={[bowl_slide.axis, head_tilt.axis, beater_drive.axis, speed_joint.axis, lock_slide.axis]}",
    )

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="carriage_plate",
        negative_elem="slide_rails",
        max_gap=0.002,
        max_penetration=0.001,
        name="bowl carriage rides on base rails",
    )
    ctx.expect_overlap(
        bowl,
        base,
        axes="xy",
        elem_a="carriage_plate",
        elem_b="slide_rails",
        min_overlap=0.060,
        name="carriage stays over slide rails",
    )
    ctx.expect_gap(
        head,
        beater,
        axis="z",
        positive_elem="drive_socket",
        negative_elem="beater_frame",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="flat beater seats under drive socket",
    )
    ctx.expect_gap(
        base,
        knob,
        axis="y",
        positive_elem="speed_panel",
        negative_elem="knob_cap",
        max_gap=0.002,
        max_penetration=0.001,
        name="speed knob is mounted on base panel",
    )
    ctx.expect_gap(
        lock,
        base,
        axis="x",
        positive_elem="lock_tab",
        negative_elem="lock_slot",
        max_gap=0.002,
        max_penetration=0.001,
        name="head lock tab sits in front base slot",
    )

    bowl_rest = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: 0.060}):
        bowl_extended = ctx.part_world_position(bowl)
        ctx.expect_overlap(
            bowl,
            base,
            axes="xy",
            elem_a="carriage_plate",
            elem_b="slide_rails",
            min_overlap=0.050,
            name="extended bowl slide retains rail support",
        )
    ctx.check(
        "bowl slide moves forward",
        bowl_rest is not None and bowl_extended is not None and bowl_extended[0] > bowl_rest[0] + 0.055,
        details=f"rest={bowl_rest}, extended={bowl_extended}",
    )

    beater_rest = ctx.part_world_position(beater)
    with ctx.pose({head_tilt: 0.65}):
        beater_raised = ctx.part_world_position(beater)
    ctx.check(
        "head tilt lifts beater upward",
        beater_rest is not None and beater_raised is not None and beater_raised[2] > beater_rest[2] + 0.07,
        details=f"rest={beater_rest}, raised={beater_raised}",
    )

    return ctx.report()


object_model = build_object_model()
