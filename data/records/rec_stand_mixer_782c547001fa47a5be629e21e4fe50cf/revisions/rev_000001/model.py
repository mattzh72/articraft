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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rounded_box(size: tuple[float, float, float], center, radius: float) -> cq.Workplane:
    """Small CadQuery rounded block authored directly in meters."""
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape.translate(center)


def _make_base_shell() -> cq.Workplane:
    foot = _rounded_box((0.56, 0.36, 0.070), (0.020, 0.0, 0.035), 0.060)
    rear_heel = _rounded_box((0.34, 0.31, 0.070), (-0.115, 0.0, 0.085), 0.050)
    column = _rounded_box((0.130, 0.210, 0.280), (-0.180, 0.0, 0.210), 0.035)
    shoulder = _rounded_box((0.180, 0.205, 0.090), (-0.145, 0.0, 0.305), 0.035)
    lug_0 = _rounded_box((0.070, 0.044, 0.135), (-0.180, 0.124, 0.390), 0.012)
    lug_1 = _rounded_box((0.070, 0.044, 0.135), (-0.180, -0.124, 0.390), 0.012)
    return foot.union(rear_heel).union(column).union(shoulder).union(lug_0).union(lug_1)


def _make_head_shell() -> cq.Workplane:
    body = _rounded_box((0.360, 0.180, 0.142), (0.230, 0.0, 0.000), 0.052)
    rear_blend = _rounded_box((0.080, 0.170, 0.110), (0.062, 0.0, -0.004), 0.035)
    nose = _rounded_box((0.085, 0.160, 0.118), (0.405, 0.0, -0.006), 0.042)
    return body.union(rear_blend).union(nose)


def _make_bowl_shell():
    outer = [
        (0.055, 0.026),
        (0.075, 0.040),
        (0.112, 0.108),
        (0.136, 0.176),
    ]
    inner = [
        (0.041, 0.034),
        (0.063, 0.047),
        (0.101, 0.108),
        (0.126, 0.166),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )


def _make_whisk_wires():
    merged = None
    for angle in (0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0):
        c = math.cos(angle)
        s = math.sin(angle)

        def p(radius: float, z: float):
            return (radius * c, radius * s, z)

        points = [
            p(0.018, -0.040),
            p(0.054, -0.082),
            p(0.070, -0.125),
            p(0.026, -0.182),
            p(-0.070, -0.125),
            p(-0.054, -0.082),
            p(-0.018, -0.040),
        ]
        wire = tube_from_spline_points(
            points,
            radius=0.0022,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        )
        merged = wire if merged is None else merged.merge(wire)
    return merged


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_countertop_stand_mixer")

    enamel = model.material("warm_ivory_enamel", rgba=(0.82, 0.73, 0.58, 1.0))
    enamel_shadow = model.material("recessed_ivory_shadow", rgba=(0.62, 0.54, 0.42, 1.0))
    stainless = model.material("brushed_stainless_steel", rgba=(0.73, 0.74, 0.72, 1.0))
    polished = model.material("polished_chrome", rgba=(0.92, 0.90, 0.84, 1.0))
    dark = model.material("dark_rubber", rgba=(0.035, 0.032, 0.030, 1.0))

    base = model.part("base")
    for visual_name, shape in (
        ("base_foot", _rounded_box((0.560, 0.360, 0.070), (0.020, 0.0, 0.035), 0.060)),
        ("rear_heel", _rounded_box((0.340, 0.310, 0.038), (-0.115, 0.0, 0.055), 0.038)),
        ("rear_column", _rounded_box((0.130, 0.210, 0.284), (-0.180, 0.0, 0.208), 0.035)),
        ("top_shoulder", _rounded_box((0.180, 0.205, 0.090), (-0.145, 0.0, 0.305), 0.035)),
        ("hinge_lug_0", _rounded_box((0.070, 0.044, 0.135), (-0.180, 0.124, 0.390), 0.012)),
        ("hinge_lug_1", _rounded_box((0.070, 0.044, 0.135), (-0.180, -0.124, 0.390), 0.012)),
    ):
        base.visual(
            mesh_from_cadquery(shape, visual_name, tolerance=0.0008),
            material=enamel,
            name=visual_name,
        )
    for y, name in ((0.086, "slide_rail_0"), (-0.086, "slide_rail_1")):
        base.visual(
            Box((0.350, 0.020, 0.012)),
            origin=Origin(xyz=(0.075, y, 0.076)),
            material=polished,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.018, length=0.290),
        origin=Origin(xyz=(-0.180, 0.0, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="hinge_pin",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(-0.125, -0.105, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel_shadow,
        name="speed_boss",
    )
    base.visual(
        Box((0.065, 0.008, 0.040)),
        origin=Origin(xyz=(-0.215, 0.106, 0.285)),
        material=enamel_shadow,
        name="lock_pocket",
    )
    for x, y in ((0.235, 0.145), (0.235, -0.145), (-0.205, 0.145), (-0.205, -0.145)):
        base.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(xyz=(x, y, -0.005)),
            material=dark,
            name=f"foot_pad_{len(base.visuals)}",
        )

    bowl = model.part("bowl")
    carriage_shape = _rounded_box((0.270, 0.235, 0.018), (0.0, 0.0, 0.009), 0.026)
    bowl.visual(
        mesh_from_cadquery(carriage_shape, "carriage_slide", tolerance=0.0008),
        material=enamel,
        name="carriage_slide",
    )
    bowl.visual(
        Cylinder(radius=0.073, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=stainless,
        name="bowl_foot",
    )
    bowl.visual(
        mesh_from_geometry(_make_bowl_shell(), "bowl_shell"),
        material=stainless,
        name="bowl_shell",
    )
    bowl.visual(
        mesh_from_geometry(TorusGeometry(radius=0.136, tube=0.004, radial_segments=96, tubular_segments=16), "rolled_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=polished,
        name="rolled_rim",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_make_head_shell(), "head_shell", tolerance=0.0008),
        material=enamel,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="head_barrel",
    )
    head.visual(
        Cylinder(radius=0.033, length=0.012),
        origin=Origin(xyz=(0.452, 0.0, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="attachment_port",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.030),
        origin=Origin(xyz=(0.250, 0.0, -0.085)),
        material=polished,
        name="drive_hub",
    )
    head.visual(
        Box((0.170, 0.008, 0.018)),
        origin=Origin(xyz=(0.225, -0.094, 0.036)),
        material=polished,
        name="side_trim",
    )

    whisk = model.part("whisk")
    whisk.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=polished,
        name="whisk_shaft",
    )
    whisk.visual(
        Cylinder(radius=0.027, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=polished,
        name="wire_collar",
    )
    whisk.visual(
        mesh_from_geometry(_make_whisk_wires(), "whisk_wires"),
        material=polished,
        name="whisk_wires",
    )
    whisk.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.184)),
        material=polished,
        name="bottom_bead",
    )

    speed_control = model.part("speed_control")
    speed_knob = KnobGeometry(
        0.058,
        0.026,
        body_style="skirted",
        top_diameter=0.046,
        edge_radius=0.0012,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0010),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
    )
    speed_control.visual(
        mesh_from_geometry(speed_knob, "speed_control"),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="speed_dial",
    )

    head_lock = model.part("head_lock")
    lock_shape = _rounded_box((0.052, 0.018, 0.028), (0.0, 0.009, 0.0), 0.006)
    head_lock.visual(
        mesh_from_cadquery(lock_shape, "head_lock_button", tolerance=0.0006),
        material=polished,
        name="lock_button",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.070, 0.0, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.060, effort=65.0, velocity=0.18),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.180, 0.0, 0.430)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.65, effort=35.0, velocity=0.8),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.250, 0.0, -0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=45.0),
    )
    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.125, -0.110, 0.235)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=0.70, effort=1.0, velocity=3.0),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.215, 0.110, 0.285)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.012, effort=6.0, velocity=0.06),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    required_types = {
        "base_to_bowl": ArticulationType.PRISMATIC,
        "base_to_head": ArticulationType.REVOLUTE,
        "head_to_whisk": ArticulationType.CONTINUOUS,
        "base_to_speed_control": ArticulationType.REVOLUTE,
        "base_to_head_lock": ArticulationType.PRISMATIC,
    }
    ctx.check(
        "golden articulation tree has exactly five joints",
        len(object_model.articulations) == 5,
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    for joint_name, joint_type in required_types.items():
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} keeps requested joint type",
            joint.articulation_type == joint_type,
            details=f"{joint_name} is {joint.articulation_type}, expected {joint_type}",
        )

    ctx.allow_overlap(
        base,
        head,
        elem_a="hinge_pin",
        elem_b="head_barrel",
        reason="The metal hinge pin is intentionally captured through the head hinge barrel.",
    )
    ctx.expect_within(
        base,
        head,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="head_barrel",
        margin=0.003,
        name="hinge pin is centered inside head barrel",
    )
    ctx.expect_overlap(
        base,
        head,
        axes="y",
        elem_a="hinge_pin",
        elem_b="head_barrel",
        min_overlap=0.090,
        name="hinge pin passes through head barrel",
    )
    ctx.allow_overlap(
        head,
        whisk,
        elem_a="drive_hub",
        elem_b="whisk_shaft",
        reason="The whisk shaft is intentionally captured inside the mixer drive hub.",
    )
    ctx.expect_overlap(
        head,
        whisk,
        axes="z",
        elem_a="drive_hub",
        elem_b="whisk_shaft",
        min_overlap=0.003,
        name="whisk shaft remains inserted into drive hub",
    )

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="carriage_slide",
        negative_elem="slide_rail_0",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="bowl carriage sits on slide rails",
    )
    ctx.expect_within(
        whisk,
        bowl,
        axes="xy",
        inner_elem="whisk_wires",
        outer_elem="bowl_shell",
        margin=0.020,
        name="whisk cage fits inside the bowl mouth",
    )

    bowl_slide = object_model.get_articulation("base_to_bowl")
    head_tilt = object_model.get_articulation("base_to_head")
    lock_slide = object_model.get_articulation("base_to_head_lock")
    bowl_rest = ctx.part_world_position(bowl)
    head_rest = ctx.part_world_position(whisk)
    lock_rest = ctx.part_world_position(head_lock)
    with ctx.pose({bowl_slide: 0.060, head_tilt: 0.65, lock_slide: 0.012}):
        bowl_forward = ctx.part_world_position(bowl)
        head_raised = ctx.part_world_position(whisk)
        lock_pressed = ctx.part_world_position(head_lock)
        ctx.expect_overlap(
            bowl,
            base,
            axes="x",
            elem_a="carriage_slide",
            elem_b="slide_rail_0",
            min_overlap=0.12,
            name="extended bowl slide remains retained on rail",
        )

    ctx.check(
        "bowl slide travels forward a short stroke",
        bowl_rest is not None and bowl_forward is not None and bowl_forward[0] > bowl_rest[0] + 0.045,
        details=f"rest={bowl_rest}, extended={bowl_forward}",
    )
    ctx.check(
        "tilt head lifts the whisk clear upward",
        head_rest is not None and head_raised is not None and head_raised[2] > head_rest[2] + 0.12,
        details=f"rest={head_rest}, tilted={head_raised}",
    )
    ctx.check(
        "head lock button depresses inward",
        lock_rest is not None and lock_pressed is not None and lock_pressed[1] < lock_rest[1] - 0.008,
        details=f"rest={lock_rest}, pressed={lock_pressed}",
    )
    ctx.expect_contact(
        speed_control,
        base,
        elem_a="speed_dial",
        elem_b="speed_boss",
        contact_tol=0.0025,
        name="side speed dial is mounted on its boss",
    )

    return ctx.report()


object_model = build_object_model()
