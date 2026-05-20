from __future__ import annotations

import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _bottle_shell_mesh():
    """Open-topped translucent calibration bottle with real wall thickness."""
    width = 0.074
    depth = 0.056
    height = 0.188
    wall = 0.0024

    outer = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .edges("<Z")
        .fillet(0.003)
    )
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height + 0.010, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, wall))
    )
    return outer.cut(inner)


def _shoulder_mesh():
    """Connected sloped bottle shoulder, left hollow at the neck."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=0.186)
        .rect(0.064, 0.046)
        .workplane(offset=0.030)
        .circle(0.016)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.186)
        .rect(0.050, 0.034)
        .workplane(offset=0.036)
        .circle(0.010)
        .loft(combine=True)
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_trigger_spray_bottle")

    clear_poly = model.material("clear_polycarbonate", rgba=(0.65, 0.86, 1.0, 0.34))
    frosted = model.material("frosted_white", rgba=(0.92, 0.96, 1.0, 0.62))
    dark = model.material("graphite_mechanism", rgba=(0.04, 0.045, 0.05, 1.0))
    satin = model.material("satin_calibration_metal", rgba=(0.56, 0.60, 0.62, 1.0))
    black = model.material("black_index_ink", rgba=(0.0, 0.0, 0.0, 1.0))
    amber = model.material("amber_trigger", rgba=(1.0, 0.48, 0.09, 1.0))
    datum_blue = model.material("datum_blue", rgba=(0.05, 0.22, 0.75, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_shell_mesh(), "open_bottle_shell", tolerance=0.0008),
        material=clear_poly,
        name="open_bottle_shell",
    )
    bottle.visual(
        mesh_from_cadquery(_shoulder_mesh(), "hollow_shoulder", tolerance=0.0008),
        material=clear_poly,
        name="hollow_shoulder",
    )
    bottle.visual(
        Box((0.070, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=clear_poly,
        name="shoulder_deck",
    )
    bottle.visual(
        Cylinder(radius=0.014, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=clear_poly,
        name="hollow_neck_wall",
    )
    bottle.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.226)),
        material=frosted,
        name="neck_bore_highlight",
    )
    # Flat datum pads make the bottle easy to fixture without destroying the translucent read.
    bottle.visual(
        Box((0.042, 0.0020, 0.052)),
        origin=Origin(xyz=(0.0, 0.0275, 0.100)),
        material=datum_blue,
        name="side_datum_pad",
    )
    bottle.visual(
        Box((0.012, 0.0030, 0.096)),
        origin=Origin(xyz=(0.0, 0.0280, 0.143)),
        material=datum_blue,
        name="side_datum_rib",
    )
    bottle.visual(
        Box((0.0022, 0.026, 0.020)),
        origin=Origin(xyz=(0.0368, 0.0, 0.135)),
        material=datum_blue,
        name="front_datum_pad",
    )
    bottle.visual(
        Box((0.0030, 0.010, 0.096)),
        origin=Origin(xyz=(0.0375, 0.0, 0.161)),
        material=datum_blue,
        name="front_datum_rib",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.263)),
        material=dark,
        name="threaded_collar",
    )
    for i in range(18):
        angle = i * 2.0 * pi / 18.0
        x = 0.0251 * __import__("math").cos(angle)
        y = 0.0251 * __import__("math").sin(angle)
        head.visual(
            Box((0.0024, 0.0045, 0.030)),
            origin=Origin(xyz=(x, y, 0.263), rpy=(0.0, 0.0, angle)),
            material=satin,
            name=f"collar_grip_{i}",
        )
    head.visual(
        Box((0.118, 0.046, 0.038)),
        origin=Origin(xyz=(0.045, 0.0, 0.269)),
        material=dark,
        name="pump_housing",
    )
    head.visual(
        Box((0.110, 0.034, 0.024)),
        origin=Origin(xyz=(0.126, 0.0, 0.252)),
        material=dark,
        name="nozzle_beam",
    )
    head.visual(
        Cylinder(radius=0.0105, length=0.095),
        origin=Origin(xyz=(0.133, 0.0, 0.252), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="nozzle_tube",
    )
    head.visual(
        Cylinder(radius=0.0065, length=0.064),
        origin=Origin(xyz=(0.145, 0.0, 0.224), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin,
        name="pump_guide_bore",
    )
    head.visual(
        Box((0.078, 0.008, 0.005)),
        origin=Origin(xyz=(0.145, -0.012, 0.238)),
        material=satin,
        name="upper_pump_way",
    )
    head.visual(
        Box((0.078, 0.008, 0.005)),
        origin=Origin(xyz=(0.145, 0.012, 0.214)),
        material=satin,
        name="lower_pump_way",
    )
    head.visual(
        Box((0.028, 0.006, 0.010)),
        origin=Origin(xyz=(0.066, -0.028, 0.218)),
        material=dark,
        name="pivot_yoke_0",
    )
    head.visual(
        Box((0.028, 0.006, 0.010)),
        origin=Origin(xyz=(0.066, 0.028, 0.218)),
        material=dark,
        name="pivot_yoke_1",
    )
    head.visual(
        Cylinder(radius=0.0032, length=0.064),
        origin=Origin(xyz=(0.065, 0.0, 0.218), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="trigger_pivot_pin",
    )
    head.visual(
        Box((0.010, 0.006, 0.026)),
        origin=Origin(xyz=(0.066, -0.028, 0.231)),
        material=dark,
        name="yoke_strut_0",
    )
    head.visual(
        Box((0.010, 0.006, 0.026)),
        origin=Origin(xyz=(0.066, 0.028, 0.231)),
        material=dark,
        name="yoke_strut_1",
    )
    head.visual(
        Box((0.132, 0.006, 0.040)),
        origin=Origin(xyz=(0.105, -0.024, 0.236)),
        material=dark,
        name="side_backbone_0",
    )
    head.visual(
        Box((0.132, 0.006, 0.040)),
        origin=Origin(xyz=(0.105, 0.024, 0.236)),
        material=dark,
        name="side_backbone_1",
    )
    head.visual(
        Box((0.078, 0.004, 0.028)),
        origin=Origin(xyz=(0.145, -0.020, 0.226)),
        material=satin,
        name="guide_side_plate_0",
    )
    head.visual(
        Box((0.078, 0.004, 0.028)),
        origin=Origin(xyz=(0.145, 0.020, 0.226)),
        material=satin,
        name="guide_side_plate_1",
    )
    head.visual(
        Box((0.064, 0.013, 0.006)),
        origin=Origin(xyz=(0.145, 0.012, 0.224)),
        material=satin,
        name="bore_web_0",
    )
    head.visual(
        Box((0.064, 0.013, 0.006)),
        origin=Origin(xyz=(0.145, -0.012, 0.224)),
        material=satin,
        name="bore_web_1",
    )
    head.visual(
        Box((0.064, 0.004, 0.014)),
        origin=Origin(xyz=(0.145, 0.014, 0.219)),
        material=satin,
        name="lower_way_web",
    )
    # Controlled-gap reference bridge around the moving pump rod.
    head.visual(
        Box((0.050, 0.003, 0.005)),
        origin=Origin(xyz=(0.145, 0.019, 0.224)),
        material=datum_blue,
        name="gap_reference_rail",
    )
    head.visual(
        Box((0.050, 0.003, 0.005)),
        origin=Origin(xyz=(0.145, -0.019, 0.224)),
        material=datum_blue,
        name="gap_reference_rail_1",
    )
    for i, x in enumerate((0.106, 0.118, 0.130, 0.142, 0.154, 0.166)):
        head.visual(
            Box((0.0012, 0.0025, 0.010 if i % 2 == 0 else 0.006)),
            origin=Origin(xyz=(x, -0.0170, 0.242)),
            material=black,
            name=f"pump_index_tick_{i}",
        )
    for i, x in enumerate((-0.010, 0.000, 0.010, 0.020, 0.030)):
        head.visual(
            Box((0.0010, 0.008, 0.006 if i != 2 else 0.010)),
            origin=Origin(xyz=(x, 0.0225, 0.276)),
            material=black,
            name=f"datum_tick_{i}",
        )

    model.articulation(
        "bottle_to_head",
        ArticulationType.FIXED,
        parent=bottle,
        child=head,
        origin=Origin(),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.008, length=0.037),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=amber,
        name="pivot_barrel",
    )
    trigger.visual(
        Box((0.018, 0.018, 0.096)),
        origin=Origin(xyz=(-0.014, 0.0, -0.046), rpy=(0.0, -0.20, 0.0)),
        material=amber,
        name="finger_blade",
    )
    trigger.visual(
        Box((0.024, 0.020, 0.018)),
        origin=Origin(xyz=(0.000, 0.0, -0.084), rpy=(0.0, -0.20, 0.0)),
        material=amber,
        name="finger_pad",
    )
    trigger.visual(
        Box((0.018, 0.018, 0.034)),
        origin=Origin(xyz=(-0.006, 0.0, -0.014)),
        material=amber,
        name="pivot_web",
    )
    trigger.visual(
        Box((0.038, 0.016, 0.014)),
        origin=Origin(xyz=(0.020, 0.0, 0.004)),
        material=amber,
        name="cam_nose",
    )
    trigger.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(0.035, 0.0, 0.004), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="cam_roller",
    )
    trigger_joint = model.articulation(
        "head_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head,
        child=trigger,
        origin=Origin(xyz=(0.065, 0.0, 0.218)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.45, effort=18.0, velocity=3.0),
    )

    pump_rod = model.part("pump_rod")
    pump_rod.visual(
        Cylinder(radius=0.0042, length=0.056),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin,
        name="visible_plunger_rod",
    )
    pump_rod.visual(
        Box((0.006, 0.018, 0.016)),
        origin=Origin(xyz=(-0.003, 0.0, 0.0)),
        material=satin,
        name="cam_contact_pad",
    )
    pump_rod.visual(
        Box((0.012, 0.018, 0.018)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=satin,
        name="piston_head",
    )
    model.articulation(
        "head_to_pump_rod",
        ArticulationType.PRISMATIC,
        parent=head,
        child=pump_rod,
        origin=Origin(xyz=(0.110, 0.0, 0.224)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.016, effort=35.0, velocity=0.20),
    )

    nozzle_dial = model.part("nozzle_dial")
    nozzle_dial.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="knurled_dial",
    )
    for i in range(16):
        angle = i * 2.0 * pi / 16.0
        y = 0.0150 * __import__("math").cos(angle)
        z = 0.0150 * __import__("math").sin(angle)
        nozzle_dial.visual(
            Box((0.012, 0.0018, 0.0040)),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, 0.0, angle)),
            material=dark,
            name=f"dial_grip_{i}",
        )
    nozzle_dial.visual(
        Box((0.004, 0.0015, 0.012)),
        origin=Origin(xyz=(0.0075, 0.0, 0.010)),
        material=black,
        name="dial_pointer",
    )
    nozzle_dial.visual(
        Cylinder(radius=0.0032, length=0.006),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="spray_orifice",
    )
    model.articulation(
        "head_to_nozzle_dial",
        ArticulationType.REVOLUTE,
        parent=head,
        child=nozzle_dial,
        origin=Origin(xyz=(0.188, 0.0, 0.252)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.57, upper=1.57, effort=1.0, velocity=2.0),
    )

    stroke_knob = model.part("stroke_knob")
    stroke_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.022,
                0.010,
                body_style="lobed",
                grip=KnobGrip(style="ribbed", count=10, depth=0.0008),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=90.0),
            ),
            "stroke_stop_knob",
        ),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=datum_blue,
        name="lobed_adjuster",
    )
    stroke_knob.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="adjuster_stem",
    )
    model.articulation(
        "head_to_stroke_knob",
        ArticulationType.REVOLUTE,
        parent=head,
        child=stroke_knob,
        origin=Origin(xyz=(0.014, -0.033, 0.246)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.60, upper=0.60, effort=0.8, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head = object_model.get_part("head")
    trigger = object_model.get_part("trigger")
    pump_rod = object_model.get_part("pump_rod")
    dial = object_model.get_part("nozzle_dial")
    trigger_joint = object_model.get_articulation("head_to_trigger")
    pump_joint = object_model.get_articulation("head_to_pump_rod")

    ctx.allow_overlap(
        head,
        trigger,
        elem_a="trigger_pivot_pin",
        elem_b="pivot_barrel",
        reason="The stainless pivot pin is intentionally captured through the trigger barrel.",
    )
    ctx.allow_overlap(
        head,
        trigger,
        elem_a="trigger_pivot_pin",
        elem_b="pivot_web",
        reason="The same pivot pin passes through the molded trigger boss web.",
    )
    ctx.allow_overlap(
        head,
        pump_rod,
        elem_a="pump_guide_bore",
        elem_b="piston_head",
        reason="The piston head is intentionally nested inside the simplified pump bore proxy.",
    )
    ctx.allow_overlap(
        head,
        pump_rod,
        elem_a="pump_guide_bore",
        elem_b="visible_plunger_rod",
        reason="The visible plunger rod is shown sliding inside the simplified pump bore proxy.",
    )
    ctx.expect_within(
        trigger,
        head,
        axes="y",
        inner_elem="pivot_barrel",
        outer_elem="trigger_pivot_pin",
        margin=0.001,
        name="trigger barrel is retained by the pivot pin",
    )
    ctx.expect_contact(
        trigger,
        pump_rod,
        elem_a="cam_nose",
        elem_b="cam_contact_pad",
        contact_tol=0.001,
        name="trigger cam visibly bears on pump pad at rest",
    )
    ctx.expect_overlap(
        pump_rod,
        head,
        axes="x",
        elem_a="visible_plunger_rod",
        elem_b="pump_guide_bore",
        min_overlap=0.040,
        name="plunger remains guided through the pump bore",
    )
    ctx.expect_gap(
        head,
        pump_rod,
        axis="y",
        positive_elem="gap_reference_rail",
        negative_elem="visible_plunger_rod",
        min_gap=0.004,
        max_gap=0.030,
        name="controlled side clearance remains visible",
    )

    rest_pos = ctx.part_world_position(pump_rod)
    rest_trigger = ctx.part_world_aabb(trigger)
    with ctx.pose({trigger_joint: 0.45, pump_joint: 0.016}):
        stroke_pos = ctx.part_world_position(pump_rod)
        stroke_trigger = ctx.part_world_aabb(trigger)
        ctx.expect_overlap(
            pump_rod,
            head,
            axes="x",
            elem_a="visible_plunger_rod",
            elem_b="pump_guide_bore",
            min_overlap=0.025,
            name="plunger is still captured at full trigger stroke",
        )

    ctx.check(
        "trigger stroke drives pump rod forward",
        rest_pos is not None and stroke_pos is not None and stroke_pos[0] > rest_pos[0] + 0.012,
        details=f"rest={rest_pos}, stroked={stroke_pos}",
    )
    ctx.check(
        "trigger blade sweeps toward bottle during stroke",
        rest_trigger is not None
        and stroke_trigger is not None
        and stroke_trigger[0][0] < rest_trigger[0][0] - 0.002,
        details=f"rest_aabb={rest_trigger}, stroked_aabb={stroke_trigger}",
    )
    ctx.expect_contact(
        dial,
        head,
        elem_a="knurled_dial",
        elem_b="nozzle_tube",
        contact_tol=0.006,
        name="nozzle adjustment dial is seated on the nozzle tube",
    )

    return ctx.report()


object_model = build_object_model()
