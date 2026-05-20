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


def _rounded_motor_base() -> cq.Workplane:
    """Soft rectangular motor housing, authored in meters."""
    return (
        cq.Workplane("XY")
        .box(0.34, 0.29, 0.14, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.025)
        .edges(">Z")
        .fillet(0.010)
    )


def _bowl_shell() -> cq.Workplane:
    """Transparent, broad, open food-processor bowl with a real cavity."""
    height = 0.230
    outer = (
        cq.Workplane("XY")
        .circle(0.100)
        .workplane(offset=height)
        .circle(0.150)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.014)
        .circle(0.074)
        .workplane(offset=height + 0.035)
        .circle(0.126)
        .loft(combine=True)
    )
    return outer.cut(inner)


def _lid_ring() -> cq.Workplane:
    """Shallow lid rim with a large central viewing/opening over the cutter."""
    lid = cq.Workplane("XY").circle(0.157).extrude(0.034)
    center_opening = (
        cq.Workplane("XY").circle(0.092).extrude(0.060).translate((0.0, 0.0, -0.010))
    )
    return lid.cut(center_opening)


def _chute_body() -> cq.Workplane:
    """Tall rectangular chute, open at both ends and visibly hollow."""
    outer = cq.Workplane("XY").rect(0.104, 0.084).extrude(0.225)
    inner = (
        cq.Workplane("XY")
        .rect(0.084, 0.064)
        .extrude(0.250)
        .translate((0.0, 0.0, -0.012))
    )
    return outer.cut(inner).edges("|Z").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_food_processor")

    satin = model.material("satin_warm_silver", rgba=(0.78, 0.75, 0.68, 1.0))
    charcoal = model.material("soft_charcoal", rgba=(0.025, 0.028, 0.030, 1.0))
    black_glass = model.material("black_glass", rgba=(0.005, 0.007, 0.010, 1.0))
    clear_bowl = model.material("clear_smoke_blue", rgba=(0.58, 0.78, 0.92, 0.34))
    smoke_lid = model.material("clear_smoke_lid", rgba=(0.45, 0.56, 0.64, 0.44))
    stainless = model.material("brushed_stainless", rgba=(0.86, 0.86, 0.82, 1.0))
    blade_dark = model.material("sharpened_steel_edge", rgba=(0.58, 0.60, 0.60, 1.0))
    accent = model.material("cool_white_marking", rgba=(0.86, 0.93, 1.0, 1.0))
    button_blue = model.material("program_blue", rgba=(0.06, 0.20, 0.34, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_motor_base(), "motor_base", tolerance=0.0015),
        material=satin,
        name="motor_base",
    )
    base.visual(
        Box((0.230, 0.010, 0.090)),
        origin=Origin(xyz=(0.020, -0.148, 0.078)),
        material=black_glass,
        name="front_panel",
    )
    base.visual(
        mesh_from_cadquery(_bowl_shell(), "bowl_shell", tolerance=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=clear_bowl,
        name="bowl_shell",
    )
    base.visual(
        mesh_from_cadquery(_lid_ring(), "lid_ring", tolerance=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=smoke_lid,
        name="lid_ring",
    )
    base.visual(
        mesh_from_cadquery(_chute_body(), "chute_body", tolerance=0.0012),
        origin=Origin(xyz=(0.055, 0.076, 0.382)),
        material=smoke_lid,
        name="chute_body",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.1875)),
        material=charcoal,
        name="drive_shaft",
    )
    base.visual(
        Box((0.094, 0.064, 0.006)),
        origin=Origin(xyz=(-0.070, -0.030, 0.402)),
        material=black_glass,
        name="ingredient_port_gasket",
    )
    base.visual(
        Box((0.095, 0.012, 0.010)),
        origin=Origin(xyz=(-0.070, 0.000, 0.404)),
        material=charcoal,
        name="flap_hinge_boss",
    )

    disc = model.part("processing_disc")
    disc.visual(
        Cylinder(radius=0.082, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=stainless,
        name="disc_plate",
    )
    disc.visual(
        Cylinder(radius=0.026, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=charcoal,
        name="disc_hub",
    )
    disc.visual(
        Box((0.100, 0.018, 0.006)),
        origin=Origin(xyz=(0.036, 0.018, 0.010), rpy=(0.0, 0.0, 0.22)),
        material=blade_dark,
        name="raised_blade_0",
    )
    disc.visual(
        Box((0.100, 0.018, 0.006)),
        origin=Origin(xyz=(-0.036, -0.018, 0.010), rpy=(0.0, 0.0, 0.22)),
        material=blade_dark,
        name="raised_blade_1",
    )
    model.articulation(
        "base_to_processing_disc",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=disc,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=40.0),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.070, 0.050, 0.220)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=charcoal,
        name="pusher_plunger",
    )
    pusher.visual(
        Box((0.118, 0.096, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=charcoal,
        name="pusher_cap",
    )
    pusher.visual(
        Box((0.055, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=satin,
        name="pusher_grip",
    )
    model.articulation(
        "base_to_pusher",
        ArticulationType.PRISMATIC,
        parent=base,
        child=pusher,
        origin=Origin(xyz=(0.055, 0.076, 0.612)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.095),
    )

    flap = model.part("ingredient_flap")
    flap.visual(
        Box((0.084, 0.058, 0.007)),
        origin=Origin(xyz=(0.0, -0.034, 0.005)),
        material=smoke_lid,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.006, length=0.088),
        origin=Origin(xyz=(0.0, -0.003, 0.007), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="flap_barrel",
    )
    model.articulation(
        "base_to_ingredient_flap",
        ArticulationType.REVOLUTE,
        parent=base,
        child=flap,
        origin=Origin(xyz=(-0.070, 0.000, 0.408)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.30),
    )

    dial = model.part("selector_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.025,
                body_style="skirted",
                top_diameter=0.042,
                skirt=KnobSkirt(0.066, 0.006, flare=0.05, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "selector_dial",
        ),
        material=charcoal,
        name="dial_cap",
    )
    model.articulation(
        "base_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(-0.074, -0.151, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    button_positions = (
        (0.035, 0.101),
        (0.083, 0.101),
        (0.035, 0.060),
        (0.083, 0.060),
    )
    for i, (x, z) in enumerate(button_positions):
        button = model.part(f"program_button_{i}")
        button.visual(
            Box((0.034, 0.020, 0.011)),
            origin=Origin(xyz=(0.0, 0.0, 0.0055)),
            material=button_blue,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.003, 0.0015)),
            origin=Origin(xyz=(0.0, 0.0, 0.0112)),
            material=accent,
            name="button_mark",
        )
        model.articulation(
            f"base_to_program_button_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, -0.151, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.010),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    disc = object_model.get_part("processing_disc")
    pusher = object_model.get_part("pusher")
    flap = object_model.get_part("ingredient_flap")
    dial = object_model.get_part("selector_dial")

    disc_joint = object_model.get_articulation("base_to_processing_disc")
    pusher_joint = object_model.get_articulation("base_to_pusher")
    flap_joint = object_model.get_articulation("base_to_ingredient_flap")
    dial_joint = object_model.get_articulation("base_to_selector_dial")

    ctx.check(
        "disc and dial are continuous rotary controls",
        disc_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"disc={disc_joint.articulation_type}, dial={dial_joint.articulation_type}",
    )
    ctx.check(
        "pusher and ingredient flap have requested mechanisms",
        pusher_joint.articulation_type == ArticulationType.PRISMATIC
        and flap_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"pusher={pusher_joint.articulation_type}, flap={flap_joint.articulation_type}",
    )

    ctx.expect_within(
        disc,
        base,
        axes="xy",
        inner_elem="disc_plate",
        outer_elem="bowl_shell",
        margin=0.010,
        name="processing disc sits inside the broad bowl footprint",
    )
    ctx.expect_gap(
        base,
        disc,
        axis="z",
        positive_elem="lid_ring",
        negative_elem="disc_plate",
        min_gap=0.110,
        name="lid opening leaves clear air above the cutter disc",
    )

    ctx.expect_within(
        pusher,
        base,
        axes="xy",
        inner_elem="pusher_plunger",
        outer_elem="chute_body",
        margin=0.002,
        name="pusher plunger is centered in the hollow chute",
    )
    ctx.expect_overlap(
        pusher,
        base,
        axes="z",
        elem_a="pusher_plunger",
        elem_b="chute_body",
        min_overlap=0.180,
        name="inserted pusher remains engaged in the chute",
    )
    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.095}):
        ctx.expect_within(
            pusher,
            base,
            axes="xy",
            inner_elem="pusher_plunger",
            outer_elem="chute_body",
            margin=0.002,
            name="raised pusher still tracks inside chute walls",
        )
        ctx.expect_overlap(
            pusher,
            base,
            axes="z",
            elem_a="pusher_plunger",
            elem_b="chute_body",
            min_overlap=0.090,
            name="raised pusher keeps retained insertion in chute",
        )
        raised_pusher = ctx.part_world_position(pusher)
    ctx.check(
        "pusher slides upward out of the chute",
        rest_pusher is not None and raised_pusher is not None and raised_pusher[2] > rest_pusher[2] + 0.080,
        details=f"rest={rest_pusher}, raised={raised_pusher}",
    )

    rest_flap = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.20}):
        open_flap = ctx.part_world_aabb(flap)
    ctx.check(
        "ingredient flap rotates upward on its short hinge",
        rest_flap is not None and open_flap is not None and open_flap[1][2] > rest_flap[1][2] + 0.025,
        details=f"rest={rest_flap}, open={open_flap}",
    )

    for i in range(4):
        joint = object_model.get_articulation(f"base_to_program_button_{i}")
        button = object_model.get_part(f"program_button_{i}")
        ctx.check(
            f"program button {i} is a prismatic push control",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=str(joint.articulation_type),
        )
        rest_button = ctx.part_world_position(button)
        with ctx.pose({joint: 0.010}):
            pressed_button = ctx.part_world_position(button)
        ctx.check(
            f"program button {i} depresses into the front panel",
            rest_button is not None
            and pressed_button is not None
            and pressed_button[1] > rest_button[1] + 0.008,
            details=f"rest={rest_button}, pressed={pressed_button}",
        )

    return ctx.report()


object_model = build_object_model()
