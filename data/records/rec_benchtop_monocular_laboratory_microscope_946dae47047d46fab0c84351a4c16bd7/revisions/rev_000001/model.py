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
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A centered, softly filleted rectangular casting."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _stage_plate(size: tuple[float, float, float], aperture_radius: float) -> cq.Workplane:
    """A centered mechanical-stage plate with a real optical aperture."""
    sx, sy, sz = size
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz)
        .edges("|Z")
        .fillet(0.004)
        .faces(">Z")
        .workplane()
        .hole(2.0 * aperture_radius)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_microscope")

    cream = model.material("warm_off_white_enamel", color=(0.82, 0.80, 0.72, 1.0))
    black = model.material("matte_black", color=(0.015, 0.015, 0.018, 1.0))
    dark = model.material("dark_oxide", color=(0.08, 0.085, 0.09, 1.0))
    metal = model.material("brushed_metal", color=(0.62, 0.64, 0.62, 1.0))
    glass = model.material("blue_green_glass", color=(0.35, 0.70, 0.85, 0.45))
    brass = model.material("aged_brass", color=(0.62, 0.45, 0.18, 1.0))

    # Root casting and support structure: broad bench base, rear focusing column,
    # and an offset side arm that reads as the microscope's carrying arm.
    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_rounded_box((0.260, 0.180, 0.028), 0.018), "rounded_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=cream,
        name="base_casting",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.086),
        origin=Origin(xyz=(-0.035, 0.0, 0.071)),
        material=cream,
        name="stage_pedestal",
    )
    stand.visual(
        Box((0.070, 0.095, 0.006)),
        origin=Origin(xyz=(-0.035, 0.0, 0.107)),
        material=cream,
        name="stage_yoke_web",
    )
    stand.visual(
        Box((0.055, 0.020, 0.008)),
        origin=Origin(xyz=(-0.035, 0.046, 0.110)),
        material=cream,
        name="stage_yoke_pad_0",
    )
    stand.visual(
        Box((0.055, 0.020, 0.008)),
        origin=Origin(xyz=(-0.035, -0.046, 0.110)),
        material=cream,
        name="stage_yoke_pad_1",
    )
    stand.visual(
        Box((0.030, 0.040, 0.246)),
        origin=Origin(xyz=(0.065, 0.0, 0.151)),
        material=cream,
        name="focus_column",
    )
    stand.visual(
        Box((0.006, 0.048, 0.170)),
        origin=Origin(xyz=(0.0475, 0.0, 0.190)),
        material=dark,
        name="focus_rail",
    )
    stand.visual(
        Box((0.230, 0.024, 0.026)),
        origin=Origin(xyz=(0.065, -0.066, 0.158), rpy=(0.0, -1.887, 0.0)),
        material=cream,
        name="side_arm",
    )
    stand.visual(
        Box((0.060, 0.026, 0.030)),
        origin=Origin(xyz=(0.102, -0.066, 0.043)),
        material=cream,
        name="arm_foot",
    )
    stand.visual(
        Box((0.036, 0.070, 0.034)),
        origin=Origin(xyz=(0.064, -0.036, 0.262)),
        material=cream,
        name="arm_boss",
    )

    # A separate fixed mechanical stage.  The plate has a real through aperture;
    # guide rails and a side bracket carry the moving side carriage and knobs.
    stage = model.part("stage")
    stage.visual(
        mesh_from_cadquery(_stage_plate((0.140, 0.100, 0.012), 0.035), "stage_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black,
        name="stage_plate",
    )
    stage.visual(
        Box((0.120, 0.006, 0.006)),
        origin=Origin(xyz=(0.000, 0.032, 0.015)),
        material=metal,
        name="inner_guide_rail",
    )
    stage.visual(
        Box((0.120, 0.006, 0.006)),
        origin=Origin(xyz=(0.000, 0.045, 0.015)),
        material=metal,
        name="outer_guide_rail",
    )
    stage.visual(
        Box((0.090, 0.020, 0.026)),
        origin=Origin(xyz=(0.035, 0.064, 0.018)),
        material=dark,
        name="stage_bracket",
    )
    stage.visual(
        Box((0.086, 0.010, 0.010)),
        origin=Origin(xyz=(0.034, 0.052, 0.009)),
        material=dark,
        name="bracket_web",
    )
    stage.visual(
        Box((0.048, 0.012, 0.004)),
        origin=Origin(xyz=(-0.030, -0.035, 0.014)),
        material=metal,
        name="slide_clip",
    )

    model.articulation(
        "stand_to_stage",
        ArticulationType.FIXED,
        parent=stand,
        child=stage,
        origin=Origin(xyz=(-0.035, 0.0, 0.114)),
    )

    # Side carriage: the moving member of the mechanical stage guide.
    side_carriage = model.part("side_carriage")
    side_carriage.visual(
        Box((0.052, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=metal,
        name="carriage_plate",
    )
    side_carriage.visual(
        Box((0.060, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.010, 0.010)),
        material=dark,
        name="carriage_grip",
    )
    model.articulation(
        "stage_to_side_carriage",
        ArticulationType.PRISMATIC,
        parent=stage,
        child=side_carriage,
        origin=Origin(xyz=(-0.010, 0.0385, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=-0.026, upper=0.026),
    )

    # Distinct coaxial stage-axis controls on the side bracket.
    inner_knob = model.part("inner_stage_knob")
    inner_knob.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="inner_stem",
    )
    inner_knob.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.0, 0.019, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="inner_knob_cap",
    )
    inner_knob.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(0.0, 0.033, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="inner_knurled_rim",
    )
    model.articulation(
        "stage_to_inner_knob",
        ArticulationType.CONTINUOUS,
        parent=stage,
        child=inner_knob,
        origin=Origin(xyz=(0.035, 0.074, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    outer_knob = model.part("outer_stage_knob")
    outer_knob.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="outer_knob_cap",
    )
    outer_knob.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="outer_knurled_rim",
    )
    model.articulation(
        "inner_to_outer_knob",
        ArticulationType.CONTINUOUS,
        parent=inner_knob,
        child=outer_knob,
        origin=Origin(xyz=(0.0, 0.044, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=10.0),
    )

    # Sliding optical body with monocular tube, eyepiece, and nosepiece collar.
    optical_body = model.part("optical_body")
    optical_body.visual(
        Box((0.018, 0.050, 0.085)),
        origin=Origin(xyz=(-0.009, 0.0, 0.000)),
        material=dark,
        name="slide_plate",
    )
    optical_body.visual(
        Box((0.048, 0.042, 0.036)),
        origin=Origin(xyz=(-0.034, 0.0, -0.010)),
        material=cream,
        name="body_bridge",
    )
    optical_body.visual(
        Box((0.078, 0.060, 0.042)),
        origin=Origin(xyz=(-0.080, 0.0, -0.026)),
        material=cream,
        name="head_block",
    )
    optical_body.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(-0.080, 0.0, -0.048)),
        material=cream,
        name="nosepiece_neck",
    )
    optical_body.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(-0.080, 0.0, -0.058)),
        material=dark,
        name="nosepiece_collar",
    )
    optical_body.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(-0.070, 0.0, 0.001), rpy=(0.0, 0.55, 0.0)),
        material=cream,
        name="tube_socket",
    )
    optical_body.visual(
        Cylinder(radius=0.014, length=0.110),
        origin=Origin(xyz=(-0.039, 0.0, 0.056), rpy=(0.0, 0.55, 0.0)),
        material=black,
        name="monocular_tube",
    )
    optical_body.visual(
        Cylinder(radius=0.019, length=0.034),
        origin=Origin(xyz=(-0.002, 0.0, 0.111), rpy=(0.0, 0.55, 0.0)),
        material=black,
        name="eyepiece",
    )
    optical_body.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.007, 0.0, 0.126), rpy=(0.0, 0.55, 0.0)),
        material=glass,
        name="ocular_lens",
    )
    model.articulation(
        "stand_to_optical_body",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=optical_body,
        origin=Origin(xyz=(0.0445, 0.0, 0.230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.05, lower=0.0, upper=0.050),
    )

    # Rotating objective turret.  Its barrels are part of the turret so the full
    # objective cluster spins continuously about the nosepiece axis.
    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=dark,
        name="turret_disk",
    )
    for i, (x, y, radius, length) in enumerate(
        (
            (0.020, 0.000, 0.0055, 0.034),
            (-0.010, 0.017, 0.0060, 0.038),
            (-0.010, -0.017, 0.0045, 0.030),
        )
    ):
        turret.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, -0.012 - length / 2.0)),
            material=brass if i == 0 else metal,
            name=f"objective_{i}",
        )
        turret.visual(
            Cylinder(radius=radius * 1.35, length=0.005),
            origin=Origin(xyz=(x, y, -0.014)),
            material=dark,
            name=f"objective_collar_{i}",
        )
    model.articulation(
        "body_to_turret",
        ArticulationType.CONTINUOUS,
        parent=optical_body,
        child=turret,
        origin=Origin(xyz=(-0.080, 0.0, -0.067)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    stage = object_model.get_part("stage")
    optical_body = object_model.get_part("optical_body")
    side_carriage = object_model.get_part("side_carriage")
    inner_knob = object_model.get_part("inner_stage_knob")
    outer_knob = object_model.get_part("outer_stage_knob")
    turret = object_model.get_part("turret")

    focus_slide = object_model.get_articulation("stand_to_optical_body")
    carriage_slide = object_model.get_articulation("stage_to_side_carriage")
    turret_spin = object_model.get_articulation("body_to_turret")
    outer_spin = object_model.get_articulation("inner_to_outer_knob")

    ctx.expect_gap(
        stage,
        stand,
        axis="z",
        positive_elem="stage_plate",
        negative_elem="stage_yoke_pad_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="separate stage sits on yoke pad",
    )
    ctx.expect_gap(
        stage,
        stand,
        axis="z",
        positive_elem="stage_plate",
        negative_elem="stage_yoke_pad_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="stage has second yoke support",
    )
    ctx.expect_gap(
        stand,
        optical_body,
        axis="x",
        positive_elem="focus_rail",
        negative_elem="slide_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="optical slide bears on focus rail",
    )
    ctx.expect_origin_distance(
        inner_knob,
        outer_knob,
        axes="xz",
        max_dist=0.001,
        name="coaxial stage knobs share xz axis",
    )
    ctx.expect_gap(
        outer_knob,
        inner_knob,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        name="coaxial knobs are distinct caps",
    )
    ctx.expect_within(
        turret,
        stage,
        axes="xy",
        inner_elem="turret_disk",
        outer_elem="stage_plate",
        margin=0.01,
        name="nosepiece centered over stage aperture area",
    )

    rest_optical = ctx.part_world_position(optical_body)
    with ctx.pose({focus_slide: 0.050}):
        lifted_optical = ctx.part_world_position(optical_body)
        ctx.expect_gap(
            stand,
            optical_body,
            axis="x",
            positive_elem="focus_rail",
            negative_elem="slide_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name="raised optical slide remains on focus rail",
        )
        ctx.expect_overlap(
            optical_body,
            stand,
            axes="z",
            elem_a="slide_plate",
            elem_b="focus_rail",
            min_overlap=0.010,
            name="raised optical slide remains retained",
        )
    ctx.check(
        "optical body focuses upward",
        rest_optical is not None
        and lifted_optical is not None
        and lifted_optical[2] > rest_optical[2] + 0.045,
        details=f"rest={rest_optical}, lifted={lifted_optical}",
    )

    rest_carriage = ctx.part_world_position(side_carriage)
    with ctx.pose({carriage_slide: 0.026}):
        shifted_carriage = ctx.part_world_position(side_carriage)
        ctx.expect_overlap(
            side_carriage,
            stage,
            axes="x",
            elem_a="carriage_plate",
            elem_b="outer_guide_rail",
            min_overlap=0.035,
            name="side carriage remains on guide at travel end",
        )
    ctx.check(
        "side carriage slides along stage guide",
        rest_carriage is not None
        and shifted_carriage is not None
        and shifted_carriage[0] > rest_carriage[0] + 0.020,
        details=f"rest={rest_carriage}, shifted={shifted_carriage}",
    )
    ctx.check(
        "turret is continuous about nosepiece",
        turret_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(turret_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={turret_spin.articulation_type}, axis={turret_spin.axis}",
    )
    ctx.check(
        "second stage control rotates continuously",
        outer_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(outer_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={outer_spin.articulation_type}, axis={outer_spin.axis}",
    )
    ctx.expect_contact(
        turret,
        optical_body,
        elem_a="turret_disk",
        elem_b="nosepiece_collar",
        contact_tol=0.001,
        name="turret seats against nosepiece collar",
    )

    return ctx.report()


object_model = build_object_model()
