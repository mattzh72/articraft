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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WARM_WHITE = Material("warm_white_enamel", rgba=(0.82, 0.80, 0.72, 1.0))
DARK_BASE = Material("charcoal_cast_metal", rgba=(0.08, 0.085, 0.09, 1.0))
BLACK = Material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
SATIN_METAL = Material("satin_chrome", rgba=(0.70, 0.72, 0.70, 1.0))
STAGE_BLACK = Material("blackened_stage", rgba=(0.015, 0.017, 0.018, 1.0))
GLASS = Material("pale_slide_glass", rgba=(0.65, 0.85, 0.95, 0.38))


def _rounded_base_mesh():
    return (
        cq.Workplane("XY")
        .box(0.240, 0.180, 0.035)
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.004)
        .translate((0.0, 0.0, 0.0175))
    )


def _swept_arm_mesh():
    # A single extruded Y-Z side profile creates the cast rear column and the
    # forward swept arm as one supported stand casting.
    side_profile_yz = [
        (0.040, 0.035),
        (0.105, 0.035),
        (0.105, 0.332),
        (0.075, 0.356),
        (0.018, 0.348),
        (0.025, 0.326),
        (0.030, 0.300),
        (0.044, 0.290),
        (0.054, 0.282),
        (0.044, 0.084),
        (0.040, 0.084),
    ]
    return (
        cq.Workplane("YZ")
        .polyline(side_profile_yz)
        .close()
        .extrude(0.060, both=True)
        .edges("|X")
        .chamfer(0.003)
    )


def _stage_plate_mesh():
    aperture = cq.Workplane("XY").cylinder(0.030, 0.010)
    return (
        cq.Workplane("XY")
        .box(0.150, 0.095, 0.012)
        .edges("|Z")
        .fillet(0.006)
        .cut(aperture)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_lab_monocular_microscope")

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_rounded_base_mesh(), "heavy_rounded_base"),
        material=DARK_BASE,
        name="heavy_base",
    )
    stand.visual(
        mesh_from_cadquery(_swept_arm_mesh(), "swept_cast_arm"),
        material=WARM_WHITE,
        name="swept_arm",
    )
    stand.visual(
        Box((0.150, 0.088, 0.012)),
        origin=Origin(xyz=(0.0, -0.035, 0.176)),
        material=BLACK,
        name="stage_support_bed",
    )
    stand.visual(
        Box((0.050, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.005, 0.164)),
        material=WARM_WHITE,
        name="stage_bracket",
    )
    stand.visual(
        Box((0.078, 0.006, 0.063)),
        origin=Origin(xyz=(0.0, 0.007, 0.239)),
        material=SATIN_METAL,
        name="focus_guide_face",
    )
    stand.visual(
        Box((0.014, 0.082, 0.063)),
        origin=Origin(xyz=(-0.044, 0.049, 0.239)),
        material=WARM_WHITE,
        name="focus_guide_tab_0",
    )
    stand.visual(
        Box((0.014, 0.082, 0.063)),
        origin=Origin(xyz=(0.044, 0.049, 0.239)),
        material=WARM_WHITE,
        name="focus_guide_tab_1",
    )
    stand.visual(
        Box((0.160, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.067, 0.186)),
        material=SATIN_METAL,
        name="stage_rail_0",
    )
    stand.visual(
        Box((0.160, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.003, 0.186)),
        material=SATIN_METAL,
        name="stage_rail_1",
    )
    # Side bushings make the two separate focus knobs look mounted to the arm.
    stand.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.069, 0.050, 0.290), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=SATIN_METAL,
        name="coarse_bushing",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.069, 0.050, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=SATIN_METAL,
        name="fine_bushing",
    )

    stage = model.part("stage_carriage")
    stage.visual(
        mesh_from_cadquery(_stage_plate_mesh(), "moving_stage_plate"),
        material=STAGE_BLACK,
        name="stage_plate",
    )
    stage.visual(
        Box((0.075, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, -0.004, 0.008)),
        material=GLASS,
        name="specimen_slide",
    )
    for x in (-0.044, 0.044):
        stage.visual(
            Box((0.030, 0.006, 0.004)),
            origin=Origin(xyz=(x, -0.004, 0.012)),
            material=SATIN_METAL,
            name=f"slide_clip_{0 if x < 0.0 else 1}",
        )
    model.articulation(
        "stand_to_stage",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage,
        origin=Origin(xyz=(0.0, -0.035, 0.196)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.08, lower=-0.035, upper=0.035),
    )

    optical = model.part("optical_body")
    optical.visual(
        Box((0.074, 0.026, 0.075)),
        origin=Origin(xyz=(0.0, 0.078, -0.070)),
        material=WARM_WHITE,
        name="slide_carriage",
    )
    optical.visual(
        Box((0.058, 0.086, 0.030)),
        origin=Origin(xyz=(0.0, 0.037, -0.022)),
        material=WARM_WHITE,
        name="carriage_bridge",
    )
    optical.visual(
        Cylinder(radius=0.025, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=BLACK,
        name="body_tube",
    )
    optical.visual(
        Cylinder(radius=0.031, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=WARM_WHITE,
        name="nose_socket",
    )
    optical.visual(
        Cylinder(radius=0.026, length=0.080),
        origin=Origin(xyz=(0.0, 0.030, 0.078), rpy=(-0.78, 0.0, 0.0)),
        material=WARM_WHITE,
        name="monocular_head",
    )
    optical.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=(0.0, 0.070, 0.118), rpy=(-0.78, 0.0, 0.0)),
        material=BLACK,
        name="eyepiece",
    )
    model.articulation(
        "stand_to_optical",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=optical,
        origin=Origin(xyz=(0.0, -0.055, 0.315)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.04, lower=0.0, upper=0.060),
    )

    turret = model.part("objective_turret")
    turret.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=SATIN_METAL,
        name="turret_disk",
    )
    turret.visual(
        Cylinder(radius=0.007, length=0.038),
        origin=Origin(xyz=(0.000, -0.020, -0.028)),
        material=BLACK,
        name="objective_0",
    )
    turret.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.018, 0.010, -0.025)),
        material=SATIN_METAL,
        name="objective_1",
    )
    turret.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(-0.018, 0.010, -0.023)),
        material=SATIN_METAL,
        name="objective_2",
    )
    model.articulation(
        "optical_to_turret",
        ArticulationType.CONTINUOUS,
        parent=optical,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0),
    )

    coarse_knob = model.part("coarse_knob")
    coarse_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.026,
                body_style="cylindrical",
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=24, depth=0.0020),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            ),
            "coarse_focus_knob",
        ),
        material=BLACK,
        name="coarse_dial",
    )
    coarse_knob.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=SATIN_METAL,
        name="coarse_stem",
    )
    model.articulation(
        "stand_to_coarse",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=coarse_knob,
        origin=Origin(xyz=(0.087, 0.050, 0.290), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )

    fine_knob = model.part("fine_knob")
    fine_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.030,
                0.020,
                body_style="faceted",
                edge_radius=0.0010,
                grip=KnobGrip(style="ribbed", count=16, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "fine_focus_knob",
        ),
        material=BLACK,
        name="fine_dial",
    )
    fine_knob.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=SATIN_METAL,
        name="fine_stem",
    )
    model.articulation(
        "stand_to_fine",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=fine_knob,
        origin=Origin(xyz=(0.085, 0.050, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    stage = object_model.get_part("stage_carriage")
    optical = object_model.get_part("optical_body")
    turret = object_model.get_part("objective_turret")
    coarse = object_model.get_part("coarse_knob")
    fine = object_model.get_part("fine_knob")

    stage_slide = object_model.get_articulation("stand_to_stage")
    focus_slide = object_model.get_articulation("stand_to_optical")

    ctx.allow_overlap(
        coarse,
        stand,
        elem_a="coarse_stem",
        elem_b="coarse_bushing",
        reason="The coarse focus knob stem is intentionally captured inside its side bushing.",
    )
    ctx.expect_within(
        coarse,
        stand,
        axes="yz",
        inner_elem="coarse_stem",
        outer_elem="coarse_bushing",
        margin=0.0,
        name="coarse stem is centered in its bushing",
    )
    ctx.expect_overlap(
        coarse,
        stand,
        axes="x",
        elem_a="coarse_stem",
        elem_b="coarse_bushing",
        min_overlap=0.010,
        name="coarse stem remains inserted in the bushing",
    )
    ctx.allow_overlap(
        fine,
        stand,
        elem_a="fine_stem",
        elem_b="fine_bushing",
        reason="The fine focus knob stem is intentionally captured inside its smaller side bushing.",
    )
    ctx.expect_within(
        fine,
        stand,
        axes="yz",
        inner_elem="fine_stem",
        outer_elem="fine_bushing",
        margin=0.0,
        name="fine stem is centered in its bushing",
    )
    ctx.expect_overlap(
        fine,
        stand,
        axes="x",
        elem_a="fine_stem",
        elem_b="fine_bushing",
        min_overlap=0.010,
        name="fine stem remains inserted in the bushing",
    )

    ctx.expect_gap(
        stage,
        stand,
        axis="z",
        positive_elem="stage_plate",
        negative_elem="stage_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="stage rides on the support rails",
    )
    ctx.expect_overlap(
        stage,
        stand,
        axes="xy",
        elem_a="stage_plate",
        elem_b="stage_support_bed",
        min_overlap=0.060,
        name="rectangular stage is supported by the bed footprint",
    )
    ctx.expect_gap(
        turret,
        stage,
        axis="z",
        positive_elem="objective_0",
        negative_elem="stage_plate",
        min_gap=0.003,
        name="low objective clears the specimen stage",
    )
    ctx.expect_gap(
        coarse,
        fine,
        axis="z",
        min_gap=0.010,
        name="coarse and fine knobs are separate controls",
    )

    rest_stage = ctx.part_world_position(stage)
    with ctx.pose({stage_slide: 0.030}):
        shifted_stage = ctx.part_world_position(stage)
    ctx.check(
        "stage carriage slides left to right",
        rest_stage is not None
        and shifted_stage is not None
        and shifted_stage[0] > rest_stage[0] + 0.025,
        details=f"rest={rest_stage}, shifted={shifted_stage}",
    )

    rest_optical = ctx.part_world_position(optical)
    with ctx.pose({focus_slide: 0.050}):
        raised_optical = ctx.part_world_position(optical)
        ctx.expect_gap(
            turret,
            stage,
            axis="z",
            positive_elem="objective_0",
            negative_elem="stage_plate",
            min_gap=0.040,
            name="focus slide raises the optical body",
        )
    ctx.check(
        "optical body carriage slides vertically",
        rest_optical is not None
        and raised_optical is not None
        and raised_optical[2] > rest_optical[2] + 0.045,
        details=f"rest={rest_optical}, raised={raised_optical}",
    )

    return ctx.report()


object_model = build_object_model()
