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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_cast_base():
    """Low, bench-scale cast base with soft radiused edges."""
    base = (
        cq.Workplane("XY")
        .box(0.260, 0.180, 0.035, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.004)
    )
    return base


def _stage_plate():
    """Compact mechanical slide stage with a true central aperture."""
    plate = (
        cq.Workplane("XY")
        .box(0.115, 0.080, 0.008, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    aperture = (
        cq.Workplane("XY")
        .box(0.034, 0.022, 0.020, centered=(True, True, True))
        .translate((0.0, 0.0, 0.004))
    )
    return plate.cut(aperture)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clinical_monocular_microscope")

    enamel = model.material("warm_enamel", rgba=(0.86, 0.88, 0.84, 1.0))
    dark = model.material("blackened_metal", rgba=(0.05, 0.055, 0.055, 1.0))
    satin = model.material("satin_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    glass = model.material("pale_slide_glass", rgba=(0.72, 0.92, 1.0, 0.45))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_cast_base(), "cast_base", tolerance=0.001),
        material=enamel,
        name="cast_base",
    )
    base.visual(
        Box((0.080, 0.070, 0.010)),
        origin=Origin(xyz=(0.0, 0.055, 0.040)),
        material=enamel,
        name="column_boss",
    )

    column = model.part("column")
    column.visual(
        Box((0.070, 0.050, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=enamel,
        name="pedestal",
    )
    column.visual(
        Box((0.035, 0.028, 0.340)),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=satin,
        name="column_post",
    )
    column.visual(
        Box((0.013, 0.004, 0.245)),
        origin=Origin(xyz=(0.0, -0.016, 0.205)),
        material=dark,
        name="rack_strip",
    )
    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.055, 0.045)),
    )

    stage_guide = model.part("stage_guide")
    stage_guide.visual(
        Box((0.140, 0.112, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=satin,
        name="guide_plate",
    )
    stage_guide.visual(
        Box((0.055, 0.050, 0.020)),
        origin=Origin(xyz=(0.0, 0.031, -0.018)),
        material=enamel,
        name="stage_bracket",
    )
    model.articulation(
        "column_to_stage_guide",
        ArticulationType.FIXED,
        parent=column,
        child=stage_guide,
        origin=Origin(xyz=(0.0, -0.070, 0.085)),
    )

    stage = model.part("stage")
    stage.visual(
        mesh_from_cadquery(_stage_plate(), "stage_plate", tolerance=0.0008),
        material=dark,
        name="stage_plate",
    )
    stage.visual(
        Box((0.076, 0.026, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=glass,
        name="glass_slide",
    )
    stage.visual(
        Box((0.006, 0.065, 0.004)),
        origin=Origin(xyz=(-0.049, 0.0, 0.010)),
        material=chrome,
        name="slide_clip_0",
    )
    stage.visual(
        Box((0.006, 0.065, 0.004)),
        origin=Origin(xyz=(0.049, 0.0, 0.010)),
        material=chrome,
        name="slide_clip_1",
    )
    model.articulation(
        "stage_guide_to_stage",
        ArticulationType.PRISMATIC,
        parent=stage_guide,
        child=stage,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=-0.018, upper=0.014),
    )

    carriage = model.part("focusing_carriage")
    carriage.visual(
        Box((0.070, 0.026, 0.075)),
        origin=Origin(xyz=(0.0, -0.013, 0.0)),
        material=enamel,
        name="carriage_block",
    )
    carriage.visual(
        Box((0.045, 0.082, 0.028)),
        origin=Origin(xyz=(0.0, -0.047, 0.015)),
        material=enamel,
        name="forward_arm",
    )
    carriage.visual(
        Cylinder(radius=0.023, length=0.038),
        origin=Origin(xyz=(0.0, -0.086, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=enamel,
        name="head_boss",
    )
    carriage.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(0.0, -0.086, -0.033)),
        material=chrome,
        name="nosepiece_socket",
    )
    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.014, 0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.05, lower=-0.015, upper=0.045),
    )

    eyepiece = model.part("eyepiece_tube")
    eyepiece_angle = -math.radians(20.0)
    eyepiece_axis = (0.0, math.sin(math.radians(20.0)), math.cos(math.radians(20.0)))
    eyepiece.visual(
        Box((0.030, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=chrome,
        name="tube_socket_foot",
    )
    eyepiece.visual(
        Cylinder(radius=0.014, length=0.130),
        origin=Origin(
            xyz=(
                0.0,
                eyepiece_axis[1] * 0.065,
                eyepiece_axis[2] * 0.065,
            ),
            rpy=(eyepiece_angle, 0.0, 0.0),
        ),
        material=chrome,
        name="inclined_tube",
    )
    eyepiece.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(
            xyz=(
                0.0,
                eyepiece_axis[1] * 0.143,
                eyepiece_axis[2] * 0.143,
            ),
            rpy=(eyepiece_angle, 0.0, 0.0),
        ),
        material=rubber,
        name="ocular_lens",
    )
    model.articulation(
        "carriage_to_eyepiece",
        ArticulationType.FIXED,
        parent=carriage,
        child=eyepiece,
        origin=Origin(xyz=(0.0, -0.086, 0.045)),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=chrome,
        name="turret_disk",
    )
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        turret.visual(
            Cylinder(radius=0.0065, length=0.044),
            origin=Origin(
                xyz=(0.020 * math.cos(angle), 0.020 * math.sin(angle), -0.034),
            ),
            material=dark if idx == 0 else satin,
            name=f"objective_{idx}",
        )
    model.articulation(
        "carriage_to_turret",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=turret,
        origin=Origin(xyz=(0.0, -0.086, -0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5),
    )

    coarse_knob = model.part("coarse_knob")
    coarse_knob.visual(
        Cylinder(radius=0.006, length=0.013),
        origin=Origin(xyz=(0.0065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="coarse_shaft",
    )
    coarse_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.028,
                body_style="cylindrical",
                grip=KnobGrip(style="fluted", count=28, depth=0.0012),
            ),
            "coarse_knob_grip",
        ),
        origin=Origin(xyz=(0.027, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="coarse_grip",
    )
    model.articulation(
        "carriage_to_coarse_knob",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=coarse_knob,
        origin=Origin(xyz=(0.035, -0.020, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    fine_knob = model.part("fine_knob")
    fine_knob.visual(
        Cylinder(radius=0.0045, length=0.011),
        origin=Origin(xyz=(0.0055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="fine_shaft",
    )
    fine_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.027,
                0.020,
                body_style="cylindrical",
                grip=KnobGrip(style="knurled", count=32, depth=0.0008, helix_angle_deg=20.0),
            ),
            "fine_knob_grip",
        ),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="fine_grip",
    )
    model.articulation(
        "carriage_to_fine_knob",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=fine_knob,
        origin=Origin(xyz=(0.035, -0.020, -0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    guide = object_model.get_part("stage_guide")
    stage = object_model.get_part("stage")
    carriage = object_model.get_part("focusing_carriage")
    coarse = object_model.get_part("coarse_knob")
    fine = object_model.get_part("fine_knob")
    eyepiece = object_model.get_part("eyepiece_tube")

    focus_slide = object_model.get_articulation("column_to_carriage")
    stage_slide = object_model.get_articulation("stage_guide_to_stage")
    turret_spin = object_model.get_articulation("carriage_to_turret")
    coarse_spin = object_model.get_articulation("carriage_to_coarse_knob")
    fine_spin = object_model.get_articulation("carriage_to_fine_knob")

    ctx.expect_gap(
        column,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="pedestal",
        negative_elem="column_boss",
        name="column pedestal sits on cast base boss",
    )
    ctx.expect_gap(
        column,
        carriage,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="column_post",
        negative_elem="carriage_block",
        name="focusing carriage is supported on the column face",
    )
    ctx.expect_gap(
        stage,
        guide,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="stage_plate",
        negative_elem="guide_plate",
        name="stage rides on the guide plate",
    )
    ctx.expect_gap(
        coarse,
        carriage,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="coarse_shaft",
        negative_elem="carriage_block",
        name="coarse knob shaft seats in the carriage side",
    )
    ctx.expect_gap(
        fine,
        carriage,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="fine_shaft",
        negative_elem="carriage_block",
        name="fine knob shaft seats in the carriage side",
    )
    ctx.expect_gap(
        eyepiece,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="tube_socket_foot",
        negative_elem="forward_arm",
        name="monocular eyepiece tube is seated on the optical head",
    )

    ctx.check(
        "required microscope motions are articulated",
        focus_slide.articulation_type == ArticulationType.PRISMATIC
        and stage_slide.articulation_type == ArticulationType.PRISMATIC
        and turret_spin.articulation_type == ArticulationType.CONTINUOUS
        and coarse_spin.articulation_type == ArticulationType.CONTINUOUS
        and fine_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"types: focus={focus_slide.articulation_type}, stage={stage_slide.articulation_type}, "
            f"turret={turret_spin.articulation_type}, coarse={coarse_spin.articulation_type}, "
            f"fine={fine_spin.articulation_type}"
        ),
    )

    carriage_rest = ctx.part_world_position(carriage)
    stage_rest = ctx.part_world_position(stage)
    with ctx.pose({focus_slide: 0.035, stage_slide: 0.012, turret_spin: math.pi / 2.0}):
        carriage_raised = ctx.part_world_position(carriage)
        stage_forward = ctx.part_world_position(stage)
        ctx.expect_gap(
            stage,
            guide,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="stage_plate",
            negative_elem="guide_plate",
            name="translated stage remains supported on its guide",
        )

    ctx.check(
        "focusing carriage slides upward on the column",
        carriage_rest is not None
        and carriage_raised is not None
        and carriage_raised[2] > carriage_rest[2] + 0.030,
        details=f"rest={carriage_rest}, raised={carriage_raised}",
    )
    ctx.check(
        "stage translates front to back on the guide",
        stage_rest is not None
        and stage_forward is not None
        and stage_forward[1] > stage_rest[1] + 0.010,
        details=f"rest={stage_rest}, moved={stage_forward}",
    )

    return ctx.report()


object_model = build_object_model()
