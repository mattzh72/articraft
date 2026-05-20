from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monocular_microscope")

    body_metal = model.material("body_metal", rgba=(0.84, 0.85, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.14, 1.0))
    stage_black = model.material("stage_black", rgba=(0.10, 0.10, 0.11, 1.0))
    objective_metal = model.material("objective_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    knob_black = model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.18, 0.20, 0.22, 0.55))

    stand = model.part("stand")
    stand.visual(
        Box((0.230, 0.175, 0.020)),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=body_metal,
        name="base_lower",
    )
    stand.visual(
        Box((0.190, 0.135, 0.026)),
        origin=Origin(xyz=(0.000, -0.004, 0.033)),
        material=body_metal,
        name="base_mid",
    )
    stand.visual(
        Box((0.120, 0.092, 0.018)),
        origin=Origin(xyz=(0.000, -0.008, 0.055)),
        material=body_metal,
        name="base_top",
    )
    stand.visual(
        Box((0.042, 0.042, 0.120)),
        origin=Origin(xyz=(0.000, 0.000, 0.115)),
        material=body_metal,
        name="stage_pillar",
    )
    stand.visual(
        Box((0.102, 0.116, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.179)),
        material=stage_black,
        name="stage_bed",
    )
    stand.visual(
        Box((0.024, 0.074, 0.030)),
        origin=Origin(xyz=(0.063, 0.000, 0.168)),
        material=dark_trim,
        name="stage_bracket",
    )
    stand.visual(
        Box((0.042, 0.032, 0.240)),
        origin=Origin(xyz=(0.000, -0.062, 0.175)),
        material=body_metal,
        name="arm_column",
    )
    stand.visual(
        Box((0.030, 0.006, 0.190)),
        origin=Origin(xyz=(0.000, -0.045, 0.220)),
        material=body_metal,
        name="guide_column",
    )
    stand.visual(
        Box((0.050, 0.032, 0.046)),
        origin=Origin(xyz=(0.000, -0.066, 0.287)),
        material=body_metal,
        name="shoulder",
    )
    _add_member(
        stand,
        (0.000, -0.058, 0.282),
        (0.000, -0.086, 0.350),
        0.019,
        body_metal,
        name="rear_spine",
    )
    _add_member(
        stand,
        (0.000, -0.086, 0.350),
        (0.000, -0.072, 0.386),
        0.015,
        body_metal,
        name="head_post",
    )

    stage_carriage = model.part("stage_carriage")
    stage_carriage.visual(
        Box((0.126, 0.092, 0.004)),
        origin=Origin(xyz=(0.000, 0.012, 0.003)),
        material=stage_black,
        name="stage_plate",
    )
    stage_carriage.visual(
        Box((0.126, 0.012, 0.004)),
        origin=Origin(xyz=(0.000, -0.038, 0.003)),
        material=stage_black,
        name="rear_lip",
    )
    stage_carriage.visual(
        Box((0.010, 0.074, 0.002)),
        origin=Origin(xyz=(-0.022, 0.010, 0.006)),
        material=dark_trim,
        name="guide_left",
    )
    stage_carriage.visual(
        Box((0.010, 0.074, 0.002)),
        origin=Origin(xyz=(0.022, 0.010, 0.006)),
        material=dark_trim,
        name="guide_right",
    )
    stage_carriage.visual(
        Box((0.004, 0.074, 0.003)),
        origin=Origin(xyz=(-0.022, 0.010, 0.0045)),
        material=stage_black,
        name="guide_left_riser",
    )
    stage_carriage.visual(
        Box((0.004, 0.074, 0.003)),
        origin=Origin(xyz=(0.022, 0.010, 0.0045)),
        material=stage_black,
        name="guide_right_riser",
    )
    stage_carriage.visual(
        Box((0.018, 0.014, 0.004)),
        origin=Origin(xyz=(-0.046, 0.042, 0.005)),
        material=dark_trim,
        name="slide_clip",
    )
    stage_carriage.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.000, -0.022, 0.005)),
        material=dark_trim,
        name="specimen_stop",
    )

    body_carriage = model.part("body_carriage")
    body_carriage.visual(
        Cylinder(radius=0.024, length=0.032),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=body_metal,
        name="lower_housing",
    )
    body_carriage.visual(
        Box((0.058, 0.021, 0.040)),
        origin=Origin(xyz=(0.000, -0.0195, 0.026)),
        material=body_metal,
        name="focus_bridge",
    )
    body_carriage.visual(
        Box((0.048, 0.012, 0.092)),
        origin=Origin(xyz=(0.000, -0.036, 0.032)),
        material=dark_trim,
        name="focus_shoe",
    )
    body_carriage.visual(
        Box((0.046, 0.030, 0.036)),
        origin=Origin(xyz=(0.000, 0.006, 0.052)),
        material=body_metal,
        name="upper_head",
    )
    _add_member(
        body_carriage,
        (0.000, -0.006, 0.060),
        (0.000, -0.040, 0.148),
        0.018,
        body_metal,
        name="eyepiece_tube",
    )
    _add_member(
        body_carriage,
        (0.000, -0.040, 0.148),
        (0.000, -0.052, 0.192),
        0.011,
        dark_trim,
        name="ocular_sleeve",
    )
    _add_member(
        body_carriage,
        (0.000, -0.052, 0.192),
        (0.000, -0.052, 0.224),
        0.013,
        knob_black,
        name="eyecup",
    )
    _add_member(
        body_carriage,
        (0.000, -0.052, 0.212),
        (0.000, -0.052, 0.222),
        0.008,
        glass_dark,
        name="eyepiece_glass",
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, -0.005)),
        material=dark_trim,
        name="hub",
    )
    nosepiece.visual(
        Cylinder(radius=0.023, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
        material=dark_trim,
        name="turret",
    )
    nosepiece.visual(
        Cylinder(radius=0.007, length=0.024),
        origin=Origin(xyz=(0.000, 0.014, -0.020)),
        material=objective_metal,
        name="objective_0",
    )
    nosepiece.visual(
        Cylinder(radius=0.0052, length=0.012),
        origin=Origin(xyz=(0.000, 0.014, -0.038)),
        material=dark_trim,
        name="objective_0_tip",
    )
    nosepiece.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(-0.012, -0.007, -0.018)),
        material=objective_metal,
        name="objective_1",
    )
    nosepiece.visual(
        Cylinder(radius=0.0047, length=0.012),
        origin=Origin(xyz=(-0.012, -0.007, -0.034)),
        material=dark_trim,
        name="objective_1_tip",
    )
    nosepiece.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.012, -0.007, -0.017)),
        material=objective_metal,
        name="objective_2",
    )
    nosepiece.visual(
        Cylinder(radius=0.0042, length=0.012),
        origin=Origin(xyz=(0.012, -0.007, -0.032)),
        material=dark_trim,
        name="objective_2_tip",
    )

    stage_knob_outer = model.part("stage_knob_outer")
    stage_knob_outer.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(0.003, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=objective_metal,
        name="shaft",
    )
    stage_knob_outer.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.012, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="outer_knob",
    )
    stage_knob_outer.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.021, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="outer_cap",
    )

    stage_knob_inner = model.part("stage_knob_inner")
    stage_knob_inner.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.005, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="inner_knob",
    )
    stage_knob_inner.visual(
        Cylinder(radius=0.006, length=0.005),
        origin=Origin(xyz=(0.0125, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=objective_metal,
        name="inner_cap",
    )

    model.articulation(
        "focus_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=body_carriage,
        origin=Origin(xyz=(0.000, 0.000, 0.272)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.08,
            lower=0.0,
            upper=0.055,
        ),
    )
    model.articulation(
        "nosepiece_spin",
        ArticulationType.CONTINUOUS,
        parent=body_carriage,
        child=nosepiece,
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0),
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage_carriage,
        origin=Origin(xyz=(0.000, 0.000, 0.184)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.04,
            lower=-0.018,
            upper=0.018,
        ),
    )
    model.articulation(
        "stage_knob_outer_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=stage_knob_outer,
        origin=Origin(xyz=(0.075, 0.000, 0.168)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )
    model.articulation(
        "stage_knob_inner_spin",
        ArticulationType.CONTINUOUS,
        parent=stage_knob_outer,
        child=stage_knob_inner,
        origin=Origin(xyz=(0.024, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.1, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    stage_carriage = object_model.get_part("stage_carriage")
    body_carriage = object_model.get_part("body_carriage")
    nosepiece = object_model.get_part("nosepiece")
    stage_knob_outer = object_model.get_part("stage_knob_outer")
    stage_knob_inner = object_model.get_part("stage_knob_inner")

    focus_slide = object_model.get_articulation("focus_slide")
    stage_slide = object_model.get_articulation("stage_slide")
    nosepiece_spin = object_model.get_articulation("nosepiece_spin")
    stage_knob_outer_spin = object_model.get_articulation("stage_knob_outer_spin")
    stage_knob_inner_spin = object_model.get_articulation("stage_knob_inner_spin")

    ctx.expect_gap(
        stage_carriage,
        stand,
        axis="z",
        positive_elem="stage_plate",
        negative_elem="stage_bed",
        min_gap=0.002,
        max_gap=0.004,
        name="stage carriage rides just above the stage bed",
    )
    ctx.expect_origin_distance(
        stage_knob_outer,
        stage_knob_inner,
        axes="yz",
        max_dist=0.0005,
        name="stage controls stay coaxial on the bracket",
    )
    ctx.expect_origin_gap(
        stage_knob_inner,
        stage_knob_outer,
        axis="x",
        min_gap=0.020,
        max_gap=0.030,
        name="inner stage knob sits outboard of the outer knob",
    )

    with ctx.pose({focus_slide: 0.0}):
        ctx.expect_gap(
            nosepiece,
            stage_carriage,
            axis="z",
            positive_elem="objective_0_tip",
            negative_elem="guide_left",
            min_gap=0.020,
            max_gap=0.035,
            name="rest focus leaves a realistic objective gap over the stage",
        )

    focus_upper = focus_slide.motion_limits.upper if focus_slide.motion_limits is not None else None
    rest_body_pos = ctx.part_world_position(body_carriage)
    if focus_upper is not None:
        with ctx.pose({focus_slide: focus_upper}):
            ctx.expect_gap(
                nosepiece,
                stage_carriage,
                axis="z",
                positive_elem="objective_0_tip",
                negative_elem="guide_left",
                min_gap=0.070,
                name="focus travel lifts the objective assembly away from the stage",
            )
            raised_body_pos = ctx.part_world_position(body_carriage)
        ctx.check(
            "focus slide raises the body carriage",
            rest_body_pos is not None
            and raised_body_pos is not None
            and raised_body_pos[2] > rest_body_pos[2] + 0.045,
            details=f"rest={rest_body_pos}, raised={raised_body_pos}",
        )

    stage_lower = stage_slide.motion_limits.lower if stage_slide.motion_limits is not None else None
    stage_upper = stage_slide.motion_limits.upper if stage_slide.motion_limits is not None else None
    if stage_lower is not None:
        with ctx.pose({stage_slide: stage_lower}):
            ctx.expect_overlap(
                stage_carriage,
                stand,
                axes="x",
                elem_a="stage_plate",
                elem_b="stage_bed",
                min_overlap=0.080,
                name="stage carriage stays captured on the bed at left travel",
            )
    rest_stage_pos = ctx.part_world_position(stage_carriage)
    if stage_upper is not None:
        with ctx.pose({stage_slide: stage_upper}):
            ctx.expect_overlap(
                stage_carriage,
                stand,
                axes="x",
                elem_a="stage_plate",
                elem_b="stage_bed",
                min_overlap=0.080,
                name="stage carriage stays captured on the bed at right travel",
            )
            moved_stage_pos = ctx.part_world_position(stage_carriage)
        ctx.check(
            "stage carriage moves along the x axis",
            rest_stage_pos is not None
            and moved_stage_pos is not None
            and moved_stage_pos[0] > rest_stage_pos[0] + 0.015,
            details=f"rest={rest_stage_pos}, moved={moved_stage_pos}",
        )

    ctx.check(
        "nosepiece uses continuous rotation",
        nosepiece_spin.motion_limits is not None
        and nosepiece_spin.motion_limits.lower is None
        and nosepiece_spin.motion_limits.upper is None,
        details=f"limits={nosepiece_spin.motion_limits}",
    )
    ctx.check(
        "stage controls use continuous rotation",
        stage_knob_outer_spin.motion_limits is not None
        and stage_knob_outer_spin.motion_limits.lower is None
        and stage_knob_outer_spin.motion_limits.upper is None
        and stage_knob_inner_spin.motion_limits is not None
        and stage_knob_inner_spin.motion_limits.lower is None
        and stage_knob_inner_spin.motion_limits.upper is None,
        details=(
            f"outer={stage_knob_outer_spin.motion_limits}, "
            f"inner={stage_knob_inner_spin.motion_limits}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
