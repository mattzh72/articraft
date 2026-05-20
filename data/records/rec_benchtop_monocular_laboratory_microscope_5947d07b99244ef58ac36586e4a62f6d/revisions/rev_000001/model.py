from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _c_collar_profile() -> list[tuple[float, float]]:
    """A C-shaped guide collar profile that wraps the column without occupying it."""
    return [
        (-0.026, -0.041),
        (0.041, -0.041),
        (0.041, 0.041),
        (-0.026, 0.041),
        (-0.026, 0.027),
        (0.020, 0.027),
        (0.020, -0.027),
        (-0.026, -0.027),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_monocular_microscope")

    enamel = Material("warm_white_enamel", rgba=(0.86, 0.84, 0.78, 1.0))
    dark = Material("matte_black", rgba=(0.02, 0.022, 0.024, 1.0))
    metal = Material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    stage_mat = Material("blackened_stage", rgba=(0.035, 0.037, 0.04, 1.0))
    glass = Material("dark_glass", rgba=(0.02, 0.05, 0.055, 1.0))

    stand = model.part("stand")

    base_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(rounded_rect_profile(0.40, 0.26, 0.045), 0.055),
        "rounded_heavy_base",
    )
    stand.visual(
        base_mesh,
        origin=Origin(xyz=(0.05, 0.0, 0.0275)),
        material=dark,
        name="heavy_base",
    )
    stand.visual(
        Cylinder(radius=0.035, length=0.030),
        origin=Origin(xyz=(-0.08, 0.0, 0.070)),
        material=metal,
        name="column_boss",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.62),
        origin=Origin(xyz=(-0.08, 0.0, 0.365)),
        material=metal,
        name="column",
    )
    stand.visual(
        Box((0.175, 0.055, 0.060)),
        origin=Origin(xyz=(-0.005, 0.0, 0.190)),
        material=enamel,
        name="stage_support",
    )
    stand.visual(
        Box((0.175, 0.135, 0.018)),
        origin=Origin(xyz=(0.07, 0.0, 0.220)),
        material=stage_mat,
        name="stage_plate",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.07, 0.0, 0.231)),
        material=glass,
        name="stage_aperture",
    )
    stand.visual(
        Box((0.105, 0.009, 0.005)),
        origin=Origin(xyz=(0.060, -0.043, 0.2315)),
        material=metal,
        name="slide_clip_0",
    )
    stand.visual(
        Box((0.105, 0.009, 0.005)),
        origin=Origin(xyz=(0.060, 0.043, 0.2315)),
        material=metal,
        name="slide_clip_1",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.035),
        origin=Origin(xyz=(0.07, 0.0, 0.185)),
        material=metal,
        name="condenser",
    )

    carriage = model.part("carriage")
    collar_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(_c_collar_profile(), 0.130),
        "c_shaped_sliding_collar",
    )
    carriage.visual(
        collar_mesh,
        material=enamel,
        name="sliding_collar",
    )
    carriage.visual(
        Box((0.112, 0.066, 0.070)),
        origin=Origin(xyz=(0.090, 0.0, 0.015)),
        material=enamel,
        name="arm_bridge",
    )
    carriage.visual(
        Box((0.085, 0.074, 0.070)),
        origin=Origin(xyz=(0.150, 0.0, 0.018)),
        material=enamel,
        name="optical_head",
    )
    carriage.visual(
        Cylinder(radius=0.022, length=0.130),
        origin=Origin(xyz=(0.096, 0.0, 0.096), rpy=(0.0, -0.62, 0.0)),
        material=enamel,
        name="eyepiece_tube",
    )
    carriage.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(0.045, 0.0, 0.167), rpy=(0.0, -0.62, 0.0)),
        material=dark,
        name="eyepiece",
    )
    carriage.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(xyz=(0.150, 0.0, -0.033)),
        material=metal,
        name="nosepiece_neck",
    )
    carriage.visual(
        Cylinder(radius=0.038, length=0.014),
        origin=Origin(xyz=(0.150, 0.0, -0.055)),
        material=metal,
        name="objective_turret",
    )
    carriage.visual(
        Cylinder(radius=0.009, length=0.080),
        origin=Origin(xyz=(0.150, 0.0, -0.102)),
        material=dark,
        name="active_objective",
    )
    carriage.visual(
        Cylinder(radius=0.007, length=0.070),
        origin=Origin(xyz=(0.126, 0.020, -0.095)),
        material=dark,
        name="objective_0",
    )
    carriage.visual(
        Cylinder(radius=0.007, length=0.070),
        origin=Origin(xyz=(0.126, -0.020, -0.095)),
        material=dark,
        name="objective_1",
    )

    focus_knobs = model.part("focus_knobs")
    focus_knobs.visual(
        Cylinder(radius=0.006, length=0.140),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="knob_shaft",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.028,
            body_style="cylindrical",
            edge_radius=0.002,
            grip=KnobGrip(style="fluted", count=28, depth=0.0016),
        ),
        "fluted_focus_knob",
    )
    focus_knobs.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, -0.066, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="knob_0",
    )
    focus_knobs.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.066, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="knob_1",
    )

    model.articulation(
        "stand_to_carriage",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=carriage,
        origin=Origin(xyz=(-0.08, 0.0, 0.382)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.12, lower=0.0, upper=0.16),
    )
    model.articulation(
        "carriage_to_focus_knobs",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=focus_knobs,
        origin=Origin(xyz=(0.075, 0.0, 0.005)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    carriage = object_model.get_part("carriage")
    focus_knobs = object_model.get_part("focus_knobs")
    focus_slide = object_model.get_articulation("stand_to_carriage")
    knob_spin = object_model.get_articulation("carriage_to_focus_knobs")

    ctx.allow_overlap(
        carriage,
        focus_knobs,
        elem_a="arm_bridge",
        elem_b="knob_shaft",
        reason="The shared focus shaft is intentionally captured through the carriage bearing block.",
    )

    with ctx.pose({focus_slide: 0.0, knob_spin: 0.0}):
        ctx.expect_within(
            stand,
            carriage,
            axes="xy",
            inner_elem="column",
            outer_elem="sliding_collar",
            margin=0.002,
            name="column is centered inside sliding collar footprint",
        )
        ctx.expect_overlap(
            carriage,
            stand,
            axes="z",
            elem_a="sliding_collar",
            elem_b="column",
            min_overlap=0.12,
            name="collar remains engaged on column at low focus",
        )
        ctx.expect_gap(
            carriage,
            stand,
            axis="z",
            positive_elem="active_objective",
            negative_elem="stage_plate",
            min_gap=0.005,
            max_gap=0.030,
            name="objective clears specimen stage at close focus",
        )
        ctx.expect_overlap(
            focus_knobs,
            carriage,
            axes="xz",
            elem_a="knob_shaft",
            elem_b="arm_bridge",
            min_overlap=0.010,
            name="focus shaft passes through carriage bearing block",
        )
        low_position = ctx.part_world_position(carriage)

    with ctx.pose({focus_slide: 0.16, knob_spin: math.pi / 2.0}):
        ctx.expect_overlap(
            carriage,
            stand,
            axes="z",
            elem_a="sliding_collar",
            elem_b="column",
            min_overlap=0.12,
            name="collar remains engaged on column at high focus",
        )
        high_position = ctx.part_world_position(carriage)
        spun_position = ctx.part_world_position(focus_knobs)

    ctx.check(
        "carriage translates upward for focus",
        low_position is not None
        and high_position is not None
        and high_position[2] > low_position[2] + 0.15,
        details=f"low={low_position}, high={high_position}",
    )
    ctx.check(
        "focus knobs spin about fixed shared axis",
        high_position is not None
        and spun_position is not None
        and abs(spun_position[2] - (high_position[2] + 0.005)) < 1e-6,
        details=f"carriage={high_position}, knobs={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()
