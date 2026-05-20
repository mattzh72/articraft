from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
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


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_lab_microscope")

    enamel = model.material("warm_enamel", rgba=(0.82, 0.80, 0.72, 1.0))
    dark = model.material("matte_black", rgba=(0.025, 0.027, 0.028, 1.0))
    charcoal = model.material("charcoal_metal", rgba=(0.12, 0.13, 0.13, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    glass = model.material("lens_glass", rgba=(0.45, 0.65, 0.75, 0.55))
    white = model.material("white_mark", rgba=(0.95, 0.95, 0.88, 1.0))

    stand = model.part("stand")

    foot_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.36, 0.32, 0.035, corner_segments=10),
            0.040,
            center=True,
        ),
        "wide_rounded_foot",
    )
    stand.visual(
        foot_mesh,
        origin=Origin(xyz=(0.0, 0.035, 0.020)),
        material=enamel,
        name="wide_foot",
    )
    stand.visual(
        Box((0.060, 0.055, 0.500)),
        origin=Origin(xyz=(0.0, 0.130, 0.290)),
        material=enamel,
        name="straight_arm",
    )
    stand.visual(
        Box((0.082, 0.205, 0.055)),
        origin=Origin(xyz=(0.0, 0.042, 0.538)),
        material=enamel,
        name="head_bridge",
    )
    stand.visual(
        Cylinder(radius=0.027, length=0.105),
        origin=Origin(xyz=(0.0, -0.045, 0.4625)),
        material=charcoal,
        name="optical_tube",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.155),
        origin=Origin(xyz=(0.0, 0.042, 0.621), rpy=(-0.75, 0.0, 0.0)),
        material=charcoal,
        name="monocular_tube",
    )
    stand.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.0, 0.106, 0.690), rpy=(-0.75, 0.0, 0.0)),
        material=dark,
        name="eyepiece",
    )
    stand.visual(
        Box((0.115, 0.190, 0.035)),
        origin=Origin(xyz=(0.0, 0.040, 0.185)),
        material=enamel,
        name="stage_support_arm",
    )
    stand.visual(
        Box((0.100, 0.095, 0.032)),
        origin=Origin(xyz=(0.0, -0.045, 0.210)),
        material=enamel,
        name="slide_pedestal",
    )
    stand.visual(
        Box((0.180, 0.185, 0.016)),
        origin=Origin(xyz=(0.0, -0.045, 0.225)),
        material=charcoal,
        name="slide_support",
    )
    stand.visual(
        Box((0.018, 0.172, 0.012)),
        origin=Origin(xyz=(-0.055, -0.045, 0.238)),
        material=steel,
        name="rail_0",
    )
    stand.visual(
        Box((0.018, 0.172, 0.012)),
        origin=Origin(xyz=(0.055, -0.045, 0.238)),
        material=steel,
        name="rail_1",
    )

    stage_block = model.part("stage_block")
    stage_block.visual(
        Box((0.145, 0.095, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=charcoal,
        name="slide_block",
    )
    stage_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            [(-0.085, -0.085), (0.085, -0.085), (0.085, 0.085), (-0.085, 0.085)],
            [_circle_profile(0.018, 40)],
            0.014,
            center=True,
        ),
        "square_stage_plate",
    )
    stage_block.visual(
        stage_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=dark,
        name="stage_plate",
    )
    for idx, x in enumerate((-0.045, 0.045)):
        stage_block.visual(
            Box((0.050, 0.010, 0.006)),
            origin=Origin(xyz=(x, -0.052, 0.047)),
            material=steel,
            name=f"stage_clip_{idx}",
        )
    stage_block.visual(
        Box((0.025, 0.030, 0.024)),
        origin=Origin(xyz=(0.085, 0.043, 0.016)),
        material=charcoal,
        name="knob_boss",
    )

    stage_slide = model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage_block,
        origin=Origin(xyz=(0.0, -0.045, 0.244)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.08, lower=-0.035, upper=0.035),
    )

    stage_knob = model.part("stage_knob")
    stage_knob.visual(
        Cylinder(radius=0.005, length=0.045),
        origin=Origin(xyz=(0.0225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft",
    )
    stage_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.045,
            0.022,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="knurled", count=34, depth=0.0010, helix_angle_deg=18.0),
        ),
        "knurled_stage_knob",
    )
    stage_knob.visual(
        stage_knob_mesh,
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="knob_cap",
    )
    stage_knob.visual(
        Cylinder(radius=0.003, length=0.004),
        origin=Origin(xyz=(0.069, 0.013, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white,
        name="indicator_dot",
    )

    model.articulation(
        "stage_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=stage_block,
        child=stage_knob,
        origin=Origin(xyz=(0.0975, 0.043, 0.016)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    objective_turret = model.part("objective_turret")
    objective_turret.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=charcoal,
        name="turret_disk",
    )
    objective_turret.visual(
        Cylinder(radius=0.011, length=0.052),
        origin=Origin(xyz=(0.032, 0.000, -0.044)),
        material=dark,
        name="objective_0",
    )
    objective_turret.visual(
        Cylinder(radius=0.0075, length=0.010),
        origin=Origin(xyz=(0.032, 0.000, -0.075)),
        material=glass,
        name="objective_0_tip",
    )
    objective_turret.visual(
        Cylinder(radius=0.009, length=0.044),
        origin=Origin(xyz=(-0.016, 0.028, -0.040)),
        material=dark,
        name="objective_1",
    )
    objective_turret.visual(
        Cylinder(radius=0.0062, length=0.010),
        origin=Origin(xyz=(-0.016, 0.028, -0.067)),
        material=glass,
        name="objective_1_tip",
    )
    objective_turret.visual(
        Cylinder(radius=0.010, length=0.048),
        origin=Origin(xyz=(-0.016, -0.028, -0.042)),
        material=dark,
        name="objective_2",
    )
    objective_turret.visual(
        Cylinder(radius=0.0068, length=0.010),
        origin=Origin(xyz=(-0.016, -0.028, -0.071)),
        material=glass,
        name="objective_2_tip",
    )

    model.articulation(
        "turret_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=objective_turret,
        origin=Origin(xyz=(0.0, -0.045, 0.410)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    stage_block = object_model.get_part("stage_block")
    stage_knob = object_model.get_part("stage_knob")
    objective_turret = object_model.get_part("objective_turret")
    stage_slide = object_model.get_articulation("stage_slide")
    knob_spin = object_model.get_articulation("stage_knob_spin")
    turret_spin = object_model.get_articulation("turret_spin")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((float(lo[i]) + float(hi[i])) * 0.5 for i in range(3))

    ctx.expect_gap(
        stage_block,
        stand,
        axis="z",
        positive_elem="slide_block",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="cross slide block rides on support rail",
    )
    ctx.expect_overlap(
        stage_block,
        stand,
        axes="xy",
        elem_a="slide_block",
        elem_b="rail_0",
        min_overlap=0.015,
        name="slide block remains engaged with rail at rest",
    )
    ctx.expect_overlap(
        objective_turret,
        stage_block,
        axes="xy",
        elem_a="turret_disk",
        elem_b="stage_plate",
        min_overlap=0.08,
        name="objective nosepiece is centered over square stage",
    )
    ctx.expect_gap(
        objective_turret,
        stage_block,
        axis="z",
        positive_elem="objective_0_tip",
        negative_elem="stage_plate",
        min_gap=0.030,
        max_gap=0.080,
        name="objective tip clears the stage",
    )

    rest_stage = ctx.part_world_position(stage_block)
    with ctx.pose({stage_slide: 0.035}):
        fore_stage = ctx.part_world_position(stage_block)
        ctx.expect_overlap(
            stage_block,
            stand,
            axes="xy",
            elem_a="slide_block",
            elem_b="rail_0",
            min_overlap=0.015,
            name="fore slide travel stays on rail",
        )
    with ctx.pose({stage_slide: -0.035}):
        aft_stage = ctx.part_world_position(stage_block)
        ctx.expect_overlap(
            stage_block,
            stand,
            axes="xy",
            elem_a="slide_block",
            elem_b="rail_0",
            min_overlap=0.015,
            name="aft slide travel stays on rail",
        )
    ctx.check(
        "stage block translates fore and aft",
        rest_stage is not None
        and fore_stage is not None
        and aft_stage is not None
        and fore_stage[1] > rest_stage[1] + 0.030
        and aft_stage[1] < rest_stage[1] - 0.030,
        details=f"rest={rest_stage}, fore={fore_stage}, aft={aft_stage}",
    )

    with ctx.pose({turret_spin: 0.0}):
        objective_zero = _aabb_center(
            ctx.part_element_world_aabb(objective_turret, elem="objective_0")
        )
    with ctx.pose({turret_spin: math.pi / 2.0}):
        objective_quarter = _aabb_center(
            ctx.part_element_world_aabb(objective_turret, elem="objective_0")
        )
    ctx.check(
        "objective turret rotates around optical axis",
        objective_zero is not None
        and objective_quarter is not None
        and abs(objective_zero[0] - objective_quarter[0]) > 0.025
        and abs(objective_zero[1] - objective_quarter[1]) > 0.025,
        details=f"q0={objective_zero}, q90={objective_quarter}",
    )

    with ctx.pose({knob_spin: 0.0}):
        knob_dot_zero = _aabb_center(
            ctx.part_element_world_aabb(stage_knob, elem="indicator_dot")
        )
    with ctx.pose({knob_spin: math.pi / 2.0}):
        knob_dot_quarter = _aabb_center(
            ctx.part_element_world_aabb(stage_knob, elem="indicator_dot")
        )
    ctx.check(
        "stage control knob rotates on horizontal shaft",
        knob_dot_zero is not None
        and knob_dot_quarter is not None
        and knob_dot_quarter[2] > knob_dot_zero[2] + 0.008,
        details=f"q0={knob_dot_zero}, q90={knob_dot_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
