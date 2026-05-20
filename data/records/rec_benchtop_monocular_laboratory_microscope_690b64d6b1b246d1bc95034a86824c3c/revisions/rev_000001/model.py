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


COLUMN_Y = -0.032
FOCUS_Z0 = 0.105
FOCUS_TRAVEL = 0.075
STAGE_Y0 = 0.072
STAGE_TRAVEL = 0.018
OPTICAL_TILT = math.radians(32.0)


def _optical_offset(distance: float) -> tuple[float, float, float]:
    return (0.0, -math.sin(OPTICAL_TILT) * distance, math.cos(OPTICAL_TILT) * distance)


def _build_base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(0.130, 0.185, 0.016)
        .translate((0.0, 0.020, 0.008))
        .edges("|Z")
        .fillet(0.010)
    )
    heel = (
        cq.Workplane("XY")
        .box(0.064, 0.072, 0.040)
        .translate((0.0, -0.030, 0.028))
        .edges("|Z")
        .fillet(0.008)
    )
    gusset = (
        cq.Workplane("XY")
        .box(0.042, 0.050, 0.090)
        .translate((0.0, -0.030, 0.072))
        .edges("|Z")
        .fillet(0.006)
    )
    column = cq.Workplane("XY").circle(0.017).extrude(0.220).translate((0.0, COLUMN_Y, 0.025))
    return foot.union(heel).union(gusset).union(column)


def _build_carriage_casting() -> cq.Workplane:
    sleeve = cq.Workplane("XY").box(0.055, 0.050, 0.060).edges("|Z").fillet(0.004)
    sleeve = sleeve.cut(cq.Workplane("XY").circle(0.017).extrude(0.072).translate((0.0, 0.0, -0.036)))
    return sleeve


def _build_stage_ring_knob() -> cq.Workplane:
    outer_ring = cq.Workplane("YZ").circle(0.011).extrude(0.010)
    front_step = cq.Workplane("YZ").circle(0.009).extrude(0.004).translate((0.010, 0.0, 0.0))
    core_bore = cq.Workplane("YZ").circle(0.0046).extrude(0.016).translate((-0.001, 0.0, 0.0))
    return outer_ring.union(front_step).cut(core_bore)


def _add_rotated_x_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clinical_monocular_microscope")

    enamel = model.material("enamel", rgba=(0.87, 0.88, 0.84, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.18, 0.19, 1.0))
    stage_black = model.material("stage_black", rgba=(0.15, 0.15, 0.16, 1.0))
    objective_black = model.material("objective_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.130, 0.185, 0.016)),
        origin=Origin(xyz=(0.0, 0.020, 0.008)),
        material=enamel,
        name="base_foot",
    )
    base.visual(
        Box((0.064, 0.072, 0.040)),
        origin=Origin(xyz=(0.0, -0.030, 0.028)),
        material=enamel,
        name="base_heel",
    )
    base.visual(
        Box((0.032, 0.042, 0.048)),
        origin=Origin(xyz=(0.0, -0.032, 0.050)),
        material=enamel,
        name="base_spine",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.220),
        origin=Origin(xyz=(0.0, COLUMN_Y, 0.135)),
        material=enamel,
        name="column",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.055, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, 0.023, 0.0)),
        material=enamel,
        name="sleeve_front",
    )
    carriage.visual(
        Box((0.011, 0.046, 0.060)),
        origin=Origin(xyz=(-0.0225, 0.0, 0.0)),
        material=enamel,
        name="sleeve_side_0",
    )
    carriage.visual(
        Box((0.011, 0.046, 0.060)),
        origin=Origin(xyz=(0.0225, 0.0, 0.0)),
        material=enamel,
        name="sleeve_side_1",
    )
    carriage.visual(
        Box((0.040, 0.094, 0.010)),
        origin=Origin(xyz=(0.0, 0.070, -0.005)),
        material=enamel,
        name="carriage_arm",
    )
    carriage.visual(
        Box((0.016, 0.024, 0.084)),
        origin=Origin(xyz=(0.0, 0.048, 0.042)),
        material=enamel,
        name="carriage_riser",
    )
    carriage.visual(
        Box((0.030, 0.032, 0.032)),
        origin=Origin(xyz=(0.0, 0.076, 0.096)),
        material=enamel,
        name="head_block",
    )
    carriage.visual(
        Box((0.018, 0.070, 0.010)),
        origin=Origin(xyz=(0.0, STAGE_Y0 + 0.006, 0.000)),
        material=enamel,
        name="guide_support",
    )
    carriage.visual(
        Box((0.040, 0.024, 0.020)),
        origin=Origin(xyz=(0.020, STAGE_Y0, -0.006)),
        material=enamel,
        name="stage_bracket",
    )
    carriage.visual(
        Box((0.018, 0.070, 0.008)),
        origin=Origin(xyz=(0.0, STAGE_Y0 + 0.006, 0.009)),
        material=satin_steel,
        name="stage_guide",
    )

    nosepiece_origin = (0.0, 0.094, 0.074)
    main_tube_center = _optical_offset(0.046)
    eyepiece_center = _optical_offset(0.1195)
    eyecup_center = _optical_offset(0.155)

    carriage.visual(
        Cylinder(radius=0.0185, length=0.092),
        origin=Origin(
            xyz=(
                nosepiece_origin[0] + main_tube_center[0],
                nosepiece_origin[1] + main_tube_center[1],
                nosepiece_origin[2] + main_tube_center[2],
            ),
            rpy=(OPTICAL_TILT, 0.0, 0.0),
        ),
        material=satin_steel,
        name="body_tube",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.055),
        origin=Origin(
            xyz=(
                nosepiece_origin[0] + eyepiece_center[0],
                nosepiece_origin[1] + eyepiece_center[1],
                nosepiece_origin[2] + eyepiece_center[2],
            ),
            rpy=(OPTICAL_TILT, 0.0, 0.0),
        ),
        material=dark_trim,
        name="eyepiece_tube",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(
            xyz=(
                nosepiece_origin[0] + eyecup_center[0],
                nosepiece_origin[1] + eyecup_center[1],
                nosepiece_origin[2] + eyecup_center[2],
            ),
            rpy=(OPTICAL_TILT, 0.0, 0.0),
        ),
        material=dark_trim,
        name="eyecup",
    )

    stage = model.part("stage")
    stage.visual(
        Box((0.072, 0.082, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=stage_black,
        name="stage_plate",
    )
    stage.visual(
        Box((0.005, 0.062, 0.018)),
        origin=Origin(xyz=(-0.0125, 0.0, 0.009)),
        material=stage_black,
        name="stage_rail_0",
    )
    stage.visual(
        Box((0.005, 0.062, 0.018)),
        origin=Origin(xyz=(0.0125, 0.0, 0.009)),
        material=stage_black,
        name="stage_rail_1",
    )
    stage.visual(
        Box((0.056, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, 0.026, 0.021)),
        material=dark_trim,
        name="slide_clip",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=satin_steel,
        name="turret_hub",
    )
    for index, (x_pos, y_pos, radius, length) in enumerate(
        (
            (0.010, 0.000, 0.0060, 0.028),
            (-0.005, 0.008, 0.0050, 0.024),
            (-0.005, -0.008, 0.0048, 0.020),
        )
    ):
        turret.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x_pos, y_pos, -0.017)),
            material=objective_black,
            name=f"objective_{index}",
        )

    stage_control_0 = model.part("stage_control_0")
    stage_control_0.visual(
        mesh_from_cadquery(_build_stage_ring_knob(), "microscope_stage_control_ring"),
        material=dark_trim,
        name="control_knob",
    )

    stage_control_1 = model.part("stage_control_1")
    _add_rotated_x_cylinder(
        stage_control_1,
        radius=0.0030,
        length=0.021,
        xyz=(0.0105, 0.0, 0.0),
        material=dark_trim,
        name="control_shaft",
    )
    _add_rotated_x_cylinder(
        stage_control_1,
        radius=0.0065,
        length=0.010,
        xyz=(0.024, 0.0, 0.0),
        material=dark_trim,
        name="control_knob",
    )
    _add_rotated_x_cylinder(
        stage_control_1,
        radius=0.0045,
        length=0.004,
        xyz=(0.031, 0.0, 0.0),
        material=dark_trim,
        name="control_tip",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, COLUMN_Y, FOCUS_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=65.0,
            velocity=0.12,
            lower=0.0,
            upper=FOCUS_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=stage,
        origin=Origin(xyz=(0.0, STAGE_Y0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=-STAGE_TRAVEL,
            upper=STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_turret",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=turret,
        origin=Origin(xyz=nosepiece_origin, rpy=(OPTICAL_TILT, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=10.0),
    )

    control_origin = Origin(xyz=(0.040, STAGE_Y0, 0.0))
    model.articulation(
        "carriage_to_stage_control_0",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=stage_control_0,
        origin=control_origin,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
    )
    model.articulation(
        "carriage_to_stage_control_1",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=stage_control_1,
        origin=control_origin,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("carriage")
    stage = object_model.get_part("stage")
    control_0 = object_model.get_part("stage_control_0")
    control_1 = object_model.get_part("stage_control_1")

    focus = object_model.get_articulation("base_to_carriage")
    focus_limits = focus.motion_limits
    if focus_limits is not None and focus_limits.upper is not None:
        rest_carriage = ctx.part_world_position(carriage)
        with ctx.pose({focus: focus_limits.upper}):
            raised_carriage = ctx.part_world_position(carriage)
        ctx.check(
            "carriage rises on the column",
            rest_carriage is not None
            and raised_carriage is not None
            and raised_carriage[2] > rest_carriage[2] + 0.05,
            details=f"rest={rest_carriage}, raised={raised_carriage}",
        )

    stage_slide = object_model.get_articulation("carriage_to_stage")
    stage_limits = stage_slide.motion_limits
    if stage_limits is not None and stage_limits.lower is not None and stage_limits.upper is not None:
        with ctx.pose({stage_slide: stage_limits.lower}):
            rear_stage = ctx.part_world_position(stage)
        with ctx.pose({stage_slide: stage_limits.upper}):
            front_stage = ctx.part_world_position(stage)
        ctx.check(
            "stage slides front to back",
            rear_stage is not None
            and front_stage is not None
            and front_stage[1] > rear_stage[1] + 0.03,
            details=f"rear={rear_stage}, front={front_stage}",
        )

        for label, pose_value in (("rear", stage_limits.lower), ("front", stage_limits.upper)):
            with ctx.pose({stage_slide: pose_value}):
                ctx.expect_overlap(
                    stage,
                    carriage,
                    axes="y",
                    elem_a="stage_rail_0",
                    elem_b="stage_guide",
                    min_overlap=0.040,
                    name=f"left stage rail retains insertion on the guide at {label} travel",
                )
                ctx.expect_overlap(
                    stage,
                    carriage,
                    axes="y",
                    elem_a="stage_rail_1",
                    elem_b="stage_guide",
                    min_overlap=0.040,
                    name=f"right stage rail retains insertion on the guide at {label} travel",
                )
                ctx.expect_gap(
                    carriage,
                    stage,
                    axis="x",
                    positive_elem="stage_guide",
                    negative_elem="stage_rail_0",
                    min_gap=0.0005,
                    max_gap=0.0025,
                    name=f"guide stays clear of the left rail at {label} travel",
                )
                ctx.expect_gap(
                    stage,
                    carriage,
                    axis="x",
                    positive_elem="stage_rail_1",
                    negative_elem="stage_guide",
                    min_gap=0.0005,
                    max_gap=0.0025,
                    name=f"guide stays clear of the right rail at {label} travel",
                )
                ctx.expect_gap(
                    stage,
                    carriage,
                    axis="z",
                    positive_elem="stage_plate",
                    negative_elem="stage_guide",
                    min_gap=0.001,
                    max_gap=0.006,
                    name=f"stage plate stays just above the guide at {label} travel",
                )

    control_0_pos = ctx.part_world_position(control_0)
    control_1_pos = ctx.part_world_position(control_1)
    ctx.check(
        "stage controls are coaxial on the bracket",
        control_0_pos is not None
        and control_1_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(control_0_pos, control_1_pos)),
        details=f"control_0={control_0_pos}, control_1={control_1_pos}",
    )

    turret_joint = object_model.get_articulation("carriage_to_turret")
    second_control_joint = object_model.get_articulation("carriage_to_stage_control_1")
    ctx.check(
        "turret rotates continuously",
        turret_joint.articulation_type == ArticulationType.CONTINUOUS
        and turret_joint.motion_limits is not None
        and turret_joint.motion_limits.lower is None
        and turret_joint.motion_limits.upper is None,
        details=str(turret_joint.motion_limits),
    )
    ctx.check(
        "second stage control rotates continuously",
        second_control_joint.articulation_type == ArticulationType.CONTINUOUS
        and second_control_joint.motion_limits is not None
        and second_control_joint.motion_limits.lower is None
        and second_control_joint.motion_limits.upper is None,
        details=str(second_control_joint.motion_limits),
    )

    return ctx.report()


object_model = build_object_model()
