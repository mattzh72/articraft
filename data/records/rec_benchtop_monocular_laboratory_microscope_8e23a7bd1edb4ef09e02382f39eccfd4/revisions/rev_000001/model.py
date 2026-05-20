from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_monocular_microscope")

    enamel = model.material("enamel", rgba=(0.91, 0.92, 0.90, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    glass = model.material("glass", rgba=(0.48, 0.65, 0.78, 0.45))

    stand = model.part("stand")
    base_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.255, 0.182, 0.030, corner_segments=10), 0.028),
        "microscope_base",
    )
    stand.visual(
        base_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=enamel,
        name="base_shell",
    )
    stand.visual(
        Box((0.112, 0.096, 0.050)),
        origin=Origin(xyz=(0.0, -0.034, 0.053)),
        material=enamel,
        name="rear_heel",
    )
    arm_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.0, -0.072, 0.028),
                (0.0, -0.070, 0.110),
                (0.0, -0.058, 0.212),
                (0.0, -0.040, 0.314),
                (0.0, -0.026, 0.390),
            ],
            profile=rounded_rect_profile(0.050, 0.032, 0.010, corner_segments=8),
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(1.0, 0.0, 0.0),
        ),
        "microscope_arm",
    )
    stand.visual(arm_mesh, material=enamel, name="arm")
    stand.visual(
        Box((0.050, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, -0.032, 0.398)),
        material=enamel,
        name="head_cap",
    )
    stand.visual(
        Box((0.024, 0.046, 0.040)),
        origin=Origin(xyz=(0.0, -0.025, 0.190)),
        material=enamel,
        name="column_mount",
    )

    column = model.part("column")
    column.visual(
        Box((0.034, 0.024, 0.180)),
        material=graphite,
        name="column_rail",
    )

    model.articulation(
        "stand_to_column",
        ArticulationType.FIXED,
        parent=stand,
        child=column,
        origin=Origin(xyz=(0.0, 0.010, 0.250)),
    )

    stage_support = model.part("stage_support")
    stage_support.visual(
        Box((0.140, 0.090, 0.010)),
        origin=Origin(xyz=(0.0, 0.022, -0.010)),
        material=graphite,
        name="slide_base",
    )
    stage_support.visual(
        Box((0.100, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -0.072, -0.010)),
        material=graphite,
        name="rear_block",
    )
    for index, brace_x in enumerate((-0.042, 0.042)):
        stage_support.visual(
            Box((0.014, 0.100, 0.018)),
            origin=Origin(xyz=(brace_x, -0.032, -0.010)),
            material=enamel,
            name=f"brace_{index}",
        )
    for index, rail_y in enumerate((0.000, 0.044)):
        stage_support.visual(
            Box((0.150, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, rail_y, -0.006)),
            material=steel,
            name=f"rail_{index}",
        )

    model.articulation(
        "stand_to_stage_support",
        ArticulationType.FIXED,
        parent=stand,
        child=stage_support,
        origin=Origin(xyz=(0.0, 0.056, 0.168)),
    )

    stage = model.part("stage")
    stage_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.160, 0.122, 0.008, corner_segments=8),
            [rounded_rect_profile(0.024, 0.018, 0.003, corner_segments=6)],
            0.004,
        ),
        "stage_plate",
    )
    stage.visual(
        stage_plate_mesh,
        origin=Origin(xyz=(0.0, 0.022, 0.005)),
        material=black,
        name="stage_plate",
    )
    stage.visual(
        Box((0.086, 0.064, 0.008)),
        origin=Origin(xyz=(0.0, 0.022, -0.001)),
        material=graphite,
        name="carriage_block",
    )
    for index, runner_y in enumerate((0.000, 0.044)):
        stage.visual(
            Box((0.094, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, runner_y, 0.000)),
            material=steel,
            name=f"runner_{index}",
        )
    stage.visual(
        Box((0.038, 0.008, 0.004)),
        origin=Origin(xyz=(0.044, 0.042, 0.009)),
        material=steel,
        name="slide_clip",
    )

    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=stage_support,
        child=stage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.08, lower=-0.018, upper=0.018),
    )

    condenser = model.part("condenser")
    condenser.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=graphite,
        name="mount_collar",
    )
    condenser.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=steel,
        name="iris_ring",
    )
    condenser.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=graphite,
        name="housing",
    )
    condenser.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.026, 0.0, -0.013)),
        material=steel,
        name="pivot_boss",
    )

    model.articulation(
        "stage_support_to_condenser",
        ArticulationType.FIXED,
        parent=stage_support,
        child=condenser,
        origin=Origin(xyz=(0.0, 0.022, -0.015)),
    )

    diaphragm_lever = model.part("diaphragm_lever")
    diaphragm_lever.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=black,
        name="pivot_collar",
    )
    diaphragm_lever.visual(
        Box((0.032, 0.004, 0.003)),
        origin=Origin(xyz=(0.016, 0.0, 0.004)),
        material=black,
        name="lever_arm",
    )
    diaphragm_lever.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(0.034, 0.0, 0.004)),
        material=black,
        name="lever_knob",
    )

    model.articulation(
        "diaphragm_control",
        ArticulationType.REVOLUTE,
        parent=condenser,
        child=diaphragm_lever,
        origin=Origin(xyz=(0.026, 0.0, -0.013)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.5,
            lower=-0.65,
            upper=0.65,
        ),
    )

    optical_body = model.part("optical_body")
    sleeve_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.088, 0.036, 0.010, corner_segments=8),
            [rounded_rect_profile(0.036, 0.026, 0.004, corner_segments=6)],
            0.090,
        ),
        "optical_body_sleeve",
    )
    optical_body.visual(sleeve_mesh, material=enamel, name="carriage_sleeve")
    optical_body.visual(
        Box((0.050, 0.050, 0.022)),
        origin=Origin(xyz=(0.0, 0.039, 0.004)),
        material=enamel,
        name="bridge_block",
    )
    optical_body.visual(
        Box((0.042, 0.046, 0.024)),
        origin=Origin(xyz=(0.0, 0.042, 0.027)),
        material=enamel,
        name="head_body",
    )
    optical_body.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.060, -0.002)),
        material=graphite,
        name="nose_mount",
    )
    for index, knob_x in enumerate((-0.050, 0.050)):
        optical_body.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(
                xyz=(knob_x * 0.96, -0.004, 0.000),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=graphite,
            name=f"focus_knob_{index}",
        )
    tube_rpy = (math.radians(32.0), 0.0, 0.0)
    optical_body.visual(
        Cylinder(radius=0.015, length=0.104),
        origin=Origin(xyz=(0.0, 0.070, 0.078), rpy=tube_rpy),
        material=graphite,
        name="eyepiece_tube",
    )
    optical_body.visual(
        Box((0.024, 0.050, 0.024)),
        origin=Origin(xyz=(0.0, 0.076, 0.032)),
        material=graphite,
        name="ocular_base",
    )
    optical_body.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(0.0, 0.055, 0.120), rpy=tube_rpy),
        material=black,
        name="eyepiece",
    )
    optical_body.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, 0.053, 0.130), rpy=tube_rpy),
        material=glass,
        name="eyelens",
    )

    model.articulation(
        "body_lift",
        ArticulationType.PRISMATIC,
        parent=column,
        child=optical_body,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.08, lower=0.0, upper=0.045),
    )

    objective_turret = model.part("objective_turret")
    objective_turret.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=graphite,
        name="turret_disc",
    )
    objective_turret.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=steel,
        name="turret_cap",
    )
    objective_specs = (
        ("objective_0", 0.0, 0.026),
        ("objective_1", 2.0 * math.pi / 3.0, 0.022),
        ("objective_2", 4.0 * math.pi / 3.0, 0.018),
    )
    objective_radius = 0.013
    for base_name, angle, body_length in objective_specs:
        x_pos = objective_radius * math.cos(angle)
        y_pos = objective_radius * math.sin(angle)
        objective_turret.visual(
            Cylinder(radius=0.006, length=body_length),
            origin=Origin(xyz=(x_pos, y_pos, -0.010 - body_length * 0.5)),
            material=steel,
            name=f"{base_name}_body",
        )
        objective_turret.visual(
            Cylinder(radius=0.004, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, -0.015 - body_length)),
            material=steel,
            name=f"{base_name}_tip",
        )

    model.articulation(
        "turret_spin",
        ArticulationType.CONTINUOUS,
        parent=optical_body,
        child=objective_turret,
        origin=Origin(xyz=(0.0, 0.060, -0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    column = object_model.get_part("column")
    stage_support = object_model.get_part("stage_support")
    stage = object_model.get_part("stage")
    condenser = object_model.get_part("condenser")
    diaphragm_lever = object_model.get_part("diaphragm_lever")
    optical_body = object_model.get_part("optical_body")
    objective_turret = object_model.get_part("objective_turret")

    stage_slide = object_model.get_articulation("stage_slide")
    body_lift = object_model.get_articulation("body_lift")
    turret_spin = object_model.get_articulation("turret_spin")
    diaphragm_control = object_model.get_articulation("diaphragm_control")

    ctx.allow_overlap(
        optical_body,
        column,
        elem_a="carriage_sleeve",
        elem_b="column_rail",
        reason="The focusing carriage is intentionally represented as a sleeve sliding on the fixed column rail.",
    )

    ctx.expect_overlap(
        optical_body,
        column,
        axes="xy",
        elem_a="carriage_sleeve",
        elem_b="column_rail",
        min_overlap=0.020,
        name="carriage stays centered on the column",
    )
    ctx.expect_overlap(
        optical_body,
        column,
        axes="z",
        elem_a="carriage_sleeve",
        elem_b="column_rail",
        min_overlap=0.080,
        name="carriage remains engaged on the column at rest",
    )
    ctx.expect_within(
        stage,
        stage_support,
        axes="y",
        inner_elem="carriage_block",
        outer_elem="slide_base",
        margin=0.002,
        name="stage carriage stays within the stage saddle width",
    )
    ctx.expect_overlap(
        stage,
        stage_support,
        axes="x",
        elem_a="carriage_block",
        elem_b="slide_base",
        min_overlap=0.060,
        name="stage carriage remains retained on the slide base",
    )
    ctx.expect_gap(
        objective_turret,
        stage,
        axis="z",
        min_gap=0.010,
        max_gap=0.030,
        name="objective turret clears the stage surface at rest",
    )
    ctx.expect_contact(
        diaphragm_lever,
        condenser,
        elem_a="pivot_collar",
        elem_b="pivot_boss",
        name="diaphragm lever is pivot-supported by the condenser housing",
    )

    body_rest = ctx.part_world_position(optical_body)
    stage_rest = ctx.part_world_position(stage)
    with ctx.pose({body_lift: 0.045}):
        ctx.expect_overlap(
            optical_body,
            column,
            axes="z",
            elem_a="carriage_sleeve",
            elem_b="column_rail",
            min_overlap=0.080,
            name="carriage remains engaged on the column when raised",
        )
        body_raised = ctx.part_world_position(optical_body)
    ctx.check(
        "optical body travels upward on the column",
        body_rest is not None
        and body_raised is not None
        and body_raised[2] > body_rest[2] + 0.030,
        details=f"rest={body_rest}, raised={body_raised}",
    )

    with ctx.pose({stage_slide: 0.018}):
        ctx.expect_overlap(
            stage,
            stage_support,
            axes="x",
            elem_a="carriage_block",
            elem_b="slide_base",
            min_overlap=0.060,
            name="stage carriage remains retained at full right travel",
        )
        stage_right = ctx.part_world_position(stage)
    ctx.check(
        "stage slides left to right",
        stage_rest is not None
        and stage_right is not None
        and stage_right[0] > stage_rest[0] + 0.015,
        details=f"rest={stage_rest}, right={stage_right}",
    )

    turret_tip_rest = _aabb_center(
        ctx.part_element_world_aabb(objective_turret, elem="objective_0_tip")
    )
    with ctx.pose({turret_spin: 1.2}):
        turret_tip_rotated = _aabb_center(
            ctx.part_element_world_aabb(objective_turret, elem="objective_0_tip")
        )
    ctx.check(
        "objective turret rotates around the optical axis",
        turret_tip_rest is not None
        and turret_tip_rotated is not None
        and math.hypot(
            turret_tip_rotated[0] - turret_tip_rest[0],
            turret_tip_rotated[1] - turret_tip_rest[1],
        )
        > 0.010
        and abs(turret_tip_rotated[2] - turret_tip_rest[2]) < 0.003,
        details=f"rest={turret_tip_rest}, rotated={turret_tip_rotated}",
    )

    lever_knob_rest = _aabb_center(
        ctx.part_element_world_aabb(diaphragm_lever, elem="lever_knob")
    )
    with ctx.pose({diaphragm_control: 0.55}):
        lever_knob_open = _aabb_center(
            ctx.part_element_world_aabb(diaphragm_lever, elem="lever_knob")
        )
    ctx.check(
        "diaphragm lever swings beneath the stage",
        lever_knob_rest is not None
        and lever_knob_open is not None
        and math.hypot(
            lever_knob_open[0] - lever_knob_rest[0],
            lever_knob_open[1] - lever_knob_rest[1],
        )
        > 0.010,
        details=f"rest={lever_knob_rest}, swung={lever_knob_open}",
    )

    return ctx.report()


object_model = build_object_model()
