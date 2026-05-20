from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_monocular_microscope")

    enamel = model.material("warm_ivory_enamel", rgba=(0.78, 0.74, 0.64, 1.0))
    dark = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    metal = model.material("brushed_steel", rgba=(0.54, 0.55, 0.55, 1.0))
    brass = model.material("aged_brass", rgba=(0.70, 0.51, 0.22, 1.0))
    glass = model.material("pale_glass", rgba=(0.72, 0.90, 1.0, 0.42))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.44, 0.30, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=enamel,
        name="heavy_base",
    )
    stand.visual(
        Box((0.32, 0.18, 0.020)),
        origin=Origin(xyz=(0.0, -0.025, 0.078)),
        material=enamel,
        name="raised_base_pad",
    )
    for i, (x, y) in enumerate(
        ((-0.165, -0.105), (0.165, -0.105), (-0.165, 0.105), (0.165, 0.105))
    ):
        stand.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(x, y, -0.0055)),
            material=rubber,
            name=f"foot_{i}",
        )

    stand.visual(
        Cylinder(radius=0.023, length=0.58),
        origin=Origin(xyz=(0.0, 0.105, 0.365)),
        material=metal,
        name="vertical_column",
    )
    stand.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, 0.105, 0.078)),
        material=enamel,
        name="column_plinth",
    )
    stand.visual(
        Box((0.012, 0.006, 0.36)),
        origin=Origin(xyz=(0.0, 0.080, 0.385)),
        material=dark,
        name="front_rack",
    )
    stand.visual(
        mesh_from_geometry(
            CapsuleGeometry(radius=0.025, length=0.48),
            "swept_rear_arm",
        ),
        origin=Origin(xyz=(0.0, 0.185, 0.325), rpy=(0.12, 0.0, 0.0)),
        material=enamel,
        name="swept_arm",
    )
    stand.visual(
        Box((0.078, 0.090, 0.050)),
        origin=Origin(xyz=(0.0, 0.145, 0.620)),
        material=enamel,
        name="upper_column_yoke",
    )
    stand.visual(
        Box((0.064, 0.060, 0.024)),
        origin=Origin(xyz=(0.0, 0.165, 0.590)),
        material=enamel,
        name="arm_lug",
    )

    # A rigid U-shaped stage support is part of the stand.  Its two rails leave
    # the condenser path open and make the later moving stage visibly supported.
    stage_y = -0.045
    stand.visual(
        Box((0.290, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, stage_y + 0.050, 0.218)),
        material=metal,
        name="stage_rail_0",
    )
    stand.visual(
        Box((0.290, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, stage_y - 0.050, 0.218)),
        material=metal,
        name="stage_rail_1",
    )
    for i, x in enumerate((-0.138, 0.138)):
        stand.visual(
            Box((0.014, 0.114, 0.014)),
            origin=Origin(xyz=(x, stage_y, 0.218)),
            material=metal,
            name=f"stage_end_bar_{i}",
        )
    stand.visual(
        Box((0.090, 0.108, 0.026)),
        origin=Origin(xyz=(0.0, 0.055, 0.205)),
        material=enamel,
        name="stage_support_neck",
    )

    stage_carriage = model.part("stage_carriage")
    stage_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.220, 0.130, 0.012, corner_segments=8),
            [rounded_rect_profile(0.056, 0.036, 0.006, corner_segments=6)],
            0.018,
            center=True,
        ),
        "perforated_stage_plate",
    )
    stage_carriage.visual(
        stage_plate_mesh,
        origin=Origin(),
        material=dark,
        name="stage_plate",
    )
    stage_carriage.visual(
        Box((0.180, 0.012, 0.011)),
        origin=Origin(xyz=(0.0, 0.050, -0.0145)),
        material=metal,
        name="rail_shoe_0",
    )
    stage_carriage.visual(
        Box((0.180, 0.012, 0.011)),
        origin=Origin(xyz=(0.0, -0.050, -0.0145)),
        material=metal,
        name="rail_shoe_1",
    )
    for i, x in enumerate((-0.056, 0.056)):
        stage_carriage.visual(
            Box((0.062, 0.010, 0.004)),
            origin=Origin(xyz=(x, -0.033, 0.011)),
            material=metal,
            name=f"slide_clip_{i}",
        )
    stage_carriage.visual(
        Box((0.095, 0.034, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=glass,
        name="glass_slide",
    )

    optical_body = model.part("optical_body")
    for i, x in enumerate((-0.030, 0.030)):
        optical_body.visual(
            Box((0.012, 0.060, 0.160)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=enamel,
            name=f"column_cheek_{i}",
        )
    optical_body.visual(
        Box((0.084, 0.014, 0.155)),
        origin=Origin(xyz=(0.0, -0.037, 0.0)),
        material=enamel,
        name="front_collar_bridge",
    )
    optical_body.visual(
        Box((0.070, 0.160, 0.040)),
        origin=Origin(xyz=(0.0, -0.105, 0.038)),
        material=enamel,
        name="body_bridge",
    )
    optical_body.visual(
        Cylinder(radius=0.038, length=0.135),
        origin=Origin(xyz=(0.0, -0.177, -0.017)),
        material=enamel,
        name="lower_tube",
    )
    optical_body.visual(
        Box((0.095, 0.080, 0.055)),
        origin=Origin(xyz=(0.0, -0.177, 0.065)),
        material=enamel,
        name="monocular_head",
    )
    optical_body.visual(
        Cylinder(radius=0.022, length=0.130),
        origin=Origin(xyz=(0.0, -0.105, 0.095), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="eyepiece_tube",
    )
    optical_body.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, -0.035, 0.095), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="ocular_lens",
    )

    objective_turret = model.part("objective_turret")
    objective_turret.visual(
        Cylinder(radius=0.052, length=0.016),
        origin=Origin(),
        material=dark,
        name="turret_disk",
    )
    objective_turret.visual(
        Cylinder(radius=0.020, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=metal,
        name="turret_hub",
    )
    objective_turret.visual(
        Cylinder(radius=0.009, length=0.064),
        origin=Origin(xyz=(0.0, 0.028, -0.040)),
        material=metal,
        name="objective_0",
    )
    objective_turret.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.028, -0.076)),
        material=glass,
        name="objective_lens_0",
    )
    objective_turret.visual(
        Cylinder(radius=0.011, length=0.064),
        origin=Origin(xyz=(-0.0242, -0.014, -0.040)),
        material=metal,
        name="objective_1",
    )
    objective_turret.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(-0.0242, -0.014, -0.076)),
        material=glass,
        name="objective_lens_1",
    )
    objective_turret.visual(
        Cylinder(radius=0.013, length=0.064),
        origin=Origin(xyz=(0.0242, -0.014, -0.040)),
        material=metal,
        name="objective_2",
    )
    objective_turret.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0242, -0.014, -0.076)),
        material=glass,
        name="objective_lens_2",
    )

    condenser_housing = model.part("condenser_housing")
    condenser_housing.visual(
        Cylinder(radius=0.030, length=0.042),
        origin=Origin(),
        material=metal,
        name="condenser_body",
    )
    condenser_housing.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=metal,
        name="top_flange",
    )
    condenser_housing.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=dark,
        name="iris_ring",
    )
    condenser_housing.visual(
        Box((0.020, 0.012, 0.008)),
        origin=Origin(xyz=(0.032, 0.0, -0.010)),
        material=metal,
        name="pivot_web",
    )
    condenser_housing.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.0375, 0.0, -0.010)),
        material=metal,
        name="pivot_boss",
    )

    diaphragm_lever = model.part("diaphragm_lever")
    diaphragm_lever.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(),
        material=brass,
        name="pivot_disk",
    )
    diaphragm_lever.visual(
        Box((0.076, 0.008, 0.004)),
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        material=brass,
        name="lever_arm",
    )
    diaphragm_lever.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.083, 0.0, 0.0)),
        material=brass,
        name="lever_tip",
    )

    model.articulation(
        "stand_to_stage",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage_carriage,
        origin=Origin(xyz=(0.0, stage_y, 0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.12, lower=-0.050, upper=0.050),
    )
    model.articulation(
        "column_to_body",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=optical_body,
        origin=Origin(xyz=(0.0, 0.105, 0.460)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.08, lower=-0.030, upper=0.035),
    )
    model.articulation(
        "body_to_turret",
        ArticulationType.CONTINUOUS,
        parent=optical_body,
        child=objective_turret,
        origin=Origin(xyz=(0.0, -0.177, -0.093)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    model.articulation(
        "stage_to_condenser",
        ArticulationType.FIXED,
        parent=stage_carriage,
        child=condenser_housing,
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
    )
    model.articulation(
        "condenser_to_lever",
        ArticulationType.REVOLUTE,
        parent=condenser_housing,
        child=diaphragm_lever,
        origin=Origin(xyz=(0.0375, 0.0, -0.019)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.80, upper=0.80),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    stage = object_model.get_part("stage_carriage")
    body = object_model.get_part("optical_body")
    turret = object_model.get_part("objective_turret")
    condenser = object_model.get_part("condenser_housing")
    lever = object_model.get_part("diaphragm_lever")

    stage_slide = object_model.get_articulation("stand_to_stage")
    body_slide = object_model.get_articulation("column_to_body")
    turret_spin = object_model.get_articulation("body_to_turret")
    lever_joint = object_model.get_articulation("condenser_to_lever")

    ctx.expect_contact(
        stage,
        stand,
        elem_a="rail_shoe_0",
        elem_b="stage_rail_0",
        contact_tol=0.001,
        name="stage carriage rests on rear rail",
    )
    ctx.expect_contact(
        stage,
        stand,
        elem_a="rail_shoe_1",
        elem_b="stage_rail_1",
        contact_tol=0.001,
        name="stage carriage rests on front rail",
    )
    ctx.expect_contact(
        condenser,
        stage,
        elem_a="top_flange",
        elem_b="stage_plate",
        contact_tol=0.002,
        name="condenser is mounted to underside of stage",
    )
    ctx.expect_contact(
        lever,
        condenser,
        elem_a="pivot_disk",
        elem_b="pivot_boss",
        contact_tol=0.001,
        name="diaphragm lever pivot is supported by condenser",
    )
    ctx.expect_contact(
        turret,
        body,
        elem_a="turret_disk",
        elem_b="lower_tube",
        contact_tol=0.001,
        name="objective turret seats under optical body",
    )
    ctx.expect_gap(
        turret,
        stage,
        axis="z",
        positive_elem="objective_0",
        negative_elem="stage_plate",
        min_gap=0.020,
        name="objectives clear the stage at nominal focus",
    )
    ctx.expect_origin_distance(
        body,
        stage,
        axes="xy",
        min_dist=0.04,
        name="optical body and stage are separate assemblies",
    )

    body_rest = ctx.part_world_position(body)
    with ctx.pose({body_slide: 0.030}):
        body_raised = ctx.part_world_position(body)
    ctx.check(
        "optical body slides upward on column",
        body_rest is not None
        and body_raised is not None
        and body_raised[2] > body_rest[2] + 0.025,
        details=f"rest={body_rest}, raised={body_raised}",
    )

    stage_rest = ctx.part_world_position(stage)
    with ctx.pose({stage_slide: 0.045}):
        stage_shifted = ctx.part_world_position(stage)
        ctx.expect_contact(
            stage,
            stand,
            elem_a="rail_shoe_0",
            elem_b="stage_rail_0",
            contact_tol=0.001,
            name="shifted stage remains supported on rear rail",
        )
    ctx.check(
        "stage carriage slides left to right",
        stage_rest is not None
        and stage_shifted is not None
        and stage_shifted[0] > stage_rest[0] + 0.040,
        details=f"rest={stage_rest}, shifted={stage_shifted}",
    )

    objective_rest_aabb = ctx.part_element_world_aabb(turret, elem="objective_0")
    with ctx.pose({turret_spin: math.pi / 2.0}):
        objective_spun_aabb = ctx.part_element_world_aabb(turret, elem="objective_0")
    if objective_rest_aabb is not None and objective_spun_aabb is not None:
        rest_center = tuple((objective_rest_aabb[0][i] + objective_rest_aabb[1][i]) / 2.0 for i in range(3))
        spun_center = tuple((objective_spun_aabb[0][i] + objective_spun_aabb[1][i]) / 2.0 for i in range(3))
        moved = math.hypot(spun_center[0] - rest_center[0], spun_center[1] - rest_center[1])
    else:
        rest_center = None
        spun_center = None
        moved = 0.0
    ctx.check(
        "objective turret rotates about optical axis",
        moved > 0.030,
        details=f"rest={rest_center}, spun={spun_center}",
    )

    lever_rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_tip")
    with ctx.pose({lever_joint: 0.70}):
        lever_swept_aabb = ctx.part_element_world_aabb(lever, elem="lever_tip")
    if lever_rest_aabb is not None and lever_swept_aabb is not None:
        lever_rest_center = tuple((lever_rest_aabb[0][i] + lever_rest_aabb[1][i]) / 2.0 for i in range(3))
        lever_swept_center = tuple((lever_swept_aabb[0][i] + lever_swept_aabb[1][i]) / 2.0 for i in range(3))
        lever_motion = math.hypot(
            lever_swept_center[0] - lever_rest_center[0],
            lever_swept_center[1] - lever_rest_center[1],
        )
    else:
        lever_rest_center = None
        lever_swept_center = None
        lever_motion = 0.0
    ctx.check(
        "diaphragm lever swings under stage",
        lever_motion > 0.045,
        details=f"rest={lever_rest_center}, swept={lever_swept_center}",
    )

    return ctx.report()


object_model = build_object_model()
