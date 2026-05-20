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
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_microscope")

    enamel = model.material("enamel", rgba=(0.88, 0.89, 0.86, 1.0))
    warm_grey = model.material("warm_grey", rgba=(0.69, 0.70, 0.69, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.24, 0.25, 0.27, 1.0))
    black = model.material("black", rgba=(0.07, 0.08, 0.09, 1.0))
    brass = model.material("brass", rgba=(0.62, 0.52, 0.21, 1.0))
    glass = model.material("glass", rgba=(0.45, 0.62, 0.74, 0.45))

    stand = model.part("stand")

    arm_mesh = _mesh(
        "microscope_arm",
        sweep_profile_along_spline(
            [
                (0.030, 0.058, 0.084),
                (0.031, 0.066, 0.142),
                (0.032, 0.076, 0.205),
                (0.032, 0.086, 0.244),
                (0.033, 0.093, 0.255),
            ],
            profile=rounded_rect_profile(0.024, 0.020, 0.005, corner_segments=8),
            samples_per_segment=18,
            cap_profile=True,
        ),
    )

    stand.visual(
        Box((0.170, 0.200, 0.024)),
        origin=Origin(xyz=(0.0, -0.015, 0.012)),
        material=enamel,
        name="base_foot",
    )
    stand.visual(
        Box((0.125, 0.110, 0.018)),
        origin=Origin(xyz=(0.0, 0.022, 0.033)),
        material=enamel,
        name="base_step",
    )
    stand.visual(
        Box((0.075, 0.055, 0.060)),
        origin=Origin(xyz=(0.0, 0.050, 0.054)),
        material=enamel,
        name="rear_pedestal",
    )
    stand.visual(
        Box((0.034, 0.028, 0.210)),
        origin=Origin(xyz=(0.0, 0.048, 0.189)),
        material=warm_grey,
        name="focus_rail",
    )
    stand.visual(
        Box((0.030, 0.085, 0.014)),
        origin=Origin(xyz=(0.0, 0.015, 0.074)),
        material=enamel,
        name="stage_bridge",
    )
    stand.visual(
        Box((0.150, 0.024, 0.008)),
        origin=Origin(xyz=(0.0, -0.010, 0.081)),
        material=warm_grey,
        name="stage_rail",
    )
    stand.visual(
        Box((0.026, 0.020, 0.074)),
        origin=Origin(xyz=(0.0, 0.050, 0.118)),
        material=enamel,
        name="stage_post",
    )
    stand.visual(arm_mesh, material=enamel, name="arm")
    stand.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.035, 0.066, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_grey,
        name="coarse_boss",
    )
    stand.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.038, 0.055, 0.154), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_grey,
        name="fine_boss",
    )

    optical_body = model.part("optical_body")
    optical_body.visual(
        Box((0.060, 0.028, 0.058)),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material=enamel,
        name="carriage_plate",
    )
    optical_body.visual(
        Box((0.012, 0.020, 0.064)),
        origin=Origin(xyz=(0.024, -0.010, 0.0)),
        material=enamel,
        name="carriage_cheek_0",
    )
    optical_body.visual(
        Box((0.012, 0.020, 0.064)),
        origin=Origin(xyz=(-0.024, -0.010, 0.0)),
        material=enamel,
        name="carriage_cheek_1",
    )
    optical_body.visual(
        Box((0.042, 0.060, 0.024)),
        origin=Origin(xyz=(0.0, -0.033, -0.002)),
        material=enamel,
        name="nose_block",
    )
    optical_body.visual(
        Box((0.034, 0.034, 0.032)),
        origin=Origin(xyz=(0.0, -0.026, 0.028)),
        material=enamel,
        name="prism_housing",
    )
    optical_body.visual(
        Cylinder(radius=0.016, length=0.118),
        origin=Origin(xyz=(0.0, -0.044, 0.074), rpy=(-0.45, 0.0, 0.0)),
        material=enamel,
        name="body_tube",
    )
    optical_body.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.0, -0.022, 0.123), rpy=(-0.45, 0.0, 0.0)),
        material=enamel,
        name="head_collar",
    )
    optical_body.visual(
        Cylinder(radius=0.011, length=0.075),
        origin=Origin(xyz=(0.0, -0.006, 0.161), rpy=(-0.45, 0.0, 0.0)),
        material=charcoal,
        name="eyepiece_tube",
    )
    optical_body.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(0.0, 0.008, 0.191), rpy=(-0.45, 0.0, 0.0)),
        material=black,
        name="ocular_cup",
    )
    optical_body.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.0, -0.006, 0.169), rpy=(-0.45, 0.0, 0.0)),
        material=glass,
        name="ocular_lens",
    )
    optical_body.visual(
        Box((0.030, 0.026, 0.044)),
        origin=Origin(xyz=(0.0, -0.046, 0.012)),
        material=enamel,
        name="nose_support",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=dark_grey,
        name="turret_disk",
    )
    turret.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=dark_grey,
        name="turret_hub",
    )
    objective_specs = (
        ("objective_0", 0.0105, 0.0060, 0.032, black),
        ("objective_1", 0.0105, 0.0053, 0.027, charcoal),
        ("objective_2", 0.0105, 0.0050, 0.023, charcoal),
    )
    for index, (name, radius_offset, barrel_radius, barrel_length, barrel_material) in enumerate(objective_specs):
        angle = (2.0 * math.pi * index) / 3.0
        cx = radius_offset * math.cos(angle)
        cy = radius_offset * math.sin(angle)
        collar_length = 0.008
        collar_center_z = -0.014 - collar_length * 0.5
        barrel_center_z = -0.014 - barrel_length * 0.5
        turret.visual(
            Cylinder(radius=barrel_radius + 0.0014, length=collar_length),
            origin=Origin(xyz=(cx, cy, collar_center_z)),
            material=brass,
            name=f"{name}_collar",
        )
        turret.visual(
            Cylinder(radius=barrel_radius, length=barrel_length),
            origin=Origin(xyz=(cx, cy, barrel_center_z)),
            material=barrel_material,
            name=name,
        )

    stage = model.part("stage")
    stage.visual(
        Box((0.095, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_grey,
        name="slide_block",
    )
    stage.visual(
        Box((0.030, 0.042, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_grey,
        name="stage_riser",
    )
    stage.visual(
        Box((0.115, 0.085, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=charcoal,
        name="stage_plate",
    )
    stage.visual(
        Box((0.006, 0.018, 0.006)),
        origin=Origin(xyz=(-0.040, 0.022, 0.032)),
        material=warm_grey,
        name="clip_0",
    )
    stage.visual(
        Box((0.006, 0.018, 0.006)),
        origin=Origin(xyz=(0.040, 0.022, 0.032)),
        material=warm_grey,
        name="clip_1",
    )
    stage.visual(
        Box((0.030, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, -0.036, 0.031)),
        material=warm_grey,
        name="specimen_stop",
    )

    coarse_knob = model.part("coarse_knob")
    coarse_knob.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="coarse_shaft",
    )
    coarse_knob.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="coarse_wheel",
    )
    coarse_knob.visual(
        Cylinder(radius=0.021, length=0.002),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="coarse_rim",
    )

    fine_knob = model.part("fine_knob")
    fine_knob.visual(
        Cylinder(radius=0.003, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="fine_shaft",
    )
    fine_knob.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="fine_wheel",
    )
    fine_knob.visual(
        Cylinder(radius=0.009, length=0.003),
        origin=Origin(xyz=(0.0135, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="fine_cap",
    )

    model.articulation(
        "optical_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=optical_body,
        origin=Origin(xyz=(0.0, 0.034, 0.184)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.050,
            lower=0.0,
            upper=0.060,
        ),
    )
    model.articulation(
        "turret_spin",
        ArticulationType.CONTINUOUS,
        parent=optical_body,
        child=turret,
        origin=Origin(xyz=(0.0, -0.046, -0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=8.0,
        ),
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage,
        origin=Origin(xyz=(0.0, -0.010, 0.091)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.060,
            lower=-0.025,
            upper=0.025,
        ),
    )
    model.articulation(
        "coarse_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=coarse_knob,
        origin=Origin(xyz=(0.044, 0.066, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=8.0,
        ),
    )
    model.articulation(
        "fine_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=fine_knob,
        origin=Origin(xyz=(0.045, 0.055, 0.154)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.1,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    optical_body = object_model.get_part("optical_body")
    turret = object_model.get_part("turret")
    stage = object_model.get_part("stage")
    coarse_knob = object_model.get_part("coarse_knob")
    fine_knob = object_model.get_part("fine_knob")

    optical_slide = object_model.get_articulation("optical_slide")
    stage_slide = object_model.get_articulation("stage_slide")
    turret_spin = object_model.get_articulation("turret_spin")

    optical_limits = optical_slide.motion_limits
    stage_limits = stage_slide.motion_limits

    ctx.expect_contact(
        optical_body,
        stand,
        elem_a="carriage_plate",
        elem_b="focus_rail",
        name="optical carriage bears on the focus rail",
    )
    ctx.expect_overlap(
        optical_body,
        stand,
        axes="z",
        elem_a="carriage_plate",
        elem_b="focus_rail",
        min_overlap=0.055,
        name="optical carriage remains engaged on the rail",
    )
    ctx.expect_gap(
        stage,
        stand,
        axis="z",
        positive_elem="slide_block",
        negative_elem="stage_rail",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="stage carriage seats on the rail",
    )
    ctx.expect_within(
        stage,
        stand,
        axes="y",
        inner_elem="slide_block",
        outer_elem="stage_rail",
        margin=0.001,
        name="stage carriage stays centered on the rail",
    )
    ctx.expect_overlap(
        stage,
        stand,
        axes="x",
        elem_a="slide_block",
        elem_b="stage_rail",
        min_overlap=0.090,
        name="stage carriage remains captured at center",
    )
    ctx.expect_gap(
        turret,
        stage,
        axis="z",
        positive_elem="objective_0",
        negative_elem="stage_plate",
        min_gap=0.006,
        max_gap=0.020,
        name="objective clears the specimen stage at rest",
    )
    ctx.expect_contact(
        coarse_knob,
        stand,
        elem_a="coarse_shaft",
        elem_b="coarse_boss",
        name="coarse focus knob mounts on the arm side",
    )
    ctx.expect_contact(
        fine_knob,
        stand,
        elem_a="fine_shaft",
        elem_b="fine_boss",
        name="fine focus knob mounts on the arm side",
    )
    ctx.expect_origin_distance(
        coarse_knob,
        fine_knob,
        axes="yz",
        min_dist=0.014,
        name="coarse and fine knobs are visibly separate controls",
    )

    rest_optical = ctx.part_world_position(optical_body)
    upper_optical = None
    if optical_limits is not None and optical_limits.upper is not None:
        with ctx.pose({optical_slide: optical_limits.upper}):
            ctx.expect_contact(
                optical_body,
                stand,
                elem_a="carriage_plate",
                elem_b="focus_rail",
                name="optical carriage stays on the rail at full raise",
            )
            ctx.expect_overlap(
                optical_body,
                stand,
                axes="z",
                elem_a="carriage_plate",
                elem_b="focus_rail",
                min_overlap=0.055,
                name="optical carriage keeps rail engagement when raised",
            )
            upper_optical = ctx.part_world_position(optical_body)
    ctx.check(
        "optical body moves upward on the column",
        rest_optical is not None
        and upper_optical is not None
        and upper_optical[2] > rest_optical[2] + 0.040,
        details=f"rest={rest_optical}, raised={upper_optical}",
    )

    left_stage = None
    right_stage = None
    if stage_limits is not None and stage_limits.lower is not None:
        with ctx.pose({stage_slide: stage_limits.lower}):
            ctx.expect_gap(
                stage,
                stand,
                axis="z",
                positive_elem="slide_block",
                negative_elem="stage_rail",
                max_gap=0.0005,
                max_penetration=1e-6,
                name="stage carriage still seats on the rail at left travel",
            )
            ctx.expect_overlap(
                stage,
                stand,
                axes="x",
                elem_a="slide_block",
                elem_b="stage_rail",
                min_overlap=0.070,
                name="stage carriage remains retained at left travel",
            )
            left_stage = ctx.part_world_position(stage)
    if stage_limits is not None and stage_limits.upper is not None:
        with ctx.pose({stage_slide: stage_limits.upper}):
            ctx.expect_gap(
                stage,
                stand,
                axis="z",
                positive_elem="slide_block",
                negative_elem="stage_rail",
                max_gap=0.0005,
                max_penetration=1e-6,
                name="stage carriage still seats on the rail at right travel",
            )
            ctx.expect_overlap(
                stage,
                stand,
                axes="x",
                elem_a="slide_block",
                elem_b="stage_rail",
                min_overlap=0.070,
                name="stage carriage remains retained at right travel",
            )
            right_stage = ctx.part_world_position(stage)
    ctx.check(
        "stage translates left to right",
        left_stage is not None and right_stage is not None and right_stage[0] > left_stage[0] + 0.045,
        details=f"left={left_stage}, right={right_stage}",
    )

    objective_rest = _aabb_center(ctx.part_element_world_aabb(turret, elem="objective_0"))
    objective_rotated = None
    with ctx.pose({turret_spin: 2.0 * math.pi / 3.0}):
        objective_rotated = _aabb_center(ctx.part_element_world_aabb(turret, elem="objective_0"))
    ctx.check(
        "objective turret indexes around the optical axis",
        objective_rest is not None
        and objective_rotated is not None
        and (
            abs(objective_rotated[0] - objective_rest[0]) > 0.008
            or abs(objective_rotated[1] - objective_rest[1]) > 0.008
        ),
        details=f"rest={objective_rest}, rotated={objective_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
