from __future__ import annotations

from math import cos, pi, sin

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


def _rounded_box_shape(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded box with its bottom face on local z=0."""
    sx, sy, sz = size
    fillet = min(radius, sx * 0.20, sy * 0.20, sz * 0.20)
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz)
        .edges()
        .fillet(fillet)
        .translate((0.0, 0.0, sz / 2.0))
    )


def _curved_arm_shape() -> cq.Workplane:
    """Low, swept side plate that forms the microscope's short curved arm."""
    # Coordinates are (front/back Y, vertical Z) in the YZ workplane.
    pts = [
        (0.070, 0.040),
        (0.090, 0.040),
        (0.093, 0.080),
        (0.091, 0.130),
        (0.086, 0.190),
        (0.069, 0.252),
        (0.034, 0.296),
        (-0.012, 0.309),
        (-0.054, 0.306),
        (-0.066, 0.282),
        (-0.051, 0.244),
        (-0.009, 0.235),
        (0.028, 0.202),
        (0.037, 0.144),
        (0.032, 0.079),
        (0.052, 0.048),
    ]
    wp = cq.Workplane("YZ").moveTo(*pts[0])
    for p in pts[1:]:
        wp = wp.lineTo(*p)
    return wp.close().extrude(0.065).translate((-0.0325, 0.0, 0.0))


def _stage_plate_shape() -> cq.Workplane:
    """Compact square stage plate with a central optical aperture."""
    return (
        cq.Workplane("XY")
        .box(0.105, 0.095, 0.010)
        .edges("|Z")
        .fillet(0.004)
        .faces(">Z")
        .workplane()
        .hole(0.026)
        .translate((0.0, 0.0, 0.005))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_monocular_microscope")

    model.material("black_enamel", rgba=(0.035, 0.040, 0.045, 1.0))
    model.material("charcoal", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("warm_gray", rgba=(0.42, 0.43, 0.42, 1.0))
    model.material("stage_metal", rgba=(0.72, 0.73, 0.70, 1.0))
    model.material("polished_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("rubber", rgba=(0.015, 0.016, 0.017, 1.0))
    model.material("glass", rgba=(0.45, 0.72, 0.92, 0.55))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_rounded_box_shape((0.240, 0.220, 0.040), 0.012), "thick_base"),
        material="black_enamel",
        name="thick_base",
    )
    stand.visual(
        mesh_from_cadquery(_curved_arm_shape(), "curved_arm"),
        material="warm_gray",
        name="curved_arm",
    )
    stand.visual(
        Box((0.065, 0.065, 0.052)),
        origin=Origin(xyz=(0.0, -0.085, 0.066)),
        material="charcoal",
        name="stage_pedestal",
    )
    for y_pos, name in ((-0.105, "stage_rail_0"), (-0.065, "stage_rail_1")):
        stand.visual(
            Cylinder(radius=0.0035, length=0.165),
            origin=Origin(xyz=(0.0, y_pos, 0.0955), rpy=(0.0, pi / 2.0, 0.0)),
            material="polished_steel",
            name=name,
        )
    for x_pos, name in ((-0.045, "guide_rod_0"), (0.045, "guide_rod_1")):
        stand.visual(
            Cylinder(radius=0.0045, length=0.150),
            origin=Origin(xyz=(x_pos, -0.0675, 0.220)),
            material="polished_steel",
            name=name,
        )
    stand.visual(
        Box((0.024, 0.019, 0.018)),
        origin=Origin(xyz=(-0.045, -0.0675, 0.145)),
        material="charcoal",
        name="lower_guide_block_0",
    )
    stand.visual(
        Box((0.024, 0.019, 0.018)),
        origin=Origin(xyz=(0.045, -0.0675, 0.145)),
        material="charcoal",
        name="lower_guide_block_1",
    )
    stand.visual(
        Box((0.120, 0.019, 0.018)),
        origin=Origin(xyz=(0.0, -0.0675, 0.294)),
        material="charcoal",
        name="upper_guide_block",
    )

    stage_carrier = model.part("stage_carrier")
    stage_carrier.visual(
        Box((0.130, 0.052, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="charcoal",
        name="slide_saddle",
    )
    stage_carrier.visual(
        mesh_from_cadquery(_stage_plate_shape(), "stage_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material="stage_metal",
        name="stage_plate",
    )
    stage_carrier.visual(
        Cylinder(radius=0.016, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.01775)),
        material="rubber",
        name="black_aperture",
    )
    for x_pos, name in ((-0.026, "slide_clip_0"), (0.026, "slide_clip_1")):
        stage_carrier.visual(
            Box((0.034, 0.006, 0.003)),
            origin=Origin(xyz=(x_pos, 0.032, 0.0185)),
            material="polished_steel",
            name=name,
        )

    head_carriage = model.part("head_carriage")
    head_carriage.visual(
        Box((0.110, 0.026, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material="charcoal",
        name="slide_block",
    )
    head_carriage.visual(
        mesh_from_cadquery(_rounded_box_shape((0.075, 0.078, 0.056), 0.008), "head_shell"),
        origin=Origin(xyz=(0.0, -0.028, 0.002)),
        material="black_enamel",
        name="head_shell",
    )
    # The socket face terminates at the turret joint plane; the rotating disk
    # contacts it from below without using a broad inter-part overlap.
    head_carriage.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material="black_enamel",
        name="turret_socket",
    )
    eyepiece_roll = 0.70
    tube_axis_y = -sin(eyepiece_roll)
    tube_axis_z = cos(eyepiece_roll)
    tube_start = (0.0, -0.030, 0.054)
    tube_len = 0.086
    tube_center = (
        0.0,
        tube_start[1] + tube_axis_y * tube_len / 2.0,
        tube_start[2] + tube_axis_z * tube_len / 2.0,
    )
    head_carriage.visual(
        Cylinder(radius=0.016, length=tube_len),
        origin=Origin(xyz=tube_center, rpy=(eyepiece_roll, 0.0, 0.0)),
        material="black_enamel",
        name="eyepiece_tube",
    )
    cap_len = 0.026
    cap_center = (
        0.0,
        tube_start[1] + tube_axis_y * (tube_len + cap_len / 2.0),
        tube_start[2] + tube_axis_z * (tube_len + cap_len / 2.0),
    )
    head_carriage.visual(
        Cylinder(radius=0.020, length=cap_len),
        origin=Origin(xyz=cap_center, rpy=(eyepiece_roll, 0.0, 0.0)),
        material="rubber",
        name="eyepiece_cup",
    )
    lens_center = (
        0.0,
        tube_start[1] + tube_axis_y * (tube_len + cap_len + 0.001),
        tube_start[2] + tube_axis_z * (tube_len + cap_len + 0.001),
    )
    head_carriage.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(xyz=lens_center, rpy=(eyepiece_roll, 0.0, 0.0)),
        material="glass",
        name="eyepiece_glass",
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material="charcoal",
        name="turret_disk",
    )
    for i, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        x_pos = 0.018 * sin(angle)
        y_pos = -0.018 * cos(angle)
        nosepiece.visual(
            Cylinder(radius=0.0065, length=0.040),
            origin=Origin(xyz=(x_pos, y_pos, -0.031)),
            material="polished_steel",
            name=f"objective_{i}",
        )
        nosepiece.visual(
            Cylinder(radius=0.0075, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, -0.041)),
            material="black_enamel",
            name=f"objective_band_{i}",
        )
        nosepiece.visual(
            Cylinder(radius=0.0045, length=0.0025),
            origin=Origin(xyz=(x_pos, y_pos, -0.05225)),
            material="glass",
            name=f"objective_lens_{i}",
        )

    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage_carrier,
        origin=Origin(xyz=(0.0, -0.085, 0.106)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.030, upper=0.030, effort=24.0, velocity=0.12),
    )
    model.articulation(
        "focus_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=head_carriage,
        origin=Origin(xyz=(0.0, -0.085, 0.198)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.055, effort=18.0, velocity=0.08),
    )
    model.articulation(
        "turret",
        ArticulationType.REVOLUTE,
        parent=head_carriage,
        child=nosepiece,
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.20, upper=2.20, effort=1.2, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    stage_carrier = object_model.get_part("stage_carrier")
    head_carriage = object_model.get_part("head_carriage")
    nosepiece = object_model.get_part("nosepiece")
    stage_slide = object_model.get_articulation("stage_slide")
    focus_slide = object_model.get_articulation("focus_slide")
    turret = object_model.get_articulation("turret")

    ctx.expect_gap(
        nosepiece,
        stage_carrier,
        axis="z",
        min_gap=0.002,
        max_gap=0.020,
        name="objectives clear the stage at low focus",
    )
    ctx.expect_overlap(
        nosepiece,
        stage_carrier,
        axes="xy",
        min_overlap=0.030,
        name="objective turret is centered over the square stage",
    )
    ctx.expect_overlap(
        stage_carrier,
        stand,
        axes="x",
        elem_a="slide_saddle",
        elem_b="stage_rail_0",
        min_overlap=0.080,
        name="stage saddle remains engaged on the fixed rail",
    )

    lower_stage_pos = None
    upper_stage_pos = None
    with ctx.pose({stage_slide: -0.030}):
        lower_stage_pos = ctx.part_world_position(stage_carrier)
    with ctx.pose({stage_slide: 0.030}):
        upper_stage_pos = ctx.part_world_position(stage_carrier)
    ctx.check(
        "stage carrier slides left to right",
        lower_stage_pos is not None
        and upper_stage_pos is not None
        and upper_stage_pos[0] > lower_stage_pos[0] + 0.055,
        details=f"lower={lower_stage_pos}, upper={upper_stage_pos}",
    )

    rest_head_pos = ctx.part_world_position(head_carriage)
    with ctx.pose({focus_slide: 0.055}):
        raised_head_pos = ctx.part_world_position(head_carriage)
        ctx.expect_gap(
            nosepiece,
            stage_carrier,
            axis="z",
            min_gap=0.050,
            name="raised focus increases objective clearance",
        )
    ctx.check(
        "head carriage moves up the vertical guide",
        rest_head_pos is not None
        and raised_head_pos is not None
        and raised_head_pos[2] > rest_head_pos[2] + 0.050,
        details=f"rest={rest_head_pos}, raised={raised_head_pos}",
    )

    def _xy_center(aabb):
        return ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][1] + aabb[1][1]) / 2.0)

    rest_obj = ctx.part_element_world_aabb(nosepiece, elem="objective_0")
    with ctx.pose({turret: 2.0 * pi / 3.0}):
        turned_obj = ctx.part_element_world_aabb(nosepiece, elem="objective_0")
    if rest_obj is not None and turned_obj is not None:
        rest_xy = _xy_center(rest_obj)
        turned_xy = _xy_center(turned_obj)
        turret_travel = ((turned_xy[0] - rest_xy[0]) ** 2 + (turned_xy[1] - rest_xy[1]) ** 2) ** 0.5
    else:
        turret_travel = 0.0
    ctx.check(
        "nosepiece rotates around the central turret",
        turret_travel > 0.025,
        details=f"objective_0_xy_travel={turret_travel}",
    )

    return ctx.report()


object_model = build_object_model()
