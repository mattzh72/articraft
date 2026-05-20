from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metal_monocular_microscope")

    painted_metal = model.material("painted_metal", rgba=(0.20, 0.22, 0.23, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.018, 1.0))
    objective_black = model.material("objective_black", rgba=(0.03, 0.03, 0.035, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.42, 0.62, 0.78, 0.45))
    slide_glass = model.material("slide_glass", rgba=(0.76, 0.91, 1.00, 0.35))

    frame = model.part("frame")
    frame.visual(
        Box((0.260, 0.180, 0.035)),
        origin=Origin(xyz=(0.010, 0.0, 0.0175)),
        material=painted_metal,
        name="deep_base",
    )
    frame.visual(
        Box((0.175, 0.135, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, 0.040)),
        material=dark_metal,
        name="base_top_plinth",
    )
    frame.visual(
        Box((0.045, 0.065, 0.315)),
        origin=Origin(xyz=(-0.075, 0.0, 0.1925)),
        material=painted_metal,
        name="upright_arm",
    )
    frame.visual(
        Box((0.085, 0.052, 0.038)),
        origin=Origin(xyz=(-0.045, 0.0, 0.335)),
        material=painted_metal,
        name="arm_head_mass",
    )
    frame.visual(
        Box((0.120, 0.095, 0.025)),
        origin=Origin(xyz=(-0.015, 0.0, 0.105)),
        material=painted_metal,
        name="stage_cantilever",
    )
    frame.visual(
        Box((0.030, 0.058, 0.095)),
        origin=Origin(xyz=(-0.050, 0.0, 0.0875)),
        material=painted_metal,
        name="stage_support_web",
    )
    frame.visual(
        Box((0.130, 0.020, 0.012)),
        origin=Origin(xyz=(0.035, -0.038, 0.119)),
        material=brushed_steel,
        name="stage_rail_0",
    )
    frame.visual(
        Box((0.130, 0.020, 0.012)),
        origin=Origin(xyz=(0.035, 0.038, 0.119)),
        material=brushed_steel,
        name="stage_rail_1",
    )
    frame.visual(
        Box((0.112, 0.010, 0.010)),
        origin=Origin(xyz=(0.035, 0.0, 0.120)),
        material=brushed_steel,
        name="stage_center_way",
    )
    frame.visual(
        Box((0.006, 0.072, 0.270)),
        origin=Origin(xyz=(-0.050, 0.0, 0.222)),
        material=brushed_steel,
        name="focus_dovetail",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(-0.053, 0.0405, 0.255), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="coarse_focus_boss",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(-0.053, 0.0395, 0.205), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="fine_focus_boss",
    )

    stage_carriage = model.part("stage_carriage")
    stage_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.120, 0.090, 0.006, corner_segments=8),
            [_circle_profile(0.018, segments=48)],
            0.010,
            center=False,
        ),
        "rectangular_slide_stage",
    )
    stage_carriage.visual(
        stage_plate_mesh,
        origin=Origin(),
        material=brushed_steel,
        name="stage_plate",
    )
    stage_carriage.visual(
        Box((0.078, 0.026, 0.002)),
        origin=Origin(xyz=(0.004, 0.0, 0.011)),
        material=slide_glass,
        name="glass_slide",
    )
    stage_carriage.visual(
        Box((0.082, 0.006, 0.006)),
        origin=Origin(xyz=(0.000, 0.022, 0.013)),
        material=brushed_steel,
        name="slide_clip_0",
    )
    stage_carriage.visual(
        Box((0.082, 0.006, 0.006)),
        origin=Origin(xyz=(0.000, -0.022, 0.013)),
        material=brushed_steel,
        name="slide_clip_1",
    )
    stage_carriage.visual(
        Box((0.118, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -0.049, 0.006)),
        material=dark_metal,
        name="guide_bar",
    )
    for x_pos, mount_name in (
        (-0.030, "guide_control_mount_0"),
        (0.030, "guide_control_mount_1"),
    ):
        stage_carriage.visual(
            Box((0.018, 0.006, 0.014)),
            origin=Origin(xyz=(x_pos, -0.054, 0.007)),
            material=dark_metal,
            name=mount_name,
        )

    body_carriage = model.part("body_carriage")
    body_carriage.visual(
        Box((0.040, 0.070, 0.095)),
        origin=Origin(xyz=(0.020, 0.0, 0.000)),
        material=painted_metal,
        name="focus_slide_block",
    )
    body_carriage.visual(
        Box((0.070, 0.055, 0.036)),
        origin=Origin(xyz=(0.080, 0.0, 0.055)),
        material=painted_metal,
        name="monocular_head",
    )
    body_carriage.visual(
        Box((0.005, 0.055, 0.021)),
        origin=Origin(xyz=(0.0425, 0.0, 0.050)),
        material=painted_metal,
        name="head_neck",
    )
    body_carriage.visual(
        Cylinder(radius=0.019, length=0.168),
        origin=Origin(xyz=(0.0955, 0.0, 0.064)),
        material=dark_metal,
        name="optical_tube",
    )
    body_carriage.visual(
        Cylinder(radius=0.015, length=0.050),
        origin=Origin(xyz=(0.0955, 0.0, 0.173)),
        material=black_rubber,
        name="eyepiece_barrel",
    )
    body_carriage.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0955, 0.0, 0.200)),
        material=lens_glass,
        name="ocular_lens",
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=brushed_steel,
        name="revolving_turret",
    )
    nosepiece.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=brushed_steel,
        name="central_hub",
    )
    objective_specs = (
        (0.000, 0.000, 0.0095, 0.076, "objective_center", "objective_center_lens"),
        (0.025, 0.000, 0.0070, 0.056, "objective_0", "objective_0_lens"),
        (-0.0125, 0.0217, 0.0070, 0.050, "objective_1", "objective_1_lens"),
        (-0.0125, -0.0217, 0.0070, 0.050, "objective_2", "objective_2_lens"),
    )
    for x_pos, y_pos, radius, length, name, lens_name in objective_specs:
        nosepiece.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x_pos, y_pos, -0.014 - length / 2.0)),
            material=objective_black,
            name=name,
        )
        nosepiece.visual(
            Cylinder(radius=radius * 0.72, length=0.003),
            origin=Origin(xyz=(x_pos, y_pos, -0.014 - length - 0.0015)),
            material=lens_glass,
            name=lens_name,
        )

    coarse_knob = model.part("coarse_knob")
    coarse_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.047,
            0.030,
            body_style="cylindrical",
            grip=KnobGrip(style="fluted", count=28, depth=0.0015),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "coarse_focus_knob",
    )
    coarse_knob.visual(coarse_knob_mesh, material=black_rubber, name="coarse_knob_cap")

    fine_knob = model.part("fine_knob")
    fine_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.026,
            0.022,
            body_style="faceted",
            grip=KnobGrip(style="ribbed", count=16, depth=0.0008),
            indicator=KnobIndicator(style="line", mode="engraved", angle_deg=0.0),
            center=False,
        ),
        "fine_focus_knob",
    )
    fine_knob.visual(fine_knob_mesh, material=black_rubber, name="fine_knob_cap")

    for index in (0, 1):
        guide_knob = model.part(f"guide_control_{index}")
        guide_knob.visual(
            mesh_from_geometry(
                KnobGeometry(
                    0.018,
                    0.014,
                    body_style="cylindrical",
                    grip=KnobGrip(style="knurled", count=20, depth=0.0006),
                    center=False,
                ),
                f"stage_guide_control_{index}",
            ),
            material=black_rubber,
            name="guide_knob_cap",
        )

    model.articulation(
        "frame_to_stage_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=stage_carriage,
        origin=Origin(xyz=(0.035, 0.0, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.08, lower=-0.030, upper=0.030),
    )
    model.articulation(
        "frame_to_body_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=body_carriage,
        origin=Origin(xyz=(-0.0465, 0.0, 0.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.05, lower=0.0, upper=0.055),
    )
    model.articulation(
        "body_to_nosepiece",
        ArticulationType.CONTINUOUS,
        parent=body_carriage,
        child=nosepiece,
        origin=Origin(xyz=(0.0955, 0.0, -0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.5),
    )
    model.articulation(
        "frame_to_coarse_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=coarse_knob,
        origin=Origin(xyz=(-0.053, 0.0485, 0.255), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "frame_to_fine_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=fine_knob,
        origin=Origin(xyz=(-0.053, 0.0465, 0.205), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
    )
    for index, x_pos in enumerate((-0.030, 0.030)):
        model.articulation(
            f"stage_to_guide_control_{index}",
            ArticulationType.CONTINUOUS,
            parent=stage_carriage,
            child=f"guide_control_{index}",
            origin=Origin(xyz=(x_pos, -0.057, 0.007), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.08, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    stage = object_model.get_part("stage_carriage")
    body = object_model.get_part("body_carriage")
    nosepiece = object_model.get_part("nosepiece")
    coarse = object_model.get_part("coarse_knob")
    fine = object_model.get_part("fine_knob")
    guide_0 = object_model.get_part("guide_control_0")
    guide_1 = object_model.get_part("guide_control_1")

    stage_slide = object_model.get_articulation("frame_to_stage_carriage")
    focus_slide = object_model.get_articulation("frame_to_body_carriage")
    nose_spin = object_model.get_articulation("body_to_nosepiece")
    coarse_spin = object_model.get_articulation("frame_to_coarse_knob")
    fine_spin = object_model.get_articulation("frame_to_fine_knob")
    guide_spin_0 = object_model.get_articulation("stage_to_guide_control_0")
    guide_spin_1 = object_model.get_articulation("stage_to_guide_control_1")

    ctx.check(
        "primary microscope mechanisms are articulated",
        stage_slide.articulation_type == ArticulationType.PRISMATIC
        and focus_slide.articulation_type == ArticulationType.PRISMATIC
        and nose_spin.articulation_type == ArticulationType.CONTINUOUS
        and coarse_spin.articulation_type == ArticulationType.CONTINUOUS
        and fine_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"stage={stage_slide.articulation_type}, focus={focus_slide.articulation_type}, "
            f"nose={nose_spin.articulation_type}, coarse={coarse_spin.articulation_type}, "
            f"fine={fine_spin.articulation_type}"
        ),
    )
    ctx.check(
        "coarse and fine knobs are separate controls",
        coarse.name != fine.name
        and abs(coarse_spin.origin.xyz[2] - fine_spin.origin.xyz[2]) > 0.030
        and coarse_spin.child != fine_spin.child,
        details=f"coarse={coarse_spin.origin.xyz}, fine={fine_spin.origin.xyz}",
    )
    ctx.check(
        "two stage guide controls are rotary",
        guide_spin_0.articulation_type == ArticulationType.CONTINUOUS
        and guide_spin_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"guide0={guide_spin_0.articulation_type}, guide1={guide_spin_1.articulation_type}",
    )

    ctx.expect_gap(
        stage,
        frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="stage_plate",
        negative_elem="stage_rail_0",
        name="stage carriage rides on fixed rail",
    )
    ctx.expect_gap(
        body,
        frame,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="focus_slide_block",
        negative_elem="focus_dovetail",
        name="focusing carriage is guided on the arm dovetail",
    )
    ctx.expect_gap(
        coarse,
        frame,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="coarse_knob_cap",
        negative_elem="coarse_focus_boss",
        name="coarse knob seats on its own boss",
    )
    ctx.expect_gap(
        fine,
        frame,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="fine_knob_cap",
        negative_elem="fine_focus_boss",
        name="fine knob seats on its own boss",
    )
    ctx.expect_gap(
        stage,
        guide_0,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="guide_control_mount_0",
        negative_elem="guide_knob_cap",
        name="first guide control mounts to stage carriage",
    )
    ctx.expect_gap(
        stage,
        guide_1,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="guide_control_mount_1",
        negative_elem="guide_knob_cap",
        name="second guide control mounts to stage carriage",
    )
    ctx.expect_gap(
        nosepiece,
        stage,
        axis="z",
        min_gap=0.001,
        positive_elem="objective_center_lens",
        negative_elem="glass_slide",
        name="objective clears the glass slide at the lower focus stop",
    )

    rest_stage = ctx.part_world_position(stage)
    with ctx.pose({stage_slide: 0.030}):
        extended_stage = ctx.part_world_position(stage)
    ctx.check(
        "stage carriage translates along the stage axis",
        rest_stage is not None
        and extended_stage is not None
        and extended_stage[0] > rest_stage[0] + 0.025
        and abs(extended_stage[1] - rest_stage[1]) < 0.001,
        details=f"rest={rest_stage}, extended={extended_stage}",
    )

    rest_body = ctx.part_world_position(body)
    with ctx.pose({focus_slide: 0.055}):
        raised_body = ctx.part_world_position(body)
    ctx.check(
        "body carriage focuses upward on the arm",
        rest_body is not None
        and raised_body is not None
        and raised_body[2] > rest_body[2] + 0.050,
        details=f"rest={rest_body}, raised={raised_body}",
    )

    rest_nose = ctx.part_world_position(nosepiece)
    with ctx.pose({nose_spin: math.pi / 2.0}):
        rotated_nose = ctx.part_world_position(nosepiece)
    ctx.check(
        "nosepiece rotation stays on the optical axis",
        rest_nose is not None
        and rotated_nose is not None
        and abs(rotated_nose[0] - rest_nose[0]) < 0.001
        and abs(rotated_nose[1] - rest_nose[1]) < 0.001,
        details=f"rest={rest_nose}, rotated={rotated_nose}",
    )

    return ctx.report()


object_model = build_object_model()
