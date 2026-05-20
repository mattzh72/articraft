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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _tube_angle_rpy(dy: float, dz: float) -> tuple[float, float, float]:
    """Rotate a cylinder's local +Z axis into a vector in the YZ plane."""
    length = math.hypot(dy, dz)
    if length <= 0.0:
        return (0.0, 0.0, 0.0)
    return (-math.asin(dy / length), 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_teaching_microscope")

    ivory = model.material("ivory_enamel", rgba=(0.86, 0.84, 0.76, 1.0))
    black = model.material("matte_black", rgba=(0.03, 0.035, 0.04, 1.0))
    dark = model.material("dark_rubber", rgba=(0.01, 0.012, 0.014, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.68, 0.70, 0.70, 1.0))
    glass = model.material("blue_glass", rgba=(0.30, 0.48, 0.65, 0.55))
    rail = model.material("satin_rail", rgba=(0.42, 0.44, 0.43, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.260, 0.180, 0.020, corner_segments=10),
                0.032,
            ),
            "microscope_rounded_base",
        ),
        material=ivory,
        name="broad_base",
    )
    for x in (-0.095, 0.095):
        for y in (-0.065, 0.065):
            stand.visual(
                Cylinder(radius=0.014, length=0.008),
                origin=Origin(xyz=(x, y, 0.002)),
                material=dark,
                name=f"rubber_foot_{x}_{y}",
            )
    stand.visual(
        Box((0.052, 0.035, 0.305)),
        origin=Origin(xyz=(0.0, 0.055, 0.1845)),
        material=ivory,
        name="focusing_column",
    )
    stand.visual(
        Box((0.040, 0.006, 0.225)),
        origin=Origin(xyz=(0.0, 0.0405, 0.205)),
        material=rail,
        name="focus_rail",
    )
    stand.visual(
        Box((0.150, 0.110, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, 0.090)),
        material=ivory,
        name="stage_support",
    )
    stand.visual(
        Box((0.050, 0.036, 0.076)),
        origin=Origin(xyz=(0.0, 0.025, 0.061)),
        material=ivory,
        name="arm_throat",
    )
    stand.visual(
        Box((0.030, 0.065, 0.040)),
        origin=Origin(xyz=(0.0, 0.020, 0.075)),
        material=ivory,
        name="stage_neck",
    )

    stage = model.part("stage")
    stage.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.145, 0.120, 0.010, corner_segments=8),
                [_circle_profile(0.018, segments=40)],
                0.012,
                center=True,
            ),
            "mechanical_stage_plate",
        ),
        material=black,
        name="stage_plate",
    )
    stage.visual(
        mesh_from_geometry(TorusGeometry(radius=0.017, tube=0.003), "stage_aperture_bezel"),
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
        material=chrome,
        name="aperture_bezel",
    )
    stage.visual(
        Box((0.134, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, -0.057, 0.0085)),
        material=rail,
        name="guide_rail_0",
    )
    stage.visual(
        Box((0.134, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, -0.044, 0.0085)),
        material=rail,
        name="guide_rail_1",
    )
    stage.visual(
        Box((0.065, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.040, 0.0075)),
        material=chrome,
        name="slide_clip",
    )

    side_carriage = model.part("side_carriage")
    side_carriage.visual(
        Box((0.090, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rail,
        name="carriage_body",
    )
    for x in (-0.035, 0.035):
        side_carriage.visual(
            Box((0.008, 0.056, 0.004)),
            origin=Origin(xyz=(x, 0.018, 0.008)),
            material=rail,
            name=f"holder_arm_{x}",
        )
    side_carriage.visual(
        Box((0.084, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.045, 0.010)),
        material=chrome,
        name="slide_holder",
    )
    side_carriage.visual(
        Box((0.030, 0.006, 0.006)),
        origin=Origin(xyz=(0.040, 0.053, 0.014)),
        material=chrome,
        name="spring_finger",
    )

    optical_body = model.part("optical_body")
    optical_body.visual(
        Box((0.065, 0.020, 0.090)),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=ivory,
        name="focus_slider",
    )
    optical_body.visual(
        Cylinder(radius=0.032, length=0.070),
        origin=Origin(xyz=(0.0, -0.047, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ivory,
        name="head_shell",
    )
    optical_body.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(xyz=(0.0, -0.065, -0.018)),
        material=ivory,
        name="nosepiece_stem",
    )
    optical_body.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, -0.065, -0.042)),
        material=chrome,
        name="nosepiece_collar",
    )

    tube_dy = 0.050
    tube_dz = 0.112
    tube_length = math.hypot(tube_dy, tube_dz)
    tube_center = (0.0, -0.040 + tube_dy * 0.5, 0.055 + tube_dz * 0.5)
    tube_rpy = _tube_angle_rpy(tube_dy, tube_dz)
    optical_body.visual(
        Cylinder(radius=0.014, length=tube_length),
        origin=Origin(xyz=tube_center, rpy=tube_rpy),
        material=black,
        name="monocular_tube",
    )
    optical_body.visual(
        Cylinder(radius=0.018, length=0.025),
        origin=Origin(
            xyz=(0.0, -0.040 + tube_dy + 0.004, 0.055 + tube_dz + 0.010),
            rpy=tube_rpy,
        ),
        material=dark,
        name="eyepiece_ring",
    )
    optical_body.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(
            xyz=(
                0.0,
                -0.040 + tube_dy + (tube_dy / tube_length) * 0.024,
                0.055 + tube_dz + (tube_dz / tube_length) * 0.024,
            ),
            rpy=tube_rpy,
        ),
        material=glass,
        name="eyepiece_lens",
    )

    objective_turret = model.part("objective_turret")
    objective_turret.visual(
        Cylinder(radius=0.033, length=0.010),
        material=chrome,
        name="turret_plate",
    )
    objective_turret.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=black,
        name="index_ring",
    )
    objective_positions = [
        (0.022, 0.000, 0.045, "long_objective"),
        (-0.022, 0.000, 0.034, "short_objective"),
        (0.000, 0.022, 0.038, "middle_objective"),
        (0.000, -0.022, 0.030, "scan_objective"),
    ]
    for x, y, length, name in objective_positions:
        objective_turret.visual(
            Cylinder(radius=0.0065, length=0.018),
            origin=Origin(xyz=(x, y, -0.013)),
            material=chrome,
            name=f"{name}_barrel",
        )
        objective_turret.visual(
            Cylinder(radius=0.0048, length=length),
            origin=Origin(xyz=(x, y, -0.021 - 0.5 * length)),
            material=black,
            name=name,
        )

    coarse_knob = model.part("coarse_knob")
    coarse_knob.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="coarse_shaft",
    )
    coarse_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.042,
                0.020,
                body_style="cylindrical",
                grip=KnobGrip(style="fluted", count=28, depth=0.0018),
                center=False,
            ),
            "coarse_focus_knob",
        ),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="coarse_wheel",
    )

    fine_knob = model.part("fine_knob")
    fine_knob.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="fine_shaft",
    )
    fine_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.026,
                0.014,
                body_style="faceted",
                grip=KnobGrip(style="ribbed", count=18, depth=0.0010, width=0.0012),
                center=False,
            ),
            "fine_focus_knob",
        ),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="fine_wheel",
    )

    model.articulation(
        "stand_to_stage",
        ArticulationType.FIXED,
        parent=stand,
        child=stage,
        origin=Origin(xyz=(0.0, -0.025, 0.105)),
    )
    model.articulation(
        "stage_to_side_carriage",
        ArticulationType.PRISMATIC,
        parent=stage,
        child=side_carriage,
        origin=Origin(xyz=(0.0, -0.049, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=-0.030, upper=0.030),
    )
    model.articulation(
        "stand_to_optical_body",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=optical_body,
        origin=Origin(xyz=(0.0, 0.0375, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.04, lower=0.0, upper=0.040),
    )
    model.articulation(
        "optical_body_to_turret",
        ArticulationType.CONTINUOUS,
        parent=optical_body,
        child=objective_turret,
        origin=Origin(xyz=(0.0, -0.065, -0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=3.0),
    )
    model.articulation(
        "stand_to_coarse_knob",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=coarse_knob,
        origin=Origin(xyz=(0.026, 0.055, 0.200)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )
    model.articulation(
        "stand_to_fine_knob",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=fine_knob,
        origin=Origin(xyz=(0.026, 0.055, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    stage = object_model.get_part("stage")
    carriage = object_model.get_part("side_carriage")
    optical = object_model.get_part("optical_body")
    turret = object_model.get_part("objective_turret")
    coarse = object_model.get_part("coarse_knob")
    fine = object_model.get_part("fine_knob")

    for joint_name in (
        "optical_body_to_turret",
        "stand_to_optical_body",
        "stage_to_side_carriage",
        "stand_to_coarse_knob",
        "stand_to_fine_knob",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name}_present", joint is not None, f"Missing {joint_name}")

    ctx.expect_gap(
        stage,
        stand,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="stage_plate",
        negative_elem="stage_support",
        name="stage rests separately on support",
    )
    ctx.expect_gap(
        carriage,
        stage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="carriage_body",
        negative_elem="guide_rail_0",
        name="side carriage rides on stage guide",
    )
    ctx.expect_contact(
        turret,
        optical,
        elem_a="turret_plate",
        elem_b="nosepiece_collar",
        contact_tol=0.001,
        name="turret seats on nosepiece collar",
    )
    ctx.expect_origin_distance(
        coarse,
        fine,
        axes="z",
        min_dist=0.030,
        name="focus knobs are independent controls",
    )

    focus_joint = object_model.get_articulation("stand_to_optical_body")
    carriage_joint = object_model.get_articulation("stage_to_side_carriage")
    turret_joint = object_model.get_articulation("optical_body_to_turret")
    coarse_joint = object_model.get_articulation("stand_to_coarse_knob")
    fine_joint = object_model.get_articulation("stand_to_fine_knob")

    focus_rest = ctx.part_world_position(optical)
    with ctx.pose({focus_joint: 0.035}):
        focus_high = ctx.part_world_position(optical)
    ctx.check(
        "optical body slides upward on column",
        focus_rest is not None and focus_high is not None and focus_high[2] > focus_rest[2] + 0.030,
        details=f"rest={focus_rest}, high={focus_high}",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({carriage_joint: 0.025}):
        carriage_shifted = ctx.part_world_position(carriage)
    ctx.check(
        "side carriage slides along guide",
        carriage_rest is not None
        and carriage_shifted is not None
        and carriage_shifted[0] > carriage_rest[0] + 0.020,
        details=f"rest={carriage_rest}, shifted={carriage_shifted}",
    )

    with ctx.pose(
        {
            turret_joint: math.pi,
            coarse_joint: 1.25,
            fine_joint: -1.75,
        }
    ):
        ctx.expect_gap(
            turret,
            stage,
            axis="z",
            min_gap=0.010,
            name="rotated objectives clear the stage",
        )

    return ctx.report()


object_model = build_object_model()
