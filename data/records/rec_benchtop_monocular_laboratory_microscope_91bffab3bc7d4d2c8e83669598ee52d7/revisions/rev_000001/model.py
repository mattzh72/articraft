from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="student_monocular_microscope")

    enamel = model.material("warm_gray_enamel", rgba=(0.78, 0.79, 0.75, 1.0))
    black = model.material("satin_black", rgba=(0.03, 0.035, 0.04, 1.0))
    dark = model.material("dark_metal", rgba=(0.14, 0.15, 0.16, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    glass = model.material("pale_glass", rgba=(0.65, 0.90, 1.0, 0.42))
    brass = model.material("objective_brass", rgba=(0.88, 0.66, 0.33, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    # A classroom monocular microscope is roughly 40 cm tall; use a single
    # connected root casting for the rectangular foot, curved rear arm, fixed
    # stage, and exposed guide hardware.
    base = model.part("base")
    base.visual(
        Box((0.255, 0.165, 0.030)),
        origin=Origin(xyz=(0.005, 0.0, 0.015)),
        material=enamel,
        name="rectangular_foot",
    )
    base.visual(
        Box((0.070, 0.095, 0.058)),
        origin=Origin(xyz=(-0.080, 0.0, 0.059)),
        material=enamel,
        name="rear_plinth",
    )

    arm_mesh = save_mesh(
        "curved_support_arm",
        sweep_profile_along_spline(
            [
                (-0.082, 0.0, 0.055),
                (-0.102, 0.0, 0.155),
                (-0.092, 0.0, 0.270),
                (-0.066, 0.0, 0.365),
                (-0.044, 0.0, 0.410),
                (-0.028, 0.0, 0.398),
            ],
            profile=rounded_rect_profile(0.038, 0.055, radius=0.010, corner_segments=8),
            samples_per_segment=14,
            cap_profile=True,
        ),
    )
    base.visual(arm_mesh, material=enamel, name="curved_arm")

    # Visible rack/rail column that actually carries the moving optical head.
    base.visual(
        Box((0.020, 0.078, 0.245)),
        origin=Origin(xyz=(-0.032, 0.0, 0.285)),
        material=dark,
        name="guide_backplate",
    )
    base.visual(
        Box((0.064, 0.088, 0.022)),
        origin=Origin(xyz=(-0.018, 0.0, 0.170)),
        material=dark,
        name="lower_guide_yoke",
    )
    base.visual(
        Box((0.064, 0.088, 0.022)),
        origin=Origin(xyz=(-0.018, 0.0, 0.400)),
        material=dark,
        name="upper_guide_yoke",
    )
    base.visual(
        Cylinder(radius=0.0048, length=0.230),
        origin=Origin(xyz=(-0.003, -0.025, 0.285)),
        material=steel,
        name="guide_rod_0",
    )
    base.visual(
        Cylinder(radius=0.0048, length=0.230),
        origin=Origin(xyz=(-0.003, 0.025, 0.285)),
        material=steel,
        name="guide_rod_1",
    )

    # Focus knob bearing bosses on the side of the arm.  The two controls are
    # intentionally separate stations rather than one fused drum.
    base.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(-0.036, 0.048, 0.312), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="coarse_bearing",
    )
    base.visual(
        Cylinder(radius=0.0075, length=0.022),
        origin=Origin(xyz=(-0.036, 0.048, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="fine_bearing",
    )

    # Fixed slide stage built as a connected frame around a visible aperture.
    stage_z = 0.138
    base.visual(
        Box((0.042, 0.078, 0.108)),
        origin=Origin(xyz=(0.040, 0.0, 0.084)),
        material=enamel,
        name="stage_support",
    )
    base.visual(
        Box((0.122, 0.022, 0.012)),
        origin=Origin(xyz=(0.045, 0.049, stage_z)),
        material=black,
        name="stage_front_rail",
    )
    base.visual(
        Box((0.122, 0.022, 0.012)),
        origin=Origin(xyz=(0.045, -0.049, stage_z)),
        material=black,
        name="stage_rear_rail",
    )
    base.visual(
        Box((0.026, 0.080, 0.012)),
        origin=Origin(xyz=(0.096, 0.0, stage_z)),
        material=black,
        name="stage_side_rail_0",
    )
    base.visual(
        Box((0.026, 0.080, 0.012)),
        origin=Origin(xyz=(-0.006, 0.0, stage_z)),
        material=black,
        name="stage_side_rail_1",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.007),
        origin=Origin(xyz=(0.045, 0.0, 0.128)),
        material=glass,
        name="condenser_lens",
    )
    base.visual(
        Box((0.020, 0.050, 0.020)),
        origin=Origin(xyz=(0.045, 0.0, 0.111)),
        material=dark,
        name="condenser_mount",
    )

    # Moving optical head and carriage.  The child frame is on the exposed
    # vertical guide centerline; positive prismatic motion raises the head.
    head = model.part("head_carriage")
    collar_mesh = save_mesh(
        "guide_collar_ring",
        TorusGeometry(radius=0.0066, tube=0.0018, radial_segments=12, tubular_segments=36),
    )
    head.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, -0.025, -0.025)),
        material=steel,
        name="collar_0_lower",
    )
    head.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, -0.025, 0.025)),
        material=steel,
        name="collar_0_upper",
    )
    head.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.025, -0.025)),
        material=steel,
        name="collar_1_lower",
    )
    head.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.025, 0.025)),
        material=steel,
        name="collar_1_upper",
    )
    head.visual(
        Box((0.052, 0.034, 0.050)),
        origin=Origin(xyz=(0.012, 0.0, 0.000)),
        material=dark,
        name="sliding_carriage_block",
    )
    head.visual(
        Box((0.085, 0.070, 0.050)),
        origin=Origin(xyz=(0.070, 0.0, 0.004)),
        material=black,
        name="optical_head_body",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.032),
        origin=Origin(xyz=(0.082, 0.0, -0.030)),
        material=dark,
        name="nosepiece_socket",
    )
    eyepiece_tube = save_mesh(
        "angled_monocular_tube",
        tube_from_spline_points(
            [
                (0.060, 0.0, 0.032),
                (0.076, 0.0, 0.078),
                (0.092, 0.0, 0.122),
            ],
            radius=0.013,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    head.visual(eyepiece_tube, material=black, name="monocular_tube")
    head.visual(
        Cylinder(radius=0.017, length=0.026),
        origin=Origin(xyz=(0.096, 0.0, 0.134)),
        material=dark,
        name="eyepiece_rim",
    )

    turret = model.part("objective_turret")
    turret.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=dark,
        name="rotating_nosepiece_disk",
    )
    for idx, angle in enumerate((math.radians(90), math.radians(210), math.radians(330))):
        x = 0.017 * math.cos(angle)
        y = 0.017 * math.sin(angle)
        turret.visual(
            Cylinder(radius=0.0065, length=0.045),
            origin=Origin(xyz=(x, y, -0.035)),
            material=brass if idx == 0 else steel,
            name=f"objective_barrel_{idx}",
        )
        turret.visual(
            Cylinder(radius=0.0042, length=0.018),
            origin=Origin(xyz=(x, y, -0.066)),
            material=black,
            name=f"objective_tip_{idx}",
        )

    stage_carriage = model.part("stage_carriage")
    stage_carriage.visual(
        Box((0.106, 0.082, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="cross_slide_plate",
    )
    stage_carriage.visual(
        Box((0.055, 0.056, 0.005)),
        origin=Origin(xyz=(0.027, 0.037, 0.0035)),
        material=steel,
        name="projecting_carriage",
    )
    stage_carriage.visual(
        Box((0.092, 0.026, 0.0022)),
        origin=Origin(xyz=(0.005, 0.000, 0.0071)),
        material=glass,
        name="glass_slide",
    )
    stage_carriage.visual(
        Box((0.018, 0.010, 0.010)),
        origin=Origin(xyz=(-0.035, 0.016, 0.012)),
        material=dark,
        name="slide_clip_0",
    )
    stage_carriage.visual(
        Box((0.018, 0.010, 0.010)),
        origin=Origin(xyz=(0.045, 0.016, 0.012)),
        material=dark,
        name="slide_clip_1",
    )
    stage_carriage.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(xyz=(0.054, 0.069, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="carriage_thumb_pin",
    )

    coarse_knob = model.part("coarse_knob")
    coarse_mesh = save_mesh(
        "coarse_focus_knob",
        KnobGeometry(
            0.047,
            0.022,
            body_style="lobed",
            grip=KnobGrip(style="ribbed", count=18, depth=0.0014),
            edge_radius=0.001,
        ),
    )
    coarse_knob.visual(
        Cylinder(radius=0.0055, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="coarse_shaft",
    )
    coarse_knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="coarse_hub",
    )
    coarse_knob.visual(
        coarse_mesh,
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="coarse_fluted_wheel",
    )

    fine_knob = model.part("fine_knob")
    fine_mesh = save_mesh(
        "fine_focus_knob",
        KnobGeometry(
            0.027,
            0.016,
            body_style="faceted",
            grip=KnobGrip(style="knurled", count=28, depth=0.0008, helix_angle_deg=18.0),
            edge_radius=0.0007,
        ),
    )
    fine_knob.visual(
        Cylinder(radius=0.0035, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fine_shaft",
    )
    fine_knob.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="fine_hub",
    )
    fine_knob.visual(
        fine_mesh,
        origin=Origin(xyz=(0.0, 0.033, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="fine_knurled_wheel",
    )

    model.articulation(
        "base_to_head_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.003, 0.0, 0.285)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.050, lower=-0.035, upper=0.045),
    )
    model.articulation(
        "head_to_turret",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=turret,
        origin=Origin(xyz=(0.082, 0.0, -0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0),
    )
    model.articulation(
        "stage_cross_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_carriage,
        origin=Origin(xyz=(0.045, 0.0, stage_z + 0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.030, lower=-0.030, upper=0.030),
    )
    model.articulation(
        "coarse_focus_axis",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=coarse_knob,
        origin=Origin(xyz=(-0.036, 0.061, 0.312)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )
    model.articulation(
        "fine_focus_axis",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=fine_knob,
        origin=Origin(xyz=(-0.036, 0.059, 0.245)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    head = object_model.get_part("head_carriage")
    base = object_model.get_part("base")
    turret = object_model.get_part("objective_turret")
    stage_carriage = object_model.get_part("stage_carriage")
    coarse = object_model.get_part("coarse_knob")
    fine = object_model.get_part("fine_knob")
    head_slide = object_model.get_articulation("base_to_head_carriage")
    stage_slide = object_model.get_articulation("stage_cross_slide")

    ctx.check(
        "primary mechanisms are present",
        object_model.get_articulation("head_to_turret").articulation_type == ArticulationType.CONTINUOUS
        and object_model.get_articulation("coarse_focus_axis").articulation_type == ArticulationType.CONTINUOUS
        and object_model.get_articulation("fine_focus_axis").articulation_type == ArticulationType.CONTINUOUS
        and head_slide.articulation_type == ArticulationType.PRISMATIC
        and stage_slide.articulation_type == ArticulationType.PRISMATIC,
        details="head carriage, turret, stage carriage, and both focus knobs must articulate distinctly",
    )

    ctx.expect_overlap(
        head,
        base,
        axes="xy",
        elem_a="collar_0_lower",
        elem_b="guide_rod_0",
        min_overlap=0.004,
        name="head collar surrounds first visible guide rod",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="xy",
        elem_a="collar_1_lower",
        elem_b="guide_rod_1",
        min_overlap=0.004,
        name="head collar surrounds second visible guide rod",
    )
    ctx.expect_gap(
        stage_carriage,
        base,
        axis="z",
        positive_elem="cross_slide_plate",
        negative_elem="stage_front_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="stage carriage rests on the fixed slide stage",
    )
    ctx.expect_contact(
        coarse,
        base,
        elem_a="coarse_shaft",
        elem_b="coarse_bearing",
        contact_tol=0.0015,
        name="coarse focus shaft is seated in its own bearing",
    )
    ctx.expect_contact(
        fine,
        base,
        elem_a="fine_shaft",
        elem_b="fine_bearing",
        contact_tol=0.0015,
        name="fine focus shaft is seated in its own bearing",
    )
    ctx.expect_gap(
        head,
        turret,
        axis="z",
        positive_elem="nosepiece_socket",
        negative_elem="rotating_nosepiece_disk",
        max_gap=0.001,
        max_penetration=0.0,
        name="objective turret seats below the head socket",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({head_slide: 0.040}):
        raised_head_pos = ctx.part_world_position(head)
        ctx.expect_overlap(
            head,
            base,
            axes="xy",
            elem_a="collar_0_upper",
            elem_b="guide_rod_0",
            min_overlap=0.004,
            name="raised head remains captured on the visible guide",
        )

    rest_stage_pos = ctx.part_world_position(stage_carriage)
    with ctx.pose({stage_slide: 0.028}):
        shifted_stage_pos = ctx.part_world_position(stage_carriage)
        ctx.expect_overlap(
            stage_carriage,
            base,
            axes="xy",
            elem_a="cross_slide_plate",
            elem_b="stage_front_rail",
            min_overlap=0.020,
            name="stage carriage remains supported at full travel",
        )

    ctx.check(
        "head carriage raises on the vertical guide",
        rest_head_pos is not None and raised_head_pos is not None and raised_head_pos[2] > rest_head_pos[2] + 0.035,
        details=f"rest={rest_head_pos}, raised={raised_head_pos}",
    )
    ctx.check(
        "stage carriage slides across the stage",
        rest_stage_pos is not None
        and shifted_stage_pos is not None
        and shifted_stage_pos[1] > rest_stage_pos[1] + 0.025,
        details=f"rest={rest_stage_pos}, shifted={shifted_stage_pos}",
    )

    return ctx.report()


object_model = build_object_model()
