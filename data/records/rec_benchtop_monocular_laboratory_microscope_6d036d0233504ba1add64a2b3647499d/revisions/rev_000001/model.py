from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
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


def _extruded_yz_profile(
    profile_yz: list[tuple[float, float]],
    width_x: float,
) -> MeshGeometry:
    """Extrude a side-profile in local Y/Z into a centered X-width solid."""
    geom = MeshGeometry()
    half_width = width_x * 0.5
    rear_ids = [geom.add_vertex(-half_width, y, z) for y, z in profile_yz]
    front_ids = [geom.add_vertex(half_width, y, z) for y, z in profile_yz]
    count = len(profile_yz)

    for index in range(count):
        nxt = (index + 1) % count
        geom.add_face(rear_ids[index], rear_ids[nxt], front_ids[nxt])
        geom.add_face(rear_ids[index], front_ids[nxt], front_ids[index])

    for index in range(1, count - 1):
        geom.add_face(rear_ids[0], rear_ids[index], rear_ids[index + 1])
        geom.add_face(front_ids[0], front_ids[index + 1], front_ids[index])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_monocular_microscope")

    enamel = model.material("warm_enamel", rgba=(0.82, 0.80, 0.72, 1.0))
    dark_enamel = model.material("dark_stage_finish", rgba=(0.045, 0.047, 0.050, 1.0))
    black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    metal = model.material("brushed_metal", rgba=(0.70, 0.72, 0.72, 1.0))
    glass = model.material("pale_slide_glass", rgba=(0.78, 0.92, 0.96, 0.42))
    rubber = model.material("black_rubber", rgba=(0.020, 0.020, 0.022, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.245, 0.305, 0.034, corner_segments=10),
                0.055,
            ),
            "heavy_rounded_base",
        ),
        material=enamel,
        name="heavy_base",
    )
    base_frame.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.165, 0.080, 0.014, corner_segments=8),
                0.014,
            ),
            "stage_pedestal_cap",
        ),
        origin=Origin(xyz=(0.000, -0.030, 0.055)),
        material=enamel,
        name="stage_pedestal",
    )
    base_frame.visual(
        Box((0.180, 0.120, 0.016)),
        origin=Origin(xyz=(0.000, -0.030, 0.126)),
        material=dark_enamel,
        name="stage_support",
    )
    base_frame.visual(
        Box((0.110, 0.060, 0.072)),
        origin=Origin(xyz=(0.000, -0.030, 0.091)),
        material=enamel,
        name="stage_riser",
    )
    base_frame.visual(
        Box((0.024, 0.044, 0.036)),
        origin=Origin(xyz=(0.102, -0.030, 0.132)),
        material=dark_enamel,
        name="stage_gearbox",
    )
    base_frame.visual(
        Cylinder(radius=0.0042, length=0.095),
        origin=Origin(xyz=(0.148, -0.030, 0.132), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="coaxial_control_shaft",
    )
    for collar_index, collar_x in enumerate((0.118, 0.149, 0.184)):
        base_frame.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(collar_x, -0.030, 0.132), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"control_collar_{collar_index}",
        )
    base_frame.visual(
        Cylinder(radius=0.018, length=0.350),
        origin=Origin(xyz=(0.000, 0.097, 0.230)),
        material=metal,
        name="vertical_column",
    )
    base_frame.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(xyz=(0.000, 0.097, 0.063)),
        material=enamel,
        name="column_foot",
    )

    stage_carriage = model.part("stage_carriage")
    stage_carriage.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.162, 0.112, 0.006, corner_segments=6),
                [_circle_profile(0.020, segments=44)],
                0.012,
            ),
            "rectangular_stage_with_aperture",
        ),
        material=dark_enamel,
        name="stage_plate",
    )
    stage_carriage.visual(
        Box((0.094, 0.031, 0.002)),
        origin=Origin(xyz=(0.000, -0.004, 0.007)),
        material=glass,
        name="glass_slide",
    )
    stage_carriage.visual(
        Box((0.044, 0.006, 0.003)),
        origin=Origin(xyz=(-0.040, 0.018, 0.010)),
        material=metal,
        name="slide_clip_0",
    )
    stage_carriage.visual(
        Box((0.044, 0.006, 0.003)),
        origin=Origin(xyz=(0.040, 0.018, 0.010)),
        material=metal,
        name="slide_clip_1",
    )
    stage_carriage.visual(
        Box((0.010, 0.020, 0.005)),
        origin=Origin(xyz=(-0.061, 0.018, 0.0085)),
        material=metal,
        name="clip_anchor_0",
    )
    stage_carriage.visual(
        Box((0.010, 0.020, 0.005)),
        origin=Origin(xyz=(0.061, 0.018, 0.0085)),
        material=metal,
        name="clip_anchor_1",
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=stage_carriage,
        origin=Origin(xyz=(0.000, -0.030, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.055, lower=-0.035, upper=0.035),
    )

    optical_body = model.part("optical_body")
    optical_body.visual(
        Box((0.012, 0.060, 0.084)),
        origin=Origin(xyz=(-0.024, 0.000, 0.000)),
        material=enamel,
        name="carriage_cheek_0",
    )
    optical_body.visual(
        Box((0.012, 0.060, 0.084)),
        origin=Origin(xyz=(0.024, 0.000, 0.000)),
        material=enamel,
        name="carriage_cheek_1",
    )
    optical_body.visual(
        Box((0.072, 0.012, 0.084)),
        origin=Origin(xyz=(0.000, -0.024, 0.000)),
        material=enamel,
        name="front_carriage_bridge",
    )
    optical_body.visual(
        Box((0.072, 0.012, 0.084)),
        origin=Origin(xyz=(0.000, 0.024, 0.000)),
        material=enamel,
        name="rear_carriage_bridge",
    )
    optical_body.visual(
        Box((0.052, 0.020, 0.050)),
        origin=Origin(xyz=(0.000, -0.040, 0.000)),
        material=enamel,
        name="arm_root_web",
    )
    optical_body.visual(
        mesh_from_geometry(
            _extruded_yz_profile(
                [
                    (-0.036, -0.030),
                    (-0.030, 0.045),
                    (-0.055, 0.074),
                    (-0.105, 0.072),
                    (-0.156, 0.044),
                    (-0.166, 0.004),
                    (-0.146, -0.026),
                    (-0.102, -0.008),
                    (-0.064, 0.013),
                ],
                0.052,
            ),
            "swept_microscope_arm",
        ),
        material=enamel,
        name="swept_arm",
    )
    optical_body.visual(
        Cylinder(radius=0.026, length=0.125),
        origin=Origin(xyz=(0.000, -0.145, 0.006)),
        material=black,
        name="optical_tube",
    )
    optical_body.visual(
        Cylinder(radius=0.020, length=0.038),
        origin=Origin(xyz=(0.000, -0.145, 0.076)),
        material=enamel,
        name="head_socket",
    )
    optical_body.visual(
        Cylinder(radius=0.019, length=0.088),
        origin=Origin(xyz=(0.000, -0.113, 0.108), rpy=(-math.pi / 4.0, 0.0, 0.0)),
        material=black,
        name="monocular_head",
    )
    optical_body.visual(
        Cylinder(radius=0.013, length=0.044),
        origin=Origin(xyz=(0.000, -0.071, 0.150), rpy=(-math.pi / 4.0, 0.0, 0.0)),
        material=rubber,
        name="eyepiece",
    )
    optical_body.visual(
        Box((0.060, 0.012, 0.070)),
        origin=Origin(xyz=(0.000, 0.036, 0.000)),
        material=metal,
        name="rack_strip",
    )
    model.articulation(
        "focus_slide",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=optical_body,
        origin=Origin(xyz=(0.000, 0.097, 0.265)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.035, lower=0.0, upper=0.065),
    )

    turret = model.part("objective_turret")
    turret.visual(
        Cylinder(radius=0.034, length=0.016),
        origin=Origin(),
        material=metal,
        name="turret_disk",
    )
    objective_names = ("objective_0", "objective_1", "objective_2")
    objective_collar_names = ("objective_collar_0", "objective_collar_1", "objective_collar_2")
    for objective_index, angle in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        radius = 0.017
        turret.visual(
            Cylinder(radius=0.0065, length=0.034),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), -0.025),
            ),
            material=black,
            name=objective_names[objective_index],
        )
        turret.visual(
            Cylinder(radius=0.0090, length=0.006),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), -0.011),
            ),
            material=metal,
            name=objective_collar_names[objective_index],
        )
    model.articulation(
        "turret_rotate",
        ArticulationType.CONTINUOUS,
        parent=optical_body,
        child=turret,
        origin=Origin(xyz=(0.000, -0.145, -0.0645)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.8),
    )

    control_mesh = mesh_from_geometry(
        KnobGeometry(
            0.028,
            0.024,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="fluted", count=28, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
            bore=KnobBore(style="round", diameter=0.010),
        ),
        "stage_axis_knob",
    )
    second_control_mesh = mesh_from_geometry(
        KnobGeometry(
            0.036,
            0.030,
            body_style="hourglass",
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=24, depth=0.0010),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            bore=KnobBore(style="round", diameter=0.010),
        ),
        "second_stage_axis_knob",
    )
    stage_control = model.part("stage_control")
    stage_control.visual(control_mesh, material=black, name="control_knob")
    model.articulation(
        "stage_control_rotate",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=stage_control,
        origin=Origin(xyz=(0.132, -0.030, 0.132), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.22, velocity=5.0),
    )

    second_stage_control = model.part("second_stage_control")
    second_stage_control.visual(second_control_mesh, material=black, name="second_control_knob")
    model.articulation(
        "second_stage_control_rotate",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=second_stage_control,
        origin=Origin(xyz=(0.169, -0.030, 0.132), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.18, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    stage = object_model.get_part("stage_carriage")
    optics = object_model.get_part("optical_body")
    turret = object_model.get_part("objective_turret")
    focus_slide = object_model.get_articulation("focus_slide")
    stage_slide = object_model.get_articulation("stage_slide")
    stage_control = object_model.get_part("stage_control")
    second_control = object_model.get_part("second_stage_control")

    ctx.expect_gap(
        stage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="stage_plate",
        negative_elem="stage_support",
        name="stage carriage sits on the fixed support",
    )
    ctx.expect_gap(
        turret,
        stage,
        axis="z",
        min_gap=0.006,
        positive_elem="objective_0",
        negative_elem="glass_slide",
        name="objective clears the specimen slide",
    )
    ctx.expect_gap(
        optics,
        stage,
        axis="z",
        min_gap=0.020,
        positive_elem="optical_tube",
        negative_elem="stage_plate",
        name="optical body remains separate from the stage assembly",
    )

    rest_focus = ctx.part_world_position(optics)
    with ctx.pose({focus_slide: 0.055}):
        raised_focus = ctx.part_world_position(optics)
    ctx.check(
        "optical carriage slides upward on the column",
        rest_focus is not None and raised_focus is not None and raised_focus[2] > rest_focus[2] + 0.050,
        details=f"rest={rest_focus}, raised={raised_focus}",
    )

    rest_stage = ctx.part_world_position(stage)
    with ctx.pose({stage_slide: 0.030}):
        shifted_stage = ctx.part_world_position(stage)
    ctx.check(
        "stage carriage slides left to right",
        rest_stage is not None and shifted_stage is not None and shifted_stage[0] > rest_stage[0] + 0.025,
        details=f"rest={rest_stage}, shifted={shifted_stage}",
    )

    first_pos = ctx.part_world_position(stage_control)
    second_pos = ctx.part_world_position(second_control)
    coaxial = (
        first_pos is not None
        and second_pos is not None
        and abs(first_pos[1] - second_pos[1]) < 1e-6
        and abs(first_pos[2] - second_pos[2]) < 1e-6
        and 0.030 <= abs(second_pos[0] - first_pos[0]) <= 0.045
    )
    ctx.check(
        "stage controls are distinct coaxial side knobs",
        coaxial,
        details=f"first={first_pos}, second={second_pos}",
    )

    return ctx.report()


object_model = build_object_model()
