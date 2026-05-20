from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for z, y in rounded_rect_profile(height, width, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_vacuum_cleaner")

    body_red = model.material("body_red", rgba=(0.73, 0.12, 0.10, 1.0))
    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    wand_metal = model.material("wand_metal", rgba=(0.74, 0.77, 0.80, 1.0))
    nozzle_gray = model.material("nozzle_gray", rgba=(0.29, 0.31, 0.34, 1.0))
    clear_bin = model.material("clear_bin", rgba=(0.70, 0.80, 0.88, 0.35))

    main_body = model.part("main_body")
    main_body.inertial = Inertial.from_geometry(
        Box((0.34, 0.22, 0.66)),
        mass=8.0,
        origin=Origin(xyz=(0.02, 0.0, 0.33)),
    )
    main_body.visual(
        Box((0.34, 0.20, 0.055)),
        origin=Origin(xyz=(0.00, 0.0, 0.0275)),
        material=graphite,
        name="base_skid",
    )
    body_shell = section_loft(
        [
            _yz_section(-0.10, width=0.16, height=0.26, radius=0.040, z_center=0.18),
            _yz_section(-0.02, width=0.19, height=0.39, radius=0.050, z_center=0.25),
            _yz_section(0.03, width=0.17, height=0.26, radius=0.040, z_center=0.38),
            _yz_section(0.05, width=0.08, height=0.08, radius=0.020, z_center=0.46),
        ]
    )
    main_body.visual(
        mesh_from_geometry(body_shell, "vacuum_body_shell"),
        material=body_red,
        name="body_shell",
    )
    main_body.visual(
        Cylinder(radius=0.056, length=0.21),
        origin=Origin(xyz=(0.025, 0.0, 0.315)),
        material=clear_bin,
        name="dust_bin",
    )
    main_body.visual(
        Box((0.11, 0.12, 0.12)),
        origin=Origin(xyz=(0.06, 0.0, 0.17)),
        material=graphite,
        name="motor_nose",
    )
    main_body.visual(
        Box((0.050, 0.100, 0.090)),
        origin=Origin(xyz=(0.030, 0.0, 0.535)),
        material=graphite,
        name="hinge_tower",
    )
    main_body.visual(
        Box((0.036, 0.018, 0.074)),
        origin=Origin(xyz=(0.070, 0.050, 0.608)),
        material=graphite,
        name="upper_fork_left",
    )
    main_body.visual(
        Box((0.036, 0.018, 0.074)),
        origin=Origin(xyz=(0.070, -0.050, 0.608)),
        material=graphite,
        name="upper_fork_right",
    )
    main_body.visual(
        Cylinder(radius=0.015, length=0.24),
        origin=Origin(xyz=(-0.070, 0.0, 0.072), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="rear_axle",
    )
    main_body.visual(
        Cylinder(radius=0.070, length=0.026),
        origin=Origin(xyz=(-0.070, 0.108, 0.070), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="left_wheel",
    )
    main_body.visual(
        Cylinder(radius=0.070, length=0.026),
        origin=Origin(xyz=(-0.070, -0.108, 0.070), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="right_wheel",
    )

    upper_wand = model.part("upper_wand")
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.27, 0.08, 0.22)),
        mass=1.1,
        origin=Origin(xyz=(0.12, 0.0, -0.08)),
    )
    upper_wand.visual(
        Cylinder(radius=0.020, length=0.082),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=wand_metal,
        name="proximal_barrel",
    )
    upper_wand.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.0),
                    (0.050, 0.0, -0.018),
                    (0.145, 0.0, -0.090),
                    (0.195, 0.0, -0.132),
                ],
                radius=0.018,
                samples_per_segment=16,
                radial_segments=18,
            ),
            "vacuum_upper_wand_tube",
        ),
        material=wand_metal,
        name="upper_tube",
    )
    upper_wand.visual(
        Box((0.036, 0.064, 0.032)),
        origin=Origin(xyz=(0.198, 0.0, -0.139)),
        material=graphite,
        name="distal_support_block",
    )
    upper_wand.visual(
        Box((0.038, 0.012, 0.050)),
        origin=Origin(xyz=(0.223, 0.031, -0.155)),
        material=graphite,
        name="distal_fork_left",
    )
    upper_wand.visual(
        Box((0.038, 0.012, 0.050)),
        origin=Origin(xyz=(0.223, -0.031, -0.155)),
        material=graphite,
        name="distal_fork_right",
    )

    lower_wand = model.part("lower_wand")
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.33, 0.07, 0.43)),
        mass=1.0,
        origin=Origin(xyz=(0.15, 0.0, -0.18)),
    )
    lower_wand.visual(
        Cylinder(radius=0.019, length=0.050),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=wand_metal,
        name="proximal_barrel",
    )
    lower_wand.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.0),
                    (0.055, 0.0, -0.028),
                    (0.165, 0.0, -0.165),
                    (0.238, 0.0, -0.328),
                ],
                radius=0.017,
                samples_per_segment=18,
                radial_segments=18,
            ),
            "vacuum_lower_wand_tube",
        ),
        material=wand_metal,
        name="lower_tube",
    )
    lower_wand.visual(
        Box((0.036, 0.060, 0.036)),
        origin=Origin(xyz=(0.250, 0.0, -0.346)),
        material=graphite,
        name="nozzle_support_block",
    )
    lower_wand.visual(
        Box((0.036, 0.012, 0.046)),
        origin=Origin(xyz=(0.270, 0.031, -0.366)),
        material=graphite,
        name="nozzle_fork_left",
    )
    lower_wand.visual(
        Box((0.036, 0.012, 0.046)),
        origin=Origin(xyz=(0.270, -0.031, -0.366)),
        material=graphite,
        name="nozzle_fork_right",
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.19, 0.31, 0.05)),
        mass=1.8,
        origin=Origin(xyz=(0.09, 0.0, -0.01)),
    )
    floor_nozzle.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    floor_nozzle.visual(
        Box((0.040, 0.065, 0.040)),
        origin=Origin(xyz=(0.015, 0.0, -0.018)),
        material=nozzle_gray,
        name="neck_block",
    )
    floor_nozzle.visual(
        Box((0.180, 0.300, 0.030)),
        origin=Origin(xyz=(0.095, 0.0, -0.015)),
        material=nozzle_gray,
        name="nozzle_body",
    )
    floor_nozzle.visual(
        Box((0.070, 0.105, 0.028)),
        origin=Origin(xyz=(0.035, 0.0, -0.002)),
        material=graphite,
        name="top_pivot_cover",
    )
    floor_nozzle.visual(
        Box((0.025, 0.260, 0.010)),
        origin=Origin(xyz=(0.175, 0.0, -0.025)),
        material=dark_rubber,
        name="front_lip",
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=upper_wand,
        origin=Origin(xyz=(0.070, 0.0, 0.608)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.90,
            upper=0.55,
        ),
    )
    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.240, 0.0, -0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=2.0,
            lower=-0.85,
            upper=0.80,
        ),
    )
    model.articulation(
        "lower_wand_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.300, 0.0, -0.400)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-0.30,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("main_body")
    upper_wand = object_model.get_part("upper_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    elbow_1 = object_model.get_articulation("body_to_upper_wand")
    elbow_2 = object_model.get_articulation("upper_to_lower_wand")
    nozzle_hinge = object_model.get_articulation("lower_wand_to_nozzle")

    ctx.expect_origin_gap(
        upper_wand,
        body,
        axis="z",
        min_gap=0.50,
        name="first wand joint sits well above the ground line",
    )
    ctx.expect_origin_gap(
        floor_nozzle,
        body,
        axis="x",
        min_gap=0.55,
        name="wand chain reaches forward to a floor nozzle",
    )

    nozzle_aabb = ctx.part_world_aabb(floor_nozzle)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "nozzle rests on the floor plane",
        nozzle_aabb is not None and abs(nozzle_aabb[0][2]) <= 0.01,
        details=f"floor_nozzle_aabb={nozzle_aabb}",
    )
    ctx.check(
        "body also starts on the floor plane",
        body_aabb is not None and abs(body_aabb[0][2]) <= 0.01,
        details=f"main_body_aabb={body_aabb}",
    )

    rest_nozzle_pos = ctx.part_world_position(floor_nozzle)
    rest_front_lip = ctx.part_element_world_aabb(floor_nozzle, elem="front_lip")
    with ctx.pose({elbow_1: 0.45, elbow_2: 0.55}):
        raised_nozzle_pos = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "wand elbows can raise the nozzle",
        rest_nozzle_pos is not None
        and raised_nozzle_pos is not None
        and raised_nozzle_pos[2] > rest_nozzle_pos[2] + 0.18,
        details=f"rest={rest_nozzle_pos}, raised={raised_nozzle_pos}",
    )

    with ctx.pose({nozzle_hinge: 0.40}):
        pitched_front_lip = ctx.part_element_world_aabb(floor_nozzle, elem="front_lip")
    ctx.check(
        "floor nozzle pitches upward on its hinge",
        rest_front_lip is not None
        and pitched_front_lip is not None
        and pitched_front_lip[0][2] > rest_front_lip[0][2] + 0.03,
        details=f"rest_front_lip={rest_front_lip}, pitched_front_lip={pitched_front_lip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
