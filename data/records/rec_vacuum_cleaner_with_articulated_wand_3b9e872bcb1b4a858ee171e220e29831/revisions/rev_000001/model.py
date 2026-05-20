from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x_pos: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z + z_center) for z, y in rounded_rect_profile(height, width, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_vacuum_cleaner")

    body_red = model.material("body_red", rgba=(0.70, 0.09, 0.08, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.71, 0.74, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    main_body = model.part("main_body")
    body_shell = section_loft(
        [
            _yz_section(0.22, 0.18, 0.045, -0.20, z_center=0.15),
            _yz_section(0.31, 0.25, 0.060, -0.04, z_center=0.17),
            _yz_section(0.35, 0.30, 0.070, 0.12, z_center=0.18),
            _yz_section(0.25, 0.23, 0.055, 0.28, z_center=0.18),
        ]
    )
    main_body.visual(_mesh("vacuum_body_shell", body_shell), material=body_red, name="body_shell")
    main_body.visual(
        Box((0.22, 0.18, 0.12)),
        origin=Origin(xyz=(0.02, 0.0, 0.16)),
        material=charcoal,
        name="dust_bin",
    )
    main_body.visual(
        Cylinder(radius=0.024, length=0.31),
        origin=Origin(xyz=(-0.08, 0.0, 0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="rear_axle",
    )
    main_body.visual(
        Cylinder(radius=0.11, length=0.045),
        origin=Origin(xyz=(-0.08, 0.18, 0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_wheel",
    )
    main_body.visual(
        Cylinder(radius=0.11, length=0.045),
        origin=Origin(xyz=(-0.08, -0.18, 0.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_wheel",
    )
    main_body.visual(
        Box((0.12, 0.18, 0.03)),
        origin=Origin(xyz=(0.18, 0.0, 0.025)),
        material=dark_plastic,
        name="front_glide",
    )
    main_body.visual(
        Cylinder(radius=0.032, length=0.07),
        origin=Origin(xyz=(0.285, 0.0, 0.26), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="body_socket",
    )
    main_body.visual(
        Box((0.09, 0.09, 0.07)),
        origin=Origin(xyz=(0.22, 0.0, 0.23)),
        material=charcoal,
        name="socket_mount",
    )
    main_body.visual(
        _mesh(
            "vacuum_handle_arch",
            tube_from_spline_points(
                [
                    (-0.03, 0.0, 0.25),
                    (0.02, 0.0, 0.31),
                    (0.10, 0.0, 0.35),
                    (0.19, 0.0, 0.31),
                    (0.22, 0.0, 0.25),
                ],
                radius=0.018,
                samples_per_segment=16,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=dark_plastic,
        name="carry_handle",
    )
    main_body.inertial = Inertial.from_geometry(
        Box((0.60, 0.42, 0.36)),
        mass=7.2,
        origin=Origin(xyz=(0.02, 0.0, 0.18)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.029, length=0.09),
        origin=Origin(xyz=(0.034, 0.0, 0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="upper_prox_housing",
    )
    upper_wand.visual(
        _mesh(
            "vacuum_upper_tube",
            tube_from_spline_points(
                [
                    (0.05, 0.0, 0.03),
                    (0.13, 0.0, 0.08),
                    (0.21, 0.0, 0.135),
                    (0.279, 0.0, 0.195),
                ],
                radius=0.021,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=steel,
        name="upper_tube",
    )
    upper_wand.visual(
        Cylinder(radius=0.026, length=0.10),
        origin=Origin(xyz=(0.242, 0.0, 0.162), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="upper_distal_housing",
    )
    upper_wand.visual(
        _mesh(
            "vacuum_upper_grip",
            tube_from_spline_points(
                [
                    (0.11, 0.0, 0.067),
                    (0.15, 0.0, 0.095),
                    (0.19, 0.0, 0.123),
                ],
                radius=0.028,
                samples_per_segment=10,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=dark_plastic,
        name="upper_grip",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.34, 0.10, 0.24)),
        mass=0.9,
        origin=Origin(xyz=(0.17, 0.0, 0.11)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.030, length=0.095),
        origin=Origin(xyz=(0.042, 0.0, -0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="lower_prox_housing",
    )
    lower_wand.visual(
        _mesh(
            "vacuum_lower_tube",
            tube_from_spline_points(
                [
                    (0.05, 0.0, -0.02),
                    (0.12, 0.0, -0.12),
                    (0.18, 0.0, -0.24),
                    (0.208, 0.0, -0.350),
                ],
                radius=0.020,
                samples_per_segment=16,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=steel,
        name="lower_tube",
    )
    lower_wand.visual(
        Box((0.050, 0.045, 0.070)),
        origin=Origin(xyz=(0.191, 0.0, -0.339)),
        material=charcoal,
        name="lower_joint_neck",
    )
    lower_wand.visual(
        Cylinder(radius=0.019, length=0.046),
        origin=Origin(xyz=(0.215, 0.0, -0.372), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="lower_distal_housing",
    )
    lower_wand.visual(
        Box((0.05, 0.05, 0.11)),
        origin=Origin(xyz=(0.15, 0.0, -0.18), rpy=(0.26, 0.0, 0.0)),
        material=dark_plastic,
        name="cord_duct",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.28, 0.10, 0.44)),
        mass=1.0,
        origin=Origin(xyz=(0.13, 0.0, -0.19)),
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Cylinder(radius=0.022, length=0.032),
        origin=Origin(xyz=(0.0, 0.039, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="nozzle_left_hinge_lug",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.022, length=0.032),
        origin=Origin(xyz=(0.0, -0.039, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="nozzle_right_hinge_lug",
    )
    floor_nozzle.visual(
        Box((0.040, 0.094, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, -0.030)),
        material=charcoal,
        name="nozzle_hinge_bridge",
    )
    floor_nozzle.visual(
        Box((0.078, 0.034, 0.060)),
        origin=Origin(xyz=(0.060, 0.0, -0.056)),
        material=charcoal,
        name="nozzle_neck",
    )
    floor_nozzle.visual(
        Box((0.28, 0.30, 0.012)),
        origin=Origin(xyz=(0.14, 0.0, -0.074)),
        material=dark_plastic,
        name="nozzle_base",
    )
    floor_nozzle.visual(
        Box((0.16, 0.29, 0.035)),
        origin=Origin(xyz=(0.13, 0.0, -0.048)),
        material=body_red,
        name="nozzle_cover",
    )
    floor_nozzle.visual(
        Box((0.10, 0.26, 0.02)),
        origin=Origin(xyz=(0.24, 0.0, -0.057), rpy=(0.0, -0.18, 0.0)),
        material=body_red,
        name="nozzle_front_taper",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.018, length=0.28),
        origin=Origin(xyz=(0.26, 0.0, -0.058), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="front_bumper",
    )
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.30, 0.32, 0.09)),
        mass=1.4,
        origin=Origin(xyz=(0.14, 0.0, -0.045)),
    )

    model.articulation(
        "body_to_upper_elbow",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=upper_wand,
        origin=Origin(xyz=(0.315, 0.0, 0.26)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.6,
            lower=-0.55,
            upper=0.85,
        ),
    )
    model.articulation(
        "upper_to_lower_elbow",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.279, 0.0, 0.195)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-0.95,
            upper=0.45,
        ),
    )
    model.articulation(
        "wand_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.215, 0.0, -0.372)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")

    body_elbow = object_model.get_articulation("body_to_upper_elbow")
    lower_elbow = object_model.get_articulation("upper_to_lower_elbow")
    nozzle_hinge = object_model.get_articulation("wand_to_floor_nozzle")

    ctx.expect_contact(
        lower_wand,
        floor_nozzle,
        elem_a="lower_distal_housing",
        elem_b="nozzle_left_hinge_lug",
        name="left hinge lug rides on the lower wand barrel",
    )
    ctx.expect_contact(
        lower_wand,
        floor_nozzle,
        elem_a="lower_distal_housing",
        elem_b="nozzle_right_hinge_lug",
        name="right hinge lug rides on the lower wand barrel",
    )

    rest_upper = ctx.part_world_position(upper_wand)
    rest_lower = ctx.part_world_position(lower_wand)
    rest_nozzle = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "wand joints read as spaced-apart elbows in profile",
        rest_upper is not None
        and rest_lower is not None
        and rest_nozzle is not None
        and rest_lower[0] > rest_upper[0] + 0.20
        and rest_nozzle[0] > rest_lower[0] + 0.16,
        details=f"upper={rest_upper}, lower={rest_lower}, nozzle={rest_nozzle}",
    )

    with ctx.pose({body_elbow: 0.60}):
        lifted_lower = ctx.part_world_position(lower_wand)
    ctx.check(
        "body elbow lifts the wand chain",
        rest_lower is not None
        and lifted_lower is not None
        and lifted_lower[2] > rest_lower[2] + 0.08,
        details=f"rest_lower={rest_lower}, lifted_lower={lifted_lower}",
    )

    with ctx.pose({lower_elbow: -0.60}):
        folded_nozzle = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "second elbow folds the distal wand toward the body",
        rest_nozzle is not None
        and folded_nozzle is not None
        and folded_nozzle[0] < rest_nozzle[0] - 0.12
        and folded_nozzle[2] < rest_nozzle[2] - 0.03,
        details=f"rest_nozzle={rest_nozzle}, folded_nozzle={folded_nozzle}",
    )

    rest_bumper = ctx.part_element_world_aabb(floor_nozzle, elem="front_bumper")
    with ctx.pose({nozzle_hinge: 0.35}):
        pitched_bumper = ctx.part_element_world_aabb(floor_nozzle, elem="front_bumper")
    ctx.check(
        "floor nozzle pitches upward around its hinge",
        rest_bumper is not None
        and pitched_bumper is not None
        and pitched_bumper[1][2] > rest_bumper[1][2] + 0.03,
        details=f"rest_bumper={rest_bumper}, pitched_bumper={pitched_bumper}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
