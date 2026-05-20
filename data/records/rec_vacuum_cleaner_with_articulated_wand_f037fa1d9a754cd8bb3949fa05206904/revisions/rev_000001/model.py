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


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    corner_radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_val, z_val + z_center)
        for y_val, z_val in rounded_rect_profile(width, height, corner_radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_vacuum_cleaner")

    body_blue = model.material("body_blue", rgba=(0.19, 0.43, 0.74, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.28, 1.0))
    silver = model.material("silver", rgba=(0.72, 0.74, 0.77, 1.0))
    clear_bin = model.material("clear_bin", rgba=(0.62, 0.80, 0.92, 0.35))

    body = model.part("body")

    shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(-0.200, width=0.220, height=0.165, corner_radius=0.030, z_center=0.145),
                _yz_section(-0.070, width=0.280, height=0.225, corner_radius=0.045, z_center=0.165),
                _yz_section(0.065, width=0.290, height=0.230, corner_radius=0.048, z_center=0.170),
                _yz_section(0.180, width=0.220, height=0.180, corner_radius=0.034, z_center=0.160),
            ]
        ),
        "vacuum_body_shell",
    )
    body.visual(shell_mesh, material=body_blue, name="main_shell")
    body.visual(
        Box((0.165, 0.160, 0.120)),
        origin=Origin(xyz=(-0.035, 0.0, 0.198)),
        material=clear_bin,
        name="dust_bin",
    )
    body.visual(
        Box((0.110, 0.220, 0.070)),
        origin=Origin(xyz=(0.082, 0.0, 0.142)),
        material=graphite,
        name="front_motor_housing",
    )
    body.visual(
        Box((0.042, 0.100, 0.086)),
        origin=Origin(xyz=(0.150, 0.0, 0.182)),
        material=charcoal,
        name="wand_mount_block",
    )
    body.visual(
        Box((0.042, 0.024, 0.086)),
        origin=Origin(xyz=(0.196, 0.050, 0.182)),
        material=graphite,
        name="left_mount_cheek",
    )
    body.visual(
        Box((0.042, 0.024, 0.086)),
        origin=Origin(xyz=(0.196, -0.050, 0.182)),
        material=graphite,
        name="right_mount_cheek",
    )
    body.visual(
        Cylinder(radius=0.072, length=0.040),
        origin=Origin(xyz=(-0.095, 0.145, 0.072), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_wheel",
    )
    body.visual(
        Cylinder(radius=0.072, length=0.040),
        origin=Origin(xyz=(-0.095, -0.145, 0.072), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_wheel",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.150),
        origin=Origin(xyz=(-0.185, 0.0, 0.032), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_axle_roller",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.155, 0.0, 0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="front_caster",
    )
    body.visual(
        Box((0.026, 0.060, 0.050)),
        origin=Origin(xyz=(0.155, 0.0, 0.051)),
        material=graphite,
        name="front_caster_fork",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.090),
        origin=Origin(xyz=(0.155, 0.0, 0.096)),
        material=graphite,
        name="front_caster_stem",
    )
    carry_handle = tube_from_spline_points(
        [
            (-0.135, 0.0, 0.238),
            (-0.095, 0.0, 0.298),
            (-0.025, 0.0, 0.333),
            (0.055, 0.0, 0.313),
            (0.115, 0.0, 0.265),
        ],
        radius=0.015,
        samples_per_segment=16,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    body.visual(
        mesh_from_geometry(carry_handle, "vacuum_carry_handle"),
        material=silver,
        name="carry_handle",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.420, 0.300, 0.330)),
        mass=8.6,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.020, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="rear_pivot_barrel",
    )
    upper_wand.visual(
        Box((0.070, 0.072, 0.088)),
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        material=graphite,
        name="rear_housing",
    )
    upper_wand.visual(
        Box((0.220, 0.078, 0.056)),
        origin=Origin(xyz=(0.175, 0.0, -0.006)),
        material=silver,
        name="beam",
    )
    upper_wand.visual(
        Box((0.086, 0.084, 0.034)),
        origin=Origin(xyz=(0.306, 0.0, 0.028)),
        material=graphite,
        name="front_housing",
    )
    upper_wand.visual(
        Box((0.050, 0.018, 0.074)),
        origin=Origin(xyz=(0.332, 0.045, -0.010)),
        material=charcoal,
        name="left_front_cheek",
    )
    upper_wand.visual(
        Box((0.050, 0.018, 0.074)),
        origin=Origin(xyz=(0.332, -0.045, -0.010)),
        material=charcoal,
        name="right_front_cheek",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.360, 0.110, 0.095)),
        mass=1.1,
        origin=Origin(xyz=(0.180, 0.0, -0.006)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.019, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="rear_pivot_barrel",
    )
    lower_wand.visual(
        Box((0.054, 0.068, 0.052)),
        origin=Origin(xyz=(0.051, 0.0, -0.032)),
        material=graphite,
        name="rear_housing",
    )
    lower_wand.visual(
        Box((0.034, 0.062, 0.034)),
        origin=Origin(xyz=(0.018, 0.0, -0.018)),
        material=graphite,
        name="pivot_bridge",
    )
    lower_wand.visual(
        Box((0.250, 0.074, 0.048)),
        origin=Origin(xyz=(0.190, 0.0, -0.020)),
        material=silver,
        name="beam",
    )
    lower_wand.visual(
        Box((0.066, 0.086, 0.040)),
        origin=Origin(xyz=(0.332, 0.0, 0.014)),
        material=graphite,
        name="front_housing",
    )
    lower_wand.visual(
        Box((0.050, 0.018, 0.070)),
        origin=Origin(xyz=(0.382, 0.046, -0.016)),
        material=charcoal,
        name="left_front_cheek",
    )
    lower_wand.visual(
        Box((0.050, 0.018, 0.070)),
        origin=Origin(xyz=(0.382, -0.046, -0.016)),
        material=charcoal,
        name="right_front_cheek",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.418, 0.110, 0.092)),
        mass=1.0,
        origin=Origin(xyz=(0.209, 0.0, -0.008)),
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        Cylinder(radius=0.0185, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )
    nozzle.visual(
        Box((0.032, 0.058, 0.032)),
        origin=Origin(xyz=(0.020, 0.0, -0.010)),
        material=graphite,
        name="hinge_bridge",
    )
    nozzle.visual(
        Box((0.060, 0.060, 0.064)),
        origin=Origin(xyz=(0.060, 0.0, -0.008)),
        material=graphite,
        name="neck_housing",
    )
    nozzle.visual(
        Box((0.092, 0.082, 0.038)),
        origin=Origin(xyz=(0.104, 0.0, -0.020)),
        material=graphite,
        name="transition_housing",
    )
    nozzle.visual(
        Box((0.295, 0.098, 0.024)),
        origin=Origin(xyz=(0.206, 0.0, -0.026)),
        material=charcoal,
        name="floor_head",
    )
    nozzle.visual(
        Box((0.250, 0.106, 0.012)),
        origin=Origin(xyz=(0.220, 0.0, -0.042)),
        material=body_blue,
        name="front_lip",
    )
    nozzle.visual(
        Box((0.120, 0.080, 0.014)),
        origin=Origin(xyz=(0.170, 0.0, -0.008)),
        material=graphite,
        name="top_cover",
    )
    nozzle.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.082, 0.055, -0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="left_roller",
    )
    nozzle.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.082, -0.055, -0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="right_roller",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.320, 0.110, 0.064)),
        mass=0.9,
        origin=Origin(xyz=(0.160, 0.0, -0.024)),
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_wand,
        origin=Origin(xyz=(0.206, 0.0, 0.182), rpy=(0.0, 0.28, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.35,
            upper=0.75,
        ),
    )
    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.332, 0.0, -0.010), rpy=(0.0, -0.12, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.4,
            lower=-0.50,
            upper=0.95,
        ),
    )
    model.articulation(
        "lower_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=nozzle,
        origin=Origin(xyz=(0.390, 0.0, -0.016), rpy=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=-0.25,
            upper=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    nozzle = object_model.get_part("nozzle")

    body_to_upper = object_model.get_articulation("body_to_upper_wand")
    upper_to_lower = object_model.get_articulation("upper_to_lower_wand")
    lower_to_nozzle = object_model.get_articulation("lower_to_nozzle")

    ctx.expect_origin_gap(
        nozzle,
        body,
        axis="x",
        min_gap=0.55,
        name="nozzle sits forward of the body",
    )
    ctx.expect_gap(
        body,
        nozzle,
        axis="z",
        positive_elem="front_motor_housing",
        negative_elem="floor_head",
        min_gap=0.11,
        name="body motor housing rides well above the floor nozzle",
    )
    ctx.expect_gap(
        upper_wand,
        nozzle,
        axis="z",
        positive_elem="beam",
        negative_elem="floor_head",
        min_gap=0.06,
        name="upper wand beam stays above the nozzle head",
    )
    ctx.expect_contact(
        body,
        upper_wand,
        elem_a="left_mount_cheek",
        elem_b="rear_pivot_barrel",
        name="upper wand barrel is captured by the body hinge cheek",
    )
    ctx.expect_contact(
        upper_wand,
        lower_wand,
        elem_a="left_front_cheek",
        elem_b="rear_pivot_barrel",
        name="lower wand barrel is captured by the upper wand elbow cheek",
    )
    ctx.expect_contact(
        lower_wand,
        nozzle,
        elem_a="left_front_cheek",
        elem_b="hinge_barrel",
        name="nozzle hinge barrel is captured by the lower wand cheek",
    )

    rest_nozzle_pos = ctx.part_world_position(nozzle)
    with ctx.pose({body_to_upper: 0.45, upper_to_lower: 0.55}):
        lifted_nozzle_pos = ctx.part_world_position(nozzle)
    ctx.check(
        "wand elbows lift the nozzle when flexed upward",
        rest_nozzle_pos is not None
        and lifted_nozzle_pos is not None
        and lifted_nozzle_pos[2] > rest_nozzle_pos[2] + 0.12,
        details=f"rest={rest_nozzle_pos}, lifted={lifted_nozzle_pos}",
    )

    rest_front_lip = ctx.part_element_world_aabb(nozzle, elem="front_lip")
    with ctx.pose({lower_to_nozzle: 0.40}):
        pitched_front_lip = ctx.part_element_world_aabb(nozzle, elem="front_lip")
    ctx.check(
        "nozzle hinge pitches the front lip upward",
        rest_front_lip is not None
        and pitched_front_lip is not None
        and pitched_front_lip[0][2] > rest_front_lip[0][2] + 0.03,
        details=f"rest={rest_front_lip}, pitched={pitched_front_lip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
