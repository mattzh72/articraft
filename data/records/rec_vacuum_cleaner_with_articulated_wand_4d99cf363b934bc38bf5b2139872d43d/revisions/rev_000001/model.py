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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_vacuum_cleaner")

    body_shell_color = model.material("body_shell_color", rgba=(0.77, 0.15, 0.11, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.17, 1.0))
    wand_metal = model.material("wand_metal", rgba=(0.67, 0.70, 0.74, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    bumper_gray = model.material("bumper_gray", rgba=(0.36, 0.38, 0.40, 1.0))
    guard_gray = model.material("guard_gray", rgba=(0.55, 0.58, 0.61, 1.0))
    filter_blue = model.material("filter_blue", rgba=(0.21, 0.53, 0.75, 1.0))

    def yz_section(x: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
        return [(x, y, z) for z, y in rounded_rect_profile(height, width, radius, corner_segments=8)]

    body_shell_geom = section_loft(
        [
            yz_section(-0.19, 0.16, 0.12, 0.03),
            yz_section(-0.06, 0.24, 0.18, 0.05),
            yz_section(0.09, 0.26, 0.19, 0.05),
            yz_section(0.16, 0.18, 0.15, 0.04),
        ]
    )
    body_shell_mesh = mesh_from_geometry(body_shell_geom, "vacuum_body_shell")

    nozzle_shell_geom = section_loft(
        [
            yz_section(0.00, 0.072, 0.045, 0.011),
            yz_section(0.10, 0.098, 0.030, 0.008),
            yz_section(0.21, 0.102, 0.022, 0.006),
            yz_section(0.29, 0.086, 0.016, 0.004),
        ]
    )
    nozzle_shell_mesh = mesh_from_geometry(nozzle_shell_geom, "vacuum_floor_nozzle_shell")

    guard_hoop_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.228, -0.108, 0.206),
                (0.245, -0.112, 0.246),
                (0.286, -0.094, 0.315),
                (0.333, 0.000, 0.360),
                (0.286, 0.094, 0.315),
                (0.245, 0.112, 0.246),
                (0.228, 0.108, 0.206),
            ],
            radius=0.010,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
        "vacuum_guard_hoop",
    )

    body = model.part("body")
    body.visual(
        body_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=body_shell_color,
        name="main_shell",
    )
    body.visual(
        Box((0.12, 0.16, 0.06)),
        origin=Origin(xyz=(-0.11, 0.0, 0.07)),
        material=charcoal,
        name="rear_motor_housing",
    )
    body.visual(
        Box((0.13, 0.11, 0.05)),
        origin=Origin(xyz=(0.07, 0.0, 0.12)),
        material=filter_blue,
        name="top_filter_pack",
    )
    body.visual(
        Cylinder(radius=0.058, length=0.050),
        origin=Origin(xyz=(-0.07, 0.122, 0.058), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="right_wheel",
    )
    body.visual(
        Cylinder(radius=0.058, length=0.050),
        origin=Origin(xyz=(-0.07, -0.122, 0.058), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="left_wheel",
    )
    body.visual(
        Box((0.090, 0.180, 0.030)),
        origin=Origin(xyz=(0.157, 0.0, 0.015)),
        material=bumper_gray,
        name="front_skid",
    )
    body.visual(
        Box((0.10, 0.07, 0.12)),
        origin=Origin(xyz=(0.135, -0.090, 0.160)),
        material=charcoal,
        name="left_shoulder_pod",
    )
    body.visual(
        Box((0.10, 0.07, 0.12)),
        origin=Origin(xyz=(0.135, 0.090, 0.160)),
        material=charcoal,
        name="right_shoulder_pod",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.18),
        origin=Origin(xyz=(-0.04, 0.0, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bumper_gray,
        name="carry_handle",
    )
    body.visual(
        Box((0.042, 0.072, 0.052)),
        origin=Origin(xyz=(0.148, 0.0, 0.177)),
        material=charcoal,
        name="wand_mount_block",
    )
    body.visual(
        Box((0.050, 0.018, 0.052)),
        origin=Origin(xyz=(0.169, -0.024, 0.180)),
        material=charcoal,
        name="wand_mount_left_yoke",
    )
    body.visual(
        Box((0.050, 0.018, 0.052)),
        origin=Origin(xyz=(0.169, 0.024, 0.180)),
        material=charcoal,
        name="wand_mount_right_yoke",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.42, 0.28, 0.23)),
        mass=5.6,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
    )

    guard_frame = model.part("guard_frame")
    guard_frame.visual(guard_hoop_mesh, material=guard_gray, name="guard_hoop")
    guard_frame.visual(
        Cylinder(radius=0.007, length=0.060),
        origin=Origin(xyz=(0.215, -0.103, 0.206), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_gray,
        name="guard_left_stay",
    )
    guard_frame.visual(
        Cylinder(radius=0.007, length=0.060),
        origin=Origin(xyz=(0.215, 0.103, 0.206), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_gray,
        name="guard_right_stay",
    )
    guard_frame.inertial = Inertial.from_geometry(
        Box((0.25, 0.24, 0.28)),
        mass=0.55,
        origin=Origin(xyz=(0.19, 0.0, 0.23)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="lower_joint_barrel",
    )
    lower_wand.visual(
        Box((0.050, 0.028, 0.030)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=charcoal,
        name="lower_joint_web",
    )
    lower_wand.visual(
        Cylinder(radius=0.0175, length=0.180),
        origin=Origin(xyz=(0.130, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wand_metal,
        name="lower_tube",
    )
    lower_wand.visual(
        Box((0.026, 0.030, 0.030)),
        origin=Origin(xyz=(0.204, 0.0, 0.0), rpy=(0.0, 0.88, 0.0)),
        material=charcoal,
        name="lower_tip_bridge",
    )
    lower_wand.visual(
        Box((0.024, 0.010, 0.034)),
        origin=Origin(xyz=(0.232, -0.020, 0.0), rpy=(0.0, 0.88, 0.0)),
        material=charcoal,
        name="lower_elbow_left_cheek",
    )
    lower_wand.visual(
        Box((0.024, 0.010, 0.034)),
        origin=Origin(xyz=(0.232, 0.020, 0.0), rpy=(0.0, 0.88, 0.0)),
        material=charcoal,
        name="lower_elbow_right_cheek",
    )
    lower_wand.visual(
        Box((0.118, 0.022, 0.028)),
        origin=Origin(xyz=(0.104, 0.0, -0.007)),
        material=bumper_gray,
        name="lower_spine",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.24, 0.12, 0.07)),
        mass=0.75,
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="upper_base_barrel",
    )
    upper_wand.visual(
        Box((0.046, 0.026, 0.028)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=charcoal,
        name="upper_base_web",
    )
    upper_wand.visual(
        Cylinder(radius=0.015, length=0.340),
        origin=Origin(xyz=(0.182, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wand_metal,
        name="upper_tube",
    )
    upper_wand.visual(
        Box((0.120, 0.022, 0.026)),
        origin=Origin(xyz=(0.148, 0.0, -0.010)),
        material=bumper_gray,
        name="upper_cable_channel",
    )
    upper_wand.visual(
        Box((0.030, 0.030, 0.032)),
        origin=Origin(xyz=(0.370, 0.0, 0.0), rpy=(0.0, -0.45, 0.0)),
        material=charcoal,
        name="upper_head_bridge",
    )
    upper_wand.visual(
        Box((0.022, 0.010, 0.030)),
        origin=Origin(xyz=(0.388, -0.020, 0.0), rpy=(0.0, -0.45, 0.0)),
        material=charcoal,
        name="upper_head_left_cheek",
    )
    upper_wand.visual(
        Box((0.022, 0.010, 0.030)),
        origin=Origin(xyz=(0.388, 0.020, 0.0), rpy=(0.0, -0.45, 0.0)),
        material=charcoal,
        name="upper_head_right_cheek",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.47, 0.10, 0.06)),
        mass=0.92,
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        nozzle_shell_mesh,
        origin=Origin(xyz=(0.018, 0.0, -0.046)),
        material=charcoal,
        name="nozzle_shell",
    )
    floor_nozzle.visual(
        Box((0.290, 0.100, 0.010)),
        origin=Origin(xyz=(0.145, 0.0, -0.055)),
        material=dark_rubber,
        name="sole_plate",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bumper_gray,
        name="nozzle_hinge_barrel",
    )
    floor_nozzle.visual(
        Box((0.056, 0.030, 0.036)),
        origin=Origin(xyz=(0.024, 0.0, -0.010)),
        material=bumper_gray,
        name="neck_bridge",
    )
    floor_nozzle.visual(
        Box((0.064, 0.050, 0.046)),
        origin=Origin(xyz=(0.032, 0.0, -0.018)),
        material=bumper_gray,
        name="neck_block",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.012, length=0.086),
        origin=Origin(xyz=(0.048, 0.0, -0.054), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bumper_gray,
        name="rear_roller",
    )
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.30, 0.11, 0.06)),
        mass=0.85,
        origin=Origin(xyz=(0.15, 0.0, 0.025)),
    )

    model.articulation(
        "body_to_guard",
        ArticulationType.FIXED,
        parent=body,
        child=guard_frame,
        origin=Origin(),
    )
    model.articulation(
        "body_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_wand,
        origin=Origin(xyz=(0.194, 0.0, 0.180), rpy=(0.0, -0.38, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.25, upper=0.55),
    )
    model.articulation(
        "lower_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=upper_wand,
        origin=Origin(xyz=(0.240, 0.0, 0.0), rpy=(0.0, 0.88, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=-0.25, upper=0.90),
    )
    model.articulation(
        "upper_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.405, 0.0, 0.0), rpy=(0.0, -0.45, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=-0.35, upper=0.75),
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
    guard_frame = object_model.get_part("guard_frame")
    lower_wand = object_model.get_part("lower_wand")
    upper_wand = object_model.get_part("upper_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")

    elbow_1 = object_model.get_articulation("body_to_lower_wand")
    elbow_2 = object_model.get_articulation("lower_to_upper_wand")
    nozzle_pitch = object_model.get_articulation("upper_to_floor_nozzle")

    ctx.expect_origin_gap(lower_wand, body, axis="x", min_gap=0.10, name="lower wand starts forward of body")
    ctx.expect_origin_gap(upper_wand, lower_wand, axis="x", min_gap=0.10, name="upper wand continues forward from first elbow")
    ctx.expect_overlap(lower_wand, guard_frame, axes="xz", min_overlap=0.08, name="guard frame spans the first wand stage")
    ctx.expect_within(lower_wand, guard_frame, axes="y", margin=0.09, name="first wand stage stays centered inside guard width")

    sole_aabb = ctx.part_element_world_aabb(floor_nozzle, elem="sole_plate")
    ctx.check(
        "floor nozzle rests near the floor plane",
        sole_aabb is not None and abs(sole_aabb[0][2]) <= 0.006,
        details=f"sole_plate_aabb={sole_aabb}",
    )

    rest_upper = ctx.part_world_position(upper_wand)
    with ctx.pose({elbow_1: 0.40, elbow_2: 0.40}):
        lifted_upper = ctx.part_world_position(upper_wand)
    ctx.check(
        "wand elbows lift the distal stage",
        rest_upper is not None and lifted_upper is not None and lifted_upper[2] > rest_upper[2] + 0.06,
        details=f"rest_upper={rest_upper}, lifted_upper={lifted_upper}",
    )

    rest_front = ctx.part_element_world_aabb(floor_nozzle, elem="sole_plate")
    with ctx.pose({nozzle_pitch: 0.45}):
        pitched_front = ctx.part_element_world_aabb(floor_nozzle, elem="sole_plate")
    ctx.check(
        "floor nozzle pitches upward around its rear hinge",
        rest_front is not None
        and pitched_front is not None
        and pitched_front[1][2] > rest_front[1][2] + 0.050,
        details=f"rest_front={rest_front}, pitched_front={pitched_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
