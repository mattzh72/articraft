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
)


TRUNNION_LENGTH = 0.022
DRUM_LENGTH = 0.060
TRUNNION_CENTER_Y = 0.041
SCALE_PLATE_CENTER_Y = 0.055
POINTER_CENTER_Y = 0.040


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _section_xy(
    width: float,
    depth: float,
    z: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    radius: float | None = None,
) -> list[tuple[float, float, float]]:
    corner = min(radius or min(width, depth) * 0.22, width * 0.48, depth * 0.48)
    return [(x + px, y + py, z) for px, py in rounded_rect_profile(width, depth, corner)]


def _section_yz(
    width_y: float,
    height_z: float,
    x: float,
    *,
    z_center: float = 0.0,
    radius: float | None = None,
) -> list[tuple[float, float, float]]:
    corner = min(radius or min(width_y, height_z) * 0.22, width_y * 0.48, height_z * 0.48)
    return [(x, py, z_center + pz) for py, pz in rounded_rect_profile(width_y, height_z, corner)]


def _add_y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_scale_plate(part, *, x: float, z: float, material, name: str) -> None:
    part.visual(
        Box((0.042, 0.006, 0.060)),
        origin=Origin(xyz=(x, SCALE_PLATE_CENTER_Y, z)),
        material=material,
        name=name,
    )


def _add_scale_ticks(part, *, x: float, z: float, material, prefix: str) -> None:
    for idx, (dx, tick_h) in enumerate(((-0.012, 0.018), (0.0, 0.028), (0.012, 0.018))):
        part.visual(
            Box((0.0025, 0.002, tick_h)),
            origin=Origin(xyz=(x + dx, 0.056, z)),
            material=material,
            name=f"{prefix}_{idx}",
        )


def _add_pointer(
    part,
    *,
    joint_x: float,
    joint_z: float,
    material,
    name: str,
    x_offset: float = 0.022,
) -> None:
    part.visual(
        Box((0.014, 0.020, 0.040)),
        origin=Origin(xyz=(joint_x + x_offset, POINTER_CENTER_Y, joint_z)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_robotic_leg")

    shell_gray = model.material("shell_gray", rgba=(0.67, 0.70, 0.73, 1.0))
    dark_shell = model.material("dark_shell", rgba=(0.16, 0.18, 0.20, 1.0))
    hard_anodized = model.material("hard_anodized", rgba=(0.28, 0.31, 0.34, 1.0))
    datum_silver = model.material("datum_silver", rgba=(0.82, 0.84, 0.86, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.16, 0.44, 0.68, 1.0))
    sole_rubber = model.material("sole_rubber", rgba=(0.08, 0.09, 0.10, 1.0))

    hip_carrier = model.part("hip_carrier")
    hip_carrier.visual(
        Box((0.18, 0.12, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=hard_anodized,
        name="mount_plate",
    )
    hip_carrier.visual(
        Box((0.092, 0.060, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=hard_anodized,
        name="carrier_core",
    )
    hip_carrier.visual(
        Box((0.080, 0.016, 0.098)),
        origin=Origin(xyz=(0.0, -TRUNNION_CENTER_Y, 0.010)),
        material=hard_anodized,
        name="left_cheek",
    )
    hip_carrier.visual(
        Box((0.080, 0.016, 0.098)),
        origin=Origin(xyz=(0.0, TRUNNION_CENTER_Y, 0.010)),
        material=hard_anodized,
        name="right_cheek",
    )
    _add_y_cylinder(
        hip_carrier,
        radius=0.034,
        length=TRUNNION_LENGTH,
        xyz=(0.0, -TRUNNION_CENTER_Y, 0.0),
        material=hard_anodized,
        name="left_hip_trunnion",
    )
    _add_y_cylinder(
        hip_carrier,
        radius=0.034,
        length=TRUNNION_LENGTH,
        xyz=(0.0, TRUNNION_CENTER_Y, 0.0),
        material=hard_anodized,
        name="right_hip_trunnion",
    )
    hip_carrier.visual(
        Box((0.008, 0.050, 0.060)),
        origin=Origin(xyz=(0.050, 0.0, 0.032)),
        material=datum_silver,
        name="hip_front_datum",
    )
    _add_scale_plate(hip_carrier, x=0.0, z=0.0, material=datum_silver, name="hip_scale_plate")
    _add_scale_ticks(hip_carrier, x=0.0, z=0.0, material=accent_blue, prefix="hip_index_tick")
    hip_carrier.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.12)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    thigh_module = model.part("thigh_module")
    _add_y_cylinder(
        thigh_module,
        radius=0.032,
        length=DRUM_LENGTH,
        xyz=(0.0, 0.0, 0.0),
        material=dark_shell,
        name="hip_drum",
    )
    thigh_module.visual(
        Box((0.055, DRUM_LENGTH, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=hard_anodized,
        name="hip_bridge",
    )
    thigh_shell = section_loft(
        [
            _section_xy(0.078, 0.054, -0.030, radius=0.012),
            _section_xy(0.096, 0.068, -0.160, radius=0.014),
            _section_xy(0.090, 0.066, -0.300, radius=0.013),
            _section_xy(0.094, 0.070, -0.400, radius=0.014),
        ]
    )
    thigh_module.visual(
        _mesh("thigh_shell", thigh_shell),
        material=shell_gray,
        name="thigh_shell",
    )
    thigh_module.visual(
        Box((0.010, 0.050, 0.210)),
        origin=Origin(xyz=(0.051, 0.0, -0.205)),
        material=datum_silver,
        name="thigh_front_datum_rail",
    )
    thigh_module.visual(
        Box((0.012, 0.046, 0.185)),
        origin=Origin(xyz=(0.047, 0.0, -0.205)),
        material=dark_shell,
        name="thigh_actuator_bay",
    )
    thigh_module.visual(
        Box((0.024, 0.040, 0.010)),
        origin=Origin(xyz=(0.050, 0.0, -0.338)),
        material=datum_silver,
        name="thigh_adjust_pad",
    )
    thigh_module.visual(
        Box((0.024, 0.072, 0.060)),
        origin=Origin(xyz=(-0.020, 0.0, -0.480)),
        material=hard_anodized,
        name="knee_housing",
    )
    thigh_module.visual(
        Box((0.024, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, -0.042, -0.448)),
        material=hard_anodized,
        name="left_knee_support",
    )
    thigh_module.visual(
        Box((0.024, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, 0.042, -0.448)),
        material=hard_anodized,
        name="right_knee_support",
    )
    _add_y_cylinder(
        thigh_module,
        radius=0.030,
        length=TRUNNION_LENGTH,
        xyz=(0.0, -TRUNNION_CENTER_Y, -0.420),
        material=hard_anodized,
        name="left_knee_trunnion",
    )
    _add_y_cylinder(
        thigh_module,
        radius=0.030,
        length=TRUNNION_LENGTH,
        xyz=(0.0, TRUNNION_CENTER_Y, -0.420),
        material=hard_anodized,
        name="right_knee_trunnion",
    )
    _add_scale_plate(thigh_module, x=0.0, z=-0.420, material=datum_silver, name="knee_scale_plate")
    thigh_module.visual(
        Box((0.016, 0.012, 0.062)),
        origin=Origin(xyz=(0.0, 0.037, -0.420)),
        material=datum_silver,
        name="knee_scale_support_base",
    )
    thigh_module.visual(
        Box((0.014, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, 0.047, -0.390)),
        material=datum_silver,
        name="knee_scale_support_upper",
    )
    _add_scale_ticks(thigh_module, x=0.0, z=-0.420, material=accent_blue, prefix="knee_index_tick")
    _add_pointer(
        thigh_module,
        joint_x=0.0,
        joint_z=0.0,
        material=accent_blue,
        name="hip_pointer",
        x_offset=0.046,
    )
    thigh_module.visual(
        Box((0.024, 0.010, 0.020)),
        origin=Origin(xyz=(0.034, 0.030, -0.010)),
        material=accent_blue,
        name="hip_pointer_bracket",
    )
    thigh_module.inertial = Inertial.from_geometry(
        Box((0.12, 0.09, 0.46)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, -0.215)),
    )

    shank_module = model.part("shank_module")
    shank_module.visual(
        Box((0.034, 0.056, 0.024)),
        origin=Origin(xyz=(0.046, 0.0, -0.018)),
        material=hard_anodized,
        name="knee_bridge",
    )
    shank_module.visual(
        Box((0.024, 0.018, 0.072)),
        origin=Origin(xyz=(0.042, -0.024, -0.036)),
        material=hard_anodized,
        name="left_knee_clevis",
    )
    shank_module.visual(
        Box((0.024, 0.018, 0.072)),
        origin=Origin(xyz=(0.042, 0.024, -0.036)),
        material=hard_anodized,
        name="right_knee_clevis",
    )
    shank_module.visual(
        Box((0.032, 0.046, 0.300)),
        origin=Origin(xyz=(0.032, 0.0, -0.200)),
        material=hard_anodized,
        name="shank_spine",
    )
    shank_module.visual(
        Box((0.074, 0.058, 0.240)),
        origin=Origin(xyz=(0.030, 0.0, -0.230)),
        material=shell_gray,
        name="shank_shell",
    )
    shank_module.visual(
        Box((0.010, 0.042, 0.198)),
        origin=Origin(xyz=(0.054, 0.0, -0.220)),
        material=datum_silver,
        name="shank_front_datum_rail",
    )
    shank_module.visual(
        Box((0.012, 0.038, 0.180)),
        origin=Origin(xyz=(0.048, 0.0, -0.220)),
        material=dark_shell,
        name="shank_actuator_bay",
    )
    shank_module.visual(
        Box((0.020, 0.036, 0.018)),
        origin=Origin(xyz=(0.049, 0.0, -0.320)),
        material=datum_silver,
        name="shank_adjust_pad",
    )
    shank_module.visual(
        Box((0.028, 0.072, 0.054)),
        origin=Origin(xyz=(-0.018, 0.0, -0.453)),
        material=hard_anodized,
        name="ankle_housing",
    )
    shank_module.visual(
        Box((0.056, 0.018, 0.068)),
        origin=Origin(xyz=(0.000, -0.042, -0.411)),
        material=hard_anodized,
        name="left_ankle_support",
    )
    shank_module.visual(
        Box((0.056, 0.018, 0.068)),
        origin=Origin(xyz=(0.000, 0.042, -0.411)),
        material=hard_anodized,
        name="right_ankle_support",
    )
    shank_module.visual(
        Box((0.020, 0.046, 0.136)),
        origin=Origin(xyz=(-0.014, 0.0, -0.366)),
        material=hard_anodized,
        name="ankle_transition",
    )
    _add_y_cylinder(
        shank_module,
        radius=0.028,
        length=TRUNNION_LENGTH,
        xyz=(0.022, -TRUNNION_CENTER_Y, -0.390),
        material=hard_anodized,
        name="left_ankle_trunnion",
    )
    _add_y_cylinder(
        shank_module,
        radius=0.028,
        length=TRUNNION_LENGTH,
        xyz=(0.022, TRUNNION_CENTER_Y, -0.390),
        material=hard_anodized,
        name="right_ankle_trunnion",
    )
    _add_scale_plate(shank_module, x=0.022, z=-0.390, material=datum_silver, name="ankle_scale_plate")
    shank_module.visual(
        Box((0.016, 0.014, 0.054)),
        origin=Origin(xyz=(0.022, 0.041, -0.418)),
        material=datum_silver,
        name="ankle_scale_support_base",
    )
    shank_module.visual(
        Box((0.014, 0.010, 0.058)),
        origin=Origin(xyz=(0.022, 0.050, -0.390)),
        material=datum_silver,
        name="ankle_scale_support_upper",
    )
    _add_scale_ticks(shank_module, x=0.022, z=-0.390, material=accent_blue, prefix="ankle_index_tick")
    _add_pointer(
        shank_module,
        joint_x=0.0,
        joint_z=0.0,
        material=accent_blue,
        name="knee_pointer",
        x_offset=0.050,
    )
    shank_module.visual(
        Box((0.020, 0.012, 0.032)),
        origin=Origin(xyz=(0.040, 0.032, -0.014)),
        material=accent_blue,
        name="knee_pointer_bracket",
    )
    shank_module.inertial = Inertial.from_geometry(
        Box((0.11, 0.08, 0.42)),
        mass=7.0,
        origin=Origin(xyz=(0.020, 0.0, -0.215)),
    )

    foot_module = model.part("foot_module")
    foot_module.visual(
        Box((0.024, 0.018, 0.060)),
        origin=Origin(xyz=(0.038, -0.024, -0.030)),
        material=hard_anodized,
        name="left_ankle_clevis",
    )
    foot_module.visual(
        Box((0.024, 0.018, 0.060)),
        origin=Origin(xyz=(0.038, 0.024, -0.030)),
        material=hard_anodized,
        name="right_ankle_clevis",
    )
    foot_module.visual(
        Box((0.050, 0.056, 0.024)),
        origin=Origin(xyz=(0.054, 0.0, -0.018)),
        material=hard_anodized,
        name="ankle_bridge",
    )
    foot_module.visual(
        Box((0.040, 0.054, 0.082)),
        origin=Origin(xyz=(0.086, 0.0, -0.053)),
        material=hard_anodized,
        name="ankle_pedestal",
    )
    foot_shell = section_loft(
        [
            _section_yz(0.066, 0.030, 0.100, z_center=-0.086, radius=0.008),
            _section_yz(0.090, 0.040, 0.162, z_center=-0.084, radius=0.010),
            _section_yz(0.080, 0.034, 0.248, z_center=-0.078, radius=0.008),
            _section_yz(0.050, 0.020, 0.292, z_center=-0.074, radius=0.005),
        ]
    )
    foot_module.visual(
        _mesh("foot_shell", foot_shell),
        material=shell_gray,
        name="foot_shell",
    )
    foot_module.visual(
        Box((0.290, 0.090, 0.018)),
        origin=Origin(xyz=(0.140, 0.0, -0.089)),
        material=sole_rubber,
        name="sole_pad",
    )
    foot_module.visual(
        Box((0.090, 0.052, 0.008)),
        origin=Origin(xyz=(0.098, 0.0, -0.022)),
        material=datum_silver,
        name="foot_top_datum",
    )
    foot_module.visual(
        Box((0.034, 0.030, 0.010)),
        origin=Origin(xyz=(0.232, 0.0, -0.049)),
        material=datum_silver,
        name="toe_adjuster",
    )
    foot_module.visual(
        Box((0.020, 0.024, 0.026)),
        origin=Origin(xyz=(0.232, 0.0, -0.062)),
        material=datum_silver,
        name="toe_adjuster_pedestal",
    )
    foot_module.visual(
        Box((0.016, 0.020, 0.038)),
        origin=Origin(xyz=(0.060, POINTER_CENTER_Y, 0.0)),
        material=accent_blue,
        name="ankle_pointer",
    )
    foot_module.visual(
        Box((0.016, 0.012, 0.038)),
        origin=Origin(xyz=(0.052, 0.034, -0.010)),
        material=accent_blue,
        name="ankle_pointer_bracket",
    )
    foot_module.inertial = Inertial.from_geometry(
        Box((0.32, 0.10, 0.11)),
        mass=3.2,
        origin=Origin(xyz=(0.140, 0.0, -0.058)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_carrier,
        child=thigh_module,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=2.0,
            lower=math.radians(-35.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_module,
        child=shank_module,
        origin=Origin(xyz=(0.0, 0.0, -0.420)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(135.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_module,
        child=foot_module,
        origin=Origin(xyz=(0.022, 0.0, -0.390)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=95.0,
            velocity=2.0,
            lower=math.radians(-25.0),
            upper=math.radians(35.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip_carrier = object_model.get_part("hip_carrier")
    thigh_module = object_model.get_part("thigh_module")
    shank_module = object_model.get_part("shank_module")
    foot_module = object_model.get_part("foot_module")

    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    hip_scale_plate = hip_carrier.get_visual("hip_scale_plate")
    knee_scale_plate = thigh_module.get_visual("knee_scale_plate")
    ankle_scale_plate = shank_module.get_visual("ankle_scale_plate")
    hip_pointer = thigh_module.get_visual("hip_pointer")
    knee_pointer = shank_module.get_visual("knee_pointer")
    ankle_pointer = foot_module.get_visual("ankle_pointer")
    toe_adjuster = foot_module.get_visual("toe_adjuster")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(hip_carrier, thigh_module, name="hip_bearing_stack_connected")
    ctx.expect_contact(thigh_module, shank_module, name="knee_bearing_stack_connected")
    ctx.expect_contact(shank_module, foot_module, name="ankle_bearing_stack_connected")

    ctx.expect_gap(
        hip_carrier,
        thigh_module,
        axis="y",
        positive_elem=hip_scale_plate,
        negative_elem=hip_pointer,
        min_gap=0.0015,
        max_gap=0.0025,
        name="hip_index_gap_controlled",
    )
    ctx.expect_gap(
        thigh_module,
        shank_module,
        axis="y",
        positive_elem=knee_scale_plate,
        negative_elem=knee_pointer,
        min_gap=0.0015,
        max_gap=0.0025,
        name="knee_index_gap_controlled",
    )
    ctx.expect_gap(
        shank_module,
        foot_module,
        axis="y",
        positive_elem=ankle_scale_plate,
        negative_elem=ankle_pointer,
        min_gap=0.0015,
        max_gap=0.0025,
        name="ankle_index_gap_controlled",
    )

    hip_limits = hip_pitch.motion_limits
    knee_limits = knee_pitch.motion_limits
    ankle_limits = ankle_pitch.motion_limits
    ctx.check(
        "serial_leg_axes_are_explicit",
        hip_pitch.axis == (0.0, -1.0, 0.0)
        and knee_pitch.axis == (0.0, 1.0, 0.0)
        and ankle_pitch.axis == (0.0, -1.0, 0.0),
        details=(
            f"Expected hip/knee/ankle pitch axes to be (-Y, +Y, -Y); "
            f"got {hip_pitch.axis}, {knee_pitch.axis}, {ankle_pitch.axis}"
        ),
    )
    ctx.check(
        "joint_limits_cover_plausible_calibration_range",
        hip_limits is not None
        and knee_limits is not None
        and ankle_limits is not None
        and hip_limits.lower is not None
        and hip_limits.upper is not None
        and knee_limits.lower == 0.0
        and knee_limits.upper is not None
        and ankle_limits.lower is not None
        and ankle_limits.upper is not None
        and hip_limits.lower < 0.0 < hip_limits.upper
        and knee_limits.upper >= math.radians(120.0)
        and ankle_limits.lower <= math.radians(-20.0)
        and ankle_limits.upper >= math.radians(30.0),
        details="Pitch limits should allow standing alignment, knee foldback, and ankle trim adjustment.",
    )

    neutral_foot_pos = ctx.part_world_position(foot_module)
    with ctx.pose({hip_pitch: math.radians(45.0)}):
        hip_flexed_foot_pos = ctx.part_world_position(foot_module)
    ctx.check(
        "hip_positive_motion_swings_leg_forward",
        neutral_foot_pos is not None
        and hip_flexed_foot_pos is not None
        and hip_flexed_foot_pos[0] > neutral_foot_pos[0] + 0.10,
        details="Positive hip pitch should move the distal leg forward in +X.",
    )

    with ctx.pose({knee_pitch: math.radians(100.0)}):
        knee_flexed_foot_pos = ctx.part_world_position(foot_module)
    ctx.check(
        "knee_positive_motion_retracts_and_lifts_distal_chain",
        neutral_foot_pos is not None
        and knee_flexed_foot_pos is not None
        and knee_flexed_foot_pos[0] < neutral_foot_pos[0] - 0.05
        and knee_flexed_foot_pos[2] > neutral_foot_pos[2] + 0.12,
        details="Positive knee pitch should lift the ankle and pull it back for fold-up calibration poses.",
    )

    neutral_toe_aabb = ctx.part_element_world_aabb(foot_module, elem=toe_adjuster)
    with ctx.pose({ankle_pitch: math.radians(25.0)}):
        dorsiflexed_toe_aabb = ctx.part_element_world_aabb(foot_module, elem=toe_adjuster)
    ctx.check(
        "ankle_positive_motion_lifts_toe_adjuster",
        neutral_toe_aabb is not None
        and dorsiflexed_toe_aabb is not None
        and dorsiflexed_toe_aabb[1][2] > neutral_toe_aabb[1][2] + 0.025,
        details="Positive ankle pitch should dorsiflex the toe and raise the front adjustment feature.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
