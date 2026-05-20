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


def _xy_section(width: float, depth: float, z: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _yz_section(
    width: float,
    height: float,
    x: float,
    z_center: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z_center + z) for y, z in rounded_rect_profile(width, height, radius)]


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_pt, max_pt = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_pt, max_pt))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    housing_gray = model.material("housing_gray", rgba=(0.50, 0.53, 0.58, 1.0))
    link_gray = model.material("link_gray", rgba=(0.26, 0.29, 0.33, 1.0))
    joint_steel = model.material("joint_steel", rgba=(0.66, 0.69, 0.72, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.10, 0.11, 0.12, 1.0))
    accent = model.material("accent", rgba=(0.78, 0.47, 0.14, 1.0))

    housing = model.part("upper_leg_housing")
    housing_shell = section_loft(
        [
            _xy_section(0.14, 0.12, 0.03, 0.020),
            _xy_section(0.18, 0.14, 0.10, 0.030),
            _xy_section(0.15, 0.11, 0.18, 0.024),
        ]
    )
    housing.visual(
        mesh_from_geometry(housing_shell, "upper_leg_housing_shell"),
        material=housing_gray,
        name="housing_shell",
    )
    housing.visual(
        Box((0.11, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=link_gray,
        name="hip_cross_block",
    )
    for side_sign, label in ((1.0, "left"), (-1.0, "right")):
        housing.visual(
            Box((0.045, 0.024, 0.060)),
            origin=Origin(xyz=(0.0, side_sign * 0.048, 0.0)),
            material=link_gray,
            name=f"{label}_hip_strut",
        )
        housing.visual(
            Cylinder(radius=0.024, length=0.028),
            origin=Origin(
                xyz=(0.0, side_sign * 0.046, -0.006),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=joint_steel,
            name=f"{label}_hip_boss",
        )
    housing.visual(
        Box((0.045, 0.035, 0.030)),
        origin=Origin(xyz=(0.055, 0.0, 0.085)),
        material=accent,
        name="sensor_pack",
    )
    housing.visual(
        Box((0.025, 0.032, 0.100)),
        origin=Origin(xyz=(0.040, 0.0, 0.055)),
        material=link_gray,
        name="sensor_mount",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.18, 0.14, 0.20)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    thigh = model.part("thigh_link")
    thigh_beam = section_loft(
        [
            _xy_section(0.095, 0.070, -0.04, 0.016),
            _xy_section(0.085, 0.060, -0.22, 0.014),
            _xy_section(0.075, 0.052, -0.40, 0.012),
        ]
    )
    thigh.visual(
        mesh_from_geometry(thigh_beam, "thigh_primary_beam"),
        material=link_gray,
        name="primary_beam",
    )
    thigh.visual(
        Cylinder(radius=0.020, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, -0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_steel,
        name="hip_barrel",
    )
    thigh.visual(
        Box((0.092, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=link_gray,
        name="upper_shoulder",
    )
    thigh.visual(
        Box((0.050, 0.046, 0.200)),
        origin=Origin(xyz=(0.028, 0.0, -0.200)),
        material=housing_gray,
        name="actuator_fairing",
    )
    thigh.visual(
        Box((0.100, 0.055, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.410)),
        material=link_gray,
        name="knee_block",
    )
    for side_sign, label in ((1.0, "left"), (-1.0, "right")):
        thigh.visual(
            Cylinder(radius=0.019, length=0.026),
            origin=Origin(
                xyz=(0.0, side_sign * 0.040, -0.442),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=joint_steel,
            name=f"{label}_knee_boss",
        )
    thigh.inertial = Inertial.from_geometry(
        Box((0.11, 0.07, 0.44)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=thigh,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.8,
            lower=-0.65,
            upper=1.10,
        ),
    )

    shank = model.part("shank_link")
    shank_beam = section_loft(
        [
            _xy_section(0.078, 0.056, -0.035, 0.014),
            _xy_section(0.066, 0.048, -0.150, 0.012),
            _xy_section(0.056, 0.042, -0.285, 0.010),
        ]
    )
    shank.visual(
        mesh_from_geometry(shank_beam, "shank_secondary_beam"),
        material=link_gray,
        name="secondary_beam",
    )
    shank.visual(
        Cylinder(radius=0.018, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_steel,
        name="knee_barrel",
    )
    shank.visual(
        Box((0.082, 0.052, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=link_gray,
        name="upper_knuckle",
    )
    shank.visual(
        Box((0.040, 0.040, 0.130)),
        origin=Origin(xyz=(-0.018, 0.0, -0.175)),
        material=housing_gray,
        name="damper_cover",
    )
    shank.visual(
        Box((0.084, 0.048, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.305)),
        material=link_gray,
        name="ankle_block",
    )
    shank.inertial = Inertial.from_geometry(
        Box((0.09, 0.06, 0.33)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, -0.165)),
    )

    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.0, 0.0, -0.440)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=2.0,
            lower=0.0,
            upper=2.20,
        ),
    )

    foot = model.part("ankle_foot")
    foot_shell = section_loft(
        [
            _yz_section(0.072, 0.056, -0.020, -0.032, 0.010),
            _yz_section(0.090, 0.070, 0.090, -0.050, 0.014),
            _yz_section(0.066, 0.040, 0.180, -0.058, 0.008),
        ]
    )
    foot.visual(
        mesh_from_geometry(foot_shell, "ankle_foot_shell"),
        material=housing_gray,
        name="foot_shell",
    )
    foot.visual(
        Box((0.060, 0.060, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=link_gray,
        name="ankle_cleat",
    )
    foot.visual(
        Box((0.220, 0.082, 0.018)),
        origin=Origin(xyz=(0.085, 0.0, -0.083)),
        material=foot_rubber,
        name="sole_pad",
    )
    foot.visual(
        Box((0.032, 0.074, 0.028)),
        origin=Origin(xyz=(0.175, 0.0, -0.060)),
        material=accent,
        name="toe_bumper",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.23, 0.09, 0.10)),
        mass=1.6,
        origin=Origin(xyz=(0.085, 0.0, -0.050)),
    )

    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.3325)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=2.2,
            lower=-0.55,
            upper=0.55,
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
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("ankle_foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    rest_knee = ctx.part_element_world_aabb(thigh, elem="knee_block")
    rest_center = _aabb_center(rest_knee)
    with ctx.pose({hip: 0.70}):
        flexed_knee = ctx.part_element_world_aabb(thigh, elem="knee_block")
    flexed_center = _aabb_center(flexed_knee)
    ctx.check(
        "hip flexion swings the thigh forward",
        rest_center is not None
        and flexed_center is not None
        and flexed_center[0] > rest_center[0] + 0.12,
        details=f"rest_center={rest_center}, flexed_center={flexed_center}",
    )

    thigh_aabb = ctx.part_world_aabb(thigh)
    shank_aabb = ctx.part_world_aabb(shank)
    thigh_drop = None if thigh_aabb is None else thigh_aabb[1][2] - thigh_aabb[0][2]
    shank_drop = None if shank_aabb is None else shank_aabb[1][2] - shank_aabb[0][2]
    ctx.check(
        "thigh remains the dominant primary link",
        thigh_drop is not None and shank_drop is not None and thigh_drop > shank_drop + 0.08,
        details=f"thigh_drop={thigh_drop}, shank_drop={shank_drop}",
    )

    rest_ankle_pos = ctx.part_world_position(foot)
    with ctx.pose({knee: 1.20}):
        flexed_ankle_pos = ctx.part_world_position(foot)
    ctx.check(
        "knee flexion carries the ankle forward",
        rest_ankle_pos is not None
        and flexed_ankle_pos is not None
        and flexed_ankle_pos[0] > rest_ankle_pos[0] + 0.12
        and flexed_ankle_pos[2] > rest_ankle_pos[2] + 0.06,
        details=f"rest_ankle_pos={rest_ankle_pos}, flexed_ankle_pos={flexed_ankle_pos}",
    )

    rest_sole = ctx.part_element_world_aabb(foot, elem="sole_pad")
    rest_sole_center = _aabb_center(rest_sole)
    with ctx.pose({ankle: 0.35}):
        toe_down_sole = ctx.part_element_world_aabb(foot, elem="sole_pad")
    toe_down_center = _aabb_center(toe_down_sole)
    ctx.check(
        "ankle pitch dorsiflexes the compact foot upward",
        rest_sole_center is not None
        and toe_down_center is not None
        and toe_down_center[2] > rest_sole_center[2] + 0.015,
        details=f"rest_sole_center={rest_sole_center}, toe_down_center={toe_down_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
