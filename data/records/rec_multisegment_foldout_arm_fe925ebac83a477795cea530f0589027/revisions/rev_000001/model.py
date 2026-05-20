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
)


HINGE_GAP = 0.040
CHEEK_THICKNESS = 0.016
OUTER_WIDTH = HINGE_GAP + 2.0 * CHEEK_THICKNESS
BOSS_RADIUS = 0.024
BOSS_LENGTH = 0.012
ROOT_LUG_LENGTH = 0.044
FORK_LENGTH = 0.048
FORK_CENTER_OFFSET = -0.010

LINK1_LENGTH = 0.220
LINK2_LENGTH = 0.190
LINK3_LENGTH = 0.155


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_y_cylinder(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _build_base(model: ArticulatedObject):
    base = model.part("base")
    material = "painted_steel"

    _add_box(base, "pedestal", (0.190, 0.150, 0.090), (-0.075, 0.0, -0.065), material)
    _add_box(base, "upper_pad", (0.076, 0.106, 0.022), (-0.044, 0.0, -0.011), material)
    _add_box(base, "left_cheek", (0.056, CHEEK_THICKNESS, 0.088), (-0.002, 0.028, 0.014), material)
    _add_box(base, "right_cheek", (0.056, CHEEK_THICKNESS, 0.088), (-0.002, -0.028, 0.014), material)
    _add_box(base, "rear_bridge", (0.026, HINGE_GAP, 0.032), (-0.030, 0.0, 0.000), material)
    _add_box(base, "left_rib", (0.054, CHEEK_THICKNESS, 0.036), (-0.030, 0.028, -0.012), material)
    _add_box(base, "right_rib", (0.054, CHEEK_THICKNESS, 0.036), (-0.030, -0.028, -0.012), material)
    _add_box(base, "top_tie", (0.022, 0.052, 0.012), (-0.018, 0.0, 0.048), material)
    _add_y_cylinder(base, "left_boss", BOSS_RADIUS, BOSS_LENGTH, (0.0, 0.041, 0.0), material)
    _add_y_cylinder(base, "right_boss", BOSS_RADIUS, BOSS_LENGTH, (0.0, -0.041, 0.0), material)

    base.inertial = Inertial.from_geometry(
        Box((0.190, 0.150, 0.102)),
        mass=8.0,
        origin=Origin(xyz=(-0.072, 0.0, -0.055)),
    )
    return base


def _build_link(
    model: ArticulatedObject,
    *,
    name: str,
    length: float,
    body_width: float,
    body_height: float,
    root_height: float,
    fork_height: float,
    mass: float,
):
    part = model.part(name)
    material = "arm_gray"

    beam_start = ROOT_LUG_LENGTH - 0.002
    beam_end = length - 0.004
    beam_length = beam_end - beam_start
    beam_center = (beam_start + beam_end) / 2.0

    _add_box(part, "root_lug", (ROOT_LUG_LENGTH, HINGE_GAP, root_height), (ROOT_LUG_LENGTH / 2.0, 0.0, 0.0), material)
    _add_box(part, "main_beam", (beam_length, body_width, body_height), (beam_center, 0.0, 0.0), material)
    _add_box(part, "top_cap", (beam_length * 0.72, body_width * 0.62, body_height * 0.24), (beam_center, 0.0, body_height * 0.18), material)
    _add_box(
        part,
        "bottom_rib",
        (beam_length * 0.46, body_width * 0.54, body_height * 0.18),
        (beam_start + beam_length * 0.34, 0.0, -body_height * 0.18),
        material,
    )
    _add_box(part, "left_fork", (FORK_LENGTH, CHEEK_THICKNESS, fork_height), (length + FORK_CENTER_OFFSET, 0.028, 0.0), material)
    _add_box(part, "right_fork", (FORK_LENGTH, CHEEK_THICKNESS, fork_height), (length + FORK_CENTER_OFFSET, -0.028, 0.0), material)
    _add_y_cylinder(part, "left_boss", BOSS_RADIUS, BOSS_LENGTH, (length, 0.041, 0.0), material)
    _add_y_cylinder(part, "right_boss", BOSS_RADIUS, BOSS_LENGTH, (length, -0.041, 0.0), material)

    part.inertial = Inertial.from_geometry(
        Box((length + 0.020, OUTER_WIDTH, body_height + 0.006)),
        mass=mass,
        origin=Origin(xyz=(length * 0.55, 0.0, 0.0)),
    )
    return part


def _build_platform_bracket(model: ArticulatedObject):
    part = model.part("platform_bracket")
    material = "machined_dark"

    _add_box(part, "root_lug", (0.040, HINGE_GAP, 0.032), (0.020, 0.0, 0.0), material)
    _add_box(part, "upright_web", (0.032, 0.022, 0.054), (0.048, 0.0, 0.021), material)
    _add_box(part, "deck", (0.090, 0.074, 0.010), (0.084, 0.0, 0.048), material)
    _add_box(part, "front_lip", (0.010, 0.058, 0.022), (0.124, 0.0, 0.032), material)
    _add_box(part, "left_rib", (0.040, 0.010, 0.036), (0.066, 0.022, 0.026), material)
    _add_box(part, "right_rib", (0.040, 0.010, 0.036), (0.066, -0.022, 0.026), material)
    _add_box(part, "deck_brace", (0.048, 0.020, 0.016), (0.076, 0.0, 0.036), material)

    part.inertial = Inertial.from_geometry(
        Box((0.134, 0.074, 0.058)),
        mass=0.85,
        origin=Origin(xyz=(0.070, 0.0, 0.028)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_foldout_arm")

    model.material("painted_steel", rgba=(0.28, 0.31, 0.34, 1.0))
    model.material("arm_gray", rgba=(0.58, 0.61, 0.64, 1.0))
    model.material("machined_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    base = _build_base(model)
    link1 = _build_link(
        model,
        name="link1",
        length=LINK1_LENGTH,
        body_width=0.052,
        body_height=0.064,
        root_height=0.046,
        fork_height=0.052,
        mass=2.2,
    )
    link2 = _build_link(
        model,
        name="link2",
        length=LINK2_LENGTH,
        body_width=0.050,
        body_height=0.058,
        root_height=0.044,
        fork_height=0.048,
        mass=1.7,
    )
    link3 = _build_link(
        model,
        name="link3",
        length=LINK3_LENGTH,
        body_width=0.046,
        body_height=0.052,
        root_height=0.040,
        fork_height=0.044,
        mass=1.25,
    )
    platform = _build_platform_bracket(model)

    model.articulation(
        "base_to_link1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.45, upper=1.20),
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=1.1, lower=-1.55, upper=0.70),
    )
    model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(LINK2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.2, lower=-1.40, upper=0.75),
    )
    model.articulation(
        "link3_to_platform",
        ArticulationType.REVOLUTE,
        parent=link3,
        child=platform,
        origin=Origin(xyz=(LINK3_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-0.80, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")
    platform = object_model.get_part("platform_bracket")

    joint1 = object_model.get_articulation("base_to_link1")
    joint2 = object_model.get_articulation("link1_to_link2")
    joint3 = object_model.get_articulation("link2_to_link3")
    joint4 = object_model.get_articulation("link3_to_platform")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    joints = (joint1, joint2, joint3, joint4)
    ctx.check(
        "four_parallel_revolute_joints",
        len(joints) == 4 and all(joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, -1.0, 0.0) for joint in joints),
        "Expected a serial chain of four revolute joints with parallel -Y hinge axes.",
    )

    ctx.expect_contact(link1, base, name="base_joint_has_supported_contact")
    ctx.expect_contact(link2, link1, name="link1_to_link2_has_supported_contact")
    ctx.expect_contact(link3, link2, name="link2_to_link3_has_supported_contact")
    ctx.expect_contact(platform, link3, name="link3_to_platform_has_supported_contact")

    with ctx.pose({joint1: 0.75, joint2: 0.55, joint3: 0.40, joint4: 0.18}):
        ctx.expect_origin_gap(platform, base, axis="z", min_gap=0.28, name="deployed_platform_rises_above_base")
        ctx.expect_origin_gap(platform, base, axis="x", min_gap=0.16, name="deployed_platform_projects_forward")
        ctx.expect_contact(link2, link1, name="middle_joint_remains_mated_in_pose")
        ctx.expect_contact(platform, link3, name="platform_joint_remains_mated_in_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
