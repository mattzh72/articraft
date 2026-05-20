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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CHEEK_THICKNESS = 0.008
ROOT_BOSS_DEPTH = 0.006
LINK_BODY_THICKNESS = 0.008

LINK1_LENGTH = 0.086
LINK2_BODY_LENGTH = 0.058
LINK2_TAB_TIP = 0.078

HALF_PI = pi / 2.0

CHEEK_CENTER_Y = -CHEEK_THICKNESS / 2.0
ROOT_BOSS_CENTER_Y = ROOT_BOSS_DEPTH / 2.0
LINK1_CENTER_Y = ROOT_BOSS_DEPTH + LINK_BODY_THICKNESS / 2.0
LINK2_CENTER_Y = ROOT_BOSS_DEPTH + LINK_BODY_THICKNESS + LINK_BODY_THICKNESS / 2.0


def _y_cylinder_origin(x: float, center_y: float, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, center_y, z), rpy=(HALF_PI, 0.0, 0.0))


def add_cheek_geometry(part, material) -> None:
    part.visual(
        Box((0.050, CHEEK_THICKNESS, 0.072)),
        origin=Origin(xyz=(-0.018, CHEEK_CENTER_Y, -0.014)),
        material=material,
        name="cheek_main_plate",
    )
    part.visual(
        Box((0.022, CHEEK_THICKNESS, 0.022)),
        origin=Origin(xyz=(-0.038, CHEEK_CENTER_Y, -0.047)),
        material=material,
        name="cheek_lower_tail",
    )
    part.visual(
        Cylinder(radius=0.014, length=ROOT_BOSS_DEPTH),
        origin=_y_cylinder_origin(0.0, ROOT_BOSS_CENTER_Y),
        material=material,
        name="cheek_root_boss",
    )


def add_link1_geometry(part, material) -> None:
    part.visual(
        Box((LINK1_LENGTH - 0.020, LINK_BODY_THICKNESS, 0.014)),
        origin=Origin(xyz=(LINK1_LENGTH / 2.0, LINK1_CENTER_Y, 0.0)),
        material=material,
        name="link1_web",
    )
    part.visual(
        Cylinder(radius=0.018, length=LINK_BODY_THICKNESS),
        origin=_y_cylinder_origin(0.0, LINK1_CENTER_Y),
        material=material,
        name="link1_root_boss",
    )
    part.visual(
        Cylinder(radius=0.016, length=LINK_BODY_THICKNESS),
        origin=_y_cylinder_origin(LINK1_LENGTH, LINK1_CENTER_Y),
        material=material,
        name="link1_distal_boss",
    )


def add_link2_geometry(part, material) -> None:
    part.visual(
        Cylinder(radius=0.015, length=LINK_BODY_THICKNESS),
        origin=_y_cylinder_origin(0.0, LINK2_CENTER_Y),
        material=material,
        name="link2_joint_boss",
    )
    part.visual(
        Box((LINK2_BODY_LENGTH - 0.012, LINK_BODY_THICKNESS, 0.012)),
        origin=Origin(xyz=(LINK2_BODY_LENGTH / 2.0, LINK2_CENTER_Y, 0.0)),
        material=material,
        name="link2_web",
    )
    part.visual(
        Box((0.014, LINK_BODY_THICKNESS, 0.010)),
        origin=Origin(xyz=(0.052, LINK2_CENTER_Y, 0.0)),
        material=material,
        name="link2_tab_neck",
    )
    part.visual(
        Box((0.022, LINK_BODY_THICKNESS, 0.012)),
        origin=Origin(xyz=(0.066, LINK2_CENTER_Y, 0.0)),
        material=material,
        name="link2_end_tab",
    )
    part.visual(
        Cylinder(radius=0.008, length=LINK_BODY_THICKNESS),
        origin=_y_cylinder_origin(LINK2_TAB_TIP, LINK2_CENTER_Y),
        material=material,
        name="link2_tab_tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_two_joint_chain")

    cheek_material = model.material("cheek_steel", color=(0.22, 0.23, 0.25))
    link1_material = model.material("link_steel", color=(0.57, 0.60, 0.64))
    link2_material = model.material("distal_link_steel", color=(0.48, 0.52, 0.57))

    cheek = model.part("cheek")
    add_cheek_geometry(cheek, cheek_material)

    link1 = model.part("link1")
    add_link1_geometry(link1, link1_material)

    link2 = model.part("link2")
    add_link2_geometry(link2, link2_material)

    model.articulation(
        "cheek_to_link1",
        ArticulationType.REVOLUTE,
        parent=cheek,
        child=link1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-1.15, upper=1.15),
    )

    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.4, lower=-1.55, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cheek = object_model.get_part("cheek")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    joint1 = object_model.get_articulation("cheek_to_link1")
    joint2 = object_model.get_articulation("link1_to_link2")

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

    ctx.expect_contact(cheek, link1, name="cheek_supports_first_link")
    ctx.expect_contact(link1, link2, name="first_link_supports_second_link")

    ctx.check(
        "joint_axes_are_parallel_supported_pivots",
        joint1.axis == (0.0, -1.0, 0.0) and joint2.axis == (0.0, -1.0, 0.0),
        details=f"joint axes were {joint1.axis!r} and {joint2.axis!r}",
    )

    limits_ok = (
        joint1.motion_limits is not None
        and joint2.motion_limits is not None
        and joint1.motion_limits.lower is not None
        and joint1.motion_limits.upper is not None
        and joint2.motion_limits.lower is not None
        and joint2.motion_limits.upper is not None
        and joint1.motion_limits.lower < 0.0 < joint1.motion_limits.upper
        and joint2.motion_limits.lower < 0.0 < joint2.motion_limits.upper
    )
    ctx.check(
        "joint_limits_allow_compact_chain_swing",
        limits_ok,
        details="expected bounded bidirectional revolute travel on both joints",
    )

    with ctx.pose({joint1: 0.0, joint2: 0.0}):
        rest_aabb = ctx.part_world_aabb(link2)

    with ctx.pose({joint1: 0.8, joint2: 0.6}):
        raised_aabb = ctx.part_world_aabb(link2)
        ctx.fail_if_parts_overlap_in_current_pose(name="raised_pose_no_unintended_overlaps")

    rest_center_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
    raised_center_z = None if raised_aabb is None else (raised_aabb[0][2] + raised_aabb[1][2]) / 2.0
    ctx.check(
        "positive_joint_motion_lifts_distal_end_tab",
        rest_center_z is not None
        and raised_center_z is not None
        and raised_center_z > rest_center_z + 0.02,
        details=f"rest z={rest_center_z!r}, raised z={raised_center_z!r}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
