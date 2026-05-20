from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_PIVOT_X = -0.035
BASE_PIVOT_Z = 0.076
ROOT_CLEVIS_GAP = 0.018
ROOT_CHEEK_THICK = 0.007

LINK1_TO_LINK2_X = 0.165
LINK1_TO_LINK2_Z = 0.028
LINK2_TO_LINK3_X = 0.145
LINK2_TO_LINK3_Z = -0.022

LINK2_ROOT_WIDTH = 0.010
LINK3_ROOT_WIDTH = 0.008

ROOT_FORK_GAP = 0.014
MID_FORK_GAP = 0.012
FORK_CHEEK_THICK = 0.006


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(1.5707963267948966, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rocker_folding_arm_chain")

    model.material("base_coat", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("link_root_finish", rgba=(0.69, 0.72, 0.76, 1.0))
    model.material("link_mid_finish", rgba=(0.58, 0.62, 0.67, 1.0))
    model.material("link_tip_finish", rgba=(0.31, 0.34, 0.38, 1.0))

    base = model.part("base_foot")
    _add_box(base, size=(0.220, 0.100, 0.016), xyz=(0.030, 0.0, 0.008), material="base_coat", name="foot_plate")
    _add_box(base, size=(0.080, 0.072, 0.024), xyz=(-0.030, 0.0, 0.028), material="base_coat", name="heel_step")
    _add_box(base, size=(0.050, 0.014, 0.022), xyz=(-0.050, 0.018, 0.050), material="base_coat", name="left_rib")
    _add_box(base, size=(0.050, 0.014, 0.022), xyz=(-0.050, -0.018, 0.050), material="base_coat", name="right_rib")
    cheek_y = ROOT_CLEVIS_GAP / 2.0 + ROOT_CHEEK_THICK / 2.0
    _add_box(
        base,
        size=(0.024, ROOT_CHEEK_THICK, 0.030),
        xyz=(BASE_PIVOT_X - 0.005, cheek_y, BASE_PIVOT_Z),
        material="base_coat",
        name="left_clevis_cheek",
    )
    _add_box(
        base,
        size=(0.024, ROOT_CHEEK_THICK, 0.030),
        xyz=(BASE_PIVOT_X - 0.005, -cheek_y, BASE_PIVOT_Z),
        material="base_coat",
        name="right_clevis_cheek",
    )

    link1 = model.part("root_rocker")
    _add_y_cylinder(
        link1,
        radius=0.011,
        length=ROOT_CLEVIS_GAP,
        xyz=(0.0, 0.0, 0.0),
        material="link_root_finish",
        name="root_barrel",
    )
    _add_box(link1, size=(0.010, 0.012, 0.010), xyz=(0.012, 0.0, 0.010), material="link_root_finish", name="root_web")
    _add_box(link1, size=(0.030, 0.012, 0.016), xyz=(0.028, 0.0, 0.018), material="link_root_finish", name="neck")
    _add_box(link1, size=(0.110, 0.012, 0.014), xyz=(0.088, 0.0, 0.023), material="link_root_finish", name="main_beam")
    _add_box(link1, size=(0.040, 0.012, 0.010), xyz=(0.128, 0.0, 0.032), material="link_root_finish", name="top_step")
    fork_y = ROOT_FORK_GAP / 2.0 + FORK_CHEEK_THICK / 2.0
    _add_box(
        link1,
        size=(0.030, FORK_CHEEK_THICK, 0.020),
        xyz=(0.150, fork_y, LINK1_TO_LINK2_Z),
        material="link_root_finish",
        name="left_tip_cheek",
    )
    _add_box(
        link1,
        size=(0.030, FORK_CHEEK_THICK, 0.020),
        xyz=(0.150, -fork_y, LINK1_TO_LINK2_Z),
        material="link_root_finish",
        name="right_tip_cheek",
    )
    _add_box(link1, size=(0.018, 0.0025, 0.018), xyz=(0.141, 0.0065, LINK1_TO_LINK2_Z), material="link_root_finish", name="left_tip_rib")
    _add_box(link1, size=(0.018, 0.0025, 0.018), xyz=(0.141, -0.0065, LINK1_TO_LINK2_Z), material="link_root_finish", name="right_tip_rib")

    link2 = model.part("mid_link")
    _add_y_cylinder(
        link2,
        radius=0.0075,
        length=ROOT_FORK_GAP,
        xyz=(0.0, 0.0, 0.0),
        material="link_mid_finish",
        name="root_barrel",
    )
    _add_box(link2, size=(0.026, 0.010, 0.012), xyz=(0.018, 0.0, -0.008), material="link_mid_finish", name="neck")
    _add_box(link2, size=(0.092, 0.010, 0.014), xyz=(0.076, 0.0, -0.012), material="link_mid_finish", name="main_beam")
    _add_box(link2, size=(0.040, 0.010, 0.010), xyz=(0.112, 0.0, -0.020), material="link_mid_finish", name="drop_step")
    mid_fork_y = MID_FORK_GAP / 2.0 + FORK_CHEEK_THICK / 2.0
    _add_box(
        link2,
        size=(0.032, FORK_CHEEK_THICK, 0.018),
        xyz=(0.129, mid_fork_y, LINK2_TO_LINK3_Z),
        material="link_mid_finish",
        name="left_tip_cheek",
    )
    _add_box(
        link2,
        size=(0.032, FORK_CHEEK_THICK, 0.018),
        xyz=(0.129, -mid_fork_y, LINK2_TO_LINK3_Z),
        material="link_mid_finish",
        name="right_tip_cheek",
    )
    _add_box(link2, size=(0.018, 0.0025, 0.016), xyz=(0.121, 0.0055, LINK2_TO_LINK3_Z), material="link_mid_finish", name="left_tip_rib")
    _add_box(link2, size=(0.018, 0.0025, 0.016), xyz=(0.121, -0.0055, LINK2_TO_LINK3_Z), material="link_mid_finish", name="right_tip_rib")

    link3 = model.part("tip_link")
    _add_y_cylinder(
        link3,
        radius=0.006,
        length=MID_FORK_GAP,
        xyz=(0.0, 0.0, 0.0),
        material="link_tip_finish",
        name="root_barrel",
    )
    _add_box(link3, size=(0.022, 0.008, 0.010), xyz=(0.016, 0.0, 0.004), material="link_tip_finish", name="neck")
    _add_box(link3, size=(0.090, 0.008, 0.012), xyz=(0.070, 0.0, 0.008), material="link_tip_finish", name="arm")
    _add_box(link3, size=(0.034, 0.008, 0.012), xyz=(0.098, 0.0, 0.020), material="link_tip_finish", name="nose_pad")
    _add_y_cylinder(
        link3,
        radius=0.008,
        length=0.008,
        xyz=(0.120, 0.0, 0.024),
        material="link_tip_finish",
        name="rounded_tip",
    )

    model.articulation(
        "base_to_rocker",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(xyz=(BASE_PIVOT_X, 0.0, BASE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.95, upper=1.10),
    )
    model.articulation(
        "rocker_to_mid",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_TO_LINK2_X, 0.0, LINK1_TO_LINK2_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-1.35, upper=1.15),
    )
    model.articulation(
        "mid_to_tip",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(LINK2_TO_LINK3_X, 0.0, LINK2_TO_LINK3_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=2.6, lower=-1.00, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_foot")
    link1 = object_model.get_part("root_rocker")
    link2 = object_model.get_part("mid_link")
    link3 = object_model.get_part("tip_link")
    joint1 = object_model.get_articulation("base_to_rocker")
    joint2 = object_model.get_articulation("rocker_to_mid")
    joint3 = object_model.get_articulation("mid_to_tip")

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

    ctx.expect_contact(base, link1, name="base clevis captures rocker")
    ctx.expect_contact(link1, link2, name="root link fork captures middle link")
    ctx.expect_contact(link2, link3, name="middle link fork captures tip link")

    axes_are_coplanar = all(
        joint.articulation_type == ArticulationType.REVOLUTE
        and joint.axis == (0.0, -1.0, 0.0)
        and abs(joint.origin.xyz[1]) < 1e-9
        for joint in (joint1, joint2, joint3)
    )
    ctx.check(
        "three revolute joints share one motion plane",
        axes_are_coplanar,
        details=(
            f"axes={[joint1.axis, joint2.axis, joint3.axis]}, "
            f"joint_y={[joint1.origin.xyz[1], joint2.origin.xyz[1], joint3.origin.xyz[1]]}"
        ),
    )

    link1_aabb = ctx.part_world_aabb(link1)
    link2_aabb = ctx.part_world_aabb(link2)
    link3_aabb = ctx.part_world_aabb(link3)
    link_widths_step = (
        link1_aabb is not None
        and link2_aabb is not None
        and link3_aabb is not None
        and (link1_aabb[1][1] - link1_aabb[0][1]) > (link2_aabb[1][1] - link2_aabb[0][1]) > (link3_aabb[1][1] - link3_aabb[0][1])
    )
    ctx.check(
        "link widths step down from root to tip",
        link_widths_step,
        details=(
            "Expected decreasing Y widths from root to tip to avoid a repeated-bar chain."
        ),
    )

    stepped_offsets = joint2.origin.xyz[2] > 0.015 and joint3.origin.xyz[2] < -0.015
    ctx.check(
        "successive links use alternating stepped offsets",
        stepped_offsets,
        details=f"joint z offsets are {[joint2.origin.xyz[2], joint3.origin.xyz[2]]}",
    )

    folded_tip = None
    raised_tip = None
    with ctx.pose(base_to_rocker=-0.20, rocker_to_mid=-0.65, mid_to_tip=-0.40):
        folded_tip = ctx.part_world_position(link3)
    with ctx.pose(base_to_rocker=0.80, rocker_to_mid=0.55, mid_to_tip=0.35):
        raised_tip = ctx.part_world_position(link3)
    ctx.check(
        "serial chain folds upward within one plane",
        folded_tip is not None
        and raised_tip is not None
        and raised_tip[2] > folded_tip[2] + 0.08
        and abs(raised_tip[1] - folded_tip[1]) < 0.002,
        details=f"folded_tip={folded_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
