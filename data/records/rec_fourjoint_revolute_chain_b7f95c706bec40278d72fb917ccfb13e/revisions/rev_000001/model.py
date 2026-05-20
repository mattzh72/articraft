from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FOOT_X = 0.220
FOOT_Y = 0.120
FOOT_Z = 0.018
TOWER_X = 0.054
TOWER_Y = 0.024
TOWER_Z = 0.064

ARM_W = 0.018
ARM_T = 0.010
TONGUE_W = 0.018
TONGUE_Z = 0.022
SHOULDER_W = 0.024
SHOULDER_Z = 0.026
JOINT_GAP_Y = 0.010
PLATE_T = 0.004
HINGE_W = 0.032
HINGE_H = 0.028
HINGE_BRIDGE_H = 0.010
HINGE_BRIDGE_W = 0.024
HINGE_OUTER_Y = JOINT_GAP_Y + 2.0 * PLATE_T

LINK1_LEN = 0.135
LINK2_LEN = 0.125
LINK3_LEN = 0.115
END_LINK_LEN = 0.082
END_TAB_L = 0.026
END_TAB_W = 0.036
END_TAB_T = 0.014

BASE_JOINT_Z = FOOT_Z + TOWER_Z + 0.014


def _add_box(part, size, center_xyz, material, name: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center_xyz),
        material=material,
        name=name,
    )


def _add_hinge_block(part, joint_z: float, material, prefix: str) -> None:
    y_offset = JOINT_GAP_Y / 2.0 + PLATE_T / 2.0
    plate_center_z = joint_z - HINGE_BRIDGE_H / 2.0 + HINGE_H / 2.0
    part.visual(
        Box((HINGE_W, PLATE_T, HINGE_H)),
        origin=Origin(xyz=(0.0, y_offset, plate_center_z)),
        material=material,
        name=f"{prefix}_plate_pos",
    )
    part.visual(
        Box((HINGE_W, PLATE_T, HINGE_H)),
        origin=Origin(xyz=(0.0, -y_offset, plate_center_z)),
        material=material,
        name=f"{prefix}_plate_neg",
    )
    part.visual(
        Box((HINGE_BRIDGE_W, HINGE_OUTER_Y, HINGE_BRIDGE_H)),
        origin=Origin(xyz=(0.0, 0.0, joint_z - HINGE_BRIDGE_H / 2.0)),
        material=material,
        name=f"{prefix}_bridge",
    )


def _add_link_geometry(part, link_len: float, material, *, end_tab: bool = False) -> None:
    _add_box(
        part,
        (TONGUE_W, JOINT_GAP_Y, TONGUE_Z),
        (0.0, 0.0, TONGUE_Z / 2.0),
        material,
        f"{part.name}_tongue",
    )
    _add_box(
        part,
        (SHOULDER_W, ARM_T, SHOULDER_Z),
        (0.0, 0.0, SHOULDER_Z / 2.0),
        material,
        f"{part.name}_shoulder",
    )

    beam_bottom = SHOULDER_Z
    beam_top = link_len if end_tab else link_len - HINGE_BRIDGE_H
    beam_h = max(beam_top - beam_bottom, 0.020)
    _add_box(
        part,
        (ARM_W, ARM_T, beam_h),
        (0.0, 0.0, beam_bottom + beam_h / 2.0),
        material,
        f"{part.name}_beam",
    )

    if end_tab:
        _add_box(
            part,
            (END_TAB_W, END_TAB_T, END_TAB_L),
            (0.0, 0.0, END_LINK_LEN + END_TAB_L / 2.0),
            material,
            "end_tab",
        )
    else:
        _add_hinge_block(part, link_len, material, f"{part.name}_hinge")


def _add_base_geometry(base, base_material, plate_material) -> None:
    _add_box(
        base,
        (FOOT_X, FOOT_Y, FOOT_Z),
        (0.0, 0.0, FOOT_Z / 2.0),
        base_material,
        "base_footpad",
    )
    _add_box(
        base,
        (0.090, 0.050, 0.010),
        (0.0, 0.0, FOOT_Z + 0.005),
        base_material,
        "base_upper_pad",
    )
    _add_box(
        base,
        (TOWER_X, TOWER_Y, TOWER_Z),
        (0.0, 0.0, FOOT_Z + TOWER_Z / 2.0),
        base_material,
        "base_tower",
    )
    riser_h = BASE_JOINT_Z - HINGE_BRIDGE_H - (FOOT_Z + TOWER_Z)
    _add_box(
        base,
        (HINGE_BRIDGE_W, TOWER_Y, riser_h),
        (0.0, 0.0, FOOT_Z + TOWER_Z + riser_h / 2.0),
        base_material,
        "base_hinge_riser",
    )
    _add_hinge_block(base, BASE_JOINT_Z, plate_material, "base_hinge")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_top_revolute_chain")

    base_mat = model.material("base_mat", color=(0.18, 0.19, 0.21, 1.0))
    link_mat = model.material("link_mat", color=(0.73, 0.75, 0.78, 1.0))
    end_mat = model.material("end_mat", color=(0.58, 0.61, 0.66, 1.0))
    plate_mat = model.material("plate_mat", color=(0.84, 0.86, 0.89, 1.0))

    base = model.part("base_foot")
    _add_base_geometry(base, base_mat, plate_mat)

    link1 = model.part("link_1")
    _add_link_geometry(link1, LINK1_LEN, link_mat)
    _add_hinge_block(link1, LINK1_LEN, plate_mat, "link_1_hinge")

    link2 = model.part("link_2")
    _add_link_geometry(link2, LINK2_LEN, link_mat)
    _add_hinge_block(link2, LINK2_LEN, plate_mat, "link_2_hinge")

    link3 = model.part("link_3")
    _add_link_geometry(link3, LINK3_LEN, link_mat)
    _add_hinge_block(link3, LINK3_LEN, plate_mat, "link_3_hinge")

    link4 = model.part("link_4")
    _add_link_geometry(link4, END_LINK_LEN, end_mat, end_tab=True)

    joint_limits = MotionLimits(
        effort=6.0,
        velocity=2.5,
        lower=0.0,
        upper=1.20,
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, BASE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(0.0, 0.0, LINK1_LEN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(0.0, 0.0, LINK2_LEN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )
    model.articulation(
        "link_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent=link3,
        child=link4,
        origin=Origin(xyz=(0.0, 0.0, LINK3_LEN)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=joint_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base_foot")
    link1 = object_model.get_part("link_1")
    link2 = object_model.get_part("link_2")
    link3 = object_model.get_part("link_3")
    link4 = object_model.get_part("link_4")

    joints = [
        object_model.get_articulation("base_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_link_3"),
        object_model.get_articulation("link_3_to_link_4"),
    ]

    for part in (base, link1, link2, link3, link4):
        ctx.check(f"part_present::{part.name}", part is not None)

    for joint in joints:
        ctx.check(
            f"joint_axis_aligned::{joint.name}",
            joint.axis == (0.0, 1.0, 0.0) and joint.origin.xyz[0] == 0.0 and joint.origin.xyz[1] == 0.0,
            details=f"axis={joint.axis}, origin={joint.origin.xyz}",
        )

    ctx.expect_contact(base, link1, contact_tol=5e-4, name="base_joint_contact")
    ctx.expect_contact(link1, link2, contact_tol=5e-4, name="joint_1_contact")
    ctx.expect_contact(link2, link3, contact_tol=5e-4, name="joint_2_contact")
    ctx.expect_contact(link3, link4, contact_tol=5e-4, name="joint_3_contact")

    ctx.expect_origin_gap(
        link2,
        link1,
        axis="z",
        min_gap=LINK1_LEN - 0.002,
        max_gap=LINK1_LEN + 0.002,
        name="link_1_to_link_2_pitch",
    )
    ctx.expect_origin_gap(
        link3,
        link2,
        axis="z",
        min_gap=LINK2_LEN - 0.002,
        max_gap=LINK2_LEN + 0.002,
        name="link_2_to_link_3_pitch",
    )
    ctx.expect_origin_gap(
        link4,
        link3,
        axis="z",
        min_gap=LINK3_LEN - 0.002,
        max_gap=LINK3_LEN + 0.002,
        name="link_3_to_link_4_pitch",
    )

    with ctx.pose(
        {
            joints[0]: 0.55,
            joints[1]: 0.50,
            joints[2]: 0.42,
            joints[3]: 0.25,
        }
    ):
        link1_pos = ctx.part_world_position(link1)
        link4_pos = ctx.part_world_position(link4)
        ctx.check(
            "forward_fold_pose",
            link1_pos is not None
            and link4_pos is not None
            and link4_pos[0] > link1_pos[0] + 0.12
            and link4_pos[2] > 0.18,
            details=f"link1={link1_pos}, link4={link4_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
