from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LINK_LENGTHS = (0.26, 0.32, 0.27, 0.33, 0.28)
LINK_THICKNESS = 0.012
LINK_HEIGHT = 0.018
JOINT_BOSS_RADIUS = 0.015
JOINT_FACE_INSET = 0.022
PAD_SIZE = 0.055
MOUNT_BLOCK = (0.045, 0.018, 0.024)

ROOT_BASE = (0.22, 0.14, 0.028)
ROOT_RISER = (0.060, 0.072, 0.070)
ROOT_CHEEK = (0.020, 0.030, MOUNT_BLOCK[2])
LINK1_ORIGIN_Z = ROOT_BASE[2] + ROOT_RISER[2] + ROOT_CHEEK[2] / 2.0

LINK_NAMES = ("link1", "link2", "link3", "link4", "link5")
REVOLUTE_NAMES = (
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
)
POSE_TOL = 1e-6


def _plate(length: float, height: float, *, x0: float = 0.0, width: float = LINK_THICKNESS):
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(False, True, True))
        .translate((x0, 0.0, 0.0))
    )


def _y_boss(x_pos: float):
    return (
        cq.Workplane("XZ")
        .center(x_pos, 0.0)
        .circle(JOINT_BOSS_RADIUS)
        .extrude(LINK_THICKNESS / 2.0, both=True)
    )


def _make_root_foot():
    base = cq.Workplane("XY").box(*ROOT_BASE, centered=(False, True, False)).translate(
        (-ROOT_BASE[0], 0.0, 0.0)
    )
    riser = cq.Workplane("XY").box(*ROOT_RISER, centered=(False, True, False)).translate(
        (-ROOT_RISER[0], 0.0, ROOT_BASE[2])
    )
    cheek = cq.Workplane("XY").box(
        *ROOT_CHEEK,
        centered=(False, True, True),
    ).translate((-ROOT_CHEEK[0], 0.0, LINK1_ORIGIN_Z))
    rear_gusset = (
        cq.Workplane("XZ")
        .moveTo(-0.135, ROOT_BASE[2])
        .lineTo(-0.060, ROOT_BASE[2])
        .lineTo(-0.020, ROOT_BASE[2] + ROOT_RISER[2] * 0.88)
        .lineTo(-0.020, LINK1_ORIGIN_Z + ROOT_CHEEK[2] / 2.0)
        .lineTo(-0.070, LINK1_ORIGIN_Z + ROOT_CHEEK[2] / 2.0)
        .close()
        .extrude(0.018, both=True)
    )
    return base.union(riser).union(cheek).union(rear_gusset)


def _make_link_shape(
    length: float,
    *,
    has_proximal_boss: bool,
    has_distal_boss: bool,
    has_mount_block: bool,
):
    shape = _plate(length, LINK_HEIGHT, x0=0.0)

    if has_mount_block:
        mount_block = _plate(
            MOUNT_BLOCK[0],
            MOUNT_BLOCK[2],
            x0=0.0,
            width=MOUNT_BLOCK[1],
        )
        shape = shape.union(mount_block)

    if has_proximal_boss:
        shape = shape.union(_y_boss(JOINT_FACE_INSET))
    if has_distal_boss:
        shape = shape.union(_y_boss(length - JOINT_FACE_INSET))

    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serial_linkage")

    foot_material = model.material("foot_powdercoat", color=(0.16, 0.17, 0.18))
    link_light = model.material("link_light", color=(0.72, 0.74, 0.77))
    link_dark = model.material("link_dark", color=(0.62, 0.65, 0.69))
    pad_material = model.material("pad_dark", color=(0.34, 0.36, 0.40))

    root_foot = model.part("root_foot")
    root_foot.visual(
        mesh_from_cadquery(_make_root_foot(), "root_foot"),
        material=foot_material,
        name="body",
    )

    link1 = model.part("link1")
    link1.visual(
        mesh_from_cadquery(
            _make_link_shape(
                LINK_LENGTHS[0],
                has_proximal_boss=False,
                has_distal_boss=True,
                has_mount_block=True,
            ),
            "link1_body",
        ),
        material=link_light,
        name="body",
    )

    link2 = model.part("link2")
    link2.visual(
        mesh_from_cadquery(
            _make_link_shape(
                LINK_LENGTHS[1],
                has_proximal_boss=True,
                has_distal_boss=True,
                has_mount_block=False,
            ),
            "link2_body",
        ),
        material=link_dark,
        name="body",
    )

    link3 = model.part("link3")
    link3.visual(
        mesh_from_cadquery(
            _make_link_shape(
                LINK_LENGTHS[2],
                has_proximal_boss=True,
                has_distal_boss=True,
                has_mount_block=False,
            ),
            "link3_body",
        ),
        material=link_light,
        name="body",
    )

    link4 = model.part("link4")
    link4.visual(
        mesh_from_cadquery(
            _make_link_shape(
                LINK_LENGTHS[3],
                has_proximal_boss=True,
                has_distal_boss=True,
                has_mount_block=False,
            ),
            "link4_body",
        ),
        material=link_dark,
        name="body",
    )

    link5 = model.part("link5")
    link5.visual(
        mesh_from_cadquery(
            _make_link_shape(
                LINK_LENGTHS[4],
                has_proximal_boss=True,
                has_distal_boss=False,
                has_mount_block=False,
            ),
            "link5_body",
        ),
        material=link_light,
        name="body",
    )
    link5.visual(
        Box((PAD_SIZE, LINK_THICKNESS, PAD_SIZE)),
        origin=Origin(
            xyz=(
                LINK_LENGTHS[4] + PAD_SIZE / 2.0,
                0.0,
                0.0,
            )
        ),
        material=pad_material,
        name="end_pad",
    )

    model.articulation(
        "root_to_link1",
        ArticulationType.FIXED,
        parent=root_foot,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, LINK1_ORIGIN_Z)),
    )

    revolute_limits = MotionLimits(
        effort=8.0,
        velocity=1.8,
        lower=-1.35,
        upper=1.35,
    )

    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK_LENGTHS[0], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(LINK_LENGTHS[1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link3_to_link4",
        ArticulationType.REVOLUTE,
        parent=link3,
        child=link4,
        origin=Origin(xyz=(LINK_LENGTHS[2], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link4_to_link5",
        ArticulationType.REVOLUTE,
        parent=link4,
        child=link5,
        origin=Origin(xyz=(LINK_LENGTHS[3], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        lower, upper = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))

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

    expected_parts = {"root_foot", *LINK_NAMES}
    ctx.check(
        "expected_parts_present",
        {part.name for part in object_model.parts} == expected_parts,
        details=f"found parts: {[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "expected_articulations_present",
        {joint.name for joint in object_model.articulations}
        == {"root_to_link1", *REVOLUTE_NAMES},
        details=f"found articulations: {[joint.name for joint in object_model.articulations]}",
    )

    foot = object_model.get_part("root_foot")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")
    link4 = object_model.get_part("link4")
    link5 = object_model.get_part("link5")

    root_to_link1 = object_model.get_articulation("root_to_link1")
    link1_to_link2 = object_model.get_articulation("link1_to_link2")
    link2_to_link3 = object_model.get_articulation("link2_to_link3")
    link3_to_link4 = object_model.get_articulation("link3_to_link4")
    link4_to_link5 = object_model.get_articulation("link4_to_link5")
    revolute_joints = (
        link1_to_link2,
        link2_to_link3,
        link3_to_link4,
        link4_to_link5,
    )

    ctx.check(
        "root_mount_is_fixed",
        root_to_link1.articulation_type == ArticulationType.FIXED,
        details=f"root mount type is {root_to_link1.articulation_type}",
    )
    ctx.check(
        "four_revolute_joints",
        len(revolute_joints) == 4
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in revolute_joints),
        details=f"joint types: {[joint.articulation_type for joint in revolute_joints]}",
    )
    ctx.check(
        "parallel_joint_axes",
        all(tuple(joint.axis) == (0.0, 1.0, 0.0) for joint in revolute_joints),
        details=f"axes: {[joint.axis for joint in revolute_joints]}",
    )
    ctx.check(
        "alternating_link_lengths",
        LINK_LENGTHS[0] < LINK_LENGTHS[1]
        and LINK_LENGTHS[1] > LINK_LENGTHS[2]
        and LINK_LENGTHS[2] < LINK_LENGTHS[3]
        and LINK_LENGTHS[3] > LINK_LENGTHS[4]
        and min(abs(a - b) for a, b in zip(LINK_LENGTHS, LINK_LENGTHS[1:])) >= 0.04,
        details=f"link lengths: {LINK_LENGTHS}",
    )
    ctx.check(
        "end_pad_visual_present",
        any(visual.name == "end_pad" for visual in link5.visuals),
        details=f"link5 visuals: {[visual.name for visual in link5.visuals]}",
    )

    ctx.expect_contact(link1, foot, name="foot_to_link1_contact")
    ctx.expect_contact(link1, link2, name="link1_to_link2_contact")
    ctx.expect_contact(link2, link3, name="link2_to_link3_contact")
    ctx.expect_contact(link3, link4, name="link3_to_link4_contact")
    ctx.expect_contact(link4, link5, name="link4_to_link5_contact")

    ctx.expect_origin_gap(
        link1,
        foot,
        axis="z",
        min_gap=LINK1_ORIGIN_Z - POSE_TOL,
        max_gap=LINK1_ORIGIN_Z + POSE_TOL,
        name="link1_mount_height",
    )
    ctx.expect_origin_gap(
        link2,
        link1,
        axis="x",
        min_gap=LINK_LENGTHS[0] - POSE_TOL,
        max_gap=LINK_LENGTHS[0] + POSE_TOL,
        name="link1_spacing_to_link2",
    )
    ctx.expect_origin_gap(
        link3,
        link2,
        axis="x",
        min_gap=LINK_LENGTHS[1] - POSE_TOL,
        max_gap=LINK_LENGTHS[1] + POSE_TOL,
        name="link2_spacing_to_link3",
    )
    ctx.expect_origin_gap(
        link4,
        link3,
        axis="x",
        min_gap=LINK_LENGTHS[2] - POSE_TOL,
        max_gap=LINK_LENGTHS[2] + POSE_TOL,
        name="link3_spacing_to_link4",
    )
    ctx.expect_origin_gap(
        link5,
        link4,
        axis="x",
        min_gap=LINK_LENGTHS[3] - POSE_TOL,
        max_gap=LINK_LENGTHS[3] + POSE_TOL,
        name="link4_spacing_to_link5",
    )
    ctx.expect_origin_distance(
        link2,
        link1,
        axes="yz",
        max_dist=POSE_TOL,
        name="link2_aligned_in_plane_with_link1",
    )
    ctx.expect_origin_distance(
        link3,
        link2,
        axes="yz",
        max_dist=POSE_TOL,
        name="link3_aligned_in_plane_with_link2",
    )
    ctx.expect_origin_distance(
        link4,
        link3,
        axes="yz",
        max_dist=POSE_TOL,
        name="link4_aligned_in_plane_with_link3",
    )
    ctx.expect_origin_distance(
        link5,
        link4,
        axes="yz",
        max_dist=POSE_TOL,
        name="link5_aligned_in_plane_with_link4",
    )

    rest_pad_aabb = ctx.part_element_world_aabb(link5, elem="end_pad")
    if rest_pad_aabb is None:
        ctx.fail("rest_end_pad_aabb", "Could not resolve the end pad AABB in the rest pose.")
    else:
        rest_pad_center = aabb_center(rest_pad_aabb)
        with ctx.pose(
            {
                link1_to_link2: -0.55,
                link2_to_link3: -0.45,
                link3_to_link4: -0.40,
                link4_to_link5: -0.35,
            }
        ):
            folded_pad_aabb = ctx.part_element_world_aabb(link5, elem="end_pad")
            if folded_pad_aabb is None:
                ctx.fail("folded_end_pad_aabb", "Could not resolve the end pad AABB in a folded pose.")
            else:
                folded_pad_center = aabb_center(folded_pad_aabb)
                ctx.check(
                    "folded_pad_lifts_up",
                    folded_pad_center[2] > rest_pad_center[2] + 0.05,
                    details=(
                        f"rest pad center={rest_pad_center}, "
                        f"folded pad center={folded_pad_center}"
                    ),
                )
                ctx.check(
                    "folded_pad_reduces_horizontal_reach",
                    folded_pad_center[0] < rest_pad_center[0] - 0.20,
                    details=(
                        f"rest pad center={rest_pad_center}, "
                        f"folded pad center={folded_pad_center}"
                    ),
                )
                ctx.check(
                    "folded_pad_stays_in_parallel_axis_stack",
                    abs(folded_pad_center[1] - rest_pad_center[1]) <= POSE_TOL,
                    details=(
                        f"rest pad center={rest_pad_center}, "
                        f"folded pad center={folded_pad_center}"
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
