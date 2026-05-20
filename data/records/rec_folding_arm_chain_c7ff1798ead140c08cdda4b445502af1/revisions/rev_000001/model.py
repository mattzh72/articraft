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


JOINT_GAP = 0.064
CHEEK_THICKNESS = 0.012
OUTER_CLEVIS_WIDTH = JOINT_GAP + 2.0 * CHEEK_THICKNESS

LINK_1_LENGTH = 0.420
LINK_2_LENGTH = 0.340
LINK_3_LENGTH = 0.240


def _add_box_span(
    part,
    x0: float,
    x1: float,
    *,
    width: float,
    depth: float,
    y: float = 0.0,
    z: float = 0.0,
    name: str,
    material: str,
) -> None:
    part.visual(
        Box((x1 - x0, width, depth)),
        origin=Origin(xyz=((x0 + x1) / 2.0, y, z)),
        material=material,
        name=name,
    )


def _add_y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    name: str,
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_forked_link(
    part,
    *,
    length: float,
    prox_boss_radius: float,
    shoulder_x0: float,
    shoulder_x1: float,
    shoulder_width: float,
    shoulder_depth: float,
    beam_x0: float,
    beam_x1: float,
    beam_width: float,
    beam_depth: float,
    bridge_x0: float,
    bridge_x1: float,
    fork_depth: float,
    cheek_x0: float,
    cheek_x1: float,
    collar_radius: float,
    material: str,
    prefix: str,
) -> None:
    cheek_y = JOINT_GAP / 2.0 + CHEEK_THICKNESS / 2.0

    _add_y_cylinder(
        part,
        radius=prox_boss_radius,
        length=JOINT_GAP,
        name=f"{prefix}_prox_boss",
        material=material,
    )
    _add_box_span(
        part,
        shoulder_x0,
        shoulder_x1,
        width=shoulder_width,
        depth=shoulder_depth,
        name=f"{prefix}_shoulder_block",
        material=material,
    )
    _add_box_span(
        part,
        beam_x0,
        beam_x1,
        width=beam_width,
        depth=beam_depth,
        name=f"{prefix}_beam",
        material=material,
    )
    _add_box_span(
        part,
        bridge_x0,
        bridge_x1,
        width=OUTER_CLEVIS_WIDTH,
        depth=fork_depth,
        name=f"{prefix}_fork_bridge",
        material=material,
    )
    _add_box_span(
        part,
        cheek_x0,
        cheek_x1,
        width=CHEEK_THICKNESS,
        depth=fork_depth,
        y=cheek_y,
        name=f"{prefix}_left_cheek",
        material=material,
    )
    _add_box_span(
        part,
        cheek_x0,
        cheek_x1,
        width=CHEEK_THICKNESS,
        depth=fork_depth,
        y=-cheek_y,
        name=f"{prefix}_right_cheek",
        material=material,
    )
    _add_y_cylinder(
        part,
        radius=collar_radius,
        length=CHEEK_THICKNESS,
        x=length,
        y=cheek_y,
        name=f"{prefix}_left_collar",
        material=material,
    )
    _add_y_cylinder(
        part,
        radius=collar_radius,
        length=CHEEK_THICKNESS,
        x=length,
        y=-cheek_y,
        name=f"{prefix}_right_collar",
        material=material,
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_folding_arm_chain")

    model.material("base_charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("arm_blue", rgba=(0.35, 0.45, 0.57, 1.0))
    model.material("arm_gray", rgba=(0.58, 0.62, 0.67, 1.0))
    model.material("pad_black", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    cheek_y = JOINT_GAP / 2.0 + CHEEK_THICKNESS / 2.0
    _add_box_span(base, -0.180, 0.060, width=0.180, depth=0.024, z=-0.144, name="base_foot", material="base_charcoal")
    _add_box_span(base, -0.126, -0.036, width=0.118, depth=0.166, z=-0.067, name="base_tower", material="base_charcoal")
    _add_box_span(base, -0.094, -0.032, width=JOINT_GAP, depth=0.104, z=-0.020, name="base_spine", material="base_charcoal")
    _add_box_span(base, -0.092, -0.024, width=OUTER_CLEVIS_WIDTH, depth=0.050, z=-0.082, name="base_bridge", material="base_charcoal")
    _add_box_span(base, -0.050, 0.000, width=CHEEK_THICKNESS, depth=0.172, y=cheek_y, z=-0.002, name="base_left_cheek", material="base_charcoal")
    _add_box_span(base, -0.050, 0.000, width=CHEEK_THICKNESS, depth=0.172, y=-cheek_y, z=-0.002, name="base_right_cheek", material="base_charcoal")
    _add_y_cylinder(base, radius=0.040, length=CHEEK_THICKNESS, x=0.0, y=cheek_y, name="base_left_boss", material="base_charcoal")
    _add_y_cylinder(base, radius=0.040, length=CHEEK_THICKNESS, x=0.0, y=-cheek_y, name="base_right_boss", material="base_charcoal")

    link_1 = model.part("link_1")
    _add_forked_link(
        link_1,
        length=LINK_1_LENGTH,
        prox_boss_radius=0.028,
        shoulder_x0=0.010,
        shoulder_x1=0.066,
        shoulder_width=0.054,
        shoulder_depth=0.074,
        beam_x0=0.058,
        beam_x1=0.344,
        beam_width=0.060,
        beam_depth=0.112,
        bridge_x0=0.330,
        bridge_x1=0.386,
        fork_depth=0.052,
        cheek_x0=0.352,
        cheek_x1=LINK_1_LENGTH,
        collar_radius=0.030,
        material="arm_blue",
        prefix="link1",
    )

    link_2 = model.part("link_2")
    _add_forked_link(
        link_2,
        length=LINK_2_LENGTH,
        prox_boss_radius=0.025,
        shoulder_x0=0.010,
        shoulder_x1=0.060,
        shoulder_width=0.052,
        shoulder_depth=0.068,
        beam_x0=0.054,
        beam_x1=0.274,
        beam_width=0.056,
        beam_depth=0.100,
        bridge_x0=0.260,
        bridge_x1=0.306,
        fork_depth=0.046,
        cheek_x0=0.280,
        cheek_x1=LINK_2_LENGTH,
        collar_radius=0.027,
        material="arm_gray",
        prefix="link2",
    )

    link_3 = model.part("link_3")
    _add_y_cylinder(link_3, radius=0.022, length=JOINT_GAP, name="link3_prox_boss", material="arm_blue")
    _add_box_span(link_3, 0.010, 0.052, width=0.052, depth=0.066, name="link3_shoulder", material="arm_blue")
    _add_box_span(link_3, 0.048, 0.204, width=0.048, depth=0.084, name="link3_beam", material="arm_blue")
    _add_box_span(link_3, 0.198, 0.247, width=0.040, depth=0.058, name="link3_nose", material="arm_blue")
    _add_box_span(link_3, 0.247, 0.261, width=0.050, depth=0.060, name="end_pad", material="pad_black")

    shoulder = model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=1.20, effort=260.0, velocity=1.0),
    )
    elbow = model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.55, upper=1.20, effort=180.0, velocity=1.2),
    )
    wrist = model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.20, upper=1.10, effort=120.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    shoulder = object_model.get_articulation("base_to_link_1")
    elbow = object_model.get_articulation("link_1_to_link_2")
    wrist = object_model.get_articulation("link_2_to_link_3")
    end_pad = link_3.get_visual("end_pad")

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

    ctx.expect_contact(base, link_1, name="base_shoulder_contact")
    ctx.expect_contact(link_1, link_2, name="elbow_contact")
    ctx.expect_contact(link_2, link_3, name="wrist_contact")

    ctx.check(
        "parallel_pitch_axes",
        shoulder.axis == elbow.axis == wrist.axis == (0.0, -1.0, 0.0),
        details=f"axes were {shoulder.axis}, {elbow.axis}, {wrist.axis}",
    )

    with ctx.pose(base_to_link_1=0.0, link_1_to_link_2=0.0, link_2_to_link_3=0.0):
        link_2_rest = ctx.part_world_position(link_2)
        link_3_rest = ctx.part_world_position(link_3)
        end_pad_rest = _aabb_center(ctx.part_element_world_aabb(link_3, elem=end_pad))
        rest_ok = (
            link_2_rest is not None
            and link_3_rest is not None
            and end_pad_rest is not None
            and link_2_rest[0] > 0.30
            and link_3_rest[0] > link_2_rest[0] + 0.20
            and end_pad_rest[0] > link_3_rest[0]
        )
        ctx.check(
            "rest_pose_serial_layout",
            rest_ok,
            details=f"link_2={link_2_rest}, link_3={link_3_rest}, end_pad={end_pad_rest}",
        )

    with ctx.pose(base_to_link_1=0.55, link_1_to_link_2=0.0, link_2_to_link_3=0.0):
        raised_link_2 = ctx.part_world_position(link_2)
    with ctx.pose(base_to_link_1=0.0, link_1_to_link_2=0.0, link_2_to_link_3=0.55):
        wrist_raised_pad = _aabb_center(ctx.part_element_world_aabb(link_3, elem=end_pad))

    shoulder_ok = (
        link_2_rest is not None
        and raised_link_2 is not None
        and raised_link_2[2] > link_2_rest[2] + 0.08
    )
    ctx.check(
        "positive_shoulder_raises_chain",
        shoulder_ok,
        details=f"rest={link_2_rest}, raised={raised_link_2}",
    )

    wrist_ok = (
        end_pad_rest is not None
        and wrist_raised_pad is not None
        and wrist_raised_pad[2] > end_pad_rest[2] + 0.03
    )
    ctx.check(
        "positive_wrist_raises_end_pad",
        wrist_ok,
        details=f"rest={end_pad_rest}, raised={wrist_raised_pad}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
