from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.008
BODY_H = 0.018
HUB_R = 0.011
PIN_R = 0.0042
CHEEK_T = 0.004
YOKE_H = 0.024
FORK_LEN = 0.026

LINK_1_LEN = 0.180
LINK_2_LEN = 0.160
LINK_3_LEN = 0.140


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x_pos, y_pos, z_pos = center
    return cq.Workplane(
        obj=cq.Solid.makeCylinder(
            radius,
            length,
            cq.Vector(x_pos, y_pos - length / 2.0, z_pos),
            cq.Vector(0.0, 1.0, 0.0),
        )
    )


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x_pos, y_pos, z_pos = center
    return cq.Workplane(
        obj=cq.Solid.makeCylinder(
            radius,
            length,
            cq.Vector(x_pos - length / 2.0, y_pos, z_pos),
            cq.Vector(1.0, 0.0, 0.0),
        )
    )


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _link_with_distal_yoke(length: float) -> cq.Workplane:
    fork_start = length - FORK_LEN
    cheek_y = BODY_W / 2.0 + CHEEK_T / 2.0

    shape = _box((fork_start, BODY_W, BODY_H), (fork_start / 2.0, 0.0, 0.0))
    shape = shape.union(_cyl_y(HUB_R, BODY_W, (0.0, 0.0, 0.0)))

    for sign in (-1.0, 1.0):
        y_pos = sign * cheek_y
        shape = shape.union(
            _box((FORK_LEN, CHEEK_T, YOKE_H), (fork_start + FORK_LEN / 2.0, y_pos, 0.0))
        )
        shape = shape.union(_cyl_y(HUB_R, CHEEK_T, (length, y_pos, 0.0)))
        shape = shape.cut(_cyl_y(PIN_R, CHEEK_T + 0.002, (length, y_pos, 0.0)))

    shape = shape.cut(_cyl_y(PIN_R, BODY_W + 0.002, (0.0, 0.0, 0.0)))
    return shape


def _terminal_link(length: float) -> cq.Workplane:
    beam_len = length - 0.016
    end_cap_r = 0.010

    shape = _box((beam_len, BODY_W, BODY_H), (beam_len / 2.0, 0.0, 0.0))
    shape = shape.union(_cyl_y(HUB_R, BODY_W, (0.0, 0.0, 0.0)))
    shape = shape.union(_box((0.020, BODY_W, BODY_H), (length - 0.010, 0.0, 0.0)))
    shape = shape.union(_cyl_y(end_cap_r, BODY_W, (length, 0.0, 0.0)))
    shape = shape.cut(_cyl_y(PIN_R, BODY_W + 0.002, (0.0, 0.0, 0.0)))
    shape = shape.cut(_cyl_y(0.004, BODY_W + 0.002, (length, 0.0, 0.0)))
    return shape


def _side_bracket() -> cq.Workplane:
    cheek_y = BODY_W / 2.0 + CHEEK_T / 2.0
    plate_center_x = -0.045
    plate_t = 0.010
    plate_w = 0.060
    plate_h = 0.120

    shape = _box((plate_t, plate_w, plate_h), (plate_center_x, 0.0, 0.0))

    for sign in (-1.0, 1.0):
        y_pos = sign * cheek_y
        shape = shape.union(_box((0.040, CHEEK_T, 0.030), (-0.020, y_pos, 0.0)))
        shape = shape.union(_cyl_y(HUB_R, CHEEK_T, (0.0, y_pos, 0.0)))
        shape = shape.cut(_cyl_y(PIN_R, CHEEK_T + 0.002, (0.0, y_pos, 0.0)))

    for z_pos in (-0.034, 0.034):
        shape = shape.cut(_cyl_x(0.0045, plate_t + 0.004, (plate_center_x, 0.0, z_pos)))

    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_chain")

    model.material("bracket_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("link_dark", rgba=(0.60, 0.63, 0.67, 1.0))
    model.material("link_light", rgba=(0.72, 0.75, 0.79, 1.0))
    model.material("link_mid", rgba=(0.66, 0.69, 0.73, 1.0))

    side_bracket = model.part("side_bracket")
    side_bracket.visual(
        mesh_from_cadquery(_side_bracket(), "side_bracket"),
        material="bracket_steel",
        name="bracket_mesh",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_link_with_distal_yoke(LINK_1_LEN), "link_1"),
        material="link_light",
        name="link_1_mesh",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_link_with_distal_yoke(LINK_2_LEN), "link_2"),
        material="link_mid",
        name="link_2_mesh",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(_terminal_link(LINK_3_LEN), "link_3"),
        material="link_dark",
        name="link_3_mesh",
    )

    model.articulation(
        "bracket_to_link_1",
        ArticulationType.REVOLUTE,
        parent=side_bracket,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.2, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.4, lower=-2.35, upper=2.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_bracket = object_model.get_part("side_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    joint_1 = object_model.get_articulation("bracket_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")
    joint_3 = object_model.get_articulation("link_2_to_link_3")

    ctx.allow_overlap(
        side_bracket,
        link_1,
        reason=(
            "The grounded clevis and first-link root eye are modeled as an interleaved "
            "coaxial hinge simplification with shared knuckle volume at the root pivot."
        ),
    )

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

    joint_axes_ok = all(joint.axis == (0.0, -1.0, 0.0) for joint in (joint_1, joint_2, joint_3))
    ctx.check(
        "parallel_planar_joint_axes",
        joint_axes_ok,
        details="All three revolute joints should share a common -Y axis for one-plane folding.",
    )

    wide_limits_ok = all(
        joint.motion_limits is not None
        and joint.motion_limits.lower is not None
        and joint.motion_limits.upper is not None
        and joint.motion_limits.lower <= -2.2
        and joint.motion_limits.upper >= 2.2
        for joint in (joint_1, joint_2, joint_3)
    )
    ctx.check(
        "wide_folding_limits",
        wide_limits_ok,
        details="Each joint should allow a deep fold in both directions.",
    )

    ctx.expect_contact(side_bracket, link_1, name="root_hinge_seated")
    ctx.expect_contact(link_1, link_2, name="middle_hinge_seated_1")
    ctx.expect_contact(link_2, link_3, name="middle_hinge_seated_2")

    ctx.expect_origin_gap(
        link_2,
        link_1,
        axis="x",
        min_gap=LINK_1_LEN - 0.002,
        max_gap=LINK_1_LEN + 0.002,
        name="link_1_joint_spacing",
    )
    ctx.expect_origin_gap(
        link_3,
        link_2,
        axis="x",
        min_gap=LINK_2_LEN - 0.002,
        max_gap=LINK_2_LEN + 0.002,
        name="link_2_joint_spacing",
    )

    straight_pos = ctx.part_world_position(link_3)
    with ctx.pose(
        {
            joint_1: 1.55,
            joint_2: -2.30,
            joint_3: 0.90,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_folded_pose")
        folded_pos = ctx.part_world_position(link_3)

    folds_compactly = (
        straight_pos is not None
        and folded_pos is not None
        and folded_pos[0] < straight_pos[0] * 0.45
    )
    ctx.check(
        "folds_to_compact_reach",
        folds_compactly,
        details=(
            f"Expected folded link_3 origin x to shrink well below straight reach; "
            f"got straight={straight_pos}, folded={folded_pos}."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
