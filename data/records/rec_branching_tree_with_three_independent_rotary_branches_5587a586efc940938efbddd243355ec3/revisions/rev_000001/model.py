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


BASE_W = 0.28
BASE_D = 0.24
BASE_T = 0.025

MAST_W = 0.12
MAST_D = 0.10
MAST_H = 0.62
MAST_WALL = 0.008
MAST_TOP_CAP = 0.01

LOWER_Z = 0.185
MID_Z = 0.355
UPPER_Z = 0.525

UPPER_SUPPORT_LEN = 0.080
UPPER_SUPPORT_W = 0.074
UPPER_SUPPORT_H = 0.062
UPPER_PIN_R = 0.011

MID_SUPPORT_LEN = 0.080
MID_SUPPORT_W = 0.096
MID_SUPPORT_H = 0.060
MID_PIN_R = 0.010

LOWER_SUPPORT_LEN = 0.080
LOWER_SUPPORT_W = 0.058
LOWER_SUPPORT_H = 0.052
LOWER_PIN_R = 0.0095

SUPPORT_PAD_T = 0.010
SUPPORT_CHEEK_T = 0.008
OUTBOARD_HUB_LEN = 0.012
PIN_TOTAL_LEN = OUTBOARD_HUB_LEN + 2.0 * SUPPORT_CHEEK_T + 0.008
ARM_STANDOFF = OUTBOARD_HUB_LEN / 2.0 + SUPPORT_CHEEK_T + 0.004


def box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def box_span(
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
) -> cq.Workplane:
    return box_at(
        (x1 - x0, y1 - y0, z1 - z0),
        ((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0),
    )


def cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate(
        (center[0] - length / 2.0, center[1], center[2])
    )


def cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate(
        (center[0], center[1] - length / 2.0, center[2])
    )


def build_mast_assembly() -> cq.Workplane:
    base = box_at((BASE_W, BASE_D, BASE_T), (0.0, 0.0, BASE_T / 2.0))

    mast_outer = box_at((MAST_W, MAST_D, MAST_H), (0.0, 0.0, BASE_T + MAST_H / 2.0))
    inner_height = MAST_H - MAST_TOP_CAP - 0.006
    mast_inner = box_at(
        (MAST_W - 2.0 * MAST_WALL, MAST_D - 2.0 * MAST_WALL, inner_height),
        (0.0, 0.0, BASE_T + 0.003 + inner_height / 2.0),
    )
    mast = mast_outer.cut(mast_inner)

    front_gusset = box_at((0.07, 0.012, 0.09), (0.0, 0.035, BASE_T + 0.045))
    rear_gusset = box_at((0.07, 0.012, 0.09), (0.0, -0.035, BASE_T + 0.045))

    upper_block = box_at(
        (UPPER_SUPPORT_LEN, UPPER_SUPPORT_W, UPPER_SUPPORT_H),
        (MAST_W / 2.0 + UPPER_SUPPORT_LEN / 2.0, 0.0, UPPER_Z),
    )
    upper_cap = cylinder_x(
        0.018,
        0.008,
        (MAST_W / 2.0 + UPPER_SUPPORT_LEN + 0.004, 0.0, UPPER_Z),
    )

    mid_block = box_at(
        (MID_SUPPORT_W, MID_SUPPORT_LEN, MID_SUPPORT_H),
        (0.0, MAST_D / 2.0 + MID_SUPPORT_LEN / 2.0, MID_Z),
    )
    mid_web_left = box_at((0.02, 0.02, 0.05), (-0.03, MAST_D / 2.0 + 0.01, MID_Z - 0.01))
    mid_web_right = box_at((0.02, 0.02, 0.05), (0.03, MAST_D / 2.0 + 0.01, MID_Z - 0.01))

    lower_pad = box_at(
        (LOWER_SUPPORT_LEN, 0.024, LOWER_SUPPORT_H),
        (-MAST_W / 2.0 - LOWER_SUPPORT_LEN / 2.0, 0.0, LOWER_Z),
    )
    lower_top_ear = box_at(
        (LOWER_SUPPORT_LEN * 0.75, LOWER_SUPPORT_W, 0.012),
        (-MAST_W / 2.0 - LOWER_SUPPORT_LEN * 0.375, 0.0, LOWER_Z + LOWER_SUPPORT_H / 2.0 - 0.006),
    )
    lower_bottom_ear = box_at(
        (LOWER_SUPPORT_LEN * 0.75, LOWER_SUPPORT_W, 0.012),
        (-MAST_W / 2.0 - LOWER_SUPPORT_LEN * 0.375, 0.0, LOWER_Z - LOWER_SUPPORT_H / 2.0 + 0.006),
    )

    assembly = (
        base.union(mast)
        .union(front_gusset)
        .union(rear_gusset)
        .union(upper_block)
        .union(upper_cap)
        .union(mid_block)
        .union(mid_web_left)
        .union(mid_web_right)
        .union(lower_pad)
        .union(lower_top_ear)
        .union(lower_bottom_ear)
    )

    return assembly


def build_fork_branch() -> cq.Workplane:
    x_off = 0.008
    collar = cylinder_x(0.014, 0.010, (0.005 + x_off, 0.0, 0.0))
    root_block = box_span(0.006 + x_off, 0.024 + x_off, -0.016, 0.026, -0.015, 0.015)
    spine = box_span(0.010 + x_off, 0.030 + x_off, 0.000, 0.164, -0.012, 0.012)
    tine_upper = box_span(0.008 + x_off, 0.022 + x_off, 0.164, 0.205, 0.010, 0.031)
    tine_lower = box_span(0.008 + x_off, 0.022 + x_off, 0.164, 0.205, -0.031, -0.010)
    return collar.union(root_block).union(spine).union(tine_upper).union(tine_lower)


def build_plate_branch() -> cq.Workplane:
    plate_t = 0.014
    y_off = 0.010
    collar = cylinder_y(0.013, 0.010, (0.0, 0.005 + y_off, 0.0))
    root_pad = box_span(-0.032, 0.0, 0.000 + y_off, 0.012 + y_off, -0.026, 0.026)
    plate_y = 0.012 + y_off
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.0, -0.03)
        .lineTo(-0.07, -0.034)
        .lineTo(-0.22, -0.016)
        .threePointArc((-0.248, 0.008), (-0.222, 0.036))
        .lineTo(-0.10, 0.048)
        .lineTo(-0.02, 0.03)
        .close()
        .extrude(plate_t)
        .translate((0.0, plate_y, 0.0))
    )
    lightening_slot = (
        cq.Workplane("XZ")
        .slot2D(0.105, 0.02, 0)
        .extrude(plate_t + 0.002)
        .translate((-0.125, plate_y + 0.001, 0.006))
    )
    tip_hole = (
        cq.Workplane("XZ")
        .circle(0.012)
        .extrude(plate_t + 0.002)
        .translate((-0.205, plate_y + 0.001, 0.01))
    )
    return collar.union(root_pad).union(profile.cut(lightening_slot).cut(tip_hole))


def build_yoke_branch() -> cq.Workplane:
    collar = cylinder_x(0.013, 0.010, (-0.005, 0.0, 0.0))
    root_block = box_span(-0.024, -0.006, -0.026, 0.016, -0.015, 0.015)
    spine = box_span(-0.028, -0.010, -0.145, 0.000, -0.011, 0.011)
    tine_upper = box_span(-0.026, -0.012, -0.145, -0.104, 0.010, 0.030)
    tine_lower = box_span(-0.026, -0.012, -0.145, -0.104, -0.030, -0.010)
    return collar.union(root_block).union(spine).union(tine_upper).union(tine_lower)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tooling_tree")

    mast_finish = model.material("mast_finish", rgba=(0.22, 0.24, 0.26, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.57, 0.59, 0.62, 1.0))
    pin_finish = model.material("pin_finish", rgba=(0.72, 0.74, 0.77, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(build_mast_assembly(), "mast_assembly"),
        origin=Origin(),
        material=mast_finish,
        name="mast_body",
    )

    fork_branch = model.part("fork_branch")
    fork_branch.visual(
        mesh_from_cadquery(build_fork_branch(), "fork_branch_arm"),
        origin=Origin(),
        material=arm_finish,
        name="fork_arm",
    )

    plate_branch = model.part("plate_branch")
    plate_branch.visual(
        mesh_from_cadquery(build_plate_branch(), "plate_branch_arm"),
        origin=Origin(),
        material=arm_finish,
        name="plate_arm",
    )

    yoke_branch = model.part("yoke_branch")
    yoke_branch.visual(
        mesh_from_cadquery(build_yoke_branch(), "yoke_branch_arm"),
        origin=Origin(),
        material=arm_finish,
        name="yoke_arm",
    )

    model.articulation(
        "mast_to_fork_branch",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=fork_branch,
        origin=Origin(
            xyz=(
                MAST_W / 2.0 + UPPER_SUPPORT_LEN,
                0.0,
                UPPER_Z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-0.85, upper=0.95),
    )
    model.articulation(
        "mast_to_plate_branch",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=plate_branch,
        origin=Origin(
            xyz=(
                0.0,
                MAST_D / 2.0 + MID_SUPPORT_LEN,
                MID_Z,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.4, lower=-0.75, upper=0.85),
    )
    model.articulation(
        "mast_to_yoke_branch",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=yoke_branch,
        origin=Origin(
            xyz=(
                -MAST_W / 2.0 - LOWER_SUPPORT_LEN,
                0.0,
                LOWER_Z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-0.95, upper=0.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    fork_branch = object_model.get_part("fork_branch")
    plate_branch = object_model.get_part("plate_branch")
    yoke_branch = object_model.get_part("yoke_branch")

    fork_joint = object_model.get_articulation("mast_to_fork_branch")
    plate_joint = object_model.get_articulation("mast_to_plate_branch")
    yoke_joint = object_model.get_articulation("mast_to_yoke_branch")

    mast_body = mast.get_visual("mast_body")
    fork_arm = fork_branch.get_visual("fork_arm")
    plate_arm = plate_branch.get_visual("plate_arm")
    yoke_arm = yoke_branch.get_visual("yoke_arm")

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

    ctx.check(
        "expected_parts_present",
        len(object_model.parts) == 4 and len(object_model.articulations) == 3,
        f"parts={len(object_model.parts)} articulations={len(object_model.articulations)}",
    )

    ctx.expect_contact(mast, fork_branch, name="fork_branch_seated_in_upper_bearing")
    ctx.expect_contact(mast, plate_branch, name="plate_branch_seated_in_mid_bearing")
    ctx.expect_contact(mast, yoke_branch, name="yoke_branch_seated_in_lower_bearing")

    ctx.expect_origin_gap(
        fork_branch,
        mast,
        axis="x",
        min_gap=0.139,
        max_gap=0.141,
        name="fork_branch_origin_on_right_face_support",
    )
    ctx.expect_origin_gap(
        plate_branch,
        mast,
        axis="y",
        min_gap=0.129,
        max_gap=0.131,
        name="plate_branch_origin_on_front_face_support",
    )
    ctx.expect_origin_gap(
        mast,
        yoke_branch,
        axis="x",
        min_gap=0.139,
        max_gap=0.141,
        name="yoke_branch_origin_on_left_face_support",
    )
    ctx.expect_origin_gap(
        fork_branch,
        plate_branch,
        axis="z",
        min_gap=0.16,
        max_gap=0.18,
        name="upper_branch_is_above_mid_branch",
    )
    ctx.expect_origin_gap(
        plate_branch,
        yoke_branch,
        axis="z",
        min_gap=0.16,
        max_gap=0.18,
        name="mid_branch_is_above_lower_branch",
    )

    ctx.check(
        "joint_axes_match_supports",
        fork_joint.axis == (1.0, 0.0, 0.0)
        and plate_joint.axis == (0.0, 1.0, 0.0)
        and yoke_joint.axis == (1.0, 0.0, 0.0),
        f"fork={fork_joint.axis} plate={plate_joint.axis} yoke={yoke_joint.axis}",
    )

    def elem_center(part, elem) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    fork_rest = elem_center(fork_branch, fork_arm)
    plate_rest = elem_center(plate_branch, plate_arm)
    yoke_rest = elem_center(yoke_branch, yoke_arm)

    fork_origin = ctx.part_world_position(fork_branch)
    plate_origin = ctx.part_world_position(plate_branch)
    yoke_origin = ctx.part_world_position(yoke_branch)

    with ctx.pose({fork_joint: 0.55}):
        fork_moved = elem_center(fork_branch, fork_arm)
    with ctx.pose({plate_joint: -0.5}):
        plate_moved = elem_center(plate_branch, plate_arm)
    with ctx.pose({yoke_joint: 0.55}):
        yoke_moved = elem_center(yoke_branch, yoke_arm)

    ctx.check(
        "branch_directions_are_visibly_distinct",
        fork_rest is not None
        and plate_rest is not None
        and yoke_rest is not None
        and fork_origin is not None
        and plate_origin is not None
        and yoke_origin is not None
        and fork_rest[1] > fork_origin[1] + 0.08
        and plate_rest[0] < plate_origin[0] - 0.08
        and yoke_rest[1] < yoke_origin[1] - 0.05,
        f"fork_rest={fork_rest} fork_origin={fork_origin} "
        f"plate_rest={plate_rest} plate_origin={plate_origin} "
        f"yoke_rest={yoke_rest} yoke_origin={yoke_origin}",
    )
    ctx.check(
        "fork_branch_rotates_about_local_x",
        fork_rest is not None
        and fork_moved is not None
        and abs(fork_rest[0] - fork_moved[0]) < 0.002
        and abs(fork_rest[2] - fork_moved[2]) > 0.03,
        f"fork_rest={fork_rest} fork_moved={fork_moved}",
    )
    ctx.check(
        "plate_branch_rotates_about_local_y",
        plate_rest is not None
        and plate_moved is not None
        and abs(plate_rest[1] - plate_moved[1]) < 0.002
        and abs(plate_rest[2] - plate_moved[2]) > 0.03,
        f"plate_rest={plate_rest} plate_moved={plate_moved}",
    )
    ctx.check(
        "yoke_branch_rotates_about_local_x",
        yoke_rest is not None
        and yoke_moved is not None
        and abs(yoke_rest[0] - yoke_moved[0]) < 0.002
        and abs(yoke_rest[2] - yoke_moved[2]) > 0.015,
        f"yoke_rest={yoke_rest} yoke_moved={yoke_moved}",
    )

    with ctx.pose({fork_joint: 0.5, plate_joint: -0.45, yoke_joint: 0.45}):
        ctx.fail_if_parts_overlap_in_current_pose(name="clear_in_offset_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
