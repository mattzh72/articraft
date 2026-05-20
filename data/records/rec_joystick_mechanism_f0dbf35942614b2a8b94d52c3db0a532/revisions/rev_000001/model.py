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


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _make_support_fork(
    *,
    base_w: float,
    base_d: float,
    base_t: float,
    inner_gap: float,
    cheek_t: float,
    cheek_d: float,
    cheek_h: float,
) -> cq.Workplane:
    cheek_x = inner_gap / 2.0 + cheek_t / 2.0

    base = cq.Workplane("XY").box(base_w, base_d, base_t).translate((0.0, 0.0, base_t / 2.0))
    left_cheek = (
        cq.Workplane("XY")
        .box(cheek_t, cheek_d, cheek_h)
        .translate((cheek_x, 0.0, base_t + cheek_h / 2.0))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(cheek_t, cheek_d, cheek_h)
        .translate((-cheek_x, 0.0, base_t + cheek_h / 2.0))
    )
    rear_brace = (
        cq.Workplane("XY")
        .box(inner_gap + 2.0 * cheek_t, 0.018, 0.050)
        .translate((0.0, -0.026, base_t + 0.025))
    )
    center_pedestal = (
        cq.Workplane("XY")
        .box(0.052, 0.050, 0.060)
        .translate((0.0, -0.010, base_t + 0.030))
    )
    front_plinth = (
        cq.Workplane("XY")
        .box(0.090, 0.052, 0.018)
        .translate((0.0, 0.012, base_t + 0.009))
    )

    return base.union(left_cheek).union(right_cheek).union(rear_brace).union(center_pedestal).union(front_plinth)


def _make_outer_yoke(
    *,
    outer_w: float,
    outer_d: float,
    outer_h: float,
    trunnion_r: float,
    trunnion_len: float,
    inner_support_r: float,
) -> cq.Workplane:
    plate_t = 0.012
    post_w = 0.010
    bottom_bar_h = 0.012
    bottom_bar_z = -0.035
    post_x = outer_w / 2.0 - post_w / 2.0
    plate_y = outer_d / 2.0 - plate_t / 2.0
    post_h = outer_h - 0.022
    side_block_w = 0.010
    side_block_d = outer_d - 2.0 * plate_t + 0.002
    side_block_h = 0.020
    side_block_x = outer_w / 2.0 - side_block_w / 2.0

    yokes = cq.Workplane("XY")
    for y_sign in (-1.0, 1.0):
        y = y_sign * plate_y
        yokes = yokes.union(
            cq.Workplane("XY").box(post_w, plate_t, post_h).translate((post_x, y, 0.0))
        )
        yokes = yokes.union(
            cq.Workplane("XY").box(post_w, plate_t, post_h).translate((-post_x, y, 0.0))
        )
        yokes = yokes.union(
            cq.Workplane("XY").box(outer_w - 2.0 * post_w, plate_t, bottom_bar_h).translate((0.0, y, bottom_bar_z))
        )
        yokes = yokes.union(
            cq.Workplane("XY").box(0.018, plate_t, 0.024).translate((0.0, y, 0.0))
        )
        yokes = yokes.union(
            cq.Workplane("XY").box(0.010, plate_t, 0.042).translate((0.0, y, -0.009))
        )

    left_side_block = cq.Workplane("XY").box(side_block_w, side_block_d, side_block_h).translate((side_block_x, 0.0, 0.0))
    right_side_block = cq.Workplane("XY").box(side_block_w, side_block_d, side_block_h).translate((-side_block_x, 0.0, 0.0))
    left_trunnion = _x_cylinder(trunnion_r, trunnion_len, (outer_w / 2.0 + trunnion_len / 2.0, 0.0, 0.0))
    right_trunnion = _x_cylinder(trunnion_r, trunnion_len, (-outer_w / 2.0 - trunnion_len / 2.0, 0.0, 0.0))
    front_support = _y_cylinder(inner_support_r, 0.010, (0.0, outer_d / 2.0 + 0.005, 0.0))
    rear_support = _y_cylinder(inner_support_r, 0.010, (0.0, -outer_d / 2.0 - 0.005, 0.0))

    return yokes.union(left_side_block).union(right_side_block).union(left_trunnion).union(right_trunnion).union(front_support).union(rear_support)


def _make_inner_yoke(
    *,
    body_w: float,
    body_d: float,
    body_h: float,
    trunnion_r: float,
    trunnion_len: float,
    collar_r: float,
    collar_h: float,
    collar_center_z: float,
) -> cq.Workplane:
    plate_t = 0.010
    post_w = 0.010
    bottom_bar_h = 0.010
    bottom_bar_z = -0.024
    post_x = body_w / 2.0 - post_w / 2.0
    plate_y = body_d / 2.0 - plate_t / 2.0
    post_h = body_h - 0.004

    frame = cq.Workplane("XY")
    for y_sign in (-1.0, 1.0):
        y = y_sign * plate_y
        frame = frame.union(
            cq.Workplane("XY").box(post_w, plate_t, post_h).translate((post_x, y, -0.003))
        )
        frame = frame.union(
            cq.Workplane("XY").box(post_w, plate_t, post_h).translate((-post_x, y, -0.003))
        )
        frame = frame.union(
            cq.Workplane("XY").box(body_w - 2.0 * post_w, plate_t, bottom_bar_h).translate((0.0, y, bottom_bar_z))
        )
        frame = frame.union(
            cq.Workplane("XY").box(0.014, plate_t, 0.020).translate((0.0, y, 0.0))
        )
        frame = frame.union(
            cq.Workplane("XY").box(0.008, plate_t, 0.038).translate((0.0, y, -0.010))
        )

    front_trunnion = _y_cylinder(trunnion_r, trunnion_len, (0.0, body_d / 2.0 + trunnion_len / 2.0 - 0.001, 0.0))
    rear_trunnion = _y_cylinder(trunnion_r, trunnion_len, (0.0, -body_d / 2.0 - trunnion_len / 2.0 + 0.001, 0.0))
    lower_bridge = cq.Workplane("XY").box(0.028, 0.030, 0.012).translate((0.0, 0.0, -0.031))
    collar = _z_cylinder(collar_r, collar_h, (0.0, 0.0, collar_center_z))

    return frame.union(front_trunnion).union(rear_trunnion).union(lower_bridge).union(collar)


def _make_lever(*, shaft_r: float, shaft_h: float, grip_r: float, grip_h: float) -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(shaft_r).extrude(shaft_h)
    grip_stem = cq.Workplane("XY").workplane(offset=shaft_h).circle(shaft_r * 1.18).extrude(grip_h * 0.35)
    grip_body = cq.Workplane("XY").sphere(grip_r).translate((0.0, 0.0, shaft_h + grip_h * 0.62))
    thumb_top = _z_cylinder(grip_r * 0.62, grip_h * 0.24, (0.0, 0.0, shaft_h + grip_h * 0.98))

    return shaft.union(grip_stem).union(grip_body).union(thumb_top)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_root_joystick_gimbal")

    model.material("powder_coat_dark", color=(0.18, 0.18, 0.20))
    model.material("satin_steel", color=(0.62, 0.64, 0.67))
    model.material("soft_black", color=(0.11, 0.11, 0.12))

    base_w = 0.182
    base_d = 0.160
    base_t = 0.012
    pivot_z = 0.110

    fork_inner_gap = 0.132
    fork_cheek_t = 0.014
    fork_cheek_d = 0.060
    fork_cheek_h = 0.120

    outer_w = 0.100
    outer_d = 0.090
    outer_h = 0.104
    outer_trunnion_r = 0.007
    outer_trunnion_len = 0.013
    inner_support_r = 0.010

    inner_body_w = 0.052
    inner_body_d = 0.038
    inner_body_h = 0.056
    inner_trunnion_r = 0.006
    inner_trunnion_len = 0.015
    collar_r = 0.014
    collar_h = 0.024
    collar_center_z = -0.020
    lever_mount_z = collar_center_z + collar_h / 2.0

    lever_shaft_r = 0.007
    lever_shaft_h = 0.150
    lever_grip_r = 0.017
    lever_grip_h = 0.034

    support_fork_shape = _make_support_fork(
        base_w=base_w,
        base_d=base_d,
        base_t=base_t,
        inner_gap=fork_inner_gap,
        cheek_t=fork_cheek_t,
        cheek_d=fork_cheek_d,
        cheek_h=fork_cheek_h,
    )
    outer_yoke_shape = _make_outer_yoke(
        outer_w=outer_w,
        outer_d=outer_d,
        outer_h=outer_h,
        trunnion_r=outer_trunnion_r,
        trunnion_len=outer_trunnion_len,
        inner_support_r=inner_support_r,
    )
    inner_yoke_shape = _make_inner_yoke(
        body_w=inner_body_w,
        body_d=inner_body_d,
        body_h=inner_body_h,
        trunnion_r=inner_trunnion_r,
        trunnion_len=inner_trunnion_len,
        collar_r=collar_r,
        collar_h=collar_h,
        collar_center_z=collar_center_z,
    )
    lever_shape = _make_lever(
        shaft_r=lever_shaft_r,
        shaft_h=lever_shaft_h,
        grip_r=lever_grip_r,
        grip_h=lever_grip_h,
    )

    support_fork = model.part("support_fork")
    support_fork.visual(
        mesh_from_cadquery(support_fork_shape, "support_fork"),
        material="powder_coat_dark",
        name="support_fork_shell",
    )

    outer_yoke = model.part("outer_yoke")
    outer_yoke.visual(
        mesh_from_cadquery(outer_yoke_shape, "outer_yoke"),
        material="satin_steel",
        name="outer_yoke_shell",
    )

    inner_yoke = model.part("inner_yoke")
    inner_yoke.visual(
        mesh_from_cadquery(inner_yoke_shape, "inner_yoke"),
        material="satin_steel",
        name="inner_yoke_shell",
    )

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(lever_shape, "lever"),
        material="soft_black",
        name="lever_shell",
    )

    model.articulation(
        "support_to_outer_yoke",
        ArticulationType.REVOLUTE,
        parent=support_fork,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.52, upper=0.52, effort=8.0, velocity=2.2),
    )
    model.articulation(
        "outer_to_inner_yoke",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_yoke,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.52, upper=0.52, effort=8.0, velocity=2.2),
    )
    model.articulation(
        "inner_to_lever",
        ArticulationType.FIXED,
        parent=inner_yoke,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, lever_mount_z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support_fork = object_model.get_part("support_fork")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_yoke = object_model.get_part("inner_yoke")
    lever = object_model.get_part("lever")

    outer_joint = object_model.get_articulation("support_to_outer_yoke")
    inner_joint = object_model.get_articulation("outer_to_inner_yoke")
    lever_joint = object_model.get_articulation("inner_to_lever")

    ctx.check(
        "support fork to outer yoke is revolute",
        outer_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(outer_joint.articulation_type),
    )
    ctx.check(
        "outer yoke to inner yoke is revolute",
        inner_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(inner_joint.articulation_type),
    )
    ctx.check(
        "lever is rigidly fixed to inner yoke",
        lever_joint.articulation_type == ArticulationType.FIXED,
        details=str(lever_joint.articulation_type),
    )

    outer_axis = outer_joint.axis
    inner_axis = inner_joint.axis
    axis_dot = sum(a * b for a, b in zip(outer_axis, inner_axis))
    ctx.check(
        "gimbal axes are perpendicular",
        abs(axis_dot) < 1e-6,
        details=f"outer_axis={outer_axis}, inner_axis={inner_axis}, dot={axis_dot}",
    )
    ctx.check(
        "outer joint uses supported fork axis",
        abs(abs(outer_axis[0]) - 1.0) < 1e-6 and abs(outer_axis[1]) < 1e-6 and abs(outer_axis[2]) < 1e-6,
        details=f"outer_axis={outer_axis}",
    )
    ctx.check(
        "inner joint uses supported yoke axis",
        abs(inner_axis[0]) < 1e-6 and abs(abs(inner_axis[1]) - 1.0) < 1e-6 and abs(inner_axis[2]) < 1e-6,
        details=f"inner_axis={inner_axis}",
    )

    ctx.allow_overlap(
        support_fork,
        outer_yoke,
        reason="The outer yoke trunnions are modeled as tightly seated in the fork-root bearing supports.",
    )
    ctx.allow_overlap(
        outer_yoke,
        inner_yoke,
        reason="The inner yoke trunnions are modeled as tightly seated in the outer yoke bearing supports.",
    )

    ctx.expect_contact(
        lever,
        inner_yoke,
        name="lever base seats on the inner yoke collar",
    )
    ctx.expect_origin_distance(
        inner_yoke,
        outer_yoke,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="inner and outer yokes share the gimbal centerline",
    )
    ctx.expect_origin_distance(
        support_fork,
        outer_yoke,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="outer yoke stays centered in the support fork",
    )

    rest_pos = ctx.part_world_position(lever)
    with ctx.pose({outer_joint: 0.35}):
        pitched_pos = ctx.part_world_position(lever)
    ctx.check(
        "outer joint positive motion pitches the lever forward",
        rest_pos is not None and pitched_pos is not None and pitched_pos[1] > rest_pos[1] + 0.002,
        details=f"rest={rest_pos}, pitched={pitched_pos}",
    )

    with ctx.pose({inner_joint: 0.35}):
        rolled_pos = ctx.part_world_position(lever)
    ctx.check(
        "inner joint positive motion rolls the lever to the right",
        rest_pos is not None and rolled_pos is not None and rolled_pos[0] > rest_pos[0] + 0.002,
        details=f"rest={rest_pos}, rolled={rolled_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
