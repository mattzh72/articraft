from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ROOT_W = 0.180
ROOT_D = 0.125
ROOT_H = 0.020
ROOT_PEDESTAL_D = 0.086
ROOT_PEDESTAL_H = 0.018

YAW_D = 0.074
YAW_H = 0.028
PITCH_AXIS_X = 0.042
PITCH_AXIS_Z = 0.020

YOKE_LEN = 0.060
YOKE_W = 0.050
YOKE_REAR_L = 0.020
YOKE_REAR_H = 0.040
YOKE_WEB_L = 0.042
YOKE_WEB_W = 0.036
YOKE_WEB_T = 0.010
YOKE_WEB_Z = 0.017
YOKE_FRONT_R = 0.022
YOKE_FRONT_L = 0.012

ROLL_HUB_R = 0.016
ROLL_HUB_L = 0.014
ROLL_FLANGE_R = 0.028
ROLL_FLANGE_T = 0.008


def _make_root_stage_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(ROOT_W, ROOT_D, ROOT_H)
        .translate((0.0, 0.0, ROOT_H / 2.0))
        .edges("|Z")
        .fillet(0.007)
    )

    pedestal = (
        cq.Workplane("XY")
        .circle(ROOT_PEDESTAL_D / 2.0)
        .extrude(ROOT_PEDESTAL_H)
        .translate((0.0, 0.0, ROOT_H))
    )

    front_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.018, ROOT_H),
                (0.060, ROOT_H),
                (0.028, ROOT_H + ROOT_PEDESTAL_H),
                (0.010, ROOT_H + ROOT_PEDESTAL_H),
            ]
        )
        .close()
        .extrude(0.007, both=True)
        .translate((0.0, 0.051, 0.0))
    )
    rear_rib = front_rib.mirror("XZ")

    return plate.union(pedestal).union(front_rib).union(rear_rib)


def _make_yaw_base_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(YAW_D / 2.0).extrude(YAW_H)
    nose = (
        cq.Workplane("XY")
        .box(0.046, 0.050, 0.020)
        .translate((0.019, 0.0, 0.014))
    )
    top_pad = (
        cq.Workplane("XY")
        .circle(0.026)
        .extrude(0.004)
        .translate((0.0, 0.0, YAW_H))
    )

    return collar.union(nose).union(top_pad)


def _make_pitch_yoke_shape() -> cq.Workplane:
    rear_knuckle = (
        cq.Workplane("XY")
        .box(YOKE_REAR_L, YOKE_W, YOKE_REAR_H)
        .translate((YOKE_REAR_L / 2.0, 0.0, 0.0))
    )
    upper_web = (
        cq.Workplane("XY")
        .box(YOKE_WEB_L, YOKE_WEB_W, YOKE_WEB_T)
        .translate((0.031, 0.0, YOKE_WEB_Z))
    )
    lower_web = (
        cq.Workplane("XY")
        .box(YOKE_WEB_L, YOKE_WEB_W, YOKE_WEB_T)
        .translate((0.031, 0.0, -YOKE_WEB_Z))
    )
    front_collar = (
        cq.Workplane("YZ")
        .circle(YOKE_FRONT_R)
        .extrude(YOKE_FRONT_L)
        .translate((YOKE_LEN - YOKE_FRONT_L, 0.0, 0.0))
    )

    window = (
        cq.Workplane("XY")
        .box(0.032, 0.022, 0.022)
        .translate((0.030, 0.0, 0.0))
    )
    spindle_bore = (
        cq.Workplane("YZ")
        .circle(0.011)
        .extrude(0.020)
        .translate((0.040, 0.0, 0.0))
    )

    body = rear_knuckle.union(upper_web).union(lower_web).union(front_collar)
    return body.cut(window).cut(spindle_bore)


def _make_roll_flange_shape() -> cq.Workplane:
    hub = cq.Workplane("YZ").circle(ROLL_HUB_R).extrude(ROLL_HUB_L)
    flange = (
        cq.Workplane("YZ")
        .circle(ROLL_FLANGE_R)
        .extrude(ROLL_FLANGE_T)
        .translate((ROLL_HUB_L, 0.0, 0.0))
    )
    face_recess = (
        cq.Workplane("YZ")
        .circle(0.014)
        .extrude(0.003)
        .translate((ROLL_HUB_L + ROLL_FLANGE_T - 0.003, 0.0, 0.0))
    )
    bolt_circle = (
        cq.Workplane("YZ")
        .pushPoints([(0.017, 0.0), (-0.017, 0.0), (0.0, 0.017), (0.0, -0.017)])
        .circle(0.0022)
        .extrude(0.010)
        .translate((ROLL_HUB_L - 0.001, 0.0, 0.0))
    )

    return hub.union(flange).cut(face_recess).cut(bolt_circle)


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(maxs[i] - mins[i] for i in range(3))


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_axis_low_profile_wrist")

    root_color = model.material("root_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    yaw_color = model.material("yaw_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    yoke_color = model.material("yoke_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    flange_color = model.material("flange_silver", rgba=(0.83, 0.84, 0.86, 1.0))
    pad_color = model.material("pad_black", rgba=(0.10, 0.10, 0.11, 1.0))

    root_stage = model.part("root_stage")
    root_stage.visual(
        mesh_from_cadquery(_make_root_stage_shape(), "root_stage_shell"),
        material=root_color,
        name="root_shell",
    )
    root_stage.inertial = Inertial.from_geometry(
        Box((ROOT_W, ROOT_D, ROOT_H + ROOT_PEDESTAL_H)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, (ROOT_H + ROOT_PEDESTAL_H) / 2.0)),
    )

    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        mesh_from_cadquery(_make_yaw_base_shape(), "yaw_base_shell"),
        material=yaw_color,
        name="yaw_shell",
    )
    yaw_base.inertial = Inertial.from_geometry(
        Box((0.078, 0.078, 0.032)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        mesh_from_cadquery(_make_pitch_yoke_shape(), "pitch_yoke_shell"),
        material=yoke_color,
        name="yoke_shell",
    )
    pitch_yoke.inertial = Inertial.from_geometry(
        Box((YOKE_LEN, YOKE_W, 0.048)),
        mass=0.75,
        origin=Origin(xyz=(YOKE_LEN / 2.0, 0.0, 0.0)),
    )

    roll_flange = model.part("roll_flange")
    roll_flange.visual(
        mesh_from_cadquery(_make_roll_flange_shape(), "roll_flange_shell"),
        material=flange_color,
        name="roll_shell",
    )
    roll_flange.visual(
        Box((0.008, 0.014, 0.010)),
        origin=Origin(xyz=(0.026, 0.0, 0.023)),
        material=pad_color,
        name="tool_pad",
    )
    roll_flange.inertial = Inertial.from_geometry(
        Box((0.030, 0.056, 0.056)),
        mass=0.35,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "root_to_yaw",
        ArticulationType.REVOLUTE,
        parent=root_stage,
        child=yaw_base,
        origin=Origin(xyz=(0.0, 0.0, ROOT_H + ROOT_PEDESTAL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_base,
        child=pitch_yoke,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.6, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_flange,
        origin=Origin(xyz=(YOKE_LEN, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-pi, upper=pi),
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

    root_stage = object_model.get_part("root_stage")
    yaw_base = object_model.get_part("yaw_base")
    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_flange = object_model.get_part("roll_flange")

    root_to_yaw = object_model.get_articulation("root_to_yaw")
    yaw_to_pitch = object_model.get_articulation("yaw_to_pitch")
    pitch_to_roll = object_model.get_articulation("pitch_to_roll")

    root_shell = root_stage.get_visual("root_shell")
    yaw_shell = yaw_base.get_visual("yaw_shell")
    yoke_shell = pitch_yoke.get_visual("yoke_shell")
    roll_shell = roll_flange.get_visual("roll_shell")
    tool_pad = roll_flange.get_visual("tool_pad")

    ctx.expect_contact(yaw_base, root_stage, elem_a=yaw_shell, elem_b=root_shell, name="yaw base mounted to root stage")
    ctx.expect_contact(pitch_yoke, yaw_base, elem_a=yoke_shell, elem_b=yaw_shell, name="pitch yoke mounted to yaw base")
    ctx.expect_contact(roll_flange, pitch_yoke, elem_a=roll_shell, elem_b=yoke_shell, name="roll flange mounted to yoke")

    root_aabb = ctx.part_world_aabb(root_stage)
    yaw_aabb = ctx.part_world_aabb(yaw_base)
    yoke_aabb = ctx.part_world_aabb(pitch_yoke)
    roll_aabb = ctx.part_world_aabb(roll_flange)

    root_size = _aabb_size(root_aabb)
    yaw_size = _aabb_size(yaw_aabb)
    yoke_size = _aabb_size(yoke_aabb)
    roll_size = _aabb_size(roll_aabb)

    size_data_ready = all(size is not None for size in (root_size, yaw_size, yoke_size, roll_size))
    if size_data_ready:
        assert root_size is not None
        assert yaw_size is not None
        assert yoke_size is not None
        assert roll_size is not None
        root_proxy = root_size[0] * root_size[1] * root_size[2]
        yaw_proxy = yaw_size[0] * yaw_size[1] * yaw_size[2]
        yoke_proxy = yoke_size[0] * yoke_size[1] * yoke_size[2]
        roll_proxy = roll_size[0] * roll_size[1] * roll_size[2]
        ctx.check(
            "stage envelopes taper from root to tip",
            root_proxy > yaw_proxy > yoke_proxy > roll_proxy,
            details=(
                f"root={root_proxy:.6f}, yaw={yaw_proxy:.6f}, "
                f"pitch={yoke_proxy:.6f}, roll={roll_proxy:.6f}"
            ),
        )
    else:
        ctx.fail("stage envelopes taper from root to tip", "could not measure one or more part AABBs")

    with ctx.pose({root_to_yaw: 0.0, yaw_to_pitch: 0.0, pitch_to_roll: 0.0}):
        rest_roll_origin = ctx.part_world_position(roll_flange)
        rest_pad_center = _aabb_center(ctx.part_element_world_aabb(roll_flange, elem=tool_pad))

    with ctx.pose({root_to_yaw: 0.65, yaw_to_pitch: 0.0, pitch_to_roll: 0.0}):
        yawed_roll_origin = ctx.part_world_position(roll_flange)

    with ctx.pose({root_to_yaw: 0.0, yaw_to_pitch: 0.75, pitch_to_roll: 0.0}):
        pitched_roll_origin = ctx.part_world_position(roll_flange)

    with ctx.pose({root_to_yaw: 0.0, yaw_to_pitch: 0.0, pitch_to_roll: pi / 2.0}):
        rolled_pad_center = _aabb_center(ctx.part_element_world_aabb(roll_flange, elem=tool_pad))

    pose_data_ready = all(
        item is not None
        for item in (rest_roll_origin, yawed_roll_origin, pitched_roll_origin, rest_pad_center, rolled_pad_center)
    )
    if pose_data_ready:
        assert rest_roll_origin is not None
        assert yawed_roll_origin is not None
        assert pitched_roll_origin is not None
        assert rest_pad_center is not None
        assert rolled_pad_center is not None

        ctx.check(
            "yaw swings output around vertical base axis",
            yawed_roll_origin[1] > rest_roll_origin[1] + 0.04,
            details=f"rest_y={rest_roll_origin[1]:.4f}, yawed_y={yawed_roll_origin[1]:.4f}",
        )
        ctx.check(
            "pitch raises the output nose upward",
            pitched_roll_origin[2] > rest_roll_origin[2] + 0.03,
            details=f"rest_z={rest_roll_origin[2]:.4f}, pitched_z={pitched_roll_origin[2]:.4f}",
        )
        ctx.check(
            "roll spins the off-axis tool pad around x",
            rest_pad_center[2] > 0.015 and rolled_pad_center[1] < -0.015,
            details=(
                f"rest_pad_center={tuple(round(v, 4) for v in rest_pad_center)}, "
                f"rolled_pad_center={tuple(round(v, 4) for v in rolled_pad_center)}"
            ),
        )
    else:
        ctx.fail("pose-driven wrist articulation checks", "could not measure one or more posed positions")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
