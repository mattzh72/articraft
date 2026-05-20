from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import tempfile

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

_SCRIPT_ORIGIN = None
if "__spec__" in globals() and __spec__ is not None:
    _SCRIPT_ORIGIN = getattr(__spec__, "origin", None)
if not isinstance(_SCRIPT_ORIGIN, str):
    _SCRIPT_ORIGIN = __file__
SCRIPT_DIR = (
    os.path.dirname(_SCRIPT_ORIGIN)
    if isinstance(_SCRIPT_ORIGIN, str) and os.path.isabs(_SCRIPT_ORIGIN)
    else tempfile.gettempdir()
)

BASE_X = 0.210
BASE_Y = 0.150
BASE_Z = 0.044
BASE_CAP_X = 0.168
BASE_CAP_Y = 0.110
BASE_CAP_Z = 0.012
COLUMN_FOOT_X = 0.074
COLUMN_FOOT_Y = 0.060
COLUMN_FOOT_Z = 0.014

COLUMN_X = 0.044
COLUMN_Y = 0.040
COLUMN_Z = 0.340
COLUMN_OFFSET_Y = -0.010
GUIDE_X = 0.022
GUIDE_Y = 0.010
GUIDE_Z = 0.250
GUIDE_BOTTOM_Z = 0.045

HEAD_SLIDE_X = 0.030
HEAD_SLIDE_Y = 0.012
HEAD_SLIDE_Z = 0.145
HEAD_NECK_X = 0.050
HEAD_NECK_Y = 0.014
HEAD_NECK_Z = 0.100
HEAD_BODY_X = 0.118
HEAD_BODY_Y = 0.086
HEAD_BODY_Z = 0.118
HEAD_GEARBOX_X = 0.086
HEAD_GEARBOX_Y = 0.054
HEAD_GEARBOX_Z = 0.064
HEAD_HOME_Z = 0.125
HEAD_TRAVEL = 0.040

SPINDLE_NOSE_RADIUS = 0.022
SPINDLE_NOSE_LENGTH = 0.026
SPINDLE_REVOLUTE_RANGE = 2.0 * math.pi

DEPTH_RAIL_X = 0.047
DEPTH_RAIL_Y = 0.121
DEPTH_RAIL_Z = 0.140
DEPTH_RAIL_LENGTH = 0.140
DEPTH_STOP_TRAVEL = 0.085


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _segment_rpy(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_segment(part, name: str, a, b, *, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_segment_rpy(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_drill_press")

    dark = model.material("dark_gray", rgba=(0.18, 0.18, 0.20, 1.0))
    machine_green = model.material("machine_green", rgba=(0.16, 0.33, 0.28, 1.0))
    deep_green = model.material("deep_green", rgba=(0.13, 0.26, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    handle_black = model.material("handle_black", rgba=(0.09, 0.09, 0.10, 1.0))
    stop_red = model.material("stop_red", rgba=(0.76, 0.15, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_X, BASE_Y, BASE_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z / 2.0)),
        material=dark,
        name="base_shell",
    )
    base.visual(
        Box((BASE_CAP_X, BASE_CAP_Y, BASE_CAP_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z + BASE_CAP_Z / 2.0)),
        material=deep_green,
        name="base_cap",
    )
    base.visual(
        Box((COLUMN_FOOT_X, COLUMN_FOOT_Y, COLUMN_FOOT_Z)),
        origin=Origin(
            xyz=(
                0.0,
                COLUMN_OFFSET_Y,
                BASE_Z + BASE_CAP_Z + COLUMN_FOOT_Z / 2.0,
            )
        ),
        material=deep_green,
        name="column_foot",
    )

    column = model.part("column")
    column.visual(
        Box((COLUMN_X, COLUMN_Y, COLUMN_Z)),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_Z / 2.0)),
        material=dark,
        name="column_mast",
    )
    column.visual(
        Box((GUIDE_X, GUIDE_Y, GUIDE_Z)),
        origin=Origin(
            xyz=(
                0.0,
                COLUMN_Y / 2.0 + GUIDE_Y / 2.0,
                GUIDE_BOTTOM_Z + GUIDE_Z / 2.0,
            )
        ),
        material=steel,
        name="guide_rail",
    )
    column.visual(
        Box((0.010, 0.006, 0.220)),
        origin=Origin(xyz=(-0.013, COLUMN_Y / 2.0 + 0.003, 0.155)),
        material=steel,
        name="rack_strip",
    )

    head = model.part("head")
    head.visual(
        Box((HEAD_SLIDE_X, HEAD_SLIDE_Y, HEAD_SLIDE_Z)),
        origin=Origin(xyz=(0.0, HEAD_SLIDE_Y / 2.0, HEAD_SLIDE_Z / 2.0)),
        material=dark,
        name="slide_plate",
    )
    head.visual(
        Box((HEAD_NECK_X, HEAD_NECK_Y, HEAD_NECK_Z)),
        origin=Origin(
            xyz=(
                0.0,
                HEAD_SLIDE_Y + HEAD_NECK_Y / 2.0,
                0.080,
            )
        ),
        material=deep_green,
        name="neck_bridge",
    )
    head.visual(
        Box((HEAD_BODY_X, HEAD_BODY_Y, HEAD_BODY_Z)),
        origin=Origin(
            xyz=(
                0.0,
                HEAD_SLIDE_Y + HEAD_NECK_Y + HEAD_BODY_Y / 2.0,
                HEAD_BODY_Z / 2.0 + 0.010,
            )
        ),
        material=machine_green,
        name="main_housing",
    )
    head.visual(
        Box((HEAD_GEARBOX_X, HEAD_GEARBOX_Y, HEAD_GEARBOX_Z)),
        origin=Origin(
            xyz=(
                0.0,
                HEAD_SLIDE_Y + HEAD_NECK_Y + HEAD_GEARBOX_Y / 2.0 + 0.010,
                0.145,
            )
        ),
        material=deep_green,
        name="gearbox_housing",
    )
    head.visual(
        Cylinder(radius=0.033, length=0.150),
        origin=Origin(
            xyz=(0.0, HEAD_SLIDE_Y + HEAD_NECK_Y + 0.041, 0.145),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machine_green,
        name="motor_body",
    )
    head.visual(
        Cylinder(radius=SPINDLE_NOSE_RADIUS, length=SPINDLE_NOSE_LENGTH),
        origin=Origin(
            xyz=(
                0.0,
                HEAD_SLIDE_Y + HEAD_NECK_Y + HEAD_BODY_Y - SPINDLE_NOSE_RADIUS,
                SPINDLE_NOSE_LENGTH / 2.0,
            )
        ),
        material=steel,
        name="spindle_nose",
    )
    head.visual(
        Box((0.012, 0.018, DEPTH_RAIL_LENGTH)),
        origin=Origin(xyz=(DEPTH_RAIL_X, DEPTH_RAIL_Y, DEPTH_RAIL_Z / 2.0)),
        material=steel,
        name="depth_rail",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(-0.053, 0.052, 0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="handle_boss",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SPINDLE_NOSE_RADIUS, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=steel,
        name="spindle_collar",
    )
    spindle.visual(
        Cylinder(radius=0.015, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=steel,
        name="annular_cutter",
    )

    depth_stop = model.part("depth_stop")
    depth_stop.visual(
        Box((0.004, 0.036, 0.028)),
        origin=Origin(xyz=(-0.008, 0.018, -0.014)),
        material=stop_red,
        name="left_jaw",
    )
    depth_stop.visual(
        Box((0.004, 0.036, 0.028)),
        origin=Origin(xyz=(0.008, 0.018, -0.014)),
        material=stop_red,
        name="right_jaw",
    )
    depth_stop.visual(
        Box((0.020, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, 0.018, -0.014)),
        material=stop_red,
        name="slider_bridge",
    )
    depth_stop.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(
            xyz=(0.020, 0.018, -0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stop_red,
        name="clamp_knob",
    )
    depth_stop.visual(
        Box((0.008, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, 0.020, -0.053)),
        material=stop_red,
        name="stop_pointer",
    )

    feed_handle = model.part("feed_handle")
    feed_handle.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    for index, angle in enumerate((math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)):
        start = (-0.016, 0.014 * math.cos(angle), 0.014 * math.sin(angle))
        end = (-0.064, 0.052 * math.cos(angle), 0.052 * math.sin(angle))
        _add_segment(
            feed_handle,
            f"spoke_{index}",
            start,
            end,
            radius=0.0038,
            material=steel,
        )
        feed_handle.visual(
            Sphere(radius=0.007),
            origin=Origin(xyz=end),
            material=handle_black,
            name=f"knob_{index}",
        )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(
            xyz=(
                0.0,
                COLUMN_OFFSET_Y,
                BASE_Z + BASE_CAP_Z + COLUMN_FOOT_Z,
            )
        ),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.PRISMATIC,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, COLUMN_Y / 2.0 + GUIDE_Y, HEAD_HOME_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=HEAD_TRAVEL,
        ),
    )
    model.articulation(
        "head_to_spindle",
        ArticulationType.REVOLUTE,
        parent=head,
        child=spindle,
        origin=Origin(
            xyz=(
                0.0,
                HEAD_SLIDE_Y + HEAD_NECK_Y + HEAD_BODY_Y - SPINDLE_NOSE_RADIUS,
                0.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=60.0,
            lower=-SPINDLE_REVOLUTE_RANGE,
            upper=SPINDLE_REVOLUTE_RANGE,
        ),
    )
    model.articulation(
        "head_to_depth_stop",
        ArticulationType.PRISMATIC,
        parent=head,
        child=depth_stop,
        origin=Origin(
            xyz=(
                DEPTH_RAIL_X,
                DEPTH_RAIL_Y + 0.009,
                DEPTH_RAIL_Z,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=DEPTH_STOP_TRAVEL,
        ),
    )
    model.articulation(
        "head_to_feed_handle",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=feed_handle,
        origin=Origin(xyz=(-0.059, 0.052, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=SCRIPT_DIR)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    head = object_model.get_part("head")
    spindle = object_model.get_part("spindle")
    depth_stop = object_model.get_part("depth_stop")
    feed_handle = object_model.get_part("feed_handle")
    head_slide = object_model.get_articulation("column_to_head")
    spindle_joint = object_model.get_articulation("head_to_spindle")
    depth_stop_joint = object_model.get_articulation("head_to_depth_stop")
    feed_handle_joint = object_model.get_articulation("head_to_feed_handle")

    base_shell = base.get_visual("base_shell")
    column_foot = base.get_visual("column_foot")
    column_mast = column.get_visual("column_mast")
    guide_rail = column.get_visual("guide_rail")
    slide_plate = head.get_visual("slide_plate")
    main_housing = head.get_visual("main_housing")
    spindle_nose = head.get_visual("spindle_nose")
    depth_rail = head.get_visual("depth_rail")
    handle_boss = head.get_visual("handle_boss")
    spindle_collar = spindle.get_visual("spindle_collar")
    annular_cutter = spindle.get_visual("annular_cutter")
    left_jaw = depth_stop.get_visual("left_jaw")
    right_jaw = depth_stop.get_visual("right_jaw")
    slider_bridge = depth_stop.get_visual("slider_bridge")
    stop_pointer = depth_stop.get_visual("stop_pointer")
    hub = feed_handle.get_visual("hub")
    knob_0 = feed_handle.get_visual("knob_0")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.fail_if_isolated_parts(max_pose_samples=12)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.check(
        "joint_axes",
        head_slide.axis == (0.0, 0.0, -1.0)
        and spindle_joint.axis == (0.0, 0.0, 1.0)
        and depth_stop_joint.axis == (0.0, 0.0, -1.0)
        and feed_handle_joint.axis == (1.0, 0.0, 0.0),
        "Unexpected articulation axes for the drill feed, spindle, depth stop, or feed handle.",
    )
    ctx.check(
        "part_count",
        len(object_model.parts) == 6,
        f"Expected 6 parts, found {len(object_model.parts)}.",
    )

    ctx.expect_within(column, base, axes="xy", inner_elem=column_mast, outer_elem=base_shell)
    ctx.expect_contact(column, base, elem_a=column_mast, elem_b=column_foot)
    ctx.expect_contact(head, column, elem_a=slide_plate, elem_b=guide_rail)
    ctx.expect_overlap(head, column, axes="xz", min_overlap=0.020, elem_a=slide_plate, elem_b=guide_rail)
    ctx.expect_contact(spindle, head, elem_a=spindle_collar, elem_b=spindle_nose)
    ctx.expect_contact(depth_stop, head, elem_a=left_jaw, elem_b=depth_rail)
    ctx.expect_contact(depth_stop, head, elem_a=right_jaw, elem_b=depth_rail)
    ctx.expect_contact(feed_handle, head, elem_a=hub, elem_b=handle_boss)
    ctx.expect_gap(spindle, base, axis="y", min_gap=0.020, positive_elem=annular_cutter, negative_elem=base_shell)
    ctx.expect_gap(
        depth_stop,
        base,
        axis="z",
        min_gap=0.080,
        positive_elem=stop_pointer,
        negative_elem=base_shell,
    )
    ctx.expect_gap(
        depth_stop,
        head,
        axis="y",
        min_gap=0.015,
        positive_elem=left_jaw,
        negative_elem=main_housing,
        name="depth_stop_in_front_of_head",
    )
    ctx.expect_overlap(
        depth_stop,
        head,
        axes="z",
        min_overlap=0.020,
        elem_a=slider_bridge,
        elem_b=depth_rail,
    )

    head_rest = ctx.part_world_position(head)
    depth_stop_rest = ctx.part_world_position(depth_stop)
    knob_rest_center = _aabb_center(ctx.part_element_world_aabb(feed_handle, elem=knob_0))

    with ctx.pose({head_slide: HEAD_TRAVEL}):
        head_low = ctx.part_world_position(head)
        ctx.check(
            "head_feed_travel",
            head_rest is not None and head_low is not None and head_low[2] < head_rest[2] - 0.030,
            "The drill head does not slide down the column by a meaningful amount.",
        )
        ctx.expect_contact(head, column, elem_a=slide_plate, elem_b=guide_rail)
        ctx.expect_contact(spindle, head, elem_a=spindle_collar, elem_b=spindle_nose)
        ctx.expect_gap(spindle, base, axis="y", min_gap=0.020, positive_elem=annular_cutter, negative_elem=base_shell)
        ctx.fail_if_parts_overlap_in_current_pose(name="head_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="head_lower_no_floating")

    with ctx.pose({depth_stop_joint: DEPTH_STOP_TRAVEL}):
        depth_stop_low = ctx.part_world_position(depth_stop)
        ctx.check(
            "depth_stop_travel",
            depth_stop_rest is not None
            and depth_stop_low is not None
            and depth_stop_low[2] < depth_stop_rest[2] - 0.060,
            "The depth stop does not slide down its rail by the expected amount.",
        )
        ctx.expect_contact(depth_stop, head, elem_a=left_jaw, elem_b=depth_rail)
        ctx.expect_contact(depth_stop, head, elem_a=right_jaw, elem_b=depth_rail)
        ctx.expect_gap(
            depth_stop,
            base,
            axis="z",
            min_gap=0.0,
            positive_elem=stop_pointer,
            negative_elem=base_shell,
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="depth_stop_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="depth_stop_lower_no_floating")

    with ctx.pose({spindle_joint: math.pi / 2.0}):
        ctx.expect_contact(spindle, head, elem_a=spindle_collar, elem_b=spindle_nose)
        ctx.fail_if_parts_overlap_in_current_pose(name="spindle_rotated_no_overlap")

    with ctx.pose({feed_handle_joint: math.pi / 2.0}):
        knob_rotated_center = _aabb_center(ctx.part_element_world_aabb(feed_handle, elem=knob_0))
        ctx.check(
            "feed_handle_rotation",
            knob_rest_center is not None
            and knob_rotated_center is not None
            and abs(knob_rotated_center[1] - knob_rest_center[1]) > 0.040
            and abs(knob_rotated_center[2] - knob_rest_center[2]) > 0.040,
            "The feed handle does not visibly rotate around its hub.",
        )
        ctx.expect_contact(feed_handle, head, elem_a=hub, elem_b=handle_boss)
        ctx.fail_if_parts_overlap_in_current_pose(name="feed_handle_rotated_no_overlap")

    spindle_limits = spindle_joint.motion_limits
    if spindle_limits is not None and spindle_limits.lower is not None and spindle_limits.upper is not None:
        with ctx.pose({spindle_joint: spindle_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="spindle_lower_limit_no_overlap")
            ctx.fail_if_isolated_parts(name="spindle_lower_limit_no_floating")
        with ctx.pose({spindle_joint: spindle_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="spindle_upper_limit_no_overlap")
            ctx.fail_if_isolated_parts(name="spindle_upper_limit_no_floating")

    head_limits = head_slide.motion_limits
    if head_limits is not None and head_limits.lower is not None and head_limits.upper is not None:
        with ctx.pose({head_slide: head_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="head_upper_limit_no_overlap")
            ctx.fail_if_isolated_parts(name="head_upper_limit_no_floating")
        with ctx.pose({head_slide: head_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="head_lower_limit_no_overlap")
            ctx.fail_if_isolated_parts(name="head_lower_limit_no_floating")

    stop_limits = depth_stop_joint.motion_limits
    if stop_limits is not None and stop_limits.lower is not None and stop_limits.upper is not None:
        with ctx.pose({depth_stop_joint: stop_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="depth_stop_upper_limit_no_overlap")
            ctx.fail_if_isolated_parts(name="depth_stop_upper_limit_no_floating")
        with ctx.pose({depth_stop_joint: stop_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="depth_stop_lower_limit_no_overlap")
            ctx.fail_if_isolated_parts(name="depth_stop_lower_limit_no_floating")
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
