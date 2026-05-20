from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib
import sys

_REAL_GETCWD = os.getcwd
_REAL_PATH_ABSOLUTE = pathlib.Path.absolute
_REAL_PATH_CWD = pathlib.Path.cwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return _REAL_GETCWD()


os.getcwd = _safe_getcwd
pathlib.os.getcwd = _safe_getcwd


def _safe_path_absolute(self):
    try:
        return _REAL_PATH_ABSOLUTE(self)
    except FileNotFoundError:
        os.chdir("/")
        return _REAL_PATH_ABSOLUTE(self)


@classmethod
def _safe_path_cwd(cls):
    try:
        return _REAL_PATH_CWD()
    except FileNotFoundError:
        os.chdir("/")
        return _REAL_PATH_CWD()


pathlib.Path.absolute = _safe_path_absolute
pathlib.Path.cwd = _safe_path_cwd

if isinstance(globals().get("__file__"), str) and not os.path.isabs(__file__):
    argv0 = sys.argv[0] if sys.argv else ""
    if isinstance(argv0, str) and os.path.isabs(argv0):
        __file__ = argv0

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


TRACK_LENGTH = 0.78
TRACK_WIDTH = 0.038
TRACK_HEIGHT = 0.030
CANOPY_WIDTH = 0.180
CANOPY_DEPTH = 0.120
CANOPY_HEIGHT = 0.022
DROP_TUBE_RADIUS = 0.016
DROP_TUBE_LENGTH = 0.023
SEAT_WIDTH = 0.050
SEAT_DEPTH = 0.030
SEAT_HEIGHT = 0.008
HEAD_SPACING = 0.240
HEAD_BASE_PITCH = 0.82
YOKE_ARM_CENTER_Y = 0.034
MOUNT_TOP_Z = 0.038
TILT_ORIGIN_Z = 0.078
TRACK_TOP_Z = CANOPY_HEIGHT + DROP_TUBE_LENGTH
TRACK_CENTER_Z = TRACK_TOP_Z + TRACK_HEIGHT / 2.0
TRACK_BOTTOM_Z = TRACK_TOP_Z + TRACK_HEIGHT
SEAT_CENTER_Z = TRACK_BOTTOM_Z + SEAT_HEIGHT / 2.0
SWIVEL_ORIGIN_Z = TRACK_BOTTOM_Z + SEAT_HEIGHT


def _axis_offset(distance: float) -> tuple[float, float, float]:
    return (
        math.sin(HEAD_BASE_PITCH) * distance,
        0.0,
        math.cos(HEAD_BASE_PITCH) * distance,
    )


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) / 2.0 for axis in range(3))


def _element_center(ctx: TestContext, part, elem_name: str):
    aabb = ctx.part_element_world_aabb(part, elem=elem_name)
    if aabb is None:
        return None
    return _aabb_center(aabb)


def _add_spotlight(
    model: ArticulatedObject,
    rail,
    *,
    name: str,
    x_pos: float,
    seat_name: str,
    head_finish,
    hardware_finish,
    trim_finish,
    lens_finish,
) -> None:
    mount = model.part(f"{name}_mount")
    mount.visual(
        Box((0.040, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=hardware_finish,
        name="track_shoe",
    )
    mount.visual(
        Box((0.022, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=hardware_finish,
        name="shoe_collar",
    )
    mount.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=hardware_finish,
        name="swivel_pedestal",
    )
    mount.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=hardware_finish,
        name="swivel_cap",
    )
    mount.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, 0.040)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    yoke = model.part(f"{name}_yoke")
    yoke.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=hardware_finish,
        name="swivel_collar",
    )
    yoke.visual(
        Box((0.010, 0.016, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=hardware_finish,
        name="upright",
    )
    yoke.visual(
        Box((0.014, 0.068, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=hardware_finish,
        name="yoke_bridge",
    )
    yoke.visual(
        Box((0.010, 0.008, 0.056)),
        origin=Origin(xyz=(0.0, -YOKE_ARM_CENTER_Y, 0.078)),
        material=hardware_finish,
        name="yoke_arm_left",
    )
    yoke.visual(
        Box((0.010, 0.008, 0.056)),
        origin=Origin(xyz=(0.0, YOKE_ARM_CENTER_Y, 0.078)),
        material=hardware_finish,
        name="yoke_arm_right",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.050, 0.090, 0.110)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
    )

    head = model.part(f"{name}_head")
    head.visual(
        Box((0.012, 0.060, 0.018)),
        origin=Origin(xyz=(-0.011, 0.0, 0.0)),
        material=trim_finish,
        name="pivot_block",
    )
    head.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_finish,
        name="pivot_knuckle",
    )
    rear_can_center = _axis_offset(0.012)
    body_center = _axis_offset(0.045)
    bezel_center = _axis_offset(0.088)
    baffle_center = _axis_offset(0.092)
    lens_center = _axis_offset(0.097)
    barrel_rpy = (0.0, HEAD_BASE_PITCH, 0.0)
    head.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=rear_can_center, rpy=barrel_rpy),
        material=head_finish,
        name="rear_can",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.078),
        origin=Origin(xyz=body_center, rpy=barrel_rpy),
        material=head_finish,
        name="body_shell",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=bezel_center, rpy=barrel_rpy),
        material=head_finish,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.021, length=0.008),
        origin=Origin(xyz=baffle_center, rpy=barrel_rpy),
        material=trim_finish,
        name="inner_baffle",
    )
    head.visual(
        Cylinder(radius=0.023, length=0.003),
        origin=Origin(xyz=lens_center, rpy=barrel_rpy),
        material=lens_finish,
        name="lens",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.110, 0.080, 0.085)),
        mass=0.38,
        origin=Origin(xyz=(0.040, 0.0, 0.030)),
    )

    model.articulation(
        f"{name}_mount_joint",
        ArticulationType.FIXED,
        parent=rail,
        child=mount,
        origin=Origin(xyz=(x_pos, 0.0, TRACK_BOTTOM_Z + SEAT_HEIGHT)),
    )
    model.articulation(
        f"{name}_swivel",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, MOUNT_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-2.8,
            upper=2.8,
        ),
        meta={"seat_visual": seat_name},
    )
    model.articulation(
        f"{name}_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, TILT_ORIGIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.45,
            upper=0.75,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_lighting_fixture")

    rail_white = model.material("rail_white", rgba=(0.95, 0.95, 0.94, 1.0))
    hardware_gray = model.material("hardware_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    trim_black = model.material("trim_black", rgba=(0.11, 0.11, 0.12, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.95, 0.90, 0.78, 0.82))

    rail = model.part("rail")
    rail.visual(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT / 2.0)),
        material=rail_white,
        name="ceiling_canopy",
    )
    rail.visual(
        Cylinder(radius=DROP_TUBE_RADIUS, length=DROP_TUBE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT + DROP_TUBE_LENGTH / 2.0)),
        material=hardware_gray,
        name="drop_tube",
    )
    rail.visual(
        Box((TRACK_LENGTH, TRACK_WIDTH, TRACK_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, TRACK_CENTER_Z)),
        material=rail_white,
        name="rail_body",
    )
    rail.visual(
        Box((TRACK_LENGTH * 0.92, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, TRACK_BOTTOM_Z - 0.002)),
        material=trim_black,
        name="conductor_slot",
    )
    for seat_name, x_pos in (
        ("seat_left", -HEAD_SPACING),
        ("seat_center", 0.0),
        ("seat_right", HEAD_SPACING),
    ):
        rail.visual(
            Box((SEAT_WIDTH, SEAT_DEPTH, SEAT_HEIGHT)),
            origin=Origin(xyz=(x_pos, 0.0, SEAT_CENTER_Z)),
            material=hardware_gray,
            name=seat_name,
        )
    rail.inertial = Inertial.from_geometry(
        Box((TRACK_LENGTH, CANOPY_DEPTH, TRACK_BOTTOM_Z)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, TRACK_BOTTOM_Z / 2.0)),
    )

    _add_spotlight(
        model,
        rail,
        name="spotlight_left",
        x_pos=-HEAD_SPACING,
        seat_name="seat_left",
        head_finish=rail_white,
        hardware_finish=hardware_gray,
        trim_finish=trim_black,
        lens_finish=lens_glass,
    )
    _add_spotlight(
        model,
        rail,
        name="spotlight_center",
        x_pos=0.0,
        seat_name="seat_center",
        head_finish=rail_white,
        hardware_finish=hardware_gray,
        trim_finish=trim_black,
        lens_finish=lens_glass,
    )
    _add_spotlight(
        model,
        rail,
        name="spotlight_right",
        x_pos=HEAD_SPACING,
        seat_name="seat_right",
        head_finish=rail_white,
        hardware_finish=hardware_gray,
        trim_finish=trim_black,
        lens_finish=lens_glass,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rail = object_model.get_part("rail")
    rail_body = rail.get_visual("rail_body")
    seat_left = rail.get_visual("seat_left")
    seat_center = rail.get_visual("seat_center")
    seat_right = rail.get_visual("seat_right")

    left_mount = object_model.get_part("spotlight_left_mount")
    center_mount = object_model.get_part("spotlight_center_mount")
    right_mount = object_model.get_part("spotlight_right_mount")

    left_yoke = object_model.get_part("spotlight_left_yoke")
    center_yoke = object_model.get_part("spotlight_center_yoke")
    right_yoke = object_model.get_part("spotlight_right_yoke")

    left_head = object_model.get_part("spotlight_left_head")
    center_head = object_model.get_part("spotlight_center_head")
    right_head = object_model.get_part("spotlight_right_head")

    left_swivel = object_model.get_articulation("spotlight_left_swivel")
    center_swivel = object_model.get_articulation("spotlight_center_swivel")
    right_swivel = object_model.get_articulation("spotlight_right_swivel")
    left_tilt = object_model.get_articulation("spotlight_left_tilt")
    center_tilt = object_model.get_articulation("spotlight_center_tilt")
    right_tilt = object_model.get_articulation("spotlight_right_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    for yoke, head in (
        (left_yoke, left_head),
        (center_yoke, center_head),
        (right_yoke, right_head),
    ):
        ctx.allow_overlap(
            yoke,
            head,
            reason="Tilt hinge captures the lamp pivot block between the yoke cheeks in this simplified spotlight assembly.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=64,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_gap(
        center_mount,
        left_mount,
        axis="x",
        min_gap=0.22,
        max_gap=0.26,
        name="left_to_center_spacing",
    )
    ctx.expect_origin_gap(
        right_mount,
        center_mount,
        axis="x",
        min_gap=0.22,
        max_gap=0.26,
        name="center_to_right_spacing",
    )

    for mount, yoke, head, seat in (
        (left_mount, left_yoke, left_head, seat_left),
        (center_mount, center_yoke, center_head, seat_center),
        (right_mount, right_yoke, right_head, seat_right),
    ):
        ctx.expect_within(
            mount,
            rail,
            inner_elem="track_shoe",
            outer_elem=seat,
            axes="xy",
        )
        ctx.expect_contact(rail, mount, elem_a=seat, elem_b="track_shoe")
        ctx.expect_contact(mount, yoke, elem_a="swivel_cap", elem_b="swivel_collar")
        ctx.expect_contact(yoke, head, elem_a="yoke_arm_left", elem_b="pivot_block")
        ctx.expect_contact(yoke, head, elem_a="yoke_arm_right", elem_b="pivot_block")
        ctx.expect_gap(
            head,
            rail,
            axis="z",
            min_gap=0.090,
            positive_elem="body_shell",
            negative_elem=rail_body,
        )

    mixed_pose = {
        left_swivel: 1.15,
        center_swivel: -0.95,
        right_swivel: 0.75,
        left_tilt: -0.30,
        center_tilt: 0.55,
        right_tilt: 0.25,
    }
    with ctx.pose(mixed_pose):
        ctx.fail_if_isolated_parts(name="mixed_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="mixed_pose_no_overlap")
        for mount, yoke, head, seat in (
            (left_mount, left_yoke, left_head, seat_left),
            (center_mount, center_yoke, center_head, seat_center),
            (right_mount, right_yoke, right_head, seat_right),
        ):
            ctx.expect_within(
                mount,
                rail,
                inner_elem="track_shoe",
                outer_elem=seat,
                axes="xy",
                name=f"{mount.name}_shoe_within_seat_mixed_pose",
            )
            ctx.expect_contact(
                rail,
                mount,
                elem_a=seat,
                elem_b="track_shoe",
                name=f"{mount.name}_shoe_contact_mixed_pose",
            )
            ctx.expect_contact(
                mount,
                yoke,
                elem_a="swivel_cap",
                elem_b="swivel_collar",
                name=f"{yoke.name}_swivel_contact_mixed_pose",
            )
            ctx.expect_contact(
                yoke,
                head,
                elem_a="yoke_arm_left",
                elem_b="pivot_block",
                name=f"{head.name}_left_pivot_contact_mixed_pose",
            )
            ctx.expect_contact(
                yoke,
                head,
                elem_a="yoke_arm_right",
                elem_b="pivot_block",
                name=f"{head.name}_right_pivot_contact_mixed_pose",
            )
            ctx.expect_gap(
                head,
                rail,
                axis="z",
                min_gap=0.090,
                positive_elem="body_shell",
                negative_elem=rail_body,
                name=f"{head.name}_rail_clearance_mixed_pose",
            )

    left_rest_center = _element_center(ctx, left_head, "body_shell")
    with ctx.pose({left_swivel: 1.35}):
        left_swiveled_center = _element_center(ctx, left_head, "body_shell")
    ctx.check(
        "left_swivel_changes_azimuth",
        left_rest_center is not None
        and left_swiveled_center is not None
        and abs(left_swiveled_center[1] - left_rest_center[1]) > 0.020,
        details=f"rest={left_rest_center}, swiveled={left_swiveled_center}",
    )

    center_rest_center = _element_center(ctx, center_head, "body_shell")
    with ctx.pose({center_tilt: 0.60}):
        center_raised_center = _element_center(ctx, center_head, "body_shell")
    with ctx.pose({center_tilt: -0.35}):
        center_dropped_center = _element_center(ctx, center_head, "body_shell")
    ctx.check(
        "center_tilt_can_raise_head",
        center_rest_center is not None
        and center_raised_center is not None
        and center_raised_center[2] < center_rest_center[2] - 0.010,
        details=f"rest={center_rest_center}, raised={center_raised_center}",
    )
    ctx.check(
        "center_tilt_can_drop_head",
        center_rest_center is not None
        and center_dropped_center is not None
        and center_dropped_center[2] > center_rest_center[2] + 0.006,
        details=f"rest={center_rest_center}, dropped={center_dropped_center}",
    )

    for articulation in (
        left_swivel,
        center_swivel,
        right_swivel,
        left_tilt,
        center_tilt,
        right_tilt,
    ):
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
            ctx.expect_gap(
                center_head if articulation is center_tilt else left_head if articulation is left_tilt else right_head if articulation is right_tilt else left_head,
                rail,
                axis="z",
                min_gap=0.085,
                positive_elem="body_shell",
                negative_elem=rail_body,
                name=f"{articulation.name}_lower_rail_gap",
            ) if articulation in (left_tilt, center_tilt, right_tilt) else None
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")
            ctx.expect_gap(
                center_head if articulation is center_tilt else left_head if articulation is left_tilt else right_head if articulation is right_tilt else left_head,
                rail,
                axis="z",
                min_gap=0.085,
                positive_elem="body_shell",
                negative_elem=rail_body,
                name=f"{articulation.name}_upper_rail_gap",
            ) if articulation in (left_tilt, center_tilt, right_tilt) else None

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
