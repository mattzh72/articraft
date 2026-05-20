from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


RIM_MAJOR_RADIUS = 0.024
RIM_WIRE_RADIUS = 0.00135
LENS_CENTER_X = 0.032
HINGE_X = 0.0595
HINGE_Y_OFFSET = 0.0024
HINGE_BARREL_RADIUS = 0.00215
HINGE_BARREL_LENGTH = 0.0045
TEMPLE_KNUCKLE_LENGTH = 0.006
HINGE_BARREL_Z = 0.00525
TEMPLE_FOLD_ANGLE = math.radians(95.0)
HINGE_AXIS_TILT = 0.33


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _circle_points(
    center_x: float,
    radius: float,
    *,
    segments: int = 24,
) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(segments):
        angle = (2.0 * math.pi * i) / segments
        pts.append((center_x + radius * math.cos(angle), 0.0, radius * math.sin(angle)))
    return pts


def _temple_arm_points(side_sign: float, z_sign: float) -> list[tuple[float, float, float]]:
    return [
        (side_sign * 0.0015, -0.0010, 0.0000),
        (side_sign * 0.0060, -0.0150, z_sign * 0.0014),
        (side_sign * 0.0150, -0.0460, z_sign * 0.0036),
        (side_sign * 0.0260, -0.0860, z_sign * 0.0056),
        (side_sign * 0.0340, -0.1220, z_sign * 0.0066),
        (side_sign * 0.0390, -0.1450, z_sign * 0.0060),
    ]


def _temple_tip_points(side_sign: float, z_sign: float) -> list[tuple[float, float, float]]:
    return [
        (side_sign * 0.0290, -0.1080, z_sign * 0.0056),
        (side_sign * 0.0340, -0.1250, z_sign * 0.0067),
        (side_sign * 0.0390, -0.1450, z_sign * 0.0058),
    ]


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_rod_visual(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_polyline_rods(
    part,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    material,
    closed: bool = False,
    name_prefix: str | None = None,
) -> None:
    stop = len(points) if closed else len(points) - 1
    for i in range(stop):
        a = points[i]
        b = points[(i + 1) % len(points)]
        seg_name = None if name_prefix is None else f"{name_prefix}_{i:02d}"
        _add_rod_visual(part, a, b, radius=radius, material=material, name=seg_name)


def _normalized(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = math.sqrt(sum(v * v for v in vec))
    return (vec[0] / mag, vec[1] / mag, vec[2] / mag)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wire_rim_glasses")

    gunmetal = model.material("gunmetal", rgba=(0.34, 0.35, 0.38, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.56, 0.57, 0.60, 1.0))
    temple_tip_black = model.material("temple_tip_black", rgba=(0.11, 0.11, 0.12, 1.0))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.126, 0.016, 0.056)),
        mass=0.028,
        origin=Origin(),
    )

    left_rim_points = _circle_points(-LENS_CENTER_X, RIM_MAJOR_RADIUS, segments=24)
    right_rim_points = _mirror_x(left_rim_points)
    _add_polyline_rods(
        front_frame,
        left_rim_points,
        radius=RIM_WIRE_RADIUS,
        material=gunmetal,
        closed=True,
        name_prefix="left_rim",
    )
    _add_polyline_rods(
        front_frame,
        right_rim_points,
        radius=RIM_WIRE_RADIUS,
        material=gunmetal,
        closed=True,
        name_prefix="right_rim",
    )

    bridge_points = [
        left_rim_points[0],
        (-0.0040, 0.0, 0.0080),
        (0.0000, 0.0, 0.0105),
        (0.0040, 0.0, 0.0080),
        right_rim_points[0],
    ]
    _add_polyline_rods(
        front_frame,
        bridge_points,
        radius=0.00125,
        material=gunmetal,
        closed=False,
        name_prefix="bridge",
    )

    left_nose_top = (-0.0040, 0.0, 0.0080)
    left_nose_mid = (-0.0046, -0.0016, 0.0010)
    left_nose_bottom = (-0.0030, -0.0038, -0.0064)
    right_nose_top = _mirror_x([left_nose_top])[0]
    right_nose_mid = _mirror_x([left_nose_mid])[0]
    right_nose_bottom = _mirror_x([left_nose_bottom])[0]
    _add_rod_visual(
        front_frame,
        left_nose_top,
        left_nose_mid,
        radius=0.0007,
        material=gunmetal,
        name="left_nose_bracket_upper",
    )
    _add_rod_visual(
        front_frame,
        left_nose_mid,
        left_nose_bottom,
        radius=0.0007,
        material=gunmetal,
        name="left_nose_bracket_lower",
    )
    _add_rod_visual(
        front_frame,
        right_nose_top,
        right_nose_mid,
        radius=0.0007,
        material=gunmetal,
        name="right_nose_bracket_upper",
    )
    _add_rod_visual(
        front_frame,
        right_nose_mid,
        right_nose_bottom,
        radius=0.0007,
        material=gunmetal,
        name="right_nose_bracket_lower",
    )

    for side_name, x_pos, y_pos in (
        ("left", -HINGE_X, -HINGE_Y_OFFSET),
        ("right", HINGE_X, HINGE_Y_OFFSET),
    ):
        front_frame.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
            origin=Origin(xyz=(x_pos, y_pos, HINGE_BARREL_Z)),
            material=hinge_steel,
            name=f"{side_name}_upper_barrel",
        )
        front_frame.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH),
            origin=Origin(xyz=(x_pos, y_pos, -HINGE_BARREL_Z)),
            material=hinge_steel,
            name=f"{side_name}_lower_barrel",
        )

    left_upper_anchor = left_rim_points[11]
    left_lower_anchor = left_rim_points[13]
    right_upper_anchor = right_rim_points[13]
    right_lower_anchor = right_rim_points[11]
    _add_rod_visual(
        front_frame,
        left_upper_anchor,
        (-HINGE_X, -HINGE_Y_OFFSET, HINGE_BARREL_Z),
        radius=0.0009,
        material=gunmetal,
        name="left_hinge_strap_upper",
    )
    _add_rod_visual(
        front_frame,
        left_lower_anchor,
        (-HINGE_X, -HINGE_Y_OFFSET, -HINGE_BARREL_Z),
        radius=0.0009,
        material=gunmetal,
        name="left_hinge_strap_lower",
    )
    _add_rod_visual(
        front_frame,
        right_upper_anchor,
        (HINGE_X, HINGE_Y_OFFSET, HINGE_BARREL_Z),
        radius=0.0009,
        material=gunmetal,
        name="right_hinge_strap_upper",
    )
    _add_rod_visual(
        front_frame,
        right_lower_anchor,
        (HINGE_X, HINGE_Y_OFFSET, -HINGE_BARREL_Z),
        radius=0.0009,
        material=gunmetal,
        name="right_hinge_strap_lower",
    )

    left_temple = model.part("left_temple")
    left_temple.inertial = Inertial.from_geometry(
        Box((0.050, 0.150, 0.040)),
        mass=0.009,
        origin=Origin(xyz=(-0.020, -0.075, -0.010)),
    )
    left_temple.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS * 0.96, length=TEMPLE_KNUCKLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0042)),
        material=hinge_steel,
        name="knuckle",
    )
    left_arm_points = _temple_arm_points(-1.0, 1.0)
    _add_polyline_rods(
        left_temple,
        left_arm_points[:-1],
        radius=0.00155,
        material=gunmetal,
        closed=False,
        name_prefix="left_arm_wire",
    )
    _add_rod_visual(
        left_temple,
        left_arm_points[-2],
        left_arm_points[-1],
        radius=0.00205,
        material=temple_tip_black,
        name="tip_cover",
    )

    right_temple = model.part("right_temple")
    right_temple.inertial = Inertial.from_geometry(
        Box((0.050, 0.150, 0.040)),
        mass=0.009,
        origin=Origin(xyz=(0.020, -0.075, -0.010)),
    )
    right_temple.visual(
        Cylinder(radius=HINGE_BARREL_RADIUS * 0.96, length=TEMPLE_KNUCKLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -0.0042)),
        material=hinge_steel,
        name="knuckle",
    )
    right_arm_points = _temple_arm_points(1.0, -1.0)
    _add_polyline_rods(
        right_temple,
        right_arm_points[:-1],
        radius=0.00155,
        material=gunmetal,
        closed=False,
        name_prefix="right_arm_wire",
    )
    _add_rod_visual(
        right_temple,
        right_arm_points[-2],
        right_arm_points[-1],
        radius=0.00205,
        material=temple_tip_black,
        name="tip_cover",
    )

    model.articulation(
        "left_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_temple,
        origin=Origin(xyz=(-HINGE_X, -HINGE_Y_OFFSET, 0.0)),
        axis=_normalized((0.0, HINGE_AXIS_TILT, 1.0)),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=6.0,
            lower=0.0,
            upper=TEMPLE_FOLD_ANGLE,
        ),
    )
    model.articulation(
        "right_temple_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_temple,
        origin=Origin(xyz=(HINGE_X, HINGE_Y_OFFSET, 0.0)),
        axis=_normalized((0.0, -HINGE_AXIS_TILT, 1.0)),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=6.0,
            lower=-TEMPLE_FOLD_ANGLE,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    front_frame = object_model.get_part("front_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_hinge = object_model.get_articulation("left_temple_hinge")
    right_hinge = object_model.get_articulation("right_temple_hinge")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="hinge_pose_sweep_clearance")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    ctx.check(
        "left_hinge_axis_realistic",
        abs(left_hinge.axis[0]) < 1e-6
        and left_hinge.axis[1] > 0.25
        and left_hinge.axis[2] > 0.90,
        f"left hinge axis is {left_hinge.axis}",
    )
    ctx.check(
        "right_hinge_axis_realistic",
        abs(right_hinge.axis[0]) < 1e-6
        and right_hinge.axis[1] < -0.25
        and right_hinge.axis[2] > 0.90,
        f"right hinge axis is {right_hinge.axis}",
    )

    left_limits = left_hinge.motion_limits
    right_limits = right_hinge.motion_limits
    ctx.check(
        "left_hinge_folds_inward",
        left_limits is not None
        and left_limits.lower == 0.0
        and left_limits.upper is not None
        and left_limits.upper > 1.5,
        f"left limits={left_limits}",
    )
    ctx.check(
        "right_hinge_folds_inward",
        right_limits is not None
        and right_limits.upper == 0.0
        and right_limits.lower is not None
        and right_limits.lower < -1.5,
        f"right limits={right_limits}",
    )

    frame_aabb = ctx.part_world_aabb(front_frame)
    if frame_aabb is None:
        ctx.fail("front_frame_has_geometry", "front frame AABB is unavailable")
    else:
        frame_width = frame_aabb[1][0] - frame_aabb[0][0]
        frame_height = frame_aabb[1][2] - frame_aabb[0][2]
        frame_depth = frame_aabb[1][1] - frame_aabb[0][1]
        ctx.check(
            "front_frame_width_realistic",
            0.115 <= frame_width <= 0.135,
            f"frame width {frame_width:.4f} m",
        )
        ctx.check(
            "front_frame_height_realistic",
            0.045 <= frame_height <= 0.060,
            f"frame height {frame_height:.4f} m",
        )
        ctx.check(
            "front_frame_is_thin_wire_frame",
            frame_depth <= 0.015,
            f"frame depth {frame_depth:.4f} m",
        )

    ctx.expect_contact(front_frame, left_temple, name="left_hinge_contact_open")
    ctx.expect_contact(front_frame, right_temple, name="right_hinge_contact_open")

    left_tip_open = _aabb_center(ctx.part_element_world_aabb(left_temple, elem="tip_cover"))
    right_tip_open = _aabb_center(ctx.part_element_world_aabb(right_temple, elem="tip_cover"))
    if left_tip_open is None:
        ctx.fail("left_tip_present", "left temple tip visual is missing")
    else:
        ctx.check(
            "left_tip_extends_backward_open",
            left_tip_open[1] < -0.11,
            f"left tip center {left_tip_open}",
        )
        ctx.check(
            "left_tip_outboard_open",
            left_tip_open[0] < -0.08,
            f"left tip center {left_tip_open}",
        )
    if right_tip_open is None:
        ctx.fail("right_tip_present", "right temple tip visual is missing")
    else:
        ctx.check(
            "right_tip_extends_backward_open",
            right_tip_open[1] < -0.10,
            f"right tip center {right_tip_open}",
        )
        ctx.check(
            "right_tip_outboard_open",
            right_tip_open[0] > 0.08,
            f"right tip center {right_tip_open}",
        )

    for hinge, temple, folded_value, side_name in (
        (
            left_hinge,
            left_temple,
            left_limits.upper if left_limits is not None else None,
            "left",
        ),
        (
            right_hinge,
            right_temple,
            right_limits.lower if right_limits is not None else None,
            "right",
        ),
    ):
        limits = hinge.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({hinge: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{side_name}_hinge_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{side_name}_hinge_lower_no_floating")
                ctx.expect_contact(front_frame, temple, name=f"{side_name}_hinge_contact_lower")
            with ctx.pose({hinge: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{side_name}_hinge_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{side_name}_hinge_upper_no_floating")
                ctx.expect_contact(front_frame, temple, name=f"{side_name}_hinge_contact_upper")

        if folded_value is not None:
            with ctx.pose({hinge: folded_value}):
                tip_center = _aabb_center(ctx.part_element_world_aabb(temple, elem="tip_cover"))
                if tip_center is None:
                    ctx.fail(f"{side_name}_tip_folded_present", "tip visual unavailable in folded pose")
                else:
                    if side_name == "left":
                        ctx.check(
                            "left_tip_swings_inboard_folded",
                            tip_center[0] > 0.02,
                            f"left folded tip center {tip_center}",
                        )
                    else:
                        ctx.check(
                            "right_tip_swings_inboard_folded",
                            tip_center[0] < -0.02,
                            f"right folded tip center {tip_center}",
                        )
                    ctx.check(
                        f"{side_name}_tip_near_frame_plane_folded",
                        abs(tip_center[1]) < 0.05,
                        f"{side_name} folded tip center {tip_center}",
                    )

    if left_limits is not None and right_limits is not None:
        with ctx.pose(
            {
                left_hinge: left_limits.upper,
                right_hinge: right_limits.lower,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="both_temples_folded_no_overlap")
            ctx.fail_if_isolated_parts(name="both_temples_folded_no_floating")
            ctx.expect_contact(front_frame, left_temple, name="left_hinge_contact_both_folded")
            ctx.expect_contact(front_frame, right_temple, name="right_hinge_contact_both_folded")

            left_tip_folded = _aabb_center(ctx.part_element_world_aabb(left_temple, elem="tip_cover"))
            right_tip_folded = _aabb_center(ctx.part_element_world_aabb(right_temple, elem="tip_cover"))
            if left_tip_folded is not None and right_tip_folded is not None:
                ctx.check(
                    "folded_temples_stack_on_opposite_sides_of_frame",
                    (left_tip_folded[1] + 0.001) < right_tip_folded[1],
                    f"left folded tip={left_tip_folded}, right folded tip={right_tip_folded}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
