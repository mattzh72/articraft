from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


TILT = math.radians(17.0)
STEER_AXIS = (0.0, -math.sin(TILT), math.cos(TILT))
HEAD_CENTER = (0.0, 0.0, 0.23)
STEERER_RADIUS = 0.0127


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _along_axis(distance: float) -> tuple[float, float, float]:
    return (
        HEAD_CENTER[0] + STEER_AXIS[0] * distance,
        HEAD_CENTER[1] + STEER_AXIS[1] * distance,
        HEAD_CENTER[2] + STEER_AXIS[2] * distance,
    )


def _axis_offset(distance: float) -> tuple[float, float, float]:
    return (
        STEER_AXIS[0] * distance,
        STEER_AXIS[1] * distance,
        STEER_AXIS[2] * distance,
    )


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _shell_mesh(outer_radius: float, inner_radius: float, length: float):
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -length * 0.5),
            (outer_radius, length * 0.5),
        ],
        [
            (inner_radius, -length * 0.5),
            (inner_radius, length * 0.5),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _profile_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=6)]


def _offset_profile_section(
    width: float,
    depth: float,
    radius: float,
    y_center: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [(x, y + y_center, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=6)]


def _crown_mesh():
    return section_loft(
        [
            _profile_section(0.098, 0.029, 0.008, 0.010),
            _profile_section(0.114, 0.041, 0.012, 0.000),
            _profile_section(0.090, 0.028, 0.007, -0.013),
        ]
    )


def _fork_blade_mesh(side: float):
    return tube_from_spline_points(
        [
            (side * 0.042, 0.031, -0.087),
            (side * 0.048, 0.060, -0.124),
            (side * 0.053, 0.090, -0.162),
            (side * 0.057, 0.117, -0.200),
            (side * 0.058, 0.126, -0.218),
        ],
        radius=0.0087,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def _handlebar_mesh():
    return tube_from_spline_points(
        [
            (-0.355, -0.010, 0.120),
            (-0.245, -0.002, 0.117),
            (-0.120, 0.020, 0.095),
            (-0.038, 0.040, 0.043),
            (0.038, 0.040, 0.043),
            (0.120, 0.020, 0.095),
            (0.245, -0.002, 0.117),
            (0.355, -0.010, 0.120),
        ],
        radius=0.0108,
        samples_per_segment=20,
        radial_segments=18,
        cap_ends=True,
    )


def _crossbar_mesh():
    return tube_from_spline_points(
        [
            (-0.120, 0.017, 0.077),
            (0.000, 0.021, 0.081),
            (0.120, 0.017, 0.077),
        ],
        radius=0.0068,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )


def _stem_body_mesh():
    return section_loft(
        [
            _offset_profile_section(0.030, 0.024, 0.006, 0.008, 0.024),
            _offset_profile_section(0.032, 0.030, 0.007, 0.020, 0.030),
            _offset_profile_section(0.038, 0.030, 0.007, 0.034, 0.036),
            _offset_profile_section(0.042, 0.024, 0.006, 0.046, 0.042),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_fork_front_end", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.10, 0.11, 0.12, 1.0))
    cr_mo_steel = model.material("cr_mo_steel", rgba=(0.23, 0.24, 0.27, 1.0))
    alloy_silver = model.material("alloy_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    headset_black = model.material("headset_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    headtube_frame = model.part("headtube_frame")
    headtube_frame.inertial = Inertial.from_geometry(
        Box((0.11, 0.22, 0.28)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.06, 0.20)),
    )
    head_tube_mesh = _save_mesh("bmx_head_tube.obj", _shell_mesh(0.0195, 0.0138, 0.110))
    headset_mesh = _save_mesh("bmx_headset_ring.obj", _shell_mesh(0.0220, 0.0146, 0.014))
    headtube_frame.visual(
        head_tube_mesh,
        origin=Origin(xyz=HEAD_CENTER, rpy=(TILT, 0.0, 0.0)),
        material=frame_paint,
        name="head_tube_shell",
    )
    headtube_frame.visual(
        headset_mesh,
        origin=Origin(xyz=_along_axis(0.062), rpy=(TILT, 0.0, 0.0)),
        material=headset_black,
        name="upper_headset",
    )
    headtube_frame.visual(
        headset_mesh,
        origin=Origin(xyz=_along_axis(-0.062), rpy=(TILT, 0.0, 0.0)),
        material=headset_black,
        name="lower_headset",
    )
    _add_member(
        headtube_frame,
        (0.0, -0.033, 0.268),
        (0.0, -0.160, 0.275),
        0.012,
        frame_paint,
        name="top_tube_stub",
    )
    _add_member(
        headtube_frame,
        (0.0, -0.022, 0.184),
        (0.0, -0.165, 0.100),
        0.017,
        frame_paint,
        name="down_tube_stub",
    )
    headtube_frame.visual(
        Box((0.028, 0.028, 0.030)),
        origin=Origin(xyz=(0.0, -0.020, 0.186), rpy=(0.35, 0.0, 0.0)),
        material=frame_paint,
        name="down_tube_gusset",
    )

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.16, 0.18, 0.43)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.055, 0.12)),
    )
    crown_mesh = _save_mesh("bmx_fork_crown.obj", _crown_mesh())
    left_blade_mesh = _save_mesh("bmx_left_blade.obj", _fork_blade_mesh(-1.0))
    right_blade_mesh = _save_mesh("bmx_right_blade.obj", _fork_blade_mesh(1.0))
    crown_race_mesh = _save_mesh("bmx_crown_race.obj", _shell_mesh(0.0149, STEERER_RADIUS, 0.006))
    fork.visual(
        Cylinder(radius=STEERER_RADIUS, length=0.210),
        origin=Origin(rpy=(TILT, 0.0, 0.0)),
        material=cr_mo_steel,
        name="steerer",
    )
    fork.visual(
        Cylinder(radius=STEERER_RADIUS * 0.96, length=0.094),
        origin=Origin(rpy=(TILT, 0.0, 0.0)),
        material=cr_mo_steel,
        name="bearing_span",
    )
    fork.visual(
        Cylinder(radius=STEERER_RADIUS * 0.96, length=0.032),
        origin=Origin(xyz=_axis_offset(0.092), rpy=(TILT, 0.0, 0.0)),
        material=cr_mo_steel,
        name="stem_clamp_span",
    )
    fork.visual(
        crown_race_mesh,
        origin=Origin(xyz=_axis_offset(-0.072), rpy=(TILT, 0.0, 0.0)),
        material=headset_black,
        name="crown_race",
    )
    fork.visual(
        crown_mesh,
        origin=Origin(xyz=(0.0, 0.028, -0.086)),
        material=cr_mo_steel,
        name="crown",
    )
    fork.visual(left_blade_mesh, material=cr_mo_steel, name="left_blade")
    fork.visual(right_blade_mesh, material=cr_mo_steel, name="right_blade")
    fork.visual(
        Box((0.013, 0.020, 0.010)),
        origin=Origin(xyz=(-0.058, 0.128, -0.220)),
        material=cr_mo_steel,
        name="left_dropout",
    )
    fork.visual(
        Box((0.013, 0.020, 0.010)),
        origin=Origin(xyz=(0.058, 0.128, -0.220)),
        material=cr_mo_steel,
        name="right_dropout",
    )

    cockpit = model.part("cockpit")
    cockpit.inertial = Inertial.from_geometry(
        Box((0.78, 0.12, 0.17)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.010, 0.085)),
    )
    stem_clamp_mesh = _save_mesh("bmx_stem_clamp.obj", _shell_mesh(0.0205, STEERER_RADIUS, 0.040))
    handlebar_mesh = _save_mesh("bmx_handlebar.obj", _handlebar_mesh())
    crossbar_mesh = _save_mesh("bmx_crossbar.obj", _crossbar_mesh())
    stem_body_mesh = _save_mesh("bmx_stem_body.obj", _stem_body_mesh())
    cockpit.visual(
        stem_clamp_mesh,
        origin=Origin(rpy=(TILT, 0.0, 0.0)),
        material=alloy_silver,
        name="steerer_clamp",
    )
    cockpit.visual(
        stem_body_mesh,
        material=alloy_silver,
        name="stem_body",
    )
    cockpit.visual(
        Cylinder(radius=0.017, length=0.064),
        origin=Origin(xyz=(0.0, 0.045, 0.042), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy_silver,
        name="bar_clamp",
    )
    cockpit.visual(
        Box((0.042, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, 0.060, 0.042)),
        material=alloy_silver,
        name="face_plate",
    )
    cockpit.visual(handlebar_mesh, material=cr_mo_steel, name="handlebar")
    cockpit.visual(crossbar_mesh, material=cr_mo_steel, name="crossbar")
    cockpit.visual(
        Cylinder(radius=0.015, length=0.100),
        origin=Origin(xyz=(-0.312, -0.010, 0.120), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    cockpit.visual(
        Cylinder(radius=0.015, length=0.100),
        origin=Origin(xyz=(0.312, -0.010, 0.120), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )

    model.articulation(
        "steerer_rotation",
        ArticulationType.REVOLUTE,
        parent=headtube_frame,
        child=fork,
        origin=Origin(xyz=HEAD_CENTER),
        axis=STEER_AXIS,
        motion_limits=MotionLimits(effort=22.0, velocity=2.8, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "stem_mount",
        ArticulationType.FIXED,
        parent=fork,
        child=cockpit,
        origin=Origin(xyz=_axis_offset(0.092)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    headtube_frame = object_model.get_part("headtube_frame")
    fork = object_model.get_part("fork")
    cockpit = object_model.get_part("cockpit")
    steering = object_model.get_articulation("steerer_rotation")

    head_tube_shell = headtube_frame.get_visual("head_tube_shell")
    lower_headset = headtube_frame.get_visual("lower_headset")
    steerer = fork.get_visual("steerer")
    bearing_span = fork.get_visual("bearing_span")
    stem_clamp_span = fork.get_visual("stem_clamp_span")
    crown_race = fork.get_visual("crown_race")
    crown = fork.get_visual("crown")
    stem_clamp = cockpit.get_visual("steerer_clamp")
    bar_clamp = cockpit.get_visual("bar_clamp")
    stem_body = cockpit.get_visual("stem_body")
    handlebar = cockpit.get_visual("handlebar")
    crossbar = cockpit.get_visual("crossbar")
    left_grip = cockpit.get_visual("left_grip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        headtube_frame,
        fork,
        elem_a=lower_headset,
        elem_b=crown_race,
        reason="the crown race is modeled as a nested bearing seat inside the lower headset cup",
    )
    ctx.allow_overlap(
        fork,
        cockpit,
        elem_a=steerer,
        elem_b=stem_clamp,
        reason="the split stem clamp is represented as a lightly preloaded grip around the steerer tube",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)
    ctx.expect_contact(
        fork,
        cockpit,
        elem_a=steerer,
        elem_b=stem_clamp,
        name="stem_physically_grips_the_steerer",
    )
    ctx.expect_within(
        fork,
        headtube_frame,
        axes="xy",
        inner_elem=bearing_span,
        outer_elem=head_tube_shell,
        name="steerer_runs_within_short_head_tube",
    )
    ctx.expect_within(
        fork,
        cockpit,
        axes="xy",
        inner_elem=stem_clamp_span,
        outer_elem=stem_clamp,
        name="short_stem_clamps_steerer",
    )
    ctx.expect_contact(
        fork,
        headtube_frame,
        elem_a=crown_race,
        elem_b=lower_headset,
        name="fork_seats_in_lower_headset",
    )
    ctx.expect_contact(
        cockpit,
        cockpit,
        elem_a=stem_body,
        elem_b=bar_clamp,
        name="short_stem_body_reaches_the_bar_clamp",
    )
    ctx.expect_overlap(
        cockpit,
        headtube_frame,
        axes="xy",
        min_overlap=0.015,
        elem_a=stem_body,
        elem_b=head_tube_shell,
        name="stem_stays_centered_over_head_tube",
    )
    ctx.expect_within(
        headtube_frame,
        cockpit,
        axes="x",
        inner_elem=head_tube_shell,
        outer_elem=handlebar,
        name="wide_riser_bars_span_far_beyond_head_tube",
    )
    ctx.expect_overlap(
        cockpit,
        headtube_frame,
        axes="x",
        min_overlap=0.035,
        elem_a=crossbar,
        elem_b=head_tube_shell,
        name="crossbar_braces_the_rise_above_the_head_tube",
    )
    ctx.expect_gap(
        headtube_frame,
        fork,
        axis="z",
        max_gap=0.050,
        max_penetration=0.0,
        positive_elem=head_tube_shell,
        negative_elem=crown,
        name="crown_sits_just_below_short_head_tube",
    )
    ctx.expect_gap(
        cockpit,
        headtube_frame,
        axis="z",
        min_gap=0.090,
        positive_elem=left_grip,
        negative_elem=head_tube_shell,
        name="riser_bar_grips_sit_well_above_the_head_tube",
    )

    steerer_geom = steerer.geometry
    ctx.check(
        "steerer_is_one_inch_chromoly_scale",
        isinstance(steerer_geom, Cylinder) and abs(steerer_geom.radius * 2.0 - 0.0254) < 1e-6,
        f"expected a 1-inch steerer tube, got radius={getattr(steerer_geom, 'radius', None)!r}",
    )
    limits = steering.motion_limits
    ctx.check(
        "steering_axis_matches_head_tube_rake",
        steering.axis[0] == 0.0 and steering.axis[1] < -0.2 and steering.axis[2] > 0.9,
        f"unexpected steering axis {steering.axis!r}",
    )
    ctx.check(
        "steering_range_is_bmx_realistic",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower + 0.70) < 1e-6
        and abs(limits.upper - 0.70) < 1e-6,
        f"unexpected steering limits {limits!r}",
    )

    head_tube_aabb = ctx.part_element_world_aabb(headtube_frame, elem=head_tube_shell)
    handlebar_aabb = ctx.part_element_world_aabb(cockpit, elem=handlebar)
    ctx.check(
        "head_tube_reads_as_short",
        head_tube_aabb is not None and (head_tube_aabb[1][2] - head_tube_aabb[0][2]) <= 0.12,
        f"head tube AABB was {head_tube_aabb!r}",
    )
    ctx.check(
        "riser_bars_are_wide",
        handlebar_aabb is not None and (handlebar_aabb[1][0] - handlebar_aabb[0][0]) >= 0.70,
        f"handlebar AABB was {handlebar_aabb!r}",
    )
    ctx.check(
        "stem_is_short",
        0.035 <= bar_clamp.origin.xyz[1] <= 0.055,
        f"bar clamp offset was {bar_clamp.origin.xyz!r}",
    )
    with ctx.pose({steering: 0.55}):
        ctx.expect_within(
            fork,
            headtube_frame,
            axes="xy",
            inner_elem=bearing_span,
            outer_elem=head_tube_shell,
            name="steerer_remains_nested_when_turned",
        )
        ctx.expect_within(
            fork,
            cockpit,
            axes="xy",
            inner_elem=stem_clamp_span,
            outer_elem=stem_clamp,
            name="stem_remains_clamped_in_steered_pose",
        )
        ctx.expect_origin_distance(
            cockpit,
            fork,
            axes="xy",
            max_dist=0.035,
            name="stem_tracks_the_steering_axis_in_pose",
        )
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({steering: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_lower_no_unallowed_overlap")
            ctx.fail_if_isolated_parts(name="steering_lower_no_floating")
            ctx.expect_within(
                fork,
                headtube_frame,
                axes="xy",
                inner_elem=bearing_span,
                outer_elem=head_tube_shell,
                name="lower_lock_steerer_stays_in_head_tube",
            )
        with ctx.pose({steering: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_upper_no_unallowed_overlap")
            ctx.fail_if_isolated_parts(name="steering_upper_no_floating")
            ctx.expect_within(
                fork,
                headtube_frame,
                axes="xy",
                inner_elem=bearing_span,
                outer_elem=head_tube_shell,
                name="upper_lock_steerer_stays_in_head_tube",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
