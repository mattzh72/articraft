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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    radius: float, *, segments: int = 72, start_angle: float = 0.0
) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(start_angle + (2.0 * math.pi * index / segments)),
            radius * math.sin(start_angle + (2.0 * math.pi * index / segments)),
        )
        for index in range(segments)
    ]


def _ring_mesh(
    *,
    name: str,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    through_holes: list[tuple[float, float, float]] | None = None,
) -> object:
    hole_profiles = [_circle_profile(inner_radius, segments=56)]
    for cx, cy, radius in through_holes or []:
        hole_profiles.append(
            [(cx + x, cy + y) for x, y in _circle_profile(radius, segments=28)]
        )
    ring_geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=96),
        hole_profiles,
        height=thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(ring_geom, name)


def _crank_arm_mesh(name: str, *, thickness: float = 0.016) -> object:
    outline = [
        (0.020, 0.018),
        (0.018, 0.002),
        (0.015, -0.030),
        (0.012, -0.078),
        (0.011, -0.126),
        (0.013, -0.160),
        (0.010, -0.176),
        (0.006, -0.184),
        (-0.006, -0.184),
        (-0.010, -0.176),
        (-0.013, -0.160),
        (-0.011, -0.126),
        (-0.012, -0.078),
        (-0.015, -0.030),
        (-0.018, 0.002),
        (-0.020, 0.018),
        (-0.012, 0.026),
        (0.012, 0.026),
    ]
    arm_geom = ExtrudeGeometry.centered(outline, thickness).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(arm_geom, name)


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


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixie_crankset")

    alloy = model.material("alloy", rgba=(0.78, 0.80, 0.83, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.27, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.15, 0.15, 0.16, 1.0))
    reflector_amber = model.material("reflector_amber", rgba=(0.82, 0.46, 0.10, 1.0))

    shell_length = 0.0865
    shell_half = shell_length * 0.5
    shell_outer_radius = 0.024
    shell_end_bore_radius = 0.016
    shell_mid_bore_radius = 0.019

    shell_outer_profile = [
        (0.023, -shell_half),
        (shell_outer_radius, -shell_half + 0.003),
        (shell_outer_radius, shell_half - 0.003),
        (0.023, shell_half),
    ]
    shell_inner_profile = [
        (shell_end_bore_radius, -shell_half),
        (shell_end_bore_radius, -shell_half + 0.010),
        (shell_mid_bore_radius, -0.020),
        (shell_mid_bore_radius, 0.020),
        (shell_end_bore_radius, shell_half - 0.010),
        (shell_end_bore_radius, shell_half),
    ]
    shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            shell_outer_profile,
            shell_inner_profile,
            segments=88,
            start_cap="flat",
            end_cap="flat",
        ).rotate_x(math.pi / 2.0),
        "bb_shell",
    )

    crank_arm_mesh = _crank_arm_mesh("forged_crank_arm")
    chainring_mesh = _ring_mesh(
        name="track_chainring",
        outer_radius=0.094,
        inner_radius=0.038,
        thickness=0.004,
        through_holes=[
            (
                0.048 * math.cos((2.0 * math.pi * index) / 5.0),
                0.048 * math.sin((2.0 * math.pi * index) / 5.0),
                0.0045,
            )
            for index in range(5)
        ]
        + [
            (
                0.067 * math.cos((2.0 * math.pi * index) / 5.0 + math.pi / 5.0),
                0.067 * math.sin((2.0 * math.pi * index) / 5.0 + math.pi / 5.0),
                0.010,
            )
            for index in range(5)
        ],
    )

    bb_shell = model.part("bb_shell")
    bb_shell.visual(shell_mesh, material=matte_black, name="shell_body")
    bb_shell.visual(
        Cylinder(radius=0.0215, length=0.004),
        origin=Origin(xyz=(0.0, shell_half + 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="drive_side_dust_cover",
    )
    bb_shell.visual(
        Cylinder(radius=0.0215, length=0.004),
        origin=Origin(xyz=(0.0, -shell_half - 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="non_drive_side_dust_cover",
    )
    bb_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=shell_outer_radius, length=shell_length),
        mass=0.30,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.012, length=shell_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="spindle_axle",
    )
    spindle.visual(
        Cylinder(radius=0.021, length=0.003),
        origin=Origin(xyz=(0.0, shell_half + 0.0015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="drive_side_collar",
    )
    spindle.visual(
        Cylinder(radius=0.021, length=0.003),
        origin=Origin(xyz=(0.0, -shell_half - 0.0015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="non_drive_side_collar",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=shell_length),
        mass=0.45,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_crank = model.part("right_crank")
    right_crank.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="spindle_boss",
    )
    right_crank.visual(
        crank_arm_mesh,
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=alloy,
        name="arm_blade",
    )
    right_crank.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="spider_carrier",
    )
    spider_root = (0.0, 0.014, 0.0)
    for index in range(5):
        angle = (2.0 * math.pi * index) / 5.0
        bolt_center = (0.048 * math.cos(angle), 0.014, 0.048 * math.sin(angle))
        _add_member(
            right_crank,
            spider_root,
            bolt_center,
            0.004,
            alloy,
            name=f"spider_arm_{index}",
        )
        right_crank.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(
                xyz=(bolt_center[0], 0.012, bolt_center[2]),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=alloy,
            name=f"bolt_pad_{index}",
        )
    right_crank.visual(
        Cylinder(radius=0.0105, length=0.020),
        origin=Origin(xyz=(0.0, 0.018, -0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="pedal_eye",
    )
    right_crank.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(0.0, 0.035, -0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pedal_stub",
    )
    right_crank.inertial = Inertial.from_geometry(
        Box((0.040, 0.035, 0.210)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.010, -0.085)),
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="spindle_boss",
    )
    left_crank.visual(
        crank_arm_mesh,
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi, 0.0, 0.0)),
        material=alloy,
        name="arm_blade",
    )
    left_crank.visual(
        Cylinder(radius=0.0105, length=0.020),
        origin=Origin(xyz=(0.0, -0.018, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="pedal_eye",
    )
    left_crank.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(0.0, -0.035, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pedal_stub",
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.040, 0.035, 0.210)),
        mass=0.40,
        origin=Origin(xyz=(0.0, -0.010, 0.085)),
    )

    chainring = model.part("chainring")
    chainring.visual(
        chainring_mesh,
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=dark_steel,
        name="ring_plate",
    )
    for index in range(5):
        angle = (2.0 * math.pi * index) / 5.0
        bx = 0.048 * math.cos(angle)
        bz = 0.048 * math.sin(angle)
        chainring.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(
                xyz=(bx, 0.016, bz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_steel,
            name=f"bolt_{index}",
        )
    chainring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.094, length=0.004),
        mass=0.19,
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pedal_sleeve",
    )
    right_pedal.visual(
        Box((0.090, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
        material=pedal_black,
        name="platform_body",
    )
    right_pedal.visual(
        Box((0.012, 0.050, 0.018)),
        origin=Origin(xyz=(0.036, 0.045, 0.0)),
        material=pedal_black,
        name="front_cage",
    )
    right_pedal.visual(
        Box((0.012, 0.050, 0.018)),
        origin=Origin(xyz=(-0.036, 0.045, 0.0)),
        material=pedal_black,
        name="rear_cage",
    )
    right_pedal.visual(
        Box((0.012, 0.004, 0.016)),
        origin=Origin(xyz=(0.030, 0.063, 0.014)),
        material=reflector_amber,
        name="pedal_reflector",
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.090, 0.070, 0.024)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pedal_sleeve",
    )
    left_pedal.visual(
        Box((0.090, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        material=pedal_black,
        name="platform_body",
    )
    left_pedal.visual(
        Box((0.012, 0.050, 0.018)),
        origin=Origin(xyz=(0.036, -0.045, 0.0)),
        material=pedal_black,
        name="front_cage",
    )
    left_pedal.visual(
        Box((0.012, 0.050, 0.018)),
        origin=Origin(xyz=(-0.036, -0.045, 0.0)),
        material=pedal_black,
        name="rear_cage",
    )
    left_pedal.visual(
        Box((0.012, 0.004, 0.016)),
        origin=Origin(xyz=(0.030, -0.063, 0.014)),
        material=reflector_amber,
        name="pedal_reflector",
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.090, 0.070, 0.024)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
    )

    model.articulation(
        "shell_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=spindle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=18.0),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(xyz=(0.0, shell_half + 0.003, 0.0)),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(xyz=(0.0, -shell_half - 0.003, 0.0)),
    )
    model.articulation(
        "right_crank_to_chainring",
        ArticulationType.FIXED,
        parent=right_crank,
        child=chainring,
        origin=Origin(),
    )
    model.articulation(
        "right_crank_to_pedal",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child=right_pedal,
        origin=Origin(xyz=(0.0, 0.028, -0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=28.0),
    )
    model.articulation(
        "left_crank_to_pedal",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal,
        origin=Origin(xyz=(0.0, -0.028, 0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=28.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bb_shell = object_model.get_part("bb_shell")
    spindle = object_model.get_part("spindle")
    right_crank = object_model.get_part("right_crank")
    left_crank = object_model.get_part("left_crank")
    chainring = object_model.get_part("chainring")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    shell_to_spindle = object_model.get_articulation("shell_to_spindle")
    right_pedal_spin = object_model.get_articulation("right_crank_to_pedal")
    left_pedal_spin = object_model.get_articulation("left_crank_to_pedal")

    right_reflector = right_pedal.get_visual("pedal_reflector")
    left_reflector = left_pedal.get_visual("pedal_reflector")

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

    ctx.expect_contact(spindle, bb_shell, name="spindle_supported_in_shell")
    ctx.expect_contact(right_crank, spindle, name="right_crank_contacts_spindle")
    ctx.expect_contact(left_crank, spindle, name="left_crank_contacts_spindle")
    ctx.expect_contact(chainring, right_crank, name="chainring_bolted_to_right_crank")
    ctx.expect_contact(right_pedal, right_crank, name="right_pedal_contacts_right_crank")
    ctx.expect_contact(left_pedal, left_crank, name="left_pedal_contacts_left_crank")

    ctx.expect_overlap(chainring, right_crank, axes="xz", min_overlap=0.06)
    ctx.expect_origin_gap(chainring, bb_shell, axis="y", min_gap=0.04)
    ctx.expect_origin_gap(right_crank, bb_shell, axis="y", min_gap=0.045)
    ctx.expect_origin_gap(bb_shell, left_crank, axis="y", min_gap=0.045)

    ctx.check(
        "spindle_axis_is_lateral",
        tuple(shell_to_spindle.axis) == (0.0, 1.0, 0.0),
        details=f"Unexpected spindle axis: {shell_to_spindle.axis}",
    )
    ctx.check(
        "pedal_axes_match_spindle_axis",
        tuple(right_pedal_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(left_pedal_spin.axis) == (0.0, 1.0, 0.0),
        details=(
            f"Right pedal axis {right_pedal_spin.axis}, "
            f"left pedal axis {left_pedal_spin.axis}"
        ),
    )

    right_rest = ctx.part_world_position(right_pedal)
    left_rest = ctx.part_world_position(left_pedal)
    ctx.check(
        "rest_pose_has_opposed_cranks",
        right_rest is not None
        and left_rest is not None
        and right_rest[2] < -0.14
        and left_rest[2] > 0.14,
        details=f"right_rest={right_rest}, left_rest={left_rest}",
    )

    with ctx.pose({shell_to_spindle: math.pi / 2.0}):
        right_quarter = ctx.part_world_position(right_pedal)
        left_quarter = ctx.part_world_position(left_pedal)
        ctx.check(
            "quarter_turn_moves_cranks_around_spindle",
            right_quarter is not None
            and left_quarter is not None
            and right_quarter[0] < -0.14
            and abs(right_quarter[2]) < 0.05
            and left_quarter[0] > 0.14
            and abs(left_quarter[2]) < 0.05,
            details=f"right_quarter={right_quarter}, left_quarter={left_quarter}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="quarter_turn_no_floating")

    with ctx.pose({shell_to_spindle: math.pi}):
        right_half = ctx.part_world_position(right_pedal)
        left_half = ctx.part_world_position(left_pedal)
        ctx.check(
            "half_turn_swaps_pedal_heights",
            right_half is not None
            and left_half is not None
            and right_half[2] > 0.14
            and left_half[2] < -0.14,
            details=f"right_half={right_half}, left_half={left_half}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="half_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="half_turn_no_floating")

    right_reflector_rest_aabb = ctx.part_element_world_aabb(right_pedal, elem=right_reflector)
    left_reflector_rest_aabb = ctx.part_element_world_aabb(left_pedal, elem=left_reflector)
    with ctx.pose({right_pedal_spin: math.pi / 2.0, left_pedal_spin: -math.pi / 2.0}):
        right_reflector_rot_aabb = ctx.part_element_world_aabb(
            right_pedal, elem=right_reflector
        )
        left_reflector_rot_aabb = ctx.part_element_world_aabb(left_pedal, elem=left_reflector)
        right_rest_center = (
            _aabb_center(right_reflector_rest_aabb)
            if right_reflector_rest_aabb is not None
            else None
        )
        left_rest_center = (
            _aabb_center(left_reflector_rest_aabb)
            if left_reflector_rest_aabb is not None
            else None
        )
        right_rot_center = (
            _aabb_center(right_reflector_rot_aabb)
            if right_reflector_rot_aabb is not None
            else None
        )
        left_rot_center = (
            _aabb_center(left_reflector_rot_aabb)
            if left_reflector_rot_aabb is not None
            else None
        )
        ctx.check(
            "pedal_spin_moves_right_reflector",
            right_rest_center is not None
            and right_rot_center is not None
            and abs(right_rot_center[2] - right_rest_center[2]) > 0.01,
            details=f"right reflector centers: rest={right_rest_center}, rot={right_rot_center}",
        )
        ctx.check(
            "pedal_spin_moves_left_reflector",
            left_rest_center is not None
            and left_rot_center is not None
            and abs(left_rot_center[2] - left_rest_center[2]) > 0.01,
            details=f"left reflector centers: rest={left_rest_center}, rot={left_rot_center}",
        )
        ctx.expect_contact(right_pedal, right_crank, name="right_pedal_contact_when_spun")
        ctx.expect_contact(left_pedal, left_crank, name="left_pedal_contact_when_spun")
        ctx.fail_if_parts_overlap_in_current_pose(name="pedal_spin_no_overlap")
        ctx.fail_if_isolated_parts(name="pedal_spin_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
