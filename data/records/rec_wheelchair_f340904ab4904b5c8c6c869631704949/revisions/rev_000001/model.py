from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v):
    return math.sqrt(_dot(v, v))


def _segment(p0, p1, radius, radial_segments=24):
    direction = _vec_sub(p1, p0)
    length = _norm(direction)
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments)
    if length > 1e-9:
        unit = (direction[0] / length, direction[1] / length, direction[2] / length)
        z_axis = (0.0, 0.0, 1.0)
        axis = _cross(z_axis, unit)
        axis_len = _norm(axis)
        if axis_len < 1e-9:
            if unit[2] < 0.0:
                geom.rotate_x(math.pi)
        else:
            geom.rotate(
                (axis[0] / axis_len, axis[1] / axis_len, axis[2] / axis_len),
                math.acos(max(-1.0, min(1.0, unit[2]))),
            )
    midpoint = _vec_mul(_vec_add(p0, p1), 0.5)
    geom.translate(*midpoint)
    return geom


def _box(size, center, rpy=(0.0, 0.0, 0.0)):
    geom = BoxGeometry(size)
    if abs(rpy[0]) > 1e-9:
        geom.rotate_x(rpy[0])
    if abs(rpy[1]) > 1e-9:
        geom.rotate_y(rpy[1])
    if abs(rpy[2]) > 1e-9:
        geom.rotate_z(rpy[2])
    geom.translate(*center)
    return geom


def _cylinder_y(radius, length, center, radial_segments=24):
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(*center)
    return geom


def _cylinder_x(radius, length, center, radial_segments=24):
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments)
    geom.rotate_y(math.pi / 2.0)
    geom.translate(*center)
    return geom


def _torus_y(radius, tube, center, radial_segments=18, tubular_segments=36):
    geom = TorusGeometry(
        radius,
        tube,
        radial_segments=radial_segments,
        tubular_segments=tubular_segments,
    )
    geom.rotate_x(math.pi / 2.0)
    geom.translate(*center)
    return geom


def _build_frame_metal():
    tube_r = 0.016
    brace_r = 0.012
    armrest_r = 0.010
    parts = []

    # Main welded side frames.
    for y in (-0.185, 0.185):
        parts.append(_segment((-0.20, y, 0.34), (-0.20, y, 0.88), tube_r))
        parts.append(_segment((-0.20, y, 0.88), (-0.25, y, 0.93), tube_r))
        parts.append(_segment((-0.20, y, 0.49), (0.23, y, 0.49), tube_r))
        parts.append(_segment((0.23, y, 0.49), (0.34, 0.15 if y > 0 else -0.15, 0.27), tube_r))
        parts.append(_segment((-0.11, y, 0.49), (-0.03, 0.205 if y > 0 else -0.205, 0.34), brace_r))
        parts.append(_segment((-0.12, y, 0.49), (-0.12, 0.215 if y > 0 else -0.215, 0.62), armrest_r))
        parts.append(_segment((0.10, y, 0.49), (0.10, 0.215 if y > 0 else -0.215, 0.62), armrest_r))
        parts.append(_segment((0.18, 0.11 if y > 0 else -0.11, 0.49), (0.23, 0.11 if y > 0 else -0.11, 0.44), brace_r))

    # Crossmembers and service braces.
    parts.append(_cylinder_y(tube_r, 0.37, (-0.20, 0.0, 0.74)))
    parts.append(_cylinder_y(tube_r, 0.32, (0.17, 0.0, 0.49)))
    parts.append(_cylinder_y(brace_r, 0.41, (-0.03, 0.0, 0.34)))

    # Axle mounts and front caster housings.
    parts.append(_cylinder_y(0.020, 0.060, (-0.03, 0.247, 0.34)))
    parts.append(_cylinder_y(0.020, 0.060, (-0.03, -0.247, 0.34)))
    parts.append(_box((0.060, 0.050, 0.080), (-0.055, 0.212, 0.385)))
    parts.append(_box((0.060, 0.050, 0.080), (-0.055, -0.212, 0.385)))
    parts.append(_box((0.032, 0.024, 0.044), (0.192, 0.110, 0.464)))
    parts.append(_box((0.032, 0.024, 0.044), (0.192, -0.110, 0.464)))
    parts.append(_segment((0.192, 0.110, 0.464), (0.170, 0.000, 0.490), brace_r))
    parts.append(_segment((0.192, -0.110, 0.464), (0.170, 0.000, 0.490), brace_r))
    parts.append(_segment((0.34, 0.15, 0.278), (0.34, 0.15, 0.220), 0.016))
    parts.append(_segment((0.34, -0.15, 0.278), (0.34, -0.15, 0.220), 0.016))
    parts.append(_box((0.034, 0.050, 0.024), (0.34, 0.15, 0.244)))
    parts.append(_box((0.034, 0.050, 0.024), (0.34, -0.15, 0.244)))

    # Footrest hinge barrels.
    parts.append(CylinderGeometry(0.012, 0.030).translate(0.23, 0.110, 0.44))
    parts[-1].translate(0.0, 0.0, 0.0)
    parts.append(CylinderGeometry(0.012, 0.030).translate(0.23, -0.110, 0.44))
    parts[-1].translate(0.0, 0.0, 0.0)

    # Armrest pads supports.
    parts.append(_segment((-0.12, 0.215, 0.62), (0.10, 0.215, 0.62), 0.008))
    parts.append(_segment((-0.12, -0.215, 0.62), (0.10, -0.215, 0.62), 0.008))

    frame_geom = parts[0]
    for geom in parts[1:]:
        frame_geom.merge(geom)
    return frame_geom


def _build_drive_wheel(side_sign):
    geom = _cylinder_y(0.060, 0.032, (0.0, 0.0, 0.0), radial_segments=28)
    geom.merge(_cylinder_y(0.050, 0.022, (0.0, -side_sign * 0.019, 0.0), radial_segments=24))
    geom.merge(_cylinder_y(0.088, 0.014, (0.0, 0.0, 0.0), radial_segments=28))
    geom.merge(_torus_y(0.260, 0.018, (0.0, 0.0, 0.0), radial_segments=18, tubular_segments=56))
    geom.merge(_torus_y(0.286, 0.024, (0.0, 0.0, 0.0), radial_segments=20, tubular_segments=56))
    geom.merge(_torus_y(0.282, 0.007, (0.0, side_sign * 0.035, 0.0), radial_segments=16, tubular_segments=48))

    for i in range(6):
        ang = i * math.tau / 6.0
        p0 = (0.050 * math.cos(ang), 0.0, 0.050 * math.sin(ang))
        p1 = (0.262 * math.cos(ang), 0.0, 0.262 * math.sin(ang))
        geom.merge(_segment(p0, p1, 0.011, radial_segments=18))

    for ang in (0.0, math.pi / 3.0, 2.0 * math.pi / 3.0, math.pi, 4.0 * math.pi / 3.0, 5.0 * math.pi / 3.0):
        rim_point = (0.258 * math.cos(ang), side_sign * 0.006, 0.258 * math.sin(ang))
        rail_point = (0.279 * math.cos(ang), side_sign * 0.035, 0.279 * math.sin(ang))
        geom.merge(_segment(rim_point, rail_point, 0.006, radial_segments=16))

    return geom


def _build_caster_fork():
    geom = CylinderGeometry(0.016, 0.018)
    geom.translate(0.0, 0.0, -0.009)
    geom.merge(_segment((0.0, 0.0, -0.009), (0.0, 0.0, -0.070), 0.010, radial_segments=18))
    geom.merge(_segment((0.0, 0.0, -0.070), (-0.030, 0.0, -0.092), 0.010, radial_segments=18))
    geom.merge(_segment((-0.030, 0.0, -0.092), (-0.030, 0.028, -0.092), 0.0085, radial_segments=16))
    geom.merge(_segment((-0.030, 0.0, -0.092), (-0.030, -0.028, -0.092), 0.0085, radial_segments=16))
    geom.merge(_segment((-0.030, 0.028, -0.092), (-0.030, 0.028, -0.168), 0.0085, radial_segments=16))
    geom.merge(_segment((-0.030, -0.028, -0.092), (-0.030, -0.028, -0.168), 0.0085, radial_segments=16))
    geom.merge(_box((0.020, 0.012, 0.020), (-0.030, 0.021, -0.168)))
    geom.merge(_box((0.020, 0.012, 0.020), (-0.030, -0.021, -0.168)))
    return geom


def _build_caster_wheel():
    geom = _cylinder_y(0.020, 0.018, (0.0, 0.0, 0.0), radial_segments=24)
    geom.merge(_cylinder_y(0.006, 0.030, (0.0, 0.0, 0.0), radial_segments=18))
    geom.merge(_torus_y(0.046, 0.010, (0.0, 0.0, 0.0), radial_segments=16, tubular_segments=36))
    geom.merge(_torus_y(0.052, 0.014, (0.0, 0.0, 0.0), radial_segments=16, tubular_segments=36))
    for i in range(3):
        ang = i * math.tau / 3.0
        p0 = (0.016 * math.cos(ang), 0.0, 0.016 * math.sin(ang))
        p1 = (0.048 * math.cos(ang), 0.0, 0.048 * math.sin(ang))
        geom.merge(_segment(p0, p1, 0.006, radial_segments=14))
    return geom


def _build_footrest_hardware():
    geom = _box((0.020, 0.024, 0.060), (0.022, 0.0, -0.002))
    geom.merge(_segment((0.028, 0.0, -0.028), (0.065, 0.0, -0.155), 0.010, radial_segments=16))
    geom.merge(_segment((0.065, 0.0, -0.155), (0.215, 0.0, -0.262), 0.011, radial_segments=16))
    geom.merge(_cylinder_y(0.012, 0.086, (0.215, 0.0, -0.272), radial_segments=16))
    geom.merge(_segment((0.215, 0.0, -0.272), (0.215, 0.0, -0.286), 0.017, radial_segments=16))
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_wheelchair")

    frame_metal = model.material("frame_metal", rgba=(0.24, 0.26, 0.29, 1.0))
    upholstery = model.material("upholstery", rgba=(0.12, 0.12, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    alloy = model.material("alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.16, 0.16, 0.17, 1.0))
    tread_pad = model.material("tread_pad", rgba=(0.11, 0.11, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(_build_frame_metal(), "frame_metalwork"),
        material=frame_metal,
        name="frame_metalwork",
    )
    frame.visual(
        Box((0.40, 0.40, 0.012)),
        origin=Origin(xyz=(0.01, 0.0, 0.512)),
        material=upholstery,
        name="seat_sling",
    )
    frame.visual(
        Box((0.018, 0.36, 0.29)),
        origin=Origin(xyz=(-0.191, 0.0, 0.695)),
        material=upholstery,
        name="backrest_sling",
    )
    frame.visual(
        Box((0.24, 0.05, 0.03)),
        origin=Origin(xyz=(-0.01, 0.215, 0.635)),
        material=upholstery,
        name="left_arm_pad",
    )
    frame.visual(
        Box((0.24, 0.05, 0.03)),
        origin=Origin(xyz=(-0.01, -0.215, 0.635)),
        material=upholstery,
        name="right_arm_pad",
    )

    left_drive_wheel = model.part("left_drive_wheel")
    left_drive_wheel.visual(
        mesh_from_geometry(_build_drive_wheel(1.0), "left_drive_wheel_mesh"),
        material=wheel_finish,
        name="wheel_assembly",
    )

    right_drive_wheel = model.part("right_drive_wheel")
    right_drive_wheel.visual(
        mesh_from_geometry(_build_drive_wheel(-1.0), "right_drive_wheel_mesh"),
        material=wheel_finish,
        name="wheel_assembly",
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.visual(
        mesh_from_geometry(_build_caster_fork(), "left_caster_fork_mesh"),
        material=frame_metal,
        name="fork_hardware",
    )

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.visual(
        mesh_from_geometry(_build_caster_fork(), "right_caster_fork_mesh"),
        material=frame_metal,
        name="fork_hardware",
    )

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.visual(
        mesh_from_geometry(_build_caster_wheel(), "left_caster_wheel_mesh"),
        material=wheel_finish,
        name="wheel_assembly",
    )

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.visual(
        mesh_from_geometry(_build_caster_wheel(), "right_caster_wheel_mesh"),
        material=wheel_finish,
        name="wheel_assembly",
    )

    left_footrest = model.part("left_footrest")
    left_footrest.visual(
        mesh_from_geometry(_build_footrest_hardware(), "left_footrest_hardware"),
        material=frame_metal,
        name="hanger_hardware",
    )
    left_footrest.visual(
        Box((0.135, 0.090, 0.012)),
        origin=Origin(xyz=(0.225, 0.0, -0.282)),
        material=tread_pad,
        name="foot_plate",
    )

    right_footrest = model.part("right_footrest")
    right_footrest.visual(
        mesh_from_geometry(_build_footrest_hardware(), "right_footrest_hardware"),
        material=frame_metal,
        name="hanger_hardware",
    )
    right_footrest.visual(
        Box((0.135, 0.090, 0.012)),
        origin=Origin(xyz=(0.225, 0.0, -0.282)),
        material=tread_pad,
        name="foot_plate",
    )

    model.articulation(
        "left_drive_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_drive_wheel,
        origin=Origin(xyz=(-0.03, 0.307, 0.33)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "right_drive_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_drive_wheel,
        origin=Origin(xyz=(-0.03, -0.307, 0.33)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.34, 0.15, 0.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster_fork,
        origin=Origin(xyz=(0.34, -0.15, 0.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        "left_caster_axle",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(-0.03, 0.0, -0.168)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    model.articulation(
        "right_caster_axle",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(-0.03, 0.0, -0.168)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    model.articulation(
        "left_footrest_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_footrest,
        origin=Origin(xyz=(0.23, 0.11, 0.44)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=1.0),
    )
    model.articulation(
        "right_footrest_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_footrest,
        origin=Origin(xyz=(0.23, -0.11, 0.44)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_drive_wheel = object_model.get_part("left_drive_wheel")
    right_drive_wheel = object_model.get_part("right_drive_wheel")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")
    left_footrest = object_model.get_part("left_footrest")
    right_footrest = object_model.get_part("right_footrest")
    left_drive_axle = object_model.get_articulation("left_drive_axle")
    right_drive_axle = object_model.get_articulation("right_drive_axle")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_caster_axle = object_model.get_articulation("left_caster_axle")
    right_caster_axle = object_model.get_articulation("right_caster_axle")
    left_footrest_hinge = object_model.get_articulation("left_footrest_hinge")
    right_footrest_hinge = object_model.get_articulation("right_footrest_hinge")

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

    ctx.expect_contact(left_drive_wheel, frame, name="left_drive_wheel_supported_by_axle_mount")
    ctx.expect_contact(right_drive_wheel, frame, name="right_drive_wheel_supported_by_axle_mount")
    ctx.expect_gap(
        left_drive_wheel,
        frame,
        axis="y",
        min_gap=-1e-5,
        max_gap=0.09,
        name="left_drive_wheel_sits_outboard_of_frame",
    )
    ctx.expect_gap(
        frame,
        right_drive_wheel,
        axis="y",
        min_gap=-1e-5,
        max_gap=0.09,
        name="right_drive_wheel_sits_outboard_of_frame",
    )

    ctx.expect_contact(left_caster_fork, frame, name="left_caster_swivel_bearing_contact")
    ctx.expect_contact(right_caster_fork, frame, name="right_caster_swivel_bearing_contact")
    ctx.expect_contact(left_caster_wheel, left_caster_fork, name="left_caster_wheel_supported")
    ctx.expect_contact(right_caster_wheel, right_caster_fork, name="right_caster_wheel_supported")
    ctx.expect_gap(
        frame,
        left_caster_wheel,
        axis="z",
        min_gap=0.07,
        name="left_caster_hangs_below_service_frame",
    )
    ctx.expect_gap(
        frame,
        right_caster_wheel,
        axis="z",
        min_gap=0.07,
        name="right_caster_hangs_below_service_frame",
    )

    ctx.expect_contact(left_footrest, frame, name="left_footrest_hinge_contact")
    ctx.expect_contact(right_footrest, frame, name="right_footrest_hinge_contact")
    ctx.expect_gap(
        left_footrest,
        frame,
        axis="x",
        positive_elem="foot_plate",
        min_gap=0.03,
        name="left_footplate_projects_forward",
    )
    ctx.expect_gap(
        right_footrest,
        frame,
        axis="x",
        positive_elem="foot_plate",
        min_gap=0.03,
        name="right_footplate_projects_forward",
    )

    ctx.check(
        "drive_and_caster_axes_are_explicit",
        left_drive_axle.axis == (0.0, 1.0, 0.0)
        and right_drive_axle.axis == (0.0, 1.0, 0.0)
        and left_caster_swivel.axis == (0.0, 0.0, 1.0)
        and right_caster_swivel.axis == (0.0, 0.0, 1.0)
        and left_caster_axle.axis == (0.0, 1.0, 0.0)
        and right_caster_axle.axis == (0.0, 1.0, 0.0),
        details="Wheel roll and caster swivel axes should stay aligned with serviceable axle and swivel hardware.",
    )
    ctx.check(
        "footrests_swing_outward",
        left_footrest_hinge.axis == (0.0, 0.0, 1.0)
        and right_footrest_hinge.axis == (0.0, 0.0, -1.0),
        details="Swing-away footrests should open away from the centerline.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
