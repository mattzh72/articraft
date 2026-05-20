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
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    sweep_profile_along_spline,
)


BODY_YAW = math.radians(8.0)
BODY_OFFSET_X = 0.035
BODY_OFFSET_Y = 0.0
BODY_OFFSET_Z = 0.002


def _merge_meshes(*geometries):
    merged = None
    for geometry in geometries:
        if merged is None:
            merged = geometry.copy()
        else:
            merged.merge(geometry)
    return merged


def _body_yaw(side_sign: int) -> float:
    return -side_sign * BODY_YAW


def _body_transform(side_sign: int) -> tuple[float, float, float, float]:
    return (
        side_sign * BODY_OFFSET_X,
        BODY_OFFSET_Y,
        BODY_OFFSET_Z,
        _body_yaw(side_sign),
    )


def _transform_point(
    point: tuple[float, float, float],
    *,
    yaw: float,
    translation: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(yaw)
    s = math.sin(yaw)
    tx, ty, tz = translation
    return (c * x - s * y + tx, s * x + c * y + ty, z + tz)


def _body_point(side_sign: int, point: tuple[float, float, float]) -> tuple[float, float, float]:
    tx, ty, tz, yaw = _body_transform(side_sign)
    return _transform_point(point, yaw=yaw, translation=(tx, ty, tz))


def _body_loop(y_pos: float, width: float, height: float, exponent: float) -> list[tuple[float, float, float]]:
    return [(x, y_pos, z) for x, z in superellipse_profile(width, height, exponent=exponent, segments=40)]


def _apply_body_transform(geometry, side_sign: int):
    tx, ty, tz, yaw = _body_transform(side_sign)
    return geometry.rotate_z(yaw).translate(tx, ty, tz)


def _body_shell_mesh(side_sign: int):
    shell = section_loft(
        [
            _body_loop(-0.010, 0.030, 0.030, 2.0),
            _body_loop(0.004, 0.032, 0.035, 2.2),
            _body_loop(0.022, 0.034, 0.044, 2.6),
            _body_loop(0.042, 0.038, 0.050, 2.9),
            _body_loop(0.060, 0.039, 0.050, 2.9),
            _body_loop(0.078, 0.036, 0.042, 2.4),
            _body_loop(0.090, 0.034, 0.036, 2.1),
        ]
    )
    objective_barrel = CylinderGeometry(radius=0.0185, height=0.024, radial_segments=32).rotate_x(math.pi / 2.0)
    objective_barrel.translate(0.0, 0.088, 0.0)
    objective_rim = CylinderGeometry(radius=0.0202, height=0.004, radial_segments=32).rotate_x(math.pi / 2.0)
    objective_rim.translate(0.0, 0.100, 0.0)
    roof_ridge = BoxGeometry((0.011, 0.032, 0.007)).translate(0.0, 0.045, 0.026)
    body_cluster = _merge_meshes(shell, objective_barrel, objective_rim, roof_ridge)
    return _apply_body_transform(body_cluster, side_sign)


def _bridge_arm_mesh(side_sign: int):
    body_attach = _body_point(side_sign, (-side_sign * 0.010, 0.018, 0.000))
    path = [
        (side_sign * 0.010, -0.006, 0.004),
        (side_sign * 0.016, -0.001, 0.0045),
        (side_sign * 0.022, 0.007, 0.0030),
        body_attach,
    ]
    return sweep_profile_along_spline(
        path,
        profile=rounded_rect_profile(0.008, 0.014, 0.0025, corner_segments=5),
        samples_per_segment=12,
        cap_profile=True,
    )


def _hinge_knuckle_mesh(side_sign: int):
    shoulder = BoxGeometry((0.014, 0.010, 0.028)).translate(side_sign * 0.013, -0.006, 0.006)
    if side_sign < 0:
        lower = CylinderGeometry(radius=0.0062, height=0.011, radial_segments=28).translate(0.0, 0.0, -0.0055)
        upper = CylinderGeometry(radius=0.0062, height=0.011, radial_segments=28).translate(0.0, 0.0, 0.0165)
        return _merge_meshes(shoulder, lower, upper)
    center = CylinderGeometry(radius=0.0062, height=0.011, radial_segments=28).translate(0.0, 0.0, 0.0055)
    return _merge_meshes(shoulder, center)


def _half_mesh(side_sign: int):
    return _merge_meshes(
        _body_shell_mesh(side_sign),
        _bridge_arm_mesh(side_sign),
        _hinge_knuckle_mesh(side_sign),
    )


def _bridge_mesh():
    rear_crossbar = BoxGeometry((0.030, 0.010, 0.010)).translate(0.0, -0.016, 0.006)
    left_upright = BoxGeometry((0.006, 0.010, 0.024)).translate(-0.011, -0.012, 0.017)
    right_upright = BoxGeometry((0.006, 0.010, 0.024)).translate(0.011, -0.012, 0.017)
    crown = BoxGeometry((0.018, 0.006, 0.006)).translate(0.0, -0.012, 0.028)
    return _merge_meshes(rear_crossbar, left_upright, right_upright, crown)


def _objective_glass_origin(side_sign: int) -> Origin:
    x, y, z = _body_point(side_sign, (0.0, 0.100, 0.0))
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, _body_yaw(side_sign)))


def _eyepiece_barrel_origin(side_sign: int) -> Origin:
    x, y, z = _body_point(side_sign, (0.0, -0.0175, 0.0))
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, _body_yaw(side_sign)))


def _eyecup_origin(side_sign: int) -> Origin:
    x, y, z = _body_point(side_sign, (0.0, -0.0305, 0.0))
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, _body_yaw(side_sign)))


def _diopter_ring_mesh():
    return LatheGeometry.from_shell_profiles(
        [(0.0180, -0.003), (0.0180, 0.003)],
        [(0.0153, -0.003), (0.0153, 0.003)],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )


def _diopter_origin() -> Origin:
    x, y, z = _body_point(1, (0.0, -0.022, 0.0))
    return Origin(xyz=(x, y, z), rpy=(0.0, 0.0, _body_yaw(1)))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_roof_prism_binocular")

    armor = model.material("armor", rgba=(0.18, 0.20, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    glass = model.material("glass", rgba=(0.10, 0.16, 0.20, 0.55))

    bridge = model.part("bridge")
    bridge.visual(
        mesh_from_geometry(_bridge_mesh(), "bridge_housing"),
        material=armor,
        name="bridge_housing",
    )

    left_half = model.part("left_half")
    left_half.visual(
        mesh_from_geometry(_half_mesh(-1), "left_half_body"),
        material=armor,
        name="left_body",
    )
    left_half.visual(
        Cylinder(radius=0.0150, length=0.015),
        origin=_eyepiece_barrel_origin(-1),
        material=armor,
        name="left_eyepiece_barrel",
    )
    left_half.visual(
        Cylinder(radius=0.0172, length=0.011),
        origin=_eyecup_origin(-1),
        material=rubber,
        name="left_eyecup",
    )
    left_half.visual(
        Cylinder(radius=0.0154, length=0.002),
        origin=_objective_glass_origin(-1),
        material=glass,
        name="left_objective_glass",
    )

    right_half = model.part("right_half")
    right_half.visual(
        mesh_from_geometry(_half_mesh(1), "right_half_body"),
        material=armor,
        name="right_body",
    )
    right_half.visual(
        Cylinder(radius=0.0150, length=0.015),
        origin=_eyepiece_barrel_origin(1),
        material=armor,
        name="right_eyepiece_barrel",
    )
    right_half.visual(
        Cylinder(radius=0.0172, length=0.011),
        origin=_eyecup_origin(1),
        material=rubber,
        name="right_eyecup",
    )
    right_half.visual(
        Cylinder(radius=0.0154, length=0.002),
        origin=_objective_glass_origin(1),
        material=glass,
        name="right_objective_glass",
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        Cylinder(radius=0.0060, length=0.016),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="wheel_axle_core",
    )
    focus_wheel.visual(
        Cylinder(radius=0.0100, length=0.010),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="focus_wheel",
    )

    diopter_ring = model.part("diopter_ring")
    diopter_ring.visual(
        mesh_from_geometry(_diopter_ring_mesh(), "right_diopter_ring"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="diopter_ring",
    )

    model.articulation(
        "bridge_to_left_half",
        ArticulationType.FIXED,
        parent=bridge,
        child=left_half,
        origin=Origin(),
    )
    model.articulation(
        "interpupillary_hinge",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=right_half,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.2,
            lower=-0.10,
            upper=0.12,
        ),
    )
    model.articulation(
        "bridge_to_focus_wheel",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=focus_wheel,
        origin=Origin(xyz=(0.0, -0.012, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=8.0,
            lower=-8.0,
            upper=8.0,
        ),
    )
    model.articulation(
        "right_half_to_diopter",
        ArticulationType.REVOLUTE,
        parent=right_half,
        child=diopter_ring,
        origin=_diopter_origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=2.0,
            lower=-0.70,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge")
    left_half = object_model.get_part("left_half")
    right_half = object_model.get_part("right_half")
    focus_wheel = object_model.get_part("focus_wheel")
    diopter_ring = object_model.get_part("diopter_ring")
    hinge = object_model.get_articulation("interpupillary_hinge")
    focus_joint = object_model.get_articulation("bridge_to_focus_wheel")
    diopter_joint = object_model.get_articulation("right_half_to_diopter")

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

    ctx.expect_contact(left_half, bridge, name="left half is mounted to the bridge housing")
    ctx.expect_contact(right_half, left_half, name="hinge knuckles stay in contact at the center joint")
    ctx.expect_contact(focus_wheel, bridge, name="focus wheel is supported by the bridge uprights")
    ctx.expect_contact(
        diopter_ring,
        right_half,
        contact_tol=0.0005,
        elem_b="right_eyepiece_barrel",
        name="diopter ring stays closely seated around the right eyepiece barrel",
    )
    ctx.expect_overlap(
        diopter_ring,
        right_half,
        axes="xz",
        min_overlap=0.020,
        elem_a="diopter_ring",
        elem_b="right_eyepiece_barrel",
        name="diopter ring stays coaxial with the right eyepiece barrel",
    )
    ctx.expect_overlap(
        diopter_ring,
        right_half,
        axes="y",
        min_overlap=0.008,
        elem_a="diopter_ring",
        elem_b="right_eyepiece_barrel",
        name="diopter ring remains axially engaged on the right eyepiece barrel",
    )

    ctx.check(
        "hinge axis is vertical",
        hinge.axis == (0.0, 0.0, -1.0),
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "focus wheel axis is transverse",
        focus_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={focus_joint.axis}",
    )
    ctx.check(
        "diopter axis follows the eyepiece tube",
        diopter_joint.axis == (0.0, 1.0, 0.0),
        details=f"axis={diopter_joint.axis}",
    )

    rest_aabb = ctx.part_element_world_aabb(right_half, elem="right_objective_glass")
    upper_limit = hinge.motion_limits.upper if hinge.motion_limits is not None else None
    with ctx.pose({hinge: upper_limit if upper_limit is not None else 0.0}):
        opened_aabb = ctx.part_element_world_aabb(right_half, elem="right_objective_glass")
    rest_x = None if rest_aabb is None else (rest_aabb[0][0] + rest_aabb[1][0]) * 0.5
    opened_x = None if opened_aabb is None else (opened_aabb[0][0] + opened_aabb[1][0]) * 0.5
    ctx.check(
        "positive hinge motion opens the right barrel outward",
        rest_x is not None and opened_x is not None and opened_x > rest_x + 0.004,
        details=f"rest_x={rest_x}, opened_x={opened_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
