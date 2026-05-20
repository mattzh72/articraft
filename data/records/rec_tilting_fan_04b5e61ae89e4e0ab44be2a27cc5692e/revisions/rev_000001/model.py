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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _point_on_x_ring(x_pos: float, radius: float, angle: float) -> tuple[float, float, float]:
    return (x_pos, math.cos(angle) * radius, math.sin(angle) * radius)


def _tube_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    radial_segments: int = 14,
) -> MeshGeometry:
    mid = tuple((a + b) * 0.5 for a, b in zip(start, end))
    return tube_from_spline_points(
        [start, mid, end],
        radius=radius,
        samples_per_segment=4,
        radial_segments=radial_segments,
        cap_ends=True,
    )


def _head_shell_mesh() -> MeshGeometry:
    def section(x_pos: float, y_radius: float, z_radius: float, *, z_shift: float = 0.0):
        points = []
        for index in range(18):
            angle = index * math.tau / 18.0
            points.append(
                (
                    x_pos,
                    math.cos(angle) * y_radius,
                    (math.sin(angle) * z_radius) + z_shift,
                )
            )
        return points

    return section_loft(
        [
            section(-0.030, 0.034, 0.042, z_shift=0.020),
            section(-0.010, 0.048, 0.055, z_shift=0.020),
            section(0.008, 0.058, 0.062, z_shift=0.021),
            section(0.022, 0.056, 0.060, z_shift=0.021),
            section(0.032, 0.046, 0.050, z_shift=0.021),
        ]
    )


def _guard_web_mesh() -> MeshGeometry:
    front_x = 0.178
    rear_x = 0.102
    support_x = 0.088
    outer_radius = 0.180
    center_radius = 0.040
    support_radius = 0.074
    wire_radius = 0.0024

    web = MeshGeometry()

    for index in range(12):
        angle = index * math.tau / 12.0
        web.merge(
            _tube_between(
                _point_on_x_ring(front_x, center_radius, angle),
                _point_on_x_ring(front_x, outer_radius, angle),
                radius=wire_radius,
            )
        )
        web.merge(
            _tube_between(
                _point_on_x_ring(rear_x, outer_radius, angle),
                _point_on_x_ring(front_x, outer_radius, angle),
                radius=wire_radius,
            )
        )

    for index in range(8):
        angle = (index * math.tau / 8.0) + (math.pi / 8.0)
        web.merge(
            _tube_between(
                _point_on_x_ring(support_x, support_radius, angle),
                _point_on_x_ring(rear_x, outer_radius, angle),
                radius=wire_radius,
            )
        )

    return web


def _blade_loop(
    radius: float,
    *,
    chord: float,
    thickness: float,
    x_shift: float,
    skew: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_shift - 0.55 * thickness, skew - 0.42 * chord, radius),
        (x_shift + 0.45 * thickness, skew - 0.14 * chord, radius),
        (x_shift + 0.18 * thickness, skew + 0.52 * chord, radius),
        (x_shift - 0.36 * thickness, skew + 0.18 * chord, radius),
    ]


def _rotor_blade_mesh() -> MeshGeometry:
    blade = section_loft(
        [
            _blade_loop(0.022, chord=0.046, thickness=0.009, x_shift=-0.003, skew=-0.009),
            _blade_loop(0.084, chord=0.039, thickness=0.0075, x_shift=0.001, skew=0.000),
            _blade_loop(0.136, chord=0.026, thickness=0.0055, x_shift=0.006, skew=0.013),
        ]
    )
    patterned = MeshGeometry()
    for index in range(5):
        patterned.merge(blade.copy().rotate((1.0, 0.0, 0.0), index * math.tau / 5.0))
    return patterned


def _spinner_mesh() -> MeshGeometry:
    def circle(x_pos: float, radius: float):
        points = []
        for index in range(16):
            angle = index * math.tau / 16.0
            points.append((x_pos, math.cos(angle) * radius, math.sin(angle) * radius))
        return points

    return section_loft(
        [
            circle(-0.020, 0.028),
            circle(-0.004, 0.022),
            circle(0.010, 0.013),
            circle(0.022, 0.004),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_tilting_fan")

    base_paint = model.material("base_paint", rgba=(0.24, 0.26, 0.29, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.33, 0.35, 0.38, 1.0))
    housing_paint = model.material("housing_paint", rgba=(0.73, 0.75, 0.78, 1.0))
    guard_steel = model.material("guard_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.15, 0.16, 0.17, 1.0))
    rotor_grey = model.material("rotor_grey", rgba=(0.48, 0.50, 0.53, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.280, 0.200, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=base_paint,
        name="base_plate",
    )
    base.visual(
        Box((0.180, 0.100, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=housing_paint,
        name="datum_pad",
    )
    base.visual(
        Box((0.120, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.080, 0.030)),
        material=guard_steel,
        name="front_datum_rail",
    )
    base.visual(
        Box((0.090, 0.070, 0.150)),
        origin=Origin(xyz=(-0.055, 0.0, 0.099)),
        material=base_paint,
        name="column",
    )
    base.visual(
        Box((0.120, 0.090, 0.022)),
        origin=Origin(xyz=(-0.030, 0.0, 0.171)),
        material=base_paint,
        name="neck_block",
    )
    base.visual(
        Box((0.016, 0.212, 0.090)),
        origin=Origin(xyz=(-0.050, 0.0, 0.255)),
        material=arm_paint,
        name="rear_bridge",
    )
    base.visual(
        _mesh(
            "left_base_brace",
            tube_from_spline_points(
                [(-0.095, 0.050, 0.028), (-0.060, 0.070, 0.150), (-0.024, 0.090, 0.230)],
                radius=0.013,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=arm_paint,
        name="left_brace",
    )
    base.visual(
        _mesh(
            "right_base_brace",
            tube_from_spline_points(
                [(-0.095, -0.050, 0.028), (-0.060, -0.070, 0.150), (-0.024, -0.090, 0.230)],
                radius=0.013,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=arm_paint,
        name="right_brace",
    )
    base.visual(
        Box((0.060, 0.014, 0.155)),
        origin=Origin(xyz=(0.000, 0.109, 0.255)),
        material=arm_paint,
        name="left_yoke_arm",
    )
    base.visual(
        Box((0.060, 0.014, 0.155)),
        origin=Origin(xyz=(0.000, -0.109, 0.255)),
        material=arm_paint,
        name="right_yoke_arm",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.008),
        origin=Origin(xyz=(0.000, 0.120, 0.255), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_steel,
        name="left_scale_plate",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.000, -0.154, 0.255), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_steel,
        name="right_clamp_knob",
    )
    base.visual(
        Box((0.038, 0.006, 0.012)),
        origin=Origin(xyz=(0.000, -0.167, 0.255)),
        material=guard_steel,
        name="right_clamp_wing",
    )
    base.visual(
        Box((0.020, 0.032, 0.030)),
        origin=Origin(xyz=(0.000, -0.128, 0.255)),
        material=guard_steel,
        name="right_clamp_stem",
    )
    for mark_index, angle_deg in enumerate(range(-30, 31, 10)):
        angle = math.radians(angle_deg)
        radius = 0.036
        base.visual(
            Box((0.0035, 0.006, 0.012 if angle_deg == 0 else 0.009)),
            origin=Origin(
                xyz=(
                    math.sin(angle) * radius,
                    0.120,
                    0.255 - math.cos(angle) * radius,
                ),
                rpy=(0.0, -angle, 0.0),
            ),
            material=rotor_black,
            name="zero_index" if angle_deg == 0 else f"index_mark_{mark_index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.280, 0.200, 0.320)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.048, length=0.072),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_paint,
        name="motor_shell",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.028),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_paint,
        name="front_shroud",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.064, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_steel,
        name="bearing_nose",
    )
    head.visual(
        Box((0.036, 0.100, 0.014)),
        origin=Origin(xyz=(0.052, 0.0, -0.042)),
        material=housing_paint,
        name="head_datum_band",
    )
    head.visual(
        Box((0.028, 0.070, 0.064)),
        origin=Origin(xyz=(0.000, 0.060, 0.0)),
        material=housing_paint,
        name="left_pivot_web",
    )
    head.visual(
        Box((0.028, 0.070, 0.064)),
        origin=Origin(xyz=(0.000, -0.060, 0.0)),
        material=housing_paint,
        name="right_pivot_web",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.000, 0.095, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_steel,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.000, -0.095, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_steel,
        name="right_trunnion",
    )
    head.visual(
        Box((0.030, 0.008, 0.060)),
        origin=Origin(xyz=(0.014, 0.077, 0.0)),
        material=housing_paint,
        name="left_clearance_pad",
    )
    head.visual(
        Box((0.030, 0.008, 0.060)),
        origin=Origin(xyz=(0.014, -0.077, 0.0)),
        material=housing_paint,
        name="right_clearance_pad",
    )
    head.visual(
        Box((0.014, 0.060, 0.010)),
        origin=Origin(xyz=(0.040, 0.063, -0.048)),
        material=rotor_black,
        name="tilt_pointer",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.140, 0.180, 0.130)),
        mass=2.8,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    guard = model.part("guard")
    guard.visual(
        _mesh(
            "guard_support_ring",
            TorusGeometry(radius=0.074, tube=0.004, radial_segments=14, tubular_segments=44)
            .rotate_y(math.pi / 2.0)
            .translate(0.088, 0.0, 0.0),
        ),
        material=guard_steel,
        name="support_ring",
    )
    guard.visual(
        _mesh(
            "guard_rear_ring",
            TorusGeometry(radius=0.180, tube=0.0035, radial_segments=14, tubular_segments=52)
            .rotate_y(math.pi / 2.0)
            .translate(0.102, 0.0, 0.0),
        ),
        material=guard_steel,
        name="guard_rear_ring",
    )
    guard.visual(
        _mesh(
            "guard_front_ring",
            TorusGeometry(radius=0.180, tube=0.0035, radial_segments=14, tubular_segments=52)
            .rotate_y(math.pi / 2.0)
            .translate(0.178, 0.0, 0.0),
        ),
        material=guard_steel,
        name="guard_front_ring",
    )
    guard.visual(
        _mesh(
            "guard_center_ring",
            TorusGeometry(radius=0.040, tube=0.003, radial_segments=12, tubular_segments=36)
            .rotate_y(math.pi / 2.0)
            .translate(0.178, 0.0, 0.0),
        ),
        material=guard_steel,
        name="guard_center_ring",
    )
    center_struts = MeshGeometry()
    for angle_deg in (90, 210, 330):
        angle = math.radians(angle_deg)
        center_struts.merge(
            _tube_between(
                _point_on_x_ring(0.064, 0.046, angle),
                _point_on_x_ring(0.088, 0.074, angle),
                radius=0.0032,
                radial_segments=12,
            )
        )
    guard.visual(
        _mesh("guard_center_struts", center_struts),
        material=guard_steel,
        name="center_struts",
    )
    guard.visual(
        _mesh("guard_web", _guard_web_mesh()),
        material=guard_steel,
        name="guard_web",
    )
    guard.inertial = Inertial.from_geometry(
        Cylinder(radius=0.182, length=0.086),
        mass=0.55,
        origin=Origin(xyz=(0.140, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(-0.0225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_steel,
        name="hub_rear_collar",
    )
    rotor.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rotor_grey,
        name="hub_body",
    )
    rotor.visual(
        Cylinder(radius=0.042, length=0.006),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rotor_grey,
        name="blade_root_disc",
    )
    rotor.visual(
        _mesh("rotor_blades", _rotor_blade_mesh()),
        material=rotor_black,
        name="blade_sweep",
    )
    rotor.visual(
        _mesh("rotor_spinner", _spinner_mesh()),
        material=rotor_grey,
        name="spinner",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.155, length=0.060),
        mass=0.9,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.000, 0.0, 0.255)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.18,
            upper=0.56,
        ),
    )
    model.articulation(
        "head_to_guard",
        ArticulationType.FIXED,
        parent=head,
        child=guard,
        origin=Origin(),
    )
    model.articulation(
        "head_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    guard = object_model.get_part("guard")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head_tilt")
    spin = object_model.get_articulation("head_to_rotor_spin")

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
        "primary_articulations_present",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and spin.articulation_type == ArticulationType.CONTINUOUS,
        details="Tilting head and rotor spin must both be explicit articulations.",
    )

    ctx.expect_contact(
        head,
        base,
        elem_a="left_trunnion",
        elem_b="left_yoke_arm",
        name="left_trunnion_supported_by_left_yoke",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="right_trunnion",
        elem_b="right_yoke_arm",
        name="right_trunnion_supported_by_right_yoke",
    )
    ctx.expect_gap(
        base,
        head,
        axis="y",
        positive_elem="left_yoke_arm",
        negative_elem="left_clearance_pad",
        min_gap=0.019,
        max_gap=0.024,
        name="left_yoke_to_head_controlled_gap",
    )
    ctx.expect_contact(
        guard,
        head,
        elem_a="center_struts",
        elem_b="front_shroud",
        name="guard_supported_by_front_shroud",
    )
    ctx.expect_contact(
        rotor,
        head,
        elem_a="hub_rear_collar",
        elem_b="bearing_nose",
        name="rotor_hub_supported_on_bearing_nose",
    )
    ctx.expect_within(
        rotor,
        guard,
        axes="yz",
        inner_elem="blade_sweep",
        outer_elem="guard_front_ring",
        margin=0.0,
        name="rotor_blades_within_front_guard_diameter",
    )
    ctx.expect_gap(
        guard,
        rotor,
        axis="x",
        positive_elem="guard_front_ring",
        negative_elem="spinner",
        min_gap=0.050,
        max_gap=0.070,
        name="spinner_to_front_guard_gap",
    )

    closed_guard = ctx.part_element_world_aabb(guard, elem="guard_front_ring")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        raised_guard = ctx.part_element_world_aabb(guard, elem="guard_front_ring")
        ctx.expect_within(
            rotor,
            guard,
            axes="yz",
            inner_elem="blade_sweep",
            outer_elem="guard_front_ring",
            margin=0.0,
            name="rotor_remains_within_guard_at_max_tilt",
        )

    if closed_guard is None or raised_guard is None:
        ctx.fail("tilt_raises_fan_head", "Could not read guard AABBs for tilt verification.")
    else:
        closed_center_z = 0.5 * (closed_guard[0][2] + closed_guard[1][2])
        raised_center_z = 0.5 * (raised_guard[0][2] + raised_guard[1][2])
        ctx.check(
            "tilt_raises_fan_head",
            raised_center_z > closed_center_z + 0.08,
            details=(
                f"Expected guard center to rise by > 0.08 m at max tilt, "
                f"got {raised_center_z - closed_center_z:.4f} m."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
