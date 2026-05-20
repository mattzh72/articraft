from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _circle_point(radius: float, angle: float, x: float) -> tuple[float, float, float]:
    return (x, radius * math.cos(angle), radius * math.sin(angle))


def _build_blade_geometry():
    root = (
        (-0.010, 0.048, -0.016),
        (0.016, 0.048, -0.005),
        (0.012, 0.048, 0.021),
        (-0.007, 0.048, 0.008),
    )
    mid = (
        (-0.005, 0.125, -0.014),
        (0.024, 0.125, -0.001),
        (0.017, 0.125, 0.030),
        (-0.008, 0.125, 0.012),
    )
    tip = (
        (0.002, 0.225, -0.007),
        (0.018, 0.225, 0.000),
        (0.010, 0.225, 0.017),
        (-0.004, 0.225, 0.006),
    )
    return repair_loft(section_loft([root, mid, tip]), repair="mesh")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_tilting_fan")

    safety_yellow = model.material("safety_yellow", rgba=(0.89, 0.74, 0.12, 1.0))
    guard_gray = model.material("guard_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.36, 0.37, 0.39, 1.0))
    blade_aluminum = model.material("blade_aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    zinc = model.material("zinc", rgba=(0.67, 0.69, 0.72, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))
    warning_red = model.material("warning_red", rgba=(0.74, 0.15, 0.10, 1.0))

    outer_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.34, tube=0.008, radial_segments=18, tubular_segments=72),
        "guard_outer_ring",
    )
    mid_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.25, tube=0.007, radial_segments=18, tubular_segments=64),
        "guard_mid_ring",
    )
    inner_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.13, tube=0.007, radial_segments=16, tubular_segments=56),
        "guard_inner_ring",
    )
    spinner_mesh = mesh_from_geometry(
        ConeGeometry(radius=0.050, height=0.090, radial_segments=40, closed=True).rotate_y(
            math.pi / 2.0
        ),
        "rotor_spinner",
    )
    blade_mesh = mesh_from_geometry(_build_blade_geometry(), "rotor_blade")

    base = model.part("base_frame")
    base.visual(
        Box((0.64, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, 0.20, 0.03)),
        material=guard_gray,
        name="left_skid",
    )
    base.visual(
        Box((0.64, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, -0.20, 0.03)),
        material=guard_gray,
        name="right_skid",
    )
    base.visual(
        Box((0.12, 0.50, 0.06)),
        origin=Origin(xyz=(0.22, 0.0, 0.03)),
        material=guard_gray,
        name="front_tie",
    )
    base.visual(
        Box((0.18, 0.50, 0.06)),
        origin=Origin(xyz=(-0.14, 0.0, 0.03)),
        material=guard_gray,
        name="rear_tie",
    )
    base.visual(
        Box((0.24, 0.32, 0.04)),
        origin=Origin(xyz=(-0.02, 0.0, 0.08)),
        material=safety_yellow,
        name="deck_plate",
    )
    base.visual(
        Box((0.26, 0.22, 0.03)),
        origin=Origin(xyz=(-0.08, 0.0, 0.115)),
        material=safety_yellow,
        name="ballast_plate",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.88),
        origin=Origin(xyz=(-0.08, 0.0, 0.54)),
        material=safety_yellow,
        name="mast",
    )
    base.visual(
        Cylinder(radius=0.085, length=0.06),
        origin=Origin(xyz=(-0.08, 0.0, 0.14)),
        material=guard_gray,
        name="mast_collar",
    )
    base.visual(
        Box((0.12, 0.42, 0.08)),
        origin=Origin(xyz=(-0.23, 0.0, 0.98)),
        material=safety_yellow,
        name="upper_beam",
    )
    base.visual(
        Box((0.10, 0.02, 0.30)),
        origin=Origin(xyz=(-0.13, 0.218, 1.09)),
        material=safety_yellow,
        name="left_yoke_plate",
    )
    base.visual(
        Box((0.10, 0.02, 0.30)),
        origin=Origin(xyz=(-0.13, -0.218, 1.09)),
        material=safety_yellow,
        name="right_yoke_plate",
    )
    base.visual(
        Box((0.06, 0.014, 0.24)),
        origin=Origin(xyz=(-0.18, 0.234, 1.09)),
        material=guard_gray,
        name="left_lock_plate",
    )
    base.visual(
        Box((0.06, 0.014, 0.24)),
        origin=Origin(xyz=(-0.18, -0.234, 1.09)),
        material=guard_gray,
        name="right_lock_plate",
    )
    base.visual(
        Box((0.05, 0.012, 0.05)),
        origin=Origin(xyz=(-0.18, 0.228, 1.20)),
        material=warning_red,
        name="left_upper_stop",
    )
    base.visual(
        Box((0.05, 0.012, 0.05)),
        origin=Origin(xyz=(-0.18, 0.228, 1.00)),
        material=warning_red,
        name="left_lower_stop",
    )
    base.visual(
        Box((0.05, 0.012, 0.05)),
        origin=Origin(xyz=(-0.18, -0.228, 1.20)),
        material=warning_red,
        name="right_upper_stop",
    )
    base.visual(
        Box((0.05, 0.012, 0.05)),
        origin=Origin(xyz=(-0.18, -0.228, 1.00)),
        material=warning_red,
        name="right_lower_stop",
    )

    for y, side in ((0.234, "left"), (-0.234, "right")):
        base.visual(
            Cylinder(radius=0.007, length=0.050),
            origin=Origin(xyz=(-0.18, y, 1.09), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"{side}_lock_shaft",
        )
        base.visual(
            Cylinder(radius=0.019, length=0.020),
            origin=Origin(xyz=(-0.18, y, 1.09), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber_black,
            name=f"{side}_lock_knob",
        )

    for sx in (-0.12, -0.04):
        for sy in (-0.07, 0.07):
            base.visual(
                Cylinder(radius=0.010, length=0.03),
                origin=Origin(xyz=(sx, sy, 0.145)),
                material=zinc,
                name=f"anchor_bolt_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )

    _add_member(
        base,
        (0.10, 0.0, 0.06),
        (-0.02, 0.0, 0.26),
        radius=0.016,
        material=guard_gray,
        name="front_gusset",
    )
    _add_member(
        base,
        (-0.22, 0.0, 0.06),
        (-0.12, 0.0, 0.26),
        radius=0.016,
        material=guard_gray,
        name="rear_gusset",
    )
    _add_member(
        base,
        (-0.08, 0.03, 0.74),
        (-0.18, 0.18, 0.94),
        radius=0.013,
        material=guard_gray,
        name="left_beam_brace",
    )
    _add_member(
        base,
        (-0.08, -0.03, 0.74),
        (-0.18, -0.18, 0.94),
        radius=0.013,
        material=guard_gray,
        name="right_beam_brace",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.64, 0.50, 1.28)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
    )

    head = model.part("fan_head")
    head.visual(
        Cylinder(radius=0.022, length=0.416),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="trunnion_shaft",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.030),
        origin=Origin(xyz=(0.0, 0.190, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_gray,
        name="left_trunnion_boss",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.030),
        origin=Origin(xyz=(0.0, -0.190, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=guard_gray,
        name="right_trunnion_boss",
    )
    head.visual(
        Cylinder(radius=0.105, length=0.18),
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_gray,
        name="motor_shell",
    )
    head.visual(
        Cylinder(radius=0.078, length=0.05),
        origin=Origin(xyz=(-0.03, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_gray,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.105, length=0.04),
        origin=Origin(xyz=(0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_gray,
        name="front_flange",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.08),
        origin=Origin(xyz=(0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="rotor_shaft",
    )
    head.visual(
        Box((0.12, 0.14, 0.16)),
        origin=Origin(xyz=(0.04, 0.125, 0.0)),
        material=guard_gray,
        name="left_brace_plate",
    )
    head.visual(
        Box((0.12, 0.14, 0.16)),
        origin=Origin(xyz=(0.04, -0.125, 0.0)),
        material=guard_gray,
        name="right_brace_plate",
    )

    for name, x_pos in (
        ("front_outer_ring", 0.36),
        ("front_mid_ring", 0.36),
        ("front_inner_ring", 0.36),
        ("rear_outer_ring", 0.16),
        ("rear_mid_ring", 0.16),
        ("rear_inner_ring", 0.16),
    ):
        mesh = (
            outer_ring_mesh
            if "outer" in name
            else mid_ring_mesh
            if "mid" in name
            else inner_ring_mesh
        )
        head.visual(
            mesh,
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=safety_yellow if "outer" in name else guard_gray,
            name=name,
        )

    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        _add_member(
            head,
            _circle_point(0.132, angle, 0.36),
            _circle_point(0.332, angle, 0.36),
            radius=0.0042,
            material=guard_gray,
            name=f"front_spoke_{index:02d}",
        )
        _add_member(
            head,
            _circle_point(0.132, angle + (math.pi / 12.0), 0.16),
            _circle_point(0.332, angle + (math.pi / 12.0), 0.16),
            radius=0.0042,
            material=guard_gray,
            name=f"rear_spoke_{index:02d}",
        )

    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        _add_member(
            head,
            _circle_point(0.334, angle, 0.16),
            _circle_point(0.334, angle, 0.36),
            radius=0.0048,
            material=guard_gray,
            name=f"side_rib_{index:02d}",
        )

    for index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        _add_member(
            head,
            _circle_point(0.132, angle, 0.16),
            (0.18, 0.092 * math.cos(angle), 0.092 * math.sin(angle)),
            radius=0.010,
            material=guard_gray,
            name=f"motor_strut_{index:02d}",
        )

    head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.34),
        mass=12.0,
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.072, length=0.03),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_gray,
        name="back_collar",
    )
    rotor.visual(
        Cylinder(radius=0.076, length=0.012),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_aluminum,
        name="blade_mount_plate",
    )
    rotor.visual(
        Cylinder(radius=0.060, length=0.06),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_gray,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=0.050, length=0.03),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_aluminum,
        name="front_cap",
    )
    rotor.visual(
        spinner_mesh,
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        material=blade_aluminum,
        name="spinner",
    )
    for index in range(4):
        rotor.visual(
            blade_mesh,
            origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(index * (math.pi / 2.0), 0.0, 0.0)),
            material=blade_aluminum,
            name=f"blade_{index:02d}",
        )

    for index in range(4):
        angle = index * (math.pi / 2.0) + (math.pi / 4.0)
        rotor.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(
                xyz=(0.022, 0.034 * math.cos(angle), 0.034 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=zinc,
            name=f"hub_bolt_{index:02d}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.23, length=0.15),
        mass=4.5,
        origin=Origin(xyz=(0.03, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.10, 0.0, 1.09)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.9,
            lower=math.radians(-20.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "head_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=32.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    head = object_model.get_part("fan_head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head_tilt")
    spin = object_model.get_articulation("head_to_rotor_spin")

    left_yoke_plate = base.get_visual("left_yoke_plate")
    right_yoke_plate = base.get_visual("right_yoke_plate")
    left_trunnion_boss = head.get_visual("left_trunnion_boss")
    right_trunnion_boss = head.get_visual("right_trunnion_boss")
    front_outer_ring = head.get_visual("front_outer_ring")
    rotor_shaft = head.get_visual("rotor_shaft")
    back_collar = rotor.get_visual("back_collar")
    hub_shell = rotor.get_visual("hub_shell")

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
        "core_parts_present",
        base is not None and head is not None and rotor is not None,
        "base frame, fan head, and rotor must all exist",
    )
    ctx.check(
        "tilt_axis_is_lateral",
        tuple(tilt.axis) == (0.0, -1.0, 0.0),
        f"expected tilt axis (0.0, -1.0, 0.0), got {tilt.axis}",
    )
    ctx.check(
        "rotor_spin_axis_is_forward",
        tuple(spin.axis) == (1.0, 0.0, 0.0),
        f"expected rotor spin axis (1.0, 0.0, 0.0), got {spin.axis}",
    )
    ctx.expect_gap(
        base,
        head,
        axis="y",
        min_gap=0.002,
        max_gap=0.010,
        positive_elem=left_yoke_plate,
        negative_elem=left_trunnion_boss,
        name="left_trunnion_sits_inside_yoke",
    )
    ctx.expect_gap(
        head,
        base,
        axis="y",
        min_gap=0.002,
        max_gap=0.010,
        positive_elem=right_trunnion_boss,
        negative_elem=right_yoke_plate,
        name="right_trunnion_sits_inside_yoke",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        margin=0.01,
        name="rotor_within_guard_envelope",
    )
    ctx.expect_gap(
        head,
        rotor,
        axis="x",
        min_gap=0.020,
        max_gap=0.090,
        positive_elem=front_outer_ring,
        negative_elem=hub_shell,
        name="front_guard_clearance",
    )
    ctx.expect_contact(
        head,
        rotor,
        elem_a=rotor_shaft,
        elem_b=back_collar,
        name="rotor_mounted_on_head_shaft",
    )

    def _center_z(part, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    closed_z = _center_z(head, "front_outer_ring")
    with ctx.pose({tilt: tilt.motion_limits.upper or 0.0}):
        opened_z = _center_z(head, "front_outer_ring")
    ctx.check(
        "positive_tilt_lifts_front_guard",
        closed_z is not None and opened_z is not None and opened_z > closed_z + 0.05,
        f"front guard center should move upward when tilting up; closed_z={closed_z}, opened_z={opened_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
