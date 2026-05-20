from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _z_axis_rpy(p1: tuple[float, float, float], p2: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _cylinder_between(
    part,
    name: str,
    p1: tuple[float, float, float],
    p2: tuple[float, float, float],
    radius: float,
    material: Material,
) -> None:
    length = math.dist(p1, p2)
    mid = ((p1[0] + p2[0]) * 0.5, (p1[1] + p2[1]) * 0.5, (p1[2] + p2[2]) * 0.5)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=_z_axis_rpy(p1, p2)),
        material=material,
        name=name,
    )


def _add_torus(
    part,
    name: str,
    *,
    center: tuple[float, float, float],
    major_radius: float,
    tube_radius: float,
    material: Material,
    vertical_xz: bool = False,
) -> None:
    mesh = mesh_from_geometry(
        TorusGeometry(
            radius=major_radius,
            tube=tube_radius,
            radial_segments=20,
            tubular_segments=72,
        ),
        name,
    )
    part.visual(
        mesh,
        origin=Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0) if vertical_xz else (0.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _plane_point(radius: float, angle: float, y: float = 0.0) -> tuple[float, float, float]:
    return (radius * math.cos(angle), y, radius * math.sin(angle))


def _blade_point(radius: float, half_width: float, angle: float, side: float, y: float) -> tuple[float, float, float]:
    radial = (math.cos(angle), 0.0, math.sin(angle))
    tangent = (-math.sin(angle), 0.0, math.cos(angle))
    return (
        radial[0] * radius + tangent[0] * half_width * side,
        y,
        radial[2] * radius + tangent[2] * half_width * side,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_windmill")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.62, 0.62, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    concrete = model.material("reinforced_concrete", rgba=(0.42, 0.42, 0.38, 1.0))
    hazard = model.material("hazard_yellow_guard", rgba=(1.0, 0.78, 0.08, 1.0))
    lock_red = model.material("lockout_red", rgba=(0.86, 0.04, 0.02, 1.0))
    blade_mat = model.material("brushed_blade_steel", rgba=(0.76, 0.75, 0.70, 1.0))
    black = model.material("black_fasteners", rgba=(0.015, 0.014, 0.012, 1.0))

    # Root structural tower: broad concrete plinth, steel lattice legs, cross-bracing,
    # top service deck, yaw thrust surface, and visible bolted base plates.
    tower = model.part("tower")
    tower.visual(
        Box((2.45, 2.45, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=concrete,
        name="concrete_plinth",
    )
    tower.visual(
        Box((1.45, 0.92, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 5.74)),
        material=galvanized,
        name="top_service_deck",
    )
    _add_torus(
        tower,
        "lower_yaw_race",
        center=(0.0, 0.0, 5.755),
        major_radius=0.42,
        tube_radius=0.032,
        material=dark_steel,
    )

    base_pts = [(-0.93, -0.93, 0.24), (0.93, -0.93, 0.24), (0.93, 0.93, 0.24), (-0.93, 0.93, 0.24)]
    top_pts = [(-0.34, -0.34, 5.78), (0.34, -0.34, 5.78), (0.34, 0.34, 5.78), (-0.34, 0.34, 5.78)]
    for i, (b, t) in enumerate(zip(base_pts, top_pts)):
        tower.visual(
            Box((0.32, 0.32, 0.045)),
            origin=Origin(xyz=(b[0], b[1], 0.29)),
            material=galvanized,
            name=f"footplate_{i}",
        )
        _cylinder_between(tower, f"tower_leg_{i}", b, t, 0.055, galvanized)
        for sx in (-0.07, 0.07):
            for sy in (-0.07, 0.07):
                tower.visual(
                    Cylinder(radius=0.025, length=0.025),
                    origin=Origin(xyz=(b[0] + sx, b[1] + sy, 0.325)),
                    material=black,
                    name=f"anchor_bolt_{i}_{sx}_{sy}",
                )

    # Stacked X-bracing on each face makes the load path explicit for heavy duty cycles.
    levels = [0.45, 1.55, 2.65, 3.75, 4.85, 5.75]
    for k in range(len(levels) - 1):
        z0 = levels[k]
        z1 = levels[k + 1]
        s0 = 0.93 - (0.93 - 0.34) * ((z0 - 0.24) / (5.78 - 0.24))
        s1 = 0.93 - (0.93 - 0.34) * ((z1 - 0.24) / (5.78 - 0.24))
        # front and rear faces
        for ysign, label in [(-1.0, "front"), (1.0, "rear")]:
            _cylinder_between(tower, f"{label}_brace_{k}_a", (-s0, ysign * s0, z0), (s1, ysign * s1, z1), 0.026, galvanized)
            _cylinder_between(tower, f"{label}_brace_{k}_b", (s0, ysign * s0, z0), (-s1, ysign * s1, z1), 0.026, galvanized)
        # side faces
        for xsign, label in [(-1.0, "side_a"), (1.0, "side_b")]:
            _cylinder_between(tower, f"{label}_brace_{k}_a", (xsign * s0, -s0, z0), (xsign * s1, s1, z1), 0.026, galvanized)
            _cylinder_between(tower, f"{label}_brace_{k}_b", (xsign * s0, s0, z0), (xsign * s1, -s1, z1), 0.026, galvanized)

    for angle, name in [(0.0, "yaw_stop_front"), (math.pi, "yaw_stop_rear")]:
        tower.visual(
            Box((0.18, 0.12, 0.20)),
            origin=Origin(xyz=(0.73 * math.cos(angle), 0.73 * math.sin(angle), 5.90), rpy=(0.0, 0.0, angle)),
            material=lock_red,
            name=name,
        )

    # Yawing cap/nacelle stage.  The open-plate construction leaves the shaft passage
    # visible instead of hiding it inside a solid box.
    cap = model.part("cap")
    cap.visual(
        Box((0.86, 0.86, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=galvanized,
        name="yaw_base_plate",
    )
    _add_torus(cap, "upper_yaw_race", center=(0.0, 0.0, 0.035), major_radius=0.42, tube_radius=0.035, material=dark_steel)
    cap.visual(
        Box((0.98, 1.50, 0.09)),
        origin=Origin(xyz=(0.0, -0.48, 0.11)),
        material=galvanized,
        name="nacelle_cradle",
    )
    for x, label in [(-0.44, "side_a"), (0.44, "side_b")]:
        cap.visual(
            Box((0.055, 1.50, 0.70)),
            origin=Origin(xyz=(x, -0.48, 0.50)),
            material=galvanized,
            name=f"{label}_cheek_plate",
        )
        for y in (-1.10, -0.66, -0.24, 0.12):
            cap.visual(
                Cylinder(radius=0.025, length=0.024),
                origin=Origin(xyz=(x + (0.034 if x > 0 else -0.034), y, 0.62), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=black,
                name=f"{label}_bolt_{y}",
            )
    cap.visual(
        Box((1.04, 1.36, 0.08)),
        origin=Origin(xyz=(0.0, -0.48, 0.86)),
        material=galvanized,
        name="flat_service_cap",
    )
    for x, label in [(-0.20, "left"), (0.20, "right")]:
        _cylinder_between(cap, f"{label}_pedestal_web", (x, -1.10, 0.23), (x, -1.10, 0.56), 0.035, galvanized)
        _cylinder_between(cap, f"{label}_rear_pedestal_web", (x, -0.66, 0.23), (x, -0.66, 0.56), 0.032, galvanized)
    for y, label in [(-1.10, "front"), (-0.66, "rear")]:
        cap.visual(
            Box((0.18, 0.08, 0.24)),
            origin=Origin(xyz=(0.0, y, 0.22)),
            material=galvanized,
            name=f"{label}_bearing_web_plate",
        )

    # Clear bearing seats around the rotor shaft.
    for y, label in [(-1.10, "front"), (-0.66, "rear")]:
        cap.visual(
            Box((0.52, 0.13, 0.18)),
            origin=Origin(xyz=(0.0, y, 0.38)),
            material=galvanized,
            name=f"{label}_bearing_pedestal",
        )
        _add_torus(
            cap,
            f"{label}_bearing_seat",
            center=(0.0, y, 0.55),
            major_radius=0.155,
            tube_radius=0.034,
            material=dark_steel,
            vertical_xz=True,
        )
        _add_torus(
            cap,
            f"{label}_bearing_liner",
            center=(0.0, y, 0.55),
            major_radius=0.100,
            tube_radius=0.020,
            material=black,
            vertical_xz=True,
        )
        cap.visual(
            Box((0.08, 0.050, 0.020)),
            origin=Origin(xyz=(0.0, y, 0.6225)),
            material=black,
            name=f"{label}_bearing_shoe",
        )
        for x in (-0.20, 0.20):
            cap.visual(
                Cylinder(radius=0.020, length=0.035),
                origin=Origin(xyz=(x, y, 0.485), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=black,
                name=f"{label}_bearing_bolt_{x}",
            )

    # Stationary cage guard: rear and front hoops, perimeter standoffs, and rear spokes.
    _add_torus(cap, "rear_guard_ring", center=(0.0, -1.28, 0.55), major_radius=2.25, tube_radius=0.030, material=hazard, vertical_xz=True)
    _add_torus(cap, "front_guard_ring", center=(0.0, -1.94, 0.55), major_radius=2.25, tube_radius=0.030, material=hazard, vertical_xz=True)
    for i in range(12):
        a = i * math.tau / 12.0
        x = 2.25 * math.cos(a)
        z = 0.55 + 2.25 * math.sin(a)
        _cylinder_between(cap, f"guard_standoff_{i}", (x, -1.28, z), (x, -1.94, z), 0.018, hazard)
    for i, a in enumerate([0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0, math.pi, 5.0 * math.pi / 4.0, 3.0 * math.pi / 2.0, 7.0 * math.pi / 4.0]):
        start = (0.24 * math.cos(a), -1.28, 0.55 + 0.24 * math.sin(a))
        end = (2.25 * math.cos(a), -1.28, 0.55 + 2.25 * math.sin(a))
        _cylinder_between(cap, f"rear_guard_spoke_{i}", start, end, 0.018, hazard)
    for x, label in [(-0.44, "side_a"), (0.44, "side_b")]:
        _cylinder_between(
            cap,
            f"{label}_guard_support",
            (x, -1.16, 0.86),
            (x, -1.28, 2.75),
            0.024,
            hazard,
        )
    cap.visual(
        Box((0.16, 0.10, 0.22)),
        origin=Origin(xyz=(0.0, -1.61, 2.91)),
        material=lock_red,
        name="top_overtravel_flag",
    )
    cap.visual(
        Box((0.16, 0.10, 0.22)),
        origin=Origin(xyz=(0.0, -1.61, -1.81)),
        material=lock_red,
        name="bottom_overtravel_flag",
    )
    cap.visual(
        Box((0.26, 0.16, 0.18)),
        origin=Origin(xyz=(-0.48, -0.48, 0.13)),
        material=lock_red,
        name="yaw_stop_lug",
    )

    # Lockout hinge bracket on the cap side.
    for x, label in [(0.58, "inner"), (0.80, "outer")]:
        cap.visual(
            Box((0.04, 0.14, 0.20)),
            origin=Origin(xyz=(x, -0.50, 0.73)),
            material=galvanized,
            name=f"lockout_{label}_clevis",
        )
    cap.visual(
        Box((0.28, 0.05, 0.10)),
        origin=Origin(xyz=(0.69, -0.56, 0.64)),
        material=galvanized,
        name="lockout_clevis_bridge",
    )
    cap.visual(
        Box((0.14, 0.06, 0.14)),
        origin=Origin(xyz=(0.51, -0.50, 0.70)),
        material=galvanized,
        name="lockout_side_standoff",
    )
    cap.visual(
        Box((0.18, 0.06, 0.10)),
        origin=Origin(xyz=(0.69, -0.86, 0.69)),
        material=lock_red,
        name="lockout_receiver",
    )
    cap.visual(
        Box((0.10, 0.30, 0.08)),
        origin=Origin(xyz=(0.565, -0.72, 0.69)),
        material=galvanized,
        name="receiver_support_strap",
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 5.80)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.18, lower=-0.78, upper=0.78),
        motion_properties=MotionProperties(damping=25.0, friction=15.0),
    )

    # Rotor stage: shaft, bearing journal, lock disk, and four lattice sails.
    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.064, length=0.88),
        origin=Origin(xyz=(0.0, 0.28, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="main_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.21, length=0.34),
        origin=Origin(xyz=(0.0, -0.32, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rotor_hub",
    )
    rotor.visual(
        Cylinder(radius=0.34, length=0.030),
        origin=Origin(xyz=(0.0, -0.100, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="lockout_disk",
    )
    for i, a in enumerate([0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]):
        # Main lattice blade in the rotor XZ plane, slightly forward of the shaft.
        blade_y = -0.34
        _cylinder_between(rotor, f"blade_{i}_main_spar", _plane_point(0.15, a, blade_y), _plane_point(2.03, a, blade_y), 0.032, blade_mat)
        root_w = 0.09
        tip_w = 0.27
        for side in (-1.0, 1.0):
            _cylinder_between(
                rotor,
                f"blade_{i}_side_rail_{side}",
                _blade_point(0.34, root_w, a, side, blade_y),
                _blade_point(1.98, tip_w, a, side, blade_y),
                0.022,
                blade_mat,
            )
        rung_radii = [0.36, 0.62, 0.92, 1.24, 1.56, 1.88]
        for j, r in enumerate(rung_radii):
            w = root_w + (tip_w - root_w) * ((r - 0.34) / (1.98 - 0.34))
            _cylinder_between(
                rotor,
                f"blade_{i}_rung_{j}",
                _blade_point(r, w, a, -1.0, blade_y),
                _blade_point(r, w, a, 1.0, blade_y),
                0.018,
                blade_mat,
            )
        for j in range(len(rung_radii) - 1):
            r0 = rung_radii[j]
            r1 = rung_radii[j + 1]
            w0 = root_w + (tip_w - root_w) * ((r0 - 0.34) / (1.98 - 0.34))
            w1 = root_w + (tip_w - root_w) * ((r1 - 0.34) / (1.98 - 0.34))
            _cylinder_between(
                rotor,
                f"blade_{i}_diagonal_{j}",
                _blade_point(r0, w0, a, -1.0 if j % 2 == 0 else 1.0, blade_y),
                _blade_point(r1, w1, a, 1.0 if j % 2 == 0 else -1.0, blade_y),
                0.014,
                blade_mat,
            )
        _cylinder_between(
            rotor,
            f"blade_{i}_root_clamp",
            _blade_point(0.26, 0.14, a, -1.0, blade_y),
            _blade_point(0.26, 0.14, a, 1.0, blade_y),
            0.028,
            dark_steel,
        )
        # Black fasteners on the lock disk at each blade root.
        disk_p = _plane_point(0.27, a, -0.100)
        rotor.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(xyz=disk_p, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"lock_hole_bolt_{i}",
        )

    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(0.0, -1.10, 0.55)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1100.0, velocity=1.2),
        motion_properties=MotionProperties(damping=3.0, friction=0.8),
    )

    # Articulated lockout handle, deliberately separate from the fixed receiver.
    lockout = model.part("lockout_lever")
    lockout.visual(
        Cylinder(radius=0.038, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    _cylinder_between(lockout, "red_handle", (0.0, 0.0, -0.03), (0.0, 0.03, -0.38), 0.030, lock_red)
    lockout.visual(
        Cylinder(radius=0.055, length=0.16),
        origin=Origin(xyz=(0.0, 0.035, -0.420), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=lock_red,
        name="grip_knob",
    )
    _cylinder_between(lockout, "lock_pin_stub", (-0.015, -0.02, -0.02), (-0.015, -0.28, -0.02), 0.024, lock_red)
    model.articulation(
        "cap_to_lockout_lever",
        ArticulationType.REVOLUTE,
        parent=cap,
        child=lockout,
        origin=Origin(xyz=(0.69, -0.50, 0.73)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.8, lower=-0.15, upper=1.05),
        motion_properties=MotionProperties(damping=2.5, friction=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    lockout = object_model.get_part("lockout_lever")
    yaw = object_model.get_articulation("tower_to_cap")
    lever = object_model.get_articulation("cap_to_lockout_lever")

    ctx.expect_contact(
        cap,
        tower,
        elem_a="upper_yaw_race",
        elem_b="top_service_deck",
        contact_tol=0.002,
        name="cap thrust ring is seated on tower deck",
    )
    ctx.expect_within(
        rotor,
        cap,
        axes="xz",
        inner_elem="main_shaft",
        outer_elem="front_bearing_seat",
        margin=0.09,
        name="shaft is centered in front bearing seat",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="y",
        elem_a="main_shaft",
        elem_b="rear_bearing_seat",
        min_overlap=0.025,
        name="shaft spans rear bearing seat",
    )
    ctx.expect_within(
        rotor,
        cap,
        axes="xz",
        inner_elem="blade_0_main_spar",
        outer_elem="front_guard_ring",
        margin=0.24,
        name="blade tips stay inside guard hoop",
    )
    closed_grip = ctx.part_element_world_aabb(lockout, elem="grip_knob")
    with ctx.pose({lever: 0.9}):
        moved_grip = ctx.part_element_world_aabb(lockout, elem="grip_knob")
        ctx.expect_origin_distance(lockout, cap, axes="yz", min_dist=0.0, max_dist=0.95, name="lockout lever remains on cap bracket")
    ctx.check(
        "lockout lever articulates on hinge",
        closed_grip is not None
        and moved_grip is not None
        and abs(float(moved_grip[0][1]) - float(closed_grip[0][1])) > 0.05,
        details=f"closed={closed_grip}, moved={moved_grip}",
    )

    ctx.check(
        "cap yaw joint has bounded travel",
        yaw.motion_limits is not None and yaw.motion_limits.lower == -0.78 and yaw.motion_limits.upper == 0.78,
        details=f"limits={yaw.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
