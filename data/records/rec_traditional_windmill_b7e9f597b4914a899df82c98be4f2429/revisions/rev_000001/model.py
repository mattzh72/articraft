from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _cylinder_y(part, radius: float, length: float, xyz, material: Material, name: str) -> None:
    """Add a cylinder whose axis reads along the part's local Y direction."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_x(part, radius: float, length: float, xyz, material: Material, name: str) -> None:
    """Add a cylinder whose axis reads along the part's local X direction."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _xz_beam(part, p0, p1, *, depth_y: float, thickness: float, material: Material, name: str) -> None:
    """A rectangular beam between two local XZ points at a shared Y station."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    angle = math.atan2(dz, dx)
    part.visual(
        Box((length, depth_y, thickness)),
        origin=Origin(
            xyz=((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0),
            rpy=(0.0, -angle, 0.0),
        ),
        material=material,
        name=name,
    )


def _radial_box(part, theta: float, radius: float, *, length: float, depth_y: float, width: float, y: float, material: Material, name: str) -> None:
    part.visual(
        Box((length, depth_y, width)),
        origin=Origin(
            xyz=(radius * math.cos(theta), y, radius * math.sin(theta)),
            rpy=(0.0, -theta, 0.0),
        ),
        material=material,
        name=name,
    )


def _tangent_box(part, theta: float, radius: float, *, length: float, depth_y: float, width: float, y: float, material: Material, name: str) -> None:
    part.visual(
        Box((length, depth_y, width)),
        origin=Origin(
            xyz=(radius * math.cos(theta), y, radius * math.sin(theta)),
            rpy=(0.0, -(theta + math.pi / 2.0), 0.0),
        ),
        material=material,
        name=name,
    )


def _blade_point(theta: float, radius: float, tangent_offset: float, y: float) -> tuple[float, float, float]:
    radial = (math.cos(theta), 0.0, math.sin(theta))
    tangent = (-math.sin(theta), 0.0, math.cos(theta))
    return (
        radius * radial[0] + tangent_offset * tangent[0],
        y,
        radius * radial[2] + tangent_offset * tangent[2],
    )


def _tower_radius_at(z: float) -> float:
    # Matches the visible tapered-shell proportions used below.
    return 0.72 + (0.48 - 0.72) * ((z - 0.20) / (3.20 - 0.20))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_traditional_windmill")

    concrete = model.material("weathered_concrete", rgba=(0.47, 0.45, 0.41, 1.0))
    whitewash = model.material("scuffed_whitewash", rgba=(0.76, 0.73, 0.64, 1.0))
    dark_wood = model.material("tarred_oak", rgba=(0.19, 0.12, 0.07, 1.0))
    painted_wood = model.material("durable_oxide_paint", rgba=(0.18, 0.30, 0.34, 1.0))
    roof_black = model.material("oiled_roofing", rgba=(0.06, 0.06, 0.055, 1.0))
    oak = model.material("sealed_oak_lattice", rgba=(0.56, 0.38, 0.19, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.60, 0.62, 0.60, 1.0))
    dark_metal = model.material("greased_dark_steel", rgba=(0.08, 0.085, 0.08, 1.0))
    glass_dark = model.material("dark_glass", rgba=(0.02, 0.035, 0.045, 1.0))

    # Root tower: a squat, thick-walled tapered tower on a rugged utility slab.
    tower = model.part("tower")
    tower.visual(
        Box((1.86, 1.86, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=concrete,
        name="foundation_slab",
    )
    tower.visual(
        Box((1.52, 1.52, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.32), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=concrete,
        name="octagonal_plinth",
    )
    shell = LatheGeometry.from_shell_profiles(
        [(0.72, 0.20), (0.68, 0.72), (0.60, 1.70), (0.54, 2.55), (0.48, 3.20)],
        [(0.52, 0.28), (0.48, 0.92), (0.41, 1.80), (0.36, 2.55), (0.31, 3.12)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )
    tower.visual(
        mesh_from_geometry(shell, "tapered_tower_shell"),
        origin=Origin(),
        material=whitewash,
        name="tapered_tower_shell",
    )

    # Steel hoop bands and an exposed yaw-seat plate reinforce the masonry shell.
    for idx, z in enumerate((0.68, 1.46, 2.34)):
        band_radius = _tower_radius_at(z) + 0.012
        tower.visual(
            mesh_from_geometry(TorusGeometry(band_radius, 0.026, radial_segments=16, tubular_segments=64), f"tower_hoop_{idx}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=galvanized,
            name=f"tower_hoop_{idx}",
        )
    tower.visual(
        Cylinder(radius=0.54, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 3.16)),
        material=dark_metal,
        name="yaw_seat_plate",
    )

    # Surface-mounted arched door, windows, service ladder, and bolted straps.
    tower.visual(
        Box((0.36, 0.040, 0.58)),
        origin=Origin(xyz=(0.0, -0.725, 0.61)),
        material=dark_wood,
        name="service_door",
    )
    tower.visual(
        Box((0.47, 0.055, 0.08)),
        origin=Origin(xyz=(0.0, -0.752, 0.93)),
        material=galvanized,
        name="door_lintel_strap",
    )
    for x, name in ((-0.235, "door_jamb_0"), (0.235, "door_jamb_1")):
        tower.visual(
            Box((0.055, 0.055, 0.72)),
            origin=Origin(xyz=(x, -0.752, 0.62)),
            material=galvanized,
            name=name,
        )
    for idx, z in enumerate((1.45, 2.17)):
        tower.visual(
            Box((0.32, 0.035, 0.28)),
            origin=Origin(xyz=(0.0, -(_tower_radius_at(z) + 0.018), z)),
            material=glass_dark,
            name=f"front_window_{idx}",
        )
        tower.visual(
            Box((0.42, 0.050, 0.045)),
            origin=Origin(xyz=(0.0, -(_tower_radius_at(z) + 0.040), z + 0.158)),
            material=galvanized,
            name=f"window_top_strap_{idx}",
        )
        tower.visual(
            Box((0.42, 0.050, 0.045)),
            origin=Origin(xyz=(0.0, -(_tower_radius_at(z) + 0.040), z - 0.158)),
            material=galvanized,
            name=f"window_sill_strap_{idx}",
        )
    for x, name in ((-0.18, "ladder_rail_0"), (0.18, "ladder_rail_1")):
        tower.visual(
            Box((0.045, 0.220, 1.86)),
            origin=Origin(xyz=(x, _tower_radius_at(1.75) + 0.060, 1.75)),
            material=dark_metal,
            name=name,
        )
    for idx, z in enumerate((0.94, 1.20, 1.46, 1.72, 1.98, 2.24, 2.50)):
        tower.visual(
            Box((0.45, 0.045, 0.035)),
            origin=Origin(xyz=(0.0, _tower_radius_at(z) + 0.085, z)),
            material=dark_metal,
            name=f"ladder_rung_{idx}",
        )
    for idx, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        z = 1.70
        r = _tower_radius_at(z) + 0.022
        tower.visual(
            Box((0.075, 0.045, 2.76)),
            origin=Origin(xyz=(r * math.cos(angle), r * math.sin(angle), z), rpy=(0.0, 0.0, angle + math.pi / 2.0)),
            material=galvanized,
            name=f"vertical_corner_strap_{idx}",
        )

    # Rotating cap/nacelle assembly.  It sits on a visible turntable bearing and
    # carries the front rotor shaft through a torus-style bearing ring.
    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.52, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_metal,
        name="turntable_plate",
    )
    cap.visual(
        mesh_from_geometry(TorusGeometry(0.43, 0.035, radial_segments=16, tubular_segments=64), "cap_yaw_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=galvanized,
        name="yaw_bearing_ring",
    )
    for idx, (x, y) in enumerate(((0.39, 0.0), (0.0, 0.39), (-0.39, 0.0), (0.0, -0.39))):
        cap.visual(
            Sphere(radius=0.055),
            origin=Origin(xyz=(x, y, 0.085)),
            material=galvanized,
            name=f"yaw_roller_{idx}",
        )
    cap.visual(
        Box((0.92, 0.78, 0.48)),
        origin=Origin(xyz=(0.0, -0.06, 0.36)),
        material=painted_wood,
        name="machinery_house",
    )
    cap.visual(
        Box((1.04, 0.90, 0.10)),
        origin=Origin(xyz=(0.0, -0.06, 0.12)),
        material=galvanized,
        name="cap_base_skid",
    )
    cap.visual(
        Box((0.72, 1.04, 0.075)),
        origin=Origin(xyz=(-0.26, -0.06, 0.71), rpy=(0.0, -0.52, 0.0)),
        material=roof_black,
        name="roof_panel_0",
    )
    cap.visual(
        Box((0.72, 1.04, 0.075)),
        origin=Origin(xyz=(0.26, -0.06, 0.71), rpy=(0.0, 0.52, 0.0)),
        material=roof_black,
        name="roof_panel_1",
    )
    _cylinder_y(cap, 0.035, 1.08, (0.0, -0.06, 0.84), roof_black, "roof_ridge_cap")
    cap.visual(
        Box((0.48, 0.10, 0.16)),
        origin=Origin(xyz=(0.0, -0.47, 0.40)),
        material=dark_metal,
        name="front_bearing_block",
    )
    bearing = TorusGeometry(0.130, 0.035, radial_segments=18, tubular_segments=64).rotate_x(math.pi / 2.0)
    cap.visual(
        mesh_from_geometry(bearing, "front_bearing_ring"),
        origin=Origin(xyz=(0.0, -0.54, 0.37)),
        material=galvanized,
        name="front_bearing_ring",
    )
    for idx, angle in enumerate([i * math.tau / 8.0 for i in range(8)]):
        _cylinder_y(
            cap,
            0.018,
            0.024,
            (0.130 * math.cos(angle), -0.586, 0.37 + 0.130 * math.sin(angle)),
            dark_metal,
            f"bearing_bolt_{idx}",
        )
    for x, name in ((-0.25, "bearing_side_frame_0"), (0.25, "bearing_side_frame_1")):
        cap.visual(
            Box((0.075, 0.42, 0.075)),
            origin=Origin(xyz=(x, -0.48, 0.27)),
            material=galvanized,
            name=name,
        )
        cap.visual(
            Box((0.065, 0.42, 0.055)),
            origin=Origin(xyz=(x, -0.48, 0.50), rpy=(0.16, 0.0, 0.0)),
            material=galvanized,
            name=f"{name}_upper_brace",
        )
    cap.visual(
        Box((0.12, 1.34, 0.09)),
        origin=Origin(xyz=(0.0, 0.78, 0.37)),
        material=dark_wood,
        name="tail_boom",
    )
    cap.visual(
        Box((0.58, 0.055, 0.38)),
        origin=Origin(xyz=(0.0, 1.46, 0.48)),
        material=painted_wood,
        name="tail_vane",
    )
    cap.visual(
        Box((0.56, 0.060, 0.055)),
        origin=Origin(xyz=(0.0, 1.36, 0.29), rpy=(0.38, 0.0, 0.0)),
        material=galvanized,
        name="tail_vane_brace",
    )

    # Spinning rotor: stout iron shaft, bolted hub, timber spars, ladder-like
    # sail lattice, and metal diagonal straps.  The child frame is the shaft
    # centerline at the front bearing.
    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.055, length=0.24),
        origin=Origin(xyz=(0.0, 0.00, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="shaft",
    )
    _cylinder_y(rotor, 0.18, 0.17, (0.0, -0.14, 0.0), galvanized, "hub_drum")
    _cylinder_y(rotor, 0.23, 0.050, (0.0, -0.245, 0.0), dark_metal, "front_hub_flange")
    for idx, angle in enumerate([i * math.tau / 8.0 for i in range(8)]):
        _cylinder_y(
            rotor,
            0.023,
            0.040,
            (0.122 * math.cos(angle), -0.282, 0.122 * math.sin(angle)),
            dark_metal,
            f"hub_bolt_{idx}",
        )

    blade_y = -0.245
    for blade_idx, theta in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        _radial_box(
            rotor,
            theta,
            0.88,
            length=1.50,
            depth_y=0.070,
            width=0.090,
            y=blade_y,
            material=oak,
            name=f"blade_{blade_idx}_spar",
        )
        for side_idx, offset in enumerate((-0.18, 0.18)):
            p0 = _blade_point(theta, 0.42, offset, blade_y - 0.006)
            p1 = _blade_point(theta, 1.52, offset, blade_y - 0.006)
            _xz_beam(
                rotor,
                p0,
                p1,
                depth_y=0.044,
                thickness=0.042,
                material=oak,
                name=f"blade_{blade_idx}_rail_{side_idx}",
            )
        for rung_idx, r in enumerate((0.42, 0.67, 0.92, 1.17, 1.42)):
            rung_width = 0.38 + 0.14 * ((r - 0.42) / 1.00)
            _tangent_box(
                rotor,
                theta,
                r,
                length=rung_width,
                depth_y=0.052,
                width=0.040,
                y=blade_y - 0.012,
                material=oak,
                name=f"blade_{blade_idx}_rung_{rung_idx}",
            )
        _xz_beam(
            rotor,
            _blade_point(theta, 0.38, -0.20, blade_y - 0.045),
            _blade_point(theta, 1.50, 0.25, blade_y - 0.045),
            depth_y=0.026,
            thickness=0.026,
            material=galvanized,
            name=f"blade_{blade_idx}_diagonal_0",
        )
        _xz_beam(
            rotor,
            _blade_point(theta, 0.48, 0.20, blade_y - 0.048),
            _blade_point(theta, 1.55, -0.25, blade_y - 0.048),
            depth_y=0.026,
            thickness=0.026,
            material=galvanized,
            name=f"blade_{blade_idx}_diagonal_1",
        )
        _tangent_box(
            rotor,
            theta,
            1.61,
            length=0.66,
            depth_y=0.060,
            width=0.060,
            y=blade_y - 0.010,
            material=galvanized,
            name=f"blade_{blade_idx}_tip_shoe",
        )

    model.articulation(
        "cap_yaw",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 3.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.35, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(0.0, -0.62, 0.37)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("cap_yaw")
    spin = object_model.get_articulation("rotor_spin")

    ctx.allow_overlap(
        cap,
        rotor,
        elem_a="front_bearing_block",
        elem_b="shaft",
        reason="The simplified pillow-block bearing is shown as a solid casting with the rotor shaft passing through its hidden bore.",
    )

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0005,
        name="cap turntable seats on tower",
    )
    ctx.expect_overlap(
        cap,
        tower,
        axes="xy",
        min_overlap=0.42,
        elem_a="turntable_plate",
        elem_b="yaw_seat_plate",
        name="yaw bearing has broad tower footprint",
    )
    ctx.expect_within(
        rotor,
        cap,
        axes="xz",
        margin=0.050,
        inner_elem="shaft",
        outer_elem="front_bearing_ring",
        name="rotor shaft is centered in bearing ring",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="y",
        min_overlap=0.050,
        elem_a="shaft",
        elem_b="front_bearing_ring",
        name="shaft passes through bearing throat",
    )
    ctx.expect_within(
        rotor,
        cap,
        axes="xz",
        margin=0.010,
        inner_elem="shaft",
        outer_elem="front_bearing_block",
        name="shaft is centered in pillow block bore",
    )
    ctx.expect_overlap(
        cap,
        rotor,
        axes="y",
        min_overlap=0.015,
        elem_a="front_bearing_block",
        elem_b="shaft",
        name="shaft penetrates bearing block bore",
    )

    rest_rotor_pos = ctx.part_world_position(rotor)
    with ctx.pose({yaw: 0.72}):
        yawed_rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "cap yaw carries rotor around tower",
        rest_rotor_pos is not None
        and yawed_rotor_pos is not None
        and abs(yawed_rotor_pos[0] - rest_rotor_pos[0]) > 0.25
        and yawed_rotor_pos[1] > rest_rotor_pos[1] + 0.08,
        details=f"rest={rest_rotor_pos}, yawed={yawed_rotor_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    rest_blade = _aabb_center(ctx.part_element_world_aabb(rotor, elem="blade_0_spar"))
    with ctx.pose({spin: 0.75}):
        spun_blade = _aabb_center(ctx.part_element_world_aabb(rotor, elem="blade_0_spar"))
    ctx.check(
        "rotor spin moves blade lattice",
        rest_blade is not None
        and spun_blade is not None
        and abs(spun_blade[2] - rest_blade[2]) > 0.35,
        details=f"rest={rest_blade}, spun={spun_blade}",
    )

    return ctx.report()


object_model = build_object_model()
