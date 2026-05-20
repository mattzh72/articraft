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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    dark_plastic = model.material("dark_plastic", rgba=(0.16, 0.17, 0.19, 1.0))
    black_metal = model.material("black_metal", rgba=(0.12, 0.12, 0.13, 1.0))
    silver_metal = model.material("silver_metal", rgba=(0.77, 0.79, 0.82, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.42, 0.45, 0.5, 1.0))

    def xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
        return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]

    def yz_section(width: float, height: float, radius: float, x: float) -> list[tuple[float, float, float]]:
        return [(x, y, z) for y, z in rounded_rect_profile(width, height, radius)]

    base = model.part("base")

    base_plate_geom = ExtrudeGeometry(rounded_rect_profile(0.24, 0.18, 0.055), 0.022)
    base_plate_mesh = mesh_from_geometry(base_plate_geom, "base_plate")
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_plastic,
        name="base_plate",
    )

    pedestal_geom = section_loft(
        [
            [(x - 0.045, y, z) for x, y, z in xy_section(0.076, 0.060, 0.020, 0.022)],
            [(x - 0.040, y, z) for x, y, z in xy_section(0.056, 0.046, 0.018, 0.150)],
            [(x - 0.018, y, z) for x, y, z in xy_section(0.040, 0.034, 0.013, 0.250)],
        ]
    )
    pedestal_mesh = mesh_from_geometry(pedestal_geom, "pedestal")
    base.visual(pedestal_mesh, material=dark_plastic, name="pedestal")

    yoke_geom = tube_from_spline_points(
        [
            (-0.028, -0.014, 0.212),
            (-0.026, -0.050, 0.245),
            (-0.010, -0.078, 0.300),
            (0.006, -0.094, 0.327),
            (-0.020, -0.094, 0.352),
            (-0.050, 0.000, 0.370),
            (-0.020, 0.094, 0.352),
            (0.006, 0.094, 0.327),
            (-0.010, 0.078, 0.300),
            (-0.026, 0.050, 0.245),
            (-0.028, 0.014, 0.212),
        ],
        radius=0.0085,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    yoke_mesh = mesh_from_geometry(yoke_geom, "yoke")
    base.visual(yoke_mesh, material=dark_plastic, name="yoke")

    bushing_radius = 0.014
    bushing_length = 0.024
    base.visual(
        Cylinder(radius=bushing_radius, length=bushing_length),
        origin=Origin(xyz=(0.012, -0.089, 0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="left_bushing",
    )
    base.visual(
        Cylinder(radius=bushing_radius, length=bushing_length),
        origin=Origin(xyz=(0.012, 0.089, 0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="right_bushing",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 0.34)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    head = model.part("head")

    head_shell_geom = section_loft(
        [
            yz_section(0.118, 0.110, 0.026, -0.020),
            yz_section(0.142, 0.160, 0.038, 0.040),
            yz_section(0.112, 0.128, 0.030, 0.096),
        ]
    )
    head_shell_mesh = mesh_from_geometry(head_shell_geom, "head_shell")
    head.visual(head_shell_mesh, material=dark_plastic, name="head_shell")
    head.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(-0.004, -0.070, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(-0.004, 0.070, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="right_trunnion",
    )
    head.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.102, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_metal,
        name="motor_nose",
    )

    guard_geom = TorusGeometry(radius=0.152, tube=0.0045).rotate_y(math.pi / 2.0).translate(0.105, 0.0, 0.0)
    guard_geom.merge(
        TorusGeometry(radius=0.158, tube=0.0045).rotate_y(math.pi / 2.0).translate(0.145, 0.0, 0.0)
    )
    for ring_radius in (0.118, 0.082, 0.030):
        guard_geom.merge(
            TorusGeometry(radius=ring_radius, tube=0.0036).rotate_y(math.pi / 2.0).translate(0.105, 0.0, 0.0)
        )
    for ring_radius in (0.122, 0.088, 0.030):
        guard_geom.merge(
            TorusGeometry(radius=ring_radius, tube=0.0036).rotate_y(math.pi / 2.0).translate(0.145, 0.0, 0.0)
        )

    for angle in [i * (math.pi / 4.0) for i in range(8)]:
        c = math.cos(angle)
        s = math.sin(angle)
        guard_geom.merge(
            wire_from_points(
                [(0.105, 0.030 * c, 0.030 * s), (0.105, 0.152 * c, 0.152 * s)],
                radius=0.0030,
                cap_ends=True,
            )
        )
        guard_geom.merge(
            wire_from_points(
                [(0.145, 0.030 * c, 0.030 * s), (0.145, 0.158 * c, 0.158 * s)],
                radius=0.0030,
                cap_ends=True,
            )
        )
        guard_geom.merge(
            wire_from_points(
                [(0.105, 0.154 * c, 0.154 * s), (0.145, 0.154 * c, 0.154 * s)],
                radius=0.0030,
                cap_ends=True,
            )
        )

    for angle in (
        math.pi / 4.0,
        3.0 * math.pi / 4.0,
        5.0 * math.pi / 4.0,
        7.0 * math.pi / 4.0,
    ):
        c = math.cos(angle)
        s = math.sin(angle)
        guard_geom.merge(
            wire_from_points(
                [(0.060, 0.050 * c, 0.050 * s), (0.105, 0.145 * c, 0.145 * s)],
                radius=0.0040,
                cap_ends=True,
            )
        )

    guard_mesh = mesh_from_geometry(guard_geom, "guard")
    head.visual(guard_mesh, material=silver_metal, name="guard_assembly")
    head.inertial = Inertial.from_geometry(
        Box((0.31, 0.32, 0.32)),
        mass=1.0,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_metal,
        name="hub",
    )

    def blade_section(
        y: float,
        chord: float,
        thickness: float,
        pitch: float,
        z_offset: float,
    ) -> list[tuple[float, float, float]]:
        pts: list[tuple[float, float, float]] = []
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        for px, pz in rounded_rect_profile(thickness, chord, min(thickness * 0.45, chord * 0.18)):
            x_rot = px * cp + (pz + z_offset) * sp
            z_rot = -(px * sp) + (pz + z_offset) * cp
            pts.append((x_rot, y, z_rot))
        return pts

    blade_geom = section_loft(
        [
            blade_section(0.020, 0.050, 0.009, 0.45, -0.002),
            blade_section(0.075, 0.040, 0.0065, 0.28, 0.002),
            blade_section(0.118, 0.020, 0.0030, 0.12, 0.006),
        ]
    )
    blade_mesh = mesh_from_geometry(blade_geom, "rotor_blade")
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        rotor.visual(
            blade_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=blade_gray,
            name=f"blade_{idx}",
        )

    rotor.inertial = Inertial.from_geometry(Cylinder(radius=0.12, length=0.028), mass=0.25)

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.012, 0.0, 0.325)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=math.radians(-25.0),
            upper=math.radians(45.0),
        ),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.126, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head")
    spin = object_model.get_articulation("head_to_rotor")

    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        margin=0.006,
        name="rotor stays inside the fan guard footprint at rest",
    )
    ctx.expect_overlap(
        rotor,
        head,
        axes="yz",
        min_overlap=0.18,
        name="fan head is substantially larger than the support and encloses the rotor",
    )
    ctx.expect_origin_gap(
        head,
        base,
        axis="z",
        min_gap=0.20,
        name="fan head sits clearly above the desk base",
    )

    rest_guard = ctx.part_element_world_aabb(head, elem="guard_assembly")
    with ctx.pose({tilt: tilt.motion_limits.upper, spin: 1.4}):
        ctx.expect_within(
            rotor,
            head,
            axes="yz",
            margin=0.006,
            name="rotor stays within the guard while tilted up",
        )
        tilted_guard = ctx.part_element_world_aabb(head, elem="guard_assembly")

    rest_max_z = rest_guard[1][2] if rest_guard is not None else None
    tilted_max_z = tilted_guard[1][2] if tilted_guard is not None else None
    ctx.check(
        "positive tilt raises the fan head",
        rest_max_z is not None and tilted_max_z is not None and tilted_max_z > rest_max_z + 0.05,
        details=f"rest_guard_max_z={rest_max_z}, tilted_guard_max_z={tilted_max_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
