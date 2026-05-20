from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _octagon_section(rx: float, ry: float, z: float) -> list[tuple[float, float, float]]:
    return [
        (rx * math.cos(i * math.tau / 8.0 + math.tau / 16.0), ry * math.sin(i * math.tau / 8.0 + math.tau / 16.0), z)
        for i in range(8)
    ]


def _cap_roof_section(y: float) -> list[tuple[float, float, float]]:
    return [
        (-0.145, y, 0.295),
        (-0.118, y, 0.330),
        (0.000, y, 0.385),
        (0.118, y, 0.330),
        (0.145, y, 0.295),
        (0.115, y, 0.275),
        (-0.115, y, 0.275),
    ]


def _box_bar_between(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    thickness: float,
    depth: float,
    material,
    name: str,
) -> None:
    dx = p1[0] - p0[0]
    dz = p1[2] - p0[2]
    length = math.hypot(dx, dz)
    angle_y = math.atan2(-dz, dx)
    part.visual(
        Box((length, depth, thickness)),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, angle_y, 0.0),
        ),
        material=material,
        name=name,
    )


def _blade_point(theta: float, radius: float, tangent_offset: float, y: float) -> tuple[float, float, float]:
    radial = (math.cos(theta), 0.0, math.sin(theta))
    tangent = (-math.sin(theta), 0.0, math.cos(theta))
    return (
        radial[0] * radius + tangent[0] * tangent_offset,
        y,
        radial[2] * radius + tangent[2] * tangent_offset,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_traditional_windmill")

    painted_metal = model.material("warm_painted_metal", rgba=(0.88, 0.86, 0.78, 1.0))
    seam_paint = model.material("soft_seam_paint", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_metal = model.material("graphite_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed_metal = model.material("brushed_bearing_metal", rgba=(0.55, 0.58, 0.60, 1.0))
    slate_polymer = model.material("slate_polymer_cap", rgba=(0.18, 0.22, 0.25, 1.0))
    blade_paint = model.material("satin_blade_paint", rgba=(0.82, 0.78, 0.68, 1.0))
    elastomer = model.material("black_elastomer", rgba=(0.035, 0.035, 0.032, 1.0))

    tower = model.part("tower")
    tower_body = section_loft(
        [
            _octagon_section(0.205, 0.205, 0.045),
            _octagon_section(0.170, 0.170, 0.560),
            _octagon_section(0.115, 0.115, 1.140),
        ]
    )
    tower.visual(
        mesh_from_geometry(tower_body, "tapered_tower_body"),
        material=painted_metal,
        name="tapered_tower_body",
    )
    tower.visual(
        Cylinder(radius=0.285, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="weighted_base",
    )
    for i, (x, y) in enumerate(((0.20, 0.20), (-0.20, 0.20), (-0.20, -0.20), (0.20, -0.20))):
        tower.visual(
            Cylinder(radius=0.038, length=0.018),
            origin=Origin(xyz=(x, y, 0.009)),
            material=elastomer,
            name=f"foot_pad_{i}",
        )
    tower.visual(
        Cylinder(radius=0.128, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.170)),
        material=dark_metal,
        name="top_bearing_ring",
    )
    tower.visual(
        Cylinder(radius=0.066, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 1.162)),
        material=brushed_metal,
        name="yaw_bearing_post",
    )
    for angle, label in ((0.0, "front_seam"), (math.pi / 2.0, "side_seam_0"), (math.pi, "rear_seam"), (3.0 * math.pi / 2.0, "side_seam_1")):
        x = 0.118 * math.sin(angle)
        y = -0.118 * math.cos(angle)
        tower.visual(
            Box((0.018, 0.010, 0.880)),
            origin=Origin(xyz=(x, y, 0.590), rpy=(0.0, 0.0, angle)),
            material=seam_paint,
            name=label,
        )
    tower.visual(
        Box((0.098, 0.010, 0.180)),
        origin=Origin(xyz=(0.0, -0.171, 0.360)),
        material=seam_paint,
        name="service_door",
    )
    tower.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.038, -0.180, 0.365), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="door_pull",
    )

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.132, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=brushed_metal,
        name="yaw_skirt",
    )
    cap.visual(
        Cylinder(radius=0.082, length=0.125),
        origin=Origin(xyz=(0.0, -0.030, 0.100)),
        material=dark_metal,
        name="cap_pedestal",
    )
    cap.visual(
        Box((0.190, 0.300, 0.055)),
        origin=Origin(xyz=(0.0, -0.155, 0.150)),
        material=dark_metal,
        name="nacelle_saddle",
    )
    cap.visual(
        Box((0.030, 0.350, 0.170)),
        origin=Origin(xyz=(-0.072, -0.180, 0.240)),
        material=painted_metal,
        name="bearing_cheek_0",
    )
    cap.visual(
        Box((0.030, 0.350, 0.170)),
        origin=Origin(xyz=(0.072, -0.180, 0.240)),
        material=painted_metal,
        name="bearing_cheek_1",
    )
    cap.visual(
        mesh_from_geometry(section_loft([_cap_roof_section(-0.350), _cap_roof_section(0.040)]), "cap_roof"),
        material=slate_polymer,
        name="cap_roof",
    )
    cap.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.0435, tube=0.0205, radial_segments=20, tubular_segments=54),
            "front_bearing_seat",
        ),
        origin=Origin(xyz=(0.0, -0.340, 0.230), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="front_bearing_seat",
    )
    cap.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.0420, tube=0.0190, radial_segments=18, tubular_segments=48),
            "rear_bearing_seat",
        ),
        origin=Origin(xyz=(0.0, -0.160, 0.230), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="rear_bearing_seat",
    )
    cap.visual(
        Box((0.026, 0.370, 0.026)),
        origin=Origin(xyz=(0.0, 0.135, 0.225)),
        material=dark_metal,
        name="tail_boom",
    )
    cap.visual(
        Box((0.150, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, -0.020, 0.225)),
        material=dark_metal,
        name="rear_bridge",
    )
    cap.visual(
        Box((0.110, 0.014, 0.180)),
        origin=Origin(xyz=(0.0, 0.310, 0.250)),
        material=slate_polymer,
        name="tail_vane",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.023, length=0.260),
        origin=Origin(xyz=(0.0, 0.070, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="drive_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.068, length=0.105),
        origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="rotor_hub",
    )
    rotor.visual(
        Sphere(radius=0.042),
        origin=Origin(xyz=(0.0, -0.170, 0.0)),
        material=dark_metal,
        name="nose_cap",
    )

    lattice_y = -0.150
    for blade_index in range(4):
        theta = blade_index * math.pi / 2.0
        _box_bar_between(
            rotor,
            _blade_point(theta, 0.035, 0.0, lattice_y),
            _blade_point(theta, 0.620, 0.0, lattice_y),
            thickness=0.026,
            depth=0.026,
            material=blade_paint,
            name=f"blade_spar_{blade_index}",
        )
        for offset, side in ((-0.058, "rail_0"), (0.058, "rail_1")):
            _box_bar_between(
                rotor,
                _blade_point(theta, 0.145, offset, lattice_y),
                _blade_point(theta, 0.600, offset, lattice_y),
                thickness=0.014,
                depth=0.020,
                material=blade_paint,
                name=f"blade_{side}_{blade_index}",
            )
        for r, width, label in ((0.145, 0.118, "root"), (0.315, 0.136, "mid"), (0.485, 0.148, "outer"), (0.615, 0.130, "tip")):
            _box_bar_between(
                rotor,
                _blade_point(theta, r, -0.5 * width, lattice_y),
                _blade_point(theta, r, 0.5 * width, lattice_y),
                thickness=0.013,
                depth=0.021,
                material=blade_paint,
                name=f"blade_{label}_rib_{blade_index}",
            )
        for start_r, start_o, end_r, end_o, diag in (
            (0.160, -0.055, 0.315, 0.058, "diag_0"),
            (0.315, 0.058, 0.485, -0.064, "diag_1"),
            (0.315, -0.058, 0.485, 0.064, "diag_2"),
            (0.485, 0.064, 0.600, -0.050, "diag_3"),
        ):
            _box_bar_between(
                rotor,
                _blade_point(theta, start_r, start_o, lattice_y),
                _blade_point(theta, end_r, end_o, lattice_y),
                thickness=0.010,
                depth=0.018,
                material=seam_paint,
                name=f"blade_{diag}_{blade_index}",
            )

    model.articulation(
        "tower_to_cap",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 1.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.8),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(0.0, -0.340, 0.230)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_cap")
    spin = object_model.get_articulation("cap_to_rotor")

    ctx.allow_overlap(
        cap,
        rotor,
        elem_a="front_bearing_seat",
        elem_b="drive_shaft",
        reason="The drive shaft is intentionally captured in the front bearing seat as a close bushing fit.",
    )
    ctx.allow_overlap(
        cap,
        rotor,
        elem_a="rear_bearing_seat",
        elem_b="drive_shaft",
        reason="The drive shaft is intentionally captured in the rear bearing seat as a close bushing fit.",
    )

    ctx.expect_contact(
        cap,
        tower,
        elem_a="yaw_skirt",
        elem_b="top_bearing_ring",
        contact_tol=0.002,
        name="cap yaw skirt is seated on the tower bearing ring",
    )
    ctx.expect_within(
        rotor,
        cap,
        axes="xz",
        inner_elem="drive_shaft",
        outer_elem="front_bearing_seat",
        margin=0.006,
        name="drive shaft is centered in the front bearing seat",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="y",
        elem_a="drive_shaft",
        elem_b="front_bearing_seat",
        min_overlap=0.015,
        name="front bearing seat surrounds the drive shaft length",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="y",
        elem_a="drive_shaft",
        elem_b="rear_bearing_seat",
        min_overlap=0.012,
        name="rear bearing seat supports the drive shaft",
    )

    rest_rotor_aabb = ctx.part_world_aabb(rotor)
    with ctx.pose({spin: math.radians(18.0)}):
        turned_rotor_aabb = ctx.part_world_aabb(rotor)
    ctx.check(
        "rotor visibly rotates about its shaft",
        rest_rotor_aabb is not None
        and turned_rotor_aabb is not None
        and abs((rest_rotor_aabb[1][0] - rest_rotor_aabb[0][0]) - (turned_rotor_aabb[1][0] - turned_rotor_aabb[0][0])) > 0.003,
        details=f"rest={rest_rotor_aabb}, turned={turned_rotor_aabb}",
    )

    rest_position = ctx.part_world_position(rotor)
    with ctx.pose({yaw: 0.45}):
        yawed_position = ctx.part_world_position(rotor)
    ctx.check(
        "cap yaw carries the supported rotor around the tower axis",
        rest_position is not None
        and yawed_position is not None
        and abs(yawed_position[0] - rest_position[0]) > 0.050,
        details=f"rest={rest_position}, yawed={yawed_position}",
    )

    return ctx.report()


object_model = build_object_model()
