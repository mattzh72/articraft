from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rpy_from_z_axis(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotate a local +Z-aligned cylinder so its axis follows ``vector``."""
    x, y, z = vector
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 0.0:
        return (0.0, 0.0, 0.0)
    x /= length
    y /= length
    z /= length
    pitch = math.atan2(math.sqrt(x * x + y * y), z)
    yaw = math.atan2(y, x)
    return (0.0, pitch, yaw)


def _cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    material: Material,
    name: str,
    overrun: float = 0.0,
) -> None:
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        return
    ux, uy, uz = vx / length, vy / length, vz / length
    center = (
        (sx + ex) * 0.5,
        (sy + ey) * 0.5,
        (sz + ez) * 0.5,
    )
    part.visual(
        Cylinder(radius=radius, length=length + overrun),
        origin=Origin(xyz=center, rpy=_rpy_from_z_axis((ux, uy, uz))),
        material=material,
        name=name,
    )


def _pitched_roof_geometry(width: float, length: float, rise: float) -> MeshGeometry:
    """Closed triangular-prism roof with ridge running along local +Y."""
    half_w = width * 0.5
    half_l = length * 0.5
    geom = MeshGeometry()
    vertices = [
        (-half_w, -half_l, 0.0),
        (half_w, -half_l, 0.0),
        (0.0, -half_l, rise),
        (-half_w, half_l, 0.0),
        (half_w, half_l, 0.0),
        (0.0, half_l, rise),
    ]
    for vertex in vertices:
        geom.add_vertex(*vertex)
    for face in (
        (0, 1, 2),
        (3, 5, 4),
        (0, 2, 5),
        (0, 5, 3),
        (1, 4, 5),
        (1, 5, 2),
        (0, 3, 4),
        (0, 4, 1),
    ):
        geom.add_face(*face)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_traditional_windmill")

    concrete = Material("sealed_concrete", rgba=(0.56, 0.56, 0.52, 1.0))
    galvanized = Material("galvanized_steel", rgba=(0.64, 0.68, 0.66, 1.0))
    stainless = Material("stainless_hardware", rgba=(0.82, 0.84, 0.80, 1.0))
    bronze = Material("bronze_bearing", rgba=(0.62, 0.42, 0.20, 1.0))
    gasket = Material("black_epdm_gasket", rgba=(0.015, 0.014, 0.012, 1.0))
    weathered_wood = Material("sealed_oak", rgba=(0.57, 0.38, 0.18, 1.0))
    white_panel = Material("white_painted_panel", rgba=(0.88, 0.90, 0.86, 1.0))
    roof_mat = Material("terne_coated_roof", rgba=(0.36, 0.40, 0.39, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((1.85, 1.85, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="sealed_slab",
    )
    tower.visual(
        Cylinder(radius=0.46, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 3.62)),
        material=stainless,
        name="fixed_yaw_seat",
    )
    tower.visual(
        Box((0.86, 0.86, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 3.54)),
        material=galvanized,
        name="top_platform",
    )

    base_corners = [
        (-0.72, -0.72, 0.12),
        (0.72, -0.72, 0.12),
        (0.72, 0.72, 0.12),
        (-0.72, 0.72, 0.12),
    ]
    top_corners = [
        (-0.34, -0.34, 3.52),
        (0.34, -0.34, 3.52),
        (0.34, 0.34, 3.52),
        (-0.34, 0.34, 3.52),
    ]
    for i, (base, top) in enumerate(zip(base_corners, top_corners)):
        _cylinder_between(
            tower,
            base,
            top,
            0.040,
            material=galvanized,
            name=f"leg_{i}",
            overrun=0.08,
        )
        bx, by, _ = base
        tower.visual(
            Cylinder(radius=0.072, length=0.025),
            origin=Origin(xyz=(bx, by, 0.145)),
            material=stainless,
            name=f"anchor_washer_{i}",
        )
        tower.visual(
            Cylinder(radius=0.030, length=0.050),
            origin=Origin(xyz=(bx, by, 0.180)),
            material=stainless,
            name=f"anchor_bolt_{i}",
        )

    for i in range(4):
        j = (i + 1) % 4
        p0_base = base_corners[i]
        p1_base = base_corners[j]
        p0_top = top_corners[i]
        p1_top = top_corners[j]
        for level, z0, z1 in (("low", 0.55, 1.55), ("mid", 1.55, 2.55), ("high", 2.55, 3.43)):
            t0 = (z0 - 0.12) / (3.52 - 0.12)
            t1 = (z1 - 0.12) / (3.52 - 0.12)
            a0 = tuple(p0_base[k] + (p0_top[k] - p0_base[k]) * t0 for k in range(3))
            b0 = tuple(p1_base[k] + (p1_top[k] - p1_base[k]) * t0 for k in range(3))
            a1 = tuple(p0_base[k] + (p0_top[k] - p0_base[k]) * t1 for k in range(3))
            b1 = tuple(p1_base[k] + (p1_top[k] - p1_base[k]) * t1 for k in range(3))
            if (i + (0 if level == "low" else 1)) % 2 == 0:
                start, end = a0, b1
            else:
                start, end = b0, a1
            _cylinder_between(
                tower,
                start,
                end,
                0.014,
                material=galvanized,
                name=f"brace_{i}_{level}",
                overrun=0.06,
            )

    yaw_height = 3.70

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.43, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=stainless,
        name="lower_ring",
    )
    cap.visual(
        Cylinder(radius=0.52, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=gasket,
        name="drip_skirt",
    )
    cap.visual(
        Cylinder(radius=0.18, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=galvanized,
        name="yaw_pedestal",
    )
    cap.visual(
        Box((0.72, 0.54, 0.08)),
        origin=Origin(xyz=(0.0, -0.18, 0.24)),
        material=galvanized,
        name="saddle_plate",
    )
    cap.visual(
        Box((0.82, 0.96, 0.50)),
        origin=Origin(xyz=(0.0, -0.36, 0.48)),
        material=white_panel,
        name="sealed_nacelle",
    )
    cap.visual(
        Box((0.92, 1.04, 0.030)),
        origin=Origin(xyz=(0.0, -0.36, 0.725)),
        material=gasket,
        name="roof_gasket",
    )
    cap.visual(
        mesh_from_geometry(_pitched_roof_geometry(1.08, 1.26, 0.34), "cap_roof"),
        origin=Origin(xyz=(0.0, -0.36, 0.74)),
        material=roof_mat,
        name="overhung_roof",
    )
    cap.visual(
        Box((1.12, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, -1.005, 0.735)),
        material=roof_mat,
        name="front_drip_edge",
    )
    cap.visual(
        Box((1.12, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, 0.285, 0.735)),
        material=roof_mat,
        name="rear_drip_edge",
    )
    cap.visual(
        Cylinder(radius=0.150, length=0.120),
        origin=Origin(xyz=(0.0, -0.900, 0.48), rpy=_rpy_from_z_axis((0.0, 1.0, 0.0))),
        material=bronze,
        name="front_bushing",
    )
    cap.visual(
        Cylinder(radius=0.210, length=0.045),
        origin=Origin(xyz=(0.0, -0.840, 0.48), rpy=_rpy_from_z_axis((0.0, 1.0, 0.0))),
        material=stainless,
        name="bearing_flange",
    )
    cap.visual(
        Box((0.48, 0.22, 0.08)),
        origin=Origin(xyz=(0.0, -0.91, 0.65)),
        material=roof_mat,
        name="bearing_hood",
    )
    _cylinder_between(
        cap,
        (0.0, 0.10, 0.49),
        (0.0, 1.42, 0.49),
        0.026,
        material=galvanized,
        name="tail_boom",
        overrun=0.06,
    )
    cap.visual(
        Box((0.56, 0.035, 0.38)),
        origin=Origin(xyz=(0.0, 1.36, 0.56)),
        material=white_panel,
        name="tail_vane",
    )
    cap.visual(
        Box((0.70, 0.045, 0.120)),
        origin=Origin(xyz=(0.0, 1.36, 0.49)),
        material=galvanized,
        name="tail_clamp",
    )

    model.articulation(
        "tower_to_cap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, yaw_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.35, lower=-0.65, upper=0.65),
    )

    rotor = model.part("rotor")
    y_axis_rpy = _rpy_from_z_axis((0.0, 1.0, 0.0))
    rotor.visual(
        Cylinder(radius=0.050, length=0.300),
        origin=Origin(xyz=(0.0, -0.130, 0.0), rpy=y_axis_rpy),
        material=stainless,
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=0.205, length=0.170),
        origin=Origin(xyz=(0.0, -0.340, 0.0), rpy=y_axis_rpy),
        material=galvanized,
        name="hub_drum",
    )
    rotor.visual(
        Sphere(radius=0.105),
        origin=Origin(xyz=(0.0, -0.455, 0.0)),
        material=stainless,
        name="nose_cap",
    )
    rotor.visual(
        Cylinder(radius=0.265, length=0.018),
        origin=Origin(xyz=(0.0, -0.432, 0.0), rpy=y_axis_rpy),
        material=stainless,
        name="bolt_circle",
    )

    blade_y = -0.430
    for blade_index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        radial = (math.cos(angle), 0.0, math.sin(angle))
        transverse = (-math.sin(angle), 0.0, math.cos(angle))

        def blade_point(radius: float, offset: float = 0.0) -> tuple[float, float, float]:
            return (
                radial[0] * radius + transverse[0] * offset,
                blade_y,
                radial[2] * radius + transverse[2] * offset,
            )

        _cylinder_between(
            rotor,
            blade_point(0.060),
            blade_point(1.62),
            0.027,
            material=weathered_wood,
            name=f"blade_{blade_index}_spar",
            overrun=0.05,
        )
        for side, offset in (("a", -0.135), ("b", 0.135)):
            _cylinder_between(
                rotor,
                blade_point(0.36, offset),
                blade_point(1.56, offset),
                0.016,
                material=weathered_wood,
                name=f"blade_{blade_index}_rail_{side}",
                overrun=0.04,
            )
        for lath_index, radius in enumerate((0.48, 0.68, 0.88, 1.08, 1.28, 1.48)):
            _cylinder_between(
                rotor,
                blade_point(radius, -0.175),
                blade_point(radius, 0.175),
                0.012,
                material=weathered_wood,
                name=f"blade_{blade_index}_lath_{lath_index}",
                overrun=0.03,
            )
        _cylinder_between(
            rotor,
            blade_point(0.40, -0.145),
            blade_point(1.50, 0.145),
            0.010,
            material=stainless,
            name=f"blade_{blade_index}_strap",
            overrun=0.04,
        )

    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(0.0, -0.900, 0.48)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=8.0),
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
        elem_a="front_bushing",
        elem_b="shaft",
        reason="The stainless rotor shaft is intentionally captured inside the bronze front bearing seat.",
    )
    ctx.expect_within(
        rotor,
        cap,
        axes="xz",
        inner_elem="shaft",
        outer_elem="front_bushing",
        margin=0.0,
        name="shaft centered in bearing",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="y",
        elem_a="shaft",
        elem_b="front_bushing",
        min_overlap=0.065,
        name="shaft retained in bushing",
    )
    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lower_ring",
        negative_elem="fixed_yaw_seat",
        name="cap ring bears on tower seat",
    )

    def _center_from_aabb(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_blade = _center_from_aabb(ctx.part_element_world_aabb(rotor, elem="blade_0_spar"))
    with ctx.pose({spin: 0.70}):
        spun_blade = _center_from_aabb(ctx.part_element_world_aabb(rotor, elem="blade_0_spar"))
    ctx.check(
        "rotor spins about bearing shaft",
        abs(spun_blade[0] - rest_blade[0]) > 0.10 and abs(spun_blade[2] - rest_blade[2]) > 0.10,
        details=f"rest={rest_blade}, spun={spun_blade}",
    )

    rest_tail = _center_from_aabb(ctx.part_element_world_aabb(cap, elem="tail_vane"))
    with ctx.pose({yaw: 0.35}):
        yawed_tail = _center_from_aabb(ctx.part_element_world_aabb(cap, elem="tail_vane"))
    ctx.check(
        "cap yaws on tower bearing",
        abs(yawed_tail[0] - rest_tail[0]) > 0.20,
        details=f"rest={rest_tail}, yawed={yawed_tail}",
    )

    return ctx.report()


object_model = build_object_model()
