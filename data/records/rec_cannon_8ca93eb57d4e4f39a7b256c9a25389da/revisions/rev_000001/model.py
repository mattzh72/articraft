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
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _segment_origin(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[float, Origin]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.sqrt(dx * dx + dy * dy))
    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return length, Origin(xyz=center, rpy=(0.0, pitch, yaw))


def _make_barrel_mesh(segments: int = 56) -> MeshGeometry:
    """Long 12-pounder-style smooth-bore tube with muzzle bore left open."""

    # x is the bore axis; y/z are radial.  The trunnion pivot is x=0.
    outer_profile = [
        (-0.62, 0.045),  # cascabel button
        (-0.57, 0.085),
        (-0.50, 0.085),
        (-0.46, 0.145),
        (-0.38, 0.195),  # heavy breech reinforce
        (-0.18, 0.188),
        (0.05, 0.170),
        (0.32, 0.166),
        (0.42, 0.182),  # trunnion reinforce band
        (0.52, 0.160),
        (0.90, 0.142),
        (1.28, 0.122),
        (1.48, 0.115),
        (1.56, 0.145),  # muzzle swell
        (1.64, 0.158),
        (1.70, 0.132),  # muzzle lip
    ]
    bore_radius = 0.056
    bore_back_x = 1.16

    mesh = MeshGeometry()

    def add_ring(x: float, radius: float) -> list[int]:
        indices = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            indices.append(mesh.add_vertex(x, radius * math.cos(a), radius * math.sin(a)))
        return indices

    outer_rings = [add_ring(x, r) for x, r in outer_profile]
    for ring_a, ring_b in zip(outer_rings[:-1], outer_rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            mesh.add_face(ring_a[i], ring_b[i], ring_b[j])
            mesh.add_face(ring_a[i], ring_b[j], ring_a[j])

    # Closed cascabel end.
    breech_center = mesh.add_vertex(outer_profile[0][0], 0.0, 0.0)
    for i in range(segments):
        mesh.add_face(breech_center, outer_rings[0][(i + 1) % segments], outer_rings[0][i])

    # Annular muzzle face and a visible smooth bore.
    muzzle_outer = outer_rings[-1]
    muzzle_inner = add_ring(outer_profile[-1][0] + 0.002, bore_radius)
    bore_inner = add_ring(bore_back_x, bore_radius)
    for i in range(segments):
        j = (i + 1) % segments
        mesh.add_face(muzzle_outer[i], muzzle_outer[j], muzzle_inner[j])
        mesh.add_face(muzzle_outer[i], muzzle_inner[j], muzzle_inner[i])
        mesh.add_face(muzzle_inner[i], muzzle_inner[j], bore_inner[j])
        mesh.add_face(muzzle_inner[i], bore_inner[j], bore_inner[i])

    # Dark-backed bore bottom: a cap well behind the muzzle, not a flat front disc.
    bore_back_center = mesh.add_vertex(bore_back_x, 0.0, 0.0)
    for i in range(segments):
        mesh.add_face(bore_back_center, bore_inner[i], bore_inner[(i + 1) % segments])

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="napoleonic_12_pounder_field_cannon")

    bronze = model.material("aged_bronze", rgba=(0.72, 0.47, 0.19, 1.0))
    dark_bronze = model.material("dark_bore_bronze", rgba=(0.06, 0.045, 0.035, 1.0))
    oak = model.material("oiled_oak", rgba=(0.50, 0.28, 0.12, 1.0))
    iron = model.material("blackened_iron", rgba=(0.035, 0.035, 0.032, 1.0))
    worn_iron = model.material("worn_iron_edge", rgba=(0.12, 0.12, 0.11, 1.0))

    carriage = model.part("carriage")
    # Main axle and axle bed.
    carriage.visual(
        Cylinder(radius=0.085, length=1.86),
        origin=Origin(xyz=(0.0, 0.0, 0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    carriage.visual(
        Box((0.34, 1.04, 0.12)),
        origin=Origin(xyz=(-0.02, 0.0, 0.80)),
        material=oak,
        name="axle_bed",
    )

    # Two upright wooden cheeks carrying the trunnions.
    for suffix, y in (("0", -0.39), ("1", 0.39)):
        carriage.visual(
            Box((1.12, 0.10, 0.24)),
            origin=Origin(xyz=(0.25, y, 0.91)),
            material=oak,
            name=f"cheek_{suffix}",
        )
        carriage.visual(
            Cylinder(radius=0.115, length=0.045),
            origin=Origin(xyz=(0.25, y, 1.06), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=f"trunnion_cap_{suffix}",
        )

    # Front and rear cross members tie the cheeks into one carriage.
    carriage.visual(
        Box((0.16, 0.86, 0.10)),
        origin=Origin(xyz=(0.72, 0.0, 0.82)),
        material=oak,
        name="front_transom",
    )
    carriage.visual(
        Box((0.18, 0.74, 0.10)),
        origin=Origin(xyz=(-0.34, 0.0, 0.72)),
        material=oak,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.62, 0.24, 0.10)),
        origin=Origin(xyz=(-0.22, 0.0, 0.72)),
        material=oak,
        name="center_stock",
    )

    # Paired trail beams converge to the limber connection at the rear.
    for suffix, y_front, y_rear in (("0", -0.30, -0.085), ("1", 0.30, 0.085)):
        length, origin = _segment_origin((-0.30, y_front, 0.72), (-1.95, y_rear, 0.40))
        carriage.visual(
            Box((length, 0.085, 0.105)),
            origin=origin,
            material=oak,
            name=f"trail_beam_{suffix}",
        )

    carriage.visual(
        Box((0.24, 0.34, 0.12)),
        origin=Origin(xyz=(-1.96, 0.0, 0.40)),
        material=oak,
        name="trail_head",
    )
    carriage.visual(
        Box((0.18, 0.30, 0.040)),
        origin=Origin(xyz=(-2.02, 0.0, 0.32)),
        material=iron,
        name="trail_shoe",
    )
    length, origin = _segment_origin((-0.40, 0.0, 0.68), (-1.96, 0.0, 0.42))
    carriage.visual(
        Box((length, 0.22, 0.090)),
        origin=origin,
        material=oak,
        name="trail_stock",
    )

    # Clevis lugs for the limber tow-hook plate hinge.
    carriage.visual(
        Box((0.115, 0.080, 0.12)),
        origin=Origin(xyz=(-2.13, -0.16, 0.40)),
        material=iron,
        name="tow_lug_0",
    )
    carriage.visual(
        Box((0.115, 0.080, 0.12)),
        origin=Origin(xyz=(-2.13, 0.16, 0.40)),
        material=iron,
        name="tow_lug_1",
    )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_make_barrel_mesh(), "bronze_barrel"),
        material=bronze,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.095, length=0.680),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="trunnion",
    )
    # Dark soot patch at the back of the visible bore.
    barrel.visual(
        Cylinder(radius=0.054, length=0.004),
        origin=Origin(xyz=(1.158, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_bronze,
        name="bore_shadow",
    )

    wheel_mesh = WheelGeometry(
        0.660,
        0.095,
        rim=WheelRim(inner_radius=0.455, flange_height=0.018, flange_thickness=0.010),
        hub=WheelHub(radius=0.130, width=0.145, cap_style="domed"),
        face=WheelFace(dish_depth=0.012, front_inset=0.003, rear_inset=0.003),
        spokes=WheelSpokes(style="straight", count=12, thickness=0.034, window_radius=0.030),
        bore=WheelBore(style="round", diameter=0.160),
    )
    tire_mesh = TireGeometry(
        0.720,
        0.120,
        inner_radius=0.652,
        sidewall=TireSidewall(style="square", bulge=0.01),
        shoulder=TireShoulder(width=0.010, radius=0.002),
    )
    for suffix, y in (("0", -0.83), ("1", 0.83)):
        wheel = model.part(f"wheel_{suffix}")
        wheel.visual(
            mesh_from_geometry(wheel_mesh, f"wood_spoked_wheel_{suffix}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=oak,
            name="spoked_wheel",
        )
        wheel.visual(
            mesh_from_geometry(tire_mesh, f"iron_tire_{suffix}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=worn_iron,
            name="iron_tire",
        )
        model.articulation(
            f"carriage_to_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel,
            origin=Origin(xyz=(0.0, y, 0.72)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=400.0, velocity=12.0),
        )

    tow_plate = model.part("tow_plate")
    tow_plate.visual(
        Cylinder(radius=0.026, length=0.360),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hinge_pin",
    )
    tow_plate.visual(
        Box((0.310, 0.190, 0.035)),
        origin=Origin(xyz=(-0.155, 0.0, -0.024)),
        material=iron,
        name="hook_plate",
    )
    tow_plate.visual(
        Box((0.145, 0.055, 0.030)),
        origin=Origin(xyz=(-0.340, 0.0, -0.024)),
        material=iron,
        name="loop_neck",
    )
    tow_plate.visual(
        mesh_from_geometry(TorusGeometry(0.070, 0.012).rotate_x(math.pi / 2.0), "tow_loop"),
        origin=Origin(xyz=(-0.435, 0.0, -0.024)),
        material=iron,
        name="tow_loop",
    )

    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.25, 0.0, 1.06)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.25, lower=-0.10, upper=0.35),
    )
    model.articulation(
        "carriage_to_tow_plate",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tow_plate,
        origin=Origin(xyz=(-2.13, 0.0, 0.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.0, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    tow_plate = object_model.get_part("tow_plate")
    barrel_joint = object_model.get_articulation("carriage_to_barrel")
    tow_joint = object_model.get_articulation("carriage_to_tow_plate")

    ctx.allow_overlap(
        carriage,
        tow_plate,
        elem_a="tow_lug_0",
        elem_b="hinge_pin",
        reason="The hinge pin is intentionally captured in the solid clevis lug proxy.",
    )
    ctx.allow_overlap(
        carriage,
        tow_plate,
        elem_a="tow_lug_1",
        elem_b="hinge_pin",
        reason="The hinge pin is intentionally captured in the solid clevis lug proxy.",
    )
    ctx.allow_overlap(
        carriage,
        wheel_0,
        elem_a="axle",
        elem_b="spoked_wheel",
        reason="The iron axle is intentionally represented as passing through the wooden hub bore.",
    )
    ctx.allow_overlap(
        carriage,
        wheel_1,
        elem_a="axle",
        elem_b="spoked_wheel",
        reason="The iron axle is intentionally represented as passing through the wooden hub bore.",
    )
    ctx.expect_overlap(
        carriage,
        tow_plate,
        axes="y",
        elem_a="tow_lug_0",
        elem_b="hinge_pin",
        min_overlap=0.025,
        name="tow pin retained in lug 0",
    )
    ctx.expect_overlap(
        carriage,
        tow_plate,
        axes="y",
        elem_a="tow_lug_1",
        elem_b="hinge_pin",
        min_overlap=0.025,
        name="tow pin retained in lug 1",
    )

    for part_name in ("carriage", "barrel", "wheel_0", "wheel_1", "tow_plate"):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)

    ctx.expect_overlap(
        carriage,
        wheel_1,
        axes="y",
        elem_a="axle",
        elem_b="spoked_wheel",
        min_overlap=0.06,
        name="positive wheel hub retained on axle",
    )
    ctx.expect_overlap(
        carriage,
        wheel_0,
        axes="y",
        elem_a="axle",
        elem_b="spoked_wheel",
        min_overlap=0.06,
        name="negative wheel hub retained on axle",
    )

    barrel_aabb = ctx.part_world_aabb(barrel)
    wheel_aabb = ctx.part_world_aabb(wheel_0)
    ctx.check(
        "long field-gun barrel",
        barrel_aabb is not None and (barrel_aabb[1][0] - barrel_aabb[0][0]) > 2.1,
        details=f"barrel_aabb={barrel_aabb!r}",
    )
    ctx.check(
        "large artillery wheel diameter",
        wheel_aabb is not None and 1.35 <= (wheel_aabb[1][2] - wheel_aabb[0][2]) <= 1.50,
        details=f"wheel_aabb={wheel_aabb!r}",
    )

    rest_barrel_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    with ctx.pose({barrel_joint: 0.30}):
        raised_barrel_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    ctx.check(
        "positive trunnion motion elevates muzzle",
        rest_barrel_aabb is not None
        and raised_barrel_aabb is not None
        and raised_barrel_aabb[1][2] > rest_barrel_aabb[1][2] + 0.25,
        details=f"rest={rest_barrel_aabb!r}, raised={raised_barrel_aabb!r}",
    )

    rest_tow_aabb = ctx.part_world_aabb(tow_plate)
    with ctx.pose({tow_joint: 0.55}):
        swung_tow_aabb = ctx.part_world_aabb(tow_plate)
    ctx.check(
        "tow plate swings on rear pin",
        rest_tow_aabb is not None
        and swung_tow_aabb is not None
        and swung_tow_aabb[1][2] > rest_tow_aabb[1][2] + 0.15,
        details=f"rest={rest_tow_aabb!r}, swung={swung_tow_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
