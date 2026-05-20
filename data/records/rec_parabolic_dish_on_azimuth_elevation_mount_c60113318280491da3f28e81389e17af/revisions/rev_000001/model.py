from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _yoke_bracket_shape() -> cq.Workplane:
    """Single welded rotating elevation bracket with bored side cheeks."""

    def box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
        return cq.Workplane("XY").box(*size).translate(center)

    base = box((0.58, 1.10, 0.070), (0.020, 0.000, 0.095))
    cheek_0 = box((0.20, 0.10, 0.76), (0.000, 0.480, 0.460))
    cheek_1 = box((0.20, 0.10, 0.76), (0.000, -0.480, 0.460))
    rear_bridge = box((0.08, 1.05, 0.16), (-0.095, 0.000, 0.190))

    bracket = base.union(cheek_0).union(cheek_1).union(rear_bridge)

    bore = (
        cq.Workplane("XY")
        .cylinder(1.30, 0.055)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0.0, 0.0, 0.600))
    )
    return bracket.cut(bore)


def _reflector_shell_mesh():
    outer_profile = [
        (0.030, 0.000),
        (0.095, 0.018),
        (0.185, 0.074),
        (0.285, 0.165),
        (0.365, 0.270),
        (0.415, 0.348),
    ]
    inner_profile = [
        (0.000, 0.018),
        (0.072, 0.032),
        (0.165, 0.088),
        (0.265, 0.178),
        (0.350, 0.282),
        (0.398, 0.340),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_ground_dish")

    pedestal_paint = model.material("pedestal_paint", rgba=(0.22, 0.25, 0.28, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.43, 0.46, 0.49, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    reflector_white = model.material("reflector_white", rgba=(0.88, 0.90, 0.86, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.68, 0.70, 0.72, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.68, 0.60, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_steel,
        name="ground_foot",
    )
    pedestal.visual(
        Box((0.50, 0.44, 0.48)),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=pedestal_paint,
        name="box_pedestal",
    )
    pedestal.visual(
        Box((0.020, 0.24, 0.23)),
        origin=Origin(xyz=(-0.260, 0.0, 0.310)),
        material=machinery_gray,
        name="service_panel",
    )
    pedestal.visual(
        Cylinder(radius=0.245, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        material=dark_steel,
        name="fixed_bearing_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.130, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.5625)),
        material=machinery_gray,
        name="azimuth_boss",
    )

    top_plate = model.part("top_plate")
    top_plate.visual(
        Cylinder(radius=0.290, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=machinery_gray,
        name="rotating_plate",
    )
    top_plate.visual(
        mesh_from_cadquery(_yoke_bracket_shape(), "elevation_yoke", tolerance=0.0008),
        material=machinery_gray,
        name="elevation_yoke",
    )
    bearing_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.078, tube=0.012, radial_segments=16, tubular_segments=48),
        "bearing_ring",
    )
    for index, y in enumerate((0.535, -0.535)):
        top_plate.visual(
            bearing_ring_mesh,
            origin=Origin(xyz=(0.0, y, 0.600), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"bearing_ring_{index}",
        )
    for index, (x, y) in enumerate(((0.20, 0.20), (0.20, -0.20), (-0.18, 0.20), (-0.18, -0.20))):
        top_plate.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, y, 0.066)),
            material=dark_steel,
            name=f"deck_bolt_{index}",
        )

    reflector = model.part("reflector")
    reflector.visual(
        Cylinder(radius=0.040, length=1.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    reflector.visual(
        Cylinder(radius=0.070, length=0.090),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="hub_barrel",
    )
    for index, y in enumerate((0.485, -0.485)):
        reflector.visual(
            Cylinder(radius=0.058, length=0.090),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"trunnion_collar_{index}",
        )

    reflector.visual(
        mesh_from_geometry(_reflector_shell_mesh(), "deep_reflector_shell"),
        origin=Origin(xyz=(0.105, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=reflector_white,
        name="deep_reflector_shell",
    )
    reflector.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.410, tube=0.015, radial_segments=18, tubular_segments=72),
            "reflector_rim",
        ),
        origin=Origin(xyz=(0.455, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="rim_stiffener",
    )

    rib_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.085, 0.0, 0.000),
                (0.175, 0.0, 0.145),
                (0.310, 0.0, 0.285),
                (0.435, 0.0, 0.390),
            ],
            radius=0.010,
            samples_per_segment=10,
            radial_segments=12,
            cap_ends=True,
        ),
        "rear_reflector_rib",
    )
    for index in range(4):
        reflector.visual(
            rib_mesh,
            origin=Origin(rpy=(index * math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"rear_rib_{index}",
        )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=top_plate,
        child=reflector,
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=0.9,
            lower=-math.radians(8.0),
            upper=math.radians(68.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    top_plate = object_model.get_part("top_plate")
    reflector = object_model.get_part("reflector")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    for collar_name in ("trunnion_collar_0", "trunnion_collar_1"):
        ctx.allow_overlap(
            reflector,
            top_plate,
            elem_a=collar_name,
            elem_b="elevation_yoke",
            reason="The trunnion collar is intentionally a light press fit in the side-support bearing bore.",
        )

    ctx.check(
        "azimuth uses a vertical continuous axis",
        azimuth.articulation_type == ArticulationType.CONTINUOUS and tuple(azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}",
    )
    ctx.check(
        "elevation uses a horizontal side-support axis",
        elevation.articulation_type == ArticulationType.REVOLUTE and tuple(elevation.axis) == (0.0, -1.0, 0.0),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )

    ctx.expect_gap(
        top_plate,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rotating_plate",
        negative_elem="azimuth_boss",
        name="rotating top plate sits on pedestal bearing",
    )
    ctx.expect_overlap(
        reflector,
        top_plate,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="elevation_yoke",
        min_overlap=0.82,
        name="trunnion spans both side supports",
    )
    for collar_name in ("trunnion_collar_0", "trunnion_collar_1"):
        ctx.expect_overlap(
            reflector,
            top_plate,
            axes="yz",
            elem_a=collar_name,
            elem_b="elevation_yoke",
            min_overlap=0.045,
            name=f"{collar_name} is seated in the bearing bore",
        )
    ctx.expect_within(
        reflector,
        top_plate,
        axes="y",
        inner_elem="deep_reflector_shell",
        outer_elem="elevation_yoke",
        margin=0.010,
        name="deep bowl clears between side supports",
    )

    def _aabb_center_z(part, elem: str) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return 0.5 * (lo[2] + hi[2])

    rest_rim_z = _aabb_center_z(reflector, "rim_stiffener")
    with ctx.pose({elevation: math.radians(45.0)}):
        raised_rim_z = _aabb_center_z(reflector, "rim_stiffener")
    ctx.check(
        "positive elevation raises the reflector mouth",
        rest_rim_z is not None and raised_rim_z is not None and raised_rim_z > rest_rim_z + 0.20,
        details=f"rest_z={rest_rim_z}, raised_z={raised_rim_z}",
    )

    rest_bearing = ctx.part_element_world_aabb(top_plate, elem="bearing_ring_0")
    with ctx.pose({azimuth: math.pi / 2.0}):
        turned_bearing = ctx.part_element_world_aabb(top_plate, elem="bearing_ring_0")
    if rest_bearing is not None and turned_bearing is not None:
        rest_center = (0.5 * (rest_bearing[0][0] + rest_bearing[1][0]), 0.5 * (rest_bearing[0][1] + rest_bearing[1][1]))
        turned_center = (
            0.5 * (turned_bearing[0][0] + turned_bearing[1][0]),
            0.5 * (turned_bearing[0][1] + turned_bearing[1][1]),
        )
    else:
        rest_center = None
        turned_center = None
    ctx.check(
        "azimuth turns the side-supported bracket around the pedestal",
        rest_center is not None
        and turned_center is not None
        and abs(turned_center[0] + rest_center[1]) < 0.040
        and abs(turned_center[1] - rest_center[0]) < 0.040,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
