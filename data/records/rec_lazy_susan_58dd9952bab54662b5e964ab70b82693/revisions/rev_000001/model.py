from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z: float = 0.0):
    """CadQuery annular solid, authored in meters, from z to z + height."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z))
    )


def _cylinder(radius: float, height: float, z: float = 0.0):
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z))


def _outer_ring_shape():
    # One fused wooden annular tray: flat serving channel with raised inner and
    # outer lips, leaving a clear center well for the independent platter.
    floor = _annular_cylinder(0.238, 0.132, 0.020, z=0.006)
    outer_lip = _annular_cylinder(0.242, 0.226, 0.040, z=0.006)
    inner_lip = _annular_cylinder(0.142, 0.126, 0.036, z=0.006)
    return floor.union(outer_lip).union(inner_lip)


def _center_platter_shape():
    # A shallow round tray with a low raised rim, smaller and higher than the
    # surrounding serving ring.
    plate = _cylinder(0.101, 0.020, z=0.006)
    rim = _annular_cylinder(0.111, 0.094, 0.034, z=0.006)
    return plate.union(rim)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_bearing_lazy_susan")

    bamboo = model.material("warm_bamboo", rgba=(0.76, 0.55, 0.32, 1.0))
    edge_bamboo = model.material("darker_bamboo_edge", rgba=(0.50, 0.32, 0.17, 1.0))
    satin_metal = model.material("satin_bearing_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_rubber = model.material("dark_nonslip_rubber", rgba=(0.025, 0.022, 0.020, 1.0))
    brass = model.material("small_brass_inlay", rgba=(0.95, 0.72, 0.28, 1.0))

    base = model.part("lower_disc")
    base.visual(
        Cylinder(radius=0.255, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=bamboo,
        name="broad_base",
    )
    base.visual(
        mesh_from_cadquery(_annular_cylinder(0.262, 0.251, 0.008), "base_edge_band"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=edge_bamboo,
        name="base_edge_band",
    )
    base.visual(
        mesh_from_cadquery(_annular_cylinder(0.207, 0.175, 0.006), "outer_bearing_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=satin_metal,
        name="outer_bearing_race",
    )
    base.visual(
        Cylinder(radius=0.056, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=edge_bamboo,
        name="center_pedestal",
    )
    base.visual(
        mesh_from_cadquery(_annular_cylinder(0.061, 0.032, 0.004), "center_bearing_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=satin_metal,
        name="center_bearing_race",
    )
    for index, (x, y) in enumerate(
        ((0.175, 0.175), (-0.175, 0.175), (-0.175, -0.175), (0.175, -0.175))
    ):
        base.visual(
            Cylinder(radius=0.028, length=0.008),
            origin=Origin(xyz=(x, y, -0.0035)),
            material=dark_rubber,
            name=f"rubber_foot_{index}",
        )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_cadquery(_outer_ring_shape(), "outer_annular_tray", tolerance=0.0007),
        material=bamboo,
        name="annular_tray",
    )
    outer_ring.visual(
        mesh_from_cadquery(_annular_cylinder(0.206, 0.176, 0.006), "outer_lower_race"),
        material=satin_metal,
        name="outer_lower_race",
    )
    outer_ring.visual(
        mesh_from_cadquery(_annular_cylinder(0.218, 0.150, 0.002, z=0.025), "outer_nonslip_mat"),
        material=dark_rubber,
        name="outer_nonslip_mat",
    )
    outer_ring.visual(
        Box((0.040, 0.012, 0.0025)),
        origin=Origin(xyz=(0.184, 0.0, 0.027)),
        material=brass,
        name="outer_index",
    )

    center_platter = model.part("center_platter")
    center_platter.visual(
        mesh_from_cadquery(_center_platter_shape(), "center_round_platter", tolerance=0.0007),
        material=bamboo,
        name="round_platter",
    )
    center_platter.visual(
        Cylinder(radius=0.052, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_metal,
        name="center_lower_race",
    )
    center_platter.visual(
        Cylinder(radius=0.073, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=dark_rubber,
        name="center_nonslip_mat",
    )
    center_platter.visual(
        Box((0.030, 0.010, 0.0025)),
        origin=Origin(xyz=(0.062, 0.0, 0.029)),
        material=brass,
        name="center_index",
    )

    model.articulation(
        "outer_bearing",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0),
        motion_properties=MotionProperties(damping=0.015, friction=0.02),
    )

    model.articulation(
        "center_bearing",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=center_platter,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=5.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.015),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("lower_disc")
    outer_ring = object_model.get_part("outer_ring")
    center_platter = object_model.get_part("center_platter")
    outer_bearing = object_model.get_articulation("outer_bearing")
    center_bearing = object_model.get_articulation("center_bearing")

    ctx.check(
        "outer ring has a vertical continuous bearing",
        outer_bearing.articulation_type == ArticulationType.CONTINUOUS
        and tuple(outer_bearing.axis) == (0.0, 0.0, 1.0),
        details=f"type={outer_bearing.articulation_type}, axis={outer_bearing.axis}",
    )
    ctx.check(
        "center platter has its own vertical continuous bearing",
        center_bearing.articulation_type == ArticulationType.CONTINUOUS
        and tuple(center_bearing.axis) == (0.0, 0.0, 1.0),
        details=f"type={center_bearing.articulation_type}, axis={center_bearing.axis}",
    )

    ctx.expect_gap(
        outer_ring,
        base,
        axis="z",
        positive_elem="outer_lower_race",
        negative_elem="outer_bearing_race",
        max_gap=0.001,
        max_penetration=0.0,
        name="outer ring rides on lower bearing race",
    )
    ctx.expect_overlap(
        outer_ring,
        base,
        axes="xy",
        elem_a="outer_lower_race",
        elem_b="outer_bearing_race",
        min_overlap=0.025,
        name="outer bearing races are concentrically engaged",
    )
    ctx.expect_gap(
        center_platter,
        base,
        axis="z",
        positive_elem="center_lower_race",
        negative_elem="center_bearing_race",
        max_gap=0.001,
        max_penetration=0.0,
        name="center platter rides on its separate bearing race",
    )
    ctx.expect_overlap(
        center_platter,
        base,
        axes="xy",
        elem_a="center_lower_race",
        elem_b="center_bearing_race",
        min_overlap=0.025,
        name="center bearing races are concentrically engaged",
    )

    def marker_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    outer_rest = marker_center(outer_ring, "outer_index")
    center_rest = marker_center(center_platter, "center_index")

    with ctx.pose({outer_bearing: math.pi / 2.0, center_bearing: 0.0}):
        outer_rotated = marker_center(outer_ring, "outer_index")
        center_after_outer = marker_center(center_platter, "center_index")

    ctx.check(
        "outer bearing rotates the serving ring independently",
        outer_rest is not None
        and outer_rotated is not None
        and outer_rotated[1] > outer_rest[1] + 0.15
        and abs(outer_rotated[0]) < 0.035,
        details=f"rest={outer_rest}, rotated={outer_rotated}",
    )
    ctx.check(
        "outer bearing motion does not carry the center platter",
        center_rest is not None
        and center_after_outer is not None
        and abs(center_after_outer[0] - center_rest[0]) < 0.002
        and abs(center_after_outer[1] - center_rest[1]) < 0.002,
        details=f"rest={center_rest}, after_outer={center_after_outer}",
    )

    with ctx.pose({outer_bearing: 0.0, center_bearing: -math.pi / 2.0}):
        center_rotated = marker_center(center_platter, "center_index")
        outer_after_center = marker_center(outer_ring, "outer_index")

    ctx.check(
        "center bearing rotates the raised center platter independently",
        center_rest is not None
        and center_rotated is not None
        and center_rotated[1] < center_rest[1] - 0.045
        and abs(center_rotated[0]) < 0.025,
        details=f"rest={center_rest}, rotated={center_rotated}",
    )
    ctx.check(
        "center bearing motion does not carry the outer serving ring",
        outer_rest is not None
        and outer_after_center is not None
        and abs(outer_after_center[0] - outer_rest[0]) < 0.002
        and abs(outer_after_center[1] - outer_rest[1]) < 0.002,
        details=f"rest={outer_rest}, after_center={outer_after_center}",
    )

    return ctx.report()


object_model = build_object_model()
