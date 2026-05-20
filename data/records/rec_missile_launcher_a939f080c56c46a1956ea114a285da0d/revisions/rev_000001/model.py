from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _hollow_tube_x(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    """Open-ended tube with its axis along +X, starting at x=0."""
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length)
    inner = (
        cq.Workplane("YZ")
        .circle(inner_radius)
        .extrude(length + 0.04)
        .translate((-0.02, 0.0, 0.0))
    )
    return outer.cut(inner)


def _perforated_plate_x(
    thickness: float,
    width_y: float,
    height_z: float,
    hole_positions: list[tuple[float, float]],
    hole_radius: float,
) -> cq.Workplane:
    """A centered YZ plate with circular tube holes through X."""
    plate = (
        cq.Workplane("YZ")
        .rect(width_y, height_z)
        .extrude(thickness)
        .translate((-thickness / 2.0, 0.0, 0.0))
    )
    for y, z in hole_positions:
        cutter = (
            cq.Workplane("YZ")
            .center(y, z)
            .circle(hole_radius)
            .extrude(thickness * 3.0)
            .translate((-1.5 * thickness, 0.0, 0.0))
        )
        plate = plate.cut(cutter)
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="truck_bed_missile_launcher_module")

    olive = model.material("matte_olive_drab", rgba=(0.18, 0.23, 0.15, 1.0))
    dark_olive = model.material("dark_olive_steel", rgba=(0.10, 0.13, 0.10, 1.0))
    black = model.material("blackened_bore", rgba=(0.015, 0.018, 0.016, 1.0))
    worn = model.material("worn_steel_edges", rgba=(0.36, 0.36, 0.32, 1.0))

    base = model.part("base_frame")
    for y in (-0.57, 0.57):
        base.visual(
            Box((2.40, 0.12, 0.18)),
            origin=Origin(xyz=(0.0, y, 0.09)),
            material=dark_olive,
            name=f"side_rail_{0 if y < 0 else 1}",
        )
    for x in (-1.10, 1.10):
        base.visual(
            Box((0.20, 1.20, 0.18)),
            origin=Origin(xyz=(x, 0.0, 0.09)),
            material=dark_olive,
            name=f"end_crossmember_{0 if x < 0 else 1}",
        )
    base.visual(
        Box((0.82, 1.20, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=dark_olive,
        name="center_crossmember",
    )
    base.visual(
        Box((1.10, 1.10, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=olive,
        name="deck_plate",
    )
    base.visual(
        Cylinder(radius=0.38, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=worn,
        name="slewing_ring",
    )
    for i in range(8):
        angle = i * math.pi / 4.0
        base.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(0.50 * math.cos(angle), 0.50 * math.sin(angle), 0.249)),
            material=black,
            name=f"ring_bolt_{i}",
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.42, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=olive,
        name="rotating_plate",
    )
    turntable.visual(
        Cylinder(radius=0.31, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=worn,
        name="gear_drum",
    )
    turntable.visual(
        Box((0.46, 0.42, 0.36)),
        origin=Origin(xyz=(-0.05, 0.0, 0.24)),
        material=olive,
        name="pedestal",
    )
    turntable.visual(
        Box((0.36, 0.86, 0.08)),
        origin=Origin(xyz=(-0.28, 0.0, 0.10)),
        material=dark_olive,
        name="yoke_base_tie",
    )
    turntable.visual(
        Box((0.24, 0.08, 0.78)),
        origin=Origin(xyz=(-0.28, -0.40, 0.45)),
        material=olive,
        name="yoke_plate_0",
    )
    turntable.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(-0.28, -0.45, 0.78), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn,
        name="bearing_cap_0",
    )
    turntable.visual(
        Box((0.24, 0.08, 0.78)),
        origin=Origin(xyz=(-0.28, 0.40, 0.45)),
        material=olive,
        name="yoke_plate_1",
    )
    turntable.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(-0.28, 0.45, 0.78), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn,
        name="bearing_cap_1",
    )
    turntable.visual(
        Cylinder(radius=0.04, length=0.74),
        origin=Origin(xyz=(-0.38, 0.0, 0.83), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_olive,
        name="upper_tie_bar",
    )

    tube_bank = model.part("tube_bank")
    tube_positions = [(y, z) for z in (-0.11, 0.11) for y in (-0.22, 0.0, 0.22)]
    tube_mesh = mesh_from_cadquery(
        _hollow_tube_x(1.55, 0.085, 0.064),
        "hollow_launch_tube",
        tolerance=0.001,
        angular_tolerance=0.08,
    )
    collar_mesh = mesh_from_cadquery(
        _hollow_tube_x(0.08, 0.098, 0.064),
        "muzzle_collar",
        tolerance=0.001,
        angular_tolerance=0.08,
    )
    clamp_mesh = mesh_from_cadquery(
        _perforated_plate_x(0.06, 0.64, 0.43, tube_positions, 0.083),
        "perforated_tube_clamp",
        tolerance=0.001,
        angular_tolerance=0.08,
    )

    for row, z in enumerate((-0.11, 0.11)):
        for col, y in enumerate((-0.22, 0.0, 0.22)):
            tube_bank.visual(
                tube_mesh,
                origin=Origin(xyz=(0.08, y, z)),
                material=olive,
                name=f"tube_{row}_{col}",
            )
            tube_bank.visual(
                collar_mesh,
                origin=Origin(xyz=(1.55, y, z)),
                material=dark_olive,
                name=f"muzzle_collar_{row}_{col}",
            )
    tube_bank.visual(
        clamp_mesh,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=dark_olive,
        name="rear_clamp",
    )
    tube_bank.visual(
        clamp_mesh,
        origin=Origin(xyz=(1.40, 0.0, 0.0)),
        material=dark_olive,
        name="front_clamp",
    )
    for y in (-0.325, 0.325):
        tube_bank.visual(
            Box((1.62, 0.05, 0.08)),
            origin=Origin(xyz=(0.86, y, 0.0)),
            material=dark_olive,
            name=f"side_cradle_{0 if y < 0 else 1}",
        )
    tube_bank.visual(
        Cylinder(radius=0.065, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn,
        name="trunnion_axle",
    )

    model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=tube_bank,
        origin=Origin(xyz=(-0.28, 0.0, 0.78)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.6, lower=0.0, upper=math.radians(70.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    turntable = object_model.get_part("turntable")
    tube_bank = object_model.get_part("tube_bank")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    az_limits = azimuth.motion_limits
    el_limits = elevation.motion_limits
    ctx.check(
        "azimuth is full-circle vertical revolute",
        azimuth.articulation_type == ArticulationType.REVOLUTE
        and tuple(azimuth.axis) == (0.0, 0.0, 1.0)
        and az_limits is not None
        and az_limits.lower is not None
        and az_limits.upper is not None
        and abs((az_limits.upper - az_limits.lower) - 2.0 * math.pi) < 1e-6,
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}, limits={az_limits}",
    )
    ctx.check(
        "elevation is transverse zero-to-seventy-degree revolute",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) == (0.0, -1.0, 0.0)
        and el_limits is not None
        and el_limits.lower == 0.0
        and el_limits.upper is not None
        and abs(el_limits.upper - math.radians(70.0)) < 1e-6,
        details=f"type={elevation.articulation_type}, axis={elevation.axis}, limits={el_limits}",
    )

    ctx.expect_contact(
        turntable,
        base,
        elem_a="rotating_plate",
        elem_b="slewing_ring",
        contact_tol=0.002,
        name="turntable rides on the slewing ring",
    )
    ctx.expect_contact(
        tube_bank,
        turntable,
        elem_a="trunnion_axle",
        elem_b="yoke_plate_0",
        contact_tol=0.002,
        name="trunnion seated in yoke plate 0",
    )
    ctx.expect_contact(
        tube_bank,
        turntable,
        elem_a="trunnion_axle",
        elem_b="yoke_plate_1",
        contact_tol=0.002,
        name="trunnion seated in yoke plate 1",
    )

    rest_front = ctx.part_element_world_aabb(tube_bank, elem="front_clamp")
    rest_z = None if rest_front is None else (rest_front[0][2] + rest_front[1][2]) * 0.5
    rest_x = None if rest_front is None else (rest_front[0][0] + rest_front[1][0]) * 0.5

    with ctx.pose({elevation: math.radians(70.0)}):
        raised_front = ctx.part_element_world_aabb(tube_bank, elem="front_clamp")
        raised_z = None if raised_front is None else (raised_front[0][2] + raised_front[1][2]) * 0.5
    ctx.check(
        "front of tube bank rises at elevation limit",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.80,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    with ctx.pose({azimuth: math.pi / 2.0}):
        slewed_front = ctx.part_element_world_aabb(tube_bank, elem="front_clamp")
        slewed_y = None if slewed_front is None else (slewed_front[0][1] + slewed_front[1][1]) * 0.5
    ctx.check(
        "azimuth slews launcher bank around the vertical axis",
        rest_x is not None and slewed_y is not None and slewed_y > rest_x * 0.75,
        details=f"rest_x={rest_x}, slewed_y={slewed_y}",
    )

    return ctx.report()


object_model = build_object_model()
