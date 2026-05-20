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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A simple vertical ring, authored from z=0 to z=height."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _armored_box(size: tuple[float, float, float], chamfer: float) -> cq.Workplane:
    """Centered chamfered box for welded/armored housings."""
    return cq.Workplane("XY").box(*size).edges("|Z").chamfer(chamfer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_remote_station")

    roof_mat = model.material("painted_roof", rgba=(0.18, 0.20, 0.19, 1.0))
    armor_mat = model.material("olive_armor", rgba=(0.17, 0.22, 0.16, 1.0))
    dark_mat = model.material("dark_weapon", rgba=(0.08, 0.085, 0.08, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.01, 0.012, 0.011, 1.0))
    steel_mat = model.material("dark_blued_steel", rgba=(0.035, 0.038, 0.040, 1.0))
    bolt_mat = model.material("blackened_bolts", rgba=(0.025, 0.025, 0.023, 1.0))

    roof = model.part("roof")
    roof.visual(
        Box((1.20, 0.80, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=roof_mat,
        name="roof_plate",
    )
    roof.visual(
        mesh_from_cadquery(_annular_cylinder(0.300, 0.200, 0.025), "fixed_yaw_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_mat,
        name="fixed_yaw_ring",
    )
    # Slightly sunken bolt heads make the fixed ring read as bolted to the roof.
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        roof.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(
                xyz=(0.293 * math.cos(a), 0.293 * math.sin(a), 0.087),
            ),
            material=bolt_mat,
            name=f"yaw_bolt_{i:02d}",
        )

    turret = model.part("turret")
    turret.visual(
        mesh_from_cadquery(_annular_cylinder(0.285, 0.205, 0.035), "rotating_yaw_ring"),
        material=dark_mat,
        name="rotating_ring",
    )
    turret.visual(
        mesh_from_cadquery(_armored_box((0.46, 0.36, 0.19), 0.035), "turret_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=armor_mat,
        name="armored_shell",
    )
    turret.visual(
        Box((0.24, 0.050, 0.252)),
        origin=Origin(xyz=(0.040, 0.160, 0.348)),
        material=armor_mat,
        name="cheek_0",
    )
    turret.visual(
        Box((0.24, 0.050, 0.252)),
        origin=Origin(xyz=(0.040, -0.160, 0.348)),
        material=armor_mat,
        name="cheek_1",
    )
    turret.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.040, 0.132, 0.370), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="pitch_bearing_0",
    )
    turret.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.040, -0.132, 0.370), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="pitch_bearing_1",
    )

    cradle = model.part("weapon_cradle")
    cradle.visual(
        mesh_from_cadquery(_armored_box((0.50, 0.22, 0.18), 0.020), "weapon_housing"),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=armor_mat,
        name="weapon_housing",
    )
    cradle.visual(
        Cylinder(radius=0.032, length=0.252),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="pitch_shaft",
    )
    cradle.visual(
        Cylinder(radius=0.035, length=0.470),
        origin=Origin(xyz=(0.625, 0.0, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="barrel",
    )
    cradle.visual(
        Cylinder(radius=0.047, length=0.080),
        origin=Origin(xyz=(0.900, 0.0, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="muzzle_brake",
    )
    for y in (-0.105, 0.105):
        suffix = "0" if y < 0.0 else "1"
        cradle.visual(
            Box((0.040, 0.028, 0.010)),
            origin=Origin(xyz=(-0.085, y, 0.093)),
            material=dark_mat,
            name=f"lid_hinge_foot_{suffix}",
        )
        cradle.visual(
            Cylinder(radius=0.011, length=0.028),
            origin=Origin(xyz=(-0.085, y, 0.108), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel_mat,
            name=f"lid_hinge_lug_{suffix}",
        )

    service_lid = model.part("service_lid")
    service_lid.visual(
        Box((0.410, 0.170, 0.014)),
        origin=Origin(xyz=(0.205, 0.0, -0.007)),
        material=armor_mat,
        name="lid_panel",
    )
    service_lid.visual(
        Cylinder(radius=0.011, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="lid_hinge_barrel",
    )
    service_lid.visual(
        Box((0.360, 0.130, 0.004)),
        origin=Origin(xyz=(0.230, 0.0, -0.016)),
        material=rubber_mat,
        name="lid_gasket",
    )

    model.articulation(
        "roof_to_turret",
        ArticulationType.CONTINUOUS,
        parent=roof,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2),
    )
    model.articulation(
        "turret_to_cradle",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=cradle,
        origin=Origin(xyz=(0.040, 0.0, 0.370)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.8, lower=-0.35, upper=0.55),
    )
    model.articulation(
        "cradle_to_lid",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=service_lid,
        origin=Origin(xyz=(-0.085, 0.0, 0.108)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof")
    turret = object_model.get_part("turret")
    cradle = object_model.get_part("weapon_cradle")
    lid = object_model.get_part("service_lid")
    yaw = object_model.get_articulation("roof_to_turret")
    pitch = object_model.get_articulation("turret_to_cradle")
    lid_hinge = object_model.get_articulation("cradle_to_lid")

    ctx.expect_gap(
        turret,
        roof,
        axis="z",
        positive_elem="rotating_ring",
        negative_elem="fixed_yaw_ring",
        min_gap=0.0,
        max_gap=0.001,
        name="rotating yaw ring sits on fixed roof ring",
    )
    ctx.expect_overlap(
        turret,
        roof,
        axes="xy",
        elem_a="rotating_ring",
        elem_b="fixed_yaw_ring",
        min_overlap=0.35,
        name="yaw rings share a broad circular footprint",
    )
    ctx.expect_gap(
        lid,
        cradle,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="weapon_housing",
        min_gap=0.002,
        max_gap=0.006,
        name="closed service lid clears the top gasket plane",
    )
    ctx.expect_overlap(
        lid,
        cradle,
        axes="xy",
        elem_a="lid_panel",
        elem_b="weapon_housing",
        min_overlap=0.15,
        name="service lid spans weapon housing top",
    )

    def elem_z_center(part, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return 0.5 * (lo[2] + hi[2])

    rest_muzzle_z = elem_z_center(cradle, "muzzle_brake")
    with ctx.pose({pitch: 0.55}):
        raised_muzzle_z = elem_z_center(cradle, "muzzle_brake")
    ctx.check(
        "positive pitch elevates the muzzle",
        rest_muzzle_z is not None
        and raised_muzzle_z is not None
        and raised_muzzle_z > rest_muzzle_z + 0.20,
        details=f"rest_z={rest_muzzle_z}, raised_z={raised_muzzle_z}",
    )

    rest_lid_z = elem_z_center(lid, "lid_panel")
    with ctx.pose({lid_hinge: 1.25}):
        open_lid_z = elem_z_center(lid, "lid_panel")
    ctx.check(
        "service lid opens upward about its top hinge",
        rest_lid_z is not None and open_lid_z is not None and open_lid_z > rest_lid_z + 0.10,
        details=f"closed_z={rest_lid_z}, open_z={open_lid_z}",
    )

    rest_cradle_pos = ctx.part_world_position(cradle)
    with ctx.pose({yaw: 0.8}):
        yawed_cradle_pos = ctx.part_world_position(cradle)
    ctx.check(
        "yaw joint swings the weapon station around the vertical axis",
        rest_cradle_pos is not None
        and yawed_cradle_pos is not None
        and abs(yawed_cradle_pos[1] - rest_cradle_pos[1]) > 0.02,
        details=f"rest={rest_cradle_pos}, yawed={yawed_cradle_pos}",
    )

    return ctx.report()


object_model = build_object_model()
