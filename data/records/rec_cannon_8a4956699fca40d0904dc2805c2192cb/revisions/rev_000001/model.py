from __future__ import annotations

import math

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
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coastal_barbette_embrasure_cannon")

    concrete = model.material("weathered_concrete", rgba=(0.58, 0.59, 0.57, 1.0))
    darker_concrete = model.material("dark_concrete", rgba=(0.44, 0.45, 0.43, 1.0))
    gunmetal = model.material("oiled_gunmetal", rgba=(0.10, 0.11, 0.12, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.30, 0.31, 0.32, 1.0))
    dark_bore = model.material("black_bore", rgba=(0.005, 0.004, 0.003, 1.0))

    parapet_mesh = _mesh(
        "round_concrete_parapet",
        LatheGeometry.from_shell_profiles(
            [(3.05, 0.88), (3.05, 1.34)],
            [(2.78, 1.34), (2.78, 0.88)],
            segments=96,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    outer_bearing_mesh = _mesh(
        "fixed_bearing_race",
        TorusGeometry(radius=1.18, tube=0.045, radial_segments=18, tubular_segments=80),
    )
    rotating_race_mesh = _mesh(
        "rotating_bearing_race",
        TorusGeometry(radius=1.10, tube=0.040, radial_segments=18, tubular_segments=80),
    )
    barrel_mesh = _mesh(
        "smoothbore_barrel_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.38, -0.66),
                (0.39, -0.43),
                (0.31, -0.22),
                (0.275, 0.10),
                (0.235, 0.85),
                (0.200, 2.50),
                (0.185, 3.28),
                (0.232, 3.44),
            ],
            [
                (0.115, -0.50),
                (0.112, 0.20),
                (0.108, 1.40),
                (0.104, 2.80),
                (0.100, 3.50),
            ],
            segments=88,
            start_cap="flat",
            end_cap="round",
            lip_samples=8,
        ),
    )

    barbette = model.part("concrete_barbette")
    barbette.visual(
        Box((6.40, 5.60, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=darker_concrete,
        name="foundation_slab",
    )
    barbette.visual(
        Cylinder(radius=2.70, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.57)),
        material=concrete,
        name="round_barbette_body",
    )
    barbette.visual(
        Cylinder(radius=3.05, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=concrete,
        name="parapet_footing",
    )
    barbette.visual(
        Cylinder(radius=1.62, length=0.19),
        origin=Origin(xyz=(0.0, 0.0, 0.995)),
        material=concrete,
        name="bearing_plinth",
    )
    barbette.visual(
        Cylinder(radius=1.22, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.13)),
        material=worn_steel,
        name="bearing_cap",
    )
    barbette.visual(
        outer_bearing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.19)),
        material=gunmetal,
        name="fixed_bearing_race",
    )
    barbette.visual(
        parapet_mesh,
        material=concrete,
        name="parapet_wall",
    )

    platform = model.part("gun_platform")
    platform.visual(
        Cylinder(radius=0.55, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=gunmetal,
        name="central_spindle",
    )
    platform.visual(
        rotating_race_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=gunmetal,
        name="rotating_bearing_race",
    )
    platform.visual(
        Cylinder(radius=1.28, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=worn_steel,
        name="turntable_drum",
    )
    platform.visual(
        Box((5.00, 2.60, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=worn_steel,
        name="rectangular_deck",
    )
    platform.visual(
        Box((5.05, 0.08, 0.16)),
        origin=Origin(xyz=(0.0, 1.34, 0.44)),
        material=gunmetal,
        name="side_curb_0",
    )
    platform.visual(
        Box((5.05, 0.08, 0.16)),
        origin=Origin(xyz=(0.0, -1.34, 0.44)),
        material=gunmetal,
        name="side_curb_1",
    )
    platform.visual(
        Box((0.08, 2.60, 0.16)),
        origin=Origin(xyz=(-2.54, 0.0, 0.44)),
        material=gunmetal,
        name="rear_curb",
    )
    platform.visual(
        Box((0.16, 0.22, 2.50)),
        origin=Origin(xyz=(0.95, 0.70, 1.61)),
        material=worn_steel,
        name="embrasure_cheek_0",
    )
    platform.visual(
        Box((0.16, 0.22, 2.50)),
        origin=Origin(xyz=(0.95, -0.70, 1.61)),
        material=worn_steel,
        name="embrasure_cheek_1",
    )
    platform.visual(
        Box((0.16, 1.62, 0.18)),
        origin=Origin(xyz=(0.95, 0.0, 2.95)),
        material=worn_steel,
        name="embrasure_lintel",
    )
    platform.visual(
        Box((0.16, 1.62, 0.18)),
        origin=Origin(xyz=(0.95, 0.0, 1.02)),
        material=worn_steel,
        name="embrasure_sill",
    )
    platform.visual(
        Box((1.62, 1.48, 0.34)),
        origin=Origin(xyz=(-0.18, 0.0, 0.53)),
        material=gunmetal,
        name="carriage_block",
    )
    platform.visual(
        Box((2.20, 1.72, 0.26)),
        origin=Origin(xyz=(-0.34, 0.0, 0.83)),
        material=worn_steel,
        name="trunnion_bed",
    )
    platform.visual(
        Box((0.44, 0.24, 0.88)),
        origin=Origin(xyz=(0.05, 0.74, 1.40)),
        material=gunmetal,
        name="trunnion_cheek_0",
    )
    platform.visual(
        Box((0.44, 0.24, 0.88)),
        origin=Origin(xyz=(0.05, -0.74, 1.40)),
        material=gunmetal,
        name="trunnion_cheek_1",
    )

    barrel = model.part("barrel")
    barrel.visual(
        barrel_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="smoothbore_barrel",
    )
    barrel.visual(
        Cylinder(radius=0.150, length=1.24),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_shaft",
    )
    barrel.visual(
        Cylinder(radius=0.255, length=0.20),
        origin=Origin(xyz=(3.46, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="muzzle_swell",
    )
    barrel.visual(
        Cylinder(radius=0.098, length=0.08),
        origin=Origin(xyz=(3.52, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_bore,
        name="muzzle_bore",
    )
    barrel.visual(
        Cylinder(radius=0.24, length=0.32),
        origin=Origin(xyz=(-0.60, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="breech_knob",
    )

    model.articulation(
        "traverse_bearing",
        ArticulationType.CONTINUOUS,
        parent=barbette,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120000.0, velocity=0.18),
    )
    model.articulation(
        "elevation_trunnion",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=barrel,
        origin=Origin(xyz=(0.05, 0.0, 1.58)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90000.0, velocity=0.35, lower=-0.08, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barbette = object_model.get_part("concrete_barbette")
    platform = object_model.get_part("gun_platform")
    barrel = object_model.get_part("barrel")
    traverse = object_model.get_articulation("traverse_bearing")
    elevation = object_model.get_articulation("elevation_trunnion")

    ctx.check(
        "platform uses continuous horizontal traverse bearing",
        traverse.articulation_type == ArticulationType.CONTINUOUS and tuple(traverse.axis) == (0.0, 0.0, 1.0),
        details=f"type={traverse.articulation_type}, axis={traverse.axis}",
    )
    ctx.check(
        "barrel uses bounded elevation trunnion",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) == (0.0, -1.0, 0.0)
        and elevation.motion_limits.lower < 0.0
        and elevation.motion_limits.upper > 0.5,
        details=f"type={elevation.articulation_type}, axis={elevation.axis}, limits={elevation.motion_limits}",
    )

    ctx.expect_contact(
        platform,
        barbette,
        elem_a="rotating_bearing_race",
        elem_b="fixed_bearing_race",
        contact_tol=0.012,
        name="rotating race is seated on fixed race",
    )
    ctx.expect_gap(
        platform,
        barbette,
        axis="z",
        positive_elem="central_spindle",
        negative_elem="bearing_cap",
        min_gap=0.0,
        max_gap=0.001,
        name="central spindle seats on bearing cap",
    )
    ctx.expect_overlap(
        barrel,
        platform,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="trunnion_bed",
        min_overlap=0.25,
        name="trunnion shaft spans the carriage bed",
    )

    rest_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_bore")
    with ctx.pose({elevation: elevation.motion_limits.upper}):
        raised_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_bore")
    ctx.check(
        "positive elevation raises the muzzle",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[0][2] > rest_aabb[0][2] + 1.2,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
