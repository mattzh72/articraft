from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_church_bell_tower")

    warm_wood = model.material("warm_oak", rgba=(0.55, 0.32, 0.15, 1.0))
    dark_wood = model.material("weathered_dark_wood", rgba=(0.24, 0.13, 0.07, 1.0))
    shingle = model.material("dark_wood_shingles", rgba=(0.16, 0.11, 0.08, 1.0))
    brass = model.material("aged_bell_bronze", rgba=(0.82, 0.56, 0.21, 1.0))
    iron = model.material("blackened_iron", rgba=(0.04, 0.04, 0.04, 1.0))

    tower = model.part("tower")

    # Square wooden lower stage.
    tower.visual(
        Box((1.65, 1.65, 0.15)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_wood,
        name="base_plinth",
    )
    tower.visual(
        Box((1.20, 1.20, 1.72)),
        origin=Origin(xyz=(0.0, 0.0, 1.005)),
        material=warm_wood,
        name="lower_body",
    )

    # Applied battens and trim make the closed stage read as vertical planking.
    for i, x in enumerate((-0.45, -0.225, 0.0, 0.225, 0.45)):
        tower.visual(
            Box((0.024, 0.035, 1.64)),
            origin=Origin(xyz=(x, -0.610, 1.00)),
            material=dark_wood,
            name=f"front_batten_{i}",
        )
        tower.visual(
            Box((0.024, 0.035, 1.64)),
            origin=Origin(xyz=(x, 0.610, 1.00)),
            material=dark_wood,
            name=f"rear_batten_{i}",
        )
    for i, y in enumerate((-0.45, -0.225, 0.0, 0.225, 0.45)):
        tower.visual(
            Box((0.035, 0.024, 1.64)),
            origin=Origin(xyz=(-0.610, y, 1.00)),
            material=dark_wood,
            name=f"side_batten_{i}",
        )
        tower.visual(
            Box((0.035, 0.024, 1.64)),
            origin=Origin(xyz=(0.610, y, 1.00)),
            material=dark_wood,
            name=f"other_side_batten_{i}",
        )
    for z, name in ((0.21, "lower_trim"), (1.80, "upper_trim")):
        tower.visual(Box((1.30, 0.06, 0.07)), origin=Origin(xyz=(0.0, -0.620, z)), material=dark_wood, name=f"front_{name}")
        tower.visual(Box((1.30, 0.06, 0.07)), origin=Origin(xyz=(0.0, 0.620, z)), material=dark_wood, name=f"rear_{name}")
        tower.visual(Box((0.06, 1.30, 0.07)), origin=Origin(xyz=(-0.620, 0.0, z)), material=dark_wood, name=f"side_{name}")
        tower.visual(Box((0.06, 1.30, 0.07)), origin=Origin(xyz=(0.620, 0.0, z)), material=dark_wood, name=f"other_side_{name}")

    # Open belfry: four posts and perimeter beams leave a clear front/rear opening.
    post_xy = (-0.66, 0.66)
    for ix, x in enumerate(post_xy):
        for iy, y in enumerate(post_xy):
            tower.visual(
                Box((0.16, 0.16, 1.15)),
                origin=Origin(xyz=(x, y, 2.475)),
                material=warm_wood,
                name=f"belfry_post_{ix}_{iy}",
            )

    for z, h, label in ((1.925, 0.17, "lower"), (3.075, 0.17, "upper")):
        tower.visual(Box((1.48, 0.16, h)), origin=Origin(xyz=(0.0, -0.66, z)), material=dark_wood, name=f"front_{label}_beam")
        tower.visual(Box((1.48, 0.16, h)), origin=Origin(xyz=(0.0, 0.66, z)), material=dark_wood, name=f"rear_{label}_beam")
        tower.visual(Box((0.16, 1.48, h)), origin=Origin(xyz=(-0.66, 0.0, z)), material=dark_wood, name=f"side_{label}_beam")
        tower.visual(Box((0.16, 1.48, h)), origin=Origin(xyz=(0.66, 0.0, z)), material=dark_wood, name=f"other_side_{label}_beam")

    # Split side bearing timbers leave a real gap for the pivot bar.
    for side, x in (("left", -0.66), ("right", 0.66)):
        tower.visual(Box((0.16, 1.26, 0.07)), origin=Origin(xyz=(x, 0.0, 2.545)), material=dark_wood, name=f"{side}_lower_bearing_timber")
        tower.visual(Box((0.16, 1.26, 0.07)), origin=Origin(xyz=(x, 0.0, 2.775)), material=dark_wood, name=f"{side}_upper_bearing_timber")

    bearing_mesh = mesh_from_geometry(TorusGeometry(radius=0.070, tube=0.016, radial_segments=32, tubular_segments=16), "iron_bearing_ring")
    for i, x in enumerate((-0.66, 0.66)):
        tower.visual(
            bearing_mesh,
            origin=Origin(xyz=(x, 0.0, 2.66), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=iron,
            name=f"bearing_ring_{i}",
        )

    spire_mesh = mesh_from_geometry(ConeGeometry(radius=1.05, height=1.45, radial_segments=4, closed=True), "square_tapered_spire")
    tower.visual(
        spire_mesh,
        origin=Origin(xyz=(0.0, 0.0, 3.885), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=shingle,
        name="tapered_spire",
    )
    tower.visual(Box((0.075, 0.075, 0.34)), origin=Origin(xyz=(0.0, 0.0, 4.76)), material=iron, name="cross_mast")
    tower.visual(Box((0.32, 0.055, 0.055)), origin=Origin(xyz=(0.0, 0.0, 4.83)), material=iron, name="cross_arm")

    bell = model.part("bell")
    bell.visual(
        Cylinder(radius=0.056, length=1.48),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="pivot_bar",
    )
    bell.visual(Box((0.48, 0.16, 0.12)), origin=Origin(xyz=(0.0, 0.0, -0.085)), material=dark_wood, name="wooden_yoke")
    bell.visual(Box((0.040, 0.090, 0.22)), origin=Origin(xyz=(-0.095, 0.0, -0.205)), material=iron, name="yoke_strap_0")
    bell.visual(Box((0.040, 0.090, 0.22)), origin=Origin(xyz=(0.095, 0.0, -0.205)), material=iron, name="yoke_strap_1")
    bell.visual(Cylinder(radius=0.095, length=0.08), origin=Origin(xyz=(0.0, 0.0, -0.205)), material=brass, name="bell_crown")

    bell_profile = [
        (0.095, -0.205),
        (0.130, -0.270),
        (0.160, -0.420),
        (0.220, -0.610),
        (0.280, -0.700),
        (0.235, -0.660),
        (0.175, -0.440),
        (0.112, -0.285),
        (0.070, -0.215),
    ]
    bell.visual(
        mesh_from_geometry(LatheGeometry(bell_profile, segments=72, closed=True), "hollow_bell_shell"),
        material=brass,
        name="bell_shell",
    )
    bell.visual(
        Cylinder(radius=0.014, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="clapper_pin",
    )
    bell.visual(Cylinder(radius=0.008, length=0.40), origin=Origin(xyz=(0.0, 0.0, -0.430)), material=iron, name="clapper_rod")
    bell.visual(Sphere(radius=0.050), origin=Origin(xyz=(0.0, 0.0, -0.650)), material=iron, name="clapper_ball")

    model.articulation(
        "tower_to_bell",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 2.66)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-0.45, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell")
    hinge = object_model.get_articulation("tower_to_bell")

    ctx.allow_overlap(
        bell,
        tower,
        elem_a="pivot_bar",
        elem_b="bearing_ring_0",
        reason="The iron pivot bar is intentionally captured inside the side bearing ring.",
    )
    ctx.allow_overlap(
        bell,
        tower,
        elem_a="pivot_bar",
        elem_b="bearing_ring_1",
        reason="The iron pivot bar is intentionally captured inside the opposite side bearing ring.",
    )
    ctx.expect_overlap(
        bell,
        tower,
        axes="yz",
        elem_a="pivot_bar",
        elem_b="bearing_ring_0",
        min_overlap=0.050,
        name="pivot bar passes through one bearing",
    )
    ctx.expect_overlap(
        bell,
        tower,
        axes="yz",
        elem_a="pivot_bar",
        elem_b="bearing_ring_1",
        min_overlap=0.050,
        name="pivot bar passes through other bearing",
    )
    ctx.expect_gap(
        bell,
        tower,
        axis="z",
        min_gap=0.030,
        positive_elem="bell_shell",
        negative_elem="lower_body",
        name="bell hangs clear of lower tower",
    )

    rest_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    with ctx.pose({hinge: 0.35}):
        swung_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
    rest_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
    swung_y = None if swung_aabb is None else (swung_aabb[0][1] + swung_aabb[1][1]) / 2.0
    ctx.check(
        "bell swings on revolute pivot",
        rest_y is not None and swung_y is not None and swung_y > rest_y + 0.05,
        details=f"rest_y={rest_y}, swung_y={swung_y}",
    )

    return ctx.report()


object_model = build_object_model()
