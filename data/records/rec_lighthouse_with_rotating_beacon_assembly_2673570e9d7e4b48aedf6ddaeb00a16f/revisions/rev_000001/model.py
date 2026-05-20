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
    model = ArticulatedObject(name="squat_harbor_beacon")

    whitewash = Material("weathered_whitewash", rgba=(0.82, 0.79, 0.68, 1.0))
    stone = Material("damp_granite", rgba=(0.38, 0.39, 0.37, 1.0))
    roof_red = Material("oxide_red", rgba=(0.55, 0.08, 0.05, 1.0))
    dark_metal = Material("blackened_bronze", rgba=(0.06, 0.055, 0.05, 1.0))
    green = Material("weathered_green_door", rgba=(0.05, 0.28, 0.20, 1.0))
    glass = Material("sea_glass_blue", rgba=(0.45, 0.80, 0.95, 0.36))
    amber = Material("warm_lamp_glass", rgba=(1.0, 0.74, 0.20, 1.0))

    tower = model.part("tower")

    # A single lathed mass gives the squat tower a believable battered masonry
    # silhouette instead of a stack of arbitrary cylinders.
    tower_profile = [
        (0.0, 0.0),
        (0.64, 0.0),
        (0.64, 0.12),
        (0.56, 0.22),
        (0.52, 0.95),
        (0.48, 1.55),
        (0.42, 1.68),
        (0.38, 1.78),
        (0.0, 1.78),
    ]
    tower.visual(
        mesh_from_geometry(LatheGeometry(tower_profile, segments=72), "tapered_tower"),
        material=whitewash,
        name="tapered_tower",
    )

    # Stone plinth and dark roof trim are separate raised courses, deliberately
    # intersecting the same tower part so the visible masonry reads connected.
    for i, (radius, z, tube) in enumerate(
        [
            (0.61, 0.13, 0.020),
            (0.54, 0.96, 0.012),
            (0.45, 1.67, 0.014),
        ]
    ):
        tower.visual(
            mesh_from_geometry(TorusGeometry(radius, tube, radial_segments=72, tubular_segments=10), f"stone_band_{i}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=stone,
            name=f"stone_band_{i}",
        )

    tower.visual(
        Cylinder(radius=0.40, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 1.802)),
        material=roof_red,
        name="flat_roof",
    )
    tower.visual(
        Cylinder(radius=0.24, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 1.8995)),
        material=dark_metal,
        name="circular_pedestal",
    )

    # Static recess and frame for the side equipment door on the upper wall.
    tower.visual(
        Box((0.020, 0.36, 0.70)),
        origin=Origin(xyz=(0.516, 0.0, 1.28)),
        material=dark_metal,
        name="door_recess",
    )
    tower.visual(
        Box((0.030, 0.050, 0.74)),
        origin=Origin(xyz=(0.522, -0.205, 1.28)),
        material=stone,
        name="door_hinge_jamb",
    )
    tower.visual(
        Box((0.030, 0.050, 0.74)),
        origin=Origin(xyz=(0.522, 0.205, 1.28)),
        material=stone,
        name="door_latch_jamb",
    )
    tower.visual(
        Box((0.030, 0.41, 0.050)),
        origin=Origin(xyz=(0.522, 0.0, 1.655)),
        material=stone,
        name="door_lintel",
    )
    tower.visual(
        Box((0.030, 0.41, 0.045)),
        origin=Origin(xyz=(0.522, 0.0, 0.905)),
        material=stone,
        name="door_sill",
    )
    tower.visual(
        Box((0.028, 0.036, 0.660)),
        origin=Origin(xyz=(0.514, -0.1325, 1.28)),
        material=dark_metal,
        name="fixed_hinge_leaf",
    )

    door = model.part("equipment_door")
    door.visual(
        Box((0.032, 0.265, 0.620)),
        # The child frame is the vertical hinge axis; the slab extends along +Y.
        origin=Origin(xyz=(0.027, 0.1325, 0.0)),
        material=green,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.660),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.036),
        origin=Origin(xyz=(0.050, 0.205, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="round_handle",
    )
    for j, z in enumerate((-0.165, -0.105, -0.045)):
        door.visual(
            Box((0.006, 0.160, 0.014)),
            origin=Origin(xyz=(0.0455, 0.138, z)),
            material=dark_metal,
            name=f"door_vent_{j}",
        )

    model.articulation(
        "tower_to_door",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=door,
        origin=Origin(xyz=(0.540, -0.1325, 1.28)),
        # Positive motion swings the panel outward from the wall.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    lantern = model.part("lantern")
    lantern.visual(
        Cylinder(radius=0.265, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_metal,
        name="turntable_disc",
    )
    lantern.visual(
        Cylinder(radius=0.185, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=glass,
        name="glass_drum",
    )
    lantern.visual(
        Sphere(radius=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=amber,
        name="lamp_bulb",
    )
    lantern.visual(
        Cylinder(radius=0.058, length=0.390),
        origin=Origin(xyz=(0.030, 0.0, 0.180), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="lamp_barrel",
    )
    lantern.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.225, 0.0, 0.180), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=amber,
        name="front_lens",
    )
    lantern.visual(
        Cylinder(radius=0.022, length=0.135),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=dark_metal,
        name="central_stem",
    )
    for i, (x, y) in enumerate(((0.145, 0.145), (-0.145, 0.145), (-0.145, -0.145), (0.145, -0.145))):
        lantern.visual(
            Cylinder(radius=0.010, length=0.275),
            origin=Origin(xyz=(x, y, 0.182)),
            material=dark_metal,
            name=f"cage_post_{i}",
        )
    lantern.visual(
        mesh_from_geometry(TorusGeometry(0.185, 0.010, radial_segments=64, tubular_segments=8), "lower_lantern_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=dark_metal,
        name="lower_lantern_ring",
    )
    lantern.visual(
        mesh_from_geometry(TorusGeometry(0.185, 0.010, radial_segments=64, tubular_segments=8), "upper_lantern_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=dark_metal,
        name="upper_lantern_ring",
    )
    lantern.visual(
        mesh_from_geometry(ConeGeometry(0.240, 0.120, radial_segments=72), "lantern_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=roof_red,
        name="lantern_cap",
    )

    model.articulation(
        "pedestal_to_lantern",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=lantern,
        origin=Origin(xyz=(0.0, 0.0, 1.9745)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    door = object_model.get_part("equipment_door")
    lantern = object_model.get_part("lantern")
    door_joint = object_model.get_articulation("tower_to_door")
    lantern_joint = object_model.get_articulation("pedestal_to_lantern")

    ctx.check(
        "lamp housing uses continuous rotation",
        lantern_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={lantern_joint.articulation_type}",
    )
    ctx.check(
        "equipment door uses vertical hinge",
        door_joint.articulation_type == ArticulationType.REVOLUTE and tuple(door_joint.axis) == (0.0, 0.0, -1.0),
        details=f"type={door_joint.articulation_type}, axis={door_joint.axis}",
    )

    ctx.expect_contact(
        lantern,
        tower,
        elem_a="turntable_disc",
        elem_b="circular_pedestal",
        contact_tol=0.001,
        name="turntable sits on the roof pedestal",
    )
    ctx.expect_overlap(
        door,
        tower,
        axes="yz",
        elem_a="door_panel",
        elem_b="door_recess",
        min_overlap=0.20,
        name="closed equipment door covers the wall recess",
    )
    ctx.expect_gap(
        door,
        tower,
        axis="x",
        positive_elem="door_panel",
        negative_elem="door_recess",
        min_gap=0.010,
        max_gap=0.050,
        name="door panel is proud of the recessed wall pocket",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 1.15}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door swings outward from tower wall",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.080,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({lantern_joint: 0.0}):
        lens_start = ctx.part_element_world_aabb(lantern, elem="front_lens")
    with ctx.pose({lantern_joint: math.pi / 2.0}):
        lens_quarter = ctx.part_element_world_aabb(lantern, elem="front_lens")
    ctx.check(
        "rotating lantern carries the front lens around the pedestal axis",
        lens_start is not None
        and lens_quarter is not None
        and ((lens_start[0][0] + lens_start[1][0]) / 2.0) > 0.15
        and ((lens_quarter[0][1] + lens_quarter[1][1]) / 2.0) > 0.15,
        details=f"start={lens_start}, quarter_turn={lens_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
