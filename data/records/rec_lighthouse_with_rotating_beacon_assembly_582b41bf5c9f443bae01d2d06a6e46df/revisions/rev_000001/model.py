from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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
    model = ArticulatedObject(name="lighthouse_lantern_assembly")

    white_masonry = model.material("white_masonry", rgba=(0.82, 0.80, 0.74, 1.0))
    black_iron = model.material("black_iron", rgba=(0.035, 0.035, 0.035, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.72, 0.52, 0.20, 1.0))
    polished_mirror = model.material("polished_mirror", rgba=(0.90, 0.92, 0.90, 1.0))
    warm_light = model.material("warm_light", rgba=(1.0, 0.78, 0.30, 0.82))
    glass = model.material("slightly_green_glass", rgba=(0.70, 0.92, 0.96, 0.33))
    roof_red = model.material("weathered_red_roof", rgba=(0.48, 0.08, 0.045, 1.0))

    tower_top = model.part("tower_top")

    tower_profile = [
        (0.0, 0.00),
        (0.93, 0.00),
        (0.86, 0.28),
        (0.77, 0.78),
        (0.66, 1.15),
        (0.0, 1.15),
        (0.0, 0.00),
    ]
    tower_top.visual(
        mesh_from_geometry(LatheGeometry(tower_profile, segments=72), "tapered_tower_top"),
        material=white_masonry,
        name="tapered_tower_top",
    )
    tower_top.visual(
        Cylinder(radius=0.99, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        material=black_iron,
        name="gallery_deck",
    )
    tower_top.visual(
        Cylinder(radius=0.76, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 1.27)),
        material=black_iron,
        name="lantern_curb",
    )
    tower_top.visual(
        Cylinder(radius=0.035, length=1.34),
        origin=Origin(xyz=(0.0, 0.0, 1.92)),
        material=dark_metal,
        name="central_shaft",
    )

    # Cylindrical lantern glazing: twelve flat panes seated into continuous
    # circular iron sill and head rings so the enclosure reads as a real hollow
    # glass room rather than a solid transparent cylinder.
    bottom_ring = mesh_from_geometry(
        TorusGeometry(radius=0.700, tube=0.045, radial_segments=18, tubular_segments=96),
        "bottom_lantern_ring",
    )
    top_ring = mesh_from_geometry(
        TorusGeometry(radius=0.700, tube=0.045, radial_segments=18, tubular_segments=96),
        "top_lantern_ring",
    )
    tower_top.visual(
        bottom_ring,
        origin=Origin(xyz=(0.0, 0.0, 1.34)),
        material=black_iron,
        name="bottom_lantern_ring",
    )
    tower_top.visual(
        top_ring,
        origin=Origin(xyz=(0.0, 0.0, 2.40)),
        material=black_iron,
        name="top_lantern_ring",
    )

    pane_count = 12
    pane_radius = 0.694
    pane_width = 2.0 * pane_radius * math.sin(math.pi / pane_count) * 0.78
    for index in range(pane_count):
        angle = 2.0 * math.pi * index / pane_count
        tower_top.visual(
            Box((pane_width, 0.014, 1.04)),
            origin=Origin(
                xyz=(pane_radius * math.cos(angle), pane_radius * math.sin(angle), 1.87),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            material=glass,
            name=f"glass_pane_{index}",
        )
        tower_top.visual(
            Cylinder(radius=0.018, length=1.13),
            origin=Origin(
                xyz=(0.722 * math.cos(angle), 0.722 * math.sin(angle), 1.87),
            ),
            material=black_iron,
            name=f"window_mullion_{index}",
        )

    rail_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.960, tube=0.018, radial_segments=12, tubular_segments=96),
        "gallery_guard_rail",
    )
    tower_top.visual(
        rail_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.44)),
        material=black_iron,
        name="gallery_guard_rail",
    )
    for index in range(16):
        angle = 2.0 * math.pi * index / 16.0
        tower_top.visual(
            Cylinder(radius=0.012, length=0.30),
            origin=Origin(xyz=(0.960 * math.cos(angle), 0.960 * math.sin(angle), 1.30)),
            material=black_iron,
            name=f"gallery_post_{index}",
        )

    roof_profile = [
        (0.0, 2.42),
        (0.80, 2.42),
        (0.73, 2.50),
        (0.35, 2.74),
        (0.14, 2.90),
        (0.0, 2.94),
        (0.0, 2.42),
    ]
    tower_top.visual(
        mesh_from_geometry(LatheGeometry(roof_profile, segments=72), "conical_roof"),
        material=roof_red,
        name="conical_roof",
    )
    tower_top.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.95)),
        material=black_iron,
        name="roof_vent_base",
    )
    tower_top.visual(
        Cylinder(radius=0.09, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 3.09)),
        material=black_iron,
        name="roof_vent_cap",
    )

    beacon_carriage = model.part("beacon_carriage")
    beacon_carriage.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.105, tube=0.024, radial_segments=16, tubular_segments=64),
            "rotating_bearing_collar",
        ),
        material=dark_metal,
        name="rotating_bearing_collar",
    )
    bushing_geom = LatheGeometry.from_shell_profiles(
        [(0.060, -0.050), (0.060, 0.050)],
        [(0.034, -0.050), (0.034, 0.050)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    beacon_carriage.visual(
        mesh_from_geometry(bushing_geom, "shaft_bushing"),
        material=dark_metal,
        name="shaft_bushing",
    )
    beacon_carriage.visual(
        Box((0.50, 0.045, 0.045)),
        origin=Origin(xyz=(0.305, 0.0, 0.0)),
        material=dark_metal,
        name="front_frame_arm",
    )
    beacon_carriage.visual(
        Box((0.50, 0.045, 0.045)),
        origin=Origin(xyz=(-0.305, 0.0, 0.0)),
        material=dark_metal,
        name="rear_frame_arm",
    )
    beacon_carriage.visual(
        Cylinder(radius=0.020, length=0.52),
        origin=Origin(xyz=(0.52, 0.0, 0.0)),
        material=dark_metal,
        name="front_yoke_post",
    )
    beacon_carriage.visual(
        Cylinder(radius=0.020, length=0.52),
        origin=Origin(xyz=(-0.52, 0.0, 0.0)),
        material=dark_metal,
        name="rear_yoke_post",
    )
    beacon_carriage.visual(
        Cylinder(radius=0.105, length=0.28),
        origin=Origin(xyz=(0.335, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="lamp_housing",
    )
    beacon_carriage.visual(
        Sphere(radius=0.070),
        origin=Origin(xyz=(0.50, 0.0, 0.0)),
        material=warm_light,
        name="glowing_lamp",
    )
    beacon_carriage.visual(
        Box((0.060, 0.050, 0.050)),
        origin=Origin(xyz=(-0.080, 0.0, 0.0)),
        material=dark_metal,
        name="reflector_stem",
    )

    reflector_outer = [
        (0.035, -0.070),
        (0.075, -0.090),
        (0.145, -0.150),
        (0.230, -0.255),
    ]
    reflector_inner = [
        (0.026, -0.076),
        (0.067, -0.097),
        (0.134, -0.155),
        (0.214, -0.244),
    ]
    reflector_geom = LatheGeometry.from_shell_profiles(
        reflector_outer,
        reflector_inner,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)
    beacon_carriage.visual(
        mesh_from_geometry(reflector_geom, "parabolic_reflector"),
        material=polished_mirror,
        name="parabolic_reflector",
    )
    beacon_carriage.visual(
        Box((0.090, 0.030, 0.030)),
        origin=Origin(xyz=(0.155, 0.0, 0.100)),
        material=dark_metal,
        name="upper_lamp_bracket",
    )
    beacon_carriage.visual(
        Box((0.090, 0.030, 0.030)),
        origin=Origin(xyz=(0.155, 0.0, -0.100)),
        material=dark_metal,
        name="lower_lamp_bracket",
    )

    model.articulation(
        "beacon_rotation",
        ArticulationType.CONTINUOUS,
        parent=tower_top,
        child=beacon_carriage,
        origin=Origin(xyz=(0.0, 0.0, 1.87)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower_top")
    carriage = object_model.get_part("beacon_carriage")
    rotation = object_model.get_articulation("beacon_rotation")

    ctx.allow_overlap(
        tower,
        carriage,
        elem_a="central_shaft",
        elem_b="shaft_bushing",
        reason=(
            "The rotating carriage is captured on the fixed vertical shaft; "
            "the thin bushing sleeve is given a tiny modeled interference so "
            "the support path is explicit at mesh tolerance."
        ),
    )

    ctx.check(
        "beacon carriage has continuous rotation",
        rotation.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotation.articulation_type}",
    )
    ctx.check(
        "beacon rotation axis is vertical",
        tuple(round(v, 6) for v in rotation.axis) == (0.0, 0.0, 1.0),
        details=f"axis={rotation.axis}",
    )
    ctx.expect_origin_distance(
        carriage,
        tower,
        axes="xy",
        max_dist=0.001,
        name="beacon rotates on central shaft axis",
    )
    ctx.expect_within(
        carriage,
        tower,
        axes="xy",
        margin=0.0,
        name="beacon carriage fits inside lantern footprint",
    )
    ctx.expect_within(
        tower,
        carriage,
        axes="xy",
        inner_elem="central_shaft",
        outer_elem="shaft_bushing",
        margin=0.0,
        name="shaft is centered inside carriage bushing",
    )
    ctx.expect_overlap(
        tower,
        carriage,
        axes="z",
        elem_a="central_shaft",
        elem_b="shaft_bushing",
        min_overlap=0.08,
        name="bushing has retained vertical bearing engagement",
    )

    rest_aabb = ctx.part_world_aabb(carriage)
    with ctx.pose({rotation: math.pi / 2.0}):
        ctx.expect_origin_distance(
            carriage,
            tower,
            axes="xy",
            max_dist=0.001,
            name="rotated carriage stays centered on shaft",
        )
        turned_aabb = ctx.part_world_aabb(carriage)

    ctx.check(
        "quarter turn changes beacon heading",
        rest_aabb is not None
        and turned_aabb is not None
        and abs((rest_aabb[1][0] - rest_aabb[0][0]) - (turned_aabb[1][0] - turned_aabb[0][0])) > 0.04,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
