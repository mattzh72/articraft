from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


GLOBE_RADIUS = 0.32
MERIDIAN_RADIUS = 0.39
GLOBE_CENTER_Z = 0.92
AXIS_TILT = math.radians(23.5)


def _arc_points(radius: float, z_center: float, start: float, end: float, steps: int):
    return [
        (radius * math.cos(a), 0.0, z_center + radius * math.sin(a))
        for a in [start + (end - start) * i / (steps - 1) for i in range(steps)]
    ]


def _tube(points, radius: float, radial_segments: int = 18):
    return wire_from_points(
        points,
        radius=radius,
        radial_segments=radial_segments,
        closed_path=False,
        cap_ends=True,
        corner_mode="miter",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_standing_globe")

    wood = model.material("warm_wood", rgba=(0.43, 0.22, 0.09, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.20, 0.11, 0.05, 1.0))
    brass = model.material("brushed_brass", rgba=(0.86, 0.63, 0.22, 1.0))
    ocean = model.material("satin_ocean_blue", rgba=(0.06, 0.28, 0.58, 1.0))
    land = model.material("raised_land_green", rgba=(0.15, 0.48, 0.18, 1.0))
    map_line = model.material("antique_map_line", rgba=(0.91, 0.77, 0.43, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.40, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_wood,
        name="base_plinth",
    )
    base.visual(
        Cylinder(radius=0.33, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=wood,
        name="upper_step",
    )
    base.visual(
        Cylinder(radius=0.17, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=brass,
        name="bearing_race",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.31, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=wood,
        name="turntable_disk",
    )
    turntable.visual(
        Cylinder(radius=0.325, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=brass,
        name="brass_rim",
    )
    turntable.visual(
        Cylinder(radius=0.048, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=wood,
        name="pedestal_column",
    )
    turntable.visual(
        Cylinder(radius=0.080, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=brass,
        name="lower_collar",
    )
    turntable.visual(
        Cylinder(radius=0.073, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        material=brass,
        name="upper_collar",
    )

    axis = (math.sin(AXIS_TILT), 0.0, math.cos(AXIS_TILT))
    north_angle = math.pi / 2.0 - AXIS_TILT
    south_angle = north_angle + math.pi
    if south_angle > math.pi:
        south_angle -= 2.0 * math.pi

    meridian = _arc_points(
        MERIDIAN_RADIUS,
        GLOBE_CENTER_Z,
        south_angle - math.radians(12.0),
        north_angle + math.radians(14.0),
        84,
    )
    turntable.visual(
        mesh_from_geometry(_tube(meridian, 0.0145, radial_segments=20), "meridian_arc"),
        material=brass,
        name="meridian_arc",
    )

    globe_center = (0.0, 0.0, GLOBE_CENTER_Z)
    north_ring = (
        axis[0] * MERIDIAN_RADIUS,
        0.0,
        GLOBE_CENTER_Z + axis[2] * MERIDIAN_RADIUS,
    )
    south_ring = (
        -axis[0] * MERIDIAN_RADIUS,
        0.0,
        GLOBE_CENTER_Z - axis[2] * MERIDIAN_RADIUS,
    )
    column_top = (0.0, 0.0, 0.505)
    turntable.visual(
        mesh_from_geometry(
            _tube([column_top, south_ring], 0.015, radial_segments=18),
            "lower_yoke",
        ),
        material=brass,
        name="lower_yoke",
    )
    turntable.visual(
        mesh_from_geometry(
            _tube(
                [
                    (0.045, 0.0, 0.505),
                    (0.130, 0.0, 0.585),
                    meridian[8],
                ],
                0.010,
                radial_segments=14,
            ),
            "side_brace",
        ),
        material=brass,
        name="side_brace",
    )

    # The stand's pivot pins are modeled as seated a few millimeters into the
    # globe's metal polar caps so the articulated globe is visibly retained by
    # the meridian rather than floating inside it.
    pivot_gap = -0.006
    cap_length = 0.018
    rod_inner = GLOBE_RADIUS + cap_length + pivot_gap
    for sign, name in ((1.0, "north_pivot"), (-1.0, "south_pivot")):
        rod = [
            (
                sign * axis[0] * rod_inner,
                0.0,
                GLOBE_CENTER_Z + sign * axis[2] * rod_inner,
            ),
            (
                sign * axis[0] * MERIDIAN_RADIUS,
                0.0,
                GLOBE_CENTER_Z + sign * axis[2] * MERIDIAN_RADIUS,
            ),
        ]
        turntable.visual(
            mesh_from_geometry(
                _tube(rod, 0.017, radial_segments=18),
                f"{name}_pin",
            ),
            material=brass,
            name=f"{name}_pin",
        )
        ring_point = north_ring if sign > 0 else south_ring
        turntable.visual(
            Cylinder(radius=0.052, length=0.026),
            origin=Origin(xyz=ring_point, rpy=(0.0, AXIS_TILT, 0.0)),
            material=brass,
            name=f"{name}_bearing",
        )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        origin=Origin(),
        material=ocean,
        name="globe_sphere",
    )

    line_radius = 0.0026
    globe.visual(
        mesh_from_geometry(TorusGeometry(GLOBE_RADIUS + line_radius, line_radius), "equator_line"),
        material=map_line,
        name="equator_line",
    )
    for z_lat, visual_name in ((0.155, "north_parallel"), (-0.155, "south_parallel")):
        parallel_radius = math.sqrt(GLOBE_RADIUS * GLOBE_RADIUS - z_lat * z_lat) + line_radius
        geom = TorusGeometry(parallel_radius, line_radius * 0.72).translate(0.0, 0.0, z_lat)
        globe.visual(
            mesh_from_geometry(geom, visual_name),
            material=map_line,
            name=visual_name,
        )
    for idx, yaw in enumerate((0.0, math.radians(60.0), math.radians(120.0))):
        geom = TorusGeometry(GLOBE_RADIUS + line_radius, line_radius * 0.72).rotate_x(math.pi / 2.0).rotate_z(yaw)
        globe.visual(
            mesh_from_geometry(geom, f"meridian_line_{idx}"),
            material=map_line,
            name=f"meridian_line_{idx}",
        )

    continent_specs = [
        ("continent_0", (0.007, 0.090, 0.052), (GLOBE_RADIUS + 0.001, -0.035, 0.030)),
        ("continent_1", (0.078, 0.007, 0.060), (-0.045, GLOBE_RADIUS + 0.001, -0.055)),
        ("continent_2", (0.007, 0.065, 0.076), (-GLOBE_RADIUS - 0.001, 0.050, -0.025)),
        ("continent_3", (0.060, 0.007, 0.038), (0.075, -0.306, 0.085)),
    ]
    for visual_name, scale, offset in continent_specs:
        geom = SphereGeometry(1.0, width_segments=18, height_segments=10).scale(*scale).translate(*offset)
        globe.visual(
            mesh_from_geometry(geom, visual_name),
            material=land,
            name=visual_name,
        )

    for sign, name in ((1.0, "north_cap"), (-1.0, "south_cap")):
        globe.visual(
            Cylinder(radius=0.045, length=cap_length),
            origin=Origin(xyz=(0.0, 0.0, sign * (GLOBE_RADIUS + cap_length / 2.0))),
            material=brass,
            name=name,
        )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
    )

    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=turntable,
        child=globe,
        origin=Origin(xyz=globe_center, rpy=(0.0, AXIS_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    globe = object_model.get_part("globe")
    swivel = object_model.get_articulation("base_swivel")
    spin = object_model.get_articulation("globe_spin")

    for cap_name, pin_name in (
        ("north_cap", "north_pivot_pin"),
        ("south_cap", "south_pivot_pin"),
    ):
        ctx.allow_overlap(
            globe,
            turntable,
            elem_a=cap_name,
            elem_b=pin_name,
            reason="The polar pivot pin is intentionally seated into the globe's metal cap as a captured bearing.",
        )
        ctx.expect_overlap(
            globe,
            turntable,
            axes="xyz",
            elem_a=cap_name,
            elem_b=pin_name,
            min_overlap=0.001,
            name=f"{cap_name} captures its pivot pin",
        )

    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="bearing_race",
        min_gap=0.0,
        max_gap=0.004,
        name="turntable disk rides just above base bearing",
    )
    ctx.expect_overlap(
        turntable,
        base,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="base_plinth",
        min_overlap=0.25,
        name="turntable is centered on circular base",
    )
    ctx.expect_origin_gap(
        globe,
        turntable,
        axis="z",
        min_gap=0.88,
        max_gap=0.96,
        name="globe is carried high over the pedestal",
    )
    ctx.expect_overlap(
        globe,
        turntable,
        axes="xz",
        elem_a="globe_sphere",
        elem_b="meridian_arc",
        min_overlap=0.35,
        name="sphere sits inside the partial meridian projection",
    )

    ctx.check(
        "stand swivel uses vertical revolute axis",
        swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.check(
        "globe spin uses polar axis in tilted joint frame",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    def _center_from_aabb(box):
        if box is None:
            return None
        lo, hi = box
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    meridian_rest = _center_from_aabb(ctx.part_element_world_aabb(turntable, elem="meridian_arc"))
    with ctx.pose({swivel: 1.0}):
        meridian_rotated = _center_from_aabb(ctx.part_element_world_aabb(turntable, elem="meridian_arc"))
    ctx.check(
        "base swivel rotates the asymmetric meridian support",
        meridian_rest is not None
        and meridian_rotated is not None
        and math.hypot(meridian_rotated[0] - meridian_rest[0], meridian_rotated[1] - meridian_rest[1]) > 0.05,
        details=f"rest={meridian_rest}, rotated={meridian_rotated}",
    )

    continent_rest = _center_from_aabb(ctx.part_element_world_aabb(globe, elem="continent_0"))
    with ctx.pose({spin: 1.1}):
        continent_rotated = _center_from_aabb(ctx.part_element_world_aabb(globe, elem="continent_0"))
    ctx.check(
        "globe spin moves map markings around the polar axis",
        continent_rest is not None
        and continent_rotated is not None
        and math.dist(continent_rest, continent_rotated) > 0.08,
        details=f"rest={continent_rest}, rotated={continent_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
