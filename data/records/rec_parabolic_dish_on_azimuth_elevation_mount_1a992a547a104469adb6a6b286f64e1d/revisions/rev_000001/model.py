from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _dish_depth(radius: float, outer_radius: float, dish_depth: float) -> float:
    return dish_depth * (radius / outer_radius) ** 2


def _build_reflector_mesh(
    *,
    outer_radius: float,
    dish_depth: float,
    hub_radius: float,
    rib_count: int,
):
    ring_radii = (0.18, 0.30, 0.42, 0.50, outer_radius)
    reflector = None

    for ring_radius in ring_radii:
        ring = TorusGeometry(
            radius=ring_radius,
            tube=0.0085 if ring_radius >= outer_radius - 1e-6 else 0.006,
            radial_segments=14,
            tubular_segments=52,
        )
        ring.rotate_y(math.pi / 2.0)
        ring.translate(_dish_depth(ring_radius, outer_radius, dish_depth), 0.0, 0.0)
        reflector = ring if reflector is None else reflector.merge(ring)

    rib_profile = [
        (_dish_depth(radius, outer_radius, dish_depth), radius, 0.0)
        for radius in (hub_radius, 0.14, 0.24, 0.36, 0.48, outer_radius)
    ]
    rib = tube_from_spline_points(
        rib_profile,
        radius=0.0055,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    reflector = reflector.merge(rib)
    for index in range(1, rib_count):
        reflector.merge(rib.copy().rotate_x((2.0 * math.pi * index) / rib_count))

    return mesh_from_geometry(reflector, "mesh_reflector")


def _build_feed_boom_mesh():
    boom = tube_from_spline_points(
        [
            (0.040, 0.070, -0.060),
            (0.125, 0.092, -0.050),
            (0.290, 0.060, -0.020),
            (0.410, 0.000, 0.000),
        ],
        radius=0.011,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(boom, "feed_boom")


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_mesh_dish")

    roof_gray = model.material("roof_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    galvanized = model.material("galvanized", rgba=(0.70, 0.72, 0.74, 1.0))
    mount_gray = model.material("mount_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.09, 0.10, 1.0))

    reflector_mesh = _build_reflector_mesh(
        outer_radius=0.55,
        dish_depth=0.16,
        hub_radius=0.055,
        rib_count=16,
    )
    feed_boom_mesh = _build_feed_boom_mesh()

    roof_mount = model.part("roof_mount")
    roof_mount.visual(
        Box((0.70, 0.48, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=roof_gray,
        name="roof_pad",
    )
    roof_mount.visual(
        Box((0.20, 0.24, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=mount_gray,
        name="base_plate",
    )
    roof_mount.visual(
        Cylinder(radius=0.038, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=galvanized,
        name="mast_tube",
    )
    roof_mount.visual(
        Cylinder(radius=0.050, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=mount_gray,
        name="mast_head",
    )
    roof_mount.visual(
        Box((0.12, 0.16, 0.08)),
        origin=Origin(xyz=(-0.06, 0.0, 0.075)),
        material=mount_gray,
        name="mast_socket",
    )
    _add_member(
        roof_mount,
        (-0.13, -0.10, 0.0275),
        (-0.01, -0.035, 0.265),
        radius=0.012,
        material=galvanized,
        name="left_backstay",
    )
    _add_member(
        roof_mount,
        (-0.13, 0.10, 0.0275),
        (-0.01, 0.035, 0.265),
        radius=0.012,
        material=galvanized,
        name="right_backstay",
    )
    roof_mount.visual(
        Box((0.18, 0.06, 0.06)),
        origin=Origin(xyz=(-0.09, 0.0, 0.05)),
        material=mount_gray,
        name="rear_bracket_block",
    )
    roof_mount.inertial = Inertial.from_geometry(
        Box((0.70, 0.48, 0.40)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    azimuth_fork = model.part("azimuth_fork")
    azimuth_fork.visual(
        Cylinder(radius=0.058, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_metal,
        name="azimuth_bearing",
    )
    azimuth_fork.visual(
        Cylinder(radius=0.076, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=mount_gray,
        name="turntable_cap",
    )
    azimuth_fork.visual(
        Box((0.14, 0.30, 0.035)),
        origin=Origin(xyz=(0.030, 0.0, 0.122)),
        material=mount_gray,
        name="fork_base_bridge",
    )
    azimuth_fork.visual(
        Box((0.05, 0.05, 0.34)),
        origin=Origin(xyz=(0.025, -0.135, 0.190)),
        material=mount_gray,
        name="left_fork_arm",
    )
    azimuth_fork.visual(
        Box((0.05, 0.05, 0.34)),
        origin=Origin(xyz=(0.025, 0.135, 0.190)),
        material=mount_gray,
        name="right_fork_arm",
    )
    azimuth_fork.visual(
        Box((0.06, 0.28, 0.04)),
        origin=Origin(xyz=(0.000, 0.0, 0.380)),
        material=mount_gray,
        name="upper_fork_bridge",
    )
    azimuth_fork.visual(
        Cylinder(radius=0.032, length=0.032),
        origin=Origin(
            xyz=(0.078, -0.152, 0.260),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="left_bearing_boss",
    )
    azimuth_fork.visual(
        Cylinder(radius=0.032, length=0.032),
        origin=Origin(
            xyz=(0.078, 0.152, 0.260),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="right_bearing_boss",
    )
    azimuth_fork.inertial = Inertial.from_geometry(
        Box((0.24, 0.32, 0.42)),
        mass=7.5,
        origin=Origin(xyz=(0.05, 0.0, 0.20)),
    )

    dish_assembly = model.part("dish_assembly")
    dish_assembly.visual(
        Cylinder(radius=0.028, length=0.304),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="trunnion_shaft",
    )
    dish_assembly.visual(
        Box((0.11, 0.14, 0.15)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=mount_gray,
        name="hub_block",
    )
    dish_assembly.visual(
        Cylinder(radius=0.060, length=0.14),
        origin=Origin(
            xyz=(0.065, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_metal,
        name="hub_barrel",
    )
    dish_assembly.visual(
        reflector_mesh,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=aluminum,
        name="reflector_mesh",
    )
    dish_assembly.visual(
        Box((0.060, 0.060, 0.050)),
        origin=Origin(xyz=(0.065, 0.070, -0.060)),
        material=mount_gray,
        name="boom_clamp",
    )
    dish_assembly.visual(
        feed_boom_mesh,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=galvanized,
        name="feed_boom",
    )
    dish_assembly.visual(
        Cylinder(radius=0.042, length=0.12),
        origin=Origin(
            xyz=(0.500, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=matte_black,
        name="feed_horn",
    )
    dish_assembly.visual(
        Box((0.070, 0.055, 0.060)),
        origin=Origin(xyz=(0.435, 0.0, 0.0)),
        material=matte_black,
        name="lnb_body",
    )
    dish_assembly.visual(
        Box((0.10, 0.03, 0.06)),
        origin=Origin(xyz=(-0.030, 0.0, -0.090)),
        material=mount_gray,
        name="elevation_hub_plate",
    )
    dish_assembly.inertial = Inertial.from_geometry(
        Box((0.62, 1.10, 1.10)),
        mass=6.5,
        origin=Origin(xyz=(0.21, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=roof_mount,
        child=azimuth_fork,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6),
    )
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=azimuth_fork,
        child=dish_assembly,
        origin=Origin(xyz=(0.10, 0.0, 0.26)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.1,
            lower=-0.25,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    roof_mount = object_model.get_part("roof_mount")
    azimuth_fork = object_model.get_part("azimuth_fork")
    dish_assembly = object_model.get_part("dish_assembly")
    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    elevation_tilt = object_model.get_articulation("elevation_tilt")
    left_bearing_boss = azimuth_fork.get_visual("left_bearing_boss")
    right_bearing_boss = azimuth_fork.get_visual("right_bearing_boss")
    trunnion_shaft = dish_assembly.get_visual("trunnion_shaft")

    for part_name, part in (
        ("roof_mount", roof_mount),
        ("azimuth_fork", azimuth_fork),
        ("dish_assembly", dish_assembly),
    ):
        ctx.check(f"{part_name} exists", part is not None, details=part_name)

    ctx.check(
        "azimuth axis is vertical",
        azimuth_rotation.axis == (0.0, 0.0, 1.0),
        details=f"axis={azimuth_rotation.axis}",
    )
    ctx.check(
        "elevation axis is horizontal",
        elevation_tilt.axis == (0.0, -1.0, 0.0),
        details=f"axis={elevation_tilt.axis}",
    )
    ctx.check(
        "elevation limits allow aiming up and slightly down",
        elevation_tilt.motion_limits is not None
        and elevation_tilt.motion_limits.lower is not None
        and elevation_tilt.motion_limits.upper is not None
        and elevation_tilt.motion_limits.lower < 0.0
        and elevation_tilt.motion_limits.upper > 0.8,
        details=f"limits={elevation_tilt.motion_limits}",
    )

    ctx.allow_overlap(
        azimuth_fork,
        dish_assembly,
        elem_a=left_bearing_boss,
        elem_b=trunnion_shaft,
        reason=(
            "The fork's left bearing boss is modeled as a solid housing around the "
            "dish trunnion instead of as a hollow bored sleeve."
        ),
    )
    ctx.allow_overlap(
        azimuth_fork,
        dish_assembly,
        elem_a=right_bearing_boss,
        elem_b=trunnion_shaft,
        reason=(
            "The fork's right bearing boss is modeled as a solid housing around the "
            "dish trunnion instead of as a hollow bored sleeve."
        ),
    )

    with ctx.pose({azimuth_rotation: 0.0, elevation_tilt: 0.0}):
        ctx.expect_gap(
            azimuth_fork,
            roof_mount,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="azimuth fork seats on the mast head",
        )
        ctx.expect_contact(
            dish_assembly,
            azimuth_fork,
            contact_tol=0.001,
            name="dish trunnion is supported by the elevation fork",
        )
        rest_feed_center = _aabb_center(
            ctx.part_element_world_aabb(dish_assembly, elem="feed_horn")
        )

    with ctx.pose({elevation_tilt: 0.65}):
        raised_feed_center = _aabb_center(
            ctx.part_element_world_aabb(dish_assembly, elem="feed_horn")
        )

    ctx.check(
        "positive elevation raises the feed horn",
        rest_feed_center is not None
        and raised_feed_center is not None
        and raised_feed_center[2] > rest_feed_center[2] + 0.20,
        details=f"rest={rest_feed_center}, raised={raised_feed_center}",
    )

    with ctx.pose({azimuth_rotation: math.pi / 2.0}):
        swung_feed_center = _aabb_center(
            ctx.part_element_world_aabb(dish_assembly, elem="feed_horn")
        )

    ctx.check(
        "positive azimuth swings the dish around the mast",
        rest_feed_center is not None
        and swung_feed_center is not None
        and swung_feed_center[1] > rest_feed_center[1] + 0.25,
        details=f"rest={rest_feed_center}, swung={swung_feed_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
