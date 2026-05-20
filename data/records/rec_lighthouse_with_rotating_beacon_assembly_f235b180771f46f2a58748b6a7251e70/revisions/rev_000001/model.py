from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _lathe_solid(profile: list[tuple[float, float]], name: str):
    return mesh_from_geometry(LatheGeometry(profile, segments=72), name)


def _ring_shell(
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    name: str,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=72,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lighthouse_lantern_assembly")

    stone = model.material("warm_white_stone", rgba=(0.78, 0.74, 0.64, 1.0))
    dark_metal = model.material("weathered_black_metal", rgba=(0.03, 0.035, 0.04, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.55, 0.39, 0.18, 1.0))
    glass = model.material("pale_aqua_glass", rgba=(0.62, 0.88, 1.0, 0.32))
    silver = model.material("polished_silver", rgba=(0.86, 0.84, 0.78, 1.0))
    warm_light = model.material("warm_lamp_glass", rgba=(1.0, 0.78, 0.18, 0.88))

    lantern = model.part("lantern_room")

    lantern.visual(
        _lathe_solid(
            [
                (0.0, 0.00),
                (0.72, 0.00),
                (0.62, 0.36),
                (0.52, 0.68),
                (0.0, 0.68),
            ],
            "tapered_tower_top_mesh",
        ),
        material=stone,
        name="tapered_tower_top",
    )
    lantern.visual(
        _ring_shell(0.64, 0.40, 0.66, 0.74, "gallery_deck_ring_mesh"),
        material=dark_metal,
        name="gallery_deck_ring",
    )
    lantern.visual(
        Cylinder(radius=0.105, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        material=bronze,
        name="shaft_pedestal",
    )

    # A hollow cylindrical lantern-room glazing held between metal annular rings.
    lantern.visual(
        _ring_shell(0.49, 0.415, 0.74, 0.81, "lower_glazing_ring_mesh"),
        material=dark_metal,
        name="lower_glazing_ring",
    )
    lantern.visual(
        _ring_shell(0.445, 0.425, 0.81, 1.46, "glass_shell_mesh"),
        material=glass,
        name="glass_shell",
    )
    lantern.visual(
        _ring_shell(0.49, 0.415, 1.46, 1.54, "upper_glazing_ring_mesh"),
        material=dark_metal,
        name="upper_glazing_ring",
    )

    for i in range(8):
        theta = i * math.tau / 8.0
        r = 0.452
        lantern.visual(
            Box((0.030, 0.026, 0.70)),
            origin=Origin(
                xyz=(r * math.cos(theta), r * math.sin(theta), 1.135),
                rpy=(0.0, 0.0, theta),
            ),
            material=dark_metal,
            name=f"glass_mullion_{i}",
        )

    # Gallery rail outside the lantern, connected to the deck ring by posts.
    for i in range(12):
        theta = i * math.tau / 12.0
        r = 0.642
        lantern.visual(
            Box((0.034, 0.022, 0.21)),
            origin=Origin(
                xyz=(r * math.cos(theta), r * math.sin(theta), 0.835),
                rpy=(0.0, 0.0, theta),
            ),
            material=dark_metal,
            name=f"gallery_post_{i}",
        )
    lantern.visual(
        _ring_shell(0.668, 0.625, 0.925, 0.955, "gallery_guard_ring_mesh"),
        material=dark_metal,
        name="gallery_guard_ring",
    )

    lantern.visual(
        Cylinder(radius=0.024, length=0.86),
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        material=bronze,
        name="central_shaft",
    )
    lantern.visual(
        Cylinder(radius=0.060, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 1.555)),
        material=bronze,
        name="upper_bearing_collar",
    )

    lantern.visual(
        _ring_shell(0.57, 0.41, 1.54, 1.60, "roof_eave_ring_mesh"),
        material=dark_metal,
        name="roof_eave_ring",
    )
    lantern.visual(
        _lathe_solid(
            [
                (0.0, 1.585),
                (0.57, 1.585),
                (0.34, 1.82),
                (0.13, 1.96),
                (0.0, 1.99),
            ],
            "copper_roof_mesh",
        ),
        material=bronze,
        name="copper_roof",
    )
    lantern.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.03)),
        material=dark_metal,
        name="roof_vent_cap",
    )
    lantern.visual(
        Sphere(radius=0.040),
        origin=Origin(xyz=(0.0, 0.0, 2.11)),
        material=dark_metal,
        name="finial_ball",
    )

    beacon = model.part("beacon_carriage")
    beacon.visual(
        _ring_shell(0.049, 0.020, -0.255, 0.255, "rotor_hub_shell_mesh"),
        material=bronze,
        name="rotor_hub_shell",
    )

    # Split arms start outside the fixed shaft clearance and tie into the hub shell.
    for sign, label in ((1.0, "front"), (-1.0, "rear")):
        beacon.visual(
            Box((0.188, 0.026, 0.026)),
            origin=Origin(xyz=(sign * 0.137, 0.0, 0.0)),
            material=bronze,
            name=f"{label}_lower_arm",
        )
        beacon.visual(
            Box((0.150, 0.018, 0.018)),
            origin=Origin(xyz=(sign * 0.161, 0.0, 0.095)),
            material=bronze,
            name=f"{label}_upper_arm",
        )
        beacon.visual(
            Box((0.018, 0.018, 0.205)),
            origin=Origin(xyz=(sign * 0.210, 0.0, 0.020)),
            material=bronze,
            name=f"{label}_side_upright",
        )

    # Opposed lamp/reflector heads, small enough to rotate clear of the glass.
    beacon.visual(
        mesh_from_geometry(DomeGeometry(0.086, radial_segments=48, height_segments=12, closed=False), "front_reflector_mesh"),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=silver,
        name="front_reflector",
    )
    beacon.visual(
        mesh_from_geometry(DomeGeometry(0.086, radial_segments=48, height_segments=12, closed=False), "rear_reflector_mesh"),
        origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="rear_reflector",
    )
    for sign, label in ((1.0, "front"), (-1.0, "rear")):
        beacon.visual(
            Box((0.055, 0.016, 0.016)),
            origin=Origin(xyz=(sign * 0.239, 0.0, 0.0)),
            material=bronze,
            name=f"{label}_lamp_stem",
        )
        beacon.visual(
            Sphere(radius=0.035),
            origin=Origin(xyz=(sign * 0.248, 0.0, 0.0)),
            material=warm_light,
            name=f"{label}_lamp_globe",
        )
    beacon.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=Origin(xyz=(0.286, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    beacon.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=Origin(xyz=(-0.286, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="rear_lens",
    )

    model.articulation(
        "shaft_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=lantern,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lantern = object_model.get_part("lantern_room")
    beacon = object_model.get_part("beacon_carriage")
    joint = object_model.get_articulation("shaft_to_beacon")

    ctx.allow_overlap(
        lantern,
        beacon,
        elem_a="central_shaft",
        elem_b="rotor_hub_shell",
        reason="The rotating hub is intentionally captured on the fixed vertical shaft with a slight hidden bearing interference proxy.",
    )

    ctx.check(
        "beacon uses continuous vertical shaft axis",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_within(
        beacon,
        lantern,
        axes="xy",
        inner_elem="front_lens",
        outer_elem="glass_shell",
        margin=0.0,
        name="front beacon head stays inside glass cylinder",
    )
    ctx.expect_within(
        beacon,
        lantern,
        axes="xy",
        inner_elem="rear_lens",
        outer_elem="glass_shell",
        margin=0.0,
        name="rear beacon head stays inside glass cylinder",
    )
    ctx.expect_within(
        beacon,
        lantern,
        axes="xy",
        inner_elem="rotor_hub_shell",
        outer_elem="glass_shell",
        margin=0.0,
        name="rotor hub is centered within the enclosure",
    )
    ctx.expect_overlap(
        beacon,
        lantern,
        axes="z",
        elem_a="rotor_hub_shell",
        elem_b="central_shaft",
        min_overlap=0.40,
        name="rotor hub is carried along the fixed shaft height",
    )

    def _center_of_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_center = _center_of_aabb(ctx.part_element_world_aabb(beacon, elem="front_lens"))
    with ctx.pose({joint: math.pi / 2.0}):
        turned_center = _center_of_aabb(ctx.part_element_world_aabb(beacon, elem="front_lens"))
        ctx.expect_within(
            beacon,
            lantern,
            axes="xy",
            inner_elem="front_lens",
            outer_elem="glass_shell",
            margin=0.0,
            name="rotated beacon remains inside the glass cylinder",
        )

    ctx.check(
        "front lens sweeps around the shaft",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.24
        and turned_center[1] > 0.24
        and abs(turned_center[0]) < 0.04,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
