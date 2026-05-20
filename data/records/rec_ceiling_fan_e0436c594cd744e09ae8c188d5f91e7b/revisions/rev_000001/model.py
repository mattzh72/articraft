from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BLADE_COUNT = 6
BLADE_LENGTH = 1.06
BLADE_WIDTH = 0.145
BLADE_THICKNESS = 0.024
BLADE_CENTER_RADIUS = 0.92
BLADE_PITCH = math.radians(7.0)
IRON_DROP = math.radians(10.0)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _make_blade_mesh() -> MeshGeometry:
    """A long, narrow rounded plank with a shallow lengthwise crown."""
    blade = ExtrudeGeometry(
        rounded_rect_profile(
            BLADE_LENGTH,
            BLADE_WIDTH,
            BLADE_WIDTH * 0.48,
            corner_segments=12,
        ),
        BLADE_THICKNESS,
        center=True,
    )
    # A subtle raised center batten makes the brown mesh read as wood-veneer
    # rather than a plain flat rectangle, while staying integral with the blade.
    batten = ExtrudeGeometry(
        rounded_rect_profile(BLADE_LENGTH * 0.92, 0.018, 0.009, corner_segments=5),
        BLADE_THICKNESS * 0.55,
        center=True,
    ).translate(0.0, 0.0, BLADE_THICKNESS * 0.63)
    return _merge_geometries([blade, batten])


def _canopy_mesh() -> MeshGeometry:
    profile = [
        (0.000, 1.080),
        (0.065, 1.080),
        (0.138, 1.095),
        (0.172, 1.132),
        (0.182, 1.180),
        (0.168, 1.224),
        (0.128, 1.252),
        (0.040, 1.270),
        (0.000, 1.270),
    ]
    return LatheGeometry(profile, segments=72)


def _motor_upper_mesh() -> MeshGeometry:
    profile = [
        (0.000, 0.082),
        (0.132, 0.082),
        (0.218, 0.096),
        (0.262, 0.140),
        (0.270, 0.196),
        (0.246, 0.252),
        (0.164, 0.302),
        (0.058, 0.322),
        (0.000, 0.322),
    ]
    return LatheGeometry(profile, segments=88)


def _rotor_shell_mesh() -> MeshGeometry:
    profile = [
        (0.000, -0.105),
        (0.106, -0.105),
        (0.205, -0.086),
        (0.252, -0.040),
        (0.258, 0.020),
        (0.230, 0.064),
        (0.092, 0.070),
        (0.000, 0.070),
    ]
    return LatheGeometry(profile, segments=88)


def _lower_cap_mesh() -> MeshGeometry:
    profile = [
        (0.000, -0.200),
        (0.050, -0.195),
        (0.092, -0.168),
        (0.115, -0.128),
        (0.108, -0.104),
        (0.000, -0.104),
    ]
    return LatheGeometry(profile, segments=72)


def _wood_grain_materials(model: ArticulatedObject) -> tuple[Material, Material, Material]:
    metal = model.material("oiled_bronze", rgba=(0.09, 0.075, 0.060, 1.0))
    dark_metal = model.material("dark_iron", rgba=(0.025, 0.024, 0.023, 1.0))
    wood = model.material("warm_wood_veneer", rgba=(0.56, 0.34, 0.16, 1.0))
    model.material("dark_wood_grain", rgba=(0.26, 0.13, 0.055, 1.0))
    return metal, dark_metal, wood


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windmill_ceiling_fan")
    metal, dark_metal, wood = _wood_grain_materials(model)
    grain = "dark_wood_grain"

    support = model.part("support")
    support.visual(
        mesh_from_geometry(_canopy_mesh(), "round_canopy"),
        material=metal,
        name="round_canopy",
    )
    support.visual(
        Cylinder(radius=0.024, length=0.780),
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        material=dark_metal,
        name="downrod",
    )
    support.visual(
        Cylinder(radius=0.048, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 1.080)),
        material=metal,
        name="upper_rod_collar",
    )
    support.visual(
        Cylinder(radius=0.058, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=metal,
        name="lower_rod_collar",
    )
    support.visual(
        mesh_from_geometry(_motor_upper_mesh(), "motor_upper"),
        material=metal,
        name="motor_upper",
    )
    support.visual(
        Cylinder(radius=0.078, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=dark_metal,
        name="bearing_ring",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(_rotor_shell_mesh(), "rotor_shell"),
        material=metal,
        name="rotor_shell",
    )
    rotor.visual(
        mesh_from_geometry(_lower_cap_mesh(), "lower_cap"),
        material=metal,
        name="lower_cap",
    )

    blade_mesh = mesh_from_geometry(_make_blade_mesh(), "rounded_wood_blade")
    root_plate = Box((0.150, 0.118, 0.018))
    iron_bar = Box((0.450, 0.018, 0.026))
    for index in range(BLADE_COUNT):
        yaw = index * math.tau / BLADE_COUNT
        rotor.visual(
            blade_mesh,
            origin=Origin(
                xyz=(
                    BLADE_CENTER_RADIUS * math.cos(yaw),
                    BLADE_CENTER_RADIUS * math.sin(yaw),
                    -0.145,
                ),
                rpy=(BLADE_PITCH, 0.0, yaw),
            ),
            material=wood,
            name=f"blade_{index}",
        )
        # Two splayed blade irons overlap the hub and the inner blade root so the
        # rotor reads as one connected moving assembly.
        for offset_index, offset in enumerate((-0.043, 0.043)):
            radial = 0.318
            x = radial * math.cos(yaw) - offset * math.sin(yaw)
            y = radial * math.sin(yaw) + offset * math.cos(yaw)
            rotor.visual(
                iron_bar,
                origin=Origin(
                    xyz=(x, y, -0.088),
                    rpy=(0.0, IRON_DROP, yaw),
                ),
                material=dark_metal,
                name=f"iron_{index}_{offset_index}",
            )
        rotor.visual(
            root_plate,
            origin=Origin(
                xyz=(0.435 * math.cos(yaw), 0.435 * math.sin(yaw), -0.128),
                rpy=(BLADE_PITCH, 0.0, yaw),
            ),
            material=dark_metal,
            name=f"blade_plate_{index}",
        )
        # Fine darker veneer lines are embedded slightly into the blade surface
        # rather than floating over it.
        for stripe_index, stripe_y in enumerate((-0.045, 0.045)):
            sx = BLADE_CENTER_RADIUS * math.cos(yaw) - stripe_y * math.sin(yaw)
            sy = BLADE_CENTER_RADIUS * math.sin(yaw) + stripe_y * math.cos(yaw)
            rotor.visual(
                Box((BLADE_LENGTH * 0.76, 0.006, 0.006)),
                origin=Origin(
                    xyz=(sx, sy, -0.129),
                    rpy=(BLADE_PITCH, 0.0, yaw),
                ),
                material=grain,
                name=f"grain_{index}_{stripe_index}",
            )

    model.articulation(
        "spin_axle",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("spin_axle")

    ctx.check("six_visible_blades", all(rotor.get_visual(f"blade_{i}") for i in range(BLADE_COUNT)))
    ctx.check(
        "central_continuous_spin",
        spin is not None
        and spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"spin={spin}",
    )
    ctx.expect_gap(
        support,
        rotor,
        axis="z",
        positive_elem="motor_upper",
        negative_elem="rotor_shell",
        min_gap=0.006,
        max_gap=0.018,
        name="motor bearing clearance",
    )
    ctx.expect_overlap(
        rotor,
        support,
        axes="xy",
        elem_a="rotor_shell",
        elem_b="motor_upper",
        min_overlap=0.35,
        name="rotor centered under motor",
    )

    blade_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")
    rest_center = None
    if blade_aabb is not None:
        mins, maxs = blade_aabb
        rest_center = ((mins[0] + maxs[0]) * 0.5, (mins[1] + maxs[1]) * 0.5)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")
        turned_center = None
        if turned_aabb is not None:
            mins, maxs = turned_aabb
            turned_center = ((mins[0] + maxs[0]) * 0.5, (mins[1] + maxs[1]) * 0.5)
    ctx.check(
        "blade_0_orbits_axle",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.70
        and turned_center[1] > 0.70,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
