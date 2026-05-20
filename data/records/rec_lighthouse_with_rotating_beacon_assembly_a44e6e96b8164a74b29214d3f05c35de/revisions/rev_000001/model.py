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
    TorusGeometry,
    mesh_from_geometry,
)


def _tower_radius_at(z: float) -> float:
    """Outer masonry radius for the tapered tower shaft."""
    bottom_radius = 2.30
    top_radius = 0.92
    shaft_height = 17.20
    t = max(0.0, min(1.0, z / shaft_height))
    return bottom_radius + (top_radius - bottom_radius) * t


def _lathed_solid(profile: list[tuple[float, float]], name: str, *, segments: int = 96):
    return mesh_from_geometry(LatheGeometry(profile, segments=segments), name)


def _lathed_shell(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    name: str,
    *,
    segments: int = 96,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_masonry_lighthouse")

    white_masonry = model.material("white_masonry", rgba=(0.86, 0.84, 0.76, 1.0))
    warm_stone = model.material("warm_stone", rgba=(0.62, 0.56, 0.48, 1.0))
    red_brick = model.material("red_brick", rgba=(0.55, 0.12, 0.08, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.04, 0.045, 0.05, 1.0))
    weathered_copper = model.material("weathered_copper", rgba=(0.44, 0.15, 0.08, 1.0))
    brass = model.material("brass", rgba=(0.85, 0.58, 0.18, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.55, 0.80, 0.96, 0.35))
    beacon_glass = model.material("beacon_glass", rgba=(1.0, 0.78, 0.20, 0.55))
    beacon_glow = model.material("beacon_glow", rgba=(1.0, 0.88, 0.30, 0.78))
    door_green = model.material("door_green", rgba=(0.05, 0.30, 0.21, 1.0))
    shadow_black = model.material("shadow_black", rgba=(0.01, 0.012, 0.014, 1.0))

    tower = model.part("tower")

    tower.visual(
        _lathed_solid(
            [(0.0, 0.0), (2.30, 0.0), (0.92, 17.20), (0.0, 17.20)],
            "tapered_masonry_tower",
        ),
        material=white_masonry,
        name="tapered_tower",
    )

    # Rounded mortar/brick courses emphasize the masonry scale of the taper.
    for index, z in enumerate((1.2, 2.9, 4.6, 6.3, 8.0, 9.7, 11.4, 13.1, 14.8, 16.2)):
        tower.visual(
            mesh_from_geometry(
                TorusGeometry(_tower_radius_at(z) + 0.010, 0.026, radial_segments=96, tubular_segments=10),
                f"masonry_course_{index}",
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=warm_stone if index % 2 else red_brick,
            name=f"course_{index}",
        )

    tower.visual(
        Cylinder(radius=2.42, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=warm_stone,
        name="stone_plinth",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(1.16, 0.055, radial_segments=96, tubular_segments=12), "red_gallery_belt"),
        origin=Origin(xyz=(0.0, 0.0, 17.18)),
        material=red_brick,
        name="gallery_belt",
    )

    # Small dark arched-looking windows sit proud of the masonry along the taper.
    for index, (z, angle) in enumerate(((4.2, -math.pi / 2), (8.2, math.pi / 2), (12.4, -math.pi / 2))):
        r = _tower_radius_at(z) + 0.025
        yaw = angle + math.pi / 2.0
        tower.visual(
            Box((0.36, 0.045, 0.78)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), z),
                rpy=(0.0, 0.0, yaw),
            ),
            material=shadow_black,
            name=f"arched_window_{index}",
        )
        tower.visual(
            Box((0.46, 0.055, 0.10)),
            origin=Origin(
                xyz=((r + 0.006) * math.cos(angle), (r + 0.006) * math.sin(angle), z + 0.44),
                rpy=(0.0, 0.0, yaw),
            ),
            material=red_brick,
            name=f"window_lintel_{index}",
        )

    # Gallery platform and railing around the service level.
    tower.visual(
        Cylinder(radius=1.84, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 17.27)),
        material=warm_stone,
        name="gallery_deck",
    )
    tower.visual(
        Cylinder(radius=1.18, length=1.18),
        origin=Origin(xyz=(0.0, 0.0, 17.81)),
        material=white_masonry,
        name="gallery_wall",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(1.18, 0.040, radial_segments=96, tubular_segments=12), "gallery_wall_cap"),
        origin=Origin(xyz=(0.0, 0.0, 18.40)),
        material=red_brick,
        name="gallery_wall_cap",
    )

    for index in range(12):
        angle = index * 2.0 * math.pi / 12.0
        tower.visual(
            Cylinder(radius=0.030, length=0.72),
            origin=Origin(xyz=(1.72 * math.cos(angle), 1.72 * math.sin(angle), 17.72)),
            material=dark_metal,
            name=f"rail_post_{index}",
        )
    tower.visual(
        mesh_from_geometry(TorusGeometry(1.72, 0.030, radial_segments=96, tubular_segments=10), "gallery_top_rail"),
        origin=Origin(xyz=(0.0, 0.0, 18.08)),
        material=dark_metal,
        name="gallery_top_rail",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(1.72, 0.020, radial_segments=96, tubular_segments=8), "gallery_mid_rail"),
        origin=Origin(xyz=(0.0, 0.0, 17.78)),
        material=dark_metal,
        name="gallery_mid_rail",
    )

    # Front service doorway trim; the moving door sits just outside this recess.
    tower.visual(
        Box((0.58, 0.020, 1.02)),
        origin=Origin(xyz=(0.0, -1.215, 17.86)),
        material=shadow_black,
        name="door_recess",
    )
    tower.visual(
        Box((0.070, 0.030, 1.12)),
        origin=Origin(xyz=(-0.325, -1.220, 17.86)),
        material=red_brick,
        name="door_jamb_0",
    )
    tower.visual(
        Box((0.070, 0.030, 1.12)),
        origin=Origin(xyz=(0.325, -1.220, 17.86)),
        material=red_brick,
        name="door_jamb_1",
    )
    tower.visual(
        Box((0.72, 0.030, 0.080)),
        origin=Origin(xyz=(0.0, -1.220, 18.44)),
        material=red_brick,
        name="door_lintel",
    )

    # Glazed lantern room: transparent cylindrical glazing held by metal mullions.
    tower.visual(
        _lathed_shell(
            [(1.02, 18.38), (1.02, 19.92)],
            [(0.96, 18.42), (0.96, 19.88)],
            "lantern_glass_shell",
        ),
        material=lantern_glass,
        name="lantern_glass",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(1.02, 0.045, radial_segments=96, tubular_segments=12), "lantern_lower_ring"),
        origin=Origin(xyz=(0.0, 0.0, 18.40)),
        material=dark_metal,
        name="lantern_lower_ring",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(1.02, 0.045, radial_segments=96, tubular_segments=12), "lantern_upper_ring"),
        origin=Origin(xyz=(0.0, 0.0, 19.92)),
        material=dark_metal,
        name="lantern_upper_ring",
    )
    for index in range(8):
        angle = index * 2.0 * math.pi / 8.0
        tower.visual(
            Cylinder(radius=0.030, length=1.52),
            origin=Origin(xyz=(1.025 * math.cos(angle), 1.025 * math.sin(angle), 19.16)),
            material=dark_metal,
            name=f"lantern_mullion_{index}",
        )

    tower.visual(
        mesh_from_geometry(DomeGeometry(1.14, radial_segments=96, height_segments=16, closed=True), "domed_roof"),
        origin=Origin(xyz=(0.0, 0.0, 19.92)),
        material=weathered_copper,
        name="domed_roof",
    )
    tower.visual(
        Cylinder(radius=0.11, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 21.22)),
        material=dark_metal,
        name="roof_finial_stem",
    )
    tower.visual(
        Sphere(radius=0.16),
        origin=Origin(xyz=(0.0, 0.0, 21.46)),
        material=weathered_copper,
        name="roof_finial_ball",
    )

    # Stationary pedestal spindle that captures the rotating beacon drum.
    tower.visual(
        Cylinder(radius=0.30, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 18.53)),
        material=dark_metal,
        name="pedestal_base",
    )
    tower.visual(
        Cylinder(radius=0.070, length=1.25),
        origin=Origin(xyz=(0.0, 0.0, 19.25)),
        material=brass,
        name="spindle",
    )
    tower.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 18.80)),
        material=brass,
        name="lower_retainer",
    )
    tower.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 19.70)),
        material=brass,
        name="upper_retainer",
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.50, 0.060, 1.00)),
        origin=Origin(xyz=(0.25, 0.0, 0.50)),
        material=door_green,
        name="door_panel",
    )
    service_door.visual(
        Cylinder(radius=0.032, length=1.04),
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        material=dark_metal,
        name="hinge_barrel",
    )
    service_door.visual(
        Box((0.42, 0.016, 0.045)),
        origin=Origin(xyz=(0.22, -0.038, 0.26)),
        material=dark_metal,
        name="strap_0",
    )
    service_door.visual(
        Box((0.42, 0.016, 0.045)),
        origin=Origin(xyz=(0.22, -0.038, 0.76)),
        material=dark_metal,
        name="strap_1",
    )
    service_door.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.43, -0.045, 0.50), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="door_pull",
    )

    model.articulation(
        "tower_to_service_door",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=service_door,
        origin=Origin(xyz=(-0.25, -1.265, 17.415)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.0, lower=0.0, upper=1.05),
    )

    beacon = model.part("beacon")
    beacon.visual(
        _lathed_shell(
            [(0.58, 0.58), (0.58, 1.22)],
            [(0.42, 0.62), (0.42, 1.18)],
            "rotating_beacon_drum",
            segments=96,
        ),
        material=beacon_glass,
        name="beacon_drum",
    )
    beacon.visual(
        mesh_from_geometry(TorusGeometry(0.50, 0.045, radial_segments=96, tubular_segments=14), "beacon_bottom_band"),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=brass,
        name="bottom_band",
    )
    beacon.visual(
        mesh_from_geometry(TorusGeometry(0.50, 0.045, radial_segments=96, tubular_segments=14), "beacon_top_band"),
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        material=brass,
        name="top_band",
    )
    beacon.visual(
        mesh_from_geometry(TorusGeometry(0.090, 0.025, radial_segments=64, tubular_segments=10), "lower_beacon_clip"),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=brass,
        name="lower_clip",
    )
    beacon.visual(
        mesh_from_geometry(TorusGeometry(0.090, 0.025, radial_segments=64, tubular_segments=10), "upper_beacon_clip"),
        origin=Origin(xyz=(0.0, 0.0, 1.25)),
        material=brass,
        name="upper_clip",
    )
    for index in range(3):
        angle = index * 2.0 * math.pi / 3.0
        beacon.visual(
            Cylinder(radius=0.016, length=0.70),
            origin=Origin(xyz=(0.112 * math.cos(angle), 0.112 * math.sin(angle), 0.90)),
            material=brass,
            name=f"clip_post_{index}",
        )
    for index, (x, y, yaw) in enumerate(((0.285, 0.0, 0.0), (-0.285, 0.0, 0.0), (0.0, 0.285, math.pi / 2), (0.0, -0.285, math.pi / 2))):
        beacon.visual(
            Cylinder(radius=0.026, length=0.36),
            origin=Origin(xyz=(x, y, 0.90), rpy=(0.0, math.pi / 2.0, yaw)),
            material=brass,
            name=f"spoke_{index}",
        )
    for index in range(6):
        angle = index * 2.0 * math.pi / 6.0
        beacon.visual(
            Cylinder(radius=0.020, length=0.62),
            origin=Origin(xyz=(0.585 * math.cos(angle), 0.585 * math.sin(angle), 0.90)),
            material=brass,
            name=f"beacon_rib_{index}",
        )
    beacon.visual(
        Box((0.22, 0.16, 0.36)),
        origin=Origin(xyz=(0.66, 0.0, 0.91)),
        material=beacon_glow,
        name="beam_lens",
    )
    beacon.visual(
        Box((0.13, 0.14, 0.30)),
        origin=Origin(xyz=(-0.52, 0.0, 0.91)),
        material=dark_metal,
        name="counterweight",
    )

    model.articulation(
        "tower_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 18.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    service_door = object_model.get_part("service_door")
    beacon_joint = object_model.get_articulation("tower_to_beacon")
    door_joint = object_model.get_articulation("tower_to_service_door")

    ctx.allow_overlap(
        beacon,
        tower,
        elem_a="lower_clip",
        elem_b="spindle",
        reason="The lower rotating clip is intentionally modeled as a lightly sprung bearing grip around the fixed spindle.",
    )
    ctx.allow_overlap(
        beacon,
        tower,
        elem_a="upper_clip",
        elem_b="spindle",
        reason="The upper rotating clip is intentionally modeled as a lightly sprung bearing grip around the fixed spindle.",
    )

    ctx.check(
        "beacon uses continuous rotation",
        beacon_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"beacon joint type is {beacon_joint.articulation_type}",
    )
    ctx.check(
        "service door has side hinge limits",
        door_joint.motion_limits.lower == 0.0 and door_joint.motion_limits.upper is not None and door_joint.motion_limits.upper > 0.9,
        details=f"door limits are {door_joint.motion_limits}",
    )

    ctx.expect_within(
        tower,
        beacon,
        axes="xy",
        inner_elem="spindle",
        outer_elem="lower_clip",
        margin=0.0,
        name="spindle centered through lower clip",
    )
    ctx.expect_within(
        tower,
        beacon,
        axes="xy",
        inner_elem="spindle",
        outer_elem="upper_clip",
        margin=0.0,
        name="spindle centered through upper clip",
    )
    ctx.expect_overlap(
        beacon,
        tower,
        axes="z",
        elem_a="lower_clip",
        elem_b="spindle",
        min_overlap=0.04,
        name="lower clip wraps the spindle height",
    )
    ctx.expect_overlap(
        beacon,
        tower,
        axes="z",
        elem_a="upper_clip",
        elem_b="spindle",
        min_overlap=0.04,
        name="upper clip wraps the spindle height",
    )
    ctx.expect_gap(
        tower,
        beacon,
        axis="z",
        positive_elem="upper_retainer",
        negative_elem="upper_clip",
        min_gap=0.015,
        max_gap=0.060,
        name="upper retainer captures beacon clip",
    )
    ctx.expect_gap(
        beacon,
        tower,
        axis="z",
        positive_elem="lower_clip",
        negative_elem="lower_retainer",
        min_gap=0.015,
        max_gap=0.060,
        name="lower retainer captures beacon clip",
    )
    ctx.expect_gap(
        tower,
        service_door,
        axis="y",
        positive_elem="door_recess",
        negative_elem="door_panel",
        min_gap=0.004,
        max_gap=0.035,
        name="closed service door sits just outside recess",
    )

    def _component(value, index: int) -> float:
        try:
            return float(value[index])
        except TypeError:
            return float((value.x, value.y, value.z)[index])

    closed_door_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")
    closed_lens_aabb = ctx.part_element_world_aabb(beacon, elem="beam_lens")
    with ctx.pose({door_joint: 1.05, beacon_joint: math.pi / 2.0}):
        open_door_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")
        rotated_lens_aabb = ctx.part_element_world_aabb(beacon, elem="beam_lens")
        ctx.expect_within(
            tower,
            beacon,
            axes="xy",
            inner_elem="spindle",
            outer_elem="lower_clip",
            margin=0.0,
            name="rotated beacon remains clipped to spindle",
        )

    closed_min_y = _component(closed_door_aabb[0], 1) if closed_door_aabb else None
    open_min_y = _component(open_door_aabb[0], 1) if open_door_aabb else None
    ctx.check(
        "service door swings outward from gallery",
        closed_min_y is not None and open_min_y is not None and open_min_y < closed_min_y - 0.25,
        details=f"closed_min_y={closed_min_y}, open_min_y={open_min_y}",
    )

    closed_lens_x = _component(closed_lens_aabb[1], 0) if closed_lens_aabb else None
    rotated_lens_y = _component(rotated_lens_aabb[1], 1) if rotated_lens_aabb else None
    ctx.check(
        "beacon lens rotates around vertical spindle",
        closed_lens_x is not None and rotated_lens_y is not None and rotated_lens_y > closed_lens_x - 0.04,
        details=f"closed_lens_x={closed_lens_x}, rotated_lens_y={rotated_lens_y}",
    )

    return ctx.report()


object_model = build_object_model()
