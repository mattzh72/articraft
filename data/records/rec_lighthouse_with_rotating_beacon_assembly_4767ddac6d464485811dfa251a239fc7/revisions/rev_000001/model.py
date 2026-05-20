from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


TOWER_TOP_Z = 27.6
GALLERY_DECK_Z0 = 28.0
GALLERY_DECK_Z1 = 28.25
LANTERN_BASE_Z0 = 28.25
LANTERN_BASE_Z1 = 28.55
LANTERN_FLOOR_Z0 = 28.55
LANTERN_FLOOR_Z1 = 28.67
GLAZING_Z0 = 28.67
GLAZING_Z1 = 31.00
LANTERN_CAP_RING_Z0 = 31.00
LANTERN_CAP_RING_Z1 = 31.28
CAP_SKIRT_Z0 = 31.28
CAP_SKIRT_Z1 = 31.46
VENT_LOWER_RING_Z0 = 31.46
VENT_LOWER_RING_Z1 = 31.58
VENT_SLAT_Z0 = 31.58
VENT_SLAT_Z1 = 31.92
VENT_UPPER_RING_Z0 = 31.92
VENT_UPPER_RING_Z1 = 32.04
ROOF_Z0 = 32.04
ROOF_Z1 = 33.10


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _tower_outer_radius(z: float) -> float:
    t = min(max(z / TOWER_TOP_Z, 0.0), 1.0)
    return _lerp(3.05, 1.62, t)


def _shell_mesh(
    outer_bottom_radius: float,
    outer_top_radius: float,
    z0: float,
    z1: float,
    *,
    thickness: float,
    name: str,
):
    inner_bottom_radius = max(outer_bottom_radius - thickness, 0.05)
    inner_top_radius = max(outer_top_radius - thickness, 0.05)
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_bottom_radius, z0), (outer_top_radius, z1)],
            [(inner_bottom_radius, z0), (inner_top_radius, z1)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def _add_radial_box(
    part,
    *,
    size: tuple[float, float, float],
    radius: float,
    theta: float,
    z_center: float,
    material,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(
            xyz=(radius * cos(theta), radius * sin(theta), z_center),
            rpy=(0.0, 0.0, theta),
        ),
        material=material,
        name=name,
    )


def _add_vertical_post(
    part,
    *,
    xy: tuple[float, float],
    z_bottom: float,
    height: float,
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=height),
        origin=Origin(xyz=(xy[0], xy[1], z_bottom + height / 2.0)),
        material=material,
        name=name,
    )


def _add_xy_link_box(
    part,
    *,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    z_center: float,
    thickness: float,
    height: float,
    material,
    name: str,
) -> None:
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    length = (dx * dx + dy * dy) ** 0.5
    part.visual(
        Box((length, thickness, height)),
        origin=Origin(
            xyz=((start_xy[0] + end_xy[0]) / 2.0, (start_xy[1] + end_xy[1]) / 2.0, z_center),
            rpy=(0.0, 0.0, atan2(dy, dx)),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="striped_lighthouse")

    white = model.material("tower_white", rgba=(0.95, 0.95, 0.93, 1.0))
    red = model.material("tower_red", rgba=(0.74, 0.12, 0.12, 1.0))
    stone = model.material("stone", rgba=(0.73, 0.73, 0.70, 1.0))
    iron = model.material("iron", rgba=(0.19, 0.22, 0.25, 1.0))
    dark_roof = model.material("dark_roof", rgba=(0.10, 0.14, 0.16, 1.0))
    glass = model.material("glass", rgba=(0.80, 0.92, 0.98, 0.34))
    lens = model.material("lens", rgba=(0.92, 0.82, 0.48, 0.82))
    deck_paint = model.material("deck_paint", rgba=(0.40, 0.44, 0.46, 1.0))
    gate_paint = model.material("gate_paint", rgba=(0.61, 0.12, 0.10, 1.0))

    lighthouse = model.part("lighthouse")
    lighthouse.inertial = Inertial.from_geometry(
        Box((6.8, 6.8, ROOF_Z1)),
        mass=28000.0,
        origin=Origin(xyz=(0.0, 0.0, ROOF_Z1 / 2.0)),
    )

    lighthouse.visual(
        Cylinder(radius=3.30, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=stone,
        name="foundation_plinth",
    )

    stripe_breaks = [0.0, 4.7, 9.4, 14.1, 18.8, 23.5, 27.6]
    for index, (z0, z1) in enumerate(zip(stripe_breaks, stripe_breaks[1:])):
        band_material = white if index % 2 == 0 else red
        lighthouse.visual(
            _shell_mesh(
                _tower_outer_radius(z0),
                _tower_outer_radius(z1),
                z0,
                z1,
                thickness=0.24,
                name=f"tower_band_mesh_{index}",
            ),
            material=band_material,
            name=f"tower_band_{index}",
        )

    lighthouse.visual(
        _shell_mesh(
            _tower_outer_radius(TOWER_TOP_Z),
            2.05,
            TOWER_TOP_Z,
            GALLERY_DECK_Z0,
            thickness=0.22,
            name="gallery_corbel_mesh",
        ),
        material=stone,
        name="gallery_corbel",
    )
    lighthouse.visual(
        Cylinder(radius=2.32, length=GALLERY_DECK_Z1 - GALLERY_DECK_Z0),
        origin=Origin(xyz=(0.0, 0.0, (GALLERY_DECK_Z0 + GALLERY_DECK_Z1) / 2.0)),
        material=stone,
        name="gallery_deck",
    )
    lighthouse.visual(
        _shell_mesh(
            1.28,
            1.28,
            LANTERN_BASE_Z0,
            LANTERN_BASE_Z1,
            thickness=0.14,
            name="lantern_base_ring_mesh",
        ),
        material=iron,
        name="lantern_base_ring",
    )
    lighthouse.visual(
        Cylinder(radius=1.14, length=LANTERN_FLOOR_Z1 - LANTERN_FLOOR_Z0),
        origin=Origin(xyz=(0.0, 0.0, (LANTERN_FLOOR_Z0 + LANTERN_FLOOR_Z1) / 2.0)),
        material=deck_paint,
        name="lantern_floor",
    )

    gallery_post_height = 1.10
    gate_post_size = (0.07, 0.10, gallery_post_height)
    gate_post_z = GALLERY_DECK_Z1 + gallery_post_height / 2.0
    gate_hinge_post_xy = (2.10, -0.465)
    gate_latch_post_xy = (2.10, 0.465)
    for name, xy in (
        ("gate_hinge_post", gate_hinge_post_xy),
        ("gate_latch_post", gate_latch_post_xy),
    ):
        lighthouse.visual(
            Box(gate_post_size),
            origin=Origin(xyz=(xy[0], xy[1], gate_post_z)),
            material=iron,
            name=name,
        )

    arc_angles = [0.55, 1.03, 1.51, 1.99, 2.47, 2.95, -2.47, -1.99, -1.51, -1.03, -0.55]
    arc_posts: list[tuple[float, float]] = []
    for index, theta in enumerate(arc_angles):
        xy = (2.12 * cos(theta), 2.12 * sin(theta))
        arc_posts.append(xy)
        _add_vertical_post(
            lighthouse,
            xy=xy,
            z_bottom=GALLERY_DECK_Z1,
            height=gallery_post_height,
            radius=0.035,
            material=iron,
            name=f"gallery_post_{index}",
        )

    rail_path = [gate_latch_post_xy] + arc_posts[:6] + arc_posts[6:] + [gate_hinge_post_xy]
    for index, (start_xy, end_xy) in enumerate(zip(rail_path, rail_path[1:])):
        _add_xy_link_box(
            lighthouse,
            start_xy=start_xy,
            end_xy=end_xy,
            z_center=29.30,
            thickness=0.08,
            height=0.08,
            material=iron,
            name=f"gallery_top_rail_{index}",
        )
        _add_xy_link_box(
            lighthouse,
            start_xy=start_xy,
            end_xy=end_xy,
            z_center=28.63,
            thickness=0.05,
            height=0.05,
            material=iron,
            name=f"gallery_kick_rail_{index}",
        )

    lantern_post_angles = [index * (pi / 4.0) for index in range(8)]
    for index, theta in enumerate(lantern_post_angles):
        _add_vertical_post(
            lighthouse,
            xy=(1.18 * cos(theta), 1.18 * sin(theta)),
            z_bottom=LANTERN_BASE_Z1,
            height=GLAZING_Z1 - GLAZING_Z0,
            radius=0.045,
            material=iron,
            name=f"lantern_mullion_{index}",
        )

    pane_angles = [pi / 8.0 + index * (pi / 4.0) for index in range(8)]
    for index, theta in enumerate(pane_angles):
        _add_radial_box(
            lighthouse,
            size=(0.02, 0.78, GLAZING_Z1 - GLAZING_Z0),
            radius=1.10,
            theta=theta,
            z_center=(GLAZING_Z0 + GLAZING_Z1) / 2.0,
            material=glass,
            name=f"lantern_glass_{index}",
        )

    lighthouse.visual(
        _shell_mesh(
            1.24,
            1.24,
            LANTERN_CAP_RING_Z0,
            LANTERN_CAP_RING_Z1,
            thickness=0.12,
            name="lantern_cap_ring_mesh",
        ),
        material=iron,
        name="lantern_cap_ring",
    )
    lighthouse.visual(
        _shell_mesh(
            1.18,
            1.18,
            CAP_SKIRT_Z0,
            CAP_SKIRT_Z1,
            thickness=0.10,
            name="cap_skirt_mesh",
        ),
        material=dark_roof,
        name="cap_skirt",
    )
    lighthouse.visual(
        Cylinder(radius=1.10, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 31.46)),
        material=dark_roof,
        name="vent_base_plate",
    )
    lighthouse.visual(
        Cylinder(radius=0.92, length=VENT_LOWER_RING_Z1 - VENT_LOWER_RING_Z0),
        origin=Origin(xyz=(0.0, 0.0, (VENT_LOWER_RING_Z0 + VENT_LOWER_RING_Z1) / 2.0)),
        material=dark_roof,
        name="vent_lower_ring",
    )
    for index, theta in enumerate(lantern_post_angles):
        _add_radial_box(
            lighthouse,
            size=(0.05, 0.14, VENT_SLAT_Z1 - VENT_SLAT_Z0),
            radius=0.78,
            theta=theta,
            z_center=(VENT_SLAT_Z0 + VENT_SLAT_Z1) / 2.0,
            material=iron,
            name=f"vent_slat_{index}",
        )
    lighthouse.visual(
        Cylinder(radius=0.88, length=VENT_UPPER_RING_Z1 - VENT_UPPER_RING_Z0),
        origin=Origin(xyz=(0.0, 0.0, (VENT_UPPER_RING_Z0 + VENT_UPPER_RING_Z1) / 2.0)),
        material=dark_roof,
        name="vent_upper_ring",
    )
    lighthouse.visual(
        mesh_from_geometry(ConeGeometry(radius=1.24, height=ROOF_Z1 - ROOF_Z0, radial_segments=56, closed=True), "roof_cone_mesh"),
        origin=Origin(xyz=(0.0, 0.0, (ROOF_Z0 + ROOF_Z1) / 2.0)),
        material=dark_roof,
        name="roof_cone",
    )

    pedestal = model.part("pedestal")
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=1.55),
        mass=550.0,
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
    )
    pedestal.visual(
        Cylinder(radius=0.34, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=iron,
        name="pedestal_base_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.20, length=1.35),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material=iron,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.25, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        material=iron,
        name="pedestal_head",
    )

    beacon = model.part("beacon")
    beacon.inertial = Inertial.from_geometry(
        Box((0.86, 0.70, 0.94)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
    )
    beacon.visual(
        Cylinder(radius=0.22, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=iron,
        name="turntable",
    )
    beacon.visual(
        Cylinder(radius=0.04, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=iron,
        name="beacon_mast",
    )
    beacon.visual(
        Box((0.24, 0.22, 0.16)),
        origin=Origin(xyz=(-0.02, 0.0, 0.16)),
        material=dark_roof,
        name="drive_box",
    )
    beacon.visual(
        Box((0.56, 0.08, 0.08)),
        origin=Origin(xyz=(0.08, 0.0, 0.38)),
        material=iron,
        name="yoke_arm",
    )
    beacon.visual(
        Box((0.18, 0.34, 0.62)),
        origin=Origin(xyz=(0.30, 0.0, 0.44)),
        material=lens,
        name="lens_housing",
    )
    beacon.visual(
        Box((0.22, 0.36, 0.06)),
        origin=Origin(xyz=(0.30, 0.0, 0.78)),
        material=iron,
        name="lens_hood",
    )
    beacon.visual(
        Box((0.14, 0.20, 0.22)),
        origin=Origin(xyz=(-0.18, 0.0, 0.34)),
        material=dark_roof,
        name="counterweight",
    )

    trap_door = model.part("trap_door")
    trap_door.inertial = Inertial.from_geometry(
        Box((0.06, 0.82, 0.98)),
        mass=18.0,
        origin=Origin(xyz=(0.03, 0.41, 0.49)),
    )
    trap_door_thickness = 0.035
    trap_door_width = 0.81
    trap_door_height = 0.96
    trap_door.visual(
        Box((trap_door_thickness, trap_door_width, trap_door_height)),
        origin=Origin(
            xyz=(trap_door_thickness / 2.0, trap_door_width / 2.0, trap_door_height / 2.0)
        ),
        material=gate_paint,
        name="gate_leaf",
    )
    for name, z_center in (("hinge_lower", 0.10), ("hinge_upper", trap_door_height - 0.10)):
        trap_door.visual(
            Cylinder(radius=0.010, length=0.18),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=iron,
            name=name,
        )
    trap_door.visual(
        Box((0.04, 0.08, 0.12)),
        origin=Origin(xyz=(trap_door_thickness + 0.02, trap_door_width - 0.06, 0.52)),
        material=iron,
        name="gate_latch",
    )

    model.articulation(
        "lighthouse_to_pedestal",
        ArticulationType.FIXED,
        parent=lighthouse,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, LANTERN_FLOOR_Z1)),
    )
    model.articulation(
        "pedestal_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.4),
    )
    model.articulation(
        "gallery_trap_hinge",
        ArticulationType.REVOLUTE,
        parent=lighthouse,
        child=trap_door,
        origin=Origin(xyz=(2.10, -0.405, 28.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.8, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lighthouse = object_model.get_part("lighthouse")
    pedestal = object_model.get_part("pedestal")
    beacon = object_model.get_part("beacon")
    trap_door = object_model.get_part("trap_door")
    beacon_spin = object_model.get_articulation("pedestal_to_beacon")
    trap_hinge = object_model.get_articulation("gallery_trap_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "required_parts_present",
        all(part is not None for part in (lighthouse, pedestal, beacon, trap_door)),
        "Expected lighthouse, pedestal, beacon, and trap_door parts.",
    )
    ctx.check(
        "beacon_joint_is_vertical_continuous",
        beacon_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(beacon_spin.axis) == (0.0, 0.0, 1.0),
        f"Expected a continuous vertical beacon joint, got type={beacon_spin.articulation_type} axis={beacon_spin.axis}.",
    )
    ctx.check(
        "trap_hinge_is_vertical_revolute",
        trap_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(trap_hinge.axis) == (0.0, 0.0, 1.0)
        and trap_hinge.motion_limits is not None
        and trap_hinge.motion_limits.lower == 0.0
        and trap_hinge.motion_limits.upper is not None
        and trap_hinge.motion_limits.upper >= 1.2,
        (
            "Expected the gallery trap door to hinge about a vertical rail axis "
            f"with a usable opening range, got {trap_hinge.articulation_type} {trap_hinge.axis} "
            f"{trap_hinge.motion_limits}."
        ),
    )

    ctx.expect_contact(pedestal, lighthouse, name="pedestal_is_mounted_to_lantern_floor")
    ctx.expect_contact(beacon, pedestal, name="beacon_is_seated_on_pedestal")
    ctx.expect_contact(trap_door, lighthouse, name="trap_door_touches_gallery_hinge_post")

    ctx.expect_origin_distance(
        beacon,
        pedestal,
        axes="xy",
        max_dist=0.001,
        name="beacon_axis_matches_pedestal_axis",
    )
    ctx.expect_origin_gap(
        beacon,
        lighthouse,
        axis="z",
        min_gap=30.0,
        max_gap=30.3,
        name="beacon_is_raised_into_lantern_room",
    )
    ctx.expect_origin_distance(
        trap_door,
        lighthouse,
        axes="x",
        min_dist=2.0,
        max_dist=2.2,
        name="trap_door_sits_on_gallery_perimeter",
    )
    ctx.expect_origin_gap(
        trap_door,
        lighthouse,
        axis="z",
        min_gap=28.2,
        max_gap=28.4,
        name="trap_door_is_at_gallery_height",
    )

    with ctx.pose({beacon_spin: pi / 2.0}):
        ctx.expect_origin_distance(
            beacon,
            pedestal,
            axes="xy",
            max_dist=0.001,
            name="rotating_beacon_stays_centered_on_pedestal",
        )
        ctx.expect_contact(beacon, pedestal, name="rotating_beacon_remains_clipped_to_pedestal")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_beacon_quarter_turn")

    with ctx.pose({trap_hinge: 1.10}):
        ctx.expect_contact(trap_door, lighthouse, name="opened_trap_door_stays_hinged_to_gallery")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_trap_door_open")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
