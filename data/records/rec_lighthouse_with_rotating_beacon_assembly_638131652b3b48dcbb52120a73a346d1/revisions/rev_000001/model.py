from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offshore_lighthouse")

    weathered_concrete = model.material("weathered_concrete", rgba=(0.73, 0.74, 0.72, 1.0))
    tower_white = model.material("tower_white", rgba=(0.94, 0.95, 0.93, 1.0))
    gallery_steel = model.material("gallery_steel", rgba=(0.46, 0.49, 0.52, 1.0))
    lantern_metal = model.material("lantern_metal", rgba=(0.36, 0.39, 0.41, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.70, 0.84, 0.90, 0.38))
    roof_red = model.material("roof_red", rgba=(0.63, 0.12, 0.11, 1.0))
    beacon_dark = model.material("beacon_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    beacon_lens = model.material("beacon_lens", rgba=(0.82, 0.28, 0.18, 0.72))

    tower_shell_mesh = _save_mesh(
        "tower_shell",
        LatheGeometry.from_shell_profiles(
            [
                (2.05, 0.00),
                (2.00, 1.30),
                (1.86, 4.60),
                (1.66, 8.90),
                (1.53, 11.80),
                (1.49, 12.30),
                (1.64, 12.60),
            ],
            [
                (1.76, 0.00),
                (1.71, 1.30),
                (1.58, 4.60),
                (1.42, 8.90),
                (1.30, 11.80),
                (1.26, 12.26),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )
    gallery_rail_mesh = _save_mesh(
        "gallery_rail",
        TorusGeometry(radius=2.20, tube=0.03, radial_segments=18, tubular_segments=72),
    )
    gallery_inner_collar_mesh = _save_mesh(
        "gallery_inner_collar",
        LatheGeometry.from_shell_profiles(
            [
                (1.70, 0.00),
                (1.70, 0.22),
                (1.66, 0.28),
            ],
            [
                (1.20, 0.00),
                (1.20, 0.22),
                (1.24, 0.28),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        ),
    )
    roof_shell_mesh = _save_mesh(
        "lantern_roof_shell",
        ConeGeometry(radius=1.18, height=0.78, radial_segments=48, closed=True).translate(
            0.0, 0.0, 0.39
        ),
    )

    caisson_base = model.part("caisson_base")
    caisson_base.visual(
        Cylinder(radius=4.60, length=2.60),
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
        material=weathered_concrete,
        name="main_caisson",
    )
    caisson_base.visual(
        Cylinder(radius=4.85, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 2.65)),
        material=gallery_steel,
        name="wave_fender_ring",
    )
    caisson_base.visual(
        Cylinder(radius=3.25, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 3.00)),
        material=weathered_concrete,
        name="service_deck",
    )
    caisson_base.visual(
        Cylinder(radius=2.45, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 3.575)),
        material=weathered_concrete,
        name="tower_seat",
    )

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=2.22, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=weathered_concrete,
        name="tower_footing",
    )
    tower.visual(
        tower_shell_mesh,
        material=tower_white,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=1.72, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 12.65)),
        material=tower_white,
        name="tower_cap",
    )
    tower.visual(
        Box((0.95, 0.10, 1.95)),
        origin=Origin(xyz=(0.0, -2.02, 1.08)),
        material=lantern_metal,
        name="entry_door",
    )
    tower.visual(
        Box((0.10, 0.48, 0.48)),
        origin=Origin(xyz=(1.56, 0.0, 6.00)),
        material=lantern_glass,
        name="window_starboard",
    )
    tower.visual(
        Box((0.10, 0.48, 0.48)),
        origin=Origin(xyz=(-1.64, 0.0, 8.30)),
        material=lantern_glass,
        name="window_port",
    )
    tower.visual(
        Box((0.48, 0.10, 0.48)),
        origin=Origin(xyz=(0.0, 1.56, 10.20)),
        material=lantern_glass,
        name="window_seaward",
    )

    gallery = model.part("gallery")
    gallery.visual(
        Cylinder(radius=2.24, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=weathered_concrete,
        name="gallery_floor",
    )
    gallery.visual(
        gallery_inner_collar_mesh,
        material=tower_white,
        name="gallery_inner_collar",
    )
    gallery.visual(
        gallery_rail_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        material=gallery_steel,
        name="mid_rail",
    )
    gallery.visual(
        gallery_rail_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        material=gallery_steel,
        name="top_rail",
    )
    for index in range(12):
        angle = math.tau * index / 12.0
        x = 2.20 * math.cos(angle)
        y = 2.20 * math.sin(angle)
        gallery.visual(
            Cylinder(radius=0.035, length=1.04),
            origin=Origin(xyz=(x, y, 0.56)),
            material=gallery_steel,
            name=f"rail_post_{index:02d}",
        )
    for index in range(6):
        angle = math.tau * index / 6.0 + math.pi / 6.0
        inner = (1.30 * math.cos(angle), 1.30 * math.sin(angle), 0.06)
        outer = (2.02 * math.cos(angle), 2.02 * math.sin(angle), 0.42)
        _add_member(
            gallery,
            inner,
            outer,
            radius=0.055,
            material=gallery_steel,
            name=f"gallery_brace_{index:02d}",
        )

    lantern_room = model.part("lantern_room")
    lantern_room.visual(
        Cylinder(radius=1.12, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=lantern_metal,
        name="base_sill",
    )
    lantern_room.visual(
        Cylinder(radius=1.10, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 1.93)),
        material=lantern_metal,
        name="cap_ring",
    )
    for index in range(8):
        angle = math.tau * index / 8.0
        side_x = 0.92 * math.cos(angle)
        side_y = 0.92 * math.sin(angle)
        lantern_room.visual(
            Box((0.78, 0.045, 1.60)),
            origin=Origin(xyz=(side_x, side_y, 1.04), rpy=(0.0, 0.0, angle - math.pi / 2.0)),
            material=lantern_glass,
            name=f"pane_{index:02d}",
        )
        post_angle = angle + (math.pi / 8.0)
        post_x = 1.04 * math.cos(post_angle)
        post_y = 1.04 * math.sin(post_angle)
        lantern_room.visual(
            Cylinder(radius=0.04, length=1.72),
            origin=Origin(xyz=(post_x, post_y, 1.05)),
            material=lantern_metal,
            name=f"mullion_{index:02d}",
        )

    roof = model.part("roof")
    roof.visual(
        Cylinder(radius=1.22, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=roof_red,
        name="eave_ring",
    )
    roof.visual(
        roof_shell_mesh,
        material=roof_red,
        name="roof_shell",
    )
    roof.visual(
        Cylinder(radius=0.18, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=lantern_metal,
        name="roof_vent",
    )
    roof.visual(
        Cylinder(radius=0.28, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        material=lantern_metal,
        name="pedestal_cap",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.28, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=gallery_steel,
        name="pedestal_base_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.12, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=gallery_steel,
        name="pedestal_stem",
    )
    pedestal.visual(
        Cylinder(radius=0.30, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=gallery_steel,
        name="turntable_drum",
    )
    for index in range(3):
        angle = math.tau * index / 3.0
        _add_member(
            pedestal,
            (0.16 * math.cos(angle), 0.16 * math.sin(angle), 0.10),
            (0.24 * math.cos(angle), 0.24 * math.sin(angle), 0.39),
            radius=0.028,
            material=gallery_steel,
            name=f"pedestal_brace_{index:02d}",
        )

    beacon_head = model.part("beacon_head")
    beacon_head.visual(
        Cylinder(radius=0.26, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=beacon_dark,
        name="turntable_collar",
    )
    beacon_head.visual(
        Cylinder(radius=0.035, length=0.30),
        origin=Origin(xyz=(0.18, 0.0, 0.23)),
        material=beacon_dark,
        name="support_post_right",
    )
    beacon_head.visual(
        Cylinder(radius=0.035, length=0.30),
        origin=Origin(xyz=(-0.18, 0.0, 0.23)),
        material=beacon_dark,
        name="support_post_left",
    )
    beacon_head.visual(
        Box((1.00, 0.42, 0.44)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=beacon_dark,
        name="housing_body",
    )
    beacon_head.visual(
        Cylinder(radius=0.21, length=0.28),
        origin=Origin(xyz=(0.64, 0.0, 0.42), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=beacon_lens,
        name="lens_drum_east",
    )
    beacon_head.visual(
        Cylinder(radius=0.21, length=0.28),
        origin=Origin(xyz=(-0.64, 0.0, 0.42), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=beacon_lens,
        name="lens_drum_west",
    )
    beacon_head.visual(
        Box((1.18, 0.52, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        material=beacon_dark,
        name="housing_hood",
    )

    model.articulation(
        "caisson_to_tower",
        ArticulationType.FIXED,
        parent=caisson_base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, 3.75)),
    )
    model.articulation(
        "tower_to_gallery",
        ArticulationType.FIXED,
        parent=tower,
        child=gallery,
        origin=Origin(xyz=(0.0, 0.0, 12.70)),
    )
    model.articulation(
        "gallery_to_lantern",
        ArticulationType.FIXED,
        parent=gallery,
        child=lantern_room,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )
    model.articulation(
        "lantern_to_roof",
        ArticulationType.FIXED,
        parent=lantern_room,
        child=roof,
        origin=Origin(xyz=(0.0, 0.0, 2.02)),
    )
    model.articulation(
        "roof_to_pedestal",
        ArticulationType.FIXED,
        parent=roof,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
    )
    model.articulation(
        "pedestal_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=beacon_head,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=1.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    caisson_base = object_model.get_part("caisson_base")
    tower = object_model.get_part("tower")
    gallery = object_model.get_part("gallery")
    lantern_room = object_model.get_part("lantern_room")
    roof = object_model.get_part("roof")
    pedestal = object_model.get_part("pedestal")
    beacon_head = object_model.get_part("beacon_head")
    beacon_spin = object_model.get_articulation("pedestal_to_beacon")

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

    ctx.expect_contact(tower, caisson_base)
    ctx.expect_contact(gallery, tower)
    ctx.expect_contact(lantern_room, gallery)
    ctx.expect_contact(roof, lantern_room)
    ctx.expect_contact(pedestal, roof)
    ctx.expect_contact(beacon_head, pedestal)

    ctx.expect_within(lantern_room, gallery, axes="xy", margin=0.0)
    ctx.expect_gap(
        beacon_head,
        roof,
        axis="z",
        positive_elem="housing_body",
        min_gap=0.18,
        name="beacon_housing_sits_above_roofline",
    )

    limits = beacon_spin.motion_limits
    ctx.check(
        "beacon_joint_is_continuous_vertical",
        beacon_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(beacon_spin.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={beacon_spin.articulation_type}, axis={beacon_spin.axis}, "
            f"limits=({None if limits is None else limits.lower}, "
            f"{None if limits is None else limits.upper})"
        ),
    )

    rest_aabb = ctx.part_world_aabb(beacon_head)
    rest_pos = ctx.part_world_position(beacon_head)
    with ctx.pose({beacon_spin: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(beacon_head)
        turned_pos = ctx.part_world_position(beacon_head)
        ctx.expect_contact(beacon_head, pedestal)

    if rest_aabb is not None and turned_aabb is not None:
        rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
        rest_dy = rest_aabb[1][1] - rest_aabb[0][1]
        turned_dx = turned_aabb[1][0] - turned_aabb[0][0]
        turned_dy = turned_aabb[1][1] - turned_aabb[0][1]
        ctx.check(
            "beacon_rotation_changes_plan_footprint",
            rest_dx > rest_dy + 0.35 and turned_dy > turned_dx + 0.35,
            details=(
                f"rest=({rest_dx:.3f}, {rest_dy:.3f}), "
                f"turned=({turned_dx:.3f}, {turned_dy:.3f})"
            ),
        )

    if rest_pos is not None and turned_pos is not None:
        dx = abs(rest_pos[0] - turned_pos[0])
        dy = abs(rest_pos[1] - turned_pos[1])
        dz = abs(rest_pos[2] - turned_pos[2])
        ctx.check(
            "beacon_rotation_keeps_turntable_center_fixed",
            dx < 1e-6 and dy < 1e-6 and dz < 1e-6,
            details=f"center shift=({dx:.6f}, {dy:.6f}, {dz:.6f})",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
