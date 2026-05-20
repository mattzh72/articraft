from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


SHAFT_RADIUS = 1.0
CLOCK_Z = 8.55
CLOCK_FACE_CENTER_Y = 1.110
CLOCK_FACE_FRONT_Y = CLOCK_FACE_CENTER_Y + 0.03
CLOCK_JOINT_Y = 1.190
CLOCK_TICK_NAMES = (
    "clock_tick_0",
    "clock_tick_1",
    "clock_tick_2",
    "clock_tick_3",
    "clock_tick_4",
    "clock_tick_5",
    "clock_tick_6",
    "clock_tick_7",
    "clock_tick_8",
    "clock_tick_9",
    "clock_tick_10",
    "clock_tick_11",
)


def _cylinder_mesh(radius: float, height: float, name: str, segments: int = 48):
    return mesh_from_geometry(
        CylinderGeometry(radius, height, radial_segments=segments),
        name,
    )


def _hand_mesh(name: str, length: float, width: float, counter: float):
    """Flat tapered hand in the local XZ clock plane, centered on the spindle."""
    profile = [
        (-width * 0.55, -counter),
        (width * 0.55, -counter),
        (width * 0.35, length * 0.55),
        (width, length * 0.78),
        (0.0, length),
        (-width, length * 0.78),
        (-width * 0.35, length * 0.55),
    ]
    geom = ExtrudeGeometry.centered(profile, 0.024, cap=True, closed=True)
    # ExtrudeGeometry's thickness is local Z. Rotate it so thickness becomes
    # local Y and the pointer length lies along local +Z.
    geom.rotate_x(pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lighthouse_clock_tower")

    model.material("warm_limestone", rgba=(0.73, 0.68, 0.57, 1.0))
    model.material("pale_masonry", rgba=(0.86, 0.83, 0.72, 1.0))
    model.material("shadow_mortar", rgba=(0.46, 0.45, 0.40, 1.0))
    model.material("dark_bronze", rgba=(0.10, 0.09, 0.075, 1.0))
    model.material("glass_blue", rgba=(0.45, 0.72, 0.92, 0.45))
    model.material("clock_enamel", rgba=(0.96, 0.94, 0.82, 1.0))
    model.material("hand_black", rgba=(0.02, 0.018, 0.015, 1.0))
    model.material("roof_copper", rgba=(0.42, 0.16, 0.09, 1.0))

    tower = model.part("tower")

    # Wide stepped octagonal masonry base.
    tower.visual(
        _cylinder_mesh(2.80, 0.70, "octagonal_plinth", segments=8),
        origin=Origin(xyz=(0.0, 0.0, 0.35), rpy=(0.0, 0.0, pi / 8.0)),
        material="warm_limestone",
        name="octagonal_plinth",
    )
    tower.visual(
        _cylinder_mesh(2.30, 0.45, "octagonal_step", segments=8),
        origin=Origin(xyz=(0.0, 0.0, 0.925), rpy=(0.0, 0.0, pi / 8.0)),
        material="pale_masonry",
        name="octagonal_step",
    )
    tower.visual(
        _cylinder_mesh(1.72, 0.55, "octagonal_pedestal", segments=8),
        origin=Origin(xyz=(0.0, 0.0, 1.425), rpy=(0.0, 0.0, pi / 8.0)),
        material="warm_limestone",
        name="octagonal_pedestal",
    )

    # Tall cylindrical masonry shaft.
    tower.visual(
        Cylinder(radius=SHAFT_RADIUS, length=7.90),
        origin=Origin(xyz=(0.0, 0.0, 5.65)),
        material="pale_masonry",
        name="shaft_wall",
    )

    # Slight raised horizontal courses so the shaft reads as masonry.
    for i, z in enumerate([2.15, 2.85, 3.55, 4.25, 4.95, 5.65, 6.35, 7.05, 7.75, 9.35]):
        tower.visual(
            Cylinder(radius=1.018, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="shadow_mortar",
            name=f"masonry_course_{i}",
        )

    # Upper band carrying the clock face.
    tower.visual(
        Cylinder(radius=1.08, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, CLOCK_Z)),
        material="warm_limestone",
        name="upper_shaft_band",
    )
    tower.visual(
        Cylinder(radius=0.72, length=0.06),
        origin=Origin(
            xyz=(0.0, CLOCK_FACE_CENTER_Y, CLOCK_Z),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="clock_enamel",
        name="clock_face",
    )
    tower.visual(
        Cylinder(radius=0.79, length=0.055),
        origin=Origin(
            xyz=(0.0, CLOCK_FACE_CENTER_Y - 0.006, CLOCK_Z),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="dark_bronze",
        name="clock_rim",
    )
    tower.visual(
        Cylinder(radius=0.06, length=0.04),
        origin=Origin(
            xyz=(0.0, CLOCK_FACE_FRONT_Y + 0.010, CLOCK_Z),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="dark_bronze",
        name="fixed_spindle_boss",
    )

    # Raised hour ticks on the clock face.
    for hour in range(12):
        theta = hour * pi / 6.0
        marker_len = 0.14 if hour % 3 == 0 else 0.09
        marker_width = 0.035 if hour % 3 == 0 else 0.025
        r = 0.59
        tower.visual(
            Box((marker_width, 0.040, marker_len)),
            origin=Origin(
                xyz=(r * sin(theta), CLOCK_FACE_FRONT_Y + 0.020, CLOCK_Z + r * cos(theta)),
                rpy=(0.0, theta, 0.0),
            ),
            material="dark_bronze",
            name=CLOCK_TICK_NAMES[hour],
        )

    # Gallery deck and lantern room with glazed octagonal panels.
    tower.visual(
        _cylinder_mesh(1.68, 0.22, "gallery_deck", segments=8),
        origin=Origin(xyz=(0.0, 0.0, 9.71), rpy=(0.0, 0.0, pi / 8.0)),
        material="dark_bronze",
        name="gallery_deck",
    )
    tower.visual(
        Cylinder(radius=1.16, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 9.95)),
        material="dark_bronze",
        name="lantern_sill",
    )
    tower.visual(
        Cylinder(radius=1.14, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 11.23)),
        material="dark_bronze",
        name="lantern_head_ring",
    )

    glass_apothem = 1.055
    panel_width = 0.86
    for i in range(8):
        theta = i * pi / 4.0
        yaw = theta - pi / 2.0
        tower.visual(
            Box((panel_width, 0.035, 1.08)),
            origin=Origin(
                xyz=(glass_apothem * cos(theta), glass_apothem * sin(theta), 10.60),
                rpy=(0.0, 0.0, yaw),
            ),
            material="glass_blue",
            name=f"glazed_panel_{i}",
        )

    post_radius = glass_apothem / cos(pi / 8.0)
    for i in range(8):
        theta = pi / 8.0 + i * pi / 4.0
        tower.visual(
            Cylinder(radius=0.048, length=1.42),
            origin=Origin(xyz=(post_radius * cos(theta), post_radius * sin(theta), 10.60)),
            material="dark_bronze",
            name=f"lantern_mullion_{i}",
        )

    tower.visual(
        mesh_from_geometry(TorusGeometry(radius=1.48, tube=0.025), "gallery_upper_rail"),
        origin=Origin(xyz=(0.0, 0.0, 10.38)),
        material="dark_bronze",
        name="gallery_upper_rail",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(radius=1.48, tube=0.020), "gallery_mid_rail"),
        origin=Origin(xyz=(0.0, 0.0, 10.08)),
        material="dark_bronze",
        name="gallery_mid_rail",
    )
    for i in range(8):
        theta = i * pi / 4.0
        tower.visual(
            Cylinder(radius=0.032, length=0.68),
            origin=Origin(xyz=(1.48 * cos(theta), 1.48 * sin(theta), 10.06)),
            material="dark_bronze",
            name=f"gallery_post_{i}",
        )

    tower.visual(
        mesh_from_geometry(
            ConeGeometry(1.23, 0.90, radial_segments=48),
            "lantern_roof",
        ),
        origin=Origin(xyz=(0.0, 0.0, 11.78)),
        material="roof_copper",
        name="lantern_roof",
    )
    tower.visual(
        Sphere(radius=0.11),
        origin=Origin(xyz=(0.0, 0.0, 12.28)),
        material="dark_bronze",
        name="roof_finial",
    )
    tower.visual(
        Cylinder(radius=0.025, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 12.62)),
        material="dark_bronze",
        name="lightning_rod",
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        _hand_mesh("hour_pointer_mesh", length=0.40, width=0.055, counter=0.10),
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        material="hand_black",
        name="hour_pointer",
    )
    hour_hand.visual(
        Cylinder(radius=0.070, length=0.034),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_bronze",
        name="hour_hub",
    )
    hour_hand.visual(
        Cylinder(radius=0.045, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_bronze",
        name="hour_collet",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        _hand_mesh("minute_pointer_mesh", length=0.58, width=0.033, counter=0.13),
        origin=Origin(xyz=(0.0, 0.064, 0.0)),
        material="hand_black",
        name="minute_pointer",
    )
    minute_hand.visual(
        Cylinder(radius=0.052, length=0.034),
        origin=Origin(xyz=(0.0, 0.052, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_bronze",
        name="minute_hub",
    )

    for child, limits in (
        (hour_hand, MotionLimits(effort=0.3, velocity=0.25, lower=0.0, upper=2.0 * pi)),
        (minute_hand, MotionLimits(effort=0.2, velocity=1.0, lower=0.0, upper=2.0 * pi)),
    ):
        model.articulation(
            f"tower_to_{child.name}",
            ArticulationType.REVOLUTE,
            parent=tower,
            child=child,
            origin=Origin(xyz=(0.0, CLOCK_JOINT_Y, CLOCK_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    hour = object_model.get_part("hour_hand")
    minute = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("tower_to_hour_hand")
    minute_joint = object_model.get_articulation("tower_to_minute_hand")

    ctx.expect_gap(
        hour,
        tower,
        axis="y",
        positive_elem="hour_pointer",
        negative_elem="clock_tick_0",
        min_gap=0.005,
        name="hour hand clears raised clock ticks",
    )
    ctx.expect_gap(
        minute,
        hour,
        axis="y",
        positive_elem="minute_pointer",
        negative_elem="hour_pointer",
        min_gap=0.006,
        name="concentric hands are layered without collision",
    )
    ctx.expect_within(
        hour,
        tower,
        axes="xz",
        inner_elem="hour_pointer",
        outer_elem="clock_face",
        margin=0.0,
        name="hour hand stays within clock face",
    )
    ctx.expect_within(
        minute,
        tower,
        axes="xz",
        inner_elem="minute_pointer",
        outer_elem="clock_face",
        margin=0.0,
        name="minute hand stays within clock face",
    )

    hour_pos = ctx.part_world_position(hour)
    minute_pos = ctx.part_world_position(minute)
    ctx.check(
        "clock hand pivots are concentric",
        hour_pos is not None
        and minute_pos is not None
        and max(abs(a - b) for a, b in zip(hour_pos, minute_pos)) < 1e-6,
        details=f"hour={hour_pos}, minute={minute_pos}",
    )

    with ctx.pose({minute_joint: pi / 2.0, hour_joint: pi / 6.0}):
        minute_aabb = ctx.part_element_world_aabb(minute, elem="minute_pointer")
        hour_aabb = ctx.part_element_world_aabb(hour, elem="hour_pointer")
        ctx.check(
            "minute hand rotates about face center",
            minute_aabb is not None and minute_aabb[1][0] > 0.50 and minute_aabb[1][2] < CLOCK_Z + 0.16,
            details=f"minute_aabb={minute_aabb}",
        )
        ctx.check(
            "hour hand has independent revolute motion",
            hour_aabb is not None and hour_aabb[1][0] > 0.16 and hour_aabb[1][2] > CLOCK_Z + 0.28,
            details=f"hour_aabb={hour_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
