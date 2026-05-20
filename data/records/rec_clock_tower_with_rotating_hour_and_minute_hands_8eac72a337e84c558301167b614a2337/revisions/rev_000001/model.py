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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _ring_shell_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    name: str,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, height)],
            [(inner_radius, 0.0), (inner_radius, height)],
            segments=72,
        ),
        name,
    )


def _flat_hand_mesh(
    *,
    length: float,
    root_half_width: float,
    mid_half_width: float,
    tip_half_width: float,
    back_length: float,
    thickness: float,
    name: str,
):
    profile = [
        (-root_half_width, -back_length),
        (root_half_width, -back_length),
        (mid_half_width, 0.0),
        (mid_half_width * 0.88, length * 0.62),
        (tip_half_width, length),
        (-tip_half_width, length),
        (-mid_half_width * 0.88, length * 0.62),
        (-mid_half_width, 0.0),
    ]
    return mesh_from_geometry(
        ExtrudeGeometry(profile, thickness, center=True).rotate_x(math.pi / 2.0),
        name,
    )


def _flat_tip_mesh(*, width: float, length: float, thickness: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            [
                (-width * 0.5, 0.0),
                (width * 0.5, 0.0),
                (0.0, length),
            ],
            thickness,
            center=True,
        ).rotate_x(math.pi / 2.0),
        name,
    )


def _world_aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lighthouse_clock_tower")

    masonry_white = model.material("masonry_white", rgba=(0.94, 0.95, 0.92, 1.0))
    stone_gray = model.material("stone_gray", rgba=(0.61, 0.61, 0.60, 1.0))
    iron_dark = model.material("iron_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    copper_roof = model.material("copper_roof", rgba=(0.34, 0.43, 0.36, 1.0))
    clock_face_white = model.material("clock_face_white", rgba=(0.96, 0.95, 0.90, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.61, 0.29, 1.0))
    glass = model.material("glass", rgba=(0.80, 0.90, 0.98, 0.35))
    lamp_glow = model.material("lamp_glow", rgba=(0.98, 0.88, 0.52, 0.75))

    tower = model.part("tower")
    tower.inertial = Inertial.from_geometry(
        Box((7.6, 7.6, 26.2)),
        mass=85000.0,
        origin=Origin(xyz=(0.0, 0.0, 13.1)),
    )

    base_radius = 3.0
    shaft_radius = 2.45
    gallery_outer_radius = 3.45
    gallery_inner_radius = 2.55
    gallery_z = 18.98
    gallery_height = 0.22
    clock_z = 10.8
    clock_joint_y = 2.58

    tower.visual(
        Cylinder(radius=base_radius, length=1.4),
        origin=Origin(xyz=(0.0, 0.0, 0.7)),
        material=stone_gray,
        name="base_plinth",
    )
    tower.visual(
        Cylinder(radius=2.72, length=0.65),
        origin=Origin(xyz=(0.0, 0.0, 1.72)),
        material=stone_gray,
        name="lower_collar",
    )
    tower.visual(
        Cylinder(radius=shaft_radius, length=17.8),
        origin=Origin(xyz=(0.0, 0.0, 10.3)),
        material=masonry_white,
        name="shaft_shell",
    )
    tower.visual(
        Cylinder(radius=2.62, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 18.85)),
        material=stone_gray,
        name="gallery_support_cornice",
    )

    gallery_floor = _ring_shell_mesh(
        outer_radius=gallery_outer_radius,
        inner_radius=gallery_inner_radius,
        height=gallery_height,
        name="gallery_floor_ring",
    )
    tower.visual(
        gallery_floor,
        origin=Origin(xyz=(0.0, 0.0, gallery_z)),
        material=stone_gray,
        name="gallery_floor_ring",
    )

    for index in range(12):
        angle = 2.0 * math.pi * index / 12.0
        bracket_radius = 2.95
        tower.visual(
            Box((1.10, 0.16, 0.18)),
            origin=Origin(
                xyz=(
                    math.cos(angle) * bracket_radius,
                    math.sin(angle) * bracket_radius,
                    18.94,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=stone_gray,
            name=f"gallery_bracket_{index}",
        )

    tower.visual(
        Cylinder(radius=1.80, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 19.42)),
        material=stone_gray,
        name="lantern_base",
    )

    lantern_glass = _ring_shell_mesh(
        outer_radius=1.55,
        inner_radius=1.48,
        height=3.00,
        name="lantern_glass_shell",
    )
    tower.visual(
        lantern_glass,
        origin=Origin(xyz=(0.0, 0.0, 19.65)),
        material=glass,
        name="lantern_glass_shell",
    )

    tower.visual(
        Cylinder(radius=1.62, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 19.73)),
        material=iron_dark,
        name="lantern_lower_ring",
    )
    tower.visual(
        Cylinder(radius=1.66, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 22.57)),
        material=iron_dark,
        name="lantern_upper_ring",
    )

    for index in range(8):
        angle = 2.0 * math.pi * index / 8.0
        mullion_radius = 1.51
        tower.visual(
            Cylinder(radius=0.045, length=2.92),
            origin=Origin(
                xyz=(
                    math.cos(angle) * mullion_radius,
                    math.sin(angle) * mullion_radius,
                    21.11,
                ),
            ),
            material=iron_dark,
            name=f"lantern_mullion_{index}",
        )

    tower.visual(
        Cylinder(radius=0.18, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 19.85)),
        material=iron_dark,
        name="lamp_pedestal",
    )
    tower.visual(
        Cylinder(radius=0.24, length=1.00),
        origin=Origin(xyz=(0.0, 0.0, 20.55)),
        material=lamp_glow,
        name="lamp_core",
    )
    tower.visual(
        Sphere(radius=0.28),
        origin=Origin(xyz=(0.0, 0.0, 21.20)),
        material=lamp_glow,
        name="lamp_lens",
    )

    lantern_roof = mesh_from_geometry(ConeGeometry(radius=1.80, height=2.40), "lantern_roof")
    tower.visual(
        lantern_roof,
        origin=Origin(xyz=(0.0, 0.0, 23.85)),
        material=copper_roof,
        name="lantern_roof",
    )
    tower.visual(
        Cylinder(radius=0.08, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 25.30)),
        material=brass,
        name="roof_finial_stem",
    )
    tower.visual(
        Sphere(radius=0.12),
        origin=Origin(xyz=(0.0, 0.0, 25.61)),
        material=brass,
        name="roof_finial_ball",
    )

    gallery_top_rail = mesh_from_geometry(TorusGeometry(radius=3.15, tube=0.05), "gallery_top_rail")
    gallery_mid_rail = mesh_from_geometry(TorusGeometry(radius=3.15, tube=0.035), "gallery_mid_rail")
    tower.visual(
        gallery_top_rail,
        origin=Origin(xyz=(0.0, 0.0, 20.20)),
        material=iron_dark,
        name="gallery_top_rail",
    )
    tower.visual(
        gallery_mid_rail,
        origin=Origin(xyz=(0.0, 0.0, 19.72)),
        material=iron_dark,
        name="gallery_mid_rail",
    )
    for index in range(16):
        angle = 2.0 * math.pi * index / 16.0
        tower.visual(
            Cylinder(radius=0.06, length=1.00),
            origin=Origin(
                xyz=(
                    math.cos(angle) * 3.15,
                    math.sin(angle) * 3.15,
                    19.70,
                ),
            ),
            material=iron_dark,
            name=f"gallery_post_{index}",
        )

    tower.visual(
        Cylinder(radius=1.68, length=0.12),
        origin=Origin(xyz=(0.0, 2.50, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clock_face_white,
        name="clock_dial",
    )
    tower.visual(
        Cylinder(radius=1.78, length=0.08),
        origin=Origin(xyz=(0.0, 2.53, clock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_dark,
        name="clock_bezel",
    )
    for index in range(12):
        angle = 2.0 * math.pi * index / 12.0
        marker_len = 0.38 if index % 3 == 0 else 0.24
        marker_width = 0.11 if index % 3 == 0 else 0.08
        marker_radius = 1.28
        tower.visual(
            Box((marker_width, 0.04, marker_len)),
            origin=Origin(
                xyz=(
                    math.sin(angle) * marker_radius,
                    2.57,
                    clock_z + math.cos(angle) * marker_radius,
                ),
                rpy=(0.0, angle, 0.0),
            ),
            material=iron_dark,
            name=f"clock_marker_{index}",
        )

    hour_hand = model.part("hour_hand")
    hour_blade_mesh = _flat_hand_mesh(
        length=0.70,
        root_half_width=0.090,
        mid_half_width=0.060,
        tip_half_width=0.020,
        back_length=0.12,
        thickness=0.024,
        name="hour_hand_blade",
    )
    hour_tip_mesh = _flat_tip_mesh(
        width=0.12,
        length=0.16,
        thickness=0.024,
        name="hour_hand_tip",
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.30, 0.05, 0.95)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )
    hour_hand.visual(
        hour_blade_mesh,
        origin=Origin(),
        material=iron_dark,
        name="hour_blade",
    )
    hour_hand.visual(
        hour_tip_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material=iron_dark,
        name="hour_tip",
    )
    hour_hand.visual(
        Cylinder(radius=0.055, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_dark,
        name="hour_counterweight",
    )
    hour_hand.visual(
        Cylinder(radius=0.16, length=0.044),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hour_hub",
    )

    minute_hand = model.part("minute_hand")
    minute_blade_mesh = _flat_hand_mesh(
        length=1.04,
        root_half_width=0.060,
        mid_half_width=0.040,
        tip_half_width=0.014,
        back_length=0.16,
        thickness=0.018,
        name="minute_hand_blade",
    )
    minute_tip_mesh = _flat_tip_mesh(
        width=0.08,
        length=0.20,
        thickness=0.018,
        name="minute_hand_tip",
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.22, 0.05, 1.45)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.04, 0.55)),
    )
    minute_hand.visual(
        minute_blade_mesh,
        origin=Origin(xyz=(0.0, 0.04, 0.0)),
        material=brass,
        name="minute_blade",
    )
    minute_hand.visual(
        minute_tip_mesh,
        origin=Origin(xyz=(0.0, 0.04, 1.04)),
        material=brass,
        name="minute_tip",
    )
    minute_hand.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.0, 0.04, -0.15), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="minute_counterweight",
    )
    minute_hand.visual(
        Cylinder(radius=0.11, length=0.022),
        origin=Origin(xyz=(0.0, 0.04, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="minute_hub",
    )

    model.articulation(
        "tower_to_hour_hand",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=hour_hand,
        origin=Origin(xyz=(0.0, clock_joint_y, clock_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=6.5),
    )
    model.articulation(
        "tower_to_minute_hand",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=minute_hand,
        origin=Origin(xyz=(0.0, clock_joint_y, clock_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    tower = object_model.get_part("tower")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("tower_to_hour_hand")
    minute_joint = object_model.get_articulation("tower_to_minute_hand")

    ctx.check(
        "clock hands use continuous coaxial joints",
        hour_joint.articulation_type == ArticulationType.CONTINUOUS
        and minute_joint.articulation_type == ArticulationType.CONTINUOUS
        and hour_joint.axis == minute_joint.axis == (0.0, 1.0, 0.0)
        and hour_joint.origin.xyz == minute_joint.origin.xyz,
        details=(
            f"hour_type={hour_joint.articulation_type}, minute_type={minute_joint.articulation_type}, "
            f"hour_axis={hour_joint.axis}, minute_axis={minute_joint.axis}, "
            f"hour_origin={hour_joint.origin.xyz}, minute_origin={minute_joint.origin.xyz}"
        ),
    )

    tower_aabb = ctx.part_world_aabb(tower)
    ctx.check(
        "tower proportions read as a lighthouse clock tower",
        tower_aabb is not None
        and (tower_aabb[1][2] - tower_aabb[0][2]) > 24.5
        and 6.0 < (tower_aabb[1][0] - tower_aabb[0][0]) < 8.5
        and 6.0 < (tower_aabb[1][1] - tower_aabb[0][1]) < 8.5,
        details=f"tower_aabb={tower_aabb}",
    )

    ctx.expect_gap(
        hour_hand,
        tower,
        axis="y",
        positive_elem="hour_blade",
        negative_elem="clock_dial",
        min_gap=0.004,
        max_gap=0.05,
        name="hour hand sits slightly proud of the clock face",
    )
    ctx.expect_gap(
        minute_hand,
        hour_hand,
        axis="y",
        positive_elem="minute_blade",
        negative_elem="hour_blade",
        min_gap=0.008,
        max_gap=0.08,
        name="minute hand clears the hour hand",
    )

    hour_rest = _world_aabb_center(ctx.part_element_world_aabb(hour_hand, elem="hour_tip"))
    minute_rest = _world_aabb_center(ctx.part_element_world_aabb(minute_hand, elem="minute_tip"))
    with ctx.pose({minute_joint: math.pi / 2.0}):
        minute_quarter = _world_aabb_center(ctx.part_element_world_aabb(minute_hand, elem="minute_tip"))
        hour_while_minute_moves = _world_aabb_center(ctx.part_element_world_aabb(hour_hand, elem="hour_tip"))
    with ctx.pose({hour_joint: math.pi / 2.0}):
        hour_quarter = _world_aabb_center(ctx.part_element_world_aabb(hour_hand, elem="hour_tip"))
        minute_while_hour_moves = _world_aabb_center(ctx.part_element_world_aabb(minute_hand, elem="minute_tip"))

    clock_center = hour_joint.origin.xyz
    ctx.check(
        "minute hand rotates clockwise around the dial without dragging the hour hand",
        minute_rest is not None
        and minute_quarter is not None
        and hour_rest is not None
        and hour_while_minute_moves is not None
        and minute_rest[2] > clock_center[2] + 0.8
        and minute_quarter[0] > clock_center[0] + 0.8
        and abs(minute_quarter[2] - clock_center[2]) < 0.25
        and abs(hour_while_minute_moves[0] - hour_rest[0]) < 0.05
        and abs(hour_while_minute_moves[2] - hour_rest[2]) < 0.05,
        details=(
            f"clock_center={clock_center}, minute_rest={minute_rest}, minute_quarter={minute_quarter}, "
            f"hour_rest={hour_rest}, hour_while_minute_moves={hour_while_minute_moves}"
        ),
    )
    ctx.check(
        "hour hand rotates on the same axis without dragging the minute hand",
        hour_rest is not None
        and hour_quarter is not None
        and minute_rest is not None
        and minute_while_hour_moves is not None
        and hour_rest[2] > clock_center[2] + 0.45
        and hour_quarter[0] > clock_center[0] + 0.45
        and abs(hour_quarter[2] - clock_center[2]) < 0.25
        and abs(minute_while_hour_moves[0] - minute_rest[0]) < 0.05
        and abs(minute_while_hour_moves[2] - minute_rest[2]) < 0.05,
        details=(
            f"clock_center={clock_center}, hour_rest={hour_rest}, hour_quarter={hour_quarter}, "
            f"minute_rest={minute_rest}, minute_while_hour_moves={minute_while_hour_moves}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
