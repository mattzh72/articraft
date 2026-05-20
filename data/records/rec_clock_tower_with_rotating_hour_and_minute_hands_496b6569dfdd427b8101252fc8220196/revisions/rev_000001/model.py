from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lighthouse_clock_tower")

    stone = model.material("stone", rgba=(0.79, 0.76, 0.69, 1.0))
    trim_stone = model.material("trim_stone", rgba=(0.67, 0.64, 0.58, 1.0))
    lantern_metal = model.material("lantern_metal", rgba=(0.24, 0.27, 0.30, 1.0))
    roof_metal = model.material("roof_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    dial_white = model.material("dial_white", rgba=(0.94, 0.92, 0.86, 1.0))
    marker_dark = model.material("marker_dark", rgba=(0.18, 0.17, 0.16, 1.0))
    hand_dark = model.material("hand_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.80, 0.92, 0.97, 0.42))

    base_height = 2.8
    shaft_height = 13.4
    shaft_radius = 1.70
    clock_band_radius = 1.95
    clock_center_z = 10.15
    support_ring_height = 0.40
    lantern_deck_bottom = shaft_height + support_ring_height

    def octagon_mesh(radius: float, height: float, name: str):
        return mesh_from_geometry(
            CylinderGeometry(radius, height, radial_segments=8, closed=True),
            name,
        )

    def clock_xy(clockwise_angle: float, radius: float) -> tuple[float, float]:
        return (-radius * cos(clockwise_angle), radius * sin(clockwise_angle))

    base = model.part("base")
    base.visual(
        octagon_mesh(3.15, 1.55, "base_lower_octagon"),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material=stone,
        name="lower_plinth",
    )
    base.visual(
        octagon_mesh(2.62, 1.25, "base_upper_octagon"),
        origin=Origin(xyz=(0.0, 0.0, 2.175)),
        material=stone,
        name="upper_plinth",
    )
    base.visual(
        octagon_mesh(2.25, 0.18, "base_cap_octagon"),
        origin=Origin(xyz=(0.0, 0.0, 2.71)),
        material=trim_stone,
        name="cap_course",
    )
    base.visual(
        Box((1.20, 0.38, 1.95)),
        origin=Origin(xyz=(2.03, 0.0, 0.975)),
        material=trim_stone,
        name="entry_block",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=shaft_radius, length=shaft_height),
        origin=Origin(xyz=(0.0, 0.0, shaft_height / 2.0)),
        material=stone,
        name="main_shaft",
    )
    shaft.visual(
        Cylinder(radius=1.94, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=trim_stone,
        name="shaft_foot_course",
    )
    shaft.visual(
        Cylinder(radius=clock_band_radius, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, clock_center_z)),
        material=trim_stone,
        name="clock_band",
    )
    shaft.visual(
        Cylinder(radius=2.18, length=support_ring_height),
        origin=Origin(xyz=(0.0, 0.0, shaft_height + support_ring_height / 2.0)),
        material=trim_stone,
        name="gallery_support",
    )

    model.articulation(
        "base_to_shaft",
        ArticulationType.FIXED,
        parent=base,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, base_height)),
    )

    lantern_room = model.part("lantern_room")
    lantern_room.visual(
        octagon_mesh(2.35, 0.26, "lantern_gallery_deck"),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=trim_stone,
        name="gallery_deck",
    )
    lantern_room.visual(
        octagon_mesh(1.52, 0.24, "lantern_lower_sill"),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=lantern_metal,
        name="lower_sill",
    )
    lantern_room.visual(
        octagon_mesh(1.52, 0.24, "lantern_upper_ring"),
        origin=Origin(xyz=(0.0, 0.0, 2.33)),
        material=lantern_metal,
        name="upper_ring",
    )
    lantern_room.visual(
        mesh_from_geometry(
            ConeGeometry(radius=1.70, height=1.55, radial_segments=8, closed=True),
            "lantern_roof",
        ),
        origin=Origin(xyz=(0.0, 0.0, 3.22)),
        material=roof_metal,
        name="roof",
    )
    lantern_room.visual(
        Cylinder(radius=0.38, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 4.21)),
        material=roof_metal,
        name="vent",
    )
    lantern_room.visual(
        Cylinder(radius=0.12, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 4.64)),
        material=roof_metal,
        name="finial",
    )

    post_radius = 1.62
    panel_radius = 1.40
    for index in range(8):
        corner_angle = index * (pi / 4.0) + (pi / 8.0)
        face_angle = index * (pi / 4.0)

        lantern_room.visual(
            Box((0.15, 0.15, 2.18)),
            origin=Origin(
                xyz=(
                    post_radius * cos(corner_angle),
                    post_radius * sin(corner_angle),
                    1.29,
                ),
                rpy=(0.0, 0.0, corner_angle),
            ),
            material=lantern_metal,
            name=f"post_{index}",
        )

        lantern_room.visual(
            Box((1.00, 0.10, 1.82)),
            origin=Origin(
                xyz=(
                    panel_radius * cos(face_angle),
                    panel_radius * sin(face_angle),
                    1.37,
                ),
                rpy=(0.0, 0.0, face_angle + pi / 2.0),
            ),
            material=glass,
            name=f"glass_{index}",
        )

    model.articulation(
        "shaft_to_lantern_room",
        ArticulationType.FIXED,
        parent=shaft,
        child=lantern_room,
        origin=Origin(xyz=(0.0, 0.0, lantern_deck_bottom)),
    )

    clock_face = model.part("clock_face")
    clock_face.visual(
        Cylinder(radius=1.125, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dial_white,
        name="dial_disk",
    )
    clock_face.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(1.28, 0.0), (1.28, 0.064)],
                [(1.12, 0.0), (1.12, 0.064)],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
            "clock_bezel_ring",
        ),
        material=lantern_metal,
        name="bezel",
    )
    clock_face.visual(
        Cylinder(radius=0.14, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=marker_dark,
        name="center_boss",
    )
    for index in range(12):
        marker_angle = index * (pi / 6.0)
        marker_x, marker_y = clock_xy(marker_angle, 0.94)
        marker_long = 0.22 if index % 3 == 0 else 0.14
        marker_wide = 0.07 if index % 3 == 0 else 0.05
        clock_face.visual(
            Box((marker_long, marker_wide, 0.010)),
            origin=Origin(
                xyz=(marker_x, marker_y, 0.026),
                rpy=(0.0, 0.0, -marker_angle),
            ),
            material=marker_dark,
            name=f"marker_{index}",
        )

    model.articulation(
        "shaft_to_clock_face",
        ArticulationType.FIXED,
        parent=shaft,
        child=clock_face,
        origin=Origin(xyz=(clock_band_radius, 0.0, clock_center_z), rpy=(0.0, pi / 2.0, 0.0)),
    )

    hour_hand = model.part("hour_hand")
    hour_angle = 11.0 * pi / 36.0
    hour_hand.visual(
        Cylinder(radius=0.13, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=hand_dark,
        name="hour_hub",
    )
    hour_hand.visual(
        Box((0.86, 0.16, 0.012)),
        origin=Origin(xyz=(-0.43, 0.0, 0.036)),
        material=hand_dark,
        name="hour_blade",
    )
    hour_hand.visual(
        Box((0.24, 0.08, 0.012)),
        origin=Origin(xyz=(-0.88, 0.0, 0.036)),
        material=hand_dark,
        name="hour_tip",
    )
    hour_hand.visual(
        Box((0.26, 0.08, 0.012)),
        origin=Origin(xyz=(0.13, 0.0, 0.036)),
        material=hand_dark,
        name="hour_counterweight",
    )

    minute_hand = model.part("minute_hand")
    minute_angle = -pi / 3.0
    minute_hand.visual(
        Cylinder(radius=0.09, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=hand_dark,
        name="minute_hub",
    )
    minute_hand.visual(
        Box((1.06, 0.12, 0.010)),
        origin=Origin(xyz=(-0.53, 0.0, 0.050)),
        material=hand_dark,
        name="minute_blade",
    )
    minute_hand.visual(
        Box((0.20, 0.07, 0.010)),
        origin=Origin(xyz=(-0.99, 0.0, 0.050)),
        material=hand_dark,
        name="minute_tip",
    )
    minute_hand.visual(
        Box((0.28, 0.07, 0.010)),
        origin=Origin(xyz=(0.14, 0.0, 0.050)),
        material=hand_dark,
        name="minute_counterweight",
    )

    hand_limits = MotionLimits(
        effort=2.0,
        velocity=6.0,
        lower=-2.0 * pi,
        upper=2.0 * pi,
    )
    model.articulation(
        "clock_face_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=clock_face,
        child=hour_hand,
        origin=Origin(rpy=(0.0, 0.0, hour_angle)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=hand_limits,
    )
    model.articulation(
        "clock_face_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=clock_face,
        child=minute_hand,
        origin=Origin(rpy=(0.0, 0.0, minute_angle)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=12.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shaft = object_model.get_part("shaft")
    lantern_room = object_model.get_part("lantern_room")
    clock_face = object_model.get_part("clock_face")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("clock_face_to_hour_hand")
    minute_joint = object_model.get_articulation("clock_face_to_minute_hand")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

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

    ctx.expect_contact(base, shaft, name="shaft is seated on the octagonal base")
    ctx.expect_contact(shaft, lantern_room, name="lantern room is seated on the shaft gallery")
    ctx.expect_contact(shaft, clock_face, name="clock face is mounted to the upper shaft band")
    ctx.expect_contact(clock_face, hour_hand, name="hour hand hub contacts the dial boss")
    ctx.expect_contact(hour_hand, minute_hand, name="minute hand is stacked concentrically above the hour hand")
    ctx.expect_gap(
        minute_hand,
        clock_face,
        axis="x",
        min_gap=0.008,
        positive_elem="minute_blade",
        negative_elem="dial_disk",
        name="minute hand sits proud of the dial face",
    )

    minute_limits = minute_joint.motion_limits
    hour_limits = hour_joint.motion_limits
    ctx.check(
        "clock hand joints share the face-normal axis",
        tuple(hour_joint.axis) == (0.0, 0.0, -1.0) and tuple(minute_joint.axis) == (0.0, 0.0, -1.0),
        details=f"hour_axis={hour_joint.axis}, minute_axis={minute_joint.axis}",
    )
    ctx.check(
        "clock hands have full circular travel",
        minute_limits is not None
        and hour_limits is not None
        and minute_limits.lower is not None
        and minute_limits.upper is not None
        and hour_limits.lower is not None
        and hour_limits.upper is not None
        and minute_limits.upper - minute_limits.lower >= 2.0 * pi
        and hour_limits.upper - hour_limits.lower >= 2.0 * pi,
        details=f"hour_limits={hour_limits}, minute_limits={minute_limits}",
    )

    face_origin = ctx.part_world_position(clock_face)
    rest_tip = aabb_center(ctx.part_element_world_aabb(minute_hand, elem="minute_tip"))
    with ctx.pose({minute_joint: pi / 3.0, hour_joint: pi / 6.0}):
        moved_tip = aabb_center(ctx.part_element_world_aabb(minute_hand, elem="minute_tip"))
    if face_origin is not None and rest_tip is not None and moved_tip is not None:
        rest_radius = sqrt((rest_tip[1] - face_origin[1]) ** 2 + (rest_tip[2] - face_origin[2]) ** 2)
        moved_radius = sqrt((moved_tip[1] - face_origin[1]) ** 2 + (moved_tip[2] - face_origin[2]) ** 2)
        tip_motion = sqrt((moved_tip[1] - rest_tip[1]) ** 2 + (moved_tip[2] - rest_tip[2]) ** 2)
        ctx.check(
            "minute hand tip sweeps around the dial center",
            abs(rest_radius - moved_radius) <= 0.08
            and abs(rest_tip[0] - moved_tip[0]) <= 0.01
            and tip_motion >= 0.35,
            details=(
                f"face_origin={face_origin}, rest_tip={rest_tip}, moved_tip={moved_tip}, "
                f"rest_radius={rest_radius}, moved_radius={moved_radius}, tip_motion={tip_motion}"
            ),
        )
    else:
        ctx.fail(
            "minute hand tip sweeps around the dial center",
            f"face_origin={face_origin}, rest_tip={rest_tip}, moved_tip={moved_tip}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
