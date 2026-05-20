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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modernist_clock_tower")

    concrete = model.material("concrete", rgba=(0.69, 0.69, 0.67, 1.0))
    dark_concrete = model.material("dark_concrete", rgba=(0.55, 0.55, 0.53, 1.0))
    dial_white = model.material("dial_white", rgba=(0.90, 0.91, 0.88, 1.0))
    bezel_gray = model.material("bezel_gray", rgba=(0.38, 0.39, 0.40, 1.0))
    hand_black = model.material("hand_black", rgba=(0.09, 0.09, 0.10, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((4.0, 4.0, 0.45)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=dark_concrete,
        name="foundation_plinth",
    )
    tower.visual(
        Box((2.6, 2.6, 1.0)),
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        material=concrete,
        name="podium_block",
    )
    tower.visual(
        Box((1.55, 1.55, 15.6)),
        origin=Origin(xyz=(0.0, 0.0, 9.25)),
        material=concrete,
        name="shaft",
    )
    tower.visual(
        Box((2.15, 2.15, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 17.20)),
        material=dark_concrete,
        name="crown_slab",
    )
    tower.visual(
        Box((0.42, 0.03, 1.85)),
        origin=Origin(xyz=(0.0, 0.76, 14.90)),
        material=dark_concrete,
        name="front_slit",
    )
    tower.visual(
        Box((0.50, 0.03, 1.05)),
        origin=Origin(xyz=(0.0, 0.76, 2.10)),
        material=dark_concrete,
        name="entry_shadow",
    )
    tower.inertial = Inertial.from_geometry(
        Box((4.0, 4.0, 17.35)),
        mass=28000.0,
        origin=Origin(xyz=(0.0, 0.0, 8.675)),
    )

    face_radius = 0.56
    face_thickness = 0.06
    face = model.part("clock_face")
    face.visual(
        Cylinder(radius=0.64, length=0.02),
        origin=Origin(xyz=(0.0, 0.01, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_concrete,
        name="clock_surround",
    )
    face.visual(
        Cylinder(radius=0.60, length=0.045),
        origin=Origin(xyz=(0.0, 0.0225, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bezel_gray,
        name="clock_bezel",
    )
    face.visual(
        Cylinder(radius=face_radius, length=face_thickness),
        origin=Origin(xyz=(0.0, face_thickness * 0.5, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_white,
        name="dial_disc",
    )
    face.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(0.0, 0.069, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bezel_gray,
        name="center_boss",
    )
    face.inertial = Inertial.from_geometry(
        Box((1.28, 0.08, 1.28)),
        mass=45.0,
        origin=Origin(xyz=(0.0, 0.04, 0.0)),
    )

    def build_hand_mesh(
        name: str,
        *,
        length: float,
        root_offset: float,
        root_width: float,
        mid_width: float,
        tip_width: float,
        thickness: float,
    ):
        profile = [
            (-root_width * 0.50, root_offset),
            (root_width * 0.50, root_offset),
            (root_width * 0.48, root_offset + (length - root_offset) * 0.10),
            (mid_width * 0.50, length * 0.78),
            (tip_width * 0.50, length * 0.93),
            (0.0, length),
            (-tip_width * 0.50, length * 0.93),
            (-mid_width * 0.50, length * 0.78),
            (-root_width * 0.48, root_offset + (length - root_offset) * 0.10),
        ]
        geom = ExtrudeGeometry(profile, thickness, center=True).rotate_x(math.pi / 2.0)
        return mesh_from_geometry(geom, name)

    hour_pointer_mesh = build_hand_mesh(
        "hour_pointer",
        length=0.27,
        root_offset=0.082,
        root_width=0.040,
        mid_width=0.032,
        tip_width=0.018,
        thickness=0.006,
    )
    minute_pointer_mesh = build_hand_mesh(
        "minute_pointer",
        length=0.43,
        root_offset=0.078,
        root_width=0.028,
        mid_width=0.022,
        tip_width=0.014,
        thickness=0.004,
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        hour_pointer_mesh,
        origin=Origin(xyz=(0.0, 0.072, 0.0)),
        material=hand_black,
        name="hour_pointer",
    )
    hour_hand.visual(
        Cylinder(radius=0.084, length=0.014),
        origin=Origin(xyz=(0.0, 0.064, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bezel_gray,
        name="hour_hub",
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.10, 0.020, 0.34)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.066, 0.12)),
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        minute_pointer_mesh,
        origin=Origin(xyz=(0.0, 0.076, 0.0)),
        material=hand_black,
        name="minute_pointer",
    )
    minute_hand.visual(
        Cylinder(radius=0.080, length=0.012),
        origin=Origin(xyz=(0.0, 0.070, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hand_black,
        name="minute_hub",
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.06, 0.016, 0.48)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.072, 0.17)),
    )

    model.articulation(
        "tower_to_clock_face",
        ArticulationType.FIXED,
        parent=tower,
        child=face,
        origin=Origin(xyz=(0.0, 0.775, 14.90)),
    )
    model.articulation(
        "clock_face_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=face,
        child=hour_hand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.5,
            lower=0.0,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "clock_face_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=face,
        child=minute_hand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=6.0,
            lower=0.0,
            upper=2.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    face = object_model.get_part("clock_face")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    tower_shaft = tower.get_visual("shaft")
    face_surround = face.get_visual("clock_surround")
    face_boss = face.get_visual("center_boss")
    hour_hub = hour_hand.get_visual("hour_hub")
    minute_hub = minute_hand.get_visual("minute_hub")

    hour_joint = object_model.get_articulation("clock_face_to_hour_hand")
    minute_joint = object_model.get_articulation("clock_face_to_minute_hand")

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
    ctx.allow_overlap(
        face,
        hour_hand,
        elem_a=face_boss,
        elem_b=hour_hub,
        reason="The hour hand hub is modeled as a nested coaxial arbor seated into the dial center boss.",
    )
    ctx.allow_overlap(
        face,
        minute_hand,
        elem_a=face_boss,
        elem_b=minute_hub,
        reason="The minute hand hub nests concentrically over the dial's central arbor boss.",
    )
    ctx.allow_overlap(
        hour_hand,
        minute_hand,
        elem_a=hour_hub,
        elem_b=minute_hub,
        reason="Stacked clock hand hubs share the same axis and intentionally interpenetrate in this simplified coaxial representation.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(face, tower, name="clock_face_is_mounted_to_tower")
    ctx.expect_contact(hour_hand, face, name="hour_hand_is_supported_by_face")
    ctx.expect_contact(minute_hand, hour_hand, name="minute_hand_is_supported_by_hour_hand")
    ctx.expect_gap(
        face,
        tower,
        axis="y",
        positive_elem=face_surround,
        negative_elem=tower_shaft,
        max_gap=0.001,
        max_penetration=0.0,
        name="clock_face_seats_on_front_surface",
    )
    ctx.expect_overlap(face, tower, axes="xz", min_overlap=1.0, name="clock_face_overlaps_tower_front_projection")
    ctx.expect_within(face, tower, axes="xz", margin=0.15, name="clock_face_stays_within_shaft_width")

    tower_aabb = ctx.part_world_aabb(tower)
    face_aabb = ctx.part_world_aabb(face)
    assert tower_aabb is not None
    assert face_aabb is not None
    tower_height = tower_aabb[1][2] - tower_aabb[0][2]
    face_center_z = (face_aabb[0][2] + face_aabb[1][2]) * 0.5
    ctx.check(
        "tower_reads_as_tall_clock_tower",
        tower_height > 17.0,
        f"tower height {tower_height:.3f} m is too short for a clock tower silhouette",
    )
    ctx.check(
        "clock_face_is_near_top_of_tower",
        face_center_z > tower_aabb[1][2] - 3.5,
        f"clock face center z={face_center_z:.3f} is not placed near tower crown",
    )
    ctx.check(
        "hour_joint_rotates_about_face_normal",
        tuple(round(v, 6) for v in hour_joint.axis) == (0.0, 1.0, 0.0),
        f"unexpected hour joint axis {hour_joint.axis}",
    )
    ctx.check(
        "minute_joint_rotates_about_face_normal",
        tuple(round(v, 6) for v in minute_joint.axis) == (0.0, 1.0, 0.0),
        f"unexpected minute joint axis {minute_joint.axis}",
    )

    hour_rest = ctx.part_element_world_aabb(hour_hand, elem="hour_pointer")
    minute_rest = ctx.part_element_world_aabb(minute_hand, elem="minute_pointer")
    assert hour_rest is not None
    assert minute_rest is not None
    hour_rest_dx = hour_rest[1][0] - hour_rest[0][0]
    hour_rest_dz = hour_rest[1][2] - hour_rest[0][2]
    minute_rest_dx = minute_rest[1][0] - minute_rest[0][0]
    minute_rest_dz = minute_rest[1][2] - minute_rest[0][2]
    ctx.check(
        "hands_start_vertically",
        hour_rest_dz > hour_rest_dx and minute_rest_dz > minute_rest_dx,
        "clock hands do not begin aligned in the vertical clock-face direction",
    )

    with ctx.pose({hour_joint: math.pi / 2.0, minute_joint: math.pi / 2.0}):
        hour_turned = ctx.part_element_world_aabb(hour_hand, elem="hour_pointer")
        minute_turned = ctx.part_element_world_aabb(minute_hand, elem="minute_pointer")
        assert hour_turned is not None
        assert minute_turned is not None
        hour_turned_dx = hour_turned[1][0] - hour_turned[0][0]
        hour_turned_dz = hour_turned[1][2] - hour_turned[0][2]
        minute_turned_dx = minute_turned[1][0] - minute_turned[0][0]
        minute_turned_dz = minute_turned[1][2] - minute_turned[0][2]
        ctx.check(
            "hour_hand_rotates_in_clock_plane",
            hour_turned_dx > hour_turned_dz,
            "hour hand did not sweep from vertical toward horizontal",
        )
        ctx.check(
            "minute_hand_rotates_in_clock_plane",
            minute_turned_dx > minute_turned_dz,
            "minute hand did not sweep from vertical toward horizontal",
        )
        ctx.expect_contact(hour_hand, face, name="hour_hand_remains_borne_by_face_when_rotated")
        ctx.expect_contact(minute_hand, hour_hand, name="minute_hand_remains_borne_by_hour_hand_when_rotated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
