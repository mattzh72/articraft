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
    section_loft,
)


TOWER_WIDTH = 4.2
TOWER_HALF = TOWER_WIDTH * 0.5
TOWER_TOP_Z = 14.8
CLOCK_CENTER_Z = 9.55


def _square_loop(width: float, z: float) -> list[tuple[float, float, float]]:
    half = width * 0.5
    return [
        (-half, -half, z),
        (half, -half, z),
        (half, half, z),
        (-half, half, z),
    ]


def _roof_mesh():
    return section_loft(
        [
            _square_loop(5.10, 0.0),
            _square_loop(4.86, 0.20),
            _square_loop(4.10, 1.10),
            _square_loop(0.18, 2.90),
        ]
    )


def _add_clock_surround(part, material) -> None:
    surround_y = TOWER_HALF + 0.08
    part.visual(
        Box((2.72, 0.16, 0.18)),
        origin=Origin(xyz=(0.0, surround_y, CLOCK_CENTER_Z + 1.27)),
        material=material,
        name="clock_surround_top",
    )
    part.visual(
        Box((2.72, 0.16, 0.18)),
        origin=Origin(xyz=(0.0, surround_y, CLOCK_CENTER_Z - 1.27)),
        material=material,
        name="clock_surround_bottom",
    )
    part.visual(
        Box((0.18, 0.16, 2.36)),
        origin=Origin(xyz=(1.27, surround_y, CLOCK_CENTER_Z)),
        material=material,
        name="clock_surround_right",
    )
    part.visual(
        Box((0.18, 0.16, 2.36)),
        origin=Origin(xyz=(-1.27, surround_y, CLOCK_CENTER_Z)),
        material=material,
        name="clock_surround_left",
    )


def _add_corbels(part, material) -> None:
    corbel_z = 14.05
    offsets = (-1.42, -0.71, 0.0, 0.71, 1.42)
    for index, offset in enumerate(offsets):
        part.visual(
            Box((0.28, 0.36, 0.34)),
            origin=Origin(xyz=(offset, TOWER_HALF + 0.12, corbel_z)),
            material=material,
            name=f"front_corbel_{index}",
        )
        part.visual(
            Box((0.28, 0.36, 0.34)),
            origin=Origin(xyz=(offset, -(TOWER_HALF + 0.12), corbel_z)),
            material=material,
            name=f"rear_corbel_{index}",
        )
        part.visual(
            Box((0.36, 0.28, 0.34)),
            origin=Origin(xyz=(TOWER_HALF + 0.12, offset, corbel_z)),
            material=material,
            name=f"right_corbel_{index}",
        )
        part.visual(
            Box((0.36, 0.28, 0.34)),
            origin=Origin(xyz=(-(TOWER_HALF + 0.12), offset, corbel_z)),
            material=material,
            name=f"left_corbel_{index}",
        )


def _add_hand_geometry(
    hand_part,
    *,
    hub_radius: float,
    hand_length: float,
    stem_width: float,
    stem_length: float,
    tip_width: float,
    tip_length: float,
    tail_length: float,
    material,
) -> None:
    hand_part.visual(
        Cylinder(radius=hub_radius, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="hub",
    )
    hand_part.visual(
        Box((stem_width, 0.008, stem_length)),
        origin=Origin(xyz=(0.0, 0.004, stem_length * 0.5 - 0.05)),
        material=material,
        name="stem",
    )
    hand_part.visual(
        Box((tip_width, 0.008, tip_length)),
        origin=Origin(xyz=(0.0, 0.004, hand_length - tip_length * 0.5)),
        material=material,
        name="tip",
    )
    hand_part.visual(
        Box((stem_width * 0.65, 0.008, tail_length)),
        origin=Origin(xyz=(0.0, 0.004, -tail_length * 0.5)),
        material=material,
        name="counterweight",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="victorian_station_clock_tower")

    brick = model.material("brick", rgba=(0.49, 0.24, 0.19, 1.0))
    stone = model.material("stone", rgba=(0.76, 0.72, 0.64, 1.0))
    slate = model.material("slate", rgba=(0.18, 0.20, 0.24, 1.0))
    dial_paint = model.material("dial_paint", rgba=(0.93, 0.91, 0.82, 1.0))
    oxidized_bronze = model.material("oxidized_bronze", rgba=(0.38, 0.34, 0.24, 1.0))
    hand_black = model.material("hand_black", rgba=(0.09, 0.09, 0.10, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((5.00, 5.00, 0.75)),
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
        material=stone,
        name="plinth",
    )
    tower.visual(
        Box((4.62, 4.62, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.84)),
        material=stone,
        name="water_table",
    )
    tower.visual(
        Box((TOWER_WIDTH, TOWER_WIDTH, 13.02)),
        origin=Origin(xyz=(0.0, 0.0, 7.44)),
        material=brick,
        name="shaft",
    )
    tower.visual(
        Box((TOWER_WIDTH, TOWER_WIDTH, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 13.96)),
        material=brick,
        name="upper_shaft",
    )
    tower.visual(
        Box((4.55, 4.55, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 14.21)),
        material=stone,
        name="cornice_lower_band",
    )
    tower.visual(
        Box((4.82, 4.82, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 14.41)),
        material=stone,
        name="cornice_upper_band",
    )
    tower.visual(
        Box((5.05, 5.05, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 14.66)),
        material=stone,
        name="coping_course",
    )
    _add_corbels(tower, stone)
    _add_clock_surround(tower, stone)
    tower.inertial = Inertial.from_geometry(
        Box((5.10, 5.10, TOWER_TOP_Z)),
        mass=42000.0,
        origin=Origin(xyz=(0.0, 0.0, TOWER_TOP_Z * 0.5)),
    )

    roof = model.part("roof")
    roof.visual(
        mesh_from_geometry(_roof_mesh(), "tower_roof"),
        origin=Origin(xyz=(0.0, 0.0, 0.006795)),
        material=slate,
        name="pyramid_roof",
    )
    roof.visual(
        Cylinder(radius=0.07, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 3.116795)),
        material=oxidized_bronze,
        name="finial",
    )
    roof.inertial = Inertial.from_geometry(
        Box((5.10, 5.10, 3.55)),
        mass=2400.0,
        origin=Origin(xyz=(0.0, 0.0, 1.78)),
    )

    clock_face = model.part("clock_face")
    clock_face.visual(
        Cylinder(radius=0.97, length=0.03),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_paint,
        name="dial_disc",
    )
    clock_face.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=1.02,
                tube=0.07,
                radial_segments=16,
                tubular_segments=64,
            ),
            "clock_bezel",
        ),
        origin=Origin(xyz=(0.0, 0.07, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oxidized_bronze,
        name="bezel",
    )
    clock_face.visual(
        Cylinder(radius=0.13, length=0.02),
        origin=Origin(xyz=(0.0, 0.04, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oxidized_bronze,
        name="center_boss",
    )
    clock_face.visual(
        Cylinder(radius=0.07, length=0.052),
        origin=Origin(xyz=(0.0, 0.056, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oxidized_bronze,
        name="spindle",
    )
    clock_face.inertial = Inertial.from_geometry(
        Box((2.25, 0.14, 2.25)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
    )

    hour_hand = model.part("hour_hand")
    _add_hand_geometry(
        hour_hand,
        hub_radius=0.115,
        hand_length=0.68,
        stem_width=0.095,
        stem_length=0.62,
        tip_width=0.19,
        tip_length=0.18,
        tail_length=0.17,
        material=hand_black,
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.22, 0.01, 0.90)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.004, 0.26)),
    )

    minute_hand = model.part("minute_hand")
    _add_hand_geometry(
        minute_hand,
        hub_radius=0.09,
        hand_length=0.92,
        stem_width=0.06,
        stem_length=0.82,
        tip_width=0.13,
        tip_length=0.20,
        tail_length=0.14,
        material=hand_black,
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.16, 0.01, 1.10)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.004, 0.36)),
    )

    model.articulation(
        "tower_to_roof",
        ArticulationType.FIXED,
        parent=tower,
        child=roof,
        origin=Origin(xyz=(0.0, 0.0, TOWER_TOP_Z)),
    )
    model.articulation(
        "tower_to_clock_face",
        ArticulationType.FIXED,
        parent=tower,
        child=clock_face,
        origin=Origin(xyz=(0.0, TOWER_HALF, CLOCK_CENTER_Z)),
    )
    model.articulation(
        "clock_face_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=clock_face,
        child=hour_hand,
        origin=Origin(xyz=(0.0, 0.082, 0.0), rpy=(0.0, -0.92, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.8,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "clock_face_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=clock_face,
        child=minute_hand,
        origin=Origin(xyz=(0.0, 0.09, 0.0), rpy=(0.0, 0.88, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    roof = object_model.get_part("roof")
    clock_face = object_model.get_part("clock_face")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(roof, tower, name="roof_seats_on_coping")
    ctx.expect_contact(clock_face, tower, name="clock_face_mounts_to_front_shaft")
    ctx.expect_contact(hour_hand, clock_face, name="hour_hand_contacts_spindle")
    ctx.expect_contact(minute_hand, hour_hand, name="minute_hand_stacks_on_hour_hand")

    ctx.expect_gap(
        clock_face,
        tower,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem="shaft",
        name="clock_face_flush_to_front_face",
    )
    ctx.expect_overlap(
        clock_face,
        tower,
        axes="xz",
        min_overlap=1.90,
        name="clock_face_centered_on_tower_front",
    )
    ctx.expect_within(
        hour_hand,
        clock_face,
        axes="xz",
        margin=0.06,
        name="hour_hand_stays_within_dial",
    )
    ctx.expect_within(
        minute_hand,
        clock_face,
        axes="xz",
        margin=0.06,
        name="minute_hand_stays_within_dial",
    )

    ctx.check(
        "hour_joint_axis_is_normal_to_clock_face",
        tuple(hour_joint.axis) == (0.0, 1.0, 0.0),
        details=f"Expected (0, 1, 0), got {hour_joint.axis}",
    )
    ctx.check(
        "minute_joint_axis_is_normal_to_clock_face",
        tuple(minute_joint.axis) == (0.0, 1.0, 0.0),
        details=f"Expected (0, 1, 0), got {minute_joint.axis}",
    )

    with ctx.pose({hour_joint: 0.55, minute_joint: -1.25}):
        ctx.expect_contact(hour_hand, clock_face, name="hour_hand_remains_hub_mounted")
        ctx.expect_contact(minute_hand, hour_hand, name="minute_hand_remains_stacked_in_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
