from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


PLINTH_SIZE = 8.6
PLINTH_HEIGHT = 1.2
SHAFT_SIZE = 6.6
SHAFT_HEIGHT = 18.8
CORNICE_SIZE = 7.4
CORNICE_HEIGHT = 0.55
UPPER_STAGE_SIZE = 7.0
UPPER_STAGE_HEIGHT = 4.2
ROOF_BASE_SIZE = 7.8
ROOF_HEIGHT = 5.4
CLOCK_PANEL_SIZE = 4.9
CLOCK_PANEL_THICKNESS = 0.18
CLOCK_CENTER_Z = 13.6
FRONT_FACE_Y = SHAFT_SIZE * 0.5


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _square_loop(size: float, z: float) -> list[tuple[float, float, float]]:
    half = size * 0.5
    return [(-half, -half, z), (half, -half, z), (half, half, z), (-half, half, z)]


def _annulus_mesh(name: str, *, outer_radius: float, inner_radius: float, thickness: float):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        thickness,
        center=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(geom, name)


def _ornate_hand_mesh(
    name: str,
    *,
    length: float,
    tail: float,
    base_width: float,
    waist_width: float,
    tip_width: float,
    thickness: float,
):
    tail_width = base_width * 0.82
    profile = [
        (-tail_width * 0.48, -tail),
        (tail_width * 0.48, -tail),
        (tail_width * 0.72, -0.10),
        (base_width * 0.48, 0.04),
        (waist_width * 0.55, length * 0.34),
        (base_width * 0.32, length * 0.72),
        (tip_width * 0.70, length * 0.90),
        (0.0, length),
        (-tip_width * 0.70, length * 0.90),
        (-base_width * 0.32, length * 0.72),
        (-waist_width * 0.55, length * 0.34),
        (-base_width * 0.48, 0.04),
        (-tail_width * 0.72, -0.10),
    ]
    geom = ExtrudeGeometry(profile, thickness, center=True).rotate_x(pi / 2.0)
    return mesh_from_geometry(geom, name)


def _span(aabb, axis_index: int) -> float | None:
    if aabb is None:
        return None
    return aabb[1][axis_index] - aabb[0][axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_astronomical_clock_tower")

    stone = model.material("stone", rgba=(0.67, 0.64, 0.58, 1.0))
    stone_dark = model.material("stone_dark", rgba=(0.54, 0.51, 0.46, 1.0))
    dial_blue = model.material("dial_blue", rgba=(0.10, 0.17, 0.28, 1.0))
    gilded_bronze = model.material("gilded_bronze", rgba=(0.78, 0.67, 0.28, 1.0))
    aged_copper = model.material("aged_copper", rgba=(0.31, 0.46, 0.39, 1.0))
    black_iron = model.material("black_iron", rgba=(0.18, 0.16, 0.14, 1.0))

    tower_shaft = model.part("tower_shaft")
    tower_shaft.inertial = Inertial.from_geometry(
        Box((PLINTH_SIZE, PLINTH_SIZE, 31.0)),
        mass=68000.0,
        origin=Origin(xyz=(0.0, 0.0, 15.5)),
    )
    tower_shaft.visual(
        Box((PLINTH_SIZE, PLINTH_SIZE, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
        material=stone_dark,
        name="plinth",
    )
    tower_shaft.visual(
        Box((SHAFT_SIZE, SHAFT_SIZE, SHAFT_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + SHAFT_HEIGHT * 0.5)),
        material=stone,
        name="shaft_core",
    )
    buttress_half = 0.38
    buttress_height = PLINTH_HEIGHT + SHAFT_HEIGHT
    buttress_z = buttress_height * 0.5
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower_shaft.visual(
                Box((buttress_half * 2.0, buttress_half * 2.0, buttress_height)),
                origin=Origin(
                    xyz=(
                        sx * (SHAFT_SIZE * 0.5 - buttress_half),
                        sy * (SHAFT_SIZE * 0.5 - buttress_half),
                        buttress_z,
                    )
                ),
                material=stone_dark,
            )
    tower_shaft.visual(
        Box((CORNICE_SIZE, CORNICE_SIZE, CORNICE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, PLINTH_HEIGHT + SHAFT_HEIGHT + CORNICE_HEIGHT * 0.5)
        ),
        material=stone_dark,
        name="cornice",
    )
    upper_stage_bottom = PLINTH_HEIGHT + SHAFT_HEIGHT + CORNICE_HEIGHT
    tower_shaft.visual(
        Box((UPPER_STAGE_SIZE, UPPER_STAGE_SIZE, UPPER_STAGE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, upper_stage_bottom + UPPER_STAGE_HEIGHT * 0.5)
        ),
        material=stone,
        name="upper_stage",
    )
    stage_pilaster = 0.54
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower_shaft.visual(
                Box((stage_pilaster, stage_pilaster, UPPER_STAGE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        sx * (UPPER_STAGE_SIZE * 0.5 - stage_pilaster * 0.5),
                        sy * (UPPER_STAGE_SIZE * 0.5 - stage_pilaster * 0.5),
                        upper_stage_bottom + UPPER_STAGE_HEIGHT * 0.5,
                    )
                ),
                material=stone_dark,
            )
    tower_shaft.visual(
        Box((UPPER_STAGE_SIZE + 0.35, UPPER_STAGE_SIZE + 0.35, 0.28)),
        origin=Origin(
            xyz=(0.0, 0.0, upper_stage_bottom + UPPER_STAGE_HEIGHT - 0.14)
        ),
        material=stone_dark,
    )

    roof = model.part("roof")
    roof.inertial = Inertial.from_geometry(
        Box((ROOF_BASE_SIZE, ROOF_BASE_SIZE, ROOF_HEIGHT + 1.2)),
        mass=9200.0,
        origin=Origin(xyz=(0.0, 0.0, (ROOF_HEIGHT + 1.2) * 0.5)),
    )
    roof_shell = section_loft(
        [
            _square_loop(ROOF_BASE_SIZE, 0.0),
            _square_loop(5.4, 1.6),
            _square_loop(2.3, 4.0),
            _square_loop(0.45, ROOF_HEIGHT),
        ]
    )
    roof.visual(
        mesh_from_geometry(roof_shell, "clock_tower_roof_shell"),
        material=aged_copper,
        name="roof_shell",
    )
    roof.visual(
        Box((0.58, 0.58, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, ROOF_HEIGHT + 0.17)),
        material=gilded_bronze,
        name="lantern_cap",
    )
    roof.visual(
        Cylinder(radius=0.14, length=0.9),
        origin=Origin(xyz=(0.0, 0.0, ROOF_HEIGHT + 0.79)),
        material=gilded_bronze,
        name="spire",
    )

    clock_face = model.part("clock_face")
    clock_face.inertial = Inertial.from_geometry(
        Box((CLOCK_PANEL_SIZE, 0.30, CLOCK_PANEL_SIZE)),
        mass=3200.0,
        origin=Origin(xyz=(0.0, 0.15, 0.0)),
    )
    clock_face.visual(
        Box((CLOCK_PANEL_SIZE, CLOCK_PANEL_THICKNESS, CLOCK_PANEL_SIZE)),
        origin=Origin(xyz=(0.0, CLOCK_PANEL_THICKNESS * 0.5, 0.0)),
        material=dial_blue,
        name="dial_panel",
    )
    clock_face.visual(
        Box((CLOCK_PANEL_SIZE, 0.06, 0.26)),
        origin=Origin(xyz=(0.0, 0.15, CLOCK_PANEL_SIZE * 0.5 - 0.18)),
        material=gilded_bronze,
    )
    clock_face.visual(
        Box((CLOCK_PANEL_SIZE, 0.06, 0.26)),
        origin=Origin(xyz=(0.0, 0.15, -CLOCK_PANEL_SIZE * 0.5 + 0.18)),
        material=gilded_bronze,
    )
    clock_face.visual(
        Box((0.26, 0.06, CLOCK_PANEL_SIZE - 0.36)),
        origin=Origin(xyz=(CLOCK_PANEL_SIZE * 0.5 - 0.18, 0.15, 0.0)),
        material=gilded_bronze,
    )
    clock_face.visual(
        Box((0.26, 0.06, CLOCK_PANEL_SIZE - 0.36)),
        origin=Origin(xyz=(-CLOCK_PANEL_SIZE * 0.5 + 0.18, 0.15, 0.0)),
        material=gilded_bronze,
    )
    clock_face.visual(
        _annulus_mesh(
            "clock_tower_zodiac_ring",
            outer_radius=1.86,
            inner_radius=1.40,
            thickness=0.055,
        ),
        origin=Origin(xyz=(0.0, 0.2075, 0.0)),
        material=gilded_bronze,
        name="zodiac_ring",
    )
    clock_face.visual(
        _annulus_mesh(
            "clock_tower_chapter_ring",
            outer_radius=1.24,
            inner_radius=0.88,
            thickness=0.045,
        ),
        origin=Origin(xyz=(0.0, 0.2025, 0.0)),
        material=gilded_bronze,
        name="chapter_ring",
    )
    clock_face.visual(
        _annulus_mesh(
            "clock_tower_center_roundel",
            outer_radius=0.58,
            inner_radius=0.24,
            thickness=0.030,
        ),
        origin=Origin(xyz=(0.0, 0.195, 0.0)),
        material=gilded_bronze,
        name="center_roundel",
    )
    for index in range(12):
        angle = index * pi / 6.0
        clock_face.visual(
            Cylinder(radius=0.085, length=0.028),
            origin=Origin(
                xyz=(sin(angle) * 1.60, 0.194, cos(angle) * 1.60),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=gilded_bronze,
        )
        clock_face.visual(
            Box((0.06, 0.034, 0.18)),
            origin=Origin(
                xyz=(sin(angle) * 1.05, 0.197, cos(angle) * 1.05),
                rpy=(0.0, angle, 0.0),
            ),
            material=gilded_bronze,
        )
    clock_face.visual(
        Cylinder(radius=0.17, length=0.012),
        origin=Origin(xyz=(0.0, 0.186, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="center_boss",
    )
    clock_face.visual(
        Cylinder(radius=0.095, length=0.095),
        origin=Origin(xyz=(0.0, 0.2275, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="hour_arbor",
    )

    hour_hand = model.part("hour_hand")
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.42, 0.09, 1.34)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.31, 0.405)),
    )
    hour_hand.visual(
        Cylinder(radius=0.18, length=0.016),
        origin=Origin(xyz=(0.0, 0.283, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="hour_hub",
    )
    hour_hand.visual(
        _ornate_hand_mesh(
            "clock_tower_hour_hand_blade",
            length=1.05,
            tail=0.24,
            base_width=0.32,
            waist_width=0.13,
            tip_width=0.19,
            thickness=0.010,
        ),
        origin=Origin(xyz=(0.0, 0.291, 0.0)),
        material=black_iron,
        name="hour_blade",
    )
    hour_hand.visual(
        Cylinder(radius=0.072, length=0.064),
        origin=Origin(xyz=(0.0, 0.323, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gilded_bronze,
        name="minute_post",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.34, 0.06, 1.86)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.372, 0.61)),
    )
    minute_hand.visual(
        Cylinder(radius=0.11, length=0.014),
        origin=Origin(xyz=(0.0, 0.362, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="minute_hub",
    )
    minute_hand.visual(
        _ornate_hand_mesh(
            "clock_tower_minute_hand_blade",
            length=1.52,
            tail=0.30,
            base_width=0.24,
            waist_width=0.09,
            tip_width=0.15,
            thickness=0.012,
        ),
        origin=Origin(xyz=(0.0, 0.369, 0.0)),
        material=black_iron,
        name="minute_blade",
    )
    minute_hand.visual(
        Cylinder(radius=0.07, length=0.010),
        origin=Origin(xyz=(0.0, 0.377, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gilded_bronze,
        name="minute_cap",
    )

    roof_base_z = upper_stage_bottom + UPPER_STAGE_HEIGHT
    model.articulation(
        "shaft_to_roof",
        ArticulationType.FIXED,
        parent=tower_shaft,
        child=roof,
        origin=Origin(xyz=(0.0, 0.0, roof_base_z)),
    )
    model.articulation(
        "shaft_to_clock_face",
        ArticulationType.FIXED,
        parent=tower_shaft,
        child=clock_face,
        origin=Origin(xyz=(0.0, FRONT_FACE_Y, CLOCK_CENTER_Z)),
    )
    model.articulation(
        "clock_face_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=clock_face,
        child=hour_hand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.8,
            lower=0.0,
            upper=2.0 * pi,
        ),
    )
    model.articulation(
        "clock_face_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=clock_face,
        child=minute_hand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=2.0 * pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_shaft = object_model.get_part("tower_shaft")
    roof = object_model.get_part("roof")
    clock_face = object_model.get_part("clock_face")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("clock_face_to_hour_hand")
    minute_joint = object_model.get_articulation("clock_face_to_minute_hand")

    clock_face.get_visual("zodiac_ring")
    clock_face.get_visual("chapter_ring")
    hour_hand.get_visual("hour_blade")
    minute_hand.get_visual("minute_blade")

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

    ctx.expect_contact(
        clock_face,
        tower_shaft,
        elem_a="dial_panel",
        elem_b="shaft_core",
        name="clock face panel mounts to the front shaft face",
    )
    ctx.expect_contact(
        roof,
        tower_shaft,
        elem_a="roof_shell",
        elem_b="upper_stage",
        name="roof seats on the tower upper stage",
    )
    ctx.expect_contact(
        hour_hand,
        clock_face,
        elem_a="hour_hub",
        elem_b="hour_arbor",
        name="hour hand is supported by the projecting center arbor",
    )
    ctx.expect_contact(
        minute_hand,
        hour_hand,
        elem_a="minute_hub",
        elem_b="minute_post",
        name="minute hand stacks onto the hour hand arbor assembly",
    )
    ctx.expect_origin_distance(
        hour_hand,
        clock_face,
        axes="xz",
        max_dist=1e-6,
        name="hour hand is concentric with the dial center",
    )
    ctx.expect_origin_distance(
        minute_hand,
        clock_face,
        axes="xz",
        max_dist=1e-6,
        name="minute hand is concentric with the dial center",
    )
    ctx.expect_gap(
        hour_hand,
        clock_face,
        axis="y",
        positive_elem="hour_blade",
        negative_elem="dial_panel",
        min_gap=0.09,
        max_gap=0.13,
        name="hour hand floats just proud of the dial panel",
    )
    ctx.expect_gap(
        minute_hand,
        clock_face,
        axis="y",
        positive_elem="minute_blade",
        negative_elem="dial_panel",
        min_gap=0.17,
        max_gap=0.21,
        name="minute hand sits in front of the hour hand and dial",
    )

    zodiac_aabb = ctx.part_element_world_aabb(clock_face, elem="zodiac_ring")
    chapter_aabb = ctx.part_element_world_aabb(clock_face, elem="chapter_ring")
    dial_aabb = ctx.part_element_world_aabb(clock_face, elem="dial_panel")
    if zodiac_aabb is not None and chapter_aabb is not None and dial_aabb is not None:
        zodiac_span_x = _span(zodiac_aabb, 0)
        chapter_span_x = _span(chapter_aabb, 0)
        zodiac_center_x = 0.5 * (zodiac_aabb[0][0] + zodiac_aabb[1][0])
        chapter_center_x = 0.5 * (chapter_aabb[0][0] + chapter_aabb[1][0])
        zodiac_center_z = 0.5 * (zodiac_aabb[0][2] + zodiac_aabb[1][2])
        chapter_center_z = 0.5 * (chapter_aabb[0][2] + chapter_aabb[1][2])
        dial_span_x = _span(dial_aabb, 0)
        ctx.check(
            "zodiac ring surrounds the inner chapter ring concentrically",
            zodiac_span_x is not None
            and chapter_span_x is not None
            and dial_span_x is not None
            and zodiac_span_x > chapter_span_x + 1.0
            and dial_span_x > zodiac_span_x + 0.8
            and abs(zodiac_center_x - chapter_center_x) < 1e-6
            and abs(zodiac_center_z - chapter_center_z) < 1e-6,
            details=(
                f"zodiac_span_x={zodiac_span_x}, chapter_span_x={chapter_span_x}, "
                f"dial_span_x={dial_span_x}, zodiac_center=({zodiac_center_x}, {zodiac_center_z}), "
                f"chapter_center=({chapter_center_x}, {chapter_center_z})"
            ),
        )

    ctx.check(
        "clock hands rotate about the dial normal",
        tuple(hour_joint.axis) == (0.0, -1.0, 0.0) and tuple(minute_joint.axis) == (0.0, -1.0, 0.0),
        details=f"hour_axis={hour_joint.axis}, minute_axis={minute_joint.axis}",
    )

    hour_rest = ctx.part_element_world_aabb(hour_hand, elem="hour_blade")
    minute_rest = ctx.part_element_world_aabb(minute_hand, elem="minute_blade")
    with ctx.pose({hour_joint: pi / 2.0, minute_joint: pi / 2.0}):
        hour_quarter = ctx.part_element_world_aabb(hour_hand, elem="hour_blade")
        minute_quarter = ctx.part_element_world_aabb(minute_hand, elem="minute_blade")

    ctx.check(
        "hour hand sweeps across the dial plane",
        hour_rest is not None
        and hour_quarter is not None
        and (_span(hour_rest, 2) or 0.0) > (_span(hour_rest, 0) or 0.0)
        and (_span(hour_quarter, 0) or 0.0) > (_span(hour_quarter, 2) or 0.0),
        details=f"rest={hour_rest}, quarter_turn={hour_quarter}",
    )
    ctx.check(
        "minute hand sweeps across the dial plane",
        minute_rest is not None
        and minute_quarter is not None
        and (_span(minute_rest, 2) or 0.0) > (_span(minute_rest, 0) or 0.0)
        and (_span(minute_quarter, 0) or 0.0) > (_span(minute_quarter, 2) or 0.0),
        details=f"rest={minute_rest}, quarter_turn={minute_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
