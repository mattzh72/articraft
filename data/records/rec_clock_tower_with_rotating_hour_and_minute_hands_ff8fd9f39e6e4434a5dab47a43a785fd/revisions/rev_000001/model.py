from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PLINTH_SIZE = 3.8
PLINTH_HEIGHT = 0.70
SHAFT_SIZE = 2.8
SHAFT_HEIGHT = 5.00
CROWN_SIZE = 3.2
CROWN_HEIGHT = 0.35

FIRST_STAGE_WIDTH = 3.00
FIRST_STAGE_HEIGHT = 1.25
SECOND_STAGE_WIDTH = 2.20
SECOND_STAGE_HEIGHT = 0.92
THIRD_STAGE_WIDTH = 1.55
THIRD_STAGE_HEIGHT = 0.75

FIRST_ROOF_WIDTH = 4.60
FIRST_ROOF_HEIGHT = 1.15
SECOND_ROOF_WIDTH = 3.35
SECOND_ROOF_HEIGHT = 0.95
THIRD_ROOF_WIDTH = 2.45
THIRD_ROOF_HEIGHT = 0.82

DIAL_RADIUS = 0.55
DIAL_BEZEL_DEPTH = 0.06
DIAL_FACE_DEPTH = 0.018
def _octagon_profile(face_to_face: float) -> list[tuple[float, float]]:
    half_span = face_to_face * 0.5
    corner = half_span * (sqrt(2.0) - 1.0)
    return [
        (corner, half_span),
        (half_span, corner),
        (half_span, -corner),
        (corner, -half_span),
        (-corner, -half_span),
        (-half_span, -corner),
        (-half_span, corner),
        (-corner, half_span),
    ]


def _roof_mesh(
    *,
    eave_width: float,
    height: float,
    top_width: float,
    corner_lift: float,
):
    lower_eave = ExtrudeGeometry.from_z0(_octagon_profile(eave_width), height * 0.10)
    lower_slope = ExtrudeGeometry.from_z0(_octagon_profile(eave_width * 0.93), height * 0.18)
    lower_eave.merge(lower_slope.translate(0.0, 0.0, height * 0.08))

    middle_width = (eave_width + top_width) * 0.62
    middle_slope = ExtrudeGeometry.from_z0(_octagon_profile(middle_width), height * 0.26)
    lower_eave.merge(middle_slope.translate(0.0, 0.0, height * 0.26))

    upper_slope = ExtrudeGeometry.from_z0(_octagon_profile(top_width * 1.25), height * 0.20)
    lower_eave.merge(upper_slope.translate(0.0, 0.0, height * 0.58))

    crown = ExtrudeGeometry.from_z0(_octagon_profile(top_width), height * 0.22)
    lower_eave.merge(crown.translate(0.0, 0.0, height * 0.78))

    corner_block_size = max(0.08, corner_lift * 0.9)
    corner_offset = eave_width * 0.5 - corner_block_size * 0.6
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            lower_eave.merge(
                ExtrudeGeometry.from_z0(
                    _octagon_profile(corner_block_size),
                    height * 0.12,
                ).translate(
                    x_sign * corner_offset * 0.72,
                    y_sign * corner_offset * 0.72,
                    height * 0.02,
                )
            )

    return lower_eave


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_octagonal_stage(
    part,
    *,
    name_prefix: str,
    width: float,
    height: float,
    material,
) -> None:
    lower_band_height = min(0.16, height * 0.18)
    upper_band_height = min(0.14, height * 0.16)
    wall_height = height - lower_band_height - upper_band_height + 0.04
    part.visual(
        _mesh(
            f"{name_prefix}_lower_band",
            ExtrudeGeometry.from_z0(_octagon_profile(width * 1.04), lower_band_height),
        ),
        material=material,
        name=f"{name_prefix}_lower_band",
    )
    part.visual(
        _mesh(
            f"{name_prefix}_wall",
            ExtrudeGeometry.from_z0(_octagon_profile(width), wall_height),
        ),
        origin=Origin(xyz=(0.0, 0.0, lower_band_height - 0.02)),
        material=material,
        name=f"{name_prefix}_wall",
    )
    part.visual(
        _mesh(
            f"{name_prefix}_upper_band",
            ExtrudeGeometry.from_z0(_octagon_profile(width * 1.03), upper_band_height),
        ),
        origin=Origin(xyz=(0.0, 0.0, height - upper_band_height)),
        material=material,
        name=f"{name_prefix}_upper_band",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="japanese_pagoda_clock_tower")

    timber_dark = model.material("timber_dark", rgba=(0.42, 0.26, 0.15, 1.0))
    timber_light = model.material("timber_light", rgba=(0.55, 0.36, 0.22, 1.0))
    roof_tile = model.material("roof_tile", rgba=(0.18, 0.23, 0.20, 1.0))
    roof_trim = model.material("roof_trim", rgba=(0.23, 0.28, 0.24, 1.0))
    clock_face = model.material("clock_face", rgba=(0.94, 0.92, 0.86, 1.0))
    clock_bezel = model.material("clock_bezel", rgba=(0.52, 0.39, 0.18, 1.0))
    hand_dark = model.material("hand_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    finial_metal = model.material("finial_metal", rgba=(0.76, 0.63, 0.28, 1.0))

    tower_shaft = model.part("tower_shaft")
    tower_shaft.visual(
        Box((PLINTH_SIZE, PLINTH_SIZE, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
        material=timber_light,
        name="plinth",
    )
    tower_shaft.visual(
        Box((SHAFT_SIZE, SHAFT_SIZE, SHAFT_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + SHAFT_HEIGHT * 0.5)),
        material=timber_dark,
        name="shaft_body",
    )
    tower_shaft.visual(
        Box((CROWN_SIZE, CROWN_SIZE, CROWN_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, PLINTH_HEIGHT + SHAFT_HEIGHT + CROWN_HEIGHT * 0.5),
        ),
        material=timber_light,
        name="shaft_crown",
    )
    pilaster_offset = SHAFT_SIZE * 0.5 - 0.16
    pilaster_height = SHAFT_HEIGHT * 0.98
    pilaster_center_z = PLINTH_HEIGHT + pilaster_height * 0.5
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower_shaft.visual(
                Box((0.24, 0.24, pilaster_height)),
                origin=Origin(
                    xyz=(x_sign * pilaster_offset, y_sign * pilaster_offset, pilaster_center_z),
                ),
                material=timber_light,
            )
    tower_shaft.inertial = Inertial.from_geometry(
        Box((PLINTH_SIZE, PLINTH_SIZE, PLINTH_HEIGHT + SHAFT_HEIGHT + CROWN_HEIGHT)),
        mass=4200.0,
        origin=Origin(
            xyz=(0.0, 0.0, (PLINTH_HEIGHT + SHAFT_HEIGHT + CROWN_HEIGHT) * 0.5),
        ),
    )

    first_stage = model.part("first_stage")
    _add_octagonal_stage(
        first_stage,
        name_prefix="first_stage",
        width=FIRST_STAGE_WIDTH,
        height=FIRST_STAGE_HEIGHT,
        material=timber_dark,
    )
    first_stage.inertial = Inertial.from_geometry(
        Box((FIRST_STAGE_WIDTH, FIRST_STAGE_WIDTH, FIRST_STAGE_HEIGHT)),
        mass=480.0,
        origin=Origin(xyz=(0.0, 0.0, FIRST_STAGE_HEIGHT * 0.5)),
    )

    first_roof = model.part("first_roof")
    first_roof.visual(
        _mesh(
            "first_roof_shell",
            _roof_mesh(
                eave_width=FIRST_ROOF_WIDTH,
                height=FIRST_ROOF_HEIGHT,
                top_width=1.55,
                corner_lift=0.15,
            ),
        ),
        material=roof_tile,
        name="first_roof_shell",
    )
    first_roof.visual(
        _mesh(
            "first_roof_cap",
            ExtrudeGeometry.from_z0(_octagon_profile(1.20), 0.08),
        ),
        origin=Origin(xyz=(0.0, 0.0, FIRST_ROOF_HEIGHT - 0.08)),
        material=roof_trim,
        name="first_roof_cap",
    )
    first_roof.inertial = Inertial.from_geometry(
        Box((FIRST_ROOF_WIDTH, FIRST_ROOF_WIDTH, FIRST_ROOF_HEIGHT)),
        mass=360.0,
        origin=Origin(xyz=(0.0, 0.0, FIRST_ROOF_HEIGHT * 0.5)),
    )

    second_stage = model.part("second_stage")
    _add_octagonal_stage(
        second_stage,
        name_prefix="second_stage",
        width=SECOND_STAGE_WIDTH,
        height=SECOND_STAGE_HEIGHT,
        material=timber_dark,
    )
    second_stage.inertial = Inertial.from_geometry(
        Box((SECOND_STAGE_WIDTH, SECOND_STAGE_WIDTH, SECOND_STAGE_HEIGHT)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, SECOND_STAGE_HEIGHT * 0.5)),
    )

    second_roof = model.part("second_roof")
    second_roof.visual(
        _mesh(
            "second_roof_shell",
            _roof_mesh(
                eave_width=SECOND_ROOF_WIDTH,
                height=SECOND_ROOF_HEIGHT,
                top_width=1.10,
                corner_lift=0.13,
            ),
        ),
        material=roof_tile,
        name="second_roof_shell",
    )
    second_roof.visual(
        _mesh(
            "second_roof_cap",
            ExtrudeGeometry.from_z0(_octagon_profile(0.86), 0.07),
        ),
        origin=Origin(xyz=(0.0, 0.0, SECOND_ROOF_HEIGHT - 0.07)),
        material=roof_trim,
        name="second_roof_cap",
    )
    second_roof.inertial = Inertial.from_geometry(
        Box((SECOND_ROOF_WIDTH, SECOND_ROOF_WIDTH, SECOND_ROOF_HEIGHT)),
        mass=210.0,
        origin=Origin(xyz=(0.0, 0.0, SECOND_ROOF_HEIGHT * 0.5)),
    )

    third_stage = model.part("third_stage")
    _add_octagonal_stage(
        third_stage,
        name_prefix="third_stage",
        width=THIRD_STAGE_WIDTH,
        height=THIRD_STAGE_HEIGHT,
        material=timber_dark,
    )
    third_stage.inertial = Inertial.from_geometry(
        Box((THIRD_STAGE_WIDTH, THIRD_STAGE_WIDTH, THIRD_STAGE_HEIGHT)),
        mass=125.0,
        origin=Origin(xyz=(0.0, 0.0, THIRD_STAGE_HEIGHT * 0.5)),
    )

    third_roof = model.part("third_roof")
    third_roof.visual(
        _mesh(
            "third_roof_shell",
            _roof_mesh(
                eave_width=THIRD_ROOF_WIDTH,
                height=THIRD_ROOF_HEIGHT,
                top_width=0.82,
                corner_lift=0.10,
            ),
        ),
        material=roof_tile,
        name="third_roof_shell",
    )
    third_roof.visual(
        _mesh(
            "third_roof_cap",
            ExtrudeGeometry.from_z0(_octagon_profile(0.62), 0.06),
        ),
        origin=Origin(xyz=(0.0, 0.0, THIRD_ROOF_HEIGHT - 0.06)),
        material=roof_trim,
        name="third_roof_cap",
    )
    third_roof.inertial = Inertial.from_geometry(
        Box((THIRD_ROOF_WIDTH, THIRD_ROOF_WIDTH, THIRD_ROOF_HEIGHT)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, THIRD_ROOF_HEIGHT * 0.5)),
    )

    finial = model.part("finial")
    finial.visual(
        Cylinder(radius=0.14, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=finial_metal,
        name="finial_base",
    )
    finial.visual(
        Sphere(radius=0.13),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=finial_metal,
        name="finial_globe",
    )
    finial.visual(
        Cylinder(radius=0.06, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=finial_metal,
        name="finial_spire",
    )
    finial.visual(
        Sphere(radius=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        material=finial_metal,
        name="finial_tip",
    )
    finial.inertial = Inertial.from_geometry(
        Box((0.30, 0.30, 0.88)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
    )

    dial = model.part("clock_dial")
    dial.visual(
        Cylinder(radius=DIAL_RADIUS + 0.07, length=DIAL_BEZEL_DEPTH),
        origin=Origin(xyz=(0.0, DIAL_BEZEL_DEPTH * 0.5, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=clock_bezel,
        name="dial_bezel",
    )
    dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_FACE_DEPTH),
        origin=Origin(
            xyz=(0.0, DIAL_BEZEL_DEPTH - DIAL_FACE_DEPTH * 0.5, 0.0),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=clock_face,
        name="dial_face",
    )
    dial.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.0, 0.050, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=clock_bezel,
        name="dial_hub",
    )
    dial.visual(
        Box((0.08, 0.008, 0.035)),
        origin=Origin(xyz=(0.0, 0.046, 0.43)),
        material=clock_bezel,
        name="marker_12",
    )
    dial.visual(
        Box((0.08, 0.008, 0.035)),
        origin=Origin(xyz=(0.0, 0.046, -0.43)),
        material=clock_bezel,
        name="marker_6",
    )
    dial.visual(
        Box((0.035, 0.008, 0.08)),
        origin=Origin(xyz=(0.43, 0.046, 0.0)),
        material=clock_bezel,
        name="marker_3",
    )
    dial.visual(
        Box((0.035, 0.008, 0.08)),
        origin=Origin(xyz=(-0.43, 0.046, 0.0)),
        material=clock_bezel,
        name="marker_9",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=DIAL_RADIUS + 0.07, length=DIAL_BEZEL_DEPTH),
        mass=24.0,
        origin=Origin(xyz=(0.0, DIAL_BEZEL_DEPTH * 0.5, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=hand_dark,
        name="hour_sleeve",
    )
    hour_hand.visual(
        Cylinder(radius=0.046, length=0.004),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=hand_dark,
        name="hour_hub_cap",
    )
    hour_hand.visual(
        Box((0.055, 0.006, 0.34)),
        origin=Origin(xyz=(0.0, 0.003, 0.17)),
        material=hand_dark,
        name="hour_blade",
    )
    hour_hand.visual(
        Box((0.030, 0.006, 0.11)),
        origin=Origin(xyz=(0.0, 0.003, -0.055)),
        material=hand_dark,
        name="hour_tail",
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.10, 0.010, 0.46)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.005, 0.06)),
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=hand_dark,
        name="minute_collar",
    )
    minute_hand.visual(
        Cylinder(radius=0.032, length=0.004),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=hand_dark,
        name="minute_hub_cap",
    )
    minute_hand.visual(
        Box((0.48, 0.005, 0.040)),
        origin=Origin(xyz=(0.24, 0.017, 0.0)),
        material=hand_dark,
        name="minute_blade",
    )
    minute_hand.visual(
        Box((0.14, 0.005, 0.026)),
        origin=Origin(xyz=(-0.07, 0.017, 0.0)),
        material=hand_dark,
        name="minute_tail",
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.62, 0.024, 0.08)),
        mass=0.9,
        origin=Origin(xyz=(0.17, 0.015, 0.0)),
    )

    shaft_top = PLINTH_HEIGHT + SHAFT_HEIGHT + CROWN_HEIGHT

    model.articulation(
        "shaft_to_first_stage",
        ArticulationType.FIXED,
        parent=tower_shaft,
        child=first_stage,
        origin=Origin(xyz=(0.0, 0.0, shaft_top)),
    )
    model.articulation(
        "first_stage_to_first_roof",
        ArticulationType.FIXED,
        parent=first_stage,
        child=first_roof,
        origin=Origin(xyz=(0.0, 0.0, FIRST_STAGE_HEIGHT)),
    )
    model.articulation(
        "first_roof_to_second_stage",
        ArticulationType.FIXED,
        parent=first_roof,
        child=second_stage,
        origin=Origin(xyz=(0.0, 0.0, FIRST_ROOF_HEIGHT)),
    )
    model.articulation(
        "second_stage_to_second_roof",
        ArticulationType.FIXED,
        parent=second_stage,
        child=second_roof,
        origin=Origin(xyz=(0.0, 0.0, SECOND_STAGE_HEIGHT)),
    )
    model.articulation(
        "second_roof_to_third_stage",
        ArticulationType.FIXED,
        parent=second_roof,
        child=third_stage,
        origin=Origin(xyz=(0.0, 0.0, SECOND_ROOF_HEIGHT)),
    )
    model.articulation(
        "third_stage_to_third_roof",
        ArticulationType.FIXED,
        parent=third_stage,
        child=third_roof,
        origin=Origin(xyz=(0.0, 0.0, THIRD_STAGE_HEIGHT)),
    )
    model.articulation(
        "third_roof_to_finial",
        ArticulationType.FIXED,
        parent=third_roof,
        child=finial,
        origin=Origin(xyz=(0.0, 0.0, THIRD_ROOF_HEIGHT)),
    )
    model.articulation(
        "first_stage_to_clock_dial",
        ArticulationType.FIXED,
        parent=first_stage,
        child=dial,
        origin=Origin(xyz=(0.0, FIRST_STAGE_WIDTH * 0.52, 0.62)),
    )
    model.articulation(
        "dial_to_hour_hand",
        ArticulationType.CONTINUOUS,
        parent=dial,
        child=hour_hand,
        origin=Origin(xyz=(0.0, DIAL_BEZEL_DEPTH, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=1.0),
    )
    model.articulation(
        "dial_to_minute_hand",
        ArticulationType.CONTINUOUS,
        parent=dial,
        child=minute_hand,
        origin=Origin(xyz=(0.0, DIAL_BEZEL_DEPTH, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_shaft = object_model.get_part("tower_shaft")
    first_stage = object_model.get_part("first_stage")
    first_roof = object_model.get_part("first_roof")
    second_stage = object_model.get_part("second_stage")
    second_roof = object_model.get_part("second_roof")
    third_stage = object_model.get_part("third_stage")
    third_roof = object_model.get_part("third_roof")
    finial = object_model.get_part("finial")
    dial = object_model.get_part("clock_dial")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")

    hour_joint = object_model.get_articulation("dial_to_hour_hand")
    minute_joint = object_model.get_articulation("dial_to_minute_hand")

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

    ctx.expect_contact(first_stage, tower_shaft)
    ctx.expect_contact(first_roof, first_stage)
    ctx.expect_contact(second_stage, first_roof)
    ctx.expect_contact(second_roof, second_stage)
    ctx.expect_contact(third_stage, second_roof)
    ctx.expect_contact(third_roof, third_stage)
    ctx.expect_contact(finial, third_roof)

    ctx.expect_contact(dial, first_stage)
    ctx.expect_within(hour_hand, dial, axes="xz", margin=0.03)
    ctx.expect_within(minute_hand, dial, axes="xz", margin=0.03)
    ctx.expect_contact(hour_hand, dial)
    ctx.expect_contact(minute_hand, hour_hand, elem_a="minute_collar", elem_b="hour_sleeve")
    ctx.expect_gap(minute_hand, dial, axis="y", min_gap=0.01, max_gap=0.03)
    ctx.expect_gap(
        minute_hand,
        hour_hand,
        axis="y",
        positive_elem="minute_blade",
        negative_elem="hour_blade",
        min_gap=0.008,
        max_gap=0.03,
    )

    ctx.check(
        "hour_hand_axis_is_face_normal",
        tuple(hour_joint.axis) == (0.0, 1.0, 0.0),
        details=f"Expected hour hand axis (0, 1, 0), got {hour_joint.axis}",
    )
    ctx.check(
        "minute_hand_axis_is_face_normal",
        tuple(minute_joint.axis) == (0.0, 1.0, 0.0),
        details=f"Expected minute hand axis (0, 1, 0), got {minute_joint.axis}",
    )

    minute_rest = ctx.part_world_aabb(minute_hand)
    hour_rest = ctx.part_world_aabb(hour_hand)
    assert minute_rest is not None
    assert hour_rest is not None
    with ctx.pose({"dial_to_minute_hand": pi * 0.5, "dial_to_hour_hand": -pi * 0.5}):
        minute_turned = ctx.part_world_aabb(minute_hand)
        hour_turned = ctx.part_world_aabb(hour_hand)
        assert minute_turned is not None
        assert hour_turned is not None
        minute_rest_x = minute_rest[1][0] - minute_rest[0][0]
        minute_rest_z = minute_rest[1][2] - minute_rest[0][2]
        minute_turned_x = minute_turned[1][0] - minute_turned[0][0]
        minute_turned_z = minute_turned[1][2] - minute_turned[0][2]
        hour_rest_x = hour_rest[1][0] - hour_rest[0][0]
        hour_rest_z = hour_rest[1][2] - hour_rest[0][2]
        hour_turned_x = hour_turned[1][0] - hour_turned[0][0]
        hour_turned_z = hour_turned[1][2] - hour_turned[0][2]
        ctx.check(
            "minute_hand_rotates_around_hub",
            minute_rest_x > minute_rest_z and minute_turned_z > minute_turned_x,
            details=(
                f"Minute hand extents did not swap from horizontal to vertical: "
                f"rest=({minute_rest_x:.3f}, {minute_rest_z:.3f}) "
                f"turned=({minute_turned_x:.3f}, {minute_turned_z:.3f})"
            ),
        )
        ctx.check(
            "hour_hand_rotates_around_hub",
            hour_rest_z > hour_rest_x and hour_turned_x > hour_turned_z,
            details=(
                f"Hour hand extents did not swap from vertical to horizontal: "
                f"rest=({hour_rest_x:.3f}, {hour_rest_z:.3f}) "
                f"turned=({hour_turned_x:.3f}, {hour_turned_z:.3f})"
            ),
        )
        ctx.expect_within(hour_hand, dial, axes="xz", margin=0.03)
        ctx.expect_within(minute_hand, dial, axes="xz", margin=0.03)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
