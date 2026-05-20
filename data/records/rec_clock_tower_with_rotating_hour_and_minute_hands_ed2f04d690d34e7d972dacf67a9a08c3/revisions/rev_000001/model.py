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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PLINTH_BASE_SIZE = 4.8
PLINTH_BASE_HEIGHT = 0.9
PLINTH_CAP_SIZE = 3.4
PLINTH_CAP_HEIGHT = 0.3

SHAFT_WIDTH = 2.4
SHAFT_HEIGHT = 14.0
POST_SIZE = 0.18
BEAM_HEIGHT = 0.30
GLASS_THICKNESS = 0.03

FACE_RADIUS = 1.02
FACE_THICKNESS = 0.10
FACE_ANCHOR_THICKNESS = 0.04
FACE_STANDOFF_LENGTH = 0.24
FACE_CENTER_HEIGHT = 11.3
FACE_OUTSET = FACE_ANCHOR_THICKNESS + FACE_STANDOFF_LENGTH + FACE_THICKNESS / 2.0

CENTER_CAP_RADIUS = 0.13
CENTER_CAP_LENGTH = 0.012

HOUR_HAND_LENGTH = 0.62
HOUR_HAND_WIDTH = 0.12
HOUR_HAND_THICKNESS = 0.018
HOUR_TAIL_LENGTH = 0.14
HOUR_TAIL_WIDTH = 0.055
HOUR_RAIL_WIDTH = 0.028
HOUR_RAIL_OFFSET = 0.061
HOUR_BRIDGE_WIDTH = 0.15
HOUR_BRIDGE_DEPTH = 0.03
HOUR_HAND_FRONT = FACE_THICKNESS / 2.0 + HOUR_HAND_THICKNESS
ARBOR_BUSHING_RADIUS = 0.045
ARBOR_BUSHING_LENGTH = HOUR_HAND_FRONT - FACE_THICKNESS / 2.0

MINUTE_SPINDLE_RADIUS = 0.065
MINUTE_SPINDLE_LENGTH = 0.05
MINUTE_SPINDLE_FRONT = HOUR_HAND_FRONT + MINUTE_SPINDLE_LENGTH

MINUTE_HAND_LENGTH = 0.86
MINUTE_HAND_WIDTH = 0.08
MINUTE_HAND_THICKNESS = 0.014
MINUTE_TAIL_LENGTH = 0.18
MINUTE_TAIL_WIDTH = 0.035


def _add_clock_face(
    model: ArticulatedObject,
    shaft,
    *,
    prefix: str,
    face_origin: tuple[float, float, float],
    face_rpy: tuple[float, float, float],
    steel,
    face_panel,
    hand_finish,
) -> None:
    face = model.part(f"{prefix}_clock_face")
    face.visual(
        Cylinder(radius=FACE_RADIUS, length=FACE_THICKNESS),
        material=face_panel,
        name="disc",
    )
    face.visual(
        Box((0.42, 0.42, FACE_ANCHOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -(FACE_OUTSET - FACE_ANCHOR_THICKNESS / 2.0))),
        material=steel,
        name="anchor_plate",
    )
    face.visual(
        Box((0.16, 0.16, FACE_STANDOFF_LENGTH)),
        origin=Origin(
            xyz=(0.0, 0.0, -(FACE_THICKNESS / 2.0 + FACE_STANDOFF_LENGTH / 2.0))
        ),
        material=steel,
        name="standoff",
    )
    face.visual(
        Cylinder(radius=CENTER_CAP_RADIUS, length=CENTER_CAP_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, FACE_THICKNESS / 2.0 - CENTER_CAP_LENGTH / 2.0)
        ),
        material=steel,
        name="center_cap",
    )

    model.articulation(
        f"shaft_to_{prefix}_clock_face",
        ArticulationType.FIXED,
        parent=shaft,
        child=face,
        origin=Origin(xyz=face_origin, rpy=face_rpy),
    )

    hour_hand = model.part(f"{prefix}_hour_hand")
    hour_hand_z = FACE_THICKNESS / 2.0 + HOUR_HAND_THICKNESS / 2.0
    hour_rail_length = HOUR_HAND_LENGTH + HOUR_TAIL_LENGTH
    hour_rail_center_y = (HOUR_HAND_LENGTH - HOUR_TAIL_LENGTH) / 2.0
    for side, x_pos in (("left", -HOUR_RAIL_OFFSET), ("right", HOUR_RAIL_OFFSET)):
        hour_hand.visual(
            Box((HOUR_RAIL_WIDTH, hour_rail_length, HOUR_HAND_THICKNESS)),
            origin=Origin(xyz=(x_pos, hour_rail_center_y, hour_hand_z)),
            material=hand_finish,
            name=f"{side}_rail",
        )
    hour_hand.visual(
        Box((HOUR_BRIDGE_WIDTH, HOUR_BRIDGE_DEPTH, HOUR_HAND_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.085, hour_hand_z)),
        material=hand_finish,
        name="blade_bridge",
    )
    hour_hand.visual(
        Box((HOUR_BRIDGE_WIDTH, HOUR_BRIDGE_DEPTH, HOUR_HAND_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.085, hour_hand_z)),
        material=hand_finish,
        name="counterweight_bridge",
    )
    model.articulation(
        f"{prefix}_clock_face_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=face,
        child=hour_hand,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.0,
            lower=0.0,
            upper=2.0 * math.pi,
        ),
    )

    minute_arbor = model.part(f"{prefix}_minute_arbor")
    minute_arbor.visual(
        Cylinder(radius=ARBOR_BUSHING_RADIUS, length=ARBOR_BUSHING_LENGTH),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                FACE_THICKNESS / 2.0 + ARBOR_BUSHING_LENGTH / 2.0,
            )
        ),
        material=steel,
        name="bushing",
    )
    minute_arbor.visual(
        Cylinder(radius=MINUTE_SPINDLE_RADIUS, length=MINUTE_SPINDLE_LENGTH),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                HOUR_HAND_FRONT + MINUTE_SPINDLE_LENGTH / 2.0,
            )
        ),
        material=steel,
        name="spindle",
    )
    model.articulation(
        f"{prefix}_clock_face_to_minute_arbor",
        ArticulationType.FIXED,
        parent=face,
        child=minute_arbor,
        origin=Origin(),
    )

    minute_hand = model.part(f"{prefix}_minute_hand")
    minute_hand_z = MINUTE_SPINDLE_FRONT + MINUTE_HAND_THICKNESS / 2.0
    minute_hand.visual(
        Cylinder(radius=0.065, length=MINUTE_HAND_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, minute_hand_z)),
        material=hand_finish,
        name="hub",
    )
    minute_hand.visual(
        Box((MINUTE_HAND_WIDTH, MINUTE_HAND_LENGTH, MINUTE_HAND_THICKNESS)),
        origin=Origin(xyz=(0.0, MINUTE_HAND_LENGTH / 2.0, minute_hand_z)),
        material=hand_finish,
        name="blade",
    )
    minute_hand.visual(
        Box((MINUTE_TAIL_WIDTH, MINUTE_TAIL_LENGTH, MINUTE_HAND_THICKNESS)),
        origin=Origin(xyz=(0.0, -MINUTE_TAIL_LENGTH / 2.0, minute_hand_z)),
        material=hand_finish,
        name="counterweight",
    )
    model.articulation(
        f"{prefix}_minute_arbor_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=minute_arbor,
        child=minute_hand,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.5,
            lower=0.0,
            upper=2.0 * math.pi,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_civic_clock_tower")

    concrete = model.material("concrete", rgba=(0.68, 0.68, 0.70, 1.0))
    steel = model.material("steel", rgba=(0.23, 0.25, 0.28, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.83, 0.90, 0.35))
    face_panel = model.material("clock_face", rgba=(0.94, 0.96, 0.97, 1.0))
    hand_finish = model.material("hands", rgba=(0.10, 0.11, 0.12, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        Box((PLINTH_BASE_SIZE, PLINTH_BASE_SIZE, PLINTH_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_BASE_HEIGHT / 2.0)),
        material=concrete,
        name="base_block",
    )
    plinth.visual(
        Box((PLINTH_CAP_SIZE, PLINTH_CAP_SIZE, PLINTH_CAP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                PLINTH_BASE_HEIGHT + PLINTH_CAP_HEIGHT / 2.0,
            )
        ),
        material=concrete,
        name="cap_block",
    )

    shaft = model.part("shaft")
    half_width = SHAFT_WIDTH / 2.0
    post_offset = half_width - POST_SIZE / 2.0
    panel_clear_span = SHAFT_WIDTH - 2.0 * POST_SIZE
    panel_height = SHAFT_HEIGHT - 2.0 * BEAM_HEIGHT
    panel_center_z = BEAM_HEIGHT + panel_height / 2.0
    beam_z_positions = (BEAM_HEIGHT / 2.0, SHAFT_HEIGHT - BEAM_HEIGHT / 2.0)

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            shaft.visual(
                Box((POST_SIZE, POST_SIZE, SHAFT_HEIGHT)),
                origin=Origin(xyz=(x_sign * post_offset, y_sign * post_offset, SHAFT_HEIGHT / 2.0)),
                material=steel,
                name=f"post_{'e' if x_sign > 0 else 'w'}{'n' if y_sign > 0 else 's'}",
            )

    for beam_z in beam_z_positions:
        shaft.visual(
            Box((panel_clear_span, POST_SIZE, BEAM_HEIGHT)),
            origin=Origin(xyz=(0.0, post_offset, beam_z)),
            material=steel,
            name=f"beam_n_{beam_z:.2f}",
        )
        shaft.visual(
            Box((panel_clear_span, POST_SIZE, BEAM_HEIGHT)),
            origin=Origin(xyz=(0.0, -post_offset, beam_z)),
            material=steel,
            name=f"beam_s_{beam_z:.2f}",
        )
        shaft.visual(
            Box((POST_SIZE, panel_clear_span, BEAM_HEIGHT)),
            origin=Origin(xyz=(post_offset, 0.0, beam_z)),
            material=steel,
            name=f"beam_e_{beam_z:.2f}",
        )
        shaft.visual(
            Box((POST_SIZE, panel_clear_span, BEAM_HEIGHT)),
            origin=Origin(xyz=(-post_offset, 0.0, beam_z)),
            material=steel,
            name=f"beam_w_{beam_z:.2f}",
        )

    shaft.visual(
        Box((panel_clear_span, GLASS_THICKNESS, panel_height)),
        origin=Origin(xyz=(0.0, half_width - GLASS_THICKNESS / 2.0, panel_center_z)),
        material=glass,
        name="north_glass",
    )
    shaft.visual(
        Box((panel_clear_span, GLASS_THICKNESS, panel_height)),
        origin=Origin(xyz=(0.0, -(half_width - GLASS_THICKNESS / 2.0), panel_center_z)),
        material=glass,
        name="south_glass",
    )
    shaft.visual(
        Box((GLASS_THICKNESS, panel_clear_span, panel_height)),
        origin=Origin(xyz=(half_width - GLASS_THICKNESS / 2.0, 0.0, panel_center_z)),
        material=glass,
        name="east_glass",
    )
    shaft.visual(
        Box((GLASS_THICKNESS, panel_clear_span, panel_height)),
        origin=Origin(xyz=(-(half_width - GLASS_THICKNESS / 2.0), 0.0, panel_center_z)),
        material=glass,
        name="west_glass",
    )

    model.articulation(
        "plinth_to_shaft",
        ArticulationType.FIXED,
        parent=plinth,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, PLINTH_BASE_HEIGHT + PLINTH_CAP_HEIGHT)),
    )

    side_configs = (
        ("north", (0.0, half_width + FACE_OUTSET, FACE_CENTER_HEIGHT), (math.pi / 2.0, 0.0, math.pi)),
        ("south", (0.0, -(half_width + FACE_OUTSET), FACE_CENTER_HEIGHT), (math.pi / 2.0, 0.0, 0.0)),
        ("east", (half_width + FACE_OUTSET, 0.0, FACE_CENTER_HEIGHT), (math.pi / 2.0, 0.0, math.pi / 2.0)),
        ("west", (-(half_width + FACE_OUTSET), 0.0, FACE_CENTER_HEIGHT), (math.pi / 2.0, 0.0, -math.pi / 2.0)),
    )

    for prefix, face_origin, face_rpy in side_configs:
        _add_clock_face(
            model,
            shaft,
            prefix=prefix,
            face_origin=face_origin,
            face_rpy=face_rpy,
            steel=steel,
            face_panel=face_panel,
            hand_finish=hand_finish,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    plinth = object_model.get_part("plinth")
    shaft = object_model.get_part("shaft")
    north_face = object_model.get_part("north_clock_face")
    south_face = object_model.get_part("south_clock_face")
    east_face = object_model.get_part("east_clock_face")
    west_face = object_model.get_part("west_clock_face")
    north_hour = object_model.get_part("north_hour_hand")
    north_minute = object_model.get_part("north_minute_hand")
    north_arbor = object_model.get_part("north_minute_arbor")
    south_hour = object_model.get_part("south_hour_hand")
    south_minute = object_model.get_part("south_minute_hand")
    south_arbor = object_model.get_part("south_minute_arbor")
    east_hour = object_model.get_part("east_hour_hand")
    east_minute = object_model.get_part("east_minute_hand")
    east_arbor = object_model.get_part("east_minute_arbor")
    west_hour = object_model.get_part("west_hour_hand")
    west_minute = object_model.get_part("west_minute_hand")
    west_arbor = object_model.get_part("west_minute_arbor")

    north_minute_joint = object_model.get_articulation("north_minute_arbor_to_minute_hand")
    east_minute_joint = object_model.get_articulation("east_minute_arbor_to_minute_hand")

    ctx.expect_contact(shaft, plinth, name="glass shaft is seated on the plinth")

    for face_name, face in (
        ("north", north_face),
        ("south", south_face),
        ("east", east_face),
        ("west", west_face),
    ):
        ctx.expect_contact(face, shaft, name=f"{face_name} clock disc cantilevers from the shaft")

    for arbor_name, arbor, face in (
        ("north", north_arbor, north_face),
        ("south", south_arbor, south_face),
        ("east", east_arbor, east_face),
        ("west", west_arbor, west_face),
    ):
        ctx.expect_contact(arbor, face, name=f"{arbor_name} minute arbor is supported by its clock face")

    for hand_name, hand, support in (
        ("north hour", north_hour, north_face),
        ("north minute", north_minute, north_arbor),
        ("south hour", south_hour, south_face),
        ("south minute", south_minute, south_arbor),
        ("east hour", east_hour, east_face),
        ("east minute", east_minute, east_arbor),
        ("west hour", west_hour, west_face),
        ("west minute", west_minute, west_arbor),
    ):
        ctx.expect_contact(hand, support, name=f"{hand_name} hand is mounted on its support")

    def _spans(aabb) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        low, high = aabb
        return (
            high[0] - low[0],
            high[1] - low[1],
            high[2] - low[2],
        )

    north_rest = ctx.part_element_world_aabb(north_minute, elem="blade")
    with ctx.pose({north_minute_joint: math.pi / 2.0}):
        north_turned = ctx.part_element_world_aabb(north_minute, elem="blade")

    north_rest_spans = _spans(north_rest)
    north_turned_spans = _spans(north_turned)
    ctx.check(
        "north minute hand rotates across the north face plane",
        north_rest_spans is not None
        and north_turned_spans is not None
        and north_rest_spans[2] > 0.75
        and north_rest_spans[0] < 0.15
        and north_turned_spans[0] > 0.75
        and north_turned_spans[2] < 0.15,
        details=f"rest={north_rest_spans}, turned={north_turned_spans}",
    )

    east_rest = ctx.part_element_world_aabb(east_minute, elem="blade")
    with ctx.pose({east_minute_joint: math.pi / 2.0}):
        east_turned = ctx.part_element_world_aabb(east_minute, elem="blade")

    east_rest_spans = _spans(east_rest)
    east_turned_spans = _spans(east_turned)
    ctx.check(
        "east minute hand rotates across the east face plane",
        east_rest_spans is not None
        and east_turned_spans is not None
        and east_rest_spans[2] > 0.75
        and east_rest_spans[1] < 0.15
        and east_turned_spans[1] > 0.75
        and east_turned_spans[2] < 0.15,
        details=f"rest={east_rest_spans}, turned={east_turned_spans}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
