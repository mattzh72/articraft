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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_WIDTH = 0.72
BODY_DEPTH = 0.34
BOTTOM_THICKNESS = 0.008
WALL_THICKNESS = 0.012
WALL_HEIGHT = 0.032
TOP_PANEL_THICKNESS = 0.004
TOP_PANEL_CENTER_Z = BOTTOM_THICKNESS + WALL_HEIGHT + (TOP_PANEL_THICKNESS * 0.5)
TOP_SURFACE_Z = TOP_PANEL_CENTER_Z + (TOP_PANEL_THICKNESS * 0.5)

INNER_WIDTH = BODY_WIDTH - (2.0 * WALL_THICKNESS)
INNER_DEPTH = BODY_DEPTH - (2.0 * WALL_THICKNESS)

LEFT_DECK_X = -0.215
RIGHT_DECK_X = 0.215
DECK_CENTER_Y = 0.030
JOG_OPENING_RADIUS = 0.086
JOG_PLATTER_RADIUS = 0.092

CROSSFADER_Y = -0.110
CROSSFADER_SLOT_LENGTH = 0.176
CROSSFADER_SLOT_WIDTH = 0.024

MIXER_ISLAND_WIDTH = 0.168
MIXER_ISLAND_DEPTH = 0.210
MIXER_ISLAND_HEIGHT = 0.006
MIXER_TOP_Z = TOP_SURFACE_Z + MIXER_ISLAND_HEIGHT


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 48,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + (radius * math.cos((2.0 * math.pi * index) / segments)),
            cy + (radius * math.sin((2.0 * math.pi * index) / segments)),
        )
        for index in range(segments)
    ]


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]],
) -> tuple[float, float, float]:
    return tuple(
        (aabb[0][axis] + aabb[1][axis]) * 0.5
        for axis in range(3)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_deck_dj_controller")

    chassis_charcoal = model.material("chassis_charcoal", rgba=(0.13, 0.14, 0.15, 1.0))
    deck_black = model.material("deck_black", rgba=(0.08, 0.09, 0.10, 1.0))
    mixer_black = model.material("mixer_black", rgba=(0.12, 0.12, 0.13, 1.0))
    platter_alloy = model.material("platter_alloy", rgba=(0.67, 0.69, 0.72, 1.0))
    platter_vinyl = model.material("platter_vinyl", rgba=(0.06, 0.06, 0.07, 1.0))
    knob_rubber = model.material("knob_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    rail_grey = model.material("rail_grey", rgba=(0.45, 0.47, 0.50, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.17, 0.37, 0.46, 0.70))
    cue_blue = model.material("cue_blue", rgba=(0.22, 0.52, 0.88, 1.0))
    play_green = model.material("play_green", rgba=(0.24, 0.78, 0.44, 1.0))
    marker_red = model.material("marker_red", rgba=(0.90, 0.22, 0.18, 1.0))
    accent_silver = model.material("accent_silver", rgba=(0.82, 0.84, 0.86, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS * 0.5)),
        material=chassis_charcoal,
        name="bottom_plate",
    )
    chassis.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                -((BODY_WIDTH * 0.5) - (WALL_THICKNESS * 0.5)),
                0.0,
                BOTTOM_THICKNESS + (WALL_HEIGHT * 0.5),
            )
        ),
        material=chassis_charcoal,
        name="left_wall",
    )
    chassis.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                (BODY_WIDTH * 0.5) - (WALL_THICKNESS * 0.5),
                0.0,
                BOTTOM_THICKNESS + (WALL_HEIGHT * 0.5),
            )
        ),
        material=chassis_charcoal,
        name="right_wall",
    )
    chassis.visual(
        Box((INNER_WIDTH, WALL_THICKNESS, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -((BODY_DEPTH * 0.5) - (WALL_THICKNESS * 0.5)),
                BOTTOM_THICKNESS + (WALL_HEIGHT * 0.5),
            )
        ),
        material=chassis_charcoal,
        name="front_wall",
    )
    chassis.visual(
        Box((INNER_WIDTH, WALL_THICKNESS, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_DEPTH * 0.5) - (WALL_THICKNESS * 0.5),
                BOTTOM_THICKNESS + (WALL_HEIGHT * 0.5),
            )
        ),
        material=chassis_charcoal,
        name="rear_wall",
    )

    top_panel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(INNER_WIDTH, INNER_DEPTH, 0.020),
            [
                _circle_profile(
                    JOG_OPENING_RADIUS,
                    center=(LEFT_DECK_X, DECK_CENTER_Y),
                    segments=56,
                ),
                _circle_profile(
                    JOG_OPENING_RADIUS,
                    center=(RIGHT_DECK_X, DECK_CENTER_Y),
                    segments=56,
                ),
                _translate_profile(
                    rounded_rect_profile(
                        CROSSFADER_SLOT_LENGTH,
                        CROSSFADER_SLOT_WIDTH,
                        0.008,
                    ),
                    0.0,
                    CROSSFADER_Y,
                ),
            ],
            TOP_PANEL_THICKNESS,
            cap=True,
            center=True,
        ),
        "dj_controller_top_panel",
    )
    chassis.visual(
        top_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, TOP_PANEL_CENTER_Z)),
        material=deck_black,
        name="top_panel",
    )
    chassis.visual(
        Box((MIXER_ISLAND_WIDTH, MIXER_ISLAND_DEPTH, MIXER_ISLAND_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.020,
                TOP_SURFACE_Z + (MIXER_ISLAND_HEIGHT * 0.5),
            )
        ),
        material=mixer_black,
        name="mixer_island",
    )
    chassis.visual(
        Box((0.120, 0.058, 0.002)),
        origin=Origin(xyz=(0.0, 0.148, MIXER_TOP_Z + 0.001)),
        material=accent_silver,
        name="display_bezel",
    )
    for direction in (-1.0, 1.0):
        x_center = direction * 0.310
        chassis.visual(
            Box((0.012, 0.185, 0.0025)),
            origin=Origin(xyz=(x_center, 0.010, TOP_SURFACE_Z + 0.00125)),
            material=rail_grey,
            name=f"pitch_track_{'left' if direction < 0.0 else 'right'}",
        )
        for pad_column, pad_x in enumerate((0.228, 0.260)):
            for pad_row, pad_y in enumerate((-0.055, -0.088)):
                chassis.visual(
                    Box((0.024, 0.020, 0.004)),
                    origin=Origin(
                        xyz=(
                            direction * pad_x,
                            pad_y,
                            TOP_SURFACE_Z + 0.002,
                        )
                    ),
                    material=deck_black,
                    name=f"pad_{'left' if direction < 0.0 else 'right'}_{pad_row}_{pad_column}",
                )
        chassis.visual(
            Box((0.026, 0.018, 0.004)),
            origin=Origin(
                xyz=(direction * 0.258, -0.132, TOP_SURFACE_Z + 0.002),
            ),
            material=cue_blue,
            name=f"cue_button_{'left' if direction < 0.0 else 'right'}",
        )
        chassis.visual(
            Box((0.026, 0.018, 0.004)),
            origin=Origin(
                xyz=(direction * 0.222, -0.132, TOP_SURFACE_Z + 0.002),
            ),
            material=play_green,
            name=f"play_button_{'left' if direction < 0.0 else 'right'}",
        )

    for strip_index, strip_x in enumerate((-0.036, 0.036)):
        chassis.visual(
            Box((0.010, 0.132, 0.002)),
            origin=Origin(xyz=(strip_x, -0.006, MIXER_TOP_Z + 0.001)),
            material=rail_grey,
            name=f"channel_fader_track_{strip_index}",
        )
    chassis.visual(
        Box((0.008, 0.126, 0.003)),
        origin=Origin(xyz=(0.0, 0.000, MIXER_TOP_Z + 0.0015)),
        material=accent_silver,
        name="meter_strip",
    )

    chassis.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, 0.050)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    for platter_name, platter_x in (
        ("left_platter", LEFT_DECK_X),
        ("right_platter", RIGHT_DECK_X),
    ):
        platter = model.part(platter_name)
        platter.visual(
            Cylinder(radius=JOG_PLATTER_RADIUS, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=platter_alloy,
            name="outer_rim",
        )
        platter.visual(
            Cylinder(radius=0.080, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=platter_vinyl,
            name="vinyl_surface",
        )
        platter.visual(
            Cylinder(radius=0.020, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0155)),
            material=accent_silver,
            name="hub_cap",
        )
        platter.visual(
            Box((0.003, 0.024, 0.0012)),
            origin=Origin(xyz=(0.0, 0.056, 0.0145)),
            material=marker_red,
            name="position_marker",
        )
        platter.inertial = Inertial.from_geometry(
            Cylinder(radius=JOG_PLATTER_RADIUS, length=0.014),
            mass=0.34,
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
        )
        model.articulation(
            f"chassis_to_{platter_name}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=platter,
            origin=Origin(xyz=(platter_x, DECK_CENTER_Y, TOP_SURFACE_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=18.0,
                lower=-(4.0 * math.pi),
                upper=4.0 * math.pi,
            ),
        )

    display = model.part("display_module")
    display.visual(
        Box((0.110, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=deck_black,
        name="display_housing",
    )
    display.visual(
        Box((0.095, 0.036, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.00675)),
        material=screen_glass,
        name="screen",
    )
    display.inertial = Inertial.from_geometry(
        Box((0.110, 0.050, 0.008)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    model.articulation(
        "chassis_to_display_module",
        ArticulationType.FIXED,
        parent=chassis,
        child=display,
        origin=Origin(xyz=(0.0, 0.148, MIXER_TOP_Z)),
    )

    knob_positions = [
        (-0.055, 0.064),
        (-0.018, 0.064),
        (0.018, 0.064),
        (0.055, 0.064),
        (-0.055, 0.104),
        (-0.018, 0.104),
        (0.018, 0.104),
        (0.055, 0.104),
    ]
    for knob_index, (knob_x, knob_y) in enumerate(knob_positions):
        knob = model.part(f"knob_{knob_index}")
        knob.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=knob_rubber,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.0095, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0135)),
            material=accent_silver,
            name="knob_cap",
        )
        knob.visual(
            Box((0.002, 0.010, 0.001)),
            origin=Origin(xyz=(0.0, 0.0085, 0.0155)),
            material=marker_red,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.012, length=0.012),
            mass=0.018,
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
        )
        model.articulation(
            f"chassis_to_knob_{knob_index}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=knob,
            origin=Origin(xyz=(knob_x, knob_y, MIXER_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.12,
                velocity=5.0,
                lower=-2.6,
                upper=2.6,
            ),
        )

    crossfader_track = model.part("crossfader_track")
    crossfader_track.visual(
        Box((0.185, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=deck_black,
        name="track_floor",
    )
    crossfader_track.visual(
        Box((0.185, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, -0.0095, 0.005)),
        material=rail_grey,
        name="left_rail",
    )
    crossfader_track.visual(
        Box((0.185, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, 0.0095, 0.005)),
        material=rail_grey,
        name="right_rail",
    )
    crossfader_track.visual(
        Box((0.004, 0.020, 0.010)),
        origin=Origin(xyz=(-0.0905, 0.0, 0.005)),
        material=rail_grey,
        name="left_stop",
    )
    crossfader_track.visual(
        Box((0.004, 0.020, 0.010)),
        origin=Origin(xyz=(0.0905, 0.0, 0.005)),
        material=rail_grey,
        name="right_stop",
    )
    crossfader_track.inertial = Inertial.from_geometry(
        Box((0.185, 0.020, 0.010)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )
    model.articulation(
        "chassis_to_crossfader_track",
        ArticulationType.FIXED,
        parent=chassis,
        child=crossfader_track,
        origin=Origin(xyz=(0.0, CROSSFADER_Y, TOP_SURFACE_Z - 0.008)),
    )

    crossfader_slider = model.part("crossfader_slider")
    crossfader_slider.visual(
        Box((0.016, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=accent_silver,
        name="carriage",
    )
    crossfader_slider.visual(
        Box((0.004, 0.004, 0.011)),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=accent_silver,
        name="stem",
    )
    crossfader_slider.visual(
        Box((0.012, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=accent_silver,
        name="grip",
    )
    crossfader_slider.inertial = Inertial.from_geometry(
        Box((0.016, 0.020, 0.021)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
    )
    model.articulation(
        "crossfader_track_to_crossfader_slider",
        ArticulationType.PRISMATIC,
        parent=crossfader_track,
        child=crossfader_slider,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=0.40,
            lower=-0.075,
            upper=0.075,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    left_platter = object_model.get_part("left_platter")
    right_platter = object_model.get_part("right_platter")
    display = object_model.get_part("display_module")
    crossfader_track = object_model.get_part("crossfader_track")
    crossfader_slider = object_model.get_part("crossfader_slider")
    knob_parts = [object_model.get_part(f"knob_{index}") for index in range(8)]

    left_platter_joint = object_model.get_articulation("chassis_to_left_platter")
    right_platter_joint = object_model.get_articulation("chassis_to_right_platter")
    crossfader_joint = object_model.get_articulation(
        "crossfader_track_to_crossfader_slider"
    )
    first_knob_joint = object_model.get_articulation("chassis_to_knob_0")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_platter, chassis, name="left platter seated on deck")
    ctx.expect_contact(right_platter, chassis, name="right platter seated on deck")
    ctx.expect_contact(display, chassis, name="display mounted on mixer island")
    ctx.expect_contact(
        crossfader_track,
        chassis,
        name="crossfader rail anchored to chassis",
    )
    ctx.expect_contact(
        crossfader_slider,
        crossfader_track,
        name="crossfader slider riding on rail",
    )
    for knob in knob_parts:
        ctx.expect_contact(knob, chassis, name=f"{knob.name} mounted to mixer")

    ctx.check(
        "left platter joint configuration",
        left_platter_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(left_platter_joint.axis) == (0.0, 0.0, 1.0),
        "Left jog platter should rotate about the vertical axis.",
    )
    ctx.check(
        "right platter joint configuration",
        right_platter_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(right_platter_joint.axis) == (0.0, 0.0, 1.0),
        "Right jog platter should rotate about the vertical axis.",
    )
    ctx.check(
        "crossfader joint configuration",
        crossfader_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(crossfader_joint.axis) == (1.0, 0.0, 0.0),
        "Crossfader should slide laterally across the mixer section.",
    )
    ctx.check(
        "knob joint configuration",
        first_knob_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(first_knob_joint.axis) == (0.0, 0.0, 1.0),
        "Rotary knobs should turn around their local vertical axes.",
    )

    left_marker_rest = ctx.part_element_world_aabb(left_platter, elem="position_marker")
    left_origin_rest = ctx.part_world_position(left_platter)
    assert left_marker_rest is not None
    assert left_origin_rest is not None
    with ctx.pose({left_platter_joint: -(math.pi * 0.5)}):
        left_marker_turned = ctx.part_element_world_aabb(left_platter, elem="position_marker")
        assert left_marker_turned is not None
        left_marker_turned_center = _aabb_center(left_marker_turned)
        ctx.check(
            "left platter visibly rotates",
            left_marker_turned_center[0] > left_origin_rest[0] + 0.045
            and abs(left_marker_turned_center[1] - left_origin_rest[1]) < 0.012,
            "The jog platter marker should swing around the hub when the joint turns.",
        )
        ctx.expect_contact(left_platter, chassis, name="left platter remains seated in pose")

    slider_rest = ctx.part_world_position(crossfader_slider)
    assert slider_rest is not None
    with ctx.pose({crossfader_joint: 0.075}):
        slider_right = ctx.part_world_position(crossfader_slider)
        assert slider_right is not None
        ctx.expect_contact(
            crossfader_slider,
            crossfader_track,
            name="crossfader stays in contact at right stop",
        )
    with ctx.pose({crossfader_joint: -0.075}):
        slider_left = ctx.part_world_position(crossfader_slider)
        assert slider_left is not None
        ctx.expect_contact(
            crossfader_slider,
            crossfader_track,
            name="crossfader stays in contact at left stop",
        )
    ctx.check(
        "crossfader traverses rail",
        slider_right[0] > slider_rest[0] + 0.070
        and slider_left[0] < slider_rest[0] - 0.070
        and abs(slider_right[1] - slider_rest[1]) < 1e-6
        and abs(slider_left[1] - slider_rest[1]) < 1e-6
        and abs(slider_right[2] - slider_rest[2]) < 1e-6
        and abs(slider_left[2] - slider_rest[2]) < 1e-6,
        "Crossfader should move sideways only.",
    )

    knob_origin_rest = ctx.part_world_position(knob_parts[0])
    knob_indicator_rest = ctx.part_element_world_aabb(knob_parts[0], elem="indicator")
    assert knob_origin_rest is not None
    assert knob_indicator_rest is not None
    with ctx.pose({first_knob_joint: -(math.pi * 0.5)}):
        knob_indicator_turned = ctx.part_element_world_aabb(knob_parts[0], elem="indicator")
        assert knob_indicator_turned is not None
        knob_indicator_center = _aabb_center(knob_indicator_turned)
        ctx.check(
            "knob indicator rotates with joint",
            knob_indicator_center[0] > knob_origin_rest[0] + 0.005
            and abs(knob_indicator_center[1] - knob_origin_rest[1]) < 0.006,
            "The knob marker should rotate around the knob stem.",
        )
        ctx.expect_contact(knob_parts[0], chassis, name="knob remains seated while turning")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
