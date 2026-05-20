from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)

BOARD_UNDERSIDE_Z = 0.888
DECK_FRAME_DEPTH = 0.014
DECK_TOP_SHEET_THICK = 0.0012
BOARD_PAD_THICK = 0.0065
BINDING_THICK = 0.0014
HINGE_AXIS_Z = 0.854
LEG_DROP = 0.845
FRONT_PIVOT_X = 0.250
REAR_PIVOT_X = -0.240
FRONT_FOOT_X = -0.520
REAR_FOOT_X = 0.500
FRONT_HALF_WIDTH = 0.192
REAR_HALF_WIDTH = 0.112
TUBE_RADIUS = 0.009
WHEEL_RADIUS = 0.024
WHEEL_WIDTH = 0.016
WHEEL_CENTER_X = 0.482
WHEEL_CENTER_Y = 0.184
WHEEL_BRACKET_CENTER_Y = 0.156
WHEEL_CENTER_Z = -0.813


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _board_outline(scale_x: float = 1.0, scale_y: float = 1.0) -> list[tuple[float, float]]:
    control_points = [
        (0.620, 0.000),
        (0.586, 0.045),
        (0.522, 0.090),
        (0.404, 0.146),
        (0.180, 0.188),
        (-0.125, 0.195),
        (-0.438, 0.186),
        (-0.594, 0.170),
        (-0.620, 0.154),
        (-0.620, -0.154),
        (-0.594, -0.170),
        (-0.438, -0.186),
        (-0.125, -0.195),
        (0.180, -0.188),
        (0.404, -0.146),
        (0.522, -0.090),
        (0.586, -0.045),
    ]
    profile = sample_catmull_rom_spline_2d(control_points, samples_per_segment=10, closed=True)
    if len(profile) > 1:
        first_x, first_y = profile[0]
        last_x, last_y = profile[-1]
        if abs(first_x - last_x) < 1e-9 and abs(first_y - last_y) < 1e-9:
            profile = profile[:-1]
    return [(x * scale_x, y * scale_y) for x, y in profile]


def _leg_side_points(foot_x: float, y_pos: float) -> list[tuple[float, float, float]]:
    if foot_x < 0.0:
        return [
            (0.000, y_pos, -0.004),
            (-0.095, y_pos, -0.145),
            (-0.250, y_pos, -0.470),
            (foot_x, y_pos, -LEG_DROP),
        ]
    return [
        (0.000, y_pos, -0.004),
        (0.090, y_pos, -0.130),
        (0.240, y_pos, -0.470),
        (foot_x, y_pos, -LEG_DROP),
    ]


def _add_transport_wheel(part, *, wheel_metal, hardware_dark, rubber) -> None:
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.015, length=WHEEL_WIDTH),
        origin=spin_origin,
        material=wheel_metal,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="hub_cap",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.18,
        origin=spin_origin,
    )


def _lerp_point(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    t: float,
) -> tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _mesh_bar(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
):
    return wire_from_points(
        [a, b],
        radius=radius,
        radial_segments=16,
        cap_ends=True,
        corner_mode="miter",
    )


def _leg_frame_mesh(
    foot_x: float,
    half_width: float,
    *,
    tie_t: float | None = None,
    tie_radius: float | None = None,
    lock_t: float | None = None,
    lock_radius: float | None = None,
    stiffener_t: float | None = None,
    stiffener_radius: float | None = None,
):
    left = _leg_side_points(foot_x, half_width)
    right = _leg_side_points(foot_x, -half_width)

    frame = wire_from_points(
        left + list(reversed(right)),
        radius=TUBE_RADIUS,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.045,
        corner_segments=10,
    )
    frame.merge(
        _mesh_bar(
            (0.0, half_width - 0.010, 0.0),
            (0.0, half_width + 0.026, 0.0),
            radius=0.011,
        )
    )
    frame.merge(
        _mesh_bar(
            (0.0, -half_width - 0.026, 0.0),
            (0.0, -half_width + 0.010, 0.0),
            radius=0.011,
        )
    )
    frame.merge(_mesh_bar(left[2], right[2], radius=0.0075))

    if tie_t is not None and tie_radius is not None:
        frame.merge(
            _mesh_bar(
                _lerp_point(left[1], left[2], tie_t),
                _lerp_point(right[1], right[2], tie_t),
                radius=tie_radius,
            )
        )

    if lock_t is not None and lock_radius is not None:
        frame.merge(
            _mesh_bar(
                _lerp_point(left[1], left[2], lock_t),
                _lerp_point(right[1], right[2], lock_t),
                radius=lock_radius,
            )
        )

    if stiffener_t is not None and stiffener_radius is not None:
        frame.merge(
            _mesh_bar(
                _lerp_point(left[1], left[2], stiffener_t),
                _lerp_point(right[1], right[2], stiffener_t),
                radius=stiffener_radius,
            )
        )

    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_ironing_board", assets=ASSETS)

    shell_satin = model.material("shell_satin", rgba=(0.78, 0.80, 0.82, 1.0))
    cover_matte = model.material("cover_matte", rgba=(0.23, 0.24, 0.26, 1.0))
    trim_polymer = model.material("trim_polymer", rgba=(0.14, 0.15, 0.17, 1.0))
    frame_satin = model.material("frame_satin", rgba=(0.62, 0.64, 0.67, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.32, 0.34, 0.37, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.70, 0.71, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    board_profile = _board_outline()
    inner_frame_profile = _board_outline(scale_x=0.948, scale_y=0.980)
    cover_profile = _board_outline(scale_x=0.974, scale_y=0.930)
    binding_outer_profile = _board_outline(scale_x=0.986, scale_y=0.950)
    binding_inner_profile = _board_outline(scale_x=0.966, scale_y=0.916)

    board_deck = model.part("board_deck")
    board_deck.visual(
        _save_mesh(
            "ironing_board_shell.obj",
            ExtrudeWithHolesGeometry(
                board_profile,
                [inner_frame_profile],
                DECK_FRAME_DEPTH,
                center=False,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, BOARD_UNDERSIDE_Z)),
        material=shell_satin,
        name="deck_shell",
    )
    board_deck.visual(
        _save_mesh(
            "ironing_board_skin.obj",
            ExtrudeGeometry(board_profile, DECK_TOP_SHEET_THICK, center=False),
        ),
        origin=Origin(xyz=(0.0, 0.0, BOARD_UNDERSIDE_Z + DECK_FRAME_DEPTH)),
        material=shell_satin,
        name="deck_skin",
    )
    board_deck.visual(
        _save_mesh(
            "ironing_board_cover.obj",
            ExtrudeGeometry(cover_profile, BOARD_PAD_THICK, center=False),
        ),
        origin=Origin(xyz=(0.0, 0.0, BOARD_UNDERSIDE_Z + DECK_FRAME_DEPTH + DECK_TOP_SHEET_THICK)),
        material=cover_matte,
        name="cover_pad",
    )
    board_deck.visual(
        _save_mesh(
            "ironing_board_cover_binding.obj",
            ExtrudeWithHolesGeometry(
                binding_outer_profile,
                [binding_inner_profile],
                BINDING_THICK,
                center=False,
            ),
        ),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BOARD_UNDERSIDE_Z + DECK_FRAME_DEPTH + DECK_TOP_SHEET_THICK + BOARD_PAD_THICK - 0.0008,
            )
        ),
        material=trim_polymer,
        name="cover_binding",
    )
    board_deck.visual(
        Box((0.062, 0.080, 0.009)),
        origin=Origin(
            xyz=(
                0.585,
                0.0,
                BOARD_UNDERSIDE_Z + DECK_FRAME_DEPTH + DECK_TOP_SHEET_THICK + BOARD_PAD_THICK - 0.0004,
            )
        ),
        material=trim_polymer,
        name="nose_bumper",
    )
    board_deck.visual(
        Box((0.042, 0.228, 0.014)),
        origin=Origin(xyz=(-0.602, 0.0, 0.895)),
        material=trim_polymer,
        name="tail_cap",
    )
    board_deck.visual(
        Box((0.762, 0.014, 0.016)),
        origin=Origin(xyz=(0.020, 0.094, 0.880)),
        material=hardware_dark,
        name="left_reinforcement_rail",
    )
    board_deck.visual(
        Box((0.762, 0.014, 0.016)),
        origin=Origin(xyz=(0.020, -0.094, 0.880)),
        material=hardware_dark,
        name="right_reinforcement_rail",
    )
    board_deck.visual(
        Box((0.076, 0.388, 0.023)),
        origin=Origin(xyz=(FRONT_PIVOT_X, 0.0, 0.8765)),
        material=hardware_dark,
        name="front_mount_bridge",
    )
    board_deck.visual(
        Box((0.076, 0.246, 0.023)),
        origin=Origin(xyz=(REAR_PIVOT_X, 0.0, 0.8765)),
        material=hardware_dark,
        name="rear_mount_bridge",
    )
    board_deck.visual(
        Box((0.580, 0.012, 0.018)),
        origin=Origin(xyz=(0.005, 0.019, 0.879)),
        material=hardware_dark,
        name="left_center_spine",
    )
    board_deck.visual(
        Box((0.580, 0.012, 0.018)),
        origin=Origin(xyz=(0.005, -0.019, 0.879)),
        material=hardware_dark,
        name="right_center_spine",
    )
    board_deck.visual(
        Box((0.030, 0.016, 0.026)),
        origin=Origin(xyz=(-0.028, 0.020, 0.875)),
        material=hardware_dark,
        name="brace_mount_plate_left",
    )
    board_deck.visual(
        Box((0.030, 0.016, 0.026)),
        origin=Origin(xyz=(-0.028, -0.020, 0.875)),
        material=hardware_dark,
        name="brace_mount_plate_right",
    )
    board_deck.visual(
        Box((0.056, 0.028, 0.015)),
        origin=Origin(xyz=(-0.102, 0.0, 0.8805)),
        material=hardware_dark,
        name="lock_receiver_block",
    )
    board_deck.inertial = Inertial.from_geometry(
        Box((1.240, 0.390, 0.036)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.902)),
    )

    front_left_points = _leg_side_points(FRONT_FOOT_X, FRONT_HALF_WIDTH)
    front_right_points = _leg_side_points(FRONT_FOOT_X, -FRONT_HALF_WIDTH)
    front_tie_point = _lerp_point(front_left_points[1], front_left_points[2], 0.60)
    pivot_interface_y = 0.5 * (FRONT_HALF_WIDTH + REAR_HALF_WIDTH)
    front_pivot_center_y = 0.5 * (FRONT_HALF_WIDTH + pivot_interface_y)
    front_pivot_length = FRONT_HALF_WIDTH - pivot_interface_y

    front_leg = model.part("front_leg_frame")
    front_leg.visual(
        _save_mesh(
            "front_leg_left_tube.obj",
            wire_from_points(
                front_left_points,
                radius=TUBE_RADIUS,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.040,
                corner_segments=10,
            ),
        ),
        material=frame_satin,
        name="left_side_tube",
    )
    front_leg.visual(
        _save_mesh(
            "front_leg_right_tube.obj",
            wire_from_points(
                front_right_points,
                radius=TUBE_RADIUS,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.040,
                corner_segments=10,
            ),
        ),
        material=frame_satin,
        name="right_side_tube",
    )
    front_leg.visual(
        Cylinder(radius=TUBE_RADIUS, length=2.0 * FRONT_HALF_WIDTH),
        origin=Origin(
            xyz=(FRONT_FOOT_X, 0.0, -LEG_DROP),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=frame_satin,
        name="foot_bar",
    )
    front_leg.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(0.0, FRONT_HALF_WIDTH, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="hinge_left",
    )
    front_leg.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(0.0, -FRONT_HALF_WIDTH, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="hinge_right",
    )
    front_leg.visual(
        Cylinder(radius=0.0065, length=2.0 * FRONT_HALF_WIDTH),
        origin=Origin(xyz=(front_tie_point[0], 0.0, front_tie_point[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="tie_bar",
    )
    front_leg.visual(
        Cylinder(radius=0.011, length=front_pivot_length),
        origin=Origin(
            xyz=(front_left_points[2][0], front_pivot_center_y, front_left_points[2][2]),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_dark,
        name="pivot_hub_left",
    )
    front_leg.visual(
        Cylinder(radius=0.011, length=front_pivot_length),
        origin=Origin(
            xyz=(front_right_points[2][0], -front_pivot_center_y, front_right_points[2][2]),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_dark,
        name="pivot_hub_right",
    )
    front_leg.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(
            xyz=(FRONT_FOOT_X, FRONT_HALF_WIDTH, -LEG_DROP),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="left_foot_cap",
    )
    front_leg.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(
            xyz=(FRONT_FOOT_X, -FRONT_HALF_WIDTH, -LEG_DROP),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="right_foot_cap",
    )
    front_leg.inertial = Inertial.from_geometry(
        Box((0.620, 0.380, 0.860)),
        mass=1.8,
        origin=Origin(xyz=(-0.240, 0.0, -0.420)),
    )

    rear_left_points = _leg_side_points(REAR_FOOT_X, REAR_HALF_WIDTH)
    rear_right_points = _leg_side_points(REAR_FOOT_X, -REAR_HALF_WIDTH)
    rear_lock_point = _lerp_point(rear_left_points[1], rear_left_points[2], 0.42)
    rear_stiffener_point = _lerp_point(rear_left_points[1], rear_left_points[2], 0.64)
    rear_pivot_center_y = 0.5 * (REAR_HALF_WIDTH + pivot_interface_y)
    rear_pivot_length = pivot_interface_y - REAR_HALF_WIDTH
    wheel_mount_contact_y = WHEEL_CENTER_Y - 0.5 * WHEEL_WIDTH
    wheel_bracket_center_y = 0.5 * (REAR_HALF_WIDTH + wheel_mount_contact_y)
    wheel_bracket_width_y = wheel_mount_contact_y - REAR_HALF_WIDTH

    rear_leg = model.part("rear_leg_frame")
    rear_leg.visual(
        _save_mesh(
            "rear_leg_left_tube.obj",
            wire_from_points(
                rear_left_points,
                radius=TUBE_RADIUS,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.040,
                corner_segments=10,
            ),
        ),
        material=frame_satin,
        name="left_side_tube",
    )
    rear_leg.visual(
        _save_mesh(
            "rear_leg_right_tube.obj",
            wire_from_points(
                rear_right_points,
                radius=TUBE_RADIUS,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.040,
                corner_segments=10,
            ),
        ),
        material=frame_satin,
        name="right_side_tube",
    )
    rear_leg.visual(
        Cylinder(radius=TUBE_RADIUS, length=2.0 * REAR_HALF_WIDTH),
        origin=Origin(
            xyz=(REAR_FOOT_X, 0.0, -LEG_DROP),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=frame_satin,
        name="foot_bar",
    )
    rear_leg.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(0.0, REAR_HALF_WIDTH, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="hinge_left",
    )
    rear_leg.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(0.0, -REAR_HALF_WIDTH, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="hinge_right",
    )
    rear_leg.visual(
        Cylinder(radius=0.006, length=2.0 * REAR_HALF_WIDTH),
        origin=Origin(xyz=(rear_lock_point[0], 0.0, rear_lock_point[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="lock_catch_bar",
    )
    rear_leg.visual(
        Cylinder(radius=0.0065, length=2.0 * REAR_HALF_WIDTH),
        origin=Origin(xyz=(rear_stiffener_point[0], 0.0, rear_stiffener_point[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="stiffener_bar",
    )
    rear_leg.visual(
        Cylinder(radius=0.011, length=rear_pivot_length),
        origin=Origin(
            xyz=(rear_left_points[2][0], rear_pivot_center_y, rear_left_points[2][2]),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_dark,
        name="pivot_hub_left",
    )
    rear_leg.visual(
        Cylinder(radius=0.011, length=rear_pivot_length),
        origin=Origin(
            xyz=(rear_right_points[2][0], -rear_pivot_center_y, rear_right_points[2][2]),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_dark,
        name="pivot_hub_right",
    )
    rear_leg.visual(
        Box((0.028, wheel_bracket_width_y, 0.060)),
        origin=Origin(xyz=(0.492, wheel_bracket_center_y, -0.820)),
        material=hardware_dark,
        name="left_wheel_bracket",
    )
    rear_leg.visual(
        Box((0.028, wheel_bracket_width_y, 0.060)),
        origin=Origin(xyz=(0.492, -wheel_bracket_center_y, -0.820)),
        material=hardware_dark,
        name="right_wheel_bracket",
    )
    rear_leg.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(
            xyz=(REAR_FOOT_X, REAR_HALF_WIDTH, -LEG_DROP),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="left_foot_cap",
    )
    rear_leg.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(
            xyz=(REAR_FOOT_X, -REAR_HALF_WIDTH, -LEG_DROP),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="right_foot_cap",
    )
    rear_leg.inertial = Inertial.from_geometry(
        Box((0.600, 0.330, 0.860)),
        mass=1.9,
        origin=Origin(xyz=(0.230, 0.0, -0.420)),
    )

    lock_brace = model.part("lock_brace")
    lock_brace.visual(
        _save_mesh(
            "lock_brace_body.obj",
            wire_from_points(
                [
                    (0.000, 0.0, -0.002),
                    (-0.014, 0.0, -0.060),
                    (-0.032, 0.0, -0.130),
                    (-0.052, 0.0, -0.214),
                    (-0.068, 0.0, -0.258),
                ],
                radius=0.0048,
                radial_segments=14,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.012,
                corner_segments=8,
            ),
        ),
        material=frame_satin,
        name="brace_body",
    )
    lock_brace.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="brace_pivot",
    )
    lock_brace.visual(
        Box((0.020, 0.018, 0.024)),
        origin=Origin(xyz=(-0.077, 0.0, -0.254)),
        material=hardware_dark,
        name="brace_tab",
    )
    lock_brace.inertial = Inertial.from_geometry(
        Box((0.120, 0.040, 0.300)),
        mass=0.18,
        origin=Origin(xyz=(-0.040, 0.0, -0.145)),
    )

    model.articulation(
        "board_to_front_leg",
        ArticulationType.REVOLUTE,
        parent=board_deck,
        child=front_leg,
        origin=Origin(xyz=(FRONT_PIVOT_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=-0.05,
            upper=0.65,
        ),
    )
    model.articulation(
        "board_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=board_deck,
        child=rear_leg,
        origin=Origin(xyz=(REAR_PIVOT_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=-0.80,
            upper=0.05,
        ),
    )
    model.articulation(
        "board_to_lock_brace",
        ArticulationType.REVOLUTE,
        parent=board_deck,
        child=lock_brace,
        origin=Origin(xyz=(-0.028, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=0.0,
            upper=1.18,
        ),
    )

    left_transport_wheel = model.part("left_transport_wheel")
    _add_transport_wheel(
        left_transport_wheel,
        wheel_metal=wheel_metal,
        hardware_dark=hardware_dark,
        rubber=rubber,
    )

    right_transport_wheel = model.part("right_transport_wheel")
    _add_transport_wheel(
        right_transport_wheel,
        wheel_metal=wheel_metal,
        hardware_dark=hardware_dark,
        rubber=rubber,
    )

    model.articulation(
        "rear_leg_to_left_transport_wheel",
        ArticulationType.CONTINUOUS,
        parent=rear_leg,
        child=left_transport_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )
    model.articulation(
        "rear_leg_to_right_transport_wheel",
        ArticulationType.CONTINUOUS,
        parent=rear_leg,
        child=right_transport_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, -WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    board_deck = object_model.get_part("board_deck")
    front_leg = object_model.get_part("front_leg_frame")
    rear_leg = object_model.get_part("rear_leg_frame")
    lock_brace = object_model.get_part("lock_brace")
    left_transport_wheel = object_model.get_part("left_transport_wheel")
    right_transport_wheel = object_model.get_part("right_transport_wheel")

    front_joint = object_model.get_articulation("board_to_front_leg")
    rear_joint = object_model.get_articulation("board_to_rear_leg")
    brace_joint = object_model.get_articulation("board_to_lock_brace")
    left_wheel_joint = object_model.get_articulation("rear_leg_to_left_transport_wheel")
    right_wheel_joint = object_model.get_articulation("rear_leg_to_right_transport_wheel")

    deck_shell = board_deck.get_visual("deck_shell")
    deck_skin = board_deck.get_visual("deck_skin")
    cover_pad = board_deck.get_visual("cover_pad")
    front_bridge = board_deck.get_visual("front_mount_bridge")
    rear_bridge = board_deck.get_visual("rear_mount_bridge")
    brace_mount_left = board_deck.get_visual("brace_mount_plate_left")
    front_hinge_left = front_leg.get_visual("hinge_left")
    front_hinge_right = front_leg.get_visual("hinge_right")
    rear_hinge_left = rear_leg.get_visual("hinge_left")
    rear_hinge_right = rear_leg.get_visual("hinge_right")
    front_pivot_hub_left = front_leg.get_visual("pivot_hub_left")
    front_pivot_hub_right = front_leg.get_visual("pivot_hub_right")
    rear_pivot_hub_left = rear_leg.get_visual("pivot_hub_left")
    rear_pivot_hub_right = rear_leg.get_visual("pivot_hub_right")
    brace_pivot = lock_brace.get_visual("brace_pivot")
    brace_tab = lock_brace.get_visual("brace_tab")
    lock_catch_bar = rear_leg.get_visual("lock_catch_bar")
    left_wheel_bracket = rear_leg.get_visual("left_wheel_bracket")
    right_wheel_bracket = rear_leg.get_visual("right_wheel_bracket")
    left_wheel_hub = left_transport_wheel.get_visual("hub_shell")
    right_wheel_hub = right_transport_wheel.get_visual("hub_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        board_deck,
        front_leg,
        elem_a=front_bridge,
        elem_b=front_hinge_left,
        name="front_leg_hinge_is_seated",
    )
    ctx.expect_contact(
        board_deck,
        front_leg,
        elem_a=front_bridge,
        elem_b=front_hinge_right,
        name="front_leg_right_hinge_is_seated",
    )
    ctx.expect_contact(
        board_deck,
        rear_leg,
        elem_a=rear_bridge,
        elem_b=rear_hinge_left,
        name="rear_leg_hinge_is_seated",
    )
    ctx.expect_contact(
        board_deck,
        rear_leg,
        elem_a=rear_bridge,
        elem_b=rear_hinge_right,
        name="rear_leg_right_hinge_is_seated",
    )
    ctx.expect_contact(
        front_leg,
        rear_leg,
        elem_a=front_pivot_hub_left,
        elem_b=rear_pivot_hub_left,
        name="left_scissor_pivot_hub_is_seated",
    )
    ctx.expect_contact(
        front_leg,
        rear_leg,
        elem_a=front_pivot_hub_right,
        elem_b=rear_pivot_hub_right,
        name="right_scissor_pivot_hub_is_seated",
    )
    ctx.expect_contact(
        board_deck,
        lock_brace,
        elem_a=brace_mount_left,
        elem_b=brace_pivot,
        name="brace_pivot_is_seated",
    )
    ctx.expect_contact(
        lock_brace,
        rear_leg,
        elem_a=brace_tab,
        elem_b=lock_catch_bar,
        contact_tol=0.005,
        name="open_lock_point_is_engaged",
    )
    ctx.expect_contact(
        rear_leg,
        left_transport_wheel,
        elem_a=left_wheel_bracket,
        elem_b=left_wheel_hub,
        name="left_transport_wheel_mount_is_seated",
    )
    ctx.expect_contact(
        rear_leg,
        right_transport_wheel,
        elem_a=right_wheel_bracket,
        elem_b=right_wheel_hub,
        name="right_transport_wheel_mount_is_seated",
    )
    ctx.expect_overlap(
        front_leg,
        rear_leg,
        axes="xz",
        min_overlap=0.25,
        name="scissor_frames_cross_in_side_view",
    )

    deck_aabb = ctx.part_element_world_aabb(board_deck, elem="deck_skin")
    cover_aabb = ctx.part_element_world_aabb(board_deck, elem="cover_pad")
    front_open_aabb = ctx.part_world_aabb(front_leg)
    rear_open_aabb = ctx.part_world_aabb(rear_leg)
    left_wheel_open_aabb = ctx.part_world_aabb(left_transport_wheel)
    right_wheel_open_aabb = ctx.part_world_aabb(right_transport_wheel)

    if (
        deck_aabb is None
        or cover_aabb is None
        or front_open_aabb is None
        or rear_open_aabb is None
        or left_wheel_open_aabb is None
        or right_wheel_open_aabb is None
    ):
        ctx.fail(
            "measurement_aabbs_available",
            "Expected world-space AABBs for deck, cover, leg frames, and transport wheels.",
        )
        return ctx.report()

    deck_length = deck_aabb[1][0] - deck_aabb[0][0]
    deck_width = deck_aabb[1][1] - deck_aabb[0][1]
    deck_thickness = deck_aabb[1][2] - deck_aabb[0][2]
    cover_length = cover_aabb[1][0] - cover_aabb[0][0]
    cover_width = cover_aabb[1][1] - cover_aabb[0][1]
    cover_gap_z = cover_aabb[0][2] - deck_aabb[1][2]
    cover_reveal_x = 0.5 * (deck_length - cover_length)
    cover_reveal_y = 0.5 * (deck_width - cover_width)

    ctx.check(
        "deck_shell_has_realistic_proportions",
        1.20 <= deck_length <= 1.27 and 0.37 <= deck_width <= 0.40 and 0.001 <= deck_thickness <= 0.0025,
        details=(
            f"deck length={deck_length:.3f}, width={deck_width:.3f}, "
            f"thickness={deck_thickness:.4f}"
        ),
    )
    ctx.check(
        "cover_pad_stays_inside_shell_reveal",
        cover_aabb[0][0] >= deck_aabb[0][0]
        and cover_aabb[1][0] <= deck_aabb[1][0]
        and cover_aabb[0][1] >= deck_aabb[0][1]
        and cover_aabb[1][1] <= deck_aabb[1][1]
        and 0.010 <= cover_reveal_x <= 0.030
        and 0.010 <= cover_reveal_y <= 0.020,
        details=(
            f"cover reveal x={cover_reveal_x:.3f}, y={cover_reveal_y:.3f}, "
            f"shell={deck_aabb}, cover={cover_aabb}"
        ),
    )
    ctx.check(
        "cover_pad_sits_cleanly_on_shell",
        0.0 <= cover_gap_z <= 0.001,
        details=f"cover-to-shell z gap={cover_gap_z:.4f}",
    )

    open_footprint = rear_open_aabb[1][0] - front_open_aabb[0][0]
    ctx.check(
        "open_stance_reaches_floor",
        abs(front_open_aabb[0][2]) <= 0.004 and abs(rear_open_aabb[0][2]) <= 0.004,
        details=(
            f"front floor z={front_open_aabb[0][2]:.4f}, "
            f"rear floor z={rear_open_aabb[0][2]:.4f}"
        ),
    )
    ctx.check(
        "open_stance_has_practical_footprint",
        open_footprint >= 0.50,
        details=f"stance footprint length={open_footprint:.3f}",
    )
    ctx.check(
        "transport_wheels_stay_clear_of_floor_in_open_pose",
        left_wheel_open_aabb[0][2] >= 0.010 and right_wheel_open_aabb[0][2] >= 0.010,
        details=(
            f"left wheel min z={left_wheel_open_aabb[0][2]:.4f}, "
            f"right wheel min z={right_wheel_open_aabb[0][2]:.4f}"
        ),
    )

    with ctx.pose({left_wheel_joint: 1.4, right_wheel_joint: -2.0}):
        ctx.expect_contact(
            rear_leg,
            left_transport_wheel,
            elem_a=left_wheel_bracket,
            elem_b=left_wheel_hub,
            name="left_transport_wheel_spins_on_mount",
        )
        ctx.expect_contact(
            rear_leg,
            right_transport_wheel,
            elem_a=right_wheel_bracket,
            elem_b=right_wheel_hub,
            name="right_transport_wheel_spins_on_mount",
        )

    with ctx.pose({front_joint: 0.30, rear_joint: -0.50, brace_joint: 1.18}):
        ctx.fail_if_isolated_parts(name="folded_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_overlap")
        ctx.expect_gap(
            lock_brace,
            rear_leg,
            axis="z",
            positive_elem=brace_tab,
            negative_elem=lock_catch_bar,
            min_gap=0.10,
            name="lock_disengages_when_folding",
        )

        front_folded_aabb = ctx.part_world_aabb(front_leg)
        rear_folded_aabb = ctx.part_world_aabb(rear_leg)
        brace_folded_aabb = ctx.part_world_aabb(lock_brace)
        left_wheel_folded_aabb = ctx.part_world_aabb(left_transport_wheel)
        right_wheel_folded_aabb = ctx.part_world_aabb(right_transport_wheel)
        if (
            front_folded_aabb is None
            or rear_folded_aabb is None
            or brace_folded_aabb is None
            or left_wheel_folded_aabb is None
            or right_wheel_folded_aabb is None
        ):
            ctx.fail(
                "folded_pose_aabbs_available",
                "Expected folded-pose AABBs for both leg frames, brace, and transport wheels.",
            )
            return ctx.report()

        ctx.check(
            "front_leg_stows_under_board",
            front_folded_aabb[0][2] >= 0.03 and front_folded_aabb[1][2] <= 1.02,
            details=(
                f"front folded z span=({front_folded_aabb[0][2]:.3f}, "
                f"{front_folded_aabb[1][2]:.3f})"
            ),
        )
        ctx.check(
            "rear_leg_stows_under_board",
            rear_folded_aabb[0][2] >= 0.10 and rear_folded_aabb[1][2] <= 1.11,
            details=(
                f"rear folded z span=({rear_folded_aabb[0][2]:.3f}, "
                f"{rear_folded_aabb[1][2]:.3f})"
            ),
        )
        ctx.check(
            "brace_stows_under_board",
            brace_folded_aabb[0][2] >= 0.73 and brace_folded_aabb[1][2] <= 0.93,
            details=(
                f"brace folded z span=({brace_folded_aabb[0][2]:.3f}, "
                f"{brace_folded_aabb[1][2]:.3f})"
            ),
        )
        ctx.check(
            "folded_pose_lifts_feet_clear",
            front_folded_aabb[0][2] >= 0.03
            and rear_folded_aabb[0][2] >= 0.10
            and brace_folded_aabb[0][2] >= 0.73
            and left_wheel_folded_aabb[0][2] >= 0.30
            and right_wheel_folded_aabb[0][2] >= 0.30,
            details=(
                f"folded mins front={front_folded_aabb[0][2]:.3f}, "
                f"rear={rear_folded_aabb[0][2]:.3f}, brace={brace_folded_aabb[0][2]:.3f}, "
                f"left_wheel={left_wheel_folded_aabb[0][2]:.3f}, "
                f"right_wheel={right_wheel_folded_aabb[0][2]:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
