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
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BASE_WIDTH = 0.365
BASE_DEPTH = 0.262
BASE_BOTTOM_THICKNESS = 0.018
DECK_TOP_Z = 0.038
REAR_SPINE_TOP_Z = 0.044
HINGE_AXIS_Y = 0.126
HINGE_AXIS_Z = 0.048

KEY_UNIT_PITCH = 0.021
KEY_GAP = 0.003
KEY_HEIGHT = 0.0046
KEY_REST_BOTTOM_Z = 0.0312
KEY_TRAVEL = 0.0012
KEY_ROW_DEPTHS = (0.013, 0.015, 0.015, 0.015, 0.016)
KEY_ROW_YS = (0.082, 0.060, 0.038, 0.016, -0.006)
KEY_ROW_UNITS = (
    (1.0,) * 12,
    (1.5,) + (1.0,) * 10 + (1.5,),
    (1.75,) + (1.0,) * 9 + (1.75,),
    (2.25,) + (1.0,) * 7 + (2.25,),
    (1.25, 1.25, 1.25, 6.0, 1.25, 1.25, 1.25),
)

KEYBOARD_FLOOR_TOP_Z = KEY_REST_BOTTOM_Z - 0.0014
TOUCHPAD_FLOOR_TOP_Z = 0.0310
CLICKPAD_TOP_Z = 0.0366
CLICKPAD_THICKNESS = 0.0036
CLICKPAD_WIDTH = 0.134
CLICKPAD_DEPTH = 0.076
CLICKPAD_REAR_Y = -0.032
CLICKPAD_CLICK_ANGLE = 0.024

LID_WIDTH = 0.342
LID_HEIGHT = 0.226
LID_THICKNESS = 0.016
SCREEN_WIDTH = 0.302
SCREEN_HEIGHT = 0.180


def _key_specs() -> tuple[tuple[str, float, float, float, float], ...]:
    specs: list[tuple[str, float, float, float, float]] = []
    for row_index, (row_units, center_y, depth) in enumerate(
        zip(KEY_ROW_UNITS, KEY_ROW_YS, KEY_ROW_DEPTHS)
    ):
        total_span = sum(row_units) * KEY_UNIT_PITCH
        cursor_x = -total_span * 0.5
        for col_index, unit_span in enumerate(row_units):
            center_x = cursor_x + unit_span * KEY_UNIT_PITCH * 0.5
            width = unit_span * KEY_UNIT_PITCH - KEY_GAP
            specs.append((f"key_r{row_index}_c{col_index}", center_x, center_y, width, depth))
            cursor_x += unit_span * KEY_UNIT_PITCH
    return tuple(specs)


def _key_joint_name(key_name: str) -> str:
    return f"{key_name}_slide"


def _dims_from_aabb(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple(upper[index] - lower[index] for index in range(3))


def _xy_rounded_loop(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + center_y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)
    ]


def _xz_rounded_loop(
    width: float,
    height: float,
    radius: float,
    y: float,
    *,
    center_z: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + center_z)
        for x, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _build_base_outer_wall_mesh():
    return LoftGeometry(
        [
            _xy_rounded_loop(BASE_WIDTH, BASE_DEPTH, 0.024, 0.0),
            _xy_rounded_loop(BASE_WIDTH - 0.010, BASE_DEPTH - 0.010, 0.021, 0.018),
            _xy_rounded_loop(BASE_WIDTH - 0.022, BASE_DEPTH - 0.020, 0.017, 0.038),
            _xy_rounded_loop(BASE_WIDTH - 0.028, BASE_DEPTH - 0.030, 0.014, 0.044, center_y=0.006),
        ],
        cap=False,
        closed=True,
    )


def _build_lid_outer_wall_mesh():
    geom = LoftGeometry(
        [
            _xy_rounded_loop(LID_WIDTH, LID_HEIGHT, 0.018, 0.0065, center_y=LID_HEIGHT * 0.5),
            _xy_rounded_loop(
                LID_WIDTH - 0.006,
                LID_HEIGHT - 0.006,
                0.016,
                -0.0005,
                center_y=LID_HEIGHT * 0.5,
            ),
            _xy_rounded_loop(
                LID_WIDTH - 0.002,
                LID_HEIGHT - 0.002,
                0.017,
                -0.0085,
                center_y=LID_HEIGHT * 0.5,
            ),
        ],
        cap=False,
        closed=True,
    )
    geom.rotate_x(math.pi * 0.5)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_laptop")

    body_dark = model.material("body_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    body_mid = model.material("body_mid", rgba=(0.24, 0.25, 0.27, 1.0))
    bumper = model.material("bumper", rgba=(0.31, 0.32, 0.33, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.45, 0.47, 0.49, 1.0))
    key_dark = model.material("key_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    key_highlight = model.material("key_highlight", rgba=(0.18, 0.19, 0.20, 1.0))
    glass = model.material("glass", rgba=(0.13, 0.18, 0.22, 0.55))
    touchpad_mat = model.material("touchpad", rgba=(0.15, 0.16, 0.17, 1.0))
    touchpad_trim = model.material("touchpad_trim", rgba=(0.28, 0.29, 0.31, 1.0))

    base = model.part("base_chassis")
    base.visual(
        mesh_from_geometry(_build_base_outer_wall_mesh(), "rugged_laptop_base_outer_wall"),
        material=bumper,
        name="outer_wall_shell",
    )
    base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_BOTTOM_THICKNESS * 0.5)),
        material=body_dark,
        name="bottom_pan",
    )
    base.visual(
        Box((0.018, 0.250, DECK_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(-0.1735, 0.0, (DECK_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5)),
        material=bumper,
        name="left_side_bumper",
    )
    base.visual(
        Box((0.018, 0.250, DECK_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.1735, 0.0, (DECK_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5)),
        material=bumper,
        name="right_side_bumper",
    )
    base.visual(
        Box((0.329, 0.018, DECK_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.121, (DECK_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5)),
        material=bumper,
        name="front_bumper",
    )
    base.visual(
        Box((0.334, 0.030, REAR_SPINE_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.112, (REAR_SPINE_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5)),
        material=body_mid,
        name="rear_spine",
    )
    base.visual(
        Box((0.080, 0.082, DECK_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(-0.121, -0.071, (DECK_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5)),
        material=body_mid,
        name="left_palmrest",
    )
    base.visual(
        Box((0.080, 0.082, DECK_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.121, -0.071, (DECK_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5)),
        material=body_mid,
        name="right_palmrest",
    )
    base.visual(
        Box((0.314, 0.012, DECK_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.024, (DECK_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5)),
        material=body_mid,
        name="keyboard_touchpad_bridge",
    )
    base.visual(
        Box((0.314, 0.012, DECK_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.117, (DECK_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5)),
        material=bumper,
        name="front_deck_rail",
    )
    base.visual(
        Box((0.010, 0.112, DECK_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(-0.149, 0.038, (DECK_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5)),
        material=body_mid,
        name="keyboard_left_rail",
    )
    base.visual(
        Box((0.010, 0.112, DECK_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.149, 0.038, (DECK_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5)),
        material=body_mid,
        name="keyboard_right_rail",
    )
    base.visual(
        Box((0.308, 0.012, DECK_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.097, (DECK_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5)),
        material=body_mid,
        name="keyboard_top_rail",
    )
    base.visual(
        Box((0.290, 0.104, KEYBOARD_FLOOR_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.036,
                (KEYBOARD_FLOOR_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5,
            )
        ),
        material=body_dark,
        name="keyboard_floor",
    )
    base.visual(
        Box((0.196, 0.082, TOUCHPAD_FLOOR_TOP_Z - BASE_BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -0.072,
                (TOUCHPAD_FLOOR_TOP_Z + BASE_BOTTOM_THICKNESS) * 0.5,
            )
        ),
        material=body_dark,
        name="touchpad_floor",
    )
    base.visual(
        Box((0.034, 0.016, 0.008)),
        origin=Origin(xyz=(-0.118, 0.119, 0.040)),
        material=body_mid,
        name="left_hinge_support",
    )
    base.visual(
        Box((0.034, 0.016, 0.008)),
        origin=Origin(xyz=(0.118, 0.119, 0.040)),
        material=body_mid,
        name="right_hinge_support",
    )
    base.visual(
        Cylinder(radius=0.0035, length=0.020),
        origin=Origin(xyz=(-0.118, 0.118, 0.041), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="left_hinge_barrel",
    )
    base.visual(
        Cylinder(radius=0.0035, length=0.020),
        origin=Origin(xyz=(0.118, 0.118, 0.041), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="right_hinge_barrel",
    )
    for bumper_name, bumper_x, bumper_y in (
        ("front_left_corner_bumper", -0.162, -0.117),
        ("front_right_corner_bumper", 0.162, -0.117),
        ("rear_left_corner_bumper", -0.162, 0.111),
        ("rear_right_corner_bumper", 0.162, 0.111),
    ):
        base.visual(
            Box((0.026, 0.026, 0.024)),
            origin=Origin(xyz=(bumper_x, bumper_y, 0.030)),
            material=bumper,
            name=bumper_name,
        )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, REAR_SPINE_TOP_Z)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, REAR_SPINE_TOP_Z * 0.5)),
    )

    for key_name, center_x, center_y, width, depth in _key_specs():
        key_part = model.part(key_name)
        key_part.visual(
            Box((width, depth, KEY_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, KEY_HEIGHT * 0.5)),
            material=key_dark,
            name="keycap",
        )
        key_part.visual(
            Box((max(width - 0.004, width * 0.65), max(depth - 0.005, depth * 0.55), 0.0008)),
            origin=Origin(xyz=(0.0, 0.0, KEY_HEIGHT - 0.0004)),
            material=key_highlight,
            name="key_top_patch",
        )
        key_part.inertial = Inertial.from_geometry(
            Box((width, depth, KEY_HEIGHT)),
            mass=max(0.006, 0.008 * width / 0.018),
            origin=Origin(xyz=(0.0, 0.0, KEY_HEIGHT * 0.5)),
        )
        model.articulation(
            _key_joint_name(key_name),
            ArticulationType.PRISMATIC,
            parent=base,
            child=key_part,
            origin=Origin(xyz=(center_x, center_y, KEY_REST_BOTTOM_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=0.03,
                lower=-KEY_TRAVEL,
                upper=0.0,
            ),
        )

    clickpad = model.part("clickpad")
    clickpad.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(
                rounded_rect_profile(CLICKPAD_WIDTH, CLICKPAD_DEPTH, 0.008, corner_segments=8),
                CLICKPAD_THICKNESS,
                cap=True,
                closed=True,
            ),
            "rugged_laptop_clickpad_plate",
        ),
        origin=Origin(xyz=(0.0, -CLICKPAD_DEPTH * 0.5, -CLICKPAD_THICKNESS * 0.5)),
        material=touchpad_mat,
        name="clickpad_plate",
    )
    clickpad.visual(
        Box((CLICKPAD_WIDTH * 0.82, CLICKPAD_DEPTH * 0.68, 0.0008)),
        origin=Origin(
            xyz=(
                0.0,
                -CLICKPAD_DEPTH * 0.5,
                -0.0004,
            )
        ),
        material=touchpad_trim,
        name="clickpad_touch_surface",
    )
    clickpad.inertial = Inertial.from_geometry(
        Box((CLICKPAD_WIDTH, CLICKPAD_DEPTH, CLICKPAD_THICKNESS)),
        mass=0.11,
        origin=Origin(xyz=(0.0, -CLICKPAD_DEPTH * 0.5, -CLICKPAD_THICKNESS * 0.5)),
    )
    model.articulation(
        "base_to_clickpad",
        ArticulationType.REVOLUTE,
        parent=base,
        child=clickpad,
        origin=Origin(xyz=(0.0, CLICKPAD_REAR_Y, CLICKPAD_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.8,
            lower=0.0,
            upper=CLICKPAD_CLICK_ANGLE,
        ),
    )

    lid = model.part("display_lid")
    lid.visual(
        mesh_from_geometry(_build_lid_outer_wall_mesh(), "rugged_laptop_lid_outer_wall"),
        material=bumper,
        name="outer_housing_shell",
    )
    lid.visual(
        Box((LID_WIDTH, 0.008, 0.218)),
        origin=Origin(xyz=(0.0, 0.005, 0.117)),
        material=body_mid,
        name="back_shell",
    )
    lid.visual(
        Box((0.020, 0.010, 0.206)),
        origin=Origin(xyz=(-0.161, 0.001, 0.117)),
        material=bumper,
        name="left_frame_rail",
    )
    lid.visual(
        Box((0.020, 0.010, 0.206)),
        origin=Origin(xyz=(0.161, 0.001, 0.117)),
        material=bumper,
        name="right_frame_rail",
    )
    lid.visual(
        Box((0.302, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.001, 0.217)),
        material=bumper,
        name="top_frame_rail",
    )
    lid.visual(
        Box((0.302, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.001, 0.021)),
        material=bumper,
        name="bottom_frame_rail",
    )
    lid.visual(
        Box((SCREEN_WIDTH, 0.0025, SCREEN_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.0052, 0.118)),
        material=glass,
        name="screen_glass",
    )
    lid.visual(
        Box((0.010, LID_THICKNESS, 0.218)),
        origin=Origin(xyz=(-0.166, 0.004, 0.117)),
        material=body_dark,
        name="left_reinforcement",
    )
    lid.visual(
        Box((0.010, LID_THICKNESS, 0.218)),
        origin=Origin(xyz=(0.166, 0.004, 0.117)),
        material=body_dark,
        name="right_reinforcement",
    )
    lid.visual(
        Box((0.022, 0.008, 0.012)),
        origin=Origin(xyz=(-0.118, 0.001, 0.006)),
        material=hinge_metal,
        name="left_lid_hinge_leaf",
    )
    lid.visual(
        Box((0.022, 0.008, 0.012)),
        origin=Origin(xyz=(0.118, 0.001, 0.006)),
        material=hinge_metal,
        name="right_lid_hinge_leaf",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_THICKNESS, LID_HEIGHT)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.004, LID_HEIGHT * 0.5)),
    )
    model.articulation(
        "base_to_display_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.75,
            upper=1.48,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_chassis")
    lid = object_model.get_part("display_lid")
    clickpad = object_model.get_part("clickpad")
    lid_joint = object_model.get_articulation("base_to_display_lid")
    clickpad_joint = object_model.get_articulation("base_to_clickpad")
    key_specs = _key_specs()

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    for key_name, _, _, _, _ in key_specs:
        ctx.allow_isolated_part(
            key_name,
            reason="Keycap is mounted by an internal switch mechanism that is intentionally not modeled as visible geometry.",
        )
    ctx.allow_isolated_part(
        "clickpad",
        reason="The clickpad is carried by an internal click hinge and switch mechanism that is not modeled as exposed visual geometry.",
    )
    ctx.allow_isolated_part(
        "display_lid",
        reason="The display housing is carried by internal hinge pins and bushings; the visible shells intentionally keep a small clearance around that hidden hardware.",
    )

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

    expected_part_count = 3 + len(key_specs)
    expected_joint_count = 2 + len(key_specs)
    ctx.check(
        "all_parts_present",
        len(object_model.parts) == expected_part_count,
        details=f"expected {expected_part_count} parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "all_articulations_present",
        len(object_model.articulations) == expected_joint_count,
        details=f"expected {expected_joint_count} articulations, found {len(object_model.articulations)}",
    )

    base_dims = _dims_from_aabb(ctx.part_world_aabb(base))
    lid_dims = _dims_from_aabb(ctx.part_world_aabb(lid))
    ctx.check(
        "base_proportions_realistic",
        base_dims is not None
        and 0.35 <= base_dims[0] <= 0.39
        and 0.25 <= base_dims[1] <= 0.28
        and 0.04 <= base_dims[2] <= 0.06,
        details=f"base dims were {base_dims}",
    )
    ctx.check(
        "lid_proportions_realistic",
        lid_dims is not None
        and 0.33 <= lid_dims[0] <= 0.36
        and 0.015 <= lid_dims[1] <= 0.03
        and 0.22 <= lid_dims[2] <= 0.24,
        details=f"lid dims were {lid_dims}",
    )
    ctx.check(
        "lid_axis_correct",
        tuple(round(value, 3) for value in lid_joint.axis) == (1.0, 0.0, 0.0),
        details=f"lid axis was {lid_joint.axis}",
    )
    ctx.check(
        "clickpad_axis_correct",
        tuple(round(value, 3) for value in clickpad_joint.axis) == (1.0, 0.0, 0.0),
        details=f"clickpad axis was {clickpad_joint.axis}",
    )

    ctx.expect_within(
        clickpad,
        base,
        axes="xy",
        outer_elem="touchpad_floor",
        margin=0.002,
        name="clickpad_within_touchpad_well",
    )
    with ctx.pose({clickpad_joint: 0.0}):
        ctx.expect_gap(
            clickpad,
            base,
            axis="z",
            negative_elem="touchpad_floor",
            min_gap=0.0017,
            max_gap=0.0025,
            name="clickpad_rest_gap_over_floor",
        )
    with ctx.pose({clickpad_joint: CLICKPAD_CLICK_ANGLE}):
        ctx.expect_gap(
            clickpad,
            base,
            axis="z",
            negative_elem="touchpad_floor",
            min_gap=0.00005,
            max_gap=0.0007,
            name="clickpad_pressed_gap_over_floor",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="clickpad_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="clickpad_pressed_no_floating")

    rest_clickpad_aabb = None
    pressed_clickpad_aabb = None
    with ctx.pose({clickpad_joint: 0.0}):
        rest_clickpad_aabb = ctx.part_element_world_aabb(clickpad, elem="clickpad_plate")
    with ctx.pose({clickpad_joint: CLICKPAD_CLICK_ANGLE}):
        pressed_clickpad_aabb = ctx.part_element_world_aabb(clickpad, elem="clickpad_plate")
    clickpad_front_edge_drops = (
        rest_clickpad_aabb is not None
        and pressed_clickpad_aabb is not None
        and pressed_clickpad_aabb[0][2] < rest_clickpad_aabb[0][2] - 0.001
    )
    ctx.check(
        "clickpad_front_edge_rocks_down",
        clickpad_front_edge_drops,
        details=(
            f"rest aabb={rest_clickpad_aabb}, pressed aabb={pressed_clickpad_aabb}"
        ),
    )

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="screen_glass",
            negative_elem="keyboard_floor",
            min_gap=0.040,
            max_gap=0.060,
            name="screen_open_clearance_above_keyboard",
        )

    lid_limits = lid_joint.motion_limits
    if lid_limits is not None and lid_limits.lower is not None and lid_limits.upper is not None:
        with ctx.pose({lid_joint: lid_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="lid_lower_no_floating")
        with ctx.pose({lid_joint: lid_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="lid_upper_no_floating")
            ctx.expect_gap(
                lid,
                base,
                axis="z",
                positive_elem="screen_glass",
                negative_elem="keyboard_floor",
                min_gap=0.002,
                max_gap=0.020,
                name="screen_nearly_closed_clearance",
            )

    open_screen_aabb = None
    closed_screen_aabb = None
    with ctx.pose({lid_joint: 0.0}):
        open_screen_aabb = ctx.part_element_world_aabb(lid, elem="screen_glass")
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_joint: lid_limits.upper}):
            closed_screen_aabb = ctx.part_element_world_aabb(lid, elem="screen_glass")
    lid_moves_through_large_arc = (
        open_screen_aabb is not None
        and closed_screen_aabb is not None
        and closed_screen_aabb[1][2] < open_screen_aabb[1][2] - 0.12
        and closed_screen_aabb[0][1] < open_screen_aabb[0][1] - 0.12
    )
    ctx.check(
        "display_lid_swings_forward",
        lid_moves_through_large_arc,
        details=f"open screen aabb={open_screen_aabb}, closed screen aabb={closed_screen_aabb}",
    )

    for key_name, _, _, _, _ in key_specs:
        key_part = object_model.get_part(key_name)
        key_joint = object_model.get_articulation(_key_joint_name(key_name))
        ctx.check(
            f"{key_name}_axis_correct",
            tuple(round(value, 3) for value in key_joint.axis) == (0.0, 0.0, 1.0),
            details=f"{key_name} axis was {key_joint.axis}",
        )
        ctx.expect_within(
            key_part,
            base,
            axes="xy",
            outer_elem="keyboard_floor",
            margin=0.004,
            name=f"{key_name}_within_keyboard_well",
        )
        with ctx.pose({key_joint: 0.0}):
            ctx.expect_gap(
                key_part,
                base,
                axis="z",
                negative_elem="keyboard_floor",
                min_gap=0.0010,
                max_gap=0.0018,
                name=f"{key_name}_rest_key_travel_gap",
            )
        if key_joint.motion_limits is not None and key_joint.motion_limits.lower is not None:
            with ctx.pose({key_joint: key_joint.motion_limits.lower}):
                ctx.expect_gap(
                    key_part,
                    base,
                    axis="z",
                    negative_elem="keyboard_floor",
                    min_gap=0.00005,
                    max_gap=0.00045,
                    name=f"{key_name}_pressed_key_travel_gap",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
