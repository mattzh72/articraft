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
    rounded_rect_profile,
)


BASE_WIDTH = 0.315
BASE_DEPTH = 0.225
BASE_HEIGHT = 0.013
BASE_WALL = 0.004
BOTTOM_THICKNESS = 0.0018
DECK_THICKNESS = 0.0014
HINGE_Y = 0.121
HINGE_Z = BASE_HEIGHT - 0.0011
HINGE_BARREL_RADIUS = 0.0045
HINGE_KNUCKLE_LENGTH = 0.008
HINGE_SPAN_X = 0.098

LID_WIDTH = 0.306
LID_HEIGHT = 0.205
LID_BACK_THICKNESS = 0.0042
LID_BEZEL_THICKNESS = 0.0022
SCREEN_WIDTH = 0.284
SCREEN_HEIGHT = 0.180
OPEN_ANGLE = 1.92

KEY_DEPTH = 0.015
KEY_GAP = 0.0035
KEY_SMALL = 0.0165
KEY_MEDIUM = 0.0225
KEY_LARGE = 0.030
KEY_SPACE = 0.093
KEY_CAP_THICKNESS = 0.0010
KEY_TRAVEL = 0.0008
KEY_STEM_LENGTH = 0.0032

TRACK_BAR_WIDTH = 0.042
TRACK_BAR_HEIGHT = 0.0085
TRACK_BAR_THICKNESS = 0.0016
TRACK_SLOT_WIDTH = 0.022
TRACK_SLOT_HEIGHT = 0.0035
SHUTTER_WIDTH = 0.012
SHUTTER_TRAVEL = 0.005


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _translate_profile(profile: list[tuple[float, float]], dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _rot_x(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    return (x, (y * cos(angle)) - (z * sin(angle)), (y * sin(angle)) + (z * cos(angle)))


def _lid_origin(local_xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=_rot_x(local_xyz, -OPEN_ANGLE), rpy=(-OPEN_ANGLE, 0.0, 0.0))


def _key_rows() -> list[tuple[float, list[tuple[str, float]]]]:
    return [
        (
            0.041,
            [
                ("grave_key", KEY_SMALL),
                ("digit_1_key", KEY_SMALL),
                ("digit_2_key", KEY_SMALL),
                ("digit_3_key", KEY_SMALL),
                ("digit_4_key", KEY_SMALL),
                ("digit_5_key", KEY_SMALL),
                ("digit_6_key", KEY_SMALL),
                ("digit_7_key", KEY_SMALL),
                ("digit_8_key", KEY_SMALL),
                ("digit_9_key", KEY_SMALL),
                ("digit_0_key", KEY_SMALL),
                ("backspace_key", KEY_LARGE),
            ],
        ),
        (
            0.022,
            [
                ("tab_key", KEY_MEDIUM),
                ("q_key", KEY_SMALL),
                ("w_key", KEY_SMALL),
                ("e_key", KEY_SMALL),
                ("r_key", KEY_SMALL),
                ("t_key", KEY_SMALL),
                ("y_key", KEY_SMALL),
                ("u_key", KEY_SMALL),
                ("i_key", KEY_SMALL),
                ("o_key", KEY_SMALL),
                ("p_key", KEY_SMALL),
                ("pipe_key", KEY_MEDIUM),
            ],
        ),
        (
            0.003,
            [
                ("caps_key", KEY_LARGE),
                ("a_key", KEY_SMALL),
                ("s_key", KEY_SMALL),
                ("d_key", KEY_SMALL),
                ("f_key", KEY_SMALL),
                ("g_key", KEY_SMALL),
                ("h_key", KEY_SMALL),
                ("j_key", KEY_SMALL),
                ("k_key", KEY_SMALL),
                ("l_key", KEY_SMALL),
                ("enter_key", KEY_LARGE),
            ],
        ),
        (
            -0.016,
            [
                ("left_shift_key", KEY_LARGE),
                ("z_key", KEY_SMALL),
                ("x_key", KEY_SMALL),
                ("c_key", KEY_SMALL),
                ("v_key", KEY_SMALL),
                ("b_key", KEY_SMALL),
                ("n_key", KEY_SMALL),
                ("m_key", KEY_SMALL),
                ("comma_key", KEY_SMALL),
                ("period_key", KEY_SMALL),
                ("slash_key", KEY_SMALL),
                ("right_shift_key", KEY_LARGE),
            ],
        ),
        (
            -0.038,
            [
                ("left_ctrl_key", KEY_SMALL),
                ("left_fn_key", KEY_SMALL),
                ("left_alt_key", KEY_SMALL),
                ("space_bar", KEY_SPACE),
                ("right_alt_key", KEY_SMALL),
                ("arrow_left_key", KEY_SMALL),
                ("arrow_down_key", KEY_SMALL),
                ("arrow_right_key", KEY_SMALL),
            ],
        ),
    ]


def _stem_offsets(cap_width: float) -> list[float]:
    if cap_width >= 0.070:
        return [-0.028, 0.0, 0.028]
    if cap_width >= 0.026:
        return [-0.0085, 0.0085]
    return [0.0]


def _key_specs() -> list[dict[str, object]]:
    specs: list[dict[str, object]] = []
    for y_pos, row in _key_rows():
        row_width = sum(width for _, width in row) + (KEY_GAP * (len(row) - 1))
        cursor = -0.5 * row_width
        for name, width in row:
            center_x = cursor + (0.5 * width)
            stem_w = min(width - 0.0045, 0.0135)
            stem_h = KEY_DEPTH - 0.0040
            specs.append(
                {
                    "name": name,
                    "joint_name": f"{name}_travel",
                    "center": (center_x, y_pos),
                    "cap_size": (width, KEY_DEPTH),
                    "stem_size": (stem_w, stem_h),
                    "stem_offsets": _stem_offsets(width),
                }
            )
            cursor += width + KEY_GAP
    return specs


KEY_SPECS = _key_specs()
KEY_PART_NAMES = [spec["name"] for spec in KEY_SPECS]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="business_ultrabook")

    chassis_gray = model.material("chassis_gray", rgba=(0.16, 0.17, 0.19, 1.0))
    chassis_dark = model.material("chassis_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    keyboard_black = model.material("keyboard_black", rgba=(0.08, 0.08, 0.09, 1.0))
    touchpad_gray = model.material("touchpad_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    screen_black = model.material("screen_black", rgba=(0.03, 0.04, 0.05, 1.0))
    shutter_red = model.material("shutter_red", rgba=(0.73, 0.17, 0.15, 1.0))
    shutter_dark = model.material("shutter_dark", rgba=(0.15, 0.15, 0.16, 1.0))

    lower_chassis = model.part("lower_chassis")
    lower_chassis.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
    )

    outer_profile = rounded_rect_profile(BASE_WIDTH, BASE_DEPTH, 0.013, corner_segments=8)
    inner_profile = rounded_rect_profile(
        BASE_WIDTH - (2.0 * BASE_WALL),
        BASE_DEPTH - (2.0 * BASE_WALL),
        0.009,
        corner_segments=8,
    )

    bottom_panel = _mesh(
        "ultrabook_bottom_panel",
        ExtrudeGeometry(outer_profile, BOTTOM_THICKNESS, center=True),
    )
    lower_chassis.visual(
        bottom_panel,
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS * 0.5)),
        material=chassis_dark,
        name="bottom_panel",
    )

    side_wall_ring = _mesh(
        "ultrabook_side_wall_ring",
        ExtrudeWithHolesGeometry(
            outer_profile,
            [inner_profile],
            BASE_HEIGHT - BOTTOM_THICKNESS,
            center=True,
        ),
    )
    lower_chassis.visual(
        side_wall_ring,
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS + ((BASE_HEIGHT - BOTTOM_THICKNESS) * 0.5))),
        material=chassis_gray,
        name="side_wall_ring",
    )

    keyboard_holes: list[list[tuple[float, float]]] = []
    for spec in KEY_SPECS:
        center_x, center_y = spec["center"]  # type: ignore[misc]
        stem_w, stem_h = spec["stem_size"]  # type: ignore[misc]
        for offset_x in spec["stem_offsets"]:  # type: ignore[misc]
            keyboard_holes.append(
                _translate_profile(
                    rounded_rect_profile(stem_w, stem_h, 0.0012, corner_segments=5),
                    dx=center_x + offset_x,
                    dy=center_y,
                )
            )

    keyboard_plate = _mesh(
        "ultrabook_keyboard_plate",
        ExtrudeWithHolesGeometry(
            inner_profile,
            keyboard_holes,
            DECK_THICKNESS,
            center=True,
        ),
    )
    lower_chassis.visual(
        keyboard_plate,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT - (DECK_THICKNESS * 0.5))),
        material=chassis_gray,
        name="keyboard_plate",
    )

    lower_chassis.visual(
        Box((BASE_WIDTH - 0.040, 0.036, 0.004)),
        origin=Origin(xyz=(0.0, HINGE_Y - 0.018, BASE_HEIGHT - 0.0020)),
        material=chassis_dark,
        name="rear_hinge_bridge",
    )
    lower_chassis.visual(
        Box((0.110, 0.068, 0.0008)),
        origin=Origin(xyz=(0.0, -0.074, BASE_HEIGHT + 0.0004)),
        material=touchpad_gray,
        name="touchpad",
    )
    hinge_rot = (0.0, pi / 2.0, 0.0)
    for side_name, side_x in (("left", HINGE_SPAN_X), ("right", -HINGE_SPAN_X)):
        lower_chassis.visual(
            Box((0.032, 0.014, 0.006)),
            origin=Origin(xyz=(side_x, HINGE_Y - 0.008, HINGE_Z - 0.0006)),
            material=chassis_dark,
            name=f"{side_name}_hinge_mount",
        )
        for knuckle_name, knuckle_x in (
            ("inner", side_x - 0.008),
            ("outer", side_x + 0.008),
        ):
            lower_chassis.visual(
                Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_KNUCKLE_LENGTH),
                origin=Origin(xyz=(knuckle_x, HINGE_Y, HINGE_Z), rpy=hinge_rot),
                material=chassis_dark,
                name=f"{side_name}_base_knuckle_{knuckle_name}",
            )

    display_lid = model.part("display_lid")
    display_lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_HEIGHT, LID_BACK_THICKNESS + LID_BEZEL_THICKNESS)),
        mass=0.7,
        origin=Origin(xyz=(0.0, LID_HEIGHT * 0.5, 0.0)),
    )

    lid_back = _mesh(
        "ultrabook_lid_back",
        ExtrudeGeometry(
            rounded_rect_profile(LID_WIDTH, LID_HEIGHT, 0.011, corner_segments=8),
            LID_BACK_THICKNESS,
            center=True,
        ),
    )
    display_lid.visual(
        lid_back,
        origin=_lid_origin((0.0, -(LID_HEIGHT * 0.5), 0.0062)),
        material=chassis_gray,
        name="lid_back_cover",
    )

    bezel_frame = _mesh(
        "ultrabook_bezel_frame",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(LID_WIDTH, LID_HEIGHT, 0.011, corner_segments=8),
            [rounded_rect_profile(SCREEN_WIDTH, SCREEN_HEIGHT, 0.003, corner_segments=6)],
            LID_BEZEL_THICKNESS,
            center=True,
        ),
    )
    display_lid.visual(
        bezel_frame,
        origin=_lid_origin((0.0, -(LID_HEIGHT * 0.5), 0.0040)),
        material=chassis_dark,
        name="bezel_frame",
    )

    display_lid.visual(
        Box((SCREEN_WIDTH, SCREEN_HEIGHT, 0.0012)),
        origin=_lid_origin((0.0, -(0.013 + (SCREEN_HEIGHT * 0.5)), 0.0034)),
        material=screen_black,
        name="screen_glass",
    )

    top_track_bar = _mesh(
        "ultrabook_top_track_bar",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(TRACK_BAR_WIDTH, TRACK_BAR_HEIGHT, 0.0015, corner_segments=5),
            [rounded_rect_profile(TRACK_SLOT_WIDTH, TRACK_SLOT_HEIGHT, 0.0007, corner_segments=4)],
            TRACK_BAR_THICKNESS,
            center=True,
        ),
    )
    top_track_center = (0.0, -(LID_HEIGHT - 0.0105), 0.0040)
    display_lid.visual(
        top_track_bar,
        origin=_lid_origin(top_track_center),
        material=chassis_dark,
        name="top_track_bar",
    )

    display_lid.visual(
        Cylinder(radius=0.0017, length=0.0014),
        origin=_lid_origin((0.0, -(LID_HEIGHT - 0.0105), 0.0031)),
        material=screen_black,
        name="webcam_lens",
    )

    for side_name, side_x in (("left", HINGE_SPAN_X), ("right", -HINGE_SPAN_X)):
        display_lid.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_KNUCKLE_LENGTH),
            origin=_lid_origin((side_x, 0.0, 0.0)),
            material=chassis_dark,
            name=f"{side_name}_lid_knuckle",
        )
        display_lid.visual(
            Box((0.028, 0.018, 0.005)),
            origin=_lid_origin((side_x, -0.010, 0.0048)),
            material=chassis_dark,
            name=f"{side_name}_hinge_leaf",
        )

    privacy_shutter = model.part("privacy_shutter")
    privacy_shutter.inertial = Inertial.from_geometry(
        Box((SHUTTER_WIDTH, TRACK_SLOT_HEIGHT, TRACK_BAR_THICKNESS)),
        mass=0.01,
    )
    privacy_shutter.visual(
        Box((SHUTTER_WIDTH, TRACK_SLOT_HEIGHT, TRACK_BAR_THICKNESS)),
        origin=Origin(rpy=(-OPEN_ANGLE, 0.0, 0.0)),
        material=shutter_dark,
        name="slider_core",
    )
    privacy_shutter.visual(
        Box((0.0050, TRACK_BAR_HEIGHT * 0.55, 0.0007)),
        origin=Origin(xyz=(0.0035, 0.0, 0.0005), rpy=(-OPEN_ANGLE, 0.0, 0.0)),
        material=shutter_red,
        name="thumb_rib",
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_chassis,
        child=display_lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=-0.30,
            upper=OPEN_ANGLE,
        ),
    )

    model.articulation(
        "privacy_shutter_slide",
        ArticulationType.PRISMATIC,
        parent=display_lid,
        child=privacy_shutter,
        origin=Origin(xyz=_rot_x(top_track_center, -OPEN_ANGLE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.04,
            lower=-SHUTTER_TRAVEL,
            upper=SHUTTER_TRAVEL,
        ),
    )

    key_center_z = BASE_HEIGHT
    for spec in KEY_SPECS:
        name = spec["name"]  # type: ignore[misc]
        joint_name = spec["joint_name"]  # type: ignore[misc]
        center_x, center_y = spec["center"]  # type: ignore[misc]
        cap_w, cap_h = spec["cap_size"]  # type: ignore[misc]
        stem_w, stem_h = spec["stem_size"]  # type: ignore[misc]
        stem_offsets = spec["stem_offsets"]  # type: ignore[misc]

        key_part = model.part(name)
        key_part.inertial = Inertial.from_geometry(
            Box((cap_w, cap_h, KEY_STEM_LENGTH + KEY_CAP_THICKNESS)),
            mass=0.006,
            origin=Origin(xyz=(0.0, 0.0, 0.0001)),
        )
        key_part.visual(
            Box((cap_w, cap_h, KEY_CAP_THICKNESS)),
            origin=Origin(xyz=(0.0, 0.0, 0.0014)),
            material=keyboard_black,
            name="cap",
        )
        for stem_index, stem_offset in enumerate(stem_offsets):
            key_part.visual(
                Box((stem_w, stem_h, KEY_STEM_LENGTH)),
                origin=Origin(xyz=(stem_offset, 0.0, -0.00065)),
                material=keyboard_black,
                name=f"stem_{stem_index}",
            )

        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=lower_chassis,
            child=key_part,
            origin=Origin(xyz=(center_x, center_y, key_center_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=0.02,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
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

    lower_chassis = object_model.get_part("lower_chassis")
    display_lid = object_model.get_part("display_lid")
    privacy_shutter = object_model.get_part("privacy_shutter")
    lid_hinge = object_model.get_articulation("lid_hinge")
    shutter_slide = object_model.get_articulation("privacy_shutter_slide")
    g_key = object_model.get_part("g_key")
    space_bar = object_model.get_part("space_bar")
    g_key_joint = object_model.get_articulation("g_key_travel")
    space_bar_joint = object_model.get_articulation("space_bar_travel")

    ctx.check(
        "lid_hinge_axis",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"Expected lid hinge axis (1,0,0), got {lid_hinge.axis}",
    )
    ctx.check(
        "shutter_axis",
        tuple(shutter_slide.axis) == (1.0, 0.0, 0.0),
        details=f"Expected shutter axis (1,0,0), got {shutter_slide.axis}",
    )
    ctx.check(
        "key_axis",
        tuple(g_key_joint.axis) == (0.0, 0.0, -1.0),
        details=f"Expected key axis (0,0,-1), got {g_key_joint.axis}",
    )

    ctx.expect_contact(
        display_lid,
        lower_chassis,
        elem_a="left_lid_knuckle",
        elem_b="left_base_knuckle_outer",
        name="left_hinge_knuckle_contacts_base",
    )
    ctx.expect_contact(
        display_lid,
        lower_chassis,
        elem_a="right_lid_knuckle",
        elem_b="right_base_knuckle_inner",
        name="right_hinge_knuckle_contacts_base",
    )
    ctx.expect_contact(privacy_shutter, display_lid, name="shutter_remains_clipped_in_track")

    for key_name in KEY_PART_NAMES:
        ctx.expect_contact(key_name, lower_chassis, name=f"{key_name}_mounted")

    ctx.expect_within("backspace_key", lower_chassis, axes="xy", margin=0.0, name="backspace_within_deck")
    ctx.expect_within("left_shift_key", lower_chassis, axes="xy", margin=0.0, name="left_shift_within_deck")
    ctx.expect_within("arrow_right_key", lower_chassis, axes="xy", margin=0.0, name="arrow_cluster_within_deck")

    g_key_rest = ctx.part_world_position(g_key)
    space_rest = ctx.part_world_position(space_bar)
    shutter_rest = ctx.part_world_position(privacy_shutter)
    lid_rest_aabb = ctx.part_world_aabb(display_lid)
    assert g_key_rest is not None
    assert space_rest is not None
    assert shutter_rest is not None
    assert lid_rest_aabb is not None

    with ctx.pose({g_key_joint: KEY_TRAVEL, space_bar_joint: KEY_TRAVEL}):
        g_key_pressed = ctx.part_world_position(g_key)
        space_pressed = ctx.part_world_position(space_bar)
        assert g_key_pressed is not None
        assert space_pressed is not None
        ctx.check(
            "g_key_presses_down",
            g_key_pressed[2] < g_key_rest[2] - 0.0006,
            details=f"Expected g key to move down by at least 0.6 mm, got rest={g_key_rest}, pressed={g_key_pressed}",
        )
        ctx.check(
            "space_bar_presses_down",
            space_pressed[2] < space_rest[2] - 0.0006,
            details=f"Expected space bar to move down by at least 0.6 mm, got rest={space_rest}, pressed={space_pressed}",
        )
        ctx.expect_contact(g_key, lower_chassis, name="g_key_stays_guided")
        ctx.expect_contact(space_bar, lower_chassis, name="space_bar_stays_guided")

    with ctx.pose({shutter_slide: SHUTTER_TRAVEL}):
        shutter_right = ctx.part_world_position(privacy_shutter)
        assert shutter_right is not None
        ctx.check(
            "shutter_moves_right",
            shutter_right[0] > shutter_rest[0] + 0.004,
            details=f"Expected shutter to slide right, got rest={shutter_rest}, right={shutter_right}",
        )
        ctx.expect_within(
            privacy_shutter,
            display_lid,
            axes="x",
            inner_elem="slider_core",
            outer_elem="top_track_bar",
            margin=0.0,
            name="shutter_right_within_track",
        )
        ctx.expect_contact(privacy_shutter, display_lid, name="shutter_right_contact")

    with ctx.pose({shutter_slide: -SHUTTER_TRAVEL}):
        shutter_left = ctx.part_world_position(privacy_shutter)
        assert shutter_left is not None
        ctx.check(
            "shutter_moves_left",
            shutter_left[0] < shutter_rest[0] - 0.004,
            details=f"Expected shutter to slide left, got rest={shutter_rest}, left={shutter_left}",
        )
        ctx.expect_within(
            privacy_shutter,
            display_lid,
            axes="x",
            inner_elem="slider_core",
            outer_elem="top_track_bar",
            margin=0.0,
            name="shutter_left_within_track",
        )
        ctx.expect_contact(privacy_shutter, display_lid, name="shutter_left_contact")

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_closed_aabb = ctx.part_world_aabb(display_lid)
        assert lid_closed_aabb is not None
        ctx.check(
            "lid_rotates_closed",
            lid_closed_aabb[1][2] < lid_rest_aabb[1][2] - 0.11,
            details=f"Expected lid top to swing downward, got rest={lid_rest_aabb}, closed={lid_closed_aabb}",
        )
        ctx.expect_gap(
            display_lid,
            lower_chassis,
            axis="z",
            positive_elem="screen_glass",
            negative_elem="keyboard_plate",
            min_gap=0.001,
            max_gap=0.012,
            name="screen_clears_keyboard_when_near_closed",
        )
        ctx.expect_overlap(
            display_lid,
            lower_chassis,
            axes="xy",
            elem_a="screen_glass",
            elem_b="keyboard_plate",
            min_overlap=0.15,
            name="screen_stays_over_keyboard_footprint",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
