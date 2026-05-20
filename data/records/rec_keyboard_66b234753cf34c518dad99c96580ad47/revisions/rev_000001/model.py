from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_WIDTH = 0.396
BODY_DEPTH = 0.146
BODY_HEIGHT = 0.0113
BOTTOM_THICKNESS = 0.0022
RIM_WIDTH = 0.0060
DECK_THICKNESS = 0.0011
DECK_TOP_Z = 0.0089
DECK_BOTTOM_Z = DECK_TOP_Z - DECK_THICKNESS
INNER_WIDTH = BODY_WIDTH - 2.0 * RIM_WIDTH
INNER_DEPTH = BODY_DEPTH - 2.0 * RIM_WIDTH
STAND_RECESS_DEPTH = 0.0160
STAND_SILL_DEPTH = 0.0048
STAND_PANEL_DEPTH = 0.0112
STAND_PANEL_WIDTH = 0.348
STAND_PANEL_THICKNESS = 0.0022
STAND_HINGE_RADIUS = 0.0015
STAND_TOP_Z = 0.0106
KEY_PITCH = 0.0185
KEY_REST_GAP = 0.0014
KEY_TRAVEL = 0.0012
KEY_CAP_THICKNESS = 0.0022
KEY_SLIDER_HEIGHT = 0.0058
MAX_ROW_UNITS = 18.35


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _row_units(items: list[object]) -> float:
    total = 0.0
    for item in items:
        if isinstance(item, (int, float)):
            total += float(item)
        else:
            total += float(item[1])
    return total


def _build_key_specs() -> list[dict[str, float | str]]:
    rows = [
        (
            -0.0405,
            [
                0.475,
                ("esc", 1.0, 0.78),
                0.50,
                ("f1", 1.0, 0.78),
                ("f2", 1.0, 0.78),
                ("f3", 1.0, 0.78),
                ("f4", 1.0, 0.78),
                0.30,
                ("f5", 1.0, 0.78),
                ("f6", 1.0, 0.78),
                ("f7", 1.0, 0.78),
                ("f8", 1.0, 0.78),
                0.30,
                ("f9", 1.0, 0.78),
                ("f10", 1.0, 0.78),
                ("f11", 1.0, 0.78),
                ("f12", 1.0, 0.78),
                0.40,
                ("delete", 1.0, 0.78),
                ("home", 1.0, 0.78),
                ("end", 1.0, 0.78),
            ],
        ),
        (
            -0.0205,
            [
                ("grave", 1.0),
                ("digit_1", 1.0),
                ("digit_2", 1.0),
                ("digit_3", 1.0),
                ("digit_4", 1.0),
                ("digit_5", 1.0),
                ("digit_6", 1.0),
                ("digit_7", 1.0),
                ("digit_8", 1.0),
                ("digit_9", 1.0),
                ("digit_0", 1.0),
                ("minus", 1.0),
                ("equals", 1.0),
                ("backspace", 2.0),
                0.35,
                ("insert", 1.0),
                ("page_up", 1.0),
                ("page_down", 1.0),
            ],
        ),
        (
            -0.0008,
            [
                ("tab", 1.5),
                ("q", 1.0),
                ("w", 1.0),
                ("e", 1.0),
                ("r", 1.0),
                ("t", 1.0),
                ("y", 1.0),
                ("u", 1.0),
                ("i", 1.0),
                ("o", 1.0),
                ("p", 1.0),
                ("left_bracket", 1.0),
                ("right_bracket", 1.0),
                ("backslash", 1.5),
                0.35,
                ("del", 1.0),
                ("pg_up_secondary", 1.0),
                ("pg_down_secondary", 1.0),
            ],
        ),
        (
            0.0189,
            [
                ("caps_lock", 1.75),
                ("a", 1.0),
                ("s", 1.0),
                ("d", 1.0),
                ("f", 1.0),
                ("g", 1.0),
                ("h", 1.0),
                ("j", 1.0),
                ("k", 1.0),
                ("l", 1.0),
                ("semicolon", 1.0),
                ("apostrophe", 1.0),
                ("enter", 2.25),
                2.35,
                ("arrow_up", 1.0),
            ],
        ),
        (
            0.0386,
            [
                ("shift_left", 2.25),
                ("z", 1.0),
                ("x", 1.0),
                ("c", 1.0),
                ("v", 1.0),
                ("b", 1.0),
                ("n", 1.0),
                ("m", 1.0),
                ("comma", 1.0),
                ("period", 1.0),
                ("slash", 1.0),
                ("shift_right", 2.75),
                0.35,
                ("arrow_left", 1.0),
                ("arrow_down", 1.0),
                ("arrow_right", 1.0),
            ],
        ),
        (
            0.0584,
            [
                1.30,
                ("ctrl_left", 1.25),
                ("fn_left", 1.0),
                ("alt_left", 1.25),
                ("meta_left", 1.25),
                ("space", 6.25),
                ("meta_right", 1.25),
                ("alt_right", 1.25),
                ("fn_right", 1.0),
                ("ctrl_right", 1.25),
            ],
        ),
    ]

    specs: list[dict[str, float | str]] = []
    left_edge = -0.5 * MAX_ROW_UNITS * KEY_PITCH
    row_index = 0
    for y, items in rows:
        cursor_x = left_edge
        for item in items:
            if isinstance(item, (int, float)):
                cursor_x += float(item) * KEY_PITCH
                continue

            name = str(item[0])
            units = float(item[1])
            depth_scale = float(item[2]) if len(item) > 2 else 1.0
            key_width = units * KEY_PITCH - 0.0030
            key_depth = 0.0156 * depth_scale
            slider_width = min(0.0125, key_width - 0.0040)
            slider_depth = min(0.0112, key_depth - 0.0036)
            center_x = cursor_x + units * KEY_PITCH * 0.5
            specs.append(
                {
                    "part": f"key_{name}",
                    "joint": f"key_{name}_travel",
                    "visual_cap": "cap_top",
                    "visual_slider": "slider",
                    "x": center_x,
                    "y": y,
                    "units": units,
                    "row": row_index,
                    "cap_width": key_width,
                    "cap_depth": key_depth,
                    "slider_width": slider_width,
                    "slider_depth": slider_depth,
                }
            )
            cursor_x += units * KEY_PITCH
        row_index += 1
    return specs


KEY_SPECS = _build_key_specs()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wireless_keyboard_with_tablet_stand")

    body_shell = model.material("body_shell", rgba=(0.27, 0.29, 0.31, 1.0))
    body_trim = model.material("body_trim", rgba=(0.18, 0.19, 0.21, 1.0))
    deck_finish = model.material("deck_finish", rgba=(0.21, 0.23, 0.25, 1.0))
    keycap_finish = model.material("keycap_finish", rgba=(0.14, 0.15, 0.17, 1.0))
    slider_finish = model.material("slider_finish", rgba=(0.11, 0.12, 0.13, 1.0))
    stand_finish = model.material("stand_finish", rgba=(0.24, 0.26, 0.29, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")

    outer_profile = rounded_rect_profile(BODY_WIDTH, BODY_DEPTH, radius=0.013, corner_segments=10)
    inner_profile = rounded_rect_profile(INNER_WIDTH, INNER_DEPTH, radius=0.009, corner_segments=10)

    bottom_geom = ExtrudeGeometry(outer_profile, BOTTOM_THICKNESS, center=True)
    body.visual(
        _mesh("keyboard_bottom_plate", bottom_geom.translate(0.0, 0.0, BOTTOM_THICKNESS * 0.5)),
        material=body_shell,
        name="bottom_plate",
    )

    ring_height = BODY_HEIGHT - BOTTOM_THICKNESS
    rim_geom = ExtrudeWithHolesGeometry(outer_profile, [inner_profile], ring_height, center=True)
    body.visual(
        _mesh("keyboard_rim_ring", rim_geom.translate(0.0, 0.0, BOTTOM_THICKNESS + ring_height * 0.5)),
        material=body_shell,
        name="rim_ring",
    )

    deck_rear_edge = -INNER_DEPTH * 0.5 + STAND_RECESS_DEPTH
    deck_depth = INNER_DEPTH - STAND_RECESS_DEPTH
    deck_center_y = deck_rear_edge + deck_depth * 0.5
    deck_profile = rounded_rect_profile(INNER_WIDTH, deck_depth, radius=0.006, corner_segments=8)

    deck_holes = [
        _rect_profile(float(spec["slider_width"]), float(spec["slider_depth"]))
        for spec in KEY_SPECS
    ]
    for hole_profile, spec in zip(deck_holes, KEY_SPECS):
        dx = float(spec["x"])
        dy = float(spec["y"]) - deck_center_y
        for index, (px, py) in enumerate(hole_profile):
            hole_profile[index] = (px + dx, py + dy)

    deck_geom = ExtrudeWithHolesGeometry(deck_profile, deck_holes, DECK_THICKNESS, center=True)
    body.visual(
        _mesh("keyboard_deck_plate", deck_geom.translate(0.0, deck_center_y, DECK_BOTTOM_Z + DECK_THICKNESS * 0.5)),
        material=deck_finish,
        name="deck_plate",
    )

    sill_depth = STAND_SILL_DEPTH + 0.0004
    sill_center_y = deck_rear_edge - STAND_SILL_DEPTH * 0.5
    sill_width = STAND_PANEL_WIDTH + 0.018
    sill_height = 0.0018
    body.visual(
        Box((sill_width, sill_depth, sill_height)),
        origin=Origin(xyz=(0.0, sill_center_y, DECK_TOP_Z + sill_height * 0.5)),
        material=body_trim,
        name="stand_sill",
    )

    hinge_axis_y = -INNER_DEPTH * 0.5 + STAND_HINGE_RADIUS
    body.visual(
        Box((STAND_PANEL_WIDTH, 0.0022, BODY_HEIGHT - DECK_BOTTOM_Z)),
        origin=Origin(
            xyz=(
                0.0,
                -INNER_DEPTH * 0.5 + 0.0011,
                DECK_BOTTOM_Z + (BODY_HEIGHT - DECK_BOTTOM_Z) * 0.5,
            )
        ),
        material=body_trim,
        name="rear_hinge_wall",
    )

    clip_span = STAND_PANEL_WIDTH * 0.5 - 0.028
    for clip_name, clip_x in (
        ("stand_clip_left", -clip_span),
        ("stand_clip_center", 0.0),
        ("stand_clip_right", clip_span),
    ):
        body.visual(
            Box((0.020, 0.0038, 0.0012)),
            origin=Origin(xyz=(clip_x, hinge_axis_y - 0.0003, 0.0110)),
            material=body_trim,
            name=clip_name,
        )

    for foot_name, foot_x, foot_y in (
        ("foot_front_left", -0.145, 0.050),
        ("foot_front_right", 0.145, 0.050),
        ("foot_rear_left", -0.145, -0.050),
        ("foot_rear_right", 0.145, -0.050),
    ):
        body.visual(
            Box((0.030, 0.008, 0.0012)),
            origin=Origin(xyz=(foot_x, foot_y, 0.0006)),
            material=foot_rubber,
            name=foot_name,
        )

    stand = model.part("stand_flap")
    stand_panel_center_y = STAND_PANEL_DEPTH * 0.5 - STAND_HINGE_RADIUS
    stand_panel_center_z = STAND_PANEL_THICKNESS * 0.5 - STAND_HINGE_RADIUS
    stand.visual(
        Box((STAND_PANEL_WIDTH, STAND_PANEL_DEPTH, STAND_PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, stand_panel_center_y, stand_panel_center_z)),
        material=stand_finish,
        name="stand_panel",
    )
    stand.visual(
        Cylinder(radius=STAND_HINGE_RADIUS, length=STAND_PANEL_WIDTH - 0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=stand_finish,
        name="hinge_rod",
    )
    stand.visual(
        Box((STAND_PANEL_WIDTH, 0.0024, 0.0026)),
        origin=Origin(
            xyz=(
                0.0,
                STAND_PANEL_DEPTH - 0.0014 - STAND_HINGE_RADIUS,
                0.0012 - STAND_HINGE_RADIUS,
            )
        ),
        material=stand_finish,
        name="support_lip",
    )

    stand_joint = model.articulation(
        "stand_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, hinge_axis_y, STAND_TOP_Z - STAND_PANEL_THICKNESS + STAND_HINGE_RADIUS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    for spec in KEY_SPECS:
        key_part = model.part(str(spec["part"]))
        key_part.visual(
            Box((float(spec["cap_width"]), float(spec["cap_depth"]), KEY_CAP_THICKNESS)),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    DECK_TOP_Z + KEY_REST_GAP + KEY_CAP_THICKNESS * 0.5
                    - (DECK_BOTTOM_Z + KEY_SLIDER_HEIGHT * 0.5),
                )
            ),
            material=keycap_finish,
            name="cap_top",
        )
        key_part.visual(
            Box((float(spec["slider_width"]), float(spec["slider_depth"]), KEY_SLIDER_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=slider_finish,
            name="slider",
        )
        model.articulation(
            str(spec["joint"]),
            ArticulationType.PRISMATIC,
            parent=body,
            child=key_part,
            origin=Origin(xyz=(float(spec["x"]), float(spec["y"]), DECK_BOTTOM_Z + KEY_SLIDER_HEIGHT * 0.5)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=0.2, lower=0.0, upper=KEY_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    stand = object_model.get_part("stand_flap")
    stand_hinge = object_model.get_articulation("stand_flap_hinge")
    sample_alpha = object_model.get_part("key_g")
    sample_alpha_joint = object_model.get_articulation("key_g_travel")
    sample_space = object_model.get_part("key_space")
    sample_space_joint = object_model.get_articulation("key_space_travel")
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

    expected_parts = {"body", "stand_flap"} | {str(spec["part"]) for spec in KEY_SPECS}
    expected_joints = {"stand_flap_hinge"} | {str(spec["joint"]) for spec in KEY_SPECS}
    actual_parts = {part.name for part in object_model.parts}
    actual_joints = {joint.name for joint in object_model.articulations}
    ctx.check(
        "all_expected_parts_present",
        actual_parts == expected_parts,
        details=f"expected {len(expected_parts)} parts, found {len(actual_parts)}",
    )
    ctx.check(
        "all_expected_articulations_present",
        actual_joints == expected_joints,
        details=f"expected {len(expected_joints)} articulations, found {len(actual_joints)}",
    )
    ctx.check(
        "stand_hinge_axis_is_horizontal",
        tuple(round(value, 6) for value in stand_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={stand_hinge.axis}",
    )
    ctx.check(
        "sample_key_axis_is_vertical",
        tuple(round(value, 6) for value in sample_alpha_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={sample_alpha_joint.axis}",
    )

    deck_min_x = -INNER_WIDTH * 0.5
    deck_max_x = INNER_WIDTH * 0.5
    deck_min_y = -INNER_DEPTH * 0.5 + STAND_RECESS_DEPTH
    deck_max_y = INNER_DEPTH * 0.5
    key_positions_ok = True
    position_issues: list[str] = []
    for spec in KEY_SPECS:
        pos = ctx.part_world_position(str(spec["part"]))
        if pos is None:
            key_positions_ok = False
            position_issues.append(f"{spec['part']}: missing position")
            continue
        if not (deck_min_x <= pos[0] <= deck_max_x and deck_min_y <= pos[1] <= deck_max_y):
            key_positions_ok = False
            position_issues.append(f"{spec['part']}: {tuple(round(v, 4) for v in pos)}")
    ctx.check(
        "all_keys_positioned_over_deck",
        key_positions_ok,
        details="; ".join(position_issues[:10]),
    )

    for spec in KEY_SPECS:
        ctx.expect_contact(
            str(spec["part"]),
            body,
            elem_a="slider",
            elem_b="deck_plate",
            name=f"{spec['part']}_guided_in_deck",
        )

    with ctx.pose({stand_hinge: 0.0}):
        ctx.expect_contact(
            stand,
            body,
            elem_a="hinge_rod",
            elem_b="rear_hinge_wall",
            name="stand_hinge_kept_clipped_at_rear_wall",
        )
        ctx.expect_gap(
            body,
            stand,
            axis="y",
            positive_elem="stand_sill",
            negative_elem="support_lip",
            min_gap=0.0,
            max_gap=0.0005,
            name="stand_flap_front_lip_stays_tucked_behind_sill_when_closed",
        )

    with ctx.pose({stand_hinge: 0.98}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_stand_pose_no_part_overlap")
        ctx.expect_contact(
            stand,
            body,
            elem_a="hinge_rod",
            elem_b="rear_hinge_wall",
            name="open_stand_remains_captured_by_hinge_line",
        )
        ctx.expect_gap(
            stand,
            body,
            axis="z",
            positive_elem="support_lip",
            negative_elem="stand_sill",
            min_gap=0.004,
            name="open_stand_lifts_support_lip_above_slot_sill",
        )
        ctx.expect_gap(
            body,
            stand,
            axis="y",
            positive_elem="stand_sill",
            negative_elem="support_lip",
            min_gap=0.0025,
            name="open_stand_creates_front_tablet_slot_gap",
        )

    with ctx.pose({sample_alpha_joint: KEY_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="pressed_alpha_key_pose_no_part_overlap")
        ctx.expect_gap(
            sample_alpha,
            body,
            axis="z",
            positive_elem="cap_top",
            negative_elem="deck_plate",
            min_gap=0.0,
            max_gap=0.0004,
            name="alpha_key_can_press_without_hitting_deck",
        )

    with ctx.pose({sample_space_joint: KEY_TRAVEL}):
        ctx.expect_gap(
            sample_space,
            body,
            axis="z",
            positive_elem="cap_top",
            negative_elem="deck_plate",
            min_gap=0.0,
            max_gap=0.0004,
            name="spacebar_can_press_without_hitting_deck",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
