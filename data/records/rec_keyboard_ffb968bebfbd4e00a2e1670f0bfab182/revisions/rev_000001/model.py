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
    section_loft,
)


CASE_WIDTH = 0.365
CASE_DEPTH = 0.145
CASE_BOTTOM_Z = 0.008
CASE_HEIGHT = 0.020
PLATE_THICKNESS = 0.0024
PLATE_TOP_Z = CASE_BOTTOM_Z + CASE_HEIGHT
PLATE_UNDERSIDE_Z = PLATE_TOP_Z - PLATE_THICKNESS
KEY_PITCH = 0.019
FUNCTION_PITCH = 0.018
SWITCH_OPENING = 0.014


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _profile_at_z(
    profile: list[tuple[float, float]],
    z: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


def _make_keycap_mesh(
    *,
    width: float,
    depth: float,
    height: float,
    name: str,
) -> object:
    corner_radius = min(width, depth) * 0.12
    lower = rounded_rect_profile(
        width,
        depth,
        corner_radius,
        corner_segments=5,
    )
    upper = rounded_rect_profile(
        max(width - 0.0026, width * 0.84),
        max(depth - 0.0022, depth * 0.84),
        max(corner_radius * 0.85, 0.0008),
        corner_segments=5,
    )
    upper = _translate_profile(upper, 0.0, 0.00035)
    geom = section_loft(
        [
            _profile_at_z(lower, 0.0),
            _profile_at_z(upper, height),
        ]
    )
    return mesh_from_geometry(geom, name)


def _add_row(
    layout: list[dict[str, float | str]],
    *,
    row_y: float,
    left_edge: float,
    pitch: float,
    depth: float,
    cap_height: float,
    labels_and_units: list[tuple[str, float]],
) -> None:
    cursor = left_edge
    for label, units in labels_and_units:
        width_clearance = 0.0024
        if units >= 2.0:
            width_clearance = 0.0040
        if units >= 6.0:
            width_clearance = 0.0060
        layout.append(
            {
                "name": f"key_{label}",
                "x": cursor + (units * pitch * 0.5),
                "y": row_y,
                "width": max(units * pitch - width_clearance, 0.010),
                "depth": depth,
                "cap_height": cap_height,
            }
        )
        cursor += units * pitch


def _build_key_layout() -> list[dict[str, float | str]]:
    layout: list[dict[str, float | str]] = []

    main_left = -0.176
    nav_left = 0.119

    function_groups = [
        ["escape"],
        ["f1", "f2", "f3", "f4"],
        ["f5", "f6", "f7", "f8"],
        ["f9", "f10", "f11", "f12"],
        ["print_screen", "scroll_lock", "pause"],
    ]
    function_total_width = (
        sum(len(group) for group in function_groups) * FUNCTION_PITCH
        + (len(function_groups) - 1) * 0.011
    )
    function_cursor = -(function_total_width * 0.5)
    for group in function_groups:
        _add_row(
            layout,
            row_y=0.049,
            left_edge=function_cursor,
            pitch=FUNCTION_PITCH,
            depth=0.0144,
            cap_height=0.0062,
            labels_and_units=[(label, 1.0) for label in group],
        )
        function_cursor += len(group) * FUNCTION_PITCH + 0.011

    _add_row(
        layout,
        row_y=0.031,
        left_edge=main_left,
        pitch=KEY_PITCH,
        depth=0.0168,
        cap_height=0.0070,
        labels_and_units=[
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
        ],
    )
    _add_row(
        layout,
        row_y=0.012,
        left_edge=main_left,
        pitch=KEY_PITCH,
        depth=0.0168,
        cap_height=0.0070,
        labels_and_units=[
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
        ],
    )
    _add_row(
        layout,
        row_y=-0.007,
        left_edge=main_left,
        pitch=KEY_PITCH,
        depth=0.0168,
        cap_height=0.0070,
        labels_and_units=[
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
        ],
    )
    _add_row(
        layout,
        row_y=-0.026,
        left_edge=main_left,
        pitch=KEY_PITCH,
        depth=0.0168,
        cap_height=0.0070,
        labels_and_units=[
            ("left_shift", 2.25),
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
            ("right_shift", 2.75),
        ],
    )
    _add_row(
        layout,
        row_y=-0.045,
        left_edge=main_left,
        pitch=KEY_PITCH,
        depth=0.0162,
        cap_height=0.0070,
        labels_and_units=[
            ("left_ctrl", 1.25),
            ("left_win", 1.25),
            ("left_alt", 1.25),
            ("spacebar", 6.25),
            ("right_alt", 1.25),
            ("fn", 1.0),
            ("menu", 1.0),
            ("right_ctrl", 1.75),
        ],
    )

    _add_row(
        layout,
        row_y=0.031,
        left_edge=nav_left,
        pitch=KEY_PITCH,
        depth=0.0168,
        cap_height=0.0070,
        labels_and_units=[
            ("insert", 1.0),
            ("home", 1.0),
            ("page_up", 1.0),
        ],
    )
    _add_row(
        layout,
        row_y=0.012,
        left_edge=nav_left,
        pitch=KEY_PITCH,
        depth=0.0168,
        cap_height=0.0070,
        labels_and_units=[
            ("delete", 1.0),
            ("end", 1.0),
            ("page_down", 1.0),
        ],
    )
    _add_row(
        layout,
        row_y=-0.026,
        left_edge=nav_left + KEY_PITCH,
        pitch=KEY_PITCH,
        depth=0.0168,
        cap_height=0.0070,
        labels_and_units=[("up", 1.0)],
    )
    _add_row(
        layout,
        row_y=-0.045,
        left_edge=nav_left,
        pitch=KEY_PITCH,
        depth=0.0168,
        cap_height=0.0070,
        labels_and_units=[
            ("left", 1.0),
            ("down", 1.0),
            ("right", 1.0),
        ],
    )

    return layout


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tenkeyless_gaming_keyboard")

    case_body = model.material("case_body", rgba=(0.10, 0.11, 0.12, 1.0))
    case_plate = model.material("case_plate", rgba=(0.07, 0.08, 0.09, 1.0))
    key_cap = model.material("key_cap", rgba=(0.15, 0.16, 0.18, 1.0))
    key_stem = model.material("key_stem", rgba=(0.10, 0.10, 0.11, 1.0))
    roller_finish = model.material("roller_finish", rgba=(0.24, 0.26, 0.28, 1.0))
    roller_axle = model.material("roller_axle", rgba=(0.48, 0.50, 0.54, 1.0))
    foot_plastic = model.material("foot_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.03, 0.03, 0.03, 1.0))
    trim = model.material("trim", rgba=(0.20, 0.22, 0.24, 1.0))

    key_specs = _build_key_layout()
    switch_opening_profile = rounded_rect_profile(
        SWITCH_OPENING,
        SWITCH_OPENING,
        0.0012,
        corner_segments=3,
    )
    top_plate_holes = [
        _translate_profile(switch_opening_profile, float(spec["x"]), float(spec["y"]))
        for spec in key_specs
    ]
    top_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(CASE_WIDTH, CASE_DEPTH, 0.010, corner_segments=8),
            top_plate_holes,
            PLATE_THICKNESS,
            center=True,
        ),
        "keyboard_top_plate",
    )

    keyboard_case = model.part("keyboard_case")
    keyboard_case.visual(
        Box((CASE_WIDTH, CASE_DEPTH, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, CASE_BOTTOM_Z + 0.0015)),
        material=case_body,
        name="bottom_tray",
    )
    keyboard_case.visual(
        Box((CASE_WIDTH, 0.008, CASE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CASE_DEPTH * 0.5) + 0.004,
                CASE_BOTTOM_Z + (CASE_HEIGHT * 0.5),
            )
        ),
        material=case_body,
        name="front_wall",
    )
    keyboard_case.visual(
        Box((CASE_WIDTH, 0.008, CASE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (CASE_DEPTH * 0.5) - 0.004,
                CASE_BOTTOM_Z + (CASE_HEIGHT * 0.5),
            )
        ),
        material=case_body,
        name="rear_wall",
    )
    keyboard_case.visual(
        Box((0.006, CASE_DEPTH, CASE_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CASE_WIDTH * 0.5) + 0.003,
                0.0,
                CASE_BOTTOM_Z + (CASE_HEIGHT * 0.5),
            )
        ),
        material=case_body,
        name="left_wall",
    )
    keyboard_case.visual(
        Box((0.006, CASE_DEPTH, CASE_HEIGHT)),
        origin=Origin(
            xyz=(
                (CASE_WIDTH * 0.5) - 0.003,
                0.0,
                CASE_BOTTOM_Z + (CASE_HEIGHT * 0.5),
            )
        ),
        material=case_body,
        name="right_wall",
    )
    keyboard_case.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, PLATE_TOP_Z - (PLATE_THICKNESS * 0.5))),
        material=case_plate,
        name="top_plate",
    )
    keyboard_case.visual(
        Box((CASE_WIDTH - 0.012, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -(CASE_DEPTH * 0.5) + 0.010, PLATE_TOP_Z)),
        material=trim,
        name="front_trim",
    )
    keyboard_case.visual(
        Box((0.040, 0.020, 0.004)),
        origin=Origin(xyz=(0.145, 0.063, PLATE_TOP_Z + 0.002)),
        material=case_body,
        name="roller_housing_base",
    )
    keyboard_case.visual(
        Box((0.004, 0.014, 0.012)),
        origin=Origin(xyz=(0.131, 0.063, PLATE_TOP_Z + 0.008)),
        material=trim,
        name="roller_left_bracket",
    )
    keyboard_case.visual(
        Box((0.004, 0.014, 0.012)),
        origin=Origin(xyz=(0.159, 0.063, PLATE_TOP_Z + 0.008)),
        material=trim,
        name="roller_right_bracket",
    )
    keyboard_case.visual(
        Box((CASE_WIDTH - 0.020, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.060, CASE_BOTTOM_Z + 0.0015)),
        material=case_body,
        name="rear_underside_rail",
    )

    for side_name, foot_x in (("left", -0.112), ("right", 0.112)):
        keyboard_case.visual(
            Cylinder(radius=0.003, length=0.004),
            origin=Origin(
                xyz=(foot_x - 0.006, 0.060, CASE_BOTTOM_Z + 0.003),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=trim,
            name=f"{side_name}_foot_mount_outer",
        )
        keyboard_case.visual(
            Cylinder(radius=0.003, length=0.004),
            origin=Origin(
                xyz=(foot_x + 0.006, 0.060, CASE_BOTTOM_Z + 0.003),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=trim,
            name=f"{side_name}_foot_mount_inner",
        )

    keyboard_case.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_HEIGHT)),
        mass=1.45,
        origin=Origin(xyz=(0.0, 0.0, CASE_BOTTOM_Z + (CASE_HEIGHT * 0.5))),
    )

    keycap_mesh_cache: dict[tuple[int, int, int], object] = {}
    for spec in key_specs:
        width = float(spec["width"])
        depth = float(spec["depth"])
        cap_height = float(spec["cap_height"])
        cache_key = (
            int(round(width * 1000.0)),
            int(round(depth * 1000.0)),
            int(round(cap_height * 1000.0)),
        )
        key_mesh = keycap_mesh_cache.get(cache_key)
        if key_mesh is None:
            key_mesh = _make_keycap_mesh(
                width=width,
                depth=depth,
                height=cap_height,
                name=f"keycap_{cache_key[0]}x{cache_key[1]}x{cache_key[2]}",
            )
            keycap_mesh_cache[cache_key] = key_mesh

        key_part = model.part(str(spec["name"]))
        key_part.visual(
            key_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0048)),
            material=key_cap,
            name="cap",
        )
        key_part.visual(
            Box((0.0054, 0.0054, 0.0082)),
            origin=Origin(xyz=(0.0, 0.0, 0.0007)),
            material=key_stem,
            name="stem",
        )
        key_part.visual(
            Box((0.0150, 0.0150, 0.0012)),
            origin=Origin(xyz=(0.0, 0.0, -(PLATE_THICKNESS + 0.0006))),
            material=key_stem,
            name="retainer",
        )
        key_part.inertial = Inertial.from_geometry(
            Box((width, depth, 0.014)),
            mass=max(0.006, 0.0065 * width / KEY_PITCH),
            origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        )
        model.articulation(
            f"{spec['name']}_travel",
            ArticulationType.PRISMATIC,
            parent=keyboard_case,
            child=key_part,
            origin=Origin(xyz=(float(spec["x"]), float(spec["y"]), PLATE_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=0.08,
                lower=0.0,
                upper=0.0035,
            ),
        )

    media_roller = model.part("media_roller")
    media_roller.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=roller_finish,
        name="roller_barrel",
    )
    media_roller.visual(
        Cylinder(radius=0.0022, length=0.004),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=roller_axle,
        name="left_axle_pin",
    )
    media_roller.visual(
        Cylinder(radius=0.0022, length=0.004),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=roller_axle,
        name="right_axle_pin",
    )
    media_roller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.006, length=0.024),
        mass=0.018,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    model.articulation(
        "case_to_media_roller",
        ArticulationType.CONTINUOUS,
        parent=keyboard_case,
        child=media_roller,
        origin=Origin(xyz=(0.145, 0.063, PLATE_TOP_Z + 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=12.0),
    )

    for foot_name, foot_x in (("rear_foot_left", -0.112), ("rear_foot_right", 0.112)):
        foot_part = model.part(foot_name)
        foot_part.visual(
            Cylinder(radius=0.003, length=0.008),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=foot_plastic,
            name="hinge_barrel",
        )
        foot_part.visual(
            Box((0.008, 0.024, 0.008)),
            origin=Origin(xyz=(0.0, 0.012, -0.007)),
            material=foot_plastic,
            name="foot_arm",
        )
        foot_part.visual(
            Box((0.014, 0.020, 0.004)),
            origin=Origin(xyz=(0.0, 0.026, -0.011)),
            material=foot_plastic,
            name="foot_body",
        )
        foot_part.visual(
            Box((0.016, 0.007, 0.005)),
            origin=Origin(xyz=(0.0, 0.038, -0.0125)),
            material=foot_rubber,
            name="foot_tip",
        )
        foot_part.inertial = Inertial.from_geometry(
            Box((0.016, 0.045, 0.016)),
            mass=0.020,
            origin=Origin(xyz=(0.0, 0.020, -0.008)),
        )
        model.articulation(
            f"case_to_{foot_name}",
            ArticulationType.REVOLUTE,
            parent=keyboard_case,
            child=foot_part,
            origin=Origin(xyz=(foot_x, 0.060, CASE_BOTTOM_Z + 0.003)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=2.0,
                lower=-1.08,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    keyboard_case = object_model.get_part("keyboard_case")
    escape_key = object_model.get_part("key_escape")
    a_key = object_model.get_part("key_a")
    spacebar = object_model.get_part("key_spacebar")
    media_roller = object_model.get_part("media_roller")
    rear_foot_left = object_model.get_part("rear_foot_left")
    rear_foot_right = object_model.get_part("rear_foot_right")

    escape_travel = object_model.get_articulation("key_escape_travel")
    media_roller_joint = object_model.get_articulation("case_to_media_roller")
    left_foot_joint = object_model.get_articulation("case_to_rear_foot_left")
    right_foot_joint = object_model.get_articulation("case_to_rear_foot_right")

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

    key_count = sum(1 for part in object_model.parts if part.name.startswith("key_"))
    ctx.check(
        "dense_tkl_key_count",
        key_count == 87,
        f"expected 87 articulated keycaps, found {key_count}",
    )

    case_aabb = ctx.part_world_aabb(keyboard_case)
    if case_aabb is None:
        ctx.fail("keyboard_case_has_geometry", "keyboard case has no measurable geometry")
    else:
        (min_corner, max_corner) = case_aabb
        width = max_corner[0] - min_corner[0]
        depth = max_corner[1] - min_corner[1]
        height = max_corner[2] - min_corner[2]
        ctx.check(
            "keyboard_overall_proportions",
            0.35 <= width <= 0.39 and 0.13 <= depth <= 0.16 and 0.03 <= height <= 0.05,
            f"measured size was {width:.4f} x {depth:.4f} x {height:.4f} m",
        )

    ctx.check(
        "primary_joint_axes",
        escape_travel.axis == (0.0, 0.0, -1.0)
        and media_roller_joint.axis == (1.0, 0.0, 0.0)
        and left_foot_joint.axis == (1.0, 0.0, 0.0)
        and right_foot_joint.axis == (1.0, 0.0, 0.0),
        "expected vertical key travel, and x-axis rotation for roller and rear feet",
    )

    ctx.expect_contact(escape_key, keyboard_case, name="escape_key_clipped_to_plate")
    ctx.expect_contact(a_key, keyboard_case, name="a_key_clipped_to_plate")
    ctx.expect_contact(spacebar, keyboard_case, name="spacebar_clipped_to_plate")
    ctx.expect_contact(media_roller, keyboard_case, name="media_roller_supported")
    ctx.expect_contact(rear_foot_left, keyboard_case, name="left_rear_foot_retained_folded")
    ctx.expect_contact(rear_foot_right, keyboard_case, name="right_rear_foot_retained_folded")

    roller_position = ctx.part_world_position(media_roller)
    left_foot_position = ctx.part_world_position(rear_foot_left)
    right_foot_position = ctx.part_world_position(rear_foot_right)
    if roller_position is not None:
        ctx.check(
            "roller_in_upper_right_corner",
            roller_position[0] > 0.12 and roller_position[1] > 0.04,
            f"roller origin was at {roller_position}",
        )
    if left_foot_position is not None and right_foot_position is not None:
        ctx.check(
            "rear_feet_hinged_under_back_edge",
            left_foot_position[1] > 0.05
            and right_foot_position[1] > 0.05
            and left_foot_position[2] < PLATE_TOP_Z
            and right_foot_position[2] < PLATE_TOP_Z
            and math.isclose(abs(left_foot_position[0]), abs(right_foot_position[0]), rel_tol=0.0, abs_tol=1e-6),
            f"rear foot origins were {left_foot_position} and {right_foot_position}",
        )

    with ctx.pose({media_roller_joint: 1.2}):
        ctx.expect_contact(media_roller, keyboard_case, name="media_roller_retained_while_rotating")

    with ctx.pose({left_foot_joint: -1.0, right_foot_joint: -1.0}):
        ctx.expect_contact(rear_foot_left, keyboard_case, name="left_rear_foot_retained_open")
        ctx.expect_contact(rear_foot_right, keyboard_case, name="right_rear_foot_retained_open")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
