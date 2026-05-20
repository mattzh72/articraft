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
    Inertial,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_side_loft,
)

ASSETS = AssetContext.from_script(__file__)

UNIT = 0.01905
TOTAL_WIDTH_UNITS = 18.25
LEFT_EDGE_X = -(TOTAL_WIDTH_UNITS * UNIT) / 2.0
CASE_WIDTH = TOTAL_WIDTH_UNITS * UNIT + 0.016
CASE_DEPTH = 0.144
CASE_FRONT_HEIGHT = 0.0102
CASE_REAR_HEIGHT = 0.0215
PLATE_WIDTH = CASE_WIDTH - 0.012
PLATE_DEPTH = 0.118
PLATE_CENTER_Y = -0.001
PLATE_THICKNESS = 0.002
PLATE_CENTER_Z = 0.0107
SWITCH_OUTER = 0.0140
SWITCH_WALL = 0.00135
SWITCH_HEIGHT = 0.0093
SWITCH_BOTTOM_Z = 0.0115
SWITCH_TOP_Z = SWITCH_BOTTOM_Z + SWITCH_HEIGHT
KEY_TRAVEL = 0.0022
CAP_DEPTH = UNIT - 0.0010
CAP_CLEARANCE_AT_BOTTOM = 0.0005
STEM_WIDTH = 0.0042
STEM_DEPTH = 0.0040
STEM_HEIGHT = 0.0036
STEM_CENTER_Z = 0.0018

ROW_Y = {
    "function": 0.0505,
    "number": 0.0286,
    "qwerty": 0.00955,
    "home": -0.0095,
    "shift": -0.02855,
    "bottom": -0.0476,
}

ROW_STYLE = {
    "function": {"angle": math.radians(11.0), "height": 0.0078},
    "number": {"angle": math.radians(7.0), "height": 0.0081},
    "qwerty": {"angle": math.radians(4.0), "height": 0.0086},
    "home": {"angle": math.radians(1.0), "height": 0.0090},
    "shift": {"angle": math.radians(-4.0), "height": 0.0086},
    "bottom": {"angle": math.radians(-8.0), "height": 0.0080},
}


def _position_from_units(start_u: float, width_u: float) -> float:
    return LEFT_EDGE_X + (start_u + width_u / 2.0) * UNIT


def _append_row(
    specs: list[dict[str, float | str]],
    *,
    row: str,
    y: float,
    start_u: float,
    keys: list[tuple[str, float]],
) -> None:
    cursor = start_u
    for name, width_u in keys:
        specs.append(
            {
                "name": name,
                "width_u": width_u,
                "start_u": cursor,
                "x": _position_from_units(cursor, width_u),
                "y": y,
                "row": row,
            }
        )
        cursor += width_u


def _build_key_specs() -> list[dict[str, float | str]]:
    specs: list[dict[str, float | str]] = []

    function_row = [
        ("key_esc", 0.0, 1.0),
        ("key_f1", 2.0, 1.0),
        ("key_f2", 3.0, 1.0),
        ("key_f3", 4.0, 1.0),
        ("key_f4", 5.0, 1.0),
        ("key_f5", 6.5, 1.0),
        ("key_f6", 7.5, 1.0),
        ("key_f7", 8.5, 1.0),
        ("key_f8", 9.5, 1.0),
        ("key_f9", 11.0, 1.0),
        ("key_f10", 12.0, 1.0),
        ("key_f11", 13.0, 1.0),
        ("key_f12", 14.0, 1.0),
        ("key_print_screen", 15.25, 1.0),
        ("key_scroll_lock", 16.25, 1.0),
        ("key_pause", 17.25, 1.0),
    ]
    for name, start_u, width_u in function_row:
        specs.append(
            {
                "name": name,
                "width_u": width_u,
                "start_u": start_u,
                "x": _position_from_units(start_u, width_u),
                "y": ROW_Y["function"],
                "row": "function",
            }
        )

    _append_row(
        specs,
        row="number",
        y=ROW_Y["number"],
        start_u=0.0,
        keys=[
            ("key_grave", 1.0),
            ("key_1", 1.0),
            ("key_2", 1.0),
            ("key_3", 1.0),
            ("key_4", 1.0),
            ("key_5", 1.0),
            ("key_6", 1.0),
            ("key_7", 1.0),
            ("key_8", 1.0),
            ("key_9", 1.0),
            ("key_0", 1.0),
            ("key_minus", 1.0),
            ("key_equals", 1.0),
            ("key_backspace", 2.0),
        ],
    )
    _append_row(
        specs,
        row="number",
        y=ROW_Y["number"],
        start_u=15.25,
        keys=[
            ("key_insert", 1.0),
            ("key_home", 1.0),
            ("key_page_up", 1.0),
        ],
    )

    _append_row(
        specs,
        row="qwerty",
        y=ROW_Y["qwerty"],
        start_u=0.0,
        keys=[
            ("key_tab", 1.5),
            ("key_q", 1.0),
            ("key_w", 1.0),
            ("key_e", 1.0),
            ("key_r", 1.0),
            ("key_t", 1.0),
            ("key_y", 1.0),
            ("key_u", 1.0),
            ("key_i", 1.0),
            ("key_o", 1.0),
            ("key_p", 1.0),
            ("key_left_bracket", 1.0),
            ("key_right_bracket", 1.0),
            ("key_backslash", 1.5),
        ],
    )
    _append_row(
        specs,
        row="qwerty",
        y=ROW_Y["qwerty"],
        start_u=15.25,
        keys=[
            ("key_delete", 1.0),
            ("key_end", 1.0),
            ("key_page_down", 1.0),
        ],
    )

    _append_row(
        specs,
        row="home",
        y=ROW_Y["home"],
        start_u=0.0,
        keys=[
            ("key_caps_lock", 1.75),
            ("key_a", 1.0),
            ("key_s", 1.0),
            ("key_d", 1.0),
            ("key_f", 1.0),
            ("key_g", 1.0),
            ("key_h", 1.0),
            ("key_j", 1.0),
            ("key_k", 1.0),
            ("key_l", 1.0),
            ("key_semicolon", 1.0),
            ("key_apostrophe", 1.0),
            ("key_enter", 2.25),
        ],
    )

    _append_row(
        specs,
        row="shift",
        y=ROW_Y["shift"],
        start_u=0.0,
        keys=[
            ("key_left_shift", 2.25),
            ("key_z", 1.0),
            ("key_x", 1.0),
            ("key_c", 1.0),
            ("key_v", 1.0),
            ("key_b", 1.0),
            ("key_n", 1.0),
            ("key_m", 1.0),
            ("key_comma", 1.0),
            ("key_period", 1.0),
            ("key_slash", 1.0),
            ("key_right_shift", 2.75),
        ],
    )
    _append_row(
        specs,
        row="shift",
        y=ROW_Y["shift"],
        start_u=16.25,
        keys=[("key_arrow_up", 1.0)],
    )

    _append_row(
        specs,
        row="bottom",
        y=ROW_Y["bottom"],
        start_u=0.0,
        keys=[
            ("key_left_ctrl", 1.25),
            ("key_left_meta", 1.25),
            ("key_left_alt", 1.25),
            ("key_space", 6.25),
            ("key_right_alt", 1.25),
            ("key_fn", 1.25),
            ("key_menu", 1.25),
            ("key_right_ctrl", 1.25),
        ],
    )
    _append_row(
        specs,
        row="bottom",
        y=ROW_Y["bottom"],
        start_u=15.25,
        keys=[
            ("key_arrow_left", 1.0),
            ("key_arrow_down", 1.0),
            ("key_arrow_right", 1.0),
        ],
    )
    return specs


KEY_SPECS = _build_key_specs()
KEY_BY_NAME = {spec["name"]: spec for spec in KEY_SPECS}
ACCENT_KEYS = {"key_esc", "key_enter"}
SECONDARY_KEYS = {
    "key_print_screen",
    "key_scroll_lock",
    "key_pause",
    "key_insert",
    "key_home",
    "key_page_up",
    "key_delete",
    "key_end",
    "key_page_down",
    "key_caps_lock",
    "key_tab",
    "key_backspace",
    "key_backslash",
    "key_left_shift",
    "key_right_shift",
    "key_left_ctrl",
    "key_left_meta",
    "key_left_alt",
    "key_space",
    "key_right_alt",
    "key_fn",
    "key_menu",
    "key_right_ctrl",
    "key_arrow_up",
    "key_arrow_left",
    "key_arrow_down",
    "key_arrow_right",
}


def _rounded_rect_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius=radius,
            corner_segments=8,
        )
    ]


def _make_case_mesh() -> Mesh:
    geometry = superellipse_side_loft(
        [
            (-CASE_DEPTH / 2.0, 0.0, CASE_FRONT_HEIGHT, CASE_WIDTH - 0.008),
            (-0.020, 0.0, 0.0109, CASE_WIDTH - 0.005),
            (0.022, 0.0, 0.0135, CASE_WIDTH - 0.002),
            (CASE_DEPTH / 2.0, 0.0, CASE_REAR_HEIGHT, CASE_WIDTH),
        ],
        exponents=4.2,
        segments=72,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / "keyboard_case.obj")


def _make_unit_keycap_mesh() -> Mesh:
    geometry = section_loft(
        [
            _rounded_rect_section(1.0, 1.0, 0.16, 0.0),
            _rounded_rect_section(0.95, 0.95, 0.18, 0.32),
            _rounded_rect_section(0.88, 0.86, 0.18, 0.68),
            _rounded_rect_section(0.80, 0.76, 0.16, 1.0),
        ]
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / "keyboard_keycap_unit.obj")


def _keycap_material_name(key_name: str) -> str:
    if key_name in ACCENT_KEYS:
        return "accent_keycap"
    if key_name in SECONDARY_KEYS:
        return "secondary_keycap"
    return "main_keycap"


def _cap_origin_z_for_row(row: str) -> float:
    angle = ROW_STYLE[row]["angle"]
    return KEY_TRAVEL + CAP_CLEARANCE_AT_BOTTOM + abs(math.sin(angle)) * (CAP_DEPTH / 2.0)


def _add_switch_housing(base_part, x: float, y: float, housing_material: str) -> None:
    center_z = SWITCH_BOTTOM_Z + SWITCH_HEIGHT / 2.0
    inner_span = SWITCH_OUTER - 2.0 * SWITCH_WALL
    side_x = (SWITCH_OUTER - SWITCH_WALL) / 2.0
    front_y = (SWITCH_OUTER - SWITCH_WALL) / 2.0
    base_part.visual(
        Box((SWITCH_WALL, SWITCH_OUTER, SWITCH_HEIGHT)),
        origin=Origin(xyz=(x - side_x, y, center_z)),
        material=housing_material,
    )
    base_part.visual(
        Box((SWITCH_WALL, SWITCH_OUTER, SWITCH_HEIGHT)),
        origin=Origin(xyz=(x + side_x, y, center_z)),
        material=housing_material,
    )
    base_part.visual(
        Box((inner_span, SWITCH_WALL, SWITCH_HEIGHT)),
        origin=Origin(xyz=(x, y - front_y, center_z)),
        material=housing_material,
    )
    base_part.visual(
        Box((inner_span, SWITCH_WALL, SWITCH_HEIGHT)),
        origin=Origin(xyz=(x, y + front_y, center_z)),
        material=housing_material,
    )


def _add_key(model: ArticulatedObject, keycap_mesh_filename: str, spec: dict[str, float | str]) -> None:
    name = str(spec["name"])
    width_u = float(spec["width_u"])
    row = str(spec["row"])
    angle = ROW_STYLE[row]["angle"]
    keycap_height = ROW_STYLE[row]["height"]
    cap_origin_z = _cap_origin_z_for_row(row)
    cap_width = width_u * UNIT - 0.0010

    key_part = model.part(name)
    key_part.visual(
        Mesh(
            filename=keycap_mesh_filename,
            scale=(cap_width, CAP_DEPTH, keycap_height),
        ),
        origin=Origin(
            xyz=(0.0, 0.0, cap_origin_z),
            rpy=(angle, 0.0, 0.0),
        ),
        material=_keycap_material_name(name),
    )
    key_part.visual(
        Box((STEM_WIDTH, STEM_DEPTH, STEM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, STEM_CENTER_Z)),
        material="stem",
    )

    if name in {"key_f", "key_j"}:
        key_part.visual(
            Box((cap_width * 0.42, 0.0012, 0.00055)),
            origin=Origin(
                xyz=(0.0, 0.0048, cap_origin_z + keycap_height * 0.88),
                rpy=(angle, 0.0, 0.0),
            ),
            material="homing_bar",
        )

    key_part.inertial = Inertial.from_geometry(
        Box((max(cap_width, STEM_WIDTH), CAP_DEPTH, keycap_height + STEM_HEIGHT)),
        mass=0.007 + 0.0025 * width_u,
        origin=Origin(xyz=(0.0, 0.0, (cap_origin_z + keycap_height) * 0.5)),
    )

    model.articulation(
        f"press_{name}",
        ArticulationType.PRISMATIC,
        parent="base",
        child=name,
        origin=Origin(
            xyz=(float(spec["x"]), float(spec["y"]), SWITCH_TOP_Z),
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.25,
            lower=0.0,
            upper=KEY_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_keyboard_tkl", assets=ASSETS)

    model.material("anodized_aluminum", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("plate_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("accent_strip", rgba=(0.30, 0.34, 0.39, 1.0))
    model.material("switch_housing", rgba=(0.06, 0.06, 0.07, 1.0))
    model.material("stem", rgba=(0.47, 0.26, 0.14, 1.0))
    model.material("main_keycap", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("secondary_keycap", rgba=(0.21, 0.22, 0.24, 1.0))
    model.material("accent_keycap", rgba=(0.42, 0.51, 0.60, 1.0))
    model.material("homing_bar", rgba=(0.70, 0.71, 0.72, 1.0))
    model.material("usb_shell", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("usb_tongue", rgba=(0.35, 0.35, 0.37, 1.0))

    case_mesh = _make_case_mesh()
    keycap_mesh = _make_unit_keycap_mesh()

    base = model.part("base")
    base.visual(case_mesh, material="anodized_aluminum")
    base.visual(
        Box((PLATE_WIDTH, PLATE_DEPTH, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, PLATE_CENTER_Y, PLATE_CENTER_Z)),
        material="plate_finish",
    )
    base.visual(
        Box((CASE_WIDTH - 0.030, 0.010, 0.0024)),
        origin=Origin(xyz=(0.0, CASE_DEPTH / 2.0 - 0.013, 0.0178)),
        material="accent_strip",
    )
    base.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_REAR_HEIGHT)),
        mass=1.45,
        origin=Origin(xyz=(0.0, 0.0, CASE_REAR_HEIGHT / 2.0)),
    )

    for spec in KEY_SPECS:
        _add_switch_housing(base, float(spec["x"]), float(spec["y"]), "switch_housing")

    usb_c_port = model.part("usb_c_port")
    usb_c_port.visual(
        Box((0.0115, 0.0044, 0.0032)),
        material="usb_shell",
    )
    usb_c_port.visual(
        Box((0.0064, 0.0018, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0005, 0.0)),
        material="usb_tongue",
    )
    usb_c_port.inertial = Inertial.from_geometry(
        Box((0.0115, 0.0044, 0.0032)),
        mass=0.01,
    )
    model.articulation(
        "mount_usb_c",
        ArticulationType.FIXED,
        parent="base",
        child="usb_c_port",
        origin=Origin(xyz=(0.0, CASE_DEPTH / 2.0 - 0.0022, 0.0128)),
    )

    for spec in KEY_SPECS:
        _add_key(model, keycap_mesh.filename, spec)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected(use="visual")
    for spec in KEY_SPECS:
        ctx.allow_overlap(
            "base",
            str(spec["name"]),
            reason="moving key stem intentionally nests into the switch housing inside the case",
        )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=96,
        overlap_tol=0.0005,
        overlap_volume_tol=0.0,
    )
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.0005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_joint_motion_axis(
        "press_key_a",
        "key_a",
        world_axis="z",
        direction="negative",
        min_delta=0.0015,
    )
    ctx.expect_joint_motion_axis(
        "press_key_space",
        "key_space",
        world_axis="z",
        direction="negative",
        min_delta=0.0015,
    )
    ctx.expect_joint_motion_axis(
        "press_key_enter",
        "key_enter",
        world_axis="z",
        direction="negative",
        min_delta=0.0015,
    )
    ctx.expect_aabb_contact("usb_c_port", "base")

    key_parts = [part for part in object_model.parts if part.name.startswith("key_")]
    prismatic_keys = [
        articulation
        for articulation in object_model.articulations
        if articulation.articulation_type == ArticulationType.PRISMATIC
    ]
    if len(KEY_SPECS) != 87:
        raise AssertionError(f"Expected 87 keys for a TKL layout, got {len(KEY_SPECS)}")
    if len(key_parts) != 87:
        raise AssertionError(f"Expected 87 key parts, got {len(key_parts)}")
    if len(prismatic_keys) != 87:
        raise AssertionError(f"Expected 87 moving switch joints, got {len(prismatic_keys)}")

    def assert_close(actual: float, expected: float, tol: float, label: str) -> None:
        if abs(actual - expected) > tol:
            raise AssertionError(f"{label}: expected {expected:.6f}, got {actual:.6f}")

    for spec in KEY_SPECS:
        position = ctx.part_world_position(str(spec["name"]))
        assert_close(position[0], float(spec["x"]), 1e-6, f"{spec['name']} x")
        assert_close(position[1], float(spec["y"]), 1e-6, f"{spec['name']} y")
        assert_close(position[2], SWITCH_TOP_Z, 1e-6, f"{spec['name']} z")

    pos_q = ctx.part_world_position("key_q")
    pos_a = ctx.part_world_position("key_a")
    pos_s = ctx.part_world_position("key_s")
    pos_z = ctx.part_world_position("key_z")
    pos_backspace = ctx.part_world_position("key_backspace")
    pos_insert = ctx.part_world_position("key_insert")
    pos_print_screen = ctx.part_world_position("key_print_screen")
    pos_space = ctx.part_world_position("key_space")
    pos_enter = ctx.part_world_position("key_enter")
    pos_arrow_up = ctx.part_world_position("key_arrow_up")
    pos_arrow_down = ctx.part_world_position("key_arrow_down")
    pos_arrow_left = ctx.part_world_position("key_arrow_left")
    pos_arrow_right = ctx.part_world_position("key_arrow_right")

    assert_close(pos_s[0] - pos_a[0], UNIT, 1e-6, "A-S pitch")
    assert_close(pos_a[0] - pos_q[0], 0.25 * UNIT, 1e-6, "Q to A stagger")
    assert_close(pos_z[0] - pos_a[0], 0.5 * UNIT, 1e-6, "A to Z stagger")
    assert_close(pos_insert[0] - pos_backspace[0], 1.75 * UNIT, 1e-6, "main-to-nav gap")
    assert_close(pos_print_screen[0], pos_insert[0], 1e-6, "top nav alignment")
    assert_close(pos_arrow_up[0], pos_arrow_down[0], 1e-6, "arrow vertical alignment")
    assert_close(pos_arrow_down[0] - pos_arrow_left[0], UNIT, 1e-6, "left-down pitch")
    assert_close(pos_arrow_right[0] - pos_arrow_down[0], UNIT, 1e-6, "down-right pitch")
    if pos_arrow_up[1] <= pos_arrow_down[1]:
        raise AssertionError("Arrow-up key should sit above the down-arrow key")

    usb_position = ctx.part_world_position("usb_c_port")
    assert_close(usb_position[0], 0.0, 1e-6, "USB-C x centering")
    assert_close(usb_position[1], CASE_DEPTH / 2.0 - 0.0022, 1e-6, "USB-C rear placement")
    assert_close(usb_position[2], 0.0128, 1e-6, "USB-C vertical placement")

    with ctx.pose({"press_key_a": KEY_TRAVEL}):
        pressed_a = ctx.part_world_position("key_a")
        assert_close(pos_a[2] - pressed_a[2], KEY_TRAVEL, 1e-6, "A key travel")
    with ctx.pose({"press_key_space": KEY_TRAVEL}):
        pressed_space = ctx.part_world_position("key_space")
        assert_close(pos_space[2] - pressed_space[2], KEY_TRAVEL, 1e-6, "Spacebar travel")
    with ctx.pose({"press_key_enter": KEY_TRAVEL}):
        pressed_enter = ctx.part_world_position("key_enter")
        assert_close(pos_enter[2] - pressed_enter[2], KEY_TRAVEL, 1e-6, "Enter key travel")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
