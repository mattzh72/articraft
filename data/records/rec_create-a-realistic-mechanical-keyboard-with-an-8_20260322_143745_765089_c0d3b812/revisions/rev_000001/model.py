from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

PITCH = 0.01905
TOTAL_WIDTH_UNITS = 19.25
MAIN_START_UNITS = 0.50
NAV_START_UNITS = 15.75

CASE_WIDTH = TOTAL_WIDTH_UNITS * PITCH
CASE_DEPTH = 0.146
CASE_HEIGHT = 0.028
PLATE_THICKNESS = 0.0012
PLATE_TOP_Z = CASE_HEIGHT
KEY_ORIGIN_Z = CASE_HEIGHT + 0.0016
KEY_TRAVEL = 0.0035
SWITCH_HOUSING_SIZE = 0.014
SWITCH_HOUSING_HEIGHT = 0.0044
GUIDE_PIN_SIZE = 0.0018
GUIDE_PIN_LENGTH = 0.010

ROW_Y = {
    "bottom": -2.0 * PITCH,
    "z": -1.0 * PITCH,
    "a": 0.0,
    "q": 1.0 * PITCH,
    "num": 2.0 * PITCH,
    "func": 3.25 * PITCH,
    "arrow_up": -1.0 * PITCH,
}

ROW_DEPTH_UNITS = {
    "func": 0.90,
    "num": 1.00,
    "q": 1.00,
    "a": 1.00,
    "z": 1.00,
    "bottom": 1.00,
}

ROW_PROFILE = {
    "func": {"height": 0.0076, "angle_deg": -7.0},
    "num": {"height": 0.0089, "angle_deg": -6.0},
    "q": {"height": 0.0094, "angle_deg": -4.0},
    "a": {"height": 0.0095, "angle_deg": -1.5},
    "z": {"height": 0.0091, "angle_deg": 2.0},
    "bottom": {"height": 0.0082, "angle_deg": 5.0},
}

_KEYCAP_CACHE: dict[tuple[str, float, float], object] = {}
_CASE_MESH = None


def _x_from_units(center_units: float) -> float:
    return (center_units - (TOTAL_WIDTH_UNITS / 2.0)) * PITCH


def _loop(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _case_mesh():
    global _CASE_MESH
    if _CASE_MESH is not None:
        return _CASE_MESH

    case_geom = section_loft(
        [
            _loop(CASE_WIDTH, CASE_DEPTH, 0.014, 0.0),
            _loop(CASE_WIDTH - 0.002, CASE_DEPTH - 0.003, 0.0135, 0.006),
            _loop(CASE_WIDTH - 0.008, CASE_DEPTH - 0.008, 0.0115, 0.020),
            _loop(CASE_WIDTH - 0.014, CASE_DEPTH - 0.012, 0.0105, CASE_HEIGHT),
        ]
    )
    _CASE_MESH = mesh_from_geometry(case_geom, ASSETS.mesh_dir / "tkl_case.obj")
    return _CASE_MESH


def _keycap_mesh(row_name: str, width_units: float, depth_units: float):
    key = (row_name, width_units, depth_units)
    if key in _KEYCAP_CACHE:
        return _KEYCAP_CACHE[key]

    height = ROW_PROFILE[row_name]["height"]
    base_w = (width_units * PITCH) - 0.0016
    base_d = (depth_units * PITCH) - 0.0016
    mid_w = max(base_w - 0.0016, base_w * 0.86)
    mid_d = max(base_d - 0.0014, base_d * 0.86)
    top_w = max(base_w - 0.0036, base_w * 0.74)
    top_d = max(base_d - 0.0032, base_d * 0.74)

    keycap_geom = section_loft(
        [
            _loop(base_w, base_d, min(base_w, base_d) * 0.16, 0.0),
            _loop(mid_w, mid_d, min(mid_w, mid_d) * 0.18, height * 0.48),
            _loop(top_w, top_d, min(top_w, top_d) * 0.22, height),
        ]
    )
    width_tag = int(round(width_units * 100))
    depth_tag = int(round(depth_units * 100))
    mesh = mesh_from_geometry(
        keycap_geom,
        ASSETS.mesh_dir / f"keycap_{row_name}_{width_tag}_{depth_tag}.obj",
    )
    _KEYCAP_CACHE[key] = mesh
    return mesh


def _append_row(
    layout: list[dict[str, float | str]],
    row_name: str,
    y_value: float,
    start_units: float,
    keys: list[tuple[str, float]],
) -> None:
    cursor = start_units
    for name, width_units in keys:
        center_units = cursor + (width_units / 2.0)
        layout.append(
            {
                "name": name,
                "joint_name": f"press_{name}",
                "row": row_name,
                "width_units": width_units,
                "depth_units": ROW_DEPTH_UNITS[row_name],
                "x": _x_from_units(center_units),
                "y": y_value,
            }
        )
        cursor += width_units


def _build_key_layout() -> list[dict[str, float | str]]:
    layout: list[dict[str, float | str]] = []

    func_cursor = MAIN_START_UNITS
    for entry in [
        ("key_esc", 1.0),
        ("__gap__", 0.5),
        ("key_f1", 1.0),
        ("key_f2", 1.0),
        ("key_f3", 1.0),
        ("key_f4", 1.0),
        ("__gap__", 0.5),
        ("key_f5", 1.0),
        ("key_f6", 1.0),
        ("key_f7", 1.0),
        ("key_f8", 1.0),
        ("__gap__", 0.5),
        ("key_f9", 1.0),
        ("key_f10", 1.0),
        ("key_f11", 1.0),
        ("key_f12", 1.0),
        ("__gap__", 0.75),
        ("key_print_screen", 1.0),
        ("key_scroll_lock", 1.0),
        ("key_pause", 1.0),
    ]:
        name, width_units = entry
        if name == "__gap__":
            func_cursor += width_units
            continue
        center_units = func_cursor + (width_units / 2.0)
        layout.append(
            {
                "name": name,
                "joint_name": f"press_{name}",
                "row": "func",
                "width_units": width_units,
                "depth_units": ROW_DEPTH_UNITS["func"],
                "x": _x_from_units(center_units),
                "y": ROW_Y["func"],
            }
        )
        func_cursor += width_units

    _append_row(
        layout,
        "num",
        ROW_Y["num"],
        MAIN_START_UNITS,
        [
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
            ("key_equal", 1.0),
            ("key_backspace", 2.0),
        ],
    )
    _append_row(
        layout,
        "q",
        ROW_Y["q"],
        MAIN_START_UNITS,
        [
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
            ("key_lbracket", 1.0),
            ("key_rbracket", 1.0),
            ("key_backslash", 1.5),
        ],
    )
    _append_row(
        layout,
        "a",
        ROW_Y["a"],
        MAIN_START_UNITS,
        [
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
            ("key_quote", 1.0),
            ("key_enter", 2.25),
        ],
    )
    _append_row(
        layout,
        "z",
        ROW_Y["z"],
        MAIN_START_UNITS,
        [
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
        layout,
        "bottom",
        ROW_Y["bottom"],
        MAIN_START_UNITS,
        [
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

    for row_name, y_value, names in [
        ("num", ROW_Y["num"], ["key_insert", "key_home", "key_page_up"]),
        ("q", ROW_Y["q"], ["key_delete", "key_end", "key_page_down"]),
    ]:
        for index, name in enumerate(names):
            center_units = NAV_START_UNITS + index + 0.5
            layout.append(
                {
                    "name": name,
                    "joint_name": f"press_{name}",
                    "row": row_name,
                    "width_units": 1.0,
                    "depth_units": 1.0,
                    "x": _x_from_units(center_units),
                    "y": y_value,
                }
            )

    layout.append(
        {
            "name": "key_up",
            "joint_name": "press_key_up",
            "row": "z",
            "width_units": 1.0,
            "depth_units": 1.0,
            "x": _x_from_units(NAV_START_UNITS + 1.5),
            "y": ROW_Y["arrow_up"],
        }
    )
    for index, name in enumerate(["key_left", "key_down", "key_right"]):
        center_units = NAV_START_UNITS + index + 0.5
        layout.append(
            {
                "name": name,
                "joint_name": f"press_{name}",
                "row": "bottom",
                "width_units": 1.0,
                "depth_units": 1.0,
                "x": _x_from_units(center_units),
                "y": ROW_Y["bottom"],
            }
        )

    return layout


KEY_LAYOUT = _build_key_layout()
KEY_BY_NAME = {str(spec["name"]): spec for spec in KEY_LAYOUT}


def _stabilizer_offset(width_units: float) -> float:
    if width_units >= 6.0:
        return 0.050
    if width_units >= 2.5:
        return 0.022
    if width_units >= 2.0:
        return 0.017
    return 0.0


def _add_key(
    model: ArticulatedObject,
    body_name: str,
    spec: dict[str, float | str],
    keycap_material,
    stem_material,
) -> None:
    name = str(spec["name"])
    row_name = str(spec["row"])
    width_units = float(spec["width_units"])
    depth_units = float(spec["depth_units"])
    x_pos = float(spec["x"])
    y_pos = float(spec["y"])

    part = model.part(name)
    cap_height = ROW_PROFILE[row_name]["height"]
    cap_angle = radians(ROW_PROFILE[row_name]["angle_deg"])
    cap_mesh = _keycap_mesh(row_name, width_units, depth_units)

    part.visual(
        cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0048), rpy=(cap_angle, 0.0, 0.0)),
        material=keycap_material,
    )
    part.visual(
        Box((0.0042, 0.0042, 0.0032)),
        origin=Origin(xyz=(0.0, 0.0, 0.0018)),
        material=stem_material,
    )
    part.visual(
        Box((GUIDE_PIN_SIZE, GUIDE_PIN_SIZE, GUIDE_PIN_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, -0.0049)),
        material=stem_material,
    )

    stab_offset = _stabilizer_offset(width_units)
    if stab_offset > 0.0:
        for direction in (-1.0, 1.0):
            part.visual(
                Box((0.0016, 0.0016, 0.0060)),
                origin=Origin(xyz=(direction * stab_offset, 0.0, -0.0014)),
                material=stem_material,
            )

    inertial_height = cap_height + 0.012
    part.inertial = Inertial.from_geometry(
        Box(
            (
                max((width_units * PITCH) - 0.0025, 0.010),
                max((depth_units * PITCH) - 0.0025, 0.010),
                inertial_height,
            )
        ),
        mass=0.004 + (0.0016 * width_units),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
    )

    model.articulation(
        str(spec["joint_name"]),
        ArticulationType.PRISMATIC,
        parent=body_name,
        child=name,
        origin=Origin(xyz=(x_pos, y_pos, KEY_ORIGIN_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.20,
            lower=0.0,
            upper=KEY_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_keyboard_tkl", assets=ASSETS)

    aluminum = model.material("aluminum", rgba=(0.72, 0.73, 0.76, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    switch_finish = model.material("switch_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    keycap_red = model.material("keycap_red", rgba=(0.74, 0.11, 0.15, 1.0))
    stem_black = model.material("stem_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.09, 1.0))

    body = model.part("body")
    body.visual(_case_mesh(), material=aluminum)
    body.visual(
        Box((CASE_WIDTH - 0.026, CASE_DEPTH - 0.032, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_TOP_Z - (PLATE_THICKNESS / 2.0))),
        material=plate_finish,
    )

    switch_center_z = CASE_HEIGHT + (SWITCH_HOUSING_HEIGHT / 2.0) - 0.0010
    for spec in KEY_LAYOUT:
        x_pos = float(spec["x"])
        y_pos = float(spec["y"])
        width_units = float(spec["width_units"])

        body.visual(
            Box((SWITCH_HOUSING_SIZE, SWITCH_HOUSING_SIZE, SWITCH_HOUSING_HEIGHT)),
            origin=Origin(xyz=(x_pos, y_pos, switch_center_z)),
            material=switch_finish,
        )

        stab_offset = _stabilizer_offset(width_units)
        if stab_offset > 0.0:
            for direction in (-1.0, 1.0):
                body.visual(
                    Box((0.0030, 0.0030, SWITCH_HOUSING_HEIGHT)),
                    origin=Origin(xyz=(x_pos + (direction * stab_offset), y_pos, switch_center_z)),
                    material=switch_finish,
                )

    for x_pos in (-CASE_WIDTH * 0.33, CASE_WIDTH * 0.33):
        for y_pos in (-CASE_DEPTH * 0.30, CASE_DEPTH * 0.30):
            body.visual(
                Box((0.020, 0.010, 0.0018)),
                origin=Origin(xyz=(x_pos, y_pos, 0.0009)),
                material=rubber,
            )

    body.visual(
        Box((0.010, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, (CASE_DEPTH / 2.0) - 0.0025, 0.015)),
        material=plate_finish,
    )
    body.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_HEIGHT)),
        mass=1.45,
        origin=Origin(xyz=(0.0, 0.0, CASE_HEIGHT / 2.0)),
    )

    for spec in KEY_LAYOUT:
        _add_key(model, "body", spec, keycap_red, stem_black)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    for spec in KEY_LAYOUT:
        ctx.allow_overlap(
            "body",
            str(spec["name"]),
            reason="Key switch stem and guide post intentionally nest into the keyboard switch cavity.",
        )

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected(use="visual")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=64, overlap_tol=0.002, overlap_volume_tol=0.0)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(
        max_pose_samples=64,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    assert len(KEY_LAYOUT) == 87, f"Expected a standard TKL key count, found {len(KEY_LAYOUT)}."
    assert len(object_model.articulations) == 87, "Every TKL key should have its own press articulation."
    assert sum(1 for spec in KEY_LAYOUT if spec["row"] == "func") == 16, "TKL function row should include Esc, F-row, and print cluster."
    assert sum(1 for spec in KEY_LAYOUT if spec["row"] == "num") == 17, "TKL top alphanumeric row should include the three-key navigation cluster."
    assert sum(1 for spec in KEY_LAYOUT if spec["row"] == "q") == 17, "TKL second row should include delete/end/page-down cluster."
    assert sum(1 for spec in KEY_LAYOUT if spec["row"] == "a") == 13, "Home row should preserve the standard ANSI TKL count."
    assert sum(1 for spec in KEY_LAYOUT if spec["row"] == "z") == 13, "Lower alpha row should include the raised arrow key."
    assert sum(1 for spec in KEY_LAYOUT if spec["row"] == "bottom") == 11, "Bottom row should include the three-key arrow base."

    for spec in KEY_LAYOUT:
        pos = ctx.part_world_position(str(spec["name"]))
        assert abs(pos[0] - float(spec["x"])) < 5e-4, f"{spec['name']} x-position drifted from the TKL grid."
        assert abs(pos[1] - float(spec["y"])) < 5e-4, f"{spec['name']} y-position drifted from the TKL grid."
        assert abs(pos[2] - KEY_ORIGIN_Z) < 5e-4, f"{spec['name']} should rest at the switch top height."

    assert abs(ctx.part_world_position("key_s")[0] - ctx.part_world_position("key_a")[0] - PITCH) < 7e-4
    assert abs(ctx.part_world_position("key_q")[1] - ctx.part_world_position("key_a")[1] - PITCH) < 7e-4
    assert abs(ctx.part_world_position("key_right")[0] - ctx.part_world_position("key_down")[0] - PITCH) < 7e-4
    assert abs(ctx.part_world_position("key_up")[0] - ctx.part_world_position("key_down")[0]) < 7e-4
    assert abs(ctx.part_world_position("key_up")[1] - ctx.part_world_position("key_down")[1] - PITCH) < 7e-4
    assert abs(ctx.part_world_position("key_right_alt")[0] - ctx.part_world_position("key_left_alt")[0] - (7.5 * PITCH)) < 7e-4
    assert ctx.part_world_position("key_print_screen")[0] > ctx.part_world_position("key_f12")[0] + (0.70 * PITCH)
    assert ctx.part_world_position("key_insert")[0] > ctx.part_world_position("key_backspace")[0] + (0.55 * PITCH)
    assert ctx.part_world_position("key_left")[0] > ctx.part_world_position("key_space")[0] + (3.6 * PITCH)

    ctx.expect_joint_motion_axis("press_key_a", "key_a", world_axis="z", direction="negative", min_delta=0.002)
    ctx.expect_joint_motion_axis("press_key_enter", "key_enter", world_axis="z", direction="negative", min_delta=0.002)
    ctx.expect_joint_motion_axis("press_key_space", "key_space", world_axis="z", direction="negative", min_delta=0.002)
    ctx.expect_joint_motion_axis("press_key_up", "key_up", world_axis="z", direction="negative", min_delta=0.002)

    ctx.expect_aabb_overlap("key_esc", "body", axes="xy", min_overlap=0.014)
    ctx.expect_aabb_overlap("key_pause", "body", axes="xy", min_overlap=0.014)
    ctx.expect_aabb_overlap("key_space", "body", axes="xy", min_overlap=0.017)

    with ctx.pose(press_key_space=KEY_TRAVEL, press_key_enter=KEY_TRAVEL, press_key_up=KEY_TRAVEL):
        assert abs(ctx.part_world_position("key_space")[2] - (KEY_ORIGIN_Z - KEY_TRAVEL)) < 6e-4
        assert abs(ctx.part_world_position("key_enter")[2] - (KEY_ORIGIN_Z - KEY_TRAVEL)) < 6e-4
        assert abs(ctx.part_world_position("key_up")[2] - (KEY_ORIGIN_Z - KEY_TRAVEL)) < 6e-4

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
