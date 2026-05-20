from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    superellipse_side_loft,
)

ASSETS = AssetContext.from_script(__file__)

U = 0.01905
SWITCH_TRAVEL = 0.004
LAYOUT_WIDTH_U = 18.25
LAYOUT_CENTER_X_U = LAYOUT_WIDTH_U / 2.0
LAYOUT_CENTER_Y_U = 3.25
LAYOUT_WIDTH = LAYOUT_WIDTH_U * U
LAYOUT_DEPTH = 6.5 * U
BODY_WIDTH = LAYOUT_WIDTH + 0.014
BODY_DEPTH = LAYOUT_DEPTH + 0.019
FRONT_HEIGHT = 0.015
MID_HEIGHT = 0.0215
REAR_HEIGHT = 0.0265
DECK_Z = 0.014
KEYCAP_HEIGHT = 0.009


def _row_entries(*entries: tuple[str, float, float]) -> list[tuple[str, float, float]]:
    return list(entries)


def _append_row(
    specs: list[dict[str, float | str]],
    *,
    y_u: float,
    start_x_u: float,
    entries: list[tuple[str, float, float]],
) -> None:
    cursor = start_x_u
    for name, width_u, gap_after_u in entries:
        center_x_u = cursor + (width_u * 0.5)
        specs.append(
            {
                "name": name,
                "part_name": f"key_{name}",
                "joint_name": f"switch_{name}",
                "width_u": width_u,
                "x": (center_x_u - LAYOUT_CENTER_X_U) * U,
                "y": (y_u - LAYOUT_CENTER_Y_U) * U,
            }
        )
        cursor += width_u + gap_after_u


def _build_key_specs() -> tuple[dict[str, float | str], ...]:
    specs: list[dict[str, float | str]] = []

    _append_row(
        specs,
        y_u=6.0,
        start_x_u=0.0,
        entries=_row_entries(
            ("esc", 1.0, 0.5),
            ("f1", 1.0, 0.0),
            ("f2", 1.0, 0.0),
            ("f3", 1.0, 0.0),
            ("f4", 1.0, 0.25),
            ("f5", 1.0, 0.0),
            ("f6", 1.0, 0.0),
            ("f7", 1.0, 0.0),
            ("f8", 1.0, 0.25),
            ("f9", 1.0, 0.0),
            ("f10", 1.0, 0.0),
            ("f11", 1.0, 0.0),
            ("f12", 1.0, 0.25),
            ("print_screen", 1.0, 0.0),
            ("scroll_lock", 1.0, 0.0),
            ("pause", 1.0, 0.0),
        ),
    )
    _append_row(
        specs,
        y_u=4.5,
        start_x_u=0.0,
        entries=_row_entries(
            ("grave", 1.0, 0.0),
            ("digit_1", 1.0, 0.0),
            ("digit_2", 1.0, 0.0),
            ("digit_3", 1.0, 0.0),
            ("digit_4", 1.0, 0.0),
            ("digit_5", 1.0, 0.0),
            ("digit_6", 1.0, 0.0),
            ("digit_7", 1.0, 0.0),
            ("digit_8", 1.0, 0.0),
            ("digit_9", 1.0, 0.0),
            ("digit_0", 1.0, 0.0),
            ("minus", 1.0, 0.0),
            ("equals", 1.0, 0.0),
            ("backspace", 2.0, 0.0),
        ),
    )
    _append_row(
        specs,
        y_u=3.5,
        start_x_u=0.0,
        entries=_row_entries(
            ("tab", 1.5, 0.0),
            ("q", 1.0, 0.0),
            ("w", 1.0, 0.0),
            ("e", 1.0, 0.0),
            ("r", 1.0, 0.0),
            ("t", 1.0, 0.0),
            ("y", 1.0, 0.0),
            ("u", 1.0, 0.0),
            ("i", 1.0, 0.0),
            ("o", 1.0, 0.0),
            ("p", 1.0, 0.0),
            ("left_bracket", 1.0, 0.0),
            ("right_bracket", 1.0, 0.0),
            ("backslash", 1.5, 0.0),
        ),
    )
    _append_row(
        specs,
        y_u=2.5,
        start_x_u=0.0,
        entries=_row_entries(
            ("caps_lock", 1.75, 0.0),
            ("a", 1.0, 0.0),
            ("s", 1.0, 0.0),
            ("d", 1.0, 0.0),
            ("f", 1.0, 0.0),
            ("g", 1.0, 0.0),
            ("h", 1.0, 0.0),
            ("j", 1.0, 0.0),
            ("k", 1.0, 0.0),
            ("l", 1.0, 0.0),
            ("semicolon", 1.0, 0.0),
            ("apostrophe", 1.0, 0.0),
            ("enter", 2.25, 0.0),
        ),
    )
    _append_row(
        specs,
        y_u=1.5,
        start_x_u=0.0,
        entries=_row_entries(
            ("left_shift", 2.25, 0.0),
            ("z", 1.0, 0.0),
            ("x", 1.0, 0.0),
            ("c", 1.0, 0.0),
            ("v", 1.0, 0.0),
            ("b", 1.0, 0.0),
            ("n", 1.0, 0.0),
            ("m", 1.0, 0.0),
            ("comma", 1.0, 0.0),
            ("period", 1.0, 0.0),
            ("slash", 1.0, 0.0),
            ("right_shift", 2.75, 0.0),
        ),
    )
    _append_row(
        specs,
        y_u=0.5,
        start_x_u=0.0,
        entries=_row_entries(
            ("left_ctrl", 1.25, 0.0),
            ("left_win", 1.25, 0.0),
            ("left_alt", 1.25, 0.0),
            ("space", 6.25, 0.0),
            ("right_alt", 1.25, 0.0),
            ("fn", 1.25, 0.0),
            ("menu", 1.25, 0.0),
            ("right_ctrl", 1.25, 0.0),
        ),
    )
    _append_row(
        specs,
        y_u=4.5,
        start_x_u=15.25,
        entries=_row_entries(
            ("insert", 1.0, 0.0),
            ("home", 1.0, 0.0),
            ("page_up", 1.0, 0.0),
        ),
    )
    _append_row(
        specs,
        y_u=3.5,
        start_x_u=15.25,
        entries=_row_entries(
            ("delete", 1.0, 0.0),
            ("end", 1.0, 0.0),
            ("page_down", 1.0, 0.0),
        ),
    )
    _append_row(
        specs,
        y_u=1.5,
        start_x_u=16.25,
        entries=_row_entries(("up", 1.0, 0.0)),
    )
    _append_row(
        specs,
        y_u=0.5,
        start_x_u=15.25,
        entries=_row_entries(
            ("left", 1.0, 0.0),
            ("down", 1.0, 0.0),
            ("right", 1.0, 0.0),
        ),
    )
    return tuple(specs)


KEY_SPECS = _build_key_specs()
KEY_SPEC_BY_PART = {str(spec["part_name"]): spec for spec in KEY_SPECS}
ACCENT_KEYS = {"key_esc", "key_enter"}
MODIFIER_KEYS = {
    "key_backspace",
    "key_tab",
    "key_caps_lock",
    "key_left_shift",
    "key_right_shift",
    "key_left_ctrl",
    "key_right_ctrl",
    "key_left_alt",
    "key_right_alt",
    "key_left_win",
    "key_fn",
    "key_menu",
    "key_insert",
    "key_delete",
    "key_home",
    "key_end",
    "key_page_up",
    "key_page_down",
    "key_print_screen",
    "key_scroll_lock",
    "key_pause",
    "key_up",
    "key_left",
    "key_down",
    "key_right",
    "key_f1",
    "key_f2",
    "key_f3",
    "key_f4",
    "key_f5",
    "key_f6",
    "key_f7",
    "key_f8",
    "key_f9",
    "key_f10",
    "key_f11",
    "key_f12",
}

_BODY_MESH = None
_KEYCAP_MESHES: dict[float, object] = {}


def _rounded_section(width: float, depth: float, z: float, shift_y: float = 0.0) -> list[tuple[float, float, float]]:
    radius = min(width, depth) * 0.14
    return [(x, y + shift_y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _body_shell_mesh():
    global _BODY_MESH
    if _BODY_MESH is None:
        shell = superellipse_side_loft(
            sections=[
                (-BODY_DEPTH * 0.5, 0.0, FRONT_HEIGHT, BODY_WIDTH - 0.010),
                (-BODY_DEPTH * 0.20, 0.0, FRONT_HEIGHT + 0.0025, BODY_WIDTH - 0.004),
                (0.0, 0.0, MID_HEIGHT, BODY_WIDTH),
                (BODY_DEPTH * 0.22, 0.0, MID_HEIGHT + 0.002, BODY_WIDTH),
                (BODY_DEPTH * 0.5, 0.0, REAR_HEIGHT, BODY_WIDTH - 0.006),
            ],
            exponents=(4.0, 3.4, 3.0, 2.9, 2.7),
            segments=72,
            cap=True,
            closed=True,
        )
        _BODY_MESH = mesh_from_geometry(shell, ASSETS.mesh_path("keyboard_tkl_shell.obj"))
    return _BODY_MESH


def _keycap_mesh(width_u: float):
    if width_u not in _KEYCAP_MESHES:
        base_width = max((width_u * U) - 0.0014, 0.0135)
        base_depth = U - 0.0016
        top_width = base_width - min(0.0032, 0.0018 + (width_u * 0.00018))
        top_depth = base_depth - 0.0026
        geometry = section_loft(
            [
                _rounded_section(base_width, base_depth, 0.0, 0.0),
                _rounded_section(top_width, top_depth, KEYCAP_HEIGHT, 0.0004),
            ]
        )
        width_tag = f"{width_u:.2f}".replace(".", "_")
        _KEYCAP_MESHES[width_u] = mesh_from_geometry(
            geometry,
            ASSETS.mesh_path(f"keyboard_keycap_{width_tag}u.obj"),
        )
    return _KEYCAP_MESHES[width_u]


def _stabilizer_offset(width_u: float) -> float:
    return max(0.0, min((width_u * U * 0.36), ((width_u * U) * 0.5) - 0.012))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_keyboard_tkl", assets=ASSETS)

    case_shell = model.material("case_shell", rgba=(0.18, 0.20, 0.23, 1.0))
    top_plate = model.material("top_plate", rgba=(0.11, 0.12, 0.14, 1.0))
    switch_housing = model.material("switch_housing", rgba=(0.28, 0.20, 0.12, 0.58))
    switch_stem = model.material("switch_stem", rgba=(0.68, 0.22, 0.18, 1.0))
    alpha_key = model.material("alpha_key", rgba=(0.15, 0.16, 0.18, 1.0))
    modifier_key = model.material("modifier_key", rgba=(0.27, 0.28, 0.31, 1.0))
    accent_key = model.material("accent_key", rgba=(0.35, 0.47, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    port_shell = model.material("port_shell", rgba=(0.74, 0.76, 0.80, 1.0))
    port_core = model.material("port_core", rgba=(0.07, 0.07, 0.08, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=case_shell, name="shell")
    body.visual(
        Box((BODY_WIDTH - 0.008, BODY_DEPTH - 0.010, 0.0045)),
        origin=Origin(xyz=(0.0, 0.0, DECK_Z - 0.00225)),
        material=top_plate,
        name="top_plate",
    )
    body.visual(
        Box((BODY_WIDTH - 0.060, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.29, REAR_HEIGHT - 0.0025)),
        material=top_plate,
        name="rear_accent",
    )

    foot_positions = (
        (-BODY_WIDTH * 0.36, -BODY_DEPTH * 0.32, 0.0014),
        (BODY_WIDTH * 0.36, -BODY_DEPTH * 0.32, 0.0014),
        (-BODY_WIDTH * 0.34, BODY_DEPTH * 0.31, 0.0022),
        (BODY_WIDTH * 0.34, BODY_DEPTH * 0.31, 0.0022),
    )
    for index, (x, y, foot_h) in enumerate(foot_positions, start=1):
        body.visual(
            Box((0.018, 0.008, foot_h)),
            origin=Origin(xyz=(x, y, foot_h * 0.5)),
            material=rubber,
            name=f"foot_{index}",
        )

    for spec in KEY_SPECS:
        x = float(spec["x"])
        y = float(spec["y"])
        width_u = float(spec["width_u"])

        body.visual(
            Box((0.014, 0.014, 0.006)),
            origin=Origin(xyz=(x, y, DECK_Z + 0.0024)),
            material=switch_housing,
            name=f"housing_{spec['name']}",
        )
        if width_u >= 2.25:
            stab_offset = _stabilizer_offset(width_u)
            for side, sign in (("left", -1.0), ("right", 1.0)):
                body.visual(
                    Box((0.007, 0.014, 0.0042)),
                    origin=Origin(
                        xyz=(x + (stab_offset * sign), y, DECK_Z + 0.0018),
                    ),
                    material=top_plate,
                    name=f"stabilizer_mount_{spec['name']}_{side}",
                )

    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, REAR_HEIGHT)),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.0, REAR_HEIGHT * 0.42)),
    )

    usb_c = model.part("usb_c_port")
    usb_c.visual(
        Box((0.0105, 0.0042, 0.0034)),
        origin=Origin(xyz=(0.0, -0.0021, 0.0)),
        material=port_shell,
        name="shell",
    )
    usb_c.visual(
        Box((0.0068, 0.0018, 0.0013)),
        origin=Origin(xyz=(0.0, -0.0010, 0.0)),
        material=port_core,
        name="inner_tongue",
    )
    usb_c.inertial = Inertial.from_geometry(
        Box((0.0105, 0.0042, 0.0034)),
        mass=0.01,
        origin=Origin(xyz=(0.0, -0.0021, 0.0)),
    )
    model.articulation(
        "body_to_usb_c",
        ArticulationType.FIXED,
        parent="body",
        child="usb_c_port",
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5, 0.0162)),
    )

    for spec in KEY_SPECS:
        part_name = str(spec["part_name"])
        joint_name = str(spec["joint_name"])
        width_u = float(spec["width_u"])

        key = model.part(part_name)
        key.visual(
            Box((0.005, 0.005, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=switch_stem,
            name="stem",
        )
        if width_u >= 2.25:
            stab_offset = _stabilizer_offset(width_u)
            for side, sign in (("left", -1.0), ("right", 1.0)):
                key.visual(
                    Box((0.0036, 0.0036, 0.0056)),
                    origin=Origin(xyz=(stab_offset * sign, 0.0, 0.0028)),
                    material=switch_stem,
                    name=f"stabilizer_post_{side}",
                )

        key_material = alpha_key
        if part_name in MODIFIER_KEYS:
            key_material = modifier_key
        if part_name in ACCENT_KEYS:
            key_material = accent_key

        key.visual(
            _keycap_mesh(width_u),
            origin=Origin(xyz=(0.0, 0.0, 0.0097)),
            material=key_material,
            name="keycap",
        )
        key.inertial = Inertial.from_geometry(
            Box((max((width_u * U) - 0.0018, 0.014), 0.0172, 0.011)),
            mass=0.008 + (0.0032 * width_u),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
        )

        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent="body",
            child=part_name,
            origin=Origin(xyz=(float(spec["x"]), float(spec["y"]), DECK_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.45,
                velocity=0.08,
                lower=0.0,
                upper=SWITCH_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, geometry_source="collision")

    for spec in KEY_SPECS:
        ctx.allow_overlap(
            "body",
            str(spec["part_name"]),
            reason="Switch stems travel into coarse solid-case deck geometry.",
        )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected(use="visual")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=32, overlap_tol=0.0015, overlap_volume_tol=0.0)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(
        max_pose_samples=32,
        overlap_tol=0.0015,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    # Add narrow allowances here when conservative QC reports acceptable cases.
    # Add prompt-specific expect_* semantic checks below; they are the main regressions.
    ctx.expect_aabb_overlap("key_space", "body", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_overlap("key_esc", "body", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_overlap("key_enter", "body", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_overlap("usb_c_port", "body", axes="xz", min_overlap=0.003)
    ctx.expect_aabb_gap("usb_c_port", "body", axis="y", max_gap=0.001, max_penetration=0.006)

    for spec in KEY_SPECS:
        ctx.expect_joint_motion_axis(
            str(spec["joint_name"]),
            str(spec["part_name"]),
            world_axis="z",
            direction="negative",
            min_delta=0.003,
        )

    def _position(name: str) -> tuple[float, float, float]:
        pos = ctx.part_world_position(name)
        return (float(pos[0]), float(pos[1]), float(pos[2]))

    def _expect_close(actual: float, expected: float, tol: float, label: str) -> None:
        if abs(actual - expected) > tol:
            raise AssertionError(f"{label}: expected {expected:.6f}, got {actual:.6f}")

    key_parts = [part.name for part in object_model.parts if part.name.startswith("key_")]
    switch_joints = [joint.name for joint in object_model.articulations if joint.name.startswith("switch_")]
    if len(key_parts) != len(KEY_SPECS):
        raise AssertionError(f"Expected {len(KEY_SPECS)} key parts, found {len(key_parts)}")
    if len(switch_joints) != len(KEY_SPECS):
        raise AssertionError(f"Expected {len(KEY_SPECS)} switch joints, found {len(switch_joints)}")

    key_a = _position("key_a")
    key_s = _position("key_s")
    key_q = _position("key_q")
    key_z = _position("key_z")
    key_backspace = _position("key_backspace")
    key_enter = _position("key_enter")
    key_insert = _position("key_insert")
    key_up = _position("key_up")
    key_down = _position("key_down")
    key_left = _position("key_left")
    key_right = _position("key_right")
    key_esc = _position("key_esc")
    key_f1 = _position("key_f1")
    key_space = _position("key_space")
    usb_c = _position("usb_c_port")

    _expect_close(key_s[0] - key_a[0], U, 0.0005, "A-to-S pitch")
    _expect_close(key_q[1] - key_a[1], U, 0.0005, "Q-row vertical pitch above home row")
    _expect_close(key_a[0] - key_q[0], 0.25 * U, 0.0005, "ANSI stagger from Q to A")
    _expect_close(key_z[0] - key_a[0], 0.5 * U, 0.0005, "ANSI stagger from A to Z")
    _expect_close(
        key_insert[0] - key_backspace[0],
        1.75 * U,
        0.0006,
        "Gap from backspace to insert cluster",
    )
    _expect_close(key_up[0] - key_down[0], 0.0, 0.0004, "Up arrow centered over down arrow")
    _expect_close(key_right[0] - key_left[0], 2.0 * U, 0.0006, "Arrow cluster width")
    _expect_close(key_up[1] - key_down[1], U, 0.0005, "Arrow cluster vertical pitch")
    _expect_close(key_f1[0] - key_esc[0], 1.5 * U, 0.0006, "Esc-to-F1 group spacing")
    _expect_close(
        key_space[0],
        -2.25 * U,
        0.0006,
        "Spacebar offset matches standard TKL bottom-row placement",
    )
    _expect_close(usb_c[0], 0.0, 0.0012, "USB-C port centered on rear edge")

    with ctx.pose(switch_space=SWITCH_TRAVEL):
        pressed_space = _position("key_space")
        pressed_a = _position("key_a")
        _expect_close(
            key_space[2] - pressed_space[2],
            SWITCH_TRAVEL,
            0.0004,
            "Spacebar press travel",
        )
        _expect_close(
            pressed_a[2],
            key_a[2],
            0.0003,
            "Neighboring alpha key stays at rest when spacebar is pressed",
        )

    with ctx.pose(switch_enter=SWITCH_TRAVEL):
        pressed_enter = _position("key_enter")
        _expect_close(
            key_enter[2] - pressed_enter[2],
            SWITCH_TRAVEL,
            0.0004,
            "Enter key press travel",
        )
        _expect_close(
            _position("key_enter")[2],
            pressed_enter[2],
            0.0001,
            "Enter key pressed pose resolves consistently",
        )
        _expect_close(
            key_esc[2] - _position("key_esc")[2],
            0.0,
            0.0003,
            "Unrelated function key stays at rest when Enter is pressed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
