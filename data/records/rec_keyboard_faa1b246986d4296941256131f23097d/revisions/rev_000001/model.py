from __future__ import annotations

from dataclasses import dataclass

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


U = 0.01905  # One keyboard unit: 19.05 mm.
CAP_GAP = 0.00135
CAP_HEIGHT = 0.0105
CAP_BOTTOM = 0.0130
SWITCH_TOP_Z = 0.0210
KEY_TRAVEL = 0.0040


@dataclass(frozen=True)
class KeySpec:
    name: str
    x_u: float
    y_u: float
    width_u: float = 1.0
    home_dot: bool = False
    accent: bool = False


KEY_SPECS: tuple[KeySpec, ...] = (
    # Function row and system cluster.
    KeySpec("key_esc", 0.0, 0.0, accent=True),
    KeySpec("key_f1", 2.0, 0.0),
    KeySpec("key_f2", 3.0, 0.0),
    KeySpec("key_f3", 4.0, 0.0),
    KeySpec("key_f4", 5.0, 0.0),
    KeySpec("key_f5", 6.5, 0.0),
    KeySpec("key_f6", 7.5, 0.0),
    KeySpec("key_f7", 8.5, 0.0),
    KeySpec("key_f8", 9.5, 0.0),
    KeySpec("key_f9", 11.0, 0.0),
    KeySpec("key_f10", 12.0, 0.0),
    KeySpec("key_f11", 13.0, 0.0),
    KeySpec("key_f12", 14.0, 0.0),
    KeySpec("key_print", 15.5, 0.0),
    KeySpec("key_scroll", 16.5, 0.0),
    KeySpec("key_pause", 17.5, 0.0),
    # Number row plus the top navigation cluster.
    KeySpec("key_grave", 0.0, -1.5),
    KeySpec("key_1", 1.0, -1.5),
    KeySpec("key_2", 2.0, -1.5),
    KeySpec("key_3", 3.0, -1.5),
    KeySpec("key_4", 4.0, -1.5),
    KeySpec("key_5", 5.0, -1.5),
    KeySpec("key_6", 6.0, -1.5),
    KeySpec("key_7", 7.0, -1.5),
    KeySpec("key_8", 8.0, -1.5),
    KeySpec("key_9", 9.0, -1.5),
    KeySpec("key_0", 10.0, -1.5),
    KeySpec("key_minus", 11.0, -1.5),
    KeySpec("key_equal", 12.0, -1.5),
    KeySpec("key_backspace", 13.5, -1.5, 2.0),
    KeySpec("key_insert", 15.5, -1.5),
    KeySpec("key_home", 16.5, -1.5),
    KeySpec("key_page_up", 17.5, -1.5),
    # QWERTY row plus lower navigation cluster.
    KeySpec("key_tab", 0.25, -2.5, 1.5),
    KeySpec("key_q", 1.5, -2.5),
    KeySpec("key_w", 2.5, -2.5),
    KeySpec("key_e", 3.5, -2.5),
    KeySpec("key_r", 4.5, -2.5),
    KeySpec("key_t", 5.5, -2.5),
    KeySpec("key_y", 6.5, -2.5),
    KeySpec("key_u", 7.5, -2.5),
    KeySpec("key_i", 8.5, -2.5),
    KeySpec("key_o", 9.5, -2.5),
    KeySpec("key_p", 10.5, -2.5),
    KeySpec("key_left_bracket", 11.5, -2.5),
    KeySpec("key_right_bracket", 12.5, -2.5),
    KeySpec("key_backslash", 13.75, -2.5, 1.5),
    KeySpec("key_delete", 15.5, -2.5),
    KeySpec("key_end", 16.5, -2.5),
    KeySpec("key_page_down", 17.5, -2.5),
    # Home row.
    KeySpec("key_caps", 0.375, -3.5, 1.75),
    KeySpec("key_a", 1.75, -3.5),
    KeySpec("key_s", 2.75, -3.5),
    KeySpec("key_d", 3.75, -3.5),
    KeySpec("key_f", 4.75, -3.5, home_dot=True),
    KeySpec("key_g", 5.75, -3.5),
    KeySpec("key_h", 6.75, -3.5),
    KeySpec("key_j", 7.75, -3.5, home_dot=True),
    KeySpec("key_k", 8.75, -3.5),
    KeySpec("key_l", 9.75, -3.5),
    KeySpec("key_semicolon", 10.75, -3.5),
    KeySpec("key_quote", 11.75, -3.5),
    KeySpec("key_enter", 13.375, -3.5, 2.25),
    # Shift row and up arrow.
    KeySpec("key_left_shift", 0.625, -4.5, 2.25),
    KeySpec("key_z", 2.25, -4.5),
    KeySpec("key_x", 3.25, -4.5),
    KeySpec("key_c", 4.25, -4.5),
    KeySpec("key_v", 5.25, -4.5),
    KeySpec("key_b", 6.25, -4.5),
    KeySpec("key_n", 7.25, -4.5),
    KeySpec("key_m", 8.25, -4.5),
    KeySpec("key_comma", 9.25, -4.5),
    KeySpec("key_period", 10.25, -4.5),
    KeySpec("key_slash", 11.25, -4.5),
    KeySpec("key_right_shift", 13.125, -4.5, 2.75),
    KeySpec("key_up", 16.5, -4.5, accent=True),
    # Bottom row and inverted-T arrows.
    KeySpec("key_left_ctrl", 0.125, -5.5, 1.25),
    KeySpec("key_left_win", 1.375, -5.5, 1.25),
    KeySpec("key_left_alt", 2.625, -5.5, 1.25),
    KeySpec("key_space", 6.375, -5.5, 6.25),
    KeySpec("key_right_alt", 10.125, -5.5, 1.25),
    KeySpec("key_fn", 11.375, -5.5, 1.25),
    KeySpec("key_menu", 12.625, -5.5, 1.25),
    KeySpec("key_right_ctrl", 13.875, -5.5, 1.25),
    KeySpec("key_left", 15.5, -5.5, accent=True),
    KeySpec("key_down", 16.5, -5.5, accent=True),
    KeySpec("key_right", 17.5, -5.5, accent=True),
)


def _layout_bounds() -> tuple[float, float, float, float]:
    min_x = min(spec.x_u - spec.width_u / 2.0 for spec in KEY_SPECS)
    max_x = max(spec.x_u + spec.width_u / 2.0 for spec in KEY_SPECS)
    min_y = min(spec.y_u - 0.5 for spec in KEY_SPECS)
    max_y = max(spec.y_u + 0.5 for spec in KEY_SPECS)
    return min_x, max_x, min_y, max_y


LAYOUT_MIN_X_U, LAYOUT_MAX_X_U, LAYOUT_MIN_Y_U, LAYOUT_MAX_Y_U = _layout_bounds()
LAYOUT_CENTER_X_U = (LAYOUT_MIN_X_U + LAYOUT_MAX_X_U) / 2.0
LAYOUT_CENTER_Y_U = (LAYOUT_MIN_Y_U + LAYOUT_MAX_Y_U) / 2.0
LAYOUT_WIDTH = (LAYOUT_MAX_X_U - LAYOUT_MIN_X_U) * U
LAYOUT_DEPTH = (LAYOUT_MAX_Y_U - LAYOUT_MIN_Y_U) * U
CASE_MARGIN_X = 0.014
CASE_MARGIN_Y = 0.014
CASE_WIDTH = LAYOUT_WIDTH + 2.0 * CASE_MARGIN_X
CASE_DEPTH = LAYOUT_DEPTH + 2.0 * CASE_MARGIN_Y


def _key_xy(spec: KeySpec) -> tuple[float, float]:
    return ((spec.x_u - LAYOUT_CENTER_X_U) * U, (spec.y_u - LAYOUT_CENTER_Y_U) * U)


def _safe_visual_suffix(name: str) -> str:
    return name.removeprefix("key_")


def _rounded_loft_mesh(
    *,
    width: float,
    depth: float,
    height: float,
    radius: float,
    top_inset: float,
    name: str,
):
    lower = rounded_rect_profile(width, depth, radius, corner_segments=5)
    upper_width = max(width - top_inset * 2.0, width * 0.65)
    upper_depth = max(depth - top_inset * 2.0, depth * 0.65)
    upper_radius = max(radius - top_inset * 0.5, radius * 0.5)
    upper = rounded_rect_profile(upper_width, upper_depth, upper_radius, corner_segments=5)
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            [(x, y, height * 0.25) for x, y in lower],
            [(x, y, height) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _make_keycap_mesh(width_u: float, cache: dict[float, object]):
    key = round(width_u, 3)
    cached = cache.get(key)
    if cached is not None:
        return cached
    width = width_u * U - CAP_GAP
    depth = U - CAP_GAP
    mesh = _rounded_loft_mesh(
        width=width,
        depth=depth,
        height=CAP_HEIGHT,
        radius=min(0.0030, depth * 0.22, width * 0.22),
        top_inset=0.0022,
        name=f"keycap_{str(key).replace('.', '_')}u",
    )
    cache[key] = mesh
    return mesh


def _make_case_mesh():
    lower = rounded_rect_profile(CASE_WIDTH, CASE_DEPTH, 0.014, corner_segments=10)
    shoulder = rounded_rect_profile(CASE_WIDTH - 0.004, CASE_DEPTH - 0.004, 0.012, corner_segments=10)
    upper = rounded_rect_profile(CASE_WIDTH - 0.008, CASE_DEPTH - 0.008, 0.010, corner_segments=10)
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            [(x, y, 0.004) for x, y in lower],
            [(x, y, 0.0105) for x, y in shoulder],
            [(x, y, 0.0125) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, "rounded_aluminum_case")


def _add_usb_c_port(case, *, materials: dict[str, Material]) -> None:
    rear_y = CASE_DEPTH / 2.0 - 0.002
    port_z = 0.0073
    # A small black mouth with a bright metal sleeve and a central plastic tongue.
    case.visual(
        Box((0.0140, 0.0020, 0.0052)),
        origin=Origin(xyz=(0.0, rear_y, port_z)),
        material=materials["port_black"],
        name="usb_c_opening",
    )
    case.visual(
        Box((0.0170, 0.0017, 0.0011)),
        origin=Origin(xyz=(0.0, rear_y - 0.0001, port_z + 0.00315)),
        material=materials["brushed_steel"],
        name="usb_c_top_lip",
    )
    case.visual(
        Box((0.0170, 0.0017, 0.0011)),
        origin=Origin(xyz=(0.0, rear_y - 0.0001, port_z - 0.00315)),
        material=materials["brushed_steel"],
        name="usb_c_bottom_lip",
    )
    case.visual(
        Box((0.0011, 0.0017, 0.0064)),
        origin=Origin(xyz=(-0.0085, rear_y - 0.0001, port_z)),
        material=materials["brushed_steel"],
        name="usb_c_side_lip_0",
    )
    case.visual(
        Box((0.0011, 0.0017, 0.0064)),
        origin=Origin(xyz=(0.0085, rear_y - 0.0001, port_z)),
        material=materials["brushed_steel"],
        name="usb_c_side_lip_1",
    )
    case.visual(
        Box((0.0090, 0.0019, 0.0012)),
        origin=Origin(xyz=(0.0, rear_y + 0.0002, port_z)),
        material=materials["port_tongue"],
        name="usb_c_tongue",
    )


def _add_stabilizers(key_part, width_u: float, material: Material) -> None:
    if width_u < 1.75:
        return
    reach = min((width_u * U) * 0.34, 0.045)
    for index, x in enumerate((-reach, reach)):
        key_part.visual(
            Box((0.0040, 0.0100, 0.0030)),
            origin=Origin(xyz=(x, 0.0, CAP_BOTTOM - 0.0010)),
            material=material,
            name=f"stabilizer_{index}",
        )
    key_part.visual(
        Box((reach * 2.0 + 0.004, 0.0014, 0.0014)),
        origin=Origin(xyz=(0.0, -0.0046, CAP_BOTTOM - 0.0003)),
        material=material,
        name="stabilizer_wire",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_tkl_mechanical_keyboard")

    materials = {
        "case": model.material("anodized_graphite", rgba=(0.12, 0.13, 0.15, 1.0)),
        "plate": model.material("matte_black_plate", rgba=(0.025, 0.027, 0.030, 1.0)),
        "key": model.material("warm_grey_keycaps", rgba=(0.72, 0.73, 0.72, 1.0)),
        "dark_key": model.material("charcoal_keycaps", rgba=(0.11, 0.115, 0.12, 1.0)),
        "accent": model.material("deep_teal_accents", rgba=(0.05, 0.34, 0.38, 1.0)),
        "legend": model.material("soft_black_legends", rgba=(0.02, 0.02, 0.022, 1.0)),
        "legend_light": model.material("pale_legends", rgba=(0.90, 0.92, 0.90, 1.0)),
        "switch": model.material("smoky_switch_housing", rgba=(0.16, 0.16, 0.17, 1.0)),
        "stem": model.material("cherry_red_stem", rgba=(0.76, 0.03, 0.02, 1.0)),
        "brushed_steel": model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0)),
        "port_black": model.material("usb_shadow_black", rgba=(0.0, 0.0, 0.0, 1.0)),
        "port_tongue": model.material("usb_c_tongue_black", rgba=(0.045, 0.045, 0.050, 1.0)),
        "rubber": model.material("soft_rubber", rgba=(0.035, 0.035, 0.035, 1.0)),
    }

    case = model.part(
        "case",
        meta={
            "layout": "ANSI 80% tenkeyless",
            "key_count": len(KEY_SPECS),
            "unit_m": U,
            "usb_c": True,
        },
    )
    case.visual(_make_case_mesh(), material=materials["case"], name="rounded_case")
    case.visual(
        Box((LAYOUT_WIDTH + 0.017, LAYOUT_DEPTH + 0.010, 0.0020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
        material=materials["plate"],
        name="switch_plate",
    )
    # Slightly raised rear rail gives the modern wedge silhouette and hides the
    # top-row switch sockets from the rear.
    case.visual(
        Box((CASE_WIDTH - 0.032, 0.0065, 0.0090)),
        origin=Origin(xyz=(0.0, CASE_DEPTH / 2.0 - 0.014, 0.0170)),
        material=materials["case"],
        name="rear_rail",
    )
    case.visual(
        Box((CASE_WIDTH - 0.036, 0.0030, 0.0022)),
        origin=Origin(xyz=(0.0, -CASE_DEPTH / 2.0 + 0.010, 0.0142)),
        material=materials["brushed_steel"],
        name="front_accent_line",
    )
    _add_usb_c_port(case, materials=materials)
    for index, (x, y) in enumerate(
        (
            (-CASE_WIDTH * 0.38, -CASE_DEPTH * 0.38),
            (CASE_WIDTH * 0.38, -CASE_DEPTH * 0.38),
            (-CASE_WIDTH * 0.38, CASE_DEPTH * 0.35),
            (CASE_WIDTH * 0.38, CASE_DEPTH * 0.35),
        )
    ):
        case.visual(
            Box((0.052, 0.014, 0.003)),
            origin=Origin(xyz=(x, y, 0.0012)),
            material=materials["rubber"],
            name=f"rubber_foot_{index}",
        )

    switch_w = 0.0140
    switch_h = 0.0070
    switch_center_z = SWITCH_TOP_Z - switch_h / 2.0
    for spec in KEY_SPECS:
        x, y = _key_xy(spec)
        suffix = _safe_visual_suffix(spec.name)
        switch_visual_name = "switch_a" if spec.name == "key_a" else f"switch_{suffix}"
        case.visual(
            Box((switch_w, switch_w, switch_h)),
            origin=Origin(xyz=(x, y, switch_center_z)),
            material=materials["switch"],
            name=switch_visual_name,
        )

    cap_cache: dict[float, object] = {}
    for spec in KEY_SPECS:
        x, y = _key_xy(spec)
        key_part = model.part(
            spec.name,
            meta={
                "row_y_units": spec.y_u,
                "x_units": spec.x_u,
                "width_units": spec.width_u,
                "switch_family": "Cherry MX style",
            },
        )
        cap_material = materials["accent"] if spec.accent else materials["key"]
        if spec.name in {"key_esc"}:
            cap_material = model.material("muted_coral_escape", rgba=(0.75, 0.20, 0.16, 1.0))
        key_part.visual(
            _make_keycap_mesh(spec.width_u, cap_cache),
            origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM)),
            material=cap_material,
            name="keycap",
        )
        # Cherry-style cross stem: two intersecting red ribs that travel with the
        # cap and bottom out on the individual switch housing.
        key_part.visual(
            Box((0.0056, 0.00155, 0.0140)),
            origin=Origin(xyz=(0.0, 0.0, 0.0070)),
            material=materials["stem"],
            name="stem_x",
        )
        key_part.visual(
            Box((0.00155, 0.0056, 0.0140)),
            origin=Origin(xyz=(0.0, 0.0, 0.0070)),
            material=materials["stem"],
            name="stem_y",
        )
        # Minimal legends as raised inlays: narrow/long bars distinguish
        # alphanumerics, modifiers, arrows, spacebar, and homing bumps without
        # depending on font assets.
        legend_material = materials["legend_light"] if spec.accent or spec.name == "key_esc" else materials["legend"]
        if spec.name == "key_space":
            legend_size = (0.040, 0.0020, 0.0007)
            legend_y = -0.001
        elif spec.width_u >= 1.75:
            legend_size = (0.012, 0.0020, 0.0007)
            legend_y = 0.0028
        elif spec.name in {"key_up", "key_down", "key_left", "key_right"}:
            legend_size = (0.007, 0.007, 0.0007)
            legend_y = 0.0
        else:
            legend_size = (0.0075, 0.0016, 0.0007)
            legend_y = 0.0030
        key_part.visual(
            Box(legend_size),
            origin=Origin(xyz=(0.0, legend_y, CAP_BOTTOM + CAP_HEIGHT + 0.00020)),
            material=legend_material,
            name="legend_mark",
        )
        if spec.home_dot:
            key_part.visual(
                Box((0.0035, 0.0035, 0.0010)),
                origin=Origin(xyz=(0.0, -0.0028, CAP_BOTTOM + CAP_HEIGHT + 0.00030)),
                material=legend_material,
                name="home_bump",
            )
        _add_stabilizers(key_part, spec.width_u, materials["brushed_steel"])
        model.articulation(
            f"case_to_{spec.name}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key_part,
            origin=Origin(xyz=(x, y, SWITCH_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.4, velocity=0.08, lower=0.0, upper=KEY_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")

    ctx.check(
        "80 percent layout has 87 keys",
        sum(1 for part in object_model.parts if part.name.startswith("key_")) == 87,
        details="Expected an ANSI tenkeyless 87-key layout.",
    )
    prismatic_keys = [
        joint
        for joint in object_model.articulations
        if joint.name.startswith("case_to_key_") and joint.articulation_type == ArticulationType.PRISMATIC
    ]
    ctx.check(
        "every key has its own vertical switch travel",
        len(prismatic_keys) == 87
        and all(joint.axis == (0.0, 0.0, -1.0) for joint in prismatic_keys)
        and all(abs((joint.motion_limits.upper or 0.0) - KEY_TRAVEL) < 1e-6 for joint in prismatic_keys),
        details=f"prismatic_key_joints={len(prismatic_keys)}",
    )
    ctx.check(
        "usb-c port is present",
        case.get_visual("usb_c_opening") is not None and case.get_visual("usb_c_tongue") is not None,
        details="Rear USB-C opening and tongue should both be modeled.",
    )
    ctx.check(
        "tenkeyless clusters are unit spaced",
        abs(object_model.get_part("key_f2").meta["x_units"] - object_model.get_part("key_f1").meta["x_units"] - 1.0) < 1e-6
        and abs(object_model.get_part("key_home").meta["x_units"] - object_model.get_part("key_insert").meta["x_units"] - 1.0) < 1e-6
        and object_model.get_part("key_space").meta["width_units"] == 6.25,
        details="Function keys, navigation keys, and 6.25u spacebar should follow standard ANSI TKL spacing.",
    )

    # Resting stems are physically seated in their switch guide; pressing a key
    # moves the Cherry-style stem downward into that housing.
    ctx.expect_contact(
        "key_a",
        case,
        elem_a="stem_x",
        elem_b="switch_a",
        contact_tol=0.0005,
        name="unpressed cherry stem is seated in switch",
    )
    key_a_joint = object_model.get_articulation("case_to_key_a")
    rest_position = ctx.part_world_position("key_a")
    with ctx.pose({key_a_joint: KEY_TRAVEL}):
        pressed_position = ctx.part_world_position("key_a")
        ctx.expect_gap(
            "key_a",
            case,
            axis="z",
            max_penetration=KEY_TRAVEL + 0.0005,
            positive_elem="stem_x",
            negative_elem="switch_a",
            name="pressed cherry stem travels into switch",
        )
    ctx.check(
        "representative key moves downward",
        rest_position is not None
        and pressed_position is not None
        and pressed_position[2] < rest_position[2] - 0.0035,
        details=f"rest={rest_position}, pressed={pressed_position}",
    )

    return ctx.report()


object_model = build_object_model()
