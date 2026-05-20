from __future__ import annotations

from dataclasses import dataclass

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PITCH = 0.0190
DECK_TOP = 0.0215
KEYCAP_BOTTOM = 0.0040


@dataclass(frozen=True)
class KeySpec:
    name: str
    x: float
    y: float
    z: float
    width_u: float = 1.0
    depth_u: float = 1.0
    family: str = "main"


def _profile_at(
    width: float,
    depth: float,
    *,
    radius: float,
    x: float = 0.0,
    y: float = 0.0,
    corner_segments: int = 4,
) -> list[tuple[float, float]]:
    return [
        (px + x, py + y)
        for px, py in rounded_rect_profile(
            width,
            depth,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _make_keycap_mesh(name: str, *, width: float, depth: float, height: float):
    lower = _profile_at(
        width,
        depth,
        radius=min(0.0030, width * 0.18, depth * 0.18),
        corner_segments=5,
    )
    shoulder = _profile_at(
        width * 0.90,
        depth * 0.90,
        radius=min(0.0028, width * 0.16, depth * 0.16),
        y=-depth * 0.015,
        corner_segments=5,
    )
    upper = _profile_at(
        width * 0.78,
        depth * 0.78,
        radius=min(0.0024, width * 0.14, depth * 0.14),
        y=-depth * 0.030,
        corner_segments=5,
    )
    return mesh_from_geometry(
        LoftGeometry(
            [
                [(x, y, 0.0) for x, y in lower],
                [(x, y, height * 0.55) for x, y in shoulder],
                [(x, y, height) for x, y in upper],
            ],
            cap=True,
            closed=True,
        ),
        name,
    )


def _row_specs(
    keys: list[tuple[str, float]],
    *,
    x_left: float,
    y: float,
    z: float,
    family: str = "main",
) -> list[KeySpec]:
    specs: list[KeySpec] = []
    cursor = x_left
    for name, width_u in keys:
        x = cursor + (width_u * PITCH) * 0.5
        specs.append(KeySpec(name=name, x=x, y=y, z=z, width_u=width_u, family=family))
        cursor += width_u * PITCH
    return specs


def _make_key_specs() -> list[KeySpec]:
    main_left = -0.225
    specs: list[KeySpec] = []

    # Function row, with real cluster gaps between Esc and F-key groups.
    f_y = 0.066
    f_z = 0.0325
    function_xs = [
        ("key_escape", -0.2155),
        ("key_f1", -0.1660),
        ("key_f2", -0.1470),
        ("key_f3", -0.1280),
        ("key_f4", -0.1090),
        ("key_f5", -0.0710),
        ("key_f6", -0.0520),
        ("key_f7", -0.0330),
        ("key_f8", -0.0140),
        ("key_f9", 0.0240),
        ("key_f10", 0.0430),
        ("key_f11", 0.0620),
        ("key_f12", 0.0810),
        ("key_print_screen", 0.1300),
        ("key_scroll_lock", 0.1490),
        ("key_pause", 0.1680),
    ]
    for name, x in function_xs:
        specs.append(KeySpec(name=name, x=x, y=f_y, z=f_z, family="function"))

    specs.extend(
        _row_specs(
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
            x_left=main_left,
            y=0.039,
            z=0.0305,
        )
    )
    specs.extend(
        _row_specs(
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
                ("key_left_bracket", 1.0),
                ("key_right_bracket", 1.0),
                ("key_backslash", 1.5),
            ],
            x_left=main_left,
            y=0.016,
            z=0.0287,
        )
    )
    specs.extend(
        _row_specs(
            [
                ("key_caps", 1.75),
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
            x_left=main_left,
            y=-0.007,
            z=0.0270,
        )
    )
    specs.extend(
        _row_specs(
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
            x_left=main_left,
            y=-0.030,
            z=0.0253,
        )
    )
    specs.extend(
        _row_specs(
            [
                ("key_left_ctrl", 1.25),
                ("key_left_meta", 1.25),
                ("key_left_alt", 1.25),
                ("key_space", 6.25),
                ("key_right_alt", 1.25),
                ("key_context", 1.25),
                ("key_right_ctrl", 1.25),
                ("key_fn", 1.25),
            ],
            x_left=main_left,
            y=-0.053,
            z=0.0238,
        )
    )

    # Navigation island between the main block and numeric keypad.
    for row_y, row_z, names in [
        (0.039, 0.0305, ("key_insert", "key_home", "key_page_up")),
        (0.016, 0.0287, ("key_delete", "key_end", "key_page_down")),
    ]:
        for col, name in enumerate(names):
            specs.append(KeySpec(name=name, x=0.087 + col * PITCH, y=row_y, z=row_z, family="nav"))
    specs.extend(
        [
            KeySpec("key_up", x=0.106, y=-0.030, z=0.0253, family="nav"),
            KeySpec("key_left", x=0.087, y=-0.053, z=0.0238, family="nav"),
            KeySpec("key_down", x=0.106, y=-0.053, z=0.0238, family="nav"),
            KeySpec("key_right", x=0.125, y=-0.053, z=0.0238, family="nav"),
        ]
    )

    # Numeric keypad, including tall plus and enter plungers and a wide zero.
    num_left = 0.157
    num_x = [num_left + (col + 0.5) * PITCH for col in range(4)]
    num_rows = [0.039, 0.016, -0.007, -0.030, -0.053]
    num_z = [0.0305, 0.0287, 0.0270, 0.0253, 0.0238]
    for col, name in enumerate(("key_num_lock", "key_num_divide", "key_num_multiply", "key_num_minus")):
        specs.append(KeySpec(name=name, x=num_x[col], y=num_rows[0], z=num_z[0], family="numpad"))
    for col, name in enumerate(("key_num_7", "key_num_8", "key_num_9")):
        specs.append(KeySpec(name=name, x=num_x[col], y=num_rows[1], z=num_z[1], family="numpad"))
    specs.append(
        KeySpec(
            "key_num_plus",
            x=num_x[3],
            y=(num_rows[1] + num_rows[2]) * 0.5,
            z=(num_z[1] + num_z[2]) * 0.5,
            depth_u=2.0,
            family="numpad",
        )
    )
    for col, name in enumerate(("key_num_4", "key_num_5", "key_num_6")):
        specs.append(KeySpec(name=name, x=num_x[col], y=num_rows[2], z=num_z[2], family="numpad"))
    for col, name in enumerate(("key_num_1", "key_num_2", "key_num_3")):
        specs.append(KeySpec(name=name, x=num_x[col], y=num_rows[3], z=num_z[3], family="numpad"))
    specs.append(
        KeySpec(
            "key_numpad_enter",
            x=num_x[3],
            y=(num_rows[3] + num_rows[4]) * 0.5,
            z=(num_z[3] + num_z[4]) * 0.5,
            depth_u=2.0,
            family="numpad",
        )
    )
    specs.append(KeySpec("key_num_0", x=(num_x[0] + num_x[1]) * 0.5, y=num_rows[4], z=num_z[4], width_u=2.0, family="numpad"))
    specs.append(KeySpec("key_num_decimal", x=num_x[2], y=num_rows[4], z=num_z[4], family="numpad"))

    return specs


def _opening_size(spec: KeySpec) -> tuple[float, float]:
    return (spec.width_u * PITCH - 0.0020, spec.depth_u * PITCH - 0.0020)


def _cap_size(spec: KeySpec) -> tuple[float, float]:
    opening_w, opening_d = _opening_size(spec)
    return (opening_w - 0.0010, opening_d - 0.0010)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_size_office_keyboard")

    body = model.material("matte_charcoal", rgba=(0.09, 0.10, 0.11, 1.0))
    deck = model.material("satin_graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    switch_black = model.material("switch_bed_black", rgba=(0.025, 0.026, 0.028, 1.0))
    key_gray = model.material("warm_gray_keycaps", rgba=(0.54, 0.55, 0.55, 1.0))
    special_gray = model.material("wide_keycaps", rgba=(0.47, 0.48, 0.49, 1.0))
    stem_dark = model.material("dark_switch_stems", rgba=(0.05, 0.05, 0.055, 1.0))
    legend = model.material("soft_white_legends", rgba=(0.82, 0.84, 0.84, 1.0))

    key_specs = _make_key_specs()

    chassis = model.part("housing")
    chassis.visual(
        Box((0.490, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body,
        name="bottom_tray",
    )
    chassis.visual(
        Box((0.490, 0.014, 0.023)),
        origin=Origin(xyz=(0.0, -0.083, 0.0115)),
        material=body,
        name="front_lip",
    )
    chassis.visual(
        Box((0.490, 0.020, 0.032)),
        origin=Origin(xyz=(0.0, 0.080, 0.016)),
        material=body,
        name="rear_lip",
    )
    chassis.visual(
        Box((0.012, 0.180, 0.026)),
        origin=Origin(xyz=(-0.245, 0.0, 0.013)),
        material=body,
        name="side_rail_0",
    )
    chassis.visual(
        Box((0.012, 0.180, 0.026)),
        origin=Origin(xyz=(0.245, 0.0, 0.013)),
        material=body,
        name="side_rail_1",
    )
    chassis.visual(
        Box((0.452, 0.142, 0.0030)),
        origin=Origin(xyz=(0.0, -0.001, 0.0160)),
        material=switch_black,
        name="internal_switch_bed",
    )

    hole_profiles = []
    for spec in key_specs:
        opening_w, opening_d = _opening_size(spec)
        hole_profiles.append(
            _profile_at(
                opening_w,
                opening_d,
                radius=min(0.0024, opening_w * 0.14, opening_d * 0.14),
                x=spec.x,
                y=spec.y,
                corner_segments=4,
            )
        )
    deck_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _profile_at(0.462, 0.150, radius=0.010, corner_segments=8),
            hole_profiles,
            0.004,
            center=True,
        ),
        "keyboard_opening_deck",
    )
    chassis.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP - 0.002)),
        material=deck,
        name="opening_deck",
    )

    # Low rubber feet and a shallow rear ridge make the tray read as a full-size
    # desktop keyboard rather than a flat plate.
    for x in (-0.200, 0.200):
        for y in (-0.070, 0.070):
            chassis.visual(
                Box((0.050, 0.018, 0.004)),
                origin=Origin(xyz=(x, y, -0.002)),
                material=switch_black,
                name=f"rubber_foot_{len(chassis.visuals)}",
            )
    chassis.visual(
        Box((0.410, 0.006, 0.004)),
        origin=Origin(xyz=(0.000, 0.072, 0.033)),
        material=deck,
        name="rear_step_ridge",
    )

    mesh_cache: dict[tuple[float, float, float], object] = {}
    key_limits = MotionLimits(effort=2.0, velocity=0.10, lower=0.0, upper=0.0040)
    for spec in key_specs:
        cap_w, cap_d = _cap_size(spec)
        cap_h = 0.0085 if spec.family == "function" else 0.0100
        cache_key = (round(cap_w, 4), round(cap_d, 4), cap_h)
        cap_mesh = mesh_cache.get(cache_key)
        if cap_mesh is None:
            mesh_name = f"keycap_{int(spec.width_u * 100)}_{int(spec.depth_u * 100)}"
            cap_mesh = _make_keycap_mesh(mesh_name, width=cap_w, depth=cap_d, height=cap_h)
            mesh_cache[cache_key] = cap_mesh

        key_part = model.part(spec.name)
        cap_material = special_gray if spec.width_u > 1.1 or spec.depth_u > 1.1 else key_gray
        key_part.visual(
            cap_mesh,
            origin=Origin(xyz=(0.0, 0.0, KEYCAP_BOTTOM)),
            material=cap_material,
            name="cap",
        )

        stem_bottom = DECK_TOP - spec.z - 0.0010
        stem_top = KEYCAP_BOTTOM
        stem_h = max(0.006, stem_top - stem_bottom)
        stem_center_z = (stem_top + stem_bottom) * 0.5
        stem_w = min(max(cap_w * 0.28, 0.0055), 0.010)
        stem_d = min(max(cap_d * 0.28, 0.0055), 0.010)
        key_part.visual(
            Box((stem_w, stem_d, stem_h)),
            origin=Origin(xyz=(0.0, 0.0, stem_center_z)),
            material=stem_dark,
            name="stem",
        )

        if spec.width_u >= 2.0:
            stabilizer_x = max(cap_w * 0.34, 0.018)
            for sign in (-1.0, 1.0):
                key_part.visual(
                    Box((0.0040, min(0.0065, cap_d * 0.45), stem_h)),
                    origin=Origin(xyz=(sign * stabilizer_x, 0.0, stem_center_z)),
                    material=stem_dark,
                    name=f"stabilizer_{0 if sign < 0 else 1}",
                )
        if spec.depth_u >= 2.0:
            stabilizer_y = max(cap_d * 0.32, 0.018)
            for sign in (-1.0, 1.0):
                key_part.visual(
                    Box((min(0.0065, cap_w * 0.45), 0.0040, stem_h)),
                    origin=Origin(xyz=(0.0, sign * stabilizer_y, stem_center_z)),
                    material=stem_dark,
                    name=f"stabilizer_{0 if sign < 0 else 1}",
                )

        if spec.name in {"key_f", "key_j", "key_num_5"}:
            key_part.visual(
                Box((cap_w * 0.42, 0.0014, 0.0006)),
                origin=Origin(xyz=(0.0, -cap_d * 0.18, KEYCAP_BOTTOM + cap_h + 0.0003)),
                material=legend,
                name="homing_bar",
            )
        elif spec.width_u >= 2.0 or spec.name in {"key_enter", "key_space", "key_backspace"}:
            key_part.visual(
                Box((min(cap_w * 0.46, 0.026), 0.0012, 0.0005)),
                origin=Origin(xyz=(0.0, -cap_d * 0.18, KEYCAP_BOTTOM + cap_h + 0.00025)),
                material=legend,
                name="legend_bar",
            )

        model.articulation(
            f"housing_to_{spec.name}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=key_part,
            origin=Origin(xyz=(spec.x, spec.y, spec.z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=key_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    key_parts = [part for part in object_model.parts if part.name.startswith("key_")]
    plunger_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.PRISMATIC
        and joint.child.startswith("key_")
    ]
    ctx.check(
        "individual key plungers",
        len(key_parts) >= 85 and len(plunger_joints) == len(key_parts),
        details=f"keys={len(key_parts)} prismatic={len(plunger_joints)}",
    )

    for key_name in ("key_a", "key_space", "key_enter", "key_num_7", "key_numpad_enter"):
        key = object_model.get_part(key_name)
        joint = object_model.get_articulation(f"housing_to_{key_name}")
        start = ctx.part_world_position(key)
        with ctx.pose({joint: 0.0040}):
            pressed = ctx.part_world_position(key)
        ctx.check(
            f"{key_name} presses downward",
            start is not None and pressed is not None and pressed[2] < start[2] - 0.0035,
            details=f"start={start}, pressed={pressed}",
        )

    ctx.expect_overlap(
        "key_space",
        "housing",
        axes="x",
        elem_a="cap",
        elem_b="opening_deck",
        min_overlap=0.05,
        name="space bar sits in tray opening",
    )
    ctx.expect_gap(
        "key_space",
        "housing",
        axis="z",
        positive_elem="cap",
        negative_elem="opening_deck",
        min_gap=0.001,
        max_gap=0.020,
        name="space bar cap clears deck",
    )
    ctx.expect_overlap(
        "key_numpad_enter",
        "housing",
        axes="xy",
        elem_a="cap",
        elem_b="opening_deck",
        min_overlap=0.010,
        name="numeric keypad has large enter opening",
    )

    return ctx.report()


object_model = build_object_model()
