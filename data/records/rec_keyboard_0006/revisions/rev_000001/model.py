from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

KEY_PITCH = 0.019
PLATE_THICKNESS = 0.002
PLATE_WIDTH = 0.450
BODY_WIDTH = 0.462
SIDE_WALL_THICKNESS = (BODY_WIDTH - PLATE_WIDTH) * 0.5
ROW_ORDER = ["bottom", "z", "a", "q", "number", "function"]
ROW_SPECS = {
    "bottom": {
        "y": -0.053,
        "surface_z": 0.0130,
        "plate_depth": 0.018,
        "hole_depth": 0.0142,
        "cap_height": 0.0051,
        "cap_tilt": 0.030,
    },
    "z": {
        "y": -0.031,
        "surface_z": 0.0156,
        "plate_depth": 0.017,
        "hole_depth": 0.0138,
        "cap_height": 0.0054,
        "cap_tilt": 0.050,
    },
    "a": {
        "y": -0.009,
        "surface_z": 0.0182,
        "plate_depth": 0.017,
        "hole_depth": 0.0138,
        "cap_height": 0.0056,
        "cap_tilt": 0.035,
    },
    "q": {
        "y": 0.013,
        "surface_z": 0.0208,
        "plate_depth": 0.017,
        "hole_depth": 0.0138,
        "cap_height": 0.0054,
        "cap_tilt": 0.008,
    },
    "number": {
        "y": 0.035,
        "surface_z": 0.0236,
        "plate_depth": 0.017,
        "hole_depth": 0.0138,
        "cap_height": 0.0051,
        "cap_tilt": -0.018,
    },
    "function": {
        "y": 0.059,
        "surface_z": 0.0266,
        "plate_depth": 0.014,
        "hole_depth": 0.0118,
        "cap_height": 0.0045,
        "cap_tilt": -0.035,
    },
}
FRONT_EDGE_Y = ROW_SPECS["bottom"]["y"] - (ROW_SPECS["bottom"]["plate_depth"] * 0.5) - 0.010
BACK_EDGE_Y = ROW_SPECS["function"]["y"] + (ROW_SPECS["function"]["plate_depth"] * 0.5) + 0.012
BODY_DEPTH = BACK_EDGE_Y - FRONT_EDGE_Y
BODY_CENTER_Y = (FRONT_EDGE_Y + BACK_EDGE_Y) * 0.5
TRAVEL = 0.0018
MAIN_WIDTH_U = 15.0
NAV_GAP_U = 1.0
NAV_WIDTH_U = 3.0
NUMPAD_GAP_U = 0.75
NUMPAD_WIDTH_U = 4.0
TOTAL_UNITS = MAIN_WIDTH_U + NAV_GAP_U + NAV_WIDTH_U + NUMPAD_GAP_U + NUMPAD_WIDTH_U
MAIN_LEFT = -(TOTAL_UNITS * KEY_PITCH) * 0.5
NAV_LEFT = MAIN_LEFT + (MAIN_WIDTH_U + NAV_GAP_U) * KEY_PITCH
NUMPAD_LEFT = NAV_LEFT + (NAV_WIDTH_U + NUMPAD_GAP_U) * KEY_PITCH


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _tapered_cap_geometry(width: float, depth: float, height: float) -> MeshGeometry:
    geom = MeshGeometry()
    top_width = width * 0.88
    top_depth = depth * 0.82
    bottom = [
        (-width * 0.5, -depth * 0.5, 0.0),
        (width * 0.5, -depth * 0.5, 0.0),
        (width * 0.5, depth * 0.5, 0.0),
        (-width * 0.5, depth * 0.5, 0.0),
    ]
    top = [
        (-top_width * 0.5, -top_depth * 0.5, height),
        (top_width * 0.5, -top_depth * 0.5, height),
        (top_width * 0.5, top_depth * 0.5, height),
        (-top_width * 0.5, top_depth * 0.5, height),
    ]
    ids = [geom.add_vertex(*vertex) for vertex in [*bottom, *top]]
    _add_quad(geom, ids[0], ids[1], ids[2], ids[3])
    _add_quad(geom, ids[4], ids[7], ids[6], ids[5])
    _add_quad(geom, ids[0], ids[4], ids[5], ids[1])
    _add_quad(geom, ids[1], ids[5], ids[6], ids[2])
    _add_quad(geom, ids[2], ids[6], ids[7], ids[3])
    _add_quad(geom, ids[3], ids[7], ids[4], ids[0])
    return geom


def _make_key_spec(
    row_name: str,
    region: str,
    key_name: str,
    units: float,
    center_x: float,
) -> dict[str, float | str]:
    row = ROW_SPECS[row_name]
    key_width = units * KEY_PITCH
    cap_width = max(key_width - 0.0018, key_width * 0.90)
    hole_width = max(key_width - 0.0032, 0.0125)
    hole_depth = row["hole_depth"]
    return {
        "name": key_name,
        "joint": f"press_{key_name}",
        "row": row_name,
        "region": region,
        "units": units,
        "x": center_x,
        "y": row["y"],
        "surface_z": row["surface_z"],
        "cap_width": cap_width,
        "cap_depth": hole_depth + 0.0032,
        "cap_height": row["cap_height"],
        "hole_width": hole_width,
        "hole_depth": hole_depth,
        "skirt_width": max(hole_width - 0.0008, 0.0105),
        "skirt_depth": max(hole_depth - 0.0008, 0.0105),
        "travel": TRAVEL,
    }


def _append_row_sequence(
    out: list[dict[str, float | str]],
    *,
    row_name: str,
    region: str,
    left_x: float,
    items: list[tuple[str | None, float]],
) -> None:
    cursor = left_x
    for key_name, units in items:
        key_width = units * KEY_PITCH
        if key_name is not None:
            out.append(_make_key_spec(row_name, region, key_name, units, cursor + (key_width * 0.5)))
        cursor += key_width


def _build_key_specs() -> list[dict[str, float | str]]:
    key_specs: list[dict[str, float | str]] = []

    _append_row_sequence(
        key_specs,
        row_name="function",
        region="function",
        left_x=MAIN_LEFT,
        items=[
            ("key_escape", 1.0),
            (None, 0.5),
            ("key_f1", 1.0),
            ("key_f2", 1.0),
            ("key_f3", 1.0),
            ("key_f4", 1.0),
            (None, 0.5),
            ("key_f5", 1.0),
            ("key_f6", 1.0),
            ("key_f7", 1.0),
            ("key_f8", 1.0),
            (None, 0.5),
            ("key_f9", 1.0),
            ("key_f10", 1.0),
            ("key_f11", 1.0),
            ("key_f12", 1.0),
            (None, 0.5),
            ("key_print_screen", 1.0),
            ("key_scroll_lock", 1.0),
            ("key_pause", 1.0),
        ],
    )

    _append_row_sequence(
        key_specs,
        row_name="number",
        region="main",
        left_x=MAIN_LEFT,
        items=[
            ("key_backtick", 1.0),
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
    _append_row_sequence(
        key_specs,
        row_name="q",
        region="main",
        left_x=MAIN_LEFT,
        items=[
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
    _append_row_sequence(
        key_specs,
        row_name="a",
        region="main",
        left_x=MAIN_LEFT,
        items=[
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
    _append_row_sequence(
        key_specs,
        row_name="z",
        region="main",
        left_x=MAIN_LEFT,
        items=[
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
    _append_row_sequence(
        key_specs,
        row_name="bottom",
        region="main",
        left_x=MAIN_LEFT,
        items=[
            ("key_left_ctrl", 1.25),
            ("key_left_win", 1.25),
            ("key_left_alt", 1.25),
            ("key_space", 6.25),
            ("key_right_alt", 1.25),
            ("key_fn", 1.25),
            ("key_menu", 1.25),
            ("key_right_ctrl", 1.25),
        ],
    )

    _append_row_sequence(
        key_specs,
        row_name="number",
        region="nav",
        left_x=NAV_LEFT,
        items=[
            ("key_insert", 1.0),
            ("key_home", 1.0),
            ("key_page_up", 1.0),
        ],
    )
    _append_row_sequence(
        key_specs,
        row_name="q",
        region="nav",
        left_x=NAV_LEFT,
        items=[
            ("key_delete", 1.0),
            ("key_end", 1.0),
            ("key_page_down", 1.0),
        ],
    )
    _append_row_sequence(
        key_specs,
        row_name="z",
        region="arrow",
        left_x=NAV_LEFT + KEY_PITCH,
        items=[("key_arrow_up", 1.0)],
    )
    _append_row_sequence(
        key_specs,
        row_name="bottom",
        region="arrow",
        left_x=NAV_LEFT,
        items=[
            ("key_arrow_left", 1.0),
            ("key_arrow_down", 1.0),
            ("key_arrow_right", 1.0),
        ],
    )

    _append_row_sequence(
        key_specs,
        row_name="number",
        region="numpad",
        left_x=NUMPAD_LEFT,
        items=[
            ("key_numpad_num_lock", 1.0),
            ("key_numpad_divide", 1.0),
            ("key_numpad_multiply", 1.0),
            ("key_numpad_minus", 1.0),
        ],
    )
    _append_row_sequence(
        key_specs,
        row_name="q",
        region="numpad",
        left_x=NUMPAD_LEFT,
        items=[
            ("key_numpad_7", 1.0),
            ("key_numpad_8", 1.0),
            ("key_numpad_9", 1.0),
            ("key_numpad_plus", 1.0),
        ],
    )
    _append_row_sequence(
        key_specs,
        row_name="a",
        region="numpad",
        left_x=NUMPAD_LEFT,
        items=[
            ("key_numpad_4", 1.0),
            ("key_numpad_5", 1.0),
            ("key_numpad_6", 1.0),
            ("key_numpad_enter", 1.0),
        ],
    )
    _append_row_sequence(
        key_specs,
        row_name="z",
        region="numpad",
        left_x=NUMPAD_LEFT,
        items=[
            ("key_numpad_1", 1.0),
            ("key_numpad_2", 1.0),
            ("key_numpad_3", 1.0),
            (None, 1.0),
        ],
    )
    _append_row_sequence(
        key_specs,
        row_name="bottom",
        region="numpad",
        left_x=NUMPAD_LEFT,
        items=[
            ("key_numpad_0", 2.0),
            ("key_numpad_decimal", 1.0),
            (None, 1.0),
        ],
    )
    return key_specs


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_keyboard", assets=ASSETS)

    housing_black = model.material("housing_black", rgba=(0.16, 0.17, 0.19, 1.0))
    deck_black = model.material("deck_black", rgba=(0.11, 0.12, 0.13, 1.0))
    switch_dark = model.material("switch_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    keycap_gray = model.material("keycap_gray", rgba=(0.84, 0.85, 0.82, 1.0))
    keycap_dark = model.material("keycap_dark", rgba=(0.74, 0.75, 0.73, 1.0))

    key_specs = _build_key_specs()
    by_row: dict[str, list[dict[str, float | str]]] = {row_name: [] for row_name in ROW_ORDER}
    for spec in key_specs:
        by_row[str(spec["row"])].append(spec)

    housing = model.part("housing")
    housing.visual(
        Box((BODY_WIDTH, BODY_DEPTH, 0.004)),
        origin=Origin(xyz=(0.0, BODY_CENTER_Y, 0.002)),
        material=housing_black,
        name="bottom_shell",
    )
    housing.visual(
        Box((BODY_WIDTH, 0.012, ROW_SPECS["bottom"]["surface_z"] - 0.0005)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_EDGE_Y + 0.006,
                (ROW_SPECS["bottom"]["surface_z"] - 0.0005) * 0.5,
            )
        ),
        material=housing_black,
        name="front_lip",
    )
    housing.visual(
        Box((BODY_WIDTH, 0.012, ROW_SPECS["function"]["surface_z"] + 0.005)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_EDGE_Y - 0.006,
                (ROW_SPECS["function"]["surface_z"] + 0.005) * 0.5,
            )
        ),
        material=housing_black,
        name="back_wall",
    )
    housing.visual(
        Box((SIDE_WALL_THICKNESS, BODY_DEPTH, ROW_SPECS["function"]["surface_z"] + 0.005)),
        origin=Origin(
            xyz=(
                -(BODY_WIDTH * 0.5) + (SIDE_WALL_THICKNESS * 0.5),
                BODY_CENTER_Y,
                (ROW_SPECS["function"]["surface_z"] + 0.005) * 0.5,
            )
        ),
        material=housing_black,
        name="left_wall",
    )
    housing.visual(
        Box((SIDE_WALL_THICKNESS, BODY_DEPTH, ROW_SPECS["function"]["surface_z"] + 0.005)),
        origin=Origin(
            xyz=(
                (BODY_WIDTH * 0.5) - (SIDE_WALL_THICKNESS * 0.5),
                BODY_CENTER_Y,
                (ROW_SPECS["function"]["surface_z"] + 0.005) * 0.5,
            )
        ),
        material=housing_black,
        name="right_wall",
    )
    for front_row, rear_row in zip(ROW_ORDER[:-1], ROW_ORDER[1:]):
        front_back = ROW_SPECS[front_row]["y"] + (ROW_SPECS[front_row]["plate_depth"] * 0.5)
        rear_front = ROW_SPECS[rear_row]["y"] - (ROW_SPECS[rear_row]["plate_depth"] * 0.5)
        gap_depth = max(rear_front - front_back + 0.002, 0.004)
        gap_y = (front_back + rear_front) * 0.5
        step_height = ROW_SPECS[rear_row]["surface_z"] - 0.0005
        housing.visual(
            Box((PLATE_WIDTH, gap_depth, step_height)),
            origin=Origin(xyz=(0.0, gap_y, step_height * 0.5)),
            material=housing_black,
            name=f"step_{front_row}_to_{rear_row}",
        )

    for row_name in ROW_ORDER:
        row = ROW_SPECS[row_name]
        hole_profiles = [
            _translate_profile(
                _rect_profile(float(spec["hole_width"]), float(spec["hole_depth"])),
                float(spec["x"]),
                0.0,
            )
            for spec in by_row[row_name]
        ]
        plate_mesh = mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _rect_profile(PLATE_WIDTH, row["plate_depth"]),
                hole_profiles,
                PLATE_THICKNESS,
                center=True,
            ),
            ASSETS.mesh_path(f"keyboard_plate_{row_name}.obj"),
        )
        housing.visual(
            plate_mesh,
            origin=Origin(xyz=(0.0, row["y"], row["surface_z"] - (PLATE_THICKNESS * 0.5))),
            material=deck_black,
            name=f"plate_{row_name}",
        )

    housing.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, ROW_SPECS["function"]["surface_z"] + 0.010)),
        mass=1.05,
        origin=Origin(
            xyz=(
                0.0,
                BODY_CENTER_Y,
                (ROW_SPECS["function"]["surface_z"] + 0.010) * 0.5,
            )
        ),
    )

    switch_bed = model.part("switch_bed")
    switch_bed.visual(
        Box((PLATE_WIDTH - 0.010, BODY_DEPTH - 0.024, 0.002)),
        origin=Origin(xyz=(0.0, BODY_CENTER_Y, 0.005)),
        material=switch_dark,
        name="bed_base",
    )
    for row_name in ROW_ORDER:
        row = ROW_SPECS[row_name]
        bed_top = row["surface_z"] - 0.006
        bed_height = max(bed_top - 0.006, 0.001)
        switch_bed.visual(
            Box((PLATE_WIDTH - 0.012, row["plate_depth"] - 0.004, bed_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    row["y"],
                    0.006 + (bed_height * 0.5),
                )
            ),
            material=switch_dark,
            name=f"bed_{row_name}",
        )
    switch_bed.inertial = Inertial.from_geometry(
        Box((PLATE_WIDTH - 0.020, BODY_DEPTH - 0.024, 0.018)),
        mass=0.42,
        origin=Origin(xyz=(0.0, BODY_CENTER_Y, 0.009)),
    )
    model.articulation(
        "housing_to_switch_bed",
        ArticulationType.FIXED,
        parent=housing,
        child=switch_bed,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    cap_mesh_cache: dict[tuple[int, int, int], object] = {}
    for spec in key_specs:
        cap_key = (
            int(round(float(spec["cap_width"]) * 1000.0)),
            int(round(float(spec["cap_depth"]) * 1000.0)),
            int(round(float(spec["cap_height"]) * 1000.0)),
        )
        cap_mesh = cap_mesh_cache.get(cap_key)
        if cap_mesh is None:
            cap_mesh = mesh_from_geometry(
                _tapered_cap_geometry(
                    float(spec["cap_width"]),
                    float(spec["cap_depth"]),
                    float(spec["cap_height"]),
                ),
                ASSETS.mesh_path(
                    f"keyboard_cap_{cap_key[0]}_{cap_key[1]}_{cap_key[2]}.obj"
                ),
            )
            cap_mesh_cache[cap_key] = cap_mesh

        key_part = model.part(str(spec["name"]))
        row_name = str(spec["row"])
        row = ROW_SPECS[row_name]
        key_part.visual(
            cap_mesh,
            origin=Origin(
                xyz=(0.0, 0.0, 0.0028),
                rpy=(row["cap_tilt"], 0.0, 0.0),
            ),
            material=keycap_gray if str(spec["region"]) != "function" else keycap_dark,
            name="cap",
        )
        key_part.visual(
            Box((float(spec["skirt_width"]), float(spec["skirt_depth"]), 0.0060)),
            origin=Origin(xyz=(0.0, 0.0, 0.0002)),
            material=keycap_dark,
            name="guide_skirt",
        )
        key_part.visual(
            Box((float(spec["hole_width"]) + 0.0010, float(spec["hole_depth"]) + 0.0010, 0.0012)),
            origin=Origin(xyz=(0.0, 0.0, -0.0026)),
            material=keycap_dark,
            name="retainer",
        )
        key_part.inertial = Inertial.from_geometry(
            Box((float(spec["cap_width"]), float(spec["cap_depth"]), 0.011)),
            mass=0.0045 + (0.0016 * float(spec["units"])),
            origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        )
        model.articulation(
            str(spec["joint"]),
            ArticulationType.PRISMATIC,
            parent=housing,
            child=key_part,
            origin=Origin(
                xyz=(
                    float(spec["x"]),
                    float(spec["y"]),
                    float(spec["surface_z"]),
                )
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.8 + (0.1 * float(spec["units"])),
                velocity=0.06,
                lower=0.0,
                upper=float(spec["travel"]),
            ),
        )

    model.meta["key_specs"] = key_specs
    model.meta["row_order"] = ROW_ORDER
    model.meta["row_plate_visuals"] = {row_name: f"plate_{row_name}" for row_name in ROW_ORDER}
    model.meta["row_bed_visuals"] = {row_name: f"bed_{row_name}" for row_name in ROW_ORDER}
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    switch_bed = object_model.get_part("switch_bed")
    key_specs = list(object_model.meta["key_specs"])
    row_order = list(object_model.meta["row_order"])
    row_plate_visuals = {
        row_name: housing.get_visual(visual_name)
        for row_name, visual_name in object_model.meta["row_plate_visuals"].items()
    }
    row_bed_visuals = {
        row_name: switch_bed.get_visual(visual_name)
        for row_name, visual_name in object_model.meta["row_bed_visuals"].items()
    }
    key_parts = {str(spec["name"]): object_model.get_part(str(spec["name"])) for spec in key_specs}
    key_joints = {
        str(spec["joint"]): object_model.get_articulation(str(spec["joint"]))
        for spec in key_specs
    }
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(
        switch_bed,
        housing,
        elem_a=switch_bed.get_visual("bed_base"),
        elem_b=housing.get_visual("bottom_shell"),
        name="switch_bed_seated_on_housing",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    if housing_aabb is not None:
        housing_width = housing_aabb[1][0] - housing_aabb[0][0]
        housing_depth = housing_aabb[1][1] - housing_aabb[0][1]
        housing_height = housing_aabb[1][2] - housing_aabb[0][2]
        ctx.check(
            "keyboard_envelope_reads_full_size",
            0.43 <= housing_width <= 0.49 and 0.13 <= housing_depth <= 0.18 and 0.02 <= housing_height <= 0.05,
            details=(
                f"expected a broad shallow office keyboard footprint, got "
                f"{housing_width:.3f} x {housing_depth:.3f} x {housing_height:.3f} m"
            ),
        )

    plate_tops: list[float] = []
    for row_name in row_order:
        plate_aabb = ctx.part_element_world_aabb(housing, elem=row_plate_visuals[row_name])
        if plate_aabb is not None:
            plate_tops.append(plate_aabb[1][2])
    ctx.check(
        "row_steps_climb_toward_back",
        len(plate_tops) == len(row_order)
        and all(earlier + 0.0015 < later for earlier, later in zip(plate_tops[:-1], plate_tops[1:])),
        details=f"expected stepped row heights, got z tops {plate_tops!r}",
    )

    main_keys = [spec for spec in key_specs if spec["region"] == "main"]
    numpad_keys = [spec for spec in key_specs if spec["region"] == "numpad"]
    ctx.check(
        "keyboard_has_main_typing_block_and_numpad",
        len(main_keys) >= 55 and len(numpad_keys) >= 15,
        details=f"expected dense typing block plus numeric keypad, got {len(main_keys)} main and {len(numpad_keys)} numpad keys",
    )

    for spec in key_specs:
        key_name = str(spec["name"])
        key_part = key_parts[key_name]
        joint = key_joints[str(spec["joint"])]
        plate = row_plate_visuals[str(spec["row"])]
        bed = row_bed_visuals[str(spec["row"])]
        guide = key_part.get_visual("guide_skirt")
        retainer = key_part.get_visual("retainer")

        axis = tuple(round(value, 6) for value in joint.axis)
        limits = joint.motion_limits
        ctx.check(
            f"{key_name}_moves_vertically",
            axis == (0.0, 0.0, -1.0),
            details=f"expected vertical plunger axis for {key_name}, got {axis!r}",
        )
        ctx.check(
            f"{key_name}_travel_is_short",
            limits is not None and abs(limits.lower) < 1e-9 and 0.001 <= limits.upper <= 0.0025,
            details=(
                f"expected short key travel for {key_name}, got "
                f"{None if limits is None else (limits.lower, limits.upper)}"
            ),
        )
        ctx.expect_contact(
            key_part,
            housing,
            elem_a=retainer,
            elem_b=plate,
            contact_tol=5e-5,
            name=f"{key_name}_retainer_captures_plate",
        )
        ctx.expect_overlap(
            key_part,
            switch_bed,
            axes="xy",
            elem_a=guide,
            elem_b=bed,
            min_overlap=min(float(spec["skirt_width"]), float(spec["skirt_depth"])) * 0.70,
            name=f"{key_name}_sits_above_switch_bed",
        )
        ctx.expect_gap(
            key_part,
            switch_bed,
            axis="z",
            positive_elem=guide,
            negative_elem=bed,
            min_gap=0.0012,
            max_gap=0.0045,
            name=f"{key_name}_rest_gap_above_switch_bed",
        )
        with ctx.pose({joint: float(spec["travel"])}):
            ctx.expect_gap(
                key_part,
                switch_bed,
                axis="z",
                positive_elem=guide,
                negative_elem=bed,
                min_gap=0.0001,
                max_gap=0.0030,
                name=f"{key_name}_pressed_gap_above_switch_bed",
            )

    ctx.expect_origin_gap(
        key_parts["key_numpad_5"],
        key_parts["key_h"],
        axis="x",
        min_gap=0.090,
        name="numpad_is_clearly_separate_from_typing_block",
    )
    ctx.expect_origin_gap(
        key_parts["key_arrow_up"],
        key_parts["key_n"],
        axis="x",
        min_gap=0.040,
        name="arrow_cluster_sits_between_main_block_and_numpad",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
