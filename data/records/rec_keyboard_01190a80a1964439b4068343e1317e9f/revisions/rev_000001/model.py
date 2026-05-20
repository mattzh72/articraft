from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)


U = 0.01905  # one keyboard unit: 19.05 mm
KEY_GAP = 0.00125
CASE_WIDTH = 0.374
CASE_DEPTH = 0.152
PLATE_TOP_Z = 0.0295
SWITCH_BASE_Z = 0.0340
SWITCH_TRAVEL = 0.0040
X_UNITS_TOTAL = 18.75
Y_UNITS_MID = 2.625


def _keycap_mesh(width: float, depth: float, height: float) -> MeshGeometry:
    """Tapered low-profile keycap mesh, local z in [0, height]."""
    geom = MeshGeometry()
    b = (width * 0.5, depth * 0.5)
    shoulder = (width * 0.44, depth * 0.44)
    top = (width * 0.38, depth * 0.39)
    z0 = 0.0
    z1 = height * 0.66
    z2 = height

    rings = []
    for (sx, sy), z in ((b, z0), (shoulder, z1), (top, z2)):
        rings.append(
            [
                geom.add_vertex(-sx, -sy, z),
                geom.add_vertex(sx, -sy, z),
                geom.add_vertex(sx, sy, z),
                geom.add_vertex(-sx, sy, z),
            ]
        )

    # side walls
    for lower, upper in ((rings[0], rings[1]), (rings[1], rings[2])):
        for i in range(4):
            j = (i + 1) % 4
            geom.add_face(lower[i], lower[j], upper[j])
            geom.add_face(lower[i], upper[j], upper[i])

    # bottom and top caps
    geom.add_face(rings[0][0], rings[0][2], rings[0][1])
    geom.add_face(rings[0][0], rings[0][3], rings[0][2])
    geom.add_face(rings[2][0], rings[2][1], rings[2][2])
    geom.add_face(rings[2][0], rings[2][2], rings[2][3])
    return geom


def _add_box_to_mesh(
    geom: MeshGeometry,
    *,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
) -> None:
    cx, cy, cz = center
    sx, sy, sz = (size[0] * 0.5, size[1] * 0.5, size[2] * 0.5)
    verts = [
        geom.add_vertex(cx - sx, cy - sy, cz - sz),
        geom.add_vertex(cx + sx, cy - sy, cz - sz),
        geom.add_vertex(cx + sx, cy + sy, cz - sz),
        geom.add_vertex(cx - sx, cy + sy, cz - sz),
        geom.add_vertex(cx - sx, cy - sy, cz + sz),
        geom.add_vertex(cx + sx, cy - sy, cz + sz),
        geom.add_vertex(cx + sx, cy + sy, cz + sz),
        geom.add_vertex(cx - sx, cy + sy, cz + sz),
    ]
    for a, b, c, d in (
        (0, 1, 2, 3),
        (4, 7, 6, 5),
        (0, 4, 5, 1),
        (1, 5, 6, 2),
        (2, 6, 7, 3),
        (3, 7, 4, 0),
    ):
        geom.add_face(verts[a], verts[b], verts[c])
        geom.add_face(verts[a], verts[c], verts[d])


def _switch_socket_mesh() -> MeshGeometry:
    """Plate-mounted square guide with a central stem slot and hidden web."""
    geom = MeshGeometry()
    stem = 0.0056
    outer = 0.0108
    wall = (outer - stem) * 0.5
    height = 0.0062
    web = 0.0007

    # Hidden base web lives just under the plate surface to make the four
    # guide walls one supported casting without blocking the visible stem slot.
    _add_box_to_mesh(geom, center=(0.0, 0.0, web * 0.5), size=(outer, outer, web))
    zc = height * 0.5
    _add_box_to_mesh(geom, center=(-(stem + wall) * 0.5, 0.0, zc), size=(wall, outer, height))
    _add_box_to_mesh(geom, center=((stem + wall) * 0.5, 0.0, zc), size=(wall, outer, height))
    _add_box_to_mesh(geom, center=(0.0, -(stem + wall) * 0.5, zc), size=(outer, wall, height))
    _add_box_to_mesh(geom, center=(0.0, (stem + wall) * 0.5, zc), size=(outer, wall, height))
    return geom


def _case_body_mesh() -> MeshGeometry:
    # A shallow wedge with softened superellipse side sections.
    sections = [
        (-CASE_DEPTH * 0.50, 0.002, 0.024, CASE_WIDTH * 0.94),
        (-CASE_DEPTH * 0.45, 0.000, 0.027, CASE_WIDTH),
        (CASE_DEPTH * 0.30, 0.000, 0.034, CASE_WIDTH),
        (CASE_DEPTH * 0.47, 0.002, 0.037, CASE_WIDTH * 0.97),
        (CASE_DEPTH * 0.50, 0.004, 0.035, CASE_WIDTH * 0.92),
    ]
    return superellipse_side_loft(sections, exponents=4.2, segments=64, cap=True)


def _row_keys(row: int, y_units: float, specs: list[tuple[str, float]]) -> list[dict[str, object]]:
    keys: list[dict[str, object]] = []
    cursor = 0.0
    for col, (label, width_u) in enumerate(specs):
        keys.append(
            {
                "name": f"key_{row}_{col}",
                "joint": f"press_{row}_{col}",
                "label": label,
                "width_u": width_u,
                "x_units": cursor + width_u * 0.5,
                "y_units": y_units,
                "role": "main",
            }
        )
        cursor += width_u
    return keys


def _layout_specs() -> list[dict[str, object]]:
    """ANSI tenkeyless / 80% layout with exact unit widths and cluster gaps."""
    keys: list[dict[str, object]] = []

    # Main 61-key ANSI cluster.
    keys += _row_keys(
        0,
        0.0,
        [
            ("Ctrl", 1.25),
            ("Win", 1.25),
            ("Alt", 1.25),
            ("Space", 6.25),
            ("Alt", 1.25),
            ("Win", 1.25),
            ("Menu", 1.25),
            ("Ctrl", 1.25),
        ],
    )
    keys += _row_keys(
        1,
        1.0,
        [
            ("Shift", 2.25),
            ("Z", 1.0),
            ("X", 1.0),
            ("C", 1.0),
            ("V", 1.0),
            ("B", 1.0),
            ("N", 1.0),
            ("M", 1.0),
            (",", 1.0),
            (".", 1.0),
            ("/", 1.0),
            ("Shift", 2.75),
        ],
    )
    keys += _row_keys(
        2,
        2.0,
        [
            ("Caps", 1.75),
            ("A", 1.0),
            ("S", 1.0),
            ("D", 1.0),
            ("F", 1.0),
            ("G", 1.0),
            ("H", 1.0),
            ("J", 1.0),
            ("K", 1.0),
            ("L", 1.0),
            (";", 1.0),
            ("'", 1.0),
            ("Enter", 2.25),
        ],
    )
    keys += _row_keys(
        3,
        3.0,
        [
            ("Tab", 1.5),
            ("Q", 1.0),
            ("W", 1.0),
            ("E", 1.0),
            ("R", 1.0),
            ("T", 1.0),
            ("Y", 1.0),
            ("U", 1.0),
            ("I", 1.0),
            ("O", 1.0),
            ("P", 1.0),
            ("[", 1.0),
            ("]", 1.0),
            ("\\", 1.5),
        ],
    )
    keys += _row_keys(
        4,
        4.0,
        [
            ("`", 1.0),
            ("1", 1.0),
            ("2", 1.0),
            ("3", 1.0),
            ("4", 1.0),
            ("5", 1.0),
            ("6", 1.0),
            ("7", 1.0),
            ("8", 1.0),
            ("9", 1.0),
            ("0", 1.0),
            ("-", 1.0),
            ("=", 1.0),
            ("Backspace", 2.0),
        ],
    )

    # Function row with real TKL group gaps.
    function_specs = [
        ("Esc", 0.5),
        ("F1", 2.5),
        ("F2", 3.5),
        ("F3", 4.5),
        ("F4", 5.5),
        ("F5", 6.75),
        ("F6", 7.75),
        ("F7", 8.75),
        ("F8", 9.75),
        ("F9", 11.0),
        ("F10", 12.0),
        ("F11", 13.0),
        ("F12", 14.0),
        ("Prt", 16.25),
        ("Scr", 17.25),
        ("Pause", 18.25),
    ]
    for col, (label, x_units) in enumerate(function_specs):
        keys.append(
            {
                "name": f"key_5_{col}",
                "joint": f"press_5_{col}",
                "label": label,
                "width_u": 1.0,
                "x_units": x_units,
                "y_units": 5.25,
                "role": "function",
            }
        )

    # Six-key navigation island and arrow cluster.
    nav_specs = [
        ("Ins", 16.25, 4.0),
        ("Home", 17.25, 4.0),
        ("PgUp", 18.25, 4.0),
        ("Del", 16.25, 3.0),
        ("End", 17.25, 3.0),
        ("PgDn", 18.25, 3.0),
        ("Up", 17.25, 1.0),
        ("Left", 16.25, 0.0),
        ("Down", 17.25, 0.0),
        ("Right", 18.25, 0.0),
    ]
    for col, (label, x_units, y_units) in enumerate(nav_specs):
        keys.append(
            {
                "name": f"key_6_{col}",
                "joint": f"press_6_{col}",
                "label": label,
                "width_u": 1.0,
                "x_units": x_units,
                "y_units": y_units,
                "role": "navigation",
            }
        )

    return keys


def _xy_from_units(x_units: float, y_units: float) -> tuple[float, float]:
    return ((x_units - X_UNITS_TOTAL * 0.5) * U, (y_units - Y_UNITS_MID) * U)


def _legend_size(label: str, key_width: float) -> tuple[float, float]:
    if label == "Space":
        return (key_width * 0.50, 0.0012)
    if len(label) >= 5:
        return (min(key_width * 0.46, 0.013), 0.0012)
    if len(label) >= 3:
        return (min(key_width * 0.36, 0.009), 0.0012)
    return (0.0045, 0.0011)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_80_percent_mechanical_keyboard")
    model.meta["layout"] = "ANSI tenkeyless 80 percent"

    matte_black = Material("anodized_matte_black", rgba=(0.015, 0.017, 0.020, 1.0))
    plate_mat = Material("graphite_switch_plate", rgba=(0.055, 0.060, 0.068, 1.0))
    key_mat = Material("charcoal_pbt_keycaps", rgba=(0.085, 0.090, 0.100, 1.0))
    mod_key_mat = Material("dark_graphite_modifiers", rgba=(0.060, 0.065, 0.075, 1.0))
    accent_mat = Material("muted_blue_accent_key", rgba=(0.030, 0.170, 0.290, 1.0))
    legend_mat = Material("soft_white_legends", rgba=(0.86, 0.88, 0.86, 1.0))
    switch_mat = Material("cream_switch_stems", rgba=(0.86, 0.78, 0.62, 1.0))
    rubber_mat = Material("black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    metal_mat = Material("brushed_usb_shell", rgba=(0.62, 0.64, 0.66, 1.0))
    port_dark = Material("usb_port_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_geometry(_case_body_mesh(), "rounded_wedge_case"),
        origin=Origin(),
        material=matte_black,
        name="rounded_wedge_case",
    )
    case.visual(
        Box((CASE_WIDTH - 0.026, CASE_DEPTH - 0.026, 0.004)),
        origin=Origin(xyz=(0.0, -0.003, PLATE_TOP_Z - 0.002)),
        material=plate_mat,
        name="switch_plate",
    )
    # Rear USB-C port: metal sleeve, black opening, and center tongue.
    rear_y = CASE_DEPTH * 0.5 + 0.0008
    case.visual(
        Box((0.018, 0.0016, 0.0080)),
        origin=Origin(xyz=(0.0, rear_y, 0.0255)),
        material=metal_mat,
        name="usb_c_shell",
    )
    case.visual(
        Box((0.0135, 0.0020, 0.0052)),
        origin=Origin(xyz=(0.0, rear_y + 0.0002, 0.0255)),
        material=port_dark,
        name="usb_c_opening",
    )
    case.visual(
        Box((0.0082, 0.0024, 0.0013)),
        origin=Origin(xyz=(0.0, rear_y + 0.00045, 0.0255)),
        material=plate_mat,
        name="usb_c_tongue",
    )
    # Low rubber feet and visible top screws keep the body grounded and detailed.
    for i, (x, y) in enumerate(
        [
            (-CASE_WIDTH * 0.30, -CASE_DEPTH * 0.30),
            (CASE_WIDTH * 0.30, -CASE_DEPTH * 0.30),
            (-CASE_WIDTH * 0.30, CASE_DEPTH * 0.30),
            (CASE_WIDTH * 0.30, CASE_DEPTH * 0.30),
        ]
    ):
        case.visual(
            Box((0.035, 0.012, 0.007)),
            origin=Origin(xyz=(x, y, 0.0015)),
            material=rubber_mat,
            name=f"rubber_foot_{i}",
        )
    for i, (x, y) in enumerate(
        [
            (-CASE_WIDTH * 0.43, -CASE_DEPTH * 0.42),
            (CASE_WIDTH * 0.43, -CASE_DEPTH * 0.42),
            (-CASE_WIDTH * 0.43, CASE_DEPTH * 0.39),
            (CASE_WIDTH * 0.43, CASE_DEPTH * 0.39),
        ]
    ):
        case.visual(
            Cylinder(0.0021, 0.0009),
            origin=Origin(xyz=(x, y, PLATE_TOP_Z + 0.00045)),
            material=metal_mat,
            name=f"plate_screw_{i}",
        )

    key_meshes: dict[float, object] = {}
    socket_mesh = mesh_from_geometry(_switch_socket_mesh(), "switch_socket_guide")
    layout = _layout_specs()
    model.meta["key_count"] = len(layout)
    model.meta["unit_pitch_m"] = U

    for spec in layout:
        name = str(spec["name"])
        label = str(spec["label"])
        width_u = float(spec["width_u"])
        x, y = _xy_from_units(float(spec["x_units"]), float(spec["y_units"]))
        key_width = width_u * U - KEY_GAP
        key_depth = U - KEY_GAP
        cap_h = 0.0087 if width_u < 6.0 else 0.0076
        mesh_key = round(width_u, 2)
        if mesh_key not in key_meshes:
            key_meshes[mesh_key] = mesh_from_geometry(
                _keycap_mesh(key_width, key_depth, cap_h),
                f"keycap_{str(mesh_key).replace('.', '_')}u",
            )

        case.visual(
            socket_mesh,
            origin=Origin(xyz=(x, y, PLATE_TOP_Z - 0.0007)),
            material=plate_mat,
            name=f"socket_{name}",
        )

        key = model.part(
            name,
            meta={
                "label": label,
                "width_u": width_u,
                "x_units": spec["x_units"],
                "y_units": spec["y_units"],
            },
        )
        key_material = key_mat
        if width_u > 1.0 or label in {"Ctrl", "Alt", "Win", "Menu", "Tab", "Caps", "Shift", "Backspace"}:
            key_material = mod_key_mat
        if label in {"Esc", "Enter"}:
            key_material = accent_mat

        key.visual(
            Box((0.0056, 0.0056, 0.0072)),
            origin=Origin(xyz=(0.0, 0.0, 0.0036)),
            material=switch_mat,
            name="switch_stem",
        )
        key.visual(
            key_meshes[mesh_key],
            origin=Origin(xyz=(0.0, 0.0, 0.0067)),
            material=key_material,
            name="keycap",
        )
        lx, ly = _legend_size(label, key_width)
        key.visual(
            Box((lx, ly, 0.00045)),
            origin=Origin(xyz=(0.0, -key_depth * 0.16, 0.0067 + cap_h + 0.00015)),
            material=legend_mat,
            name="legend_mark",
        )

        joint_name = str(spec["joint"])
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=case,
            child=key,
            origin=Origin(xyz=(x, y, SWITCH_BASE_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.5, velocity=0.35, lower=0.0, upper=SWITCH_TRAVEL),
            meta={"label": label, "travel_mm": SWITCH_TRAVEL * 1000.0},
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    key_parts = [p for p in object_model.parts if p.name.startswith("key_")]
    key_joints = [j for j in object_model.articulations if j.name.startswith("press_")]
    ctx.check("80 percent layout has 87 movable keys", len(key_parts) == 87, details=f"keys={len(key_parts)}")
    ctx.check(
        "every key uses a prismatic switch joint",
        len(key_joints) == 87
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in key_joints),
        details=f"joints={len(key_joints)}",
    )
    ctx.check(
        "all switches travel downward four millimeters",
        all(
            j.axis == (0.0, 0.0, -1.0)
            and j.motion_limits is not None
            and abs((j.motion_limits.upper or 0.0) - SWITCH_TRAVEL) < 1e-9
            for j in key_joints
        ),
    )

    case = object_model.get_part("case")
    space = next(p for p in key_parts if p.meta.get("label") == "Space")
    enter = next(p for p in key_parts if p.meta.get("label") == "Enter")
    esc = next(p for p in key_parts if p.meta.get("label") == "Esc")
    left_arrow = next(p for p in key_parts if p.meta.get("label") == "Left")
    down_arrow = next(p for p in key_parts if p.meta.get("label") == "Down")
    right_arrow = next(p for p in key_parts if p.meta.get("label") == "Right")
    home = next(p for p in key_parts if p.meta.get("label") == "Home")

    ctx.expect_gap(
        space,
        case,
        axis="z",
        min_gap=0.0001,
        positive_elem="keycap",
        name="spacebar keycap clears switch plate",
    )
    ctx.expect_overlap(space, case, axes="xy", min_overlap=0.015, name="spacebar mounted over case")
    ctx.expect_gap(
        esc,
        case,
        axis="z",
        min_gap=0.0001,
        positive_elem="keycap",
        name="escape keycap clears case",
    )
    ctx.expect_origin_gap(home, enter, axis="x", min_gap=0.035, name="navigation island separated from main keys")
    ctx.expect_origin_gap(right_arrow, left_arrow, axis="x", min_gap=2.0 * U - 0.001, max_gap=2.0 * U + 0.001)
    ctx.expect_origin_gap(down_arrow, left_arrow, axis="x", min_gap=U - 0.001, max_gap=U + 0.001)

    space_joint = object_model.get_articulation("press_0_3")
    rest_pos = ctx.part_world_position(space)
    with ctx.pose({space_joint: SWITCH_TRAVEL}):
        pressed_pos = ctx.part_world_position(space)
    ctx.check(
        "space switch presses downward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.0035,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
