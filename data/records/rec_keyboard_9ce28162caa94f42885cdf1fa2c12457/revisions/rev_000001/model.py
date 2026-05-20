from __future__ import annotations

from collections import defaultdict

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


U = 0.01905
CASE_W = 0.360
CASE_D = 0.150
CASE_H = 0.010
PLATE_W = 0.340
PLATE_D = 0.118
PLATE_T = 0.002
PLATE_Z = CASE_H + PLATE_T / 2.0
JOINT_Z = 0.016
KEY_D = 0.0174
KEY_H = 0.008
KEY_BOTTOM = 0.004
KEY_TOP = KEY_BOTTOM + KEY_H
KEY_GAP = 0.0015
KEY_TRAVEL = 0.0035


def _safe_key_id(label: str) -> str:
    names = {
        "`": "grave",
        "-": "minus",
        "=": "equal",
        "[": "lbracket",
        "]": "rbracket",
        "\\": "backslash",
        ";": "semicolon",
        "'": "quote",
        ",": "comma",
        ".": "period",
        "/": "slash",
        "←": "arrow_left",
        "↓": "arrow_down",
        "→": "arrow_right",
    }
    return names.get(label, label.lower().replace(" ", "_"))


def _layout_keys() -> list[dict[str, object]]:
    rows = [
        (
            0.050,
            [
                ("Esc", 1.0),
                (None, 0.50),
                ("F1", 1.0),
                ("F2", 1.0),
                ("F3", 1.0),
                ("F4", 1.0),
                (None, 0.25),
                ("F5", 1.0),
                ("F6", 1.0),
                ("F7", 1.0),
                ("F8", 1.0),
                (None, 0.25),
                ("F9", 1.0),
                ("F10", 1.0),
                ("F11", 1.0),
                ("F12", 1.0),
                (None, 0.50),
                ("Del", 1.0),
            ],
        ),
        (
            0.027,
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
        ),
        (
            0.006,
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
        ),
        (
            -0.015,
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
        ),
        (
            -0.036,
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
        ),
        (
            -0.057,
            [
                ("Ctrl", 1.25),
                ("Win", 1.25),
                ("Alt", 1.25),
                ("Space", 6.25),
                ("Alt", 1.25),
                ("Fn", 1.25),
                ("Menu", 1.25),
                ("Ctrl", 1.25),
                (None, 0.25),
                ("←", 1.0),
                ("↓", 1.0),
                ("→", 1.0),
            ],
        ),
    ]

    counts: defaultdict[str, int] = defaultdict(int)
    keys: list[dict[str, object]] = []
    for row_index, (y, items) in enumerate(rows):
        total_units = sum(units for _, units in items)
        x = -0.5 * total_units * U
        for label, units in items:
            if label is None:
                x += units * U
                continue
            base_id = _safe_key_id(label)
            idx = counts[base_id]
            counts[base_id] += 1
            key_id = base_id if idx == 0 else f"{base_id}_{idx}"
            x_center = x + 0.5 * units * U
            keys.append(
                {
                    "label": label,
                    "id": key_id,
                    "units": float(units),
                    "x": x_center,
                    "y": y,
                    "row": row_index,
                }
            )
            x += units * U
    return keys


def _case_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(CASE_W, CASE_D, CASE_H)
        .translate((0.0, 0.0, CASE_H / 2.0))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .chamfer(0.002)
    )


def _top_plate_shape(keys: list[dict[str, object]]) -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_W, PLATE_D, PLATE_T).translate((0.0, 0.0, PLATE_Z))
    for key in keys:
        x = float(key["x"])
        y = float(key["y"])
        units = float(key["units"])
        stem_offsets = [0.0] if units < 2.0 else [-0.36 * (units * U), 0.36 * (units * U)]
        for dx in stem_offsets:
            socket = (
                cq.Workplane("XY")
                .box(0.0115, 0.0115, 0.003)
                .translate((x + dx, y, CASE_H + PLATE_T + 0.0015))
            )
            plate = plate.union(socket)
    return plate


def _keycap_shape(width: float, depth: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, KEY_H)
        .translate((0.0, 0.0, KEY_BOTTOM + KEY_H / 2.0))
        .edges()
        .chamfer(0.0011)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="polished_mechanical_keyboard")

    mat_case = model.material("satin_graphite_aluminum", rgba=(0.075, 0.078, 0.086, 1.0))
    mat_plate = model.material("black_anodized_plate", rgba=(0.010, 0.012, 0.015, 1.0))
    mat_key = model.material("matte_charcoal_keycaps", rgba=(0.030, 0.032, 0.037, 1.0))
    mat_mod = model.material("warm_gray_modifiers", rgba=(0.120, 0.125, 0.135, 1.0))
    mat_accent = model.material("teal_escape_key", rgba=(0.000, 0.390, 0.430, 1.0))
    mat_legend = model.material("soft_white_legends", rgba=(0.86, 0.88, 0.88, 1.0))
    mat_rubber = model.material("matte_black_rubber", rgba=(0.006, 0.006, 0.007, 1.0))
    mat_led = model.material("frosted_green_led", rgba=(0.20, 0.95, 0.42, 1.0))
    mat_metal = model.material("brushed_port_metal", rgba=(0.55, 0.56, 0.58, 1.0))

    keys = _layout_keys()

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_case_shape(), "rounded_keyboard_case", tolerance=0.0006),
        material=mat_case,
        name="case_shell",
    )
    case.visual(
        mesh_from_cadquery(_top_plate_shape(keys), "perforated_switch_plate", tolerance=0.0007),
        material=mat_plate,
        name="top_plate",
    )
    case.visual(
        Box((CASE_W - 0.030, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 - 0.007, CASE_H + 0.007)),
        material=mat_case,
        name="rear_tilt_bar",
    )
    case.visual(
        Box((0.024, 0.0015, 0.006)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 + 0.0007, CASE_H + 0.007)),
        material=mat_metal,
        name="usb_c_sleeve",
    )
    case.visual(
        Box((0.017, 0.0018, 0.0032)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 + 0.0016, CASE_H + 0.007)),
        material=mat_plate,
        name="usb_c_shadow",
    )
    for i, x in enumerate((0.128, 0.137, 0.146)):
        case.visual(
            Cylinder(radius=0.0020, length=0.00045),
            origin=Origin(xyz=(x, 0.040, CASE_H + PLATE_T + 0.00015)),
            material=mat_led if i == 0 else mat_rubber,
            name=f"status_led_{i}",
        )
    for i, (x, y) in enumerate(((-0.145, -0.062), (0.145, -0.062), (-0.145, 0.060), (0.145, 0.060))):
        case.visual(
            Box((0.034, 0.010, 0.0020)),
            origin=Origin(xyz=(x, y, 0.0010)),
            material=mat_rubber,
            name=f"rubber_foot_{i}",
        )

    keycap_meshes: dict[float, object] = {}
    modifier_ids = {
        "tab",
        "caps",
        "shift",
        "shift_1",
        "ctrl",
        "ctrl_1",
        "win",
        "alt",
        "alt_1",
        "fn",
        "menu",
        "backspace",
        "enter",
        "del",
        "arrow_left",
        "arrow_down",
        "arrow_right",
    }
    for key in keys:
        key_id = str(key["id"])
        units = float(key["units"])
        cap_width = units * U - KEY_GAP
        if units not in keycap_meshes:
            name = f"keycap_{str(units).replace('.', 'p')}u"
            keycap_meshes[units] = mesh_from_cadquery(
                _keycap_shape(cap_width, KEY_D),
                name,
                tolerance=0.0004,
                angular_tolerance=0.08,
            )

        key_part = model.part(f"key_{key_id}")
        if key_id == "esc":
            cap_material = mat_accent
        elif key_id in modifier_ids or units > 1.0:
            cap_material = mat_mod
        else:
            cap_material = mat_key

        key_part.visual(
            keycap_meshes[units],
            material=cap_material,
            name="keycap",
        )

        stem_offsets = [0.0] if units < 2.0 else [-0.36 * (units * U), 0.36 * (units * U)]
        for stem_i, dx in enumerate(stem_offsets):
            key_part.visual(
                Box((0.0060, 0.0060, 0.0050)),
                origin=Origin(xyz=(dx, 0.0, 0.0015)),
                material=mat_rubber,
                name=f"stem_{stem_i}",
            )
        if units >= 2.0:
            key_part.visual(
                Box((cap_width * 0.72, 0.0022, 0.0020)),
                origin=Origin(xyz=(0.0, -0.0047, 0.0046)),
                material=mat_rubber,
                name="stabilizer_bar",
            )

        if key_id == "space":
            key_part.visual(
                Box((0.052, 0.0012, 0.00032)),
                origin=Origin(xyz=(0.0, 0.0022, KEY_TOP + 0.00014)),
                material=mat_legend,
                name="thumb_highlight",
            )
        else:
            legend_w = min(0.010, max(0.0055, cap_width * 0.34))
            legend_x = -0.0025 if units >= 1.5 else 0.0
            key_part.visual(
                Box((legend_w, 0.00105, 0.00030)),
                origin=Origin(xyz=(legend_x, 0.0022, KEY_TOP + 0.00013)),
                material=mat_legend,
                name="legend_bar",
            )
            if str(key["label"]).startswith("F") or str(key["label"]).isdigit():
                key_part.visual(
                    Box((legend_w * 0.55, 0.00085, 0.00028)),
                    origin=Origin(xyz=(legend_x, -0.0023, KEY_TOP + 0.00013)),
                    material=mat_legend,
                    name="legend_subbar",
                )

        model.articulation(
            f"case_to_key_{key_id}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key_part,
            origin=Origin(xyz=(float(key["x"]), float(key["y"]), JOINT_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.20, lower=0.0, upper=KEY_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    key_parts = [part for part in object_model.parts if part.name.startswith("key_")]
    key_joints = [joint for joint in object_model.articulations if joint.name.startswith("case_to_key_")]
    ctx.check(
        "keyboard has a dense consumer key layout",
        len(key_parts) >= 65 and len(key_joints) == len(key_parts),
        details=f"key_parts={len(key_parts)}, key_joints={len(key_joints)}",
    )

    case = object_model.get_part("case")
    key_a = object_model.get_part("key_a")
    key_space = object_model.get_part("key_space")
    key_enter = object_model.get_part("key_enter")

    ctx.expect_within(
        key_a,
        case,
        axes="xy",
        inner_elem="keycap",
        outer_elem="top_plate",
        margin=0.004,
        name="home-row key sits inside the switch plate footprint",
    )
    ctx.expect_gap(
        key_a,
        case,
        axis="z",
        min_gap=0.004,
        max_gap=0.007,
        positive_elem="keycap",
        negative_elem="top_plate",
        name="home-row key has realistic clearance over switch housings",
    )
    ctx.expect_gap(
        key_space,
        case,
        axis="z",
        min_gap=0.004,
        max_gap=0.007,
        positive_elem="keycap",
        negative_elem="top_plate",
        name="spacebar clears the raised switch housings",
    )
    ctx.expect_overlap(
        key_enter,
        case,
        axes="xy",
        elem_a="keycap",
        elem_b="top_plate",
        min_overlap=0.015,
        name="wide enter key is seated over the plate",
    )

    space_joint = object_model.get_articulation("case_to_key_space")
    a_joint = object_model.get_articulation("case_to_key_a")
    rest_space = ctx.part_world_position(key_space)
    with ctx.pose({space_joint: KEY_TRAVEL, a_joint: KEY_TRAVEL}):
        pressed_space = ctx.part_world_position(key_space)
        ctx.expect_gap(
            key_a,
            case,
            axis="z",
            min_gap=0.001,
            max_gap=0.007,
            positive_elem="keycap",
            negative_elem="top_plate",
            name="pressed key still clears the metal plate",
        )

    ctx.check(
        "spacebar press travels downward",
        rest_space is not None
        and pressed_space is not None
        and pressed_space[2] < rest_space[2] - 0.0030,
        details=f"rest={rest_space}, pressed={pressed_space}",
    )

    return ctx.report()


object_model = build_object_model()
