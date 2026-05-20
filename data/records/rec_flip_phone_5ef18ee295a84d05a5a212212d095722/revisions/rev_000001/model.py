from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


LID_OPEN_ANGLE = 1.95
LID_CLOSED_ANGLE = -LID_OPEN_ANGLE
KEY_TRAVEL = 0.003
BATTERY_TRAVEL = 0.035


def _rounded_slab(width: float, depth: float, height: float, radius: float):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, depth, radius, corner_segments=8),
            height,
        ),
        f"rounded_slab_{int(width * 1000)}_{int(depth * 1000)}_{int(height * 10000)}",
    )


def _rounded_centered_slab(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(width, depth, radius, corner_segments=8),
            height,
        ),
        name,
    )


def _rotate_x(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _lid_mount(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=_rotate_x((x, y, z), -LID_OPEN_ANGLE), rpy=(-LID_OPEN_ANGLE, 0.0, 0.0))


FONT_5X7 = {
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "B": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
    "C": ("01111", "10000", "10000", "10000", "10000", "10000", "01111"),
    "D": ("11110", "10001", "10001", "10001", "10001", "10001", "11110"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "F": ("11111", "10000", "10000", "11110", "10000", "10000", "10000"),
    "G": ("01111", "10000", "10000", "10111", "10001", "10001", "01111"),
    "H": ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    "I": ("11111", "00100", "00100", "00100", "00100", "00100", "11111"),
    "J": ("00111", "00010", "00010", "00010", "10010", "10010", "01100"),
    "K": ("10001", "10010", "10100", "11000", "10100", "10010", "10001"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "P": ("11110", "10001", "10001", "11110", "10000", "10000", "10000"),
    "Q": ("01110", "10001", "10001", "10001", "10101", "10010", "01101"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "U": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    "V": ("10001", "10001", "10001", "10001", "10001", "01010", "00100"),
    "W": ("10001", "10001", "10001", "10101", "10101", "10101", "01010"),
    "X": ("10001", "10001", "01010", "00100", "01010", "10001", "10001"),
    "Y": ("10001", "10001", "01010", "00100", "00100", "00100", "00100"),
    "Z": ("11111", "00001", "00010", "00100", "01000", "10000", "11111"),
}


def _add_pixel_label(part, label: str, *, key_width: float, key_depth: float, top_z: float, material) -> None:
    if len(label) == 1 and label.upper() in FONT_5X7:
        pattern = FONT_5X7[label.upper()]
        pixel = min(0.00082, key_width / 8.0, key_depth / 11.0)
        gap = pixel * 0.28
        pitch = pixel + gap
        label_width = 5 * pixel + 4 * gap
        label_height = 7 * pixel + 6 * gap
        for row_index, row in enumerate(pattern):
            for column_index, filled in enumerate(row):
                if filled != "1":
                    continue
                x = -label_width * 0.5 + pixel * 0.5 + column_index * pitch
                y = label_height * 0.5 - pixel * 0.5 - row_index * pitch
                part.visual(
                    Box((pixel, pixel, 0.00035)),
                    origin=Origin(xyz=(x, y, top_z + 0.000175)),
                    material=material,
                    name=f"legend_{label.lower()}_{row_index}_{column_index}",
                )
        return

    # Non-letter control keys get simple white bars/icons instead of text.
    if label == "space":
        part.visual(
            Box((key_width * 0.55, 0.0011, 0.00035)),
            origin=Origin(xyz=(0.0, 0.0, top_z + 0.000175)),
            material=material,
            name="legend_space",
        )
    elif label == "shift":
        part.visual(
            Box((key_width * 0.32, 0.0010, 0.00035)),
            origin=Origin(xyz=(0.0, 0.0016, top_z + 0.000175)),
            material=material,
            name="legend_shift_head",
        )
        part.visual(
            Box((0.0012, key_depth * 0.34, 0.00035)),
            origin=Origin(xyz=(0.0, -0.0004, top_z + 0.000175)),
            material=material,
            name="legend_shift_stem",
        )
    elif label == "bksp":
        part.visual(
            Box((key_width * 0.46, 0.0010, 0.00035)),
            origin=Origin(xyz=(0.0, 0.0, top_z + 0.000175)),
            material=material,
            name="legend_backspace_bar",
        )
        part.visual(
            Box((0.0010, key_depth * 0.32, 0.00035)),
            origin=Origin(xyz=(-key_width * 0.20, 0.0, top_z + 0.000175)),
            material=material,
            name="legend_backspace_stop",
        )
    elif label == "enter":
        part.visual(
            Box((key_width * 0.36, 0.0010, 0.00035)),
            origin=Origin(xyz=(0.0, -0.0012, top_z + 0.000175)),
            material=material,
            name="legend_enter_bar",
        )
        part.visual(
            Box((0.0010, key_depth * 0.34, 0.00035)),
            origin=Origin(xyz=(key_width * 0.16, 0.0005, top_z + 0.000175)),
            material=material,
            name="legend_enter_stem",
        )
    else:
        part.visual(
            Box((key_width * 0.36, 0.0010, 0.00035)),
            origin=Origin(xyz=(0.0, 0.0, top_z + 0.000175)),
            material=material,
            name=f"legend_{label}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_flip_communicator")

    shell_mat = model.material("warm_graphite_shell", rgba=(0.12, 0.13, 0.145, 1.0))
    edge_mat = model.material("soft_black_edges", rgba=(0.035, 0.038, 0.045, 1.0))
    deck_mat = model.material("recessed_key_deck", rgba=(0.055, 0.060, 0.068, 1.0))
    key_mat = model.material("matte_rubber_keys", rgba=(0.020, 0.022, 0.026, 1.0))
    legend_mat = model.material("silk_screen_white", rgba=(0.88, 0.90, 0.86, 1.0))
    glass_mat = model.material("blue_black_glass", rgba=(0.02, 0.07, 0.10, 0.68))
    lcd_mat = model.material("pale_lcd_green", rgba=(0.45, 0.70, 0.63, 0.92))
    hinge_mat = model.material("dark_hinge_metal", rgba=(0.055, 0.055, 0.060, 1.0))
    rail_mat = model.material("battery_track_black", rgba=(0.025, 0.027, 0.032, 1.0))

    lower_width = 0.185
    lower_depth = 0.125
    lower_height = 0.018
    deck_top = 0.020
    hinge_y = lower_depth * 0.5 + 0.0045
    hinge_z = 0.034

    lower = model.part("lower_body")
    lower.visual(
        _rounded_slab(lower_width, lower_depth, lower_height, 0.011),
        origin=Origin(),
        material=shell_mat,
        name="lower_shell",
    )
    lower.visual(
        _rounded_slab(0.170, 0.091, 0.002, 0.007),
        origin=Origin(xyz=(0.0, -0.017, lower_height)),
        material=deck_mat,
        name="keypad_deck",
    )
    lower.visual(
        Box((0.176, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, lower_depth * 0.5 - 0.005, lower_height + 0.003)),
        material=edge_mat,
        name="rear_lip",
    )
    lower.visual(
        Box((0.017, 0.012, 0.019)),
        origin=Origin(xyz=(-0.078, hinge_y - 0.003, 0.0275)),
        material=shell_mat,
        name="hinge_cheek_0",
    )
    lower.visual(
        Box((0.017, 0.012, 0.019)),
        origin=Origin(xyz=(0.078, hinge_y - 0.003, 0.0275)),
        material=shell_mat,
        name="hinge_cheek_1",
    )
    lower.visual(
        Cylinder(radius=0.0058, length=0.170),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="hinge_pin",
    )
    lower.visual(
        Box((0.004, 0.072, 0.003)),
        origin=Origin(xyz=(-0.036, -0.006, -0.0015)),
        material=rail_mat,
        name="battery_rail_0",
    )
    lower.visual(
        Box((0.004, 0.072, 0.003)),
        origin=Origin(xyz=(0.036, -0.006, -0.0015)),
        material=rail_mat,
        name="battery_rail_1",
    )

    lid_width = 0.178
    lid_depth = 0.122
    lid_thickness = 0.014
    lid_root_gap = 0.010
    lid_center_y = -lid_root_gap - lid_depth * 0.5
    lid = model.part("upper_lid")
    lid.visual(
        _rounded_centered_slab(lid_width, lid_depth, lid_thickness, 0.011, "upper_lid_shell"),
        origin=_lid_mount(0.0, lid_center_y, 0.0),
        material=shell_mat,
        name="lid_shell",
    )
    lid.visual(
        Box((0.134, 0.078, 0.0012)),
        origin=_lid_mount(0.0, lid_center_y - 0.006, -lid_thickness * 0.5 - 0.0006),
        material=edge_mat,
        name="display_bezel",
    )
    lid.visual(
        Box((0.121, 0.064, 0.0014)),
        origin=_lid_mount(0.0, lid_center_y - 0.006, -lid_thickness * 0.5 - 0.0015),
        material=glass_mat,
        name="large_display",
    )
    for slot_index, slot_x in enumerate((-0.018, -0.009, 0.0, 0.009, 0.018)):
        lid.visual(
            Box((0.0055, 0.0013, 0.0010)),
            origin=_lid_mount(slot_x, lid_center_y + 0.045, -lid_thickness * 0.5 - 0.0005),
            material=edge_mat,
            name=f"earpiece_slot_{slot_index}",
        )
    lid.visual(
        Box((0.068, 0.030, 0.0012)),
        origin=_lid_mount(0.0, lid_center_y + 0.006, lid_thickness * 0.5 + 0.0006),
        material=edge_mat,
        name="outer_display_bezel",
    )
    lid.visual(
        Box((0.055, 0.019, 0.0014)),
        origin=_lid_mount(0.0, lid_center_y + 0.006, lid_thickness * 0.5 + 0.0015),
        material=lcd_mat,
        name="external_display",
    )
    lid.visual(
        Cylinder(radius=0.0074, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="hinge_sleeve",
    )
    lid.visual(
        Box((0.122, 0.010, 0.004)),
        origin=_lid_mount(0.0, -0.007, -0.006),
        material=shell_mat,
        name="hinge_strap",
    )

    lid_joint = model.articulation(
        "lower_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.5, lower=LID_CLOSED_ANGLE, upper=0.0),
    )
    lid_joint.meta["qc_samples"] = [LID_CLOSED_ANGLE, -1.0, 0.0]

    key_mesh_cache: dict[tuple[int, int], object] = {}

    def key_mesh(width: float, depth: float):
        key = (int(width * 10000), int(depth * 10000))
        if key not in key_mesh_cache:
            key_mesh_cache[key] = mesh_from_geometry(
                ExtrudeGeometry.from_z0(
                    rounded_rect_profile(width, depth, min(width, depth) * 0.22, corner_segments=5),
                    0.0042,
                ),
                f"keycap_{key[0]}_{key[1]}",
            )
        return key_mesh_cache[key]

    def add_key(name: str, label: str, x: float, y: float, width: float, depth: float = 0.0138) -> None:
        key_part = model.part(name)
        key_part.visual(
            key_mesh(width, depth),
            origin=Origin(),
            material=key_mat,
            name="keycap",
        )
        _add_pixel_label(
            key_part,
            label,
            key_width=width,
            key_depth=depth,
            top_z=0.0042,
            material=legend_mat,
        )
        joint = model.articulation(
            f"lower_to_{name}",
            ArticulationType.PRISMATIC,
            parent=lower,
            child=key_part,
            origin=Origin(xyz=(x, y, deck_top)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.08, lower=0.0, upper=KEY_TRAVEL),
        )
        joint.meta["qc_samples"] = [0.0, KEY_TRAVEL]

    pitch = 0.0157
    for index, letter in enumerate("QWERTYUIOP"):
        add_key(f"key_{letter.lower()}", letter, (index - 4.5) * pitch, 0.0195, 0.0128)
    for index, letter in enumerate("ASDFGHJKL"):
        add_key(f"key_{letter.lower()}", letter, (index - 4.0) * pitch + 0.0035, 0.0015, 0.0128)
    third_row = [
        ("key_shift", "shift", 0.0180),
        ("key_z", "Z", 0.0128),
        ("key_x", "X", 0.0128),
        ("key_c", "C", 0.0128),
        ("key_v", "V", 0.0128),
        ("key_b", "B", 0.0128),
        ("key_n", "N", 0.0128),
        ("key_m", "M", 0.0128),
        ("key_bksp", "bksp", 0.0180),
    ]
    row_width = sum(width for _, _, width in third_row) + (len(third_row) - 1) * 0.0028
    cursor = -row_width * 0.5
    for name, label, width in third_row:
        add_key(name, label, cursor + width * 0.5, -0.0165, width)
        cursor += width + 0.0028
    bottom_row = [
        ("key_alt", "alt", 0.017),
        ("key_sym", "sym", 0.017),
        ("key_space", "space", 0.053),
        ("key_dot", ".", 0.017),
        ("key_enter", "enter", 0.022),
    ]
    row_width = sum(width for _, _, width in bottom_row) + (len(bottom_row) - 1) * 0.0040
    cursor = -row_width * 0.5
    for name, label, width in bottom_row:
        add_key(name, label, cursor + width * 0.5, -0.0365, width)
        cursor += width + 0.0040

    battery = model.part("battery_door")
    battery.visual(
        _rounded_slab(0.060, 0.061, 0.0020, 0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.0020)),
        material=shell_mat,
        name="door_panel",
    )
    battery.visual(
        Box((0.032, 0.0022, 0.0010)),
        origin=Origin(xyz=(0.0, -0.012, -0.0025)),
        material=rail_mat,
        name="finger_rib_0",
    )
    battery.visual(
        Box((0.032, 0.0022, 0.0010)),
        origin=Origin(xyz=(0.0, -0.006, -0.0025)),
        material=rail_mat,
        name="finger_rib_1",
    )
    model.articulation(
        "lower_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=battery,
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=BATTERY_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_body")
    lid = object_model.get_part("upper_lid")
    battery = object_model.get_part("battery_door")
    hinge = object_model.get_articulation("lower_to_lid")
    battery_slide = object_model.get_articulation("lower_to_battery_door")
    key_q = object_model.get_part("key_q")
    key_q_slide = object_model.get_articulation("lower_to_key_q")

    ctx.allow_overlap(
        lower,
        lid,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The lid sleeve is intentionally shown wrapped around the fixed hinge pin.",
    )
    ctx.expect_overlap(
        lower,
        lid,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.06,
        name="hinge sleeve captures the horizontal pin",
    )

    open_aabb = ctx.part_world_aabb(lid)
    lower_aabb = ctx.part_world_aabb(lower)
    ctx.check(
        "lid is shown open above the keypad at rest",
        open_aabb is not None
        and lower_aabb is not None
        and open_aabb[1][2] > lower_aabb[1][2] + 0.070,
        details=f"lid_aabb={open_aabb}, lower_aabb={lower_aabb}",
    )

    with ctx.pose({hinge: LID_CLOSED_ANGLE}):
        ctx.expect_overlap(
            lid,
            lower,
            axes="xy",
            elem_a="lid_shell",
            elem_b="lower_shell",
            min_overlap=0.080,
            name="closed lid covers the square lower body",
        )
        ctx.expect_gap(
            lid,
            lower,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="lower_shell",
            min_gap=0.006,
            max_gap=0.014,
            name="closed lid clears the keypad stack",
        )

    ctx.expect_gap(
        lower,
        battery,
        axis="z",
        positive_elem="lower_shell",
        negative_elem="door_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="battery door is seated on the rear face",
    )
    rest_battery = ctx.part_world_position(battery)
    with ctx.pose({battery_slide: BATTERY_TRAVEL}):
        slid_battery = ctx.part_world_position(battery)
    ctx.check(
        "battery door slides off along the rear",
        rest_battery is not None
        and slid_battery is not None
        and slid_battery[1] < rest_battery[1] - 0.025,
        details=f"rest={rest_battery}, slid={slid_battery}",
    )

    rest_key = ctx.part_world_position(key_q)
    with ctx.pose({key_q_slide: KEY_TRAVEL}):
        pressed_key = ctx.part_world_position(key_q)
    ctx.check(
        "QWERTY keys press downward",
        rest_key is not None and pressed_key is not None and pressed_key[2] < rest_key[2] - 0.002,
        details=f"rest={rest_key}, pressed={pressed_key}",
    )

    return ctx.report()


object_model = build_object_model()
