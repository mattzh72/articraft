from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


U = 0.01905  # one keyboard unit: 19.05 mm switch pitch
CAP_GAP = 0.0022
CASE_HEIGHT = 0.024
PLATE_TOP = 0.027
CAP_BOTTOM = 0.014
CAP_HEIGHT = 0.0115


def _keycap_shape(width_u: float) -> cq.Workplane:
    """A low-profile, gently tapered keycap body with a flat top."""
    bottom_w = width_u * U - CAP_GAP
    bottom_d = U - CAP_GAP
    top_w = max(bottom_w - 0.0055, bottom_w * 0.86)
    top_d = bottom_d - 0.0046
    return (
        cq.Workplane("XY")
        .rect(bottom_w, bottom_d)
        .workplane(offset=CAP_HEIGHT)
        .rect(top_w, top_d)
        .loft(combine=True)
    )


def _case_shape(width: float, depth: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, CASE_HEIGHT)
        .edges("|Z")
        .fillet(0.006)
        .edges(">Z")
        .fillet(0.002)
    )


def _switch_frame_shape() -> cq.Workplane:
    """Cherry-MX-like top housing with a square center opening for the slider."""
    frame = cq.Workplane("XY").box(0.0140, 0.0140, 0.0100)
    bore = cq.Workplane("XY").box(0.0072, 0.0072, 0.0120)
    return frame.cut(bore)


def _layout():
    """ANSI 80% / tenkeyless key layout in keyboard units."""
    keys = []

    def add(name: str, x: float, y: float, w: float = 1.0):
        keys.append({"name": name, "x": x, "y": y, "w": w})

    # Function row with the characteristic cluster gaps.
    add("esc_key", 0.0, 5.5)
    for i in range(1, 13):
        cluster_gap = 0.0 if i <= 4 else (0.5 if i <= 8 else 1.0)
        add(f"f{i}_key", 1.5 + i + cluster_gap, 5.5)
    add("print_key", 15.5, 5.5)
    add("scroll_key", 16.5, 5.5)
    add("pause_key", 17.5, 5.5)

    # Main typing block.
    for x, name in enumerate(
        ["grave_key", "key_1", "key_2", "key_3", "key_4", "key_5", "key_6", "key_7", "key_8", "key_9", "key_0", "minus_key", "equal_key"]
    ):
        add(name, float(x), 4.0)
    add("backspace_key", 13.75, 4.0, 2.5)

    add("tab_key", 0.25, 3.0, 1.5)
    for i, letter in enumerate("qwertyuiop"):
        add(f"key_{letter}", 2.0 + i, 3.0)
    add("bracket_key", 12.0, 3.0)
    add("brace_key", 13.0, 3.0)
    add("backslash_key", 14.25, 3.0, 1.5)

    add("caps_key", 0.375, 2.0, 1.75)
    for i, letter in enumerate("asdfghjkl"):
        add(f"key_{letter}", 2.125 + i, 2.0)
    add("semicolon_key", 11.125, 2.0)
    add("quote_key", 12.125, 2.0)
    add("enter_key", 13.875, 2.0, 2.25)

    add("shift_key_0", 0.625, 1.0, 2.25)
    for i, letter in enumerate("zxcvbnm"):
        add(f"key_{letter}", 2.75 + i, 1.0)
    add("comma_key", 9.75, 1.0)
    add("period_key", 10.75, 1.0)
    add("slash_key", 11.75, 1.0)
    add("shift_key_1", 13.625, 1.0, 2.75)

    bottom = [
        ("ctrl_key_0", 0.125, 1.25),
        ("win_key", 1.375, 1.25),
        ("alt_key_0", 2.625, 1.25),
        ("space_key", 6.375, 6.25),
        ("alt_key_1", 10.125, 1.25),
        ("fn_key", 11.375, 1.25),
        ("menu_key", 12.625, 1.25),
        ("ctrl_key_1", 13.875, 1.25),
    ]
    for name, x, w in bottom:
        add(name, x, 0.0, w)

    # Navigation and arrow islands.
    for i, name in enumerate(["insert_key", "home_key", "page_key_0"]):
        add(name, 15.5 + i, 4.0)
    for i, name in enumerate(["delete_key", "end_key", "page_key_1"]):
        add(name, 15.5 + i, 3.0)
    add("arrow_key_0", 16.5, 1.0)
    add("arrow_key_1", 15.5, 0.0)
    add("arrow_key_2", 16.5, 0.0)
    add("arrow_key_3", 17.5, 0.0)

    return keys


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aluminium_tkl_keyboard")

    aluminium = Material("brushed_aluminium", rgba=(0.70, 0.72, 0.73, 1.0))
    plate_black = Material("anodized_black_plate", rgba=(0.025, 0.027, 0.030, 1.0))
    switch_black = Material("black_switch_housings", rgba=(0.015, 0.014, 0.013, 1.0))
    switch_stem = Material("dark_red_switch_stems", rgba=(0.42, 0.025, 0.025, 1.0))
    key_red = Material("deep_red_keycaps", rgba=(0.82, 0.035, 0.030, 1.0))
    legend_white = Material("warm_white_legends", rgba=(0.94, 0.89, 0.78, 1.0))
    screw_dark = Material("dark_screw_heads", rgba=(0.055, 0.058, 0.060, 1.0))

    keys = _layout()
    min_x = min(k["x"] - k["w"] / 2.0 for k in keys)
    max_x = max(k["x"] + k["w"] / 2.0 for k in keys)
    min_y = min(k["y"] - 0.5 for k in keys)
    max_y = max(k["y"] + 0.5 for k in keys)
    layout_cx = (min_x + max_x) / 2.0
    layout_cy = (min_y + max_y) / 2.0
    span_x = (max_x - min_x) * U
    span_y = (max_y - min_y) * U
    case_w = span_x + 0.027
    case_d = span_y + 0.027

    cap_meshes = {
        w: mesh_from_cadquery(_keycap_shape(w), f"red_keycap_{str(w).replace('.', '_')}u", tolerance=0.00055)
        for w in sorted({k["w"] for k in keys})
    }
    switch_mesh = mesh_from_cadquery(_switch_frame_shape(), "mx_switch_housing_frame", tolerance=0.00045)

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_case_shape(case_w, case_d), "beveled_aluminium_case", tolerance=0.0007),
        origin=Origin(xyz=(0.0, 0.0, CASE_HEIGHT / 2.0)),
        material=aluminium,
        name="aluminium_case",
    )
    case.visual(
        Box((case_w - 0.028, case_d - 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_TOP - 0.002)),
        material=plate_black,
        name="recessed_switch_plate",
    )

    # Bezel screws seated into the top plate.
    for i, (sx, sy) in enumerate(
        [
            (-case_w / 2 + 0.025, -case_d / 2 + 0.018),
            (case_w / 2 - 0.025, -case_d / 2 + 0.018),
            (-case_w / 2 + 0.025, case_d / 2 - 0.018),
            (case_w / 2 - 0.025, case_d / 2 - 0.018),
            (0.0, case_d / 2 - 0.018),
        ]
    ):
        case.visual(
            Cylinder(radius=0.0022, length=0.0012),
            origin=Origin(xyz=(sx, sy, PLATE_TOP + 0.0002)),
            material=screw_dark,
            name=f"screw_{i}",
        )

    switch_z = PLATE_TOP + 0.0048

    for k in keys:
        x = (k["x"] - layout_cx) * U
        y = (k["y"] - layout_cy) * U
        w = k["w"]
        name = k["name"]

        # Fixed switch housing: every key has a centered Cherry-style switch body
        # on the 19.05 mm grid. It slightly nests into the plate so the housing
        # reads as clipped into the aluminium deck rather than floating.
        switch_visual_name = "switch_space_key" if name == "space_key" else f"switch_{name}"
        case.visual(
            switch_mesh,
            origin=Origin(xyz=(x, y, switch_z)),
            material=switch_black,
            name=switch_visual_name,
        )
        if w >= 2.0:
            stab_offset = min((w * U) * 0.28, 0.038)
            for side in (-1.0, 1.0):
                case.visual(
                    Box((0.0070, 0.0038, 0.0048)),
                    origin=Origin(xyz=(x + side * stab_offset, y, PLATE_TOP + 0.0020)),
                    material=switch_black,
                    name=f"stabilizer_{name}_{'a' if side < 0 else 'b'}",
                )

        key = model.part(name)
        key.visual(
            cap_meshes[w],
            origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM)),
            material=key_red,
            name="cap",
        )
        key.visual(
            Box((0.0059, 0.0059, 0.0115)),
            origin=Origin(xyz=(0.0, 0.0, 0.01125)),
            material=switch_stem,
            name="stem",
        )
        key.visual(
            Box((0.00065, 0.0022, 0.0105)),
            origin=Origin(xyz=(0.003275, 0.0, 0.01075)),
            material=switch_stem,
            name="guide_x_pos",
        )
        key.visual(
            Box((0.00065, 0.0022, 0.0105)),
            origin=Origin(xyz=(-0.003275, 0.0, 0.01075)),
            material=switch_stem,
            name="guide_x_neg",
        )
        key.visual(
            Box((0.0022, 0.00065, 0.0105)),
            origin=Origin(xyz=(0.0, 0.003275, 0.01075)),
            material=switch_stem,
            name="guide_y_pos",
        )
        key.visual(
            Box((0.0022, 0.00065, 0.0105)),
            origin=Origin(xyz=(0.0, -0.003275, 0.01075)),
            material=switch_stem,
            name="guide_y_neg",
        )
        if w >= 2.0:
            stab_offset = min((w * U) * 0.28, 0.038)
            for side in (-1.0, 1.0):
                key.visual(
                    Box((0.0047, 0.0047, 0.0078)),
                    origin=Origin(xyz=(side * stab_offset, 0.0, 0.0105)),
                    material=switch_stem,
                    name=f"stabilizer_stem_{'a' if side < 0 else 'b'}",
                )

        legend_w = min(max(w * U * 0.22, 0.0030), 0.020)
        key.visual(
            Box((legend_w, 0.00115, 0.00034)),
            origin=Origin(xyz=(0.0, -0.0018, CAP_BOTTOM + CAP_HEIGHT + 0.00004)),
            material=legend_white,
            name="legend_mark",
        )

        model.articulation(
            f"case_to_{name}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key,
            origin=Origin(xyz=(x, y, PLATE_TOP)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.2, velocity=0.45, lower=0.0, upper=0.0040),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    key_parts = [p for p in object_model.parts if p.name != "case"]
    key_joints = [j for j in object_model.articulations if j.name.startswith("case_to_")]
    ctx.check("80 percent TKL has 87 pressable keys", len(key_parts) == 87 and len(key_joints) == 87)
    ctx.check(
        "all key mechanisms are prismatic presses",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in key_joints),
    )
    ctx.check(
        "each switch press travels four millimetres",
        all(
            j.motion_limits is not None
            and abs(j.motion_limits.lower - 0.0) < 1e-9
            and abs(j.motion_limits.upper - 0.0040) < 1e-9
            for j in key_joints
        ),
    )

    key_q = object_model.get_part("key_q")
    key_w = object_model.get_part("key_w")
    key_a = object_model.get_part("key_a")
    ctx.expect_origin_gap(key_w, key_q, axis="x", min_gap=U - 0.0004, max_gap=U + 0.0004, name="adjacent switches use 19.05 mm pitch")
    ctx.expect_origin_gap(key_q, key_a, axis="y", min_gap=U - 0.0004, max_gap=U + 0.0004, name="switch rows use 19.05 mm pitch")

    case = object_model.get_part("case")
    space = object_model.get_part("space_key")
    space_joint = object_model.get_articulation("case_to_space_key")
    ctx.expect_gap(space, case, axis="z", positive_elem="cap", negative_elem="switch_space_key", min_gap=0.0035, name="spacebar cap clears fixed switch housing at rest")
    ctx.expect_overlap(space, case, axes="xy", elem_a="stem", elem_b="switch_space_key", min_overlap=0.004, name="spacebar stem is centered over its switch")
    with ctx.pose({space_joint: 0.0040}):
        ctx.expect_gap(space, case, axis="z", positive_elem="cap", negative_elem="switch_space_key", min_gap=0.0001, name="pressed spacebar cap clears switch housing")
        ctx.expect_overlap(space, case, axes="z", elem_a="stem", elem_b="switch_space_key", min_overlap=0.006, name="pressed stem remains retained in switch housing")

    a_joint = object_model.get_articulation("case_to_key_a")
    rest = ctx.part_world_position(key_a)
    with ctx.pose({a_joint: 0.0040}):
        pressed = ctx.part_world_position(key_a)
    ctx.check(
        "pressed key moves downward",
        rest is not None and pressed is not None and pressed[2] < rest[2] - 0.0035,
        details=f"rest={rest}, pressed={pressed}",
    )

    return ctx.report()


object_model = build_object_model()
