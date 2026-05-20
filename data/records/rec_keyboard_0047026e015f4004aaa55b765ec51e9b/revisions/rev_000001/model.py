from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ExtrudeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_programming_keyboard")

    case_w = 0.340
    case_d = 0.140
    case_h = 0.018
    plate_h = 0.0015
    top_z = case_h + plate_h
    switch_h = 0.0030
    key_pitch = 0.019
    key_unit = 0.0172

    model.material("case_black", color=(0.015, 0.017, 0.020, 1.0))
    model.material("top_plate", color=(0.045, 0.048, 0.052, 1.0))
    model.material("switch_black", color=(0.008, 0.008, 0.009, 1.0))
    model.material("key_dark", color=(0.075, 0.078, 0.083, 1.0))
    model.material("key_top", color=(0.115, 0.118, 0.124, 1.0))
    model.material("macro_blue", color=(0.055, 0.115, 0.190, 1.0))
    model.material("arrow_gray", color=(0.105, 0.108, 0.115, 1.0))
    model.material("legend_white", color=(0.82, 0.86, 0.88, 1.0))
    model.material("knob_gray", color=(0.22, 0.23, 0.24, 1.0))
    model.material("shaft_metal", color=(0.58, 0.60, 0.60, 1.0))
    model.material("rubber", color=(0.018, 0.018, 0.016, 1.0))

    case = model.part("case")
    shell_profile = rounded_rect_profile(case_w, case_d, 0.012, corner_segments=10)
    case_shell = ExtrudeGeometry.from_z0(shell_profile, case_h)
    case.visual(
        mesh_from_geometry(case_shell, "case_shell"),
        material="case_black",
        name="case_shell",
    )
    case.visual(
        Box((case_w - 0.018, case_d - 0.018, plate_h)),
        origin=Origin(xyz=(0.0, 0.0, case_h + plate_h / 2.0)),
        material="top_plate",
        name="top_plate",
    )

    def add_switch_socket(name: str, x: float, y: float, w_units: float = 1.0) -> None:
        socket_w = min(0.012, key_unit * w_units * 0.62)
        socket_name = "macro_0_socket" if name == "macro_0" else f"{name}_socket"
        case.visual(
            Box((socket_w, 0.012, switch_h)),
            origin=Origin(xyz=(x, y, top_z + switch_h / 2.0)),
            material="switch_black",
            name=socket_name,
        )

    def add_key(
        name: str,
        x: float,
        y: float,
        *,
        w_units: float = 1.0,
        material: str = "key_dark",
        top_material: str = "key_top",
        legend: str = "dash",
    ) -> None:
        cap_w = key_unit * w_units - 0.0012
        cap_d = key_unit - 0.0012
        add_switch_socket(name, x, y, w_units=w_units)

        key = model.part(name)
        key.visual(
            Box((0.006, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material="switch_black",
            name="plunger_stem",
        )
        key.visual(
            Box((cap_w, cap_d, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=material,
            name="keycap_skirt",
        )
        key.visual(
            Box((cap_w * 0.82, cap_d * 0.78, 0.0014)),
            origin=Origin(xyz=(0.0, 0.0, 0.0107)),
            material=top_material,
            name="keycap_top",
        )

        # Small raised legends make the grid read as a keyboard without needing
        # text rendering.  They are physical, shallow plastic inlays on the cap.
        if legend == "wide":
            legend_size = (cap_w * 0.52, 0.0016, 0.00045)
        elif legend == "arrow_up":
            legend_size = (0.004, 0.007, 0.00045)
        elif legend == "arrow_side":
            legend_size = (0.007, 0.004, 0.00045)
        else:
            legend_size = (0.005, 0.0015, 0.00045)
        key.visual(
            Box(legend_size),
            origin=Origin(xyz=(0.0, 0.0, 0.01150)),
            material="legend_white",
            name="legend_mark",
        )

        model.articulation(
            f"case_to_{name}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key,
            origin=Origin(xyz=(x, y, top_z + switch_h)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.18, lower=0.0, upper=0.004),
        )

    # Left-side macro column: five deliberately different blue keys.
    macro_x = -0.150
    row_y = [0.046, 0.027, 0.008, -0.011, -0.030]
    for index, y in enumerate(row_y):
        add_key(
            f"macro_{index}",
            macro_x,
            y,
            material="macro_blue",
            top_material="macro_blue",
            legend="wide" if index == 0 else "dash",
        )

    # Dense compact main alphanumeric block.
    start_x = -0.121
    for row_index, y in enumerate([0.046, 0.027, 0.008, -0.011]):
        columns = 12 if row_index < 2 else 11
        offset = 0.000 if row_index < 2 else 0.004
        for col in range(columns):
            add_key(f"key_{row_index}_{col}", start_x + offset + col * key_pitch, y)

    # Bottom row with real compact-keyboard width variation.
    bottom_y = -0.036
    bottom_keys = [
        ("key_4_0", -0.117, 1.15),
        ("key_4_1", -0.094, 1.15),
        ("key_4_2", -0.066, 1.25),
        ("spacebar", -0.017, 4.70),
        ("key_4_3", 0.044, 1.15),
        ("key_4_4", 0.066, 1.15),
        ("key_4_5", 0.088, 1.15),
    ]
    for name, x, units in bottom_keys:
        add_key(name, x, bottom_y, w_units=units, legend="wide" if units > 2.0 else "dash")

    # Inverted-T arrow cluster at the front right; the rotary encoder is above it.
    add_key("arrow_up", 0.130, -0.016, material="arrow_gray", top_material="arrow_gray", legend="arrow_up")
    add_key("arrow_left", 0.111, -0.036, material="arrow_gray", top_material="arrow_gray", legend="arrow_side")
    add_key("arrow_down", 0.130, -0.036, material="arrow_gray", top_material="arrow_gray", legend="arrow_up")
    add_key("arrow_right", 0.149, -0.036, material="arrow_gray", top_material="arrow_gray", legend="arrow_side")

    knob_x = 0.130
    knob_y = 0.026
    collar_h = 0.004
    case.visual(
        Cylinder(radius=0.013, length=collar_h),
        origin=Origin(xyz=(knob_x, knob_y, top_z + collar_h / 2.0)),
        material="switch_black",
        name="knob_collar",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.0042, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="shaft_metal",
        name="shaft",
    )
    knob_mesh = KnobGeometry(
        0.036,
        0.018,
        body_style="faceted",
        base_diameter=0.038,
        top_diameter=0.030,
        edge_radius=0.0008,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0009, width=0.0018),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_mesh, "oversized_encoder_knob"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="knob_gray",
        name="knob_cap",
    )
    knob.visual(
        Box((0.0030, 0.014, 0.00065)),
        origin=Origin(xyz=(0.0, 0.0048, 0.0243)),
        material="legend_white",
        name="indicator",
    )
    model.articulation(
        "case_to_knob",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=knob,
        origin=Origin(xyz=(knob_x, knob_y, top_z + collar_h)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    # Fold-out rear feet mounted under the back edge.
    foot_y = case_d / 2.0 - 0.014
    for index, x in enumerate((-0.105, 0.105)):
        foot = model.part(f"rear_foot_{index}")
        foot.visual(
            Cylinder(radius=0.003, length=0.024),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material="rubber",
            name="hinge_barrel",
        )
        foot.visual(
            Box((0.022, 0.038, 0.004)),
            origin=Origin(xyz=(0.0, -0.021, -0.004)),
            material="rubber",
            name="foot_pad",
        )
        foot.visual(
            Box((0.018, 0.010, 0.003)),
            origin=Origin(xyz=(0.0, -0.006, -0.003)),
            material="rubber",
            name="hinge_web",
        )
        model.articulation(
            f"case_to_rear_foot_{index}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=foot,
            origin=Origin(xyz=(x, foot_y, -0.003)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=0.0, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    macro = object_model.get_part("macro_0")
    arrow = object_model.get_part("arrow_down")
    knob = object_model.get_part("knob")
    foot = object_model.get_part("rear_foot_0")
    macro_slide = object_model.get_articulation("case_to_macro_0")
    arrow_slide = object_model.get_articulation("case_to_arrow_down")
    knob_spin = object_model.get_articulation("case_to_knob")
    foot_hinge = object_model.get_articulation("case_to_rear_foot_0")

    ctx.expect_gap(
        macro,
        case,
        axis="z",
        positive_elem="plunger_stem",
        negative_elem="macro_0_socket",
        max_gap=0.0005,
        max_penetration=0.0001,
        name="macro key stem sits on its switch socket",
    )
    ctx.expect_gap(
        knob,
        case,
        axis="z",
        positive_elem="shaft",
        negative_elem="knob_collar",
        max_gap=0.0005,
        max_penetration=0.0,
        name="rotary knob shaft sits on the encoder collar",
    )
    ctx.expect_gap(
        case,
        foot,
        axis="z",
        positive_elem="case_shell",
        negative_elem="hinge_barrel",
        max_gap=0.0005,
        max_penetration=0.0002,
        name="rear foot hinge barrel is tucked against the underside",
    )

    rest_macro = ctx.part_world_position(macro)
    with ctx.pose({macro_slide: 0.004}):
        pressed_macro = ctx.part_world_position(macro)
    ctx.check(
        "macro key plunges downward",
        rest_macro is not None
        and pressed_macro is not None
        and pressed_macro[2] < rest_macro[2] - 0.0035,
        details=f"rest={rest_macro}, pressed={pressed_macro}",
    )

    rest_arrow = ctx.part_world_position(arrow)
    with ctx.pose({arrow_slide: 0.004}):
        pressed_arrow = ctx.part_world_position(arrow)
    ctx.check(
        "arrow key plunges downward",
        rest_arrow is not None
        and pressed_arrow is not None
        and pressed_arrow[2] < rest_arrow[2] - 0.0035,
        details=f"rest={rest_arrow}, pressed={pressed_arrow}",
    )

    rest_indicator = ctx.part_element_world_aabb(knob, elem="indicator")
    with ctx.pose({knob_spin: math.pi / 2.0}):
        turned_indicator = ctx.part_element_world_aabb(knob, elem="indicator")
    if rest_indicator is not None and turned_indicator is not None:
        rest_center_x = (rest_indicator[0][0] + rest_indicator[1][0]) / 2.0
        turned_center_x = (turned_indicator[0][0] + turned_indicator[1][0]) / 2.0
        knob_turns = turned_center_x < rest_center_x - 0.003
    else:
        knob_turns = False
    ctx.check(
        "knob indicator rotates around the shaft",
        knob_turns,
        details=f"rest_indicator={rest_indicator}, turned_indicator={turned_indicator}",
    )

    rest_foot_aabb = ctx.part_element_world_aabb(foot, elem="foot_pad")
    with ctx.pose({foot_hinge: 1.05}):
        deployed_foot_aabb = ctx.part_element_world_aabb(foot, elem="foot_pad")
    if rest_foot_aabb is not None and deployed_foot_aabb is not None:
        foot_drops = deployed_foot_aabb[0][2] < rest_foot_aabb[0][2] - 0.025
    else:
        foot_drops = False
    ctx.check(
        "rear foot folds down from the back edge",
        foot_drops,
        details=f"rest_foot={rest_foot_aabb}, deployed_foot={deployed_foot_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
