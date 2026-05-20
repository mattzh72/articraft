from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.330
BODY_D = 0.140
BODY_H = 0.016
KEY_H = 0.006
KEY_BOTTOM_Z = 0.021
KEY_TRAVEL = 0.0032
KEY_DECK_TOP_Z = BODY_H + 0.0018
KEY_STEM_H = KEY_BOTTOM_Z - KEY_DECK_TOP_Z


def _rounded_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    """CadQuery box with real dimensions and a modest molded radius."""
    safe_radius = min(radius, width * 0.22, depth * 0.22, height * 0.42)
    shape = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    if safe_radius > 0.00005:
        shape = shape.edges().fillet(safe_radius)
    return shape


def _keyboard_body_shape() -> cq.Workplane:
    """Shallow rounded keyboard case with the rear phone slot cut into the top."""
    shell = _rounded_box(BODY_W, BODY_D, BODY_H, 0.0045)
    slot_w = 0.286
    slot_d = 0.012
    slot_y = 0.055
    slot_depth = 0.0055
    return (
        shell.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(0.0, slot_y)
        .rect(slot_w, slot_d)
        .cutBlind(-slot_depth)
    )


def _keycap_shape(width: float, depth: float = 0.015, height: float = KEY_H) -> cq.Workplane:
    """Low-profile office keyboard cap: rectangular, radiused, and slightly soft-edged."""
    return _rounded_box(width, depth, height, 0.0012)


def _row_positions(widths: list[float], gap: float = 0.003) -> list[float]:
    total = sum(widths) + gap * (len(widths) - 1)
    cursor = -total / 2.0
    centers: list[float] = []
    for width in widths:
        cursor += width / 2.0
        centers.append(cursor)
        cursor += width / 2.0 + gap
    return centers


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_office_keyboard")

    case_mat = model.material("warm_graphite_case", rgba=(0.20, 0.21, 0.22, 1.0))
    deck_mat = model.material("recessed_black_deck", rgba=(0.025, 0.027, 0.030, 1.0))
    key_mat = model.material("charcoal_keycaps", rgba=(0.075, 0.078, 0.082, 1.0))
    mod_key_mat = model.material("dark_modifier_keys", rgba=(0.055, 0.058, 0.064, 1.0))
    legend_mat = model.material("soft_white_legends", rgba=(0.78, 0.80, 0.78, 1.0))
    switch_mat = model.material("blue_power_slider", rgba=(0.05, 0.32, 0.68, 1.0))

    body_mesh = mesh_from_cadquery(_keyboard_body_shape(), "rounded_keyboard_body")
    key_meshes = {
        0.017: mesh_from_cadquery(_keycap_shape(0.017), "keycap_1u"),
        0.024: mesh_from_cadquery(_keycap_shape(0.024), "keycap_1_4u"),
        0.027: mesh_from_cadquery(_keycap_shape(0.027), "keycap_backspace"),
        0.029: mesh_from_cadquery(_keycap_shape(0.029), "keycap_caps"),
        0.030: mesh_from_cadquery(_keycap_shape(0.030), "keycap_enter"),
        0.037: mesh_from_cadquery(_keycap_shape(0.037), "keycap_wide_enter"),
        0.038: mesh_from_cadquery(_keycap_shape(0.038), "keycap_shift"),
        0.046: mesh_from_cadquery(_keycap_shape(0.046), "keycap_right_shift"),
        0.095: mesh_from_cadquery(_keycap_shape(0.095), "keycap_spacebar"),
    }

    body = model.part("body")
    body.visual(body_mesh, material=case_mat, name="case_shell")
    body.visual(
        Box((0.302, 0.106, 0.0020)),
        origin=Origin(xyz=(0.0, -0.012, BODY_H + 0.0008)),
        material=deck_mat,
        name="key_deck",
    )
    body.visual(
        Box((0.286, 0.011, 0.0008)),
        origin=Origin(xyz=(0.0, 0.055, BODY_H - 0.0050)),
        material=deck_mat,
        name="phone_slot_floor",
    )
    body.visual(
        Box((0.292, 0.006, 0.013)),
        origin=Origin(xyz=(0.0, 0.067, BODY_H + 0.0055)),
        material=case_mat,
        name="rear_phone_lip",
    )
    body.visual(
        Box((0.0010, 0.039, 0.011)),
        origin=Origin(xyz=(BODY_W / 2.0 - 0.0005, -0.024, 0.010)),
        material=deck_mat,
        name="power_guide",
    )

    rows = [
        (0.033, [0.017] * 13 + [0.027]),
        (0.012, [0.024] + [0.017] * 11 + [0.030]),
        (-0.009, [0.029] + [0.017] * 10 + [0.037]),
        (-0.030, [0.038] + [0.017] * 9 + [0.046]),
        (-0.052, [0.024, 0.024, 0.024, 0.095, 0.024, 0.024, 0.024]),
    ]

    for row_index, (y, widths) in enumerate(rows):
        for col_index, (x, width) in enumerate(zip(_row_positions(widths), widths)):
            key = model.part(f"key_{row_index}_{col_index}")
            material = mod_key_mat if width > 0.020 else key_mat
            key.visual(key_meshes[width], material=material, name="cap")
            key.visual(
                Box((0.006, 0.006, KEY_STEM_H + 0.0002)),
                origin=Origin(xyz=(0.0, 0.0, -KEY_STEM_H / 2.0 + 0.0001)),
                material=deck_mat,
                name="plunger",
            )

            # A small raised dash gives the separate keycaps readable legends
            # without making every key a different mesh.
            if width < 0.090:
                key.visual(
                    Box((min(width * 0.38, 0.008), 0.0012, 0.00035)),
                    origin=Origin(xyz=(-width * 0.16, -0.002, KEY_H + 0.00005)),
                    material=legend_mat,
                    name="legend",
                )
            else:
                key.visual(
                    Box((0.040, 0.0015, 0.00035)),
                    origin=Origin(xyz=(0.0, 0.000, KEY_H + 0.00005)),
                    material=legend_mat,
                    name="legend",
                )

            model.articulation(
                f"body_to_key_{row_index}_{col_index}",
                ArticulationType.PRISMATIC,
                parent=body,
                child=key,
                origin=Origin(xyz=(x, y, KEY_BOTTOM_Z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(effort=2.0, velocity=0.20, lower=0.0, upper=KEY_TRAVEL),
            )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Box((0.008, 0.012, 0.007)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=switch_mat,
        name="thumb",
    )
    power_switch.visual(
        Box((0.0014, 0.008, 0.0045)),
        origin=Origin(xyz=(0.0087, 0.0, 0.0)),
        material=case_mat,
        name="grip_ridge",
    )
    model.articulation(
        "body_to_power_switch",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_switch,
        origin=Origin(xyz=(BODY_W / 2.0, -0.033, 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.08, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    sample_key = object_model.get_part("key_2_5")
    sample_key_joint = object_model.get_articulation("body_to_key_2_5")
    switch = object_model.get_part("power_switch")
    switch_joint = object_model.get_articulation("body_to_power_switch")

    ctx.expect_gap(
        sample_key,
        body,
        axis="z",
        min_gap=0.002,
        max_gap=0.005,
        positive_elem="cap",
        negative_elem="key_deck",
        name="keycaps sit just above the recessed deck",
    )
    ctx.expect_overlap(
        sample_key,
        body,
        axes="xy",
        min_overlap=0.010,
        elem_a="cap",
        elem_b="key_deck",
        name="keycap footprint lands on the keyboard deck",
    )

    rest_key_pos = ctx.part_world_position(sample_key)
    with ctx.pose({sample_key_joint: KEY_TRAVEL}):
        pressed_key_pos = ctx.part_world_position(sample_key)
    ctx.check(
        "sample key plunges normal to the top",
        rest_key_pos is not None
        and pressed_key_pos is not None
        and pressed_key_pos[2] < rest_key_pos[2] - KEY_TRAVEL * 0.8,
        details=f"rest={rest_key_pos}, pressed={pressed_key_pos}",
    )

    ctx.expect_overlap(
        switch,
        body,
        axes="yz",
        min_overlap=0.006,
        elem_a="thumb",
        elem_b="power_guide",
        name="side power slider sits in its guide slot",
    )
    ctx.expect_gap(
        switch,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.0010,
        positive_elem="thumb",
        negative_elem="power_guide",
        name="slider rides against the side guide without intersecting",
    )

    rest_switch_pos = ctx.part_world_position(switch)
    with ctx.pose({switch_joint: 0.018}):
        on_switch_pos = ctx.part_world_position(switch)
    ctx.check(
        "power switch translates along the side slot",
        rest_switch_pos is not None
        and on_switch_pos is not None
        and on_switch_pos[1] > rest_switch_pos[1] + 0.015,
        details=f"rest={rest_switch_pos}, on={on_switch_pos}",
    )

    return ctx.report()


object_model = build_object_model()
