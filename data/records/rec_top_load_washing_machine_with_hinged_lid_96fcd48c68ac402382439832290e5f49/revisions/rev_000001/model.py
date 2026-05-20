from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_tub_shell_mesh():
    outer_profile = [
        (0.030, -0.540),
        (0.145, -0.536),
        (0.180, -0.505),
        (0.191, -0.145),
        (0.196, -0.040),
        (0.200, 0.000),
    ]
    inner_profile = [
        (0.000, -0.515),
        (0.135, -0.502),
        (0.166, -0.145),
        (0.176, -0.040),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
            start_cap="flat",
            end_cap="round",
            lip_samples=10,
        ),
        "washer_tub_shell",
    )


def _build_cycle_dial_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.092,
            0.034,
            body_style="skirted",
            top_diameter=0.074,
            skirt=KnobSkirt(0.108, 0.008, flare=0.05),
            grip=KnobGrip(style="fluted", count=18, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "washer_cycle_dial",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.95, 0.96, 0.97, 1.0))
    console_dark = model.material("console_dark", rgba=(0.23, 0.25, 0.28, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    glass_smoke = model.material("glass_smoke", rgba=(0.25, 0.33, 0.38, 0.38))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    button_silver = model.material("button_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    impeller_gray = model.material("impeller_gray", rgba=(0.62, 0.64, 0.67, 1.0))

    cabinet_width = 0.68
    cabinet_depth = 0.70
    panel_thickness = 0.022
    panel_height = 0.93
    deck_thickness = 0.035
    deck_center_z = 0.9425
    deck_top_z = deck_center_z + (deck_thickness * 0.5)
    control_face_y = 0.252

    opening_width = 0.43
    opening_depth = 0.43
    opening_center_y = -0.05
    opening_front_y = opening_center_y - (opening_depth * 0.5)
    opening_rear_y = opening_center_y + (opening_depth * 0.5)
    side_strip_width = 0.5 * (cabinet_width - opening_width)
    front_strip_depth = opening_front_y - (-cabinet_depth * 0.5)
    rear_strip_depth = control_face_y - opening_rear_y
    lock_slot_width = 0.028

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((panel_thickness, cabinet_depth, panel_height)),
        origin=Origin(xyz=(-(cabinet_width - panel_thickness) * 0.5, 0.0, panel_height * 0.5)),
        material=cabinet_white,
        name="side_left",
    )
    cabinet.visual(
        Box((panel_thickness, cabinet_depth, panel_height)),
        origin=Origin(xyz=((cabinet_width - panel_thickness) * 0.5, 0.0, panel_height * 0.5)),
        material=cabinet_white,
        name="side_right",
    )
    cabinet.visual(
        Box((cabinet_width - (2.0 * panel_thickness), panel_thickness, panel_height)),
        origin=Origin(xyz=(0.0, -((cabinet_depth - panel_thickness) * 0.5), panel_height * 0.5)),
        material=cabinet_white,
        name="front_panel",
    )
    cabinet.visual(
        Box((cabinet_width - (2.0 * panel_thickness), panel_thickness, panel_height)),
        origin=Origin(xyz=(0.0, ((cabinet_depth - panel_thickness) * 0.5), panel_height * 0.5)),
        material=cabinet_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((cabinet_width - (2.0 * panel_thickness), cabinet_depth - (2.0 * panel_thickness), 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=trim_dark,
        name="base_pan",
    )
    cabinet.visual(
        Box((((cabinet_width - lock_slot_width) * 0.5), front_strip_depth, deck_thickness)),
        origin=Origin(
            xyz=(
                -((lock_slot_width + ((cabinet_width - lock_slot_width) * 0.5)) * 0.5),
                (-cabinet_depth * 0.5) + (front_strip_depth * 0.5),
                deck_center_z,
            )
        ),
        material=cabinet_white,
        name="deck_front_left",
    )
    cabinet.visual(
        Box((((cabinet_width - lock_slot_width) * 0.5), front_strip_depth, deck_thickness)),
        origin=Origin(
            xyz=(
                ((lock_slot_width + ((cabinet_width - lock_slot_width) * 0.5)) * 0.5),
                (-cabinet_depth * 0.5) + (front_strip_depth * 0.5),
                deck_center_z,
            )
        ),
        material=cabinet_white,
        name="deck_front_right",
    )
    cabinet.visual(
        Box((side_strip_width, opening_depth, deck_thickness)),
        origin=Origin(
            xyz=(
                -((opening_width + side_strip_width) * 0.5),
                opening_center_y,
                deck_center_z,
            )
        ),
        material=cabinet_white,
        name="deck_side_0",
    )
    cabinet.visual(
        Box((side_strip_width, opening_depth, deck_thickness)),
        origin=Origin(
            xyz=(
                ((opening_width + side_strip_width) * 0.5),
                opening_center_y,
                deck_center_z,
            )
        ),
        material=cabinet_white,
        name="deck_side_1",
    )
    cabinet.visual(
        Box((cabinet_width, rear_strip_depth, deck_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                opening_rear_y + (rear_strip_depth * 0.5),
                deck_center_z,
            )
        ),
        material=cabinet_white,
        name="deck_rear_strip",
    )
    cabinet.visual(
        Box((0.58, 0.090, 0.120)),
        origin=Origin(xyz=(0.0, 0.305, 1.010)),
        material=cabinet_white,
        name="console_box",
    )
    cabinet.visual(
        Box((0.54, 0.008, 0.082)),
        origin=Origin(xyz=(0.0, 0.256, 1.000)),
        material=console_dark,
        name="control_panel",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.62, 0.51, 0.022)),
        origin=Origin(xyz=(0.0, -0.255, 0.011)),
        material=glass_smoke,
        name="lid_glass",
    )
    lid.visual(
        Box((0.62, 0.090, 0.028)),
        origin=Origin(xyz=(0.0, -0.045, 0.014)),
        material=cabinet_white,
        name="lid_rear_strip",
    )
    lid.visual(
        Box((0.065, 0.51, 0.028)),
        origin=Origin(xyz=(-0.2775, -0.255, 0.014)),
        material=cabinet_white,
        name="lid_side_0",
    )
    lid.visual(
        Box((0.065, 0.51, 0.028)),
        origin=Origin(xyz=(0.2775, -0.255, 0.014)),
        material=cabinet_white,
        name="lid_side_1",
    )
    lid.visual(
        Box((0.62, 0.105, 0.030)),
        origin=Origin(xyz=(0.0, -0.4575, 0.015)),
        material=cabinet_white,
        name="lid_front_strip",
    )
    lid.visual(
        Box((0.20, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.505, 0.006)),
        material=button_silver,
        name="lid_handle",
    )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.175, deck_top_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    tub = model.part("tub")
    tub.visual(_build_tub_shell_mesh(), material=stainless, name="tub_shell")
    tub.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.4925)),
        material=impeller_gray,
        name="impeller_hub",
    )
    tub.visual(
        Box((0.110, 0.025, 0.035)),
        origin=Origin(xyz=(0.055, 0.0, -0.475)),
        material=impeller_gray,
        name="tub_vane",
    )
    tub.visual(
        Cylinder(radius=0.018, length=0.355),
        origin=Origin(xyz=(0.0, 0.0, -0.6625)),
        material=trim_dark,
        name="drive_shaft",
    )
    tub.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.825)),
        material=trim_dark,
        name="drive_collar",
    )

    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, opening_center_y, 0.890)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=12.0,
        ),
    )

    dial = model.part("dial")
    dial.visual(
        _build_cycle_dial_mesh(),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=button_silver,
        name="dial_knob",
    )
    dial.visual(
        Box((0.016, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, -0.015, 0.023)),
        material=trim_dark,
        name="dial_marker",
    )

    model.articulation(
        "cabinet_to_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(-0.145, control_face_y, 1.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
        ),
    )

    button_x_positions = (0.065, 0.125, 0.185, 0.245)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.038, 0.016, 0.018)),
            origin=Origin(xyz=(0.0, -0.008, 0.0)),
            material=button_silver,
            name="button_cap",
        )
        model.articulation(
            f"cabinet_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, control_face_y, 0.990)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.050,
                lower=0.0,
                upper=0.0045,
            ),
        )

    plunger = model.part("plunger")
    plunger.visual(
        Box((0.026, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=trim_dark,
        name="plunger_head",
    )
    plunger.visual(
        Box((0.012, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=trim_dark,
        name="plunger_shaft",
    )

    model.articulation(
        "lid_to_plunger",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=plunger,
        origin=Origin(xyz=(0.0, -0.495, 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.050,
            lower=0.0,
            upper=0.014,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    plunger = object_model.get_part("plunger")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    tub_spin = object_model.get_articulation("cabinet_to_tub")
    dial_spin = object_model.get_articulation("cabinet_to_dial")
    button_joints = [object_model.get_articulation(f"cabinet_to_button_{index}") for index in range(4)]
    plunger_slide = object_model.get_articulation("lid_to_plunger")

    ctx.check(
        "primary joints use the requested motion models",
        (
            lid_hinge.articulation_type == ArticulationType.REVOLUTE
            and tub_spin.articulation_type == ArticulationType.CONTINUOUS
            and dial_spin.articulation_type == ArticulationType.CONTINUOUS
            and all(joint.articulation_type == ArticulationType.PRISMATIC for joint in button_joints)
            and plunger_slide.articulation_type == ArticulationType.PRISMATIC
        ),
        details=(
            f"lid={lid_hinge.articulation_type}, tub={tub_spin.articulation_type}, "
            f"dial={dial_spin.articulation_type}, buttons={[joint.articulation_type for joint in button_joints]}, "
            f"plunger={plunger_slide.articulation_type}"
        ),
    )

    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        positive_elem="lid_side_0",
        negative_elem="deck_side_0",
        min_gap=0.0,
        max_gap=0.004,
        name="closed lid sits neatly on the top deck",
    )

    ctx.expect_contact(
        tub,
        cabinet,
        elem_a="drive_collar",
        elem_b="base_pan",
        contact_tol=0.002,
        name="wash tub support collar seats on the cabinet base",
    )

    tub_shell_aabb = ctx.part_element_world_aabb(tub, elem="tub_shell")
    tub_shell_depth = None
    if tub_shell_aabb is not None:
        tub_shell_depth = tub_shell_aabb[1][2] - tub_shell_aabb[0][2]
    ctx.check(
        "wash tub reads as a deep cavity below the deck",
        (
            tub_shell_aabb is not None
            and tub_shell_depth is not None
            and tub_shell_depth > 0.50
            and tub_shell_aabb[1][2] < 0.91
            and tub_shell_aabb[0][2] < 0.37
        ),
        details=f"tub_shell_aabb={tub_shell_aabb}, tub_shell_depth={tub_shell_depth}",
    )

    lid_rest_aabb = ctx.part_world_aabb(lid)
    lid_open_aabb = None
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward above the cabinet",
        (
            lid_rest_aabb is not None
            and lid_open_aabb is not None
            and lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.22
        ),
        details=f"rest={lid_rest_aabb}, open={lid_open_aabb}",
    )

    tub_vane_rest = _aabb_center(ctx.part_element_world_aabb(tub, elem="tub_vane"))
    tub_vane_turn = None
    with ctx.pose({tub_spin: math.pi * 0.5}):
        tub_vane_turn = _aabb_center(ctx.part_element_world_aabb(tub, elem="tub_vane"))
    ctx.check(
        "wash tub rotates around the vertical axis",
        (
            tub_vane_rest is not None
            and tub_vane_turn is not None
            and abs(tub_vane_turn[0] - tub_vane_rest[0]) > 0.03
            and abs(tub_vane_turn[1] - tub_vane_rest[1]) > 0.03
        ),
        details=f"rest={tub_vane_rest}, turned={tub_vane_turn}",
    )

    dial_marker_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_marker"))
    dial_marker_turn = None
    with ctx.pose({dial_spin: math.pi * 0.5}):
        dial_marker_turn = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_marker"))
    ctx.check(
        "cycle dial visibly turns on the rear console",
        (
            dial_marker_rest is not None
            and dial_marker_turn is not None
            and abs(dial_marker_turn[0] - dial_marker_rest[0]) > 0.01
            and abs(dial_marker_turn[2] - dial_marker_rest[2]) > 0.01
        ),
        details=f"rest={dial_marker_rest}, turned={dial_marker_turn}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    button_0_pressed = None
    button_1_unchanged = None
    with ctx.pose({button_joints[0]: button_joints[0].motion_limits.upper}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_unchanged = ctx.part_world_position(button_1)
    ctx.check(
        "buttons press inward independently",
        (
            button_0_rest is not None
            and button_1_rest is not None
            and button_0_pressed is not None
            and button_1_unchanged is not None
            and button_0_pressed[1] > button_0_rest[1] + 0.003
            and abs(button_1_unchanged[1] - button_1_rest[1]) < 1e-6
        ),
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_during_press={button_1_unchanged}"
        ),
    )

    plunger_rest = ctx.part_world_position(plunger)
    plunger_retracted = None
    with ctx.pose({plunger_slide: plunger_slide.motion_limits.upper}):
        plunger_retracted = ctx.part_world_position(plunger)
    ctx.check(
        "lid lock plunger retracts upward out of the deck slot",
        (
            plunger_rest is not None
            and plunger_retracted is not None
            and plunger_retracted[2] > plunger_rest[2] + 0.010
        ),
        details=f"rest={plunger_rest}, retracted={plunger_retracted}",
    )

    return ctx.report()


object_model = build_object_model()
