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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wireless_presenter_keyboard")

    body_mat = model.material("soft_touch_black", rgba=(0.025, 0.027, 0.030, 1.0))
    edge_mat = model.material("satin_edge", rgba=(0.10, 0.105, 0.11, 1.0))
    rail_mat = model.material("dark_anodized_rail", rgba=(0.045, 0.047, 0.050, 1.0))
    tray_mat = model.material("tray_graphite", rgba=(0.075, 0.078, 0.084, 1.0))
    key_mat = model.material("rubber_key_gray", rgba=(0.60, 0.61, 0.60, 1.0))
    button_mat = model.material("control_button", rgba=(0.13, 0.135, 0.14, 1.0))
    red_mat = model.material("laser_red", rgba=(0.85, 0.03, 0.025, 1.0))
    stand_mat = model.material("stand_black", rgba=(0.035, 0.036, 0.040, 1.0))

    body_width = 0.060
    body_length = 0.170
    body_thick = 0.014
    bottom_y = -body_length / 2.0
    top_z = body_thick / 2.0

    body = model.part("body")
    body_shell = ExtrudeGeometry(
        rounded_rect_profile(body_width, body_length, 0.012, corner_segments=10),
        body_thick,
        cap=True,
        center=True,
    )
    body.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        origin=Origin(),
        material=body_mat,
        name="body_shell",
    )
    body.visual(
        Box((0.050, 0.002, 0.006)),
        origin=Origin(xyz=(0.0, bottom_y - 0.0007, -0.0025)),
        material=edge_mat,
        name="keyboard_slot",
    )
    body.visual(
        Box((0.038, 0.0015, 0.004)),
        origin=Origin(xyz=(0.0, body_length / 2.0 + 0.0007, 0.001)),
        material=red_mat,
        name="laser_window",
    )

    # Two fixed guide rails protrude from the bottom edge and flank the sliding tray.
    body.visual(
        Box((0.005, 0.075, 0.004)),
        origin=Origin(xyz=(-0.0235, bottom_y - 0.0375, -0.008)),
        material=rail_mat,
        name="guide_rail_0",
    )
    body.visual(
        Box((0.005, 0.008, 0.004)),
        origin=Origin(xyz=(-0.0235, bottom_y - 0.002, -0.0065)),
        material=rail_mat,
        name="rail_mount_0",
    )
    body.visual(
        Box((0.005, 0.075, 0.004)),
        origin=Origin(xyz=(0.0235, bottom_y - 0.0375, -0.008)),
        material=rail_mat,
        name="guide_rail_1",
    )
    body.visual(
        Box((0.005, 0.008, 0.004)),
        origin=Origin(xyz=(0.0235, bottom_y - 0.002, -0.0065)),
        material=rail_mat,
        name="rail_mount_1",
    )

    # Rear kickstand hinge hardware fixed to the lower back edge.
    body.visual(
        Box((0.052, 0.014, 0.002)),
        origin=Origin(xyz=(0.0, bottom_y - 0.004, -0.0074)),
        material=stand_mat,
        name="stand_hinge_leaf",
    )
    for idx, x in enumerate((-0.0175, 0.0175)):
        body.visual(
            Cylinder(radius=0.0026, length=0.014),
            origin=Origin(
                xyz=(x, bottom_y - 0.007, -0.011),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=stand_mat,
            name=f"stand_knuckle_{idx}",
        )

    tray = model.part("keyboard_tray")
    tray_panel = ExtrudeGeometry(
        rounded_rect_profile(0.054, 0.078, 0.004, corner_segments=6),
        0.004,
        cap=True,
        center=True,
    )
    tray.visual(
        mesh_from_geometry(tray_panel, "keyboard_tray_panel"),
        origin=Origin(xyz=(0.0, -0.039, -0.004)),
        material=tray_mat,
        name="tray_panel",
    )
    tray.visual(
        Box((0.050, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, -0.0795, -0.0025)),
        material=edge_mat,
        name="pull_lip",
    )
    tray.visual(
        Box((0.002, 0.130, 0.003)),
        origin=Origin(xyz=(-0.0200, -0.013, -0.0075)),
        material=rail_mat,
        name="runner_0",
    )
    tray.visual(
        Box((0.002, 0.130, 0.003)),
        origin=Origin(xyz=(0.0200, -0.013, -0.0075)),
        material=rail_mat,
        name="runner_1",
    )

    tray_joint = model.articulation(
        "body_to_keyboard_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, bottom_y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=0.055),
    )

    # Pressable QWERTY keycaps mounted on the sliding tray.
    key_rows = (
        ("qwertyuiop", -0.0200, 0.0000),
        ("asdfghjkl", -0.0325, 0.0026),
        ("zxcvbnm", -0.0450, 0.0078),
    )
    key_w = 0.0045
    key_d = 0.0075
    key_pitch = 0.0052
    for row_idx, (letters, y, x_offset) in enumerate(key_rows):
        row_width = (len(letters) - 1) * key_pitch
        for col_idx, _letter in enumerate(letters):
            x = -row_width / 2.0 + col_idx * key_pitch + x_offset
            key = model.part(f"key_{row_idx}_{col_idx}")
            key.visual(
                Box((key_w, key_d, 0.0016)),
                origin=Origin(xyz=(0.0, 0.0, 0.0008)),
                material=key_mat,
                name="keycap",
            )
            model.articulation(
                f"tray_to_key_{row_idx}_{col_idx}",
                ArticulationType.PRISMATIC,
                parent=tray,
                child=key,
                origin=Origin(xyz=(x, y, -0.002)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(effort=0.6, velocity=0.05, lower=0.0, upper=0.0012),
            )

    for name, x, width in (
        ("fn_key", -0.018, 0.010),
        ("space_key", 0.000, 0.018),
        ("enter_key", 0.020, 0.010),
    ):
        key = model.part(name)
        key.visual(
            Box((width, key_d, 0.0016)),
            origin=Origin(xyz=(0.0, 0.0, 0.0008)),
            material=key_mat,
            name="keycap",
        )
        model.articulation(
            f"tray_to_{name}",
            ArticulationType.PRISMATIC,
            parent=tray,
            child=key,
            origin=Origin(xyz=(x, -0.0595, -0.002)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.05, lower=0.0, upper=0.0012),
        )

    # Presenter controls on the top face are separate pressable parts.
    button_specs = (
        ("laser_button", 0.0, 0.054, 0.0055, red_mat),
        ("select_button", 0.0, 0.018, 0.0065, button_mat),
        ("next_button", 0.0, -0.010, 0.0058, button_mat),
        ("back_button", 0.0, -0.034, 0.0058, button_mat),
    )
    for name, x, y, radius, mat in button_specs:
        button = model.part(name)
        button.visual(
            Cylinder(radius=radius, length=0.0024),
            origin=Origin(xyz=(0.0, 0.0, 0.0012)),
            material=mat,
            name="button_cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, y, top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.2, velocity=0.06, lower=0.0, upper=0.0015),
        )

    stand = model.part("tilt_stand")
    stand.visual(
        Cylinder(radius=0.0024, length=0.021),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_mat,
        name="stand_barrel",
    )
    stand.visual(
        Box((0.014, 0.008, 0.005)),
        origin=Origin(xyz=(0.0, 0.004, -0.004)),
        material=stand_mat,
        name="barrel_web",
    )
    stand.visual(
        Box((0.018, 0.095, 0.003)),
        origin=Origin(xyz=(0.0, 0.052, -0.006)),
        material=stand_mat,
        name="leg_leaf",
    )
    model.articulation(
        "body_to_tilt_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, bottom_y - 0.007, -0.011)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.10),
    )

    # The variable name is intentionally kept local so the joint remains authored even
    # though the returned object is not otherwise used in construction.
    _ = tray_joint
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    tray = object_model.get_part("keyboard_tray")
    stand = object_model.get_part("tilt_stand")
    tray_joint = object_model.get_articulation("body_to_keyboard_tray")
    stand_joint = object_model.get_articulation("body_to_tilt_stand")

    ctx.expect_gap(
        body,
        tray,
        axis="y",
        positive_elem="body_shell",
        negative_elem="tray_panel",
        max_gap=0.002,
        max_penetration=0.0005,
        name="keyboard tray starts at the bottom edge",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="y",
        elem_a="runner_0",
        elem_b="guide_rail_0",
        min_overlap=0.050,
        name="tray runner remains inside guide rail",
    )

    rest_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: 0.055}):
        extended_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            elem_a="runner_0",
            elem_b="guide_rail_0",
            min_overlap=0.045,
            name="extended tray still retained by guide rail",
        )
    ctx.check(
        "keyboard tray slides outward from bottom edge",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] < rest_pos[1] - 0.045,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_gap(
        body,
        stand,
        axis="z",
        positive_elem="body_shell",
        negative_elem="leg_leaf",
        min_gap=0.004,
        max_gap=0.012,
        name="folded stand rests on rear side of slim body",
    )
    ctx.expect_overlap(
        stand,
        body,
        axes="y",
        elem_a="leg_leaf",
        elem_b="body_shell",
        min_overlap=0.060,
        name="folded stand lies along the presenter back",
    )
    folded_aabb = ctx.part_element_world_aabb(stand, elem="leg_leaf")
    with ctx.pose({stand_joint: 1.10}):
        open_aabb = ctx.part_element_world_aabb(stand, elem="leg_leaf")
    ctx.check(
        "tilt stand pivots away from the back",
        folded_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < folded_aabb[0][2] - 0.045,
        details=f"folded={folded_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
