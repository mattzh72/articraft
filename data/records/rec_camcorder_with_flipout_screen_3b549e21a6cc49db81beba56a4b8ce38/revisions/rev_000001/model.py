from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PI_2 = math.pi / 2.0


def _tube_mesh(name: str, outer_radius: float, inner_radius: float, length: float):
    """A simple hollow revolved tube, authored on local Z."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
            [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_shoulder_camcorder")

    mat_body = model.material("satin_black", rgba=(0.015, 0.017, 0.020, 1.0))
    mat_panel = model.material("dark_graphite", rgba=(0.055, 0.060, 0.066, 1.0))
    mat_rubber = model.material("matte_rubber", rgba=(0.003, 0.003, 0.004, 1.0))
    mat_lens = model.material("coated_glass", rgba=(0.08, 0.14, 0.20, 0.78))
    mat_screen = model.material("blue_black_screen", rgba=(0.02, 0.045, 0.075, 1.0))
    mat_metal = model.material("hinge_metal", rgba=(0.26, 0.27, 0.28, 1.0))
    mat_red = model.material("record_red", rgba=(0.75, 0.025, 0.02, 1.0))
    mat_label = model.material("white_marking", rgba=(0.86, 0.88, 0.82, 1.0))

    body = model.part("body")
    # Main boxy shoulder-camcorder chassis, about a real ENG-style hand/shoulder scale.
    body.visual(
        Box((0.42, 0.16, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=mat_body,
        name="main_shell",
    )
    body.visual(
        Box((0.23, 0.115, 0.030)),
        origin=Origin(xyz=(-0.03, 0.0, 0.222)),
        material=mat_body,
        name="top_handle_grip",
    )
    for idx, x in enumerate((-0.125, 0.055)):
        body.visual(
            Box((0.040, 0.040, 0.090)),
            origin=Origin(xyz=(x, 0.0, 0.205)),
            material=mat_body,
            name=f"handle_post_{idx}",
        )
    body.visual(
        Box((0.25, 0.125, 0.025)),
        origin=Origin(xyz=(-0.035, 0.0, -0.0125)),
        material=mat_rubber,
        name="shoulder_pad",
    )

    # Front lens stack: fixed collars and barrel support the rotating focus ring.
    body.visual(
        Cylinder(radius=0.074, length=0.030),
        origin=Origin(xyz=(0.205, 0.0, 0.105), rpy=(0.0, PI_2, 0.0)),
        material=mat_panel,
        name="lens_mount_collar",
    )
    body.visual(
        Cylinder(radius=0.052, length=0.270),
        origin=Origin(xyz=(0.325, 0.0, 0.105), rpy=(0.0, PI_2, 0.0)),
        material=mat_body,
        name="lens_barrel",
    )
    for idx, x in enumerate((0.265, 0.337)):
        body.visual(
            Cylinder(radius=0.057, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.105), rpy=(0.0, PI_2, 0.0)),
            material=mat_panel,
            name=f"ring_stop_{idx}",
        )
    body.visual(
        Cylinder(radius=0.047, length=0.060),
        origin=Origin(xyz=(0.455, 0.0, 0.105), rpy=(0.0, PI_2, 0.0)),
        material=mat_rubber,
        name="front_lens_hood",
    )
    body.visual(
        Cylinder(radius=0.039, length=0.006),
        origin=Origin(xyz=(0.427, 0.0, 0.105), rpy=(0.0, PI_2, 0.0)),
        material=mat_lens,
        name="front_glass",
    )

    # Rear viewfinder/eyepiece with a rubber eyecup.
    body.visual(
        Box((0.045, 0.055, 0.045)),
        origin=Origin(xyz=(-0.205, 0.0, 0.135)),
        material=mat_panel,
        name="viewfinder_block",
    )
    body.visual(
        Cylinder(radius=0.027, length=0.085),
        origin=Origin(xyz=(-0.248, 0.0, 0.135), rpy=(0.0, PI_2, 0.0)),
        material=mat_body,
        name="eyepiece_tube",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.026),
        origin=Origin(xyz=(-0.303, 0.0, 0.135), rpy=(0.0, PI_2, 0.0)),
        material=mat_rubber,
        name="rubber_eyecup",
    )

    # LCD side hinge fixed leaves and knuckles on the left side of the body.
    body.visual(
        Box((0.034, 0.006, 0.122)),
        origin=Origin(xyz=(-0.062, 0.083, 0.095)),
        material=mat_metal,
        name="lcd_hinge_leaf",
    )
    for idx, z in enumerate((0.056, 0.134)):
        body.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(xyz=(-0.062, 0.089, z)),
            material=mat_metal,
            name=f"lcd_fixed_knuckle_{idx}",
        )

    # Media-door fixed hinge support on the right side.
    body.visual(
        Box((0.018, 0.006, 0.112)),
        origin=Origin(xyz=(0.155, -0.083, 0.090)),
        material=mat_metal,
        name="door_hinge_leaf",
    )
    for idx, z in enumerate((0.054, 0.126)):
        body.visual(
            Cylinder(radius=0.006, length=0.026),
            origin=Origin(xyz=(0.155, -0.089, z)),
            material=mat_metal,
            name=f"door_fixed_knuckle_{idx}",
        )

    # Surface-mounted top control cradle.
    body.visual(
        Box((0.125, 0.050, 0.006)),
        origin=Origin(xyz=(0.010, -0.036, 0.183)),
        material=mat_panel,
        name="top_control_panel",
    )
    for idx, y in enumerate((-0.060, -0.012)):
        body.visual(
            Box((0.070, 0.006, 0.012)),
            origin=Origin(xyz=(0.010, y, 0.192)),
            material=mat_panel,
            name=f"rocker_rail_{idx}",
        )

    # Flip-out LCD first moving stage: a supported hinge arm with a distal yoke.
    lcd_arm = model.part("lcd_arm")
    lcd_arm.visual(
        Cylinder(radius=0.007, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mat_metal,
        name="swing_knuckle",
    )
    lcd_arm.visual(
        Box((0.112, 0.014, 0.018)),
        origin=Origin(xyz=(0.056, 0.008, 0.0)),
        material=mat_panel,
        name="swing_arm_bar",
    )
    for idx, z in enumerate((-0.032, 0.032)):
        lcd_arm.visual(
            Cylinder(radius=0.007, length=0.018),
            origin=Origin(xyz=(0.115, 0.008, z)),
            material=mat_metal,
            name=f"screen_yoke_{idx}",
        )
    lcd_arm.visual(
        Box((0.024, 0.018, 0.082)),
        origin=Origin(xyz=(0.108, 0.023, 0.0)),
        material=mat_panel,
        name="screen_yoke_bridge",
    )

    model.articulation(
        "body_to_lcd_arm",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lcd_arm,
        origin=Origin(xyz=(-0.062, 0.089, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    # Flip-out LCD second moving stage: screen panel rotating on the arm yoke.
    lcd_panel = model.part("lcd_panel")
    lcd_panel.visual(
        Box((0.136, 0.012, 0.092)),
        origin=Origin(xyz=(0.080, 0.006, 0.0)),
        material=mat_panel,
        name="screen_case",
    )
    lcd_panel.visual(
        Box((0.112, 0.002, 0.064)),
        origin=Origin(xyz=(0.080, 0.013, 0.0)),
        material=mat_screen,
        name="display_glass",
    )
    lcd_panel.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mat_metal,
        name="panel_pivot_knuckle",
    )
    lcd_panel.visual(
        Box((0.024, 0.008, 0.038)),
        origin=Origin(xyz=(0.006, 0.000, 0.0)),
        material=mat_panel,
        name="panel_hinge_leaf",
    )
    model.articulation(
        "arm_to_lcd_panel",
        ArticulationType.REVOLUTE,
        parent=lcd_arm,
        child=lcd_panel,
        origin=Origin(xyz=(0.115, 0.008, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=-1.65, upper=1.65),
    )

    # Hinged media-card door, carried by real side knuckles.
    media_door = model.part("media_door")
    media_door.visual(
        Box((0.108, 0.006, 0.086)),
        origin=Origin(xyz=(-0.054, -0.006, 0.0)),
        material=mat_panel,
        name="door_panel",
    )
    media_door.visual(
        Box((0.085, 0.002, 0.052)),
        origin=Origin(xyz=(-0.057, -0.010, 0.0)),
        material=mat_body,
        name="door_recess",
    )
    media_door.visual(
        Cylinder(radius=0.0055, length=0.041),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mat_metal,
        name="door_moving_knuckle",
    )
    media_door.visual(
        Box((0.014, 0.006, 0.038)),
        origin=Origin(xyz=(-0.006, -0.003, 0.0)),
        material=mat_metal,
        name="door_hinge_leaf",
    )
    model.articulation(
        "body_to_media_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=media_door,
        origin=Origin(xyz=(0.155, -0.089, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.5, lower=0.0, upper=1.35),
    )

    # Rotating, hollow focus/zoom ring around the front lens, with raised grip ribs.
    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _tube_mesh("focus_ring_tube", outer_radius=0.061, inner_radius=0.055, length=0.058),
        origin=Origin(rpy=(0.0, PI_2, 0.0)),
        material=mat_rubber,
        name="ring_shell",
    )
    for idx in range(18):
        theta = idx * 2.0 * math.pi / 18.0
        focus_ring.visual(
            Box((0.044, 0.004, 0.007)),
            origin=Origin(
                xyz=(0.0, -0.063 * math.sin(theta), 0.063 * math.cos(theta)),
                rpy=(theta, 0.0, 0.0),
            ),
            material=mat_rubber,
            name=f"ring_grip_{idx}",
        )
    focus_ring.visual(
        Box((0.030, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=mat_label,
        name="ring_index_mark",
    )
    model.articulation(
        "body_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.301, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=5.0, lower=-math.pi, upper=math.pi),
    )

    # User controls are separate supported moving parts, not painted-on lumps.
    zoom_rocker = model.part("zoom_rocker")
    zoom_rocker.visual(
        Box((0.060, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=mat_rubber,
        name="rocker_cap",
    )
    model.articulation(
        "body_to_zoom_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=zoom_rocker,
        origin=Origin(xyz=(0.010, -0.036, 0.186)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=2.0, lower=-0.22, upper=0.22),
    )

    record_button = model.part("record_button")
    record_button.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=mat_red,
        name="record_cap",
    )
    model.articulation(
        "body_to_record_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=record_button,
        origin=Origin(xyz=(-0.035, -0.052, 0.186)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.12, velocity=0.06, lower=0.0, upper=0.004),
    )

    for idx, y in enumerate((-0.046, 0.046)):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.006, 0.025, 0.015)),
            origin=Origin(xyz=(-0.003, y, 0.055)),
            material=mat_panel,
            name=f"rear_button_cap_{idx}",
        )
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(-0.210, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.10, velocity=0.05, lower=0.0, upper=0.003),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lcd_arm = object_model.get_part("lcd_arm")
    lcd_panel = object_model.get_part("lcd_panel")
    media_door = object_model.get_part("media_door")
    focus_ring = object_model.get_part("focus_ring")
    record_button = object_model.get_part("record_button")
    zoom_rocker = object_model.get_part("zoom_rocker")

    lcd_swing = object_model.get_articulation("body_to_lcd_arm")
    lcd_pivot = object_model.get_articulation("arm_to_lcd_panel")
    door_hinge = object_model.get_articulation("body_to_media_door")
    ring_joint = object_model.get_articulation("body_to_focus_ring")

    ctx.check(
        "lcd uses two revolute joints",
        lcd_swing.articulation_type == ArticulationType.REVOLUTE
        and lcd_pivot.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "focus ring is revolute",
        ring_joint.articulation_type == ArticulationType.REVOLUTE,
    )

    # Closed LCD and media door sit proud of their side faces while being carried by hinges.
    with ctx.pose({"body_to_lcd_arm": 0.0, "arm_to_lcd_panel": 0.0}):
        ctx.expect_gap(
            lcd_panel,
            body,
            axis="y",
            min_gap=0.006,
            max_gap=0.030,
            positive_elem="screen_case",
            negative_elem="main_shell",
            name="closed lcd clears left side",
        )
        ctx.expect_overlap(
            lcd_panel,
            body,
            axes="xz",
            min_overlap=0.045,
            elem_a="screen_case",
            elem_b="main_shell",
            name="closed lcd lies on side of body",
        )

    rest_lcd_pos = ctx.part_world_position(lcd_panel)
    with ctx.pose({"body_to_lcd_arm": 1.20}):
        swung_lcd_pos = ctx.part_world_position(lcd_panel)
    ctx.check(
        "lcd arm swings outward",
        rest_lcd_pos is not None
        and swung_lcd_pos is not None
        and swung_lcd_pos[1] > rest_lcd_pos[1] + 0.07,
        details=f"rest={rest_lcd_pos}, swung={swung_lcd_pos}",
    )

    with ctx.pose({"arm_to_lcd_panel": 1.20}):
        aabb = ctx.part_world_aabb(lcd_panel)
    ctx.check(
        "lcd panel pivots on arm",
        aabb is not None and (aabb[1][1] - aabb[0][1]) > 0.10,
        details=f"aabb={aabb}",
    )

    with ctx.pose({"body_to_media_door": 0.0}):
        ctx.expect_gap(
            body,
            media_door,
            axis="y",
            min_gap=0.006,
            max_gap=0.020,
            positive_elem="main_shell",
            negative_elem="door_panel",
            name="closed media door clears side",
        )
        closed_door_aabb = ctx.part_world_aabb(media_door)
    with ctx.pose({"body_to_media_door": 1.05}):
        open_door_aabb = ctx.part_world_aabb(media_door)
    ctx.check(
        "media door hinges outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.045,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    ctx.expect_within(
        focus_ring,
        body,
        axes="yz",
        margin=0.014,
        inner_elem="ring_shell",
        outer_elem="lens_barrel",
        name="focus ring stays coaxial with lens barrel",
    )
    ctx.expect_overlap(
        focus_ring,
        body,
        axes="x",
        min_overlap=0.050,
        elem_a="ring_shell",
        elem_b="lens_barrel",
        name="focus ring surrounds lens barrel length",
    )
    ctx.expect_gap(
        record_button,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="record_cap",
        negative_elem="top_control_panel",
        name="record button is seated on top panel",
    )
    ctx.expect_gap(
        zoom_rocker,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rocker_cap",
        negative_elem="top_control_panel",
        name="zoom rocker is seated in rails",
    )

    return ctx.report()


object_model = build_object_model()
