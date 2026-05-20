from __future__ import annotations

import math

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


BODY_WIDTH = 0.430
BODY_DEPTH = 0.340
BODY_HEIGHT = 0.180


def _build_body_mesh():
    wall = 0.006

    base = cq.Workplane("XY").box(0.418, 0.332, wall, centered=(True, True, False))
    side_left = (
        cq.Workplane("XY")
        .box(wall, BODY_DEPTH - 0.012, BODY_HEIGHT - wall, centered=(True, True, False))
        .translate((-(BODY_WIDTH * 0.5 - wall * 0.5), 0.0, wall))
    )
    side_right = (
        cq.Workplane("XY")
        .box(wall, BODY_DEPTH - 0.012, BODY_HEIGHT - wall, centered=(True, True, False))
        .translate(((BODY_WIDTH * 0.5 - wall * 0.5), 0.0, wall))
    )

    front_wall_blank = (
        cq.Workplane("XY")
        .box(BODY_WIDTH - 0.012, wall, BODY_HEIGHT - 0.012, centered=(True, True, False))
        .translate((0.0, BODY_DEPTH * 0.5 - wall * 0.5, wall))
    )
    front_opening = (
        cq.Workplane("XY")
        .box(0.314, 0.020, 0.054, centered=(True, True, False))
        .translate((0.0, BODY_DEPTH * 0.5 - wall * 0.5, 0.022))
    )
    front_wall = front_wall_blank.cut(front_opening)

    rear_wall_blank = (
        cq.Workplane("XY")
        .box(BODY_WIDTH - 0.012, wall, BODY_HEIGHT - 0.012, centered=(True, True, False))
        .translate((0.0, -(BODY_DEPTH * 0.5 - wall * 0.5), wall))
    )
    rear_opening = (
        cq.Workplane("XY")
        .box(0.208, 0.020, 0.060, centered=(True, True, False))
        .translate((0.0, -(BODY_DEPTH * 0.5 - wall * 0.5), 0.096))
    )
    rear_wall = rear_wall_blank.cut(rear_opening)

    top_plate_blank = (
        cq.Workplane("XY")
        .box(BODY_WIDTH - 0.012, BODY_DEPTH - 0.012, wall, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_HEIGHT - wall))
    )
    scanner_top_opening = (
        cq.Workplane("XY")
        .box(0.344, 0.254, 0.020, centered=(True, True, False))
        .translate((0.0, -0.004, BODY_HEIGHT - 0.010))
    )
    top_plate = top_plate_blank.cut(scanner_top_opening)

    scanner_well_blank = (
        cq.Workplane("XY")
        .box(0.344, 0.254, 0.014, centered=(True, True, False))
        .translate((0.0, -0.004, 0.160))
    )
    scanner_well_opening = (
        cq.Workplane("XY")
        .box(0.332, 0.242, 0.020, centered=(True, True, False))
        .translate((0.0, -0.004, 0.159))
    )
    scanner_well = scanner_well_blank.cut(scanner_well_opening)

    body = (
        base.union(side_left)
        .union(side_right)
        .union(front_wall)
        .union(rear_wall)
        .union(top_plate)
        .union(scanner_well)
    )
    return body


def _build_lid_mesh():
    outer = cq.Workplane("XY").box(0.394, 0.286, 0.018, centered=(True, False, False))
    inner = (
        cq.Workplane("XY")
        .box(0.352, 0.244, 0.013, centered=(True, False, False))
        .translate((0.0, 0.021, 0.0))
    )
    lid = outer.cut(inner)
    return lid


def _build_cassette_mesh():
    panel_width = 0.326
    tray_width = 0.304
    depth = 0.246
    height = 0.040
    wall = 0.003
    floor = 0.003
    front = 0.004

    floor_panel = (
        cq.Workplane("XY")
        .box(tray_width, depth, floor, centered=(True, False, False))
        .translate((0.0, -(depth - front), 0.0))
    )
    left_wall = (
        cq.Workplane("XY")
        .box(wall, depth - front, height - floor, centered=(True, False, False))
        .translate((-(tray_width * 0.5 - wall * 0.5), -(depth - front), floor))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(wall, depth - front, height - floor, centered=(True, False, False))
        .translate(((tray_width * 0.5 - wall * 0.5), -(depth - front), floor))
    )
    rear_wall = (
        cq.Workplane("XY")
        .box(tray_width, wall, height - floor, centered=(True, False, False))
        .translate((0.0, -depth, floor))
    )
    front_panel = (
        cq.Workplane("XY")
        .box(panel_width, front, height, centered=(True, False, False))
    )
    handle_lip = (
        cq.Workplane("XY")
        .box(0.180, 0.006, 0.010, centered=(True, False, False))
        .translate((0.0, front, 0.012))
    )

    cassette = (
        floor_panel.union(left_wall)
        .union(right_wall)
        .union(rear_wall)
        .union(front_panel)
        .union(handle_lip)
    )
    return cassette


def _build_feed_tray_mesh():
    width = 0.216
    depth = 0.170
    thickness = 0.003
    rail = 0.004

    panel = (
        cq.Workplane("XY")
        .box(width, depth, thickness, centered=(True, False, False))
        .translate((0.0, -depth, 0.0))
    )
    left_rail = (
        cq.Workplane("XY")
        .box(rail, depth * 0.92, 0.020, centered=(True, False, False))
        .translate((-(width * 0.5 - rail * 0.5), -(depth * 0.92), thickness))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(rail, depth * 0.92, 0.020, centered=(True, False, False))
        .translate(((width * 0.5 - rail * 0.5), -(depth * 0.92), thickness))
    )
    stop_lip = (
        cq.Workplane("XY")
        .box(width * 0.72, 0.006, 0.014, centered=(True, False, False))
        .translate((0.0, -depth, thickness))
    )
    return panel.union(left_rail).union(right_rail).union(stop_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_printer")

    shell = model.material("shell", rgba=(0.84, 0.86, 0.88, 1.0))
    trim = model.material("trim", rgba=(0.20, 0.21, 0.23, 1.0))
    dark = model.material("dark", rgba=(0.13, 0.14, 0.15, 1.0))
    glass = model.material("glass", rgba=(0.32, 0.46, 0.52, 0.35))

    body = model.part("body")
    body.visual(Box((0.418, 0.332, 0.008)), origin=Origin(xyz=(0.0, 0.0, 0.004)), material=shell, name="floor")
    body.visual(Box((0.010, 0.334, 0.172)), origin=Origin(xyz=(-0.210, 0.0, 0.092)), material=shell, name="side_wall_0")
    body.visual(Box((0.010, 0.334, 0.172)), origin=Origin(xyz=(0.210, 0.0, 0.092)), material=shell, name="side_wall_1")
    body.visual(Box((0.418, 0.010, 0.088)), origin=Origin(xyz=(0.0, -0.165, 0.044)), material=shell, name="rear_lower")
    body.visual(Box((0.418, 0.010, 0.038)), origin=Origin(xyz=(0.0, -0.165, 0.155)), material=shell, name="rear_upper")
    body.visual(Box((0.105, 0.010, 0.060)), origin=Origin(xyz=(-0.1565, -0.165, 0.120)), material=shell, name="rear_jamb_0")
    body.visual(Box((0.105, 0.010, 0.060)), origin=Origin(xyz=(0.1565, -0.165, 0.120)), material=shell, name="rear_jamb_1")
    body.visual(Box((0.418, 0.010, 0.024)), origin=Origin(xyz=(0.0, 0.165, 0.012)), material=shell, name="front_lower")
    body.visual(Box((0.418, 0.010, 0.094)), origin=Origin(xyz=(0.0, 0.165, 0.127)), material=shell, name="front_upper")
    body.visual(Box((0.052, 0.010, 0.056)), origin=Origin(xyz=(-0.181, 0.165, 0.052)), material=shell, name="front_jamb_0")
    body.visual(Box((0.052, 0.010, 0.056)), origin=Origin(xyz=(0.181, 0.165, 0.052)), material=shell, name="front_jamb_1")
    body.visual(Box((0.418, 0.060, 0.008)), origin=Origin(xyz=(0.0, 0.130, 0.176)), material=shell, name="deck_front")
    body.visual(Box((0.418, 0.038, 0.008)), origin=Origin(xyz=(0.0, -0.147, 0.176)), material=shell, name="deck_rear")
    body.visual(Box((0.041, 0.246, 0.008)), origin=Origin(xyz=(-0.1885, -0.004, 0.176)), material=shell, name="deck_side_0")
    body.visual(Box((0.041, 0.246, 0.008)), origin=Origin(xyz=(0.1885, -0.004, 0.176)), material=shell, name="deck_side_1")
    body.visual(Box((0.344, 0.008, 0.014)), origin=Origin(xyz=(0.0, 0.119, 0.167)), material=shell, name="scanner_wall_front")
    body.visual(Box((0.344, 0.008, 0.014)), origin=Origin(xyz=(0.0, -0.127, 0.167)), material=shell, name="scanner_wall_rear")
    body.visual(Box((0.008, 0.246, 0.014)), origin=Origin(xyz=(-0.168, -0.004, 0.167)), material=shell, name="scanner_wall_0")
    body.visual(Box((0.008, 0.246, 0.014)), origin=Origin(xyz=(0.168, -0.004, 0.167)), material=shell, name="scanner_wall_1")
    body.visual(Box((0.190, 0.008, 0.008)), origin=Origin(xyz=(0.0, -0.172, 0.176)), material=dark, name="tray_hinge_mount")
    body.visual(Box((0.338, 0.240, 0.004)), origin=Origin(xyz=(0.0, -0.004, 0.158)), material=trim, name="scanner_floor")
    body.visual(Box((0.330, 0.232, 0.002)), origin=Origin(xyz=(0.0, -0.004, 0.1605)), material=glass, name="scanner_glass")
    body.visual(Box((0.138, 0.028, 0.052)), origin=Origin(xyz=(0.106, 0.155, 0.122)), material=trim, name="control_bank")

    lid = model.part("lid")
    lid.visual(Box((0.394, 0.286, 0.006)), origin=Origin(xyz=(0.0, 0.143, 0.015)), material=shell, name="lid_skin")
    lid.visual(Box((0.394, 0.014, 0.012)), origin=Origin(xyz=(0.0, 0.007, 0.006)), material=shell, name="lid_rear_rim")
    lid.visual(Box((0.394, 0.018, 0.018)), origin=Origin(xyz=(0.0, 0.277, 0.009)), material=shell, name="lid_front_rim")
    lid.visual(Box((0.018, 0.286, 0.018)), origin=Origin(xyz=(-0.188, 0.143, 0.009)), material=shell, name="lid_side_0")
    lid.visual(Box((0.018, 0.286, 0.018)), origin=Origin(xyz=(0.188, 0.143, 0.009)), material=shell, name="lid_side_1")
    lid.visual(Box((0.334, 0.228, 0.010)), origin=Origin(xyz=(0.0, 0.143, 0.008)), material=dark, name="lid_backing")
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.143, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    cassette = model.part("cassette")
    cassette.visual(Box((0.326, 0.004, 0.040)), origin=Origin(xyz=(0.0, 0.002, 0.020)), material=dark, name="front_panel")
    cassette.visual(Box((0.304, 0.242, 0.003)), origin=Origin(xyz=(0.0, -0.117, 0.0015)), material=dark, name="floor")
    cassette.visual(Box((0.003, 0.242, 0.037)), origin=Origin(xyz=(-0.1505, -0.117, 0.0215)), material=dark, name="side_0")
    cassette.visual(Box((0.003, 0.242, 0.037)), origin=Origin(xyz=(0.1505, -0.117, 0.0215)), material=dark, name="side_1")
    cassette.visual(Box((0.304, 0.005, 0.037)), origin=Origin(xyz=(0.0, -0.2395, 0.0215)), material=dark, name="rear_wall")
    cassette.visual(Box((0.180, 0.006, 0.010)), origin=Origin(xyz=(0.0, 0.006, 0.029)), material=trim, name="handle_lip")
    model.articulation(
        "body_to_cassette",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cassette,
        origin=Origin(xyz=(0.0, 0.170, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.20, lower=0.0, upper=0.118),
    )

    feed_tray = model.part("feed_tray")
    feed_tray.visual(Box((0.216, 0.170, 0.003)), origin=Origin(xyz=(0.0, -0.085, 0.0015)), material=dark, name="panel")
    feed_tray.visual(Box((0.004, 0.158, 0.020)), origin=Origin(xyz=(-0.106, -0.087, 0.012)), material=dark, name="rail_0")
    feed_tray.visual(Box((0.004, 0.158, 0.020)), origin=Origin(xyz=(0.106, -0.087, 0.012)), material=dark, name="rail_1")
    feed_tray.visual(Box((0.156, 0.006, 0.014)), origin=Origin(xyz=(0.0, -0.167, 0.010)), material=dark, name="stop_lip")
    feed_tray.visual(Box((0.190, 0.008, 0.006)), origin=Origin(xyz=(0.0, -0.002, 0.003)), material=dark, name="hinge_tab")
    model.articulation(
        "body_to_feed_tray",
        ArticulationType.REVOLUTE,
        parent=body,
        child=feed_tray,
        origin=Origin(xyz=(0.0, -0.170, 0.180)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="knob",
    )
    dial.visual(
        Box((0.004, 0.015, 0.003)),
        origin=Origin(xyz=(0.0, 0.015, 0.010)),
        material=trim,
        name="pointer",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.060, 0.169, 0.126)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=3.5, lower=-1.8, upper=1.8),
    )

    button_x = (0.098, 0.126, 0.154)
    for index, x_pos in enumerate(button_x):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.016, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, 0.003, 0.0)),
            material=shell,
            name="cap",
        )
        button.visual(
            Box((0.010, 0.002, 0.006)),
            origin=Origin(xyz=(0.0, 0.001, 0.0)),
            material=trim,
            name="collar",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.169, 0.126)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.04, lower=0.0, upper=0.001),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cassette = object_model.get_part("cassette")
    feed_tray = object_model.get_part("feed_tray")
    dial = object_model.get_part("dial")
    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]

    lid_hinge = object_model.get_articulation("body_to_lid")
    cassette_slide = object_model.get_articulation("body_to_cassette")
    tray_hinge = object_model.get_articulation("body_to_feed_tray")
    dial_joint = object_model.get_articulation("body_to_dial")
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(3)]

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.22,
        name="closed lid covers the scanner bed",
    )
    ctx.expect_overlap(
        cassette,
        body,
        axes="x",
        min_overlap=0.30,
        name="cassette stays centered in the printer width",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="y",
        positive_elem="knob",
        negative_elem="control_bank",
        max_gap=0.002,
        max_penetration=0.0,
        name="dial mounts against the control bank face",
    )
    for index, button in enumerate(buttons):
        ctx.expect_gap(
            button,
            body,
            axis="y",
            positive_elem="collar",
            negative_elem="control_bank",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"button_{index} rests on the control bank face",
        )

    cassette_rest = ctx.part_world_position(cassette)
    lid_rest_aabb = ctx.part_world_aabb(lid)
    tray_rest_aabb = ctx.part_world_aabb(feed_tray)
    dial_rest_pointer = ctx.part_element_world_aabb(dial, elem="pointer")
    button_rest = [ctx.part_world_position(button) for button in buttons]

    with ctx.pose({cassette_slide: cassette_slide.motion_limits.upper}):
        ctx.expect_overlap(
            cassette,
            body,
            axes="y",
            min_overlap=0.12,
            name="cassette remains retained at full extension",
        )
        cassette_extended = ctx.part_world_position(cassette)

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_open_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({tray_hinge: tray_hinge.motion_limits.upper}):
        tray_open_aabb = ctx.part_world_aabb(feed_tray)

    with ctx.pose({dial_joint: 1.1}):
        dial_turned_pointer = ctx.part_element_world_aabb(dial, elem="pointer")

    button_pressed: list[tuple[float, float, float] | None] = []
    for joint, button in zip(button_joints, buttons):
        with ctx.pose({joint: joint.motion_limits.upper}):
            button_pressed.append(ctx.part_world_position(button))

    ctx.check(
        "cassette extends forward",
        cassette_rest is not None
        and cassette_extended is not None
        and cassette_extended[1] > cassette_rest[1] + 0.09,
        details=f"rest={cassette_rest}, extended={cassette_extended}",
    )
    ctx.check(
        "lid opens upward",
        lid_rest_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.14,
        details=f"rest={lid_rest_aabb}, open={lid_open_aabb}",
    )
    ctx.check(
        "rear tray folds upward",
        tray_rest_aabb is not None
        and tray_open_aabb is not None
        and tray_open_aabb[1][2] > tray_rest_aabb[1][2] + 0.12,
        details=f"rest={tray_rest_aabb}, open={tray_open_aabb}",
    )
    ctx.check(
        "dial rotates its pointer",
        dial_rest_pointer is not None
        and dial_turned_pointer is not None
        and abs(
            ((dial_turned_pointer[0][0] + dial_turned_pointer[1][0]) * 0.5)
            - ((dial_rest_pointer[0][0] + dial_rest_pointer[1][0]) * 0.5)
        )
        > 0.006,
        details=f"rest={dial_rest_pointer}, turned={dial_turned_pointer}",
    )
    for index, (rest_pos, pressed_pos) in enumerate(zip(button_rest, button_pressed)):
        ctx.check(
            f"button_{index} presses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] < rest_pos[1] - 0.0007,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
