from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.600
BODY_D = 0.560
BODY_H = 0.835
BODY_WALL = 0.006

DOOR_W = 0.596
DOOR_H = 0.785
DOOR_BOTTOM_Z = 0.026
BODY_FRONT_Y = BODY_D * 0.5
RAIL_FRONT_Y = 0.215
RAIL_LENGTH = 0.430
LOWER_RAIL_Z = 0.128
UPPER_RAIL_Z = 0.432
TRAY_RAIL_Z = 0.696


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _union_boxes(boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]]) -> cq.Workplane:
    result: cq.Workplane | None = None
    for size, center in boxes:
        solid = _cq_box(size, center)
        result = solid if result is None else result.union(solid)
    if result is None:
        raise ValueError("expected at least one box to union")
    return result


def _build_body_shape() -> cq.Workplane:
    outer = _cq_box((BODY_W, BODY_D, BODY_H), (0.0, 0.0, BODY_H * 0.5))
    cavity = _cq_box(
        (BODY_W - 2.0 * BODY_WALL, BODY_D - BODY_WALL + 0.026, BODY_H - 2.0 * BODY_WALL),
        (0.0, BODY_WALL * 0.5 + 0.013, BODY_WALL + (BODY_H - 2.0 * BODY_WALL) * 0.5),
    )
    shell = outer.cut(cavity)

    top_flange = _cq_box((BODY_W - 0.050, 0.030, 0.030), (0.0, BODY_FRONT_Y - 0.015, BODY_H - 0.018))
    lower_threshold = _cq_box((BODY_W - 0.090, 0.036, 0.020), (0.0, BODY_FRONT_Y - 0.018, 0.046))
    hinge_block_0 = _cq_box((0.040, 0.050, 0.060), (-0.245, BODY_FRONT_Y - 0.025, 0.030))
    hinge_block_1 = _cq_box((0.040, 0.050, 0.060), (0.245, BODY_FRONT_Y - 0.025, 0.030))
    sump_hub = _cq_box((0.090, 0.090, 0.016), (0.0, -0.030, 0.008))

    return shell.union(top_flange).union(lower_threshold).union(hinge_block_0).union(hinge_block_1).union(sump_hub)


def _build_door_shape() -> cq.Workplane:
    outer_panel = _cq_box((DOOR_W, 0.016, DOOR_H), (0.0, 0.010, DOOR_H * 0.5))

    liner = _cq_box((0.544, 0.024, 0.716), (0.0, -0.010, 0.378))
    liner = liner.cut(_cq_box((0.500, 0.020, 0.636), (0.0, -0.012, 0.380)))

    detergent_cup = _cq_box((0.118, 0.018, 0.088), (0.166, -0.014, 0.620))
    lower_relief = _cq_box((0.420, 0.016, 0.150), (0.0, -0.008, 0.175))
    dispenser_housing = _cq_box((0.200, 0.010, 0.130), (0.196, -0.010, 0.620))

    return outer_panel.union(liner).union(dispenser_housing).cut(detergent_cup).cut(lower_relief)


def _build_main_rack_shape(
    *,
    width: float,
    depth: float,
    side_height: float,
    front_height: float,
    rear_height: float,
    runner_x: float,
    include_spray_mount: bool = False,
) -> cq.Workplane:
    side_x = width * 0.5 - 0.010
    rear_y = -depth + 0.010
    boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []

    for x_pos in (-runner_x, runner_x):
        boxes.append(((0.026, RAIL_LENGTH, 0.012), (x_pos, -RAIL_LENGTH * 0.5, 0.000)))

    boxes.extend(
        [
            ((width - 0.010, depth - 0.010, 0.008), (0.0, -depth * 0.5, 0.008)),
            ((width, 0.012, front_height), (0.0, -0.006, 0.004 + front_height * 0.5)),
            ((width - 0.020, 0.012, rear_height), (0.0, rear_y, 0.004 + rear_height * 0.5)),
            ((0.012, depth, side_height), (-side_x, -depth * 0.5, 0.004 + side_height * 0.5)),
            ((0.012, depth, side_height), (side_x, -depth * 0.5, 0.004 + side_height * 0.5)),
            ((width - 0.050, 0.010, 0.010), (0.0, -0.006, front_height)),
            ((width - 0.090, 0.010, 0.010), (0.0, rear_y, rear_height)),
        ]
    )

    for x_pos in (-width * 0.18, 0.0, width * 0.18):
        boxes.append(((0.010, depth - 0.060, side_height * 0.75), (x_pos, -depth * 0.52, 0.004 + side_height * 0.375)))

    if include_spray_mount:
        boxes.append(((0.060, 0.040, 0.022), (0.0, -0.225, -0.004)))

    return _union_boxes(boxes)


def _build_third_tray_shape() -> cq.Workplane:
    width = 0.470
    depth = 0.470
    side_x = width * 0.5 - 0.008
    rear_y = -depth + 0.010
    boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []

    for x_pos in (-0.260, 0.260):
        boxes.append(((0.018, RAIL_LENGTH, 0.010), (x_pos, -RAIL_LENGTH * 0.5, 0.000)))

    boxes.extend(
        [
            ((width - 0.010, depth - 0.010, 0.006), (0.0, -depth * 0.5, 0.005)),
            ((width, 0.010, 0.028), (0.0, -0.006, 0.014)),
            ((width, 0.010, 0.028), (0.0, rear_y, 0.014)),
            ((0.010, depth, 0.028), (-side_x, -depth * 0.5, 0.014)),
            ((0.010, depth, 0.028), (side_x, -depth * 0.5, 0.014)),
            ((0.040, depth - 0.020, 0.008), (-0.245, -depth * 0.5, 0.005)),
            ((0.040, depth - 0.020, 0.008), (0.245, -depth * 0.5, 0.005)),
            ((0.020, 0.180, 0.018), (-0.226, -0.235, 0.020)),
            ((0.020, 0.180, 0.018), (0.226, -0.235, 0.020)),
        ]
    )

    return _union_boxes(boxes)


def _build_wing_shape(side: str) -> cq.Workplane:
    sign = -1.0 if side == "left" else 1.0
    boxes = [
        ((0.034, 0.180, 0.006), (sign * 0.017, 0.000, 0.000)),
        ((0.008, 0.180, 0.008), (sign * 0.004, 0.000, 0.001)),
        ((0.020, 0.010, 0.010), (sign * 0.018, 0.070, 0.004)),
        ((0.020, 0.010, 0.010), (sign * 0.018, -0.070, 0.004)),
    ]
    return _union_boxes(boxes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quiet_tall_tub_dishwasher")

    panel_dark = model.material("panel_dark", rgba=(0.22, 0.23, 0.24, 1.0))
    tub_steel = model.material("tub_steel", rgba=(0.77, 0.79, 0.81, 1.0))
    rack_grey = model.material("rack_grey", rgba=(0.68, 0.70, 0.73, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.09, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "dishwasher_body"),
        material=tub_steel,
        name="body_shell",
    )
    for rail_name, rail_x, rail_z, rail_size in (
        ("lower_rail_left", -0.282, LOWER_RAIL_Z, (0.026, RAIL_LENGTH, 0.012)),
        ("lower_rail_right", 0.282, LOWER_RAIL_Z, (0.026, RAIL_LENGTH, 0.012)),
        ("upper_rail_left", -0.282, UPPER_RAIL_Z, (0.026, RAIL_LENGTH, 0.012)),
        ("upper_rail_right", 0.282, UPPER_RAIL_Z, (0.026, RAIL_LENGTH, 0.012)),
        ("tray_rail_left", -0.279, TRAY_RAIL_Z, (0.034, RAIL_LENGTH, 0.010)),
        ("tray_rail_right", 0.279, TRAY_RAIL_Z, (0.034, RAIL_LENGTH, 0.010)),
    ):
        body.visual(
            Box(rail_size),
            origin=Origin(xyz=(rail_x, 0.000, rail_z)),
            material=rack_grey,
            name=rail_name,
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_build_door_shape(), "dishwasher_door"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=panel_dark,
        name="door_shell",
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.016,
                body_style="skirted",
                top_diameter=0.026,
                skirt=KnobSkirt(0.040, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=14, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
                center=False,
            ),
            "dishwasher_selector_dial",
        ),
        material=control_black,
        name="dial_cap",
    )

    for index, x_pos in enumerate((0.078, 0.106, 0.134)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.018, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=control_black,
            name="button_cap",
        )
        model.articulation(
            f"door_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x_pos, 0.016, DOOR_H)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=0.08, lower=0.0, upper=0.004),
        )

    detergent_cover = model.part("detergent_cover")
    detergent_cover.visual(
        Box((0.108, 0.004, 0.080)),
        origin=Origin(xyz=(0.0, -0.002, -0.040)),
        material=rack_grey,
        name="cover_plate",
    )
    detergent_cover.visual(
        Box((0.108, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.006, -0.075)),
        material=rack_grey,
        name="cover_handle",
    )
    detergent_cover.visual(
        Box((0.094, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.001, -0.002)),
        material=rack_grey,
        name="cover_hinge_barrel",
    )

    lower_rack = model.part("lower_rack")
    lower_rack.visual(
        mesh_from_cadquery(
            _build_main_rack_shape(
                width=0.500,
                depth=0.450,
                side_height=0.138,
                front_height=0.154,
                rear_height=0.126,
                runner_x=0.260,
            ),
            "lower_rack",
        ),
        material=rack_grey,
        name="rack_shell",
    )

    upper_rack = model.part("upper_rack")
    upper_rack.visual(
        mesh_from_cadquery(
            _build_main_rack_shape(
                width=0.490,
                depth=0.430,
                side_height=0.130,
                front_height=0.142,
                rear_height=0.120,
                runner_x=0.257,
                include_spray_mount=True,
            ),
            "upper_rack",
        ),
        material=rack_grey,
        name="rack_shell",
    )

    third_tray = model.part("third_tray")
    third_tray.visual(
        mesh_from_cadquery(_build_third_tray_shape(), "third_tray"),
        material=rack_grey,
        name="tray_shell",
    )

    wing_0 = model.part("wing_0")
    wing_0.visual(
        mesh_from_cadquery(_build_wing_shape("left"), "third_tray_wing_left"),
        material=rack_grey,
        name="wing_shell",
    )
    wing_1 = model.part("wing_1")
    wing_1.visual(
        mesh_from_cadquery(_build_wing_shape("right"), "third_tray_wing_right"),
        material=rack_grey,
        name="wing_shell",
    )

    lower_spray_arm = model.part("lower_spray_arm")
    lower_spray_arm.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rack_grey,
        name="hub",
    )
    lower_spray_arm.visual(
        Box((0.210, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=rack_grey,
        name="arm_bar",
    )
    lower_spray_arm.visual(
        Box((0.055, 0.022, 0.008)),
        origin=Origin(xyz=(0.102, 0.020, 0.010)),
        material=rack_grey,
        name="front_nozzle",
    )
    lower_spray_arm.visual(
        Box((0.048, 0.022, 0.008)),
        origin=Origin(xyz=(-0.100, -0.018, 0.010)),
        material=rack_grey,
        name="rear_nozzle",
    )

    upper_spray_arm = model.part("upper_spray_arm")
    upper_spray_arm.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=rack_grey,
        name="hub",
    )
    upper_spray_arm.visual(
        Box((0.165, 0.016, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=rack_grey,
        name="arm_bar",
    )
    upper_spray_arm.visual(
        Box((0.045, 0.020, 0.007)),
        origin=Origin(xyz=(0.078, 0.016, 0.009)),
        material=rack_grey,
        name="front_nozzle",
    )
    upper_spray_arm.visual(
        Box((0.040, 0.020, 0.007)),
        origin=Origin(xyz=(-0.077, -0.015, 0.009)),
        material=rack_grey,
        name="rear_nozzle",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.022, DOOR_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "door_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=selector_dial,
        origin=Origin(xyz=(-0.145, 0.016, DOOR_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )
    model.articulation(
        "door_to_detergent_cover",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_cover,
        origin=Origin(xyz=(0.166, -0.006, 0.670)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )
    model.articulation(
        "body_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.0, RAIL_FRONT_Y, LOWER_RAIL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.260),
    )
    model.articulation(
        "body_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.0, RAIL_FRONT_Y, UPPER_RAIL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.240),
    )
    model.articulation(
        "body_to_third_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=third_tray,
        origin=Origin(xyz=(0.0, RAIL_FRONT_Y, TRAY_RAIL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.30, lower=0.0, upper=0.200),
    )
    model.articulation(
        "body_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, -0.030, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=9.0),
    )
    model.articulation(
        "upper_rack_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, -0.225, -0.0275)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=9.0),
    )
    model.articulation(
        "third_tray_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=third_tray,
        child=wing_0,
        origin=Origin(xyz=(-0.230, -0.235, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.5, lower=0.0, upper=math.radians(72.0)),
    )
    model.articulation(
        "third_tray_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=third_tray,
        child=wing_1,
        origin=Origin(xyz=(0.230, -0.235, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.5, lower=0.0, upper=math.radians(72.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    button_0 = object_model.get_part("button_0")
    detergent_cover = object_model.get_part("detergent_cover")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    third_tray = object_model.get_part("third_tray")
    wing_0 = object_model.get_part("wing_0")
    lower_spray_arm = object_model.get_part("lower_spray_arm")
    upper_spray_arm = object_model.get_part("upper_spray_arm")

    ctx.allow_overlap(
        detergent_cover,
        door,
        elem_a="cover_hinge_barrel",
        elem_b="door_shell",
        reason="The detergent lid hinge barrel is intentionally nested into the molded dispenser housing on the inner door.",
    )
    ctx.allow_overlap(
        body,
        lower_spray_arm,
        elem_a="body_shell",
        elem_b="hub",
        reason="The lower spray arm hub is intentionally represented as nested over the sump spindle.",
    )
    ctx.allow_overlap(
        upper_rack,
        upper_spray_arm,
        elem_a="rack_shell",
        elem_b="hub",
        reason="The upper spray arm hub is intentionally represented as nested into the rack-mounted spray support.",
    )
    ctx.allow_overlap(
        body,
        third_tray,
        elem_a="tray_rail_left",
        elem_b="tray_shell",
        reason="The narrow third-tray runners are simplified as a retained slide fit inside the left guide rail proxy.",
    )
    ctx.allow_overlap(
        body,
        third_tray,
        elem_a="tray_rail_right",
        elem_b="tray_shell",
        reason="The narrow third-tray runners are simplified as a retained slide fit inside the right guide rail proxy.",
    )
    ctx.allow_overlap(
        third_tray,
        wing_0,
        elem_a="tray_shell",
        elem_b="wing_shell",
        reason="The left utensil wing uses a simplified nested side pivot inside the tray hinge block.",
    )
    ctx.allow_overlap(
        third_tray,
        object_model.get_part("wing_1"),
        elem_a="tray_shell",
        elem_b="wing_shell",
        reason="The right utensil wing uses a simplified nested side pivot inside the tray hinge block.",
    )

    door_joint = object_model.get_articulation("body_to_door")
    lower_rack_joint = object_model.get_articulation("body_to_lower_rack")
    upper_rack_joint = object_model.get_articulation("body_to_upper_rack")
    third_tray_joint = object_model.get_articulation("body_to_third_tray")
    button_joint = object_model.get_articulation("door_to_button_0")
    detergent_joint = object_model.get_articulation("door_to_detergent_cover")
    wing_joint = object_model.get_articulation("third_tray_to_wing_0")

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.40,
            name="closed door spans the dishwasher front opening",
        )

        door_aabb = ctx.part_world_aabb(door)
        body_aabb = ctx.part_world_aabb(body)
        front_ok = (
            door_aabb is not None
            and body_aabb is not None
            and door_aabb[1][1] > body_aabb[1][1] + 0.010
        )
        ctx.check(
            "closed door sits proud of the tub shell",
            front_ok,
            details=f"door_aabb={door_aabb}, body_aabb={body_aabb}",
        )

        button_rest = ctx.part_world_position(button_0)
        with ctx.pose({button_joint: 0.004}):
            button_pressed = ctx.part_world_position(button_0)
        ctx.check(
            "control button presses into the hidden strip",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[2] < button_rest[2] - 0.0015,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

        cover_rest = ctx.part_world_aabb(detergent_cover)
        with ctx.pose({detergent_joint: math.radians(110.0)}):
            cover_open = ctx.part_world_aabb(detergent_cover)
        ctx.check(
            "detergent cover swings off the inner door liner",
            cover_rest is not None
            and cover_open is not None
            and cover_open[0][1] < cover_rest[0][1] - 0.015,
            details=f"rest={cover_rest}, open={cover_open}",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: math.radians(88.0)}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens downward and outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.12
        and open_aabb[1][2] < closed_aabb[1][2] - 0.50,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for rack, joint, travel, name in (
        (lower_rack, lower_rack_joint, 0.260, "lower rack"),
        (upper_rack, upper_rack_joint, 0.240, "upper rack"),
        (third_tray, third_tray_joint, 0.200, "third tray"),
    ):
        ctx.expect_within(
            rack,
            body,
            axes="xz",
            margin=0.0,
            name=f"{name} stays centered in the tub opening",
        )
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            min_overlap=0.10,
            name=f"{name} remains inserted at rest",
        )
        rest_pos = ctx.part_world_position(rack)
        with ctx.pose({joint: travel}):
            ctx.expect_within(
                rack,
                body,
                axes="xz",
                margin=0.0,
                name=f"{name} stays centered when extended",
            )
            ctx.expect_overlap(
                rack,
                body,
                axes="y",
                min_overlap=0.05,
                name=f"{name} retains insertion when extended",
            )
            extended_pos = ctx.part_world_position(rack)
        ctx.check(
            f"{name} slides forward",
            rest_pos is not None and extended_pos is not None and extended_pos[1] > rest_pos[1] + 0.12,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    wing_rest = ctx.part_world_aabb(wing_0)
    with ctx.pose({wing_joint: math.radians(68.0)}):
        wing_folded = ctx.part_world_aabb(wing_0)
    ctx.check(
        "utensil wing folds downward",
        wing_rest is not None
        and wing_folded is not None
        and wing_folded[0][2] < wing_rest[0][2] - 0.020,
        details=f"rest={wing_rest}, folded={wing_folded}",
    )

    return ctx.report()


object_model = build_object_model()
