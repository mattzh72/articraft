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


BODY_W = 0.60
BODY_D = 0.62
BODY_H = 0.86
SIDE_WALL = 0.035
BACK_WALL = 0.035
ROOF_WALL = 0.030
FLOOR_WALL = 0.060
FASCIA_RADIUS = 0.035
DOOR_W = BODY_W - 0.030
DOOR_H = BODY_H - 0.110
DOOR_T = 0.040
DOOR_HINGE_Z = 0.065
RUNNER_W = 0.022
RUNNER_H = 0.010
LOWER_RACK_W = 0.490
LOWER_RACK_D = 0.450
LOWER_RACK_H = 0.170
LOWER_RACK_Y = -0.020
LOWER_RACK_Z = 0.208
LOWER_RUNNER_Z = LOWER_RACK_Z - LOWER_RACK_H / 2.0 - RUNNER_H / 2.0
LOWER_RACK_TRAVEL = 0.220
UPPER_RACK_W = 0.490
UPPER_RACK_D = 0.420
UPPER_RACK_H = 0.120
UPPER_RACK_Y = -0.030
UPPER_RACK_Z = 0.555
UPPER_RUNNER_Z = UPPER_RACK_Z - UPPER_RACK_H / 2.0 - RUNNER_H / 2.0
UPPER_RACK_TRAVEL = 0.200
LOWER_SPRAY_Z = 0.118
UPPER_SPRAY_LOCAL_Z = -0.078
CUP_SHELF_X = UPPER_RACK_W / 2.0 - 0.018
CUP_SHELF_Y = 0.055
CUP_SHELF_Z = 0.020


def _box_solid(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Solid:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Solid.makeBox(
        sx,
        sy,
        sz,
        pnt=cq.Vector(cx - sx / 2.0, cy - sy / 2.0, cz - sz / 2.0),
    )


def _cylinder_solid(
    length: float,
    radius: float,
    start: tuple[float, float, float],
    direction: tuple[float, float, float],
) -> cq.Solid:
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start), cq.Vector(*direction))


def _union_solids(solids: list[cq.Solid]) -> cq.Shape:
    shape: cq.Shape = solids[0]
    for solid in solids[1:]:
        shape = shape.fuse(solid)
    return shape


def _cabinet_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .faces("<Y").edges("|Z").fillet(FASCIA_RADIUS)
        .faces("<Y").edges("|X").fillet(0.018)
    )

    cavity = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * SIDE_WALL,
            BODY_D - BACK_WALL,
            BODY_H - FLOOR_WALL - ROOF_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, -BACK_WALL / 2.0, FLOOR_WALL))
    )

    plinth_recess = (
        cq.Workplane("XY")
        .box(BODY_W - 0.080, 0.060, 0.090, centered=(True, True, False))
        .translate((0.0, BODY_D / 2.0 - 0.030, 0.0))
    )

    return outer.cut(cavity).cut(plinth_recess)


def _door_panel() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H, centered=(True, True, False))
        .translate((0.0, -DOOR_T / 2.0, 0.0))
        .edges("|X").fillet(0.010)
        .faces("<Y").edges("|Z").fillet(0.018)
    )

    inner_recess = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.090, DOOR_T - 0.012, DOOR_H - 0.140, centered=(True, True, False))
        .translate((0.0, -0.002, 0.070))
    )

    handle = (
        cq.Workplane("XY")
        .box(DOOR_W * 0.52, 0.022, 0.028, centered=(True, True, True))
        .translate((0.0, -DOOR_T - 0.006, DOOR_H - 0.075))
        .edges("|X").fillet(0.010)
    )

    return panel.cut(inner_recess).union(handle)


def _wire_rack(width: float, depth: float, height: float, *, cross_wires: int) -> cq.Workplane:
    wall = 0.008
    floor = 0.006
    support_w = 0.018
    support_h = 0.010
    support_len = depth * 0.92

    basket = cq.Workplane("XY").box(width, depth, height, centered=(True, True, True))
    basket = basket.cut(
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height - 0.020, centered=(True, True, True))
        .translate((0.0, 0.0, 0.012))
    )
    basket = basket.cut(
        cq.Workplane("XY")
        .box(width - 0.060, wall * 2.6, height - 0.090, centered=(True, True, True))
        .translate((0.0, -depth / 2.0 + wall * 1.2, 0.0))
    )
    basket = basket.cut(
        cq.Workplane("XY")
        .box(wall * 2.6, depth - 0.090, height - 0.060, centered=(True, True, True))
        .translate((width / 2.0 - wall * 1.2, 0.0, 0.006))
    )
    basket = basket.cut(
        cq.Workplane("XY")
        .box(wall * 2.6, depth - 0.090, height - 0.060, centered=(True, True, True))
        .translate((-width / 2.0 + wall * 1.2, 0.0, 0.006))
    )

    slot_height = floor + 0.004
    for index in range(max(cross_wires - 1, 1)):
        x = -width / 2.0 + 0.090 + index * (width - 0.180) / max(cross_wires - 2, 1)
        basket = basket.cut(
            cq.Workplane("XY")
            .box(0.020, depth - 0.100, slot_height, centered=(True, True, True))
            .translate((x, 0.0, -height / 2.0 + floor / 2.0 + 0.001))
        )

    basket = basket.union(
        cq.Workplane("XY")
        .box(support_w, support_len, support_h, centered=(True, True, True))
        .translate((width / 2.0 - support_w / 2.0 - 0.001, 0.0, -height / 2.0 + support_h / 2.0))
    )
    basket = basket.union(
        cq.Workplane("XY")
        .box(support_w, support_len, support_h, centered=(True, True, True))
        .translate((-width / 2.0 + support_w / 2.0 + 0.001, 0.0, -height / 2.0 + support_h / 2.0))
    )
    basket = basket.union(
        cq.Workplane("XY")
        .box(width * 0.68, 0.010, 0.010, centered=(True, True, True))
        .translate((0.0, -depth / 2.0 + 0.005, height / 2.0 + 0.028))
    )
    basket = basket.union(
        cq.Workplane("XY")
        .box(0.010, 0.010, 0.030, centered=(True, True, True))
        .translate((-width * 0.34, -depth / 2.0 + 0.005, height / 2.0 + 0.013))
    )
    basket = basket.union(
        cq.Workplane("XY")
        .box(0.010, 0.010, 0.030, centered=(True, True, True))
        .translate((width * 0.34, -depth / 2.0 + 0.005, height / 2.0 + 0.013))
    )

    return basket


def _spray_arm(main_span: float, side_span: float) -> cq.Shape:
    solids: list[cq.Solid] = [
        _cylinder_solid(0.012, 0.018, (0.0, 0.0, -0.012), (0, 0, 1)),
        _box_solid((main_span, 0.020, 0.010), (0.0, 0.0, -0.015)),
        _box_solid((0.130, side_span, 0.009), (0.050, 0.0, -0.014)),
        _box_solid((0.040, 0.018, 0.010), (main_span / 2.0 - 0.018, 0.010, -0.015)),
        _box_solid((0.028, 0.016, 0.009), (-main_span / 2.0 + 0.022, -0.008, -0.015)),
        _box_solid((0.012, 0.010, 0.010), (main_span / 2.0 - 0.020, 0.014, -0.010)),
        _box_solid((0.010, 0.010, 0.008), (-main_span / 2.0 + 0.030, -0.012, -0.010)),
    ]
    return _union_solids(solids)


def _cup_shelf() -> cq.Workplane:
    shelf_x = 0.120
    shelf_y = 0.140
    plate = (
        cq.Workplane("XY")
        .box(shelf_x, shelf_y, 0.010, centered=(True, True, True))
        .translate((-shelf_x / 2.0, 0.0, -0.004))
    )
    for index in range(4):
        y = -shelf_y / 2.0 + 0.026 + index * 0.030
        plate = plate.cut(
            cq.Workplane("XY")
            .box(shelf_x - 0.030, 0.012, 0.016, centered=(True, True, True))
            .translate((-shelf_x / 2.0, y, -0.004))
        )

    hinge = cq.Workplane("XY").box(0.010, shelf_y, 0.014, centered=(True, True, True)).translate((-0.005, 0.0, 0.0))
    return plate.union(hinge)


def _dial_shape() -> cq.Shape:
    solids: list[cq.Solid] = [
        _cylinder_solid(0.010, 0.026, (0.0, 0.0, 0.0), (0, -1, 0)),
        _cylinder_solid(0.020, 0.021, (0.0, -0.002, 0.0), (0, -1, 0)),
        _box_solid((0.008, 0.014, 0.026), (0.0, -0.017, 0.015)),
        _box_solid((0.010, 0.012, 0.012), (0.0, -0.017, -0.010)),
    ]
    return _union_solids(solids)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_dishwasher")

    model.material("enamel_cream", rgba=(0.86, 0.84, 0.76, 1.0))
    model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    model.material("rack_coating", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("spray_gray", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("chrome", rgba=(0.83, 0.84, 0.86, 1.0))
    model.material("button_cream", rgba=(0.90, 0.88, 0.80, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "cabinet_shell"),
        material="enamel_cream",
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((RUNNER_W, 0.470, RUNNER_H)),
        origin=Origin(xyz=(INNER_W := BODY_W / 2.0 - SIDE_WALL - RUNNER_W / 2.0, -0.030, LOWER_RUNNER_Z)),
        material="rack_coating",
        name="right_lower_runner",
    )
    cabinet.visual(
        Box((RUNNER_W, 0.470, RUNNER_H)),
        origin=Origin(xyz=(-INNER_W, -0.030, LOWER_RUNNER_Z)),
        material="rack_coating",
        name="left_lower_runner",
    )
    cabinet.visual(
        Box((RUNNER_W, 0.430, RUNNER_H)),
        origin=Origin(xyz=(INNER_W, -0.030, UPPER_RUNNER_Z)),
        material="rack_coating",
        name="right_upper_runner",
    )
    cabinet.visual(
        Box((RUNNER_W, 0.430, RUNNER_H)),
        origin=Origin(xyz=(-INNER_W, -0.030, UPPER_RUNNER_Z)),
        material="rack_coating",
        name="left_upper_runner",
    )
    cabinet.visual(
        Cylinder(radius=0.012, length=LOWER_SPRAY_Z - FLOOR_WALL),
        origin=Origin(xyz=(0.0, 0.020, (LOWER_SPRAY_Z + FLOOR_WALL) / 2.0)),
        material="spray_gray",
        name="lower_spray_mount",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_panel(), "door_panel"),
        material="brushed_steel",
        name="door_panel",
    )
    door.visual(
        Box((0.160, 0.004, 0.150)),
        origin=Origin(xyz=(0.190, -DOOR_T - 0.002, DOOR_H - 0.130)),
        material="chrome",
        name="control_trim",
    )

    lower_rack = model.part("lower_rack")
    lower_rack.visual(
        mesh_from_cadquery(_wire_rack(LOWER_RACK_W, LOWER_RACK_D, LOWER_RACK_H, cross_wires=5), "lower_rack"),
        material="rack_coating",
        name="lower_rack_frame",
    )

    upper_rack = model.part("upper_rack")
    upper_rack.visual(
        mesh_from_cadquery(_wire_rack(UPPER_RACK_W, UPPER_RACK_D, UPPER_RACK_H, cross_wires=4), "upper_rack"),
        material="rack_coating",
        name="upper_rack_frame",
    )
    upper_rack.visual(
        Box((0.016, 0.140, 0.040)),
        origin=Origin(xyz=(CUP_SHELF_X + 0.008, CUP_SHELF_Y, CUP_SHELF_Z)),
        material="rack_coating",
        name="cup_shelf_bracket",
    )
    upper_rack.visual(
        Box((0.030, 0.024, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, UPPER_SPRAY_LOCAL_Z + 0.014)),
        material="spray_gray",
        name="upper_spray_mount",
    )

    lower_spray_arm = model.part("lower_spray_arm")
    lower_spray_arm.visual(
        mesh_from_cadquery(_spray_arm(0.260, 0.090), "lower_spray_arm"),
        material="spray_gray",
        name="lower_spray_arm",
    )

    upper_spray_arm = model.part("upper_spray_arm")
    upper_spray_arm.visual(
        mesh_from_cadquery(_spray_arm(0.220, 0.070), "upper_spray_arm"),
        material="spray_gray",
        name="upper_spray_arm",
    )

    cup_shelf = model.part("cup_shelf")
    cup_shelf.visual(
        mesh_from_cadquery(_cup_shelf(), "cup_shelf"),
        material="rack_coating",
        name="cup_shelf",
    )

    program_dial = model.part("program_dial")
    program_dial.visual(
        mesh_from_cadquery(_dial_shape(), "program_dial"),
        material="chrome",
        name="program_dial",
    )

    temperature_dial = model.part("temperature_dial")
    temperature_dial.visual(
        mesh_from_cadquery(_dial_shape(), "temperature_dial"),
        material="chrome",
        name="temperature_dial",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.026, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, -0.007, 0.0)),
        material="button_cream",
        name="start_button",
    )

    heat_button = model.part("heat_button")
    heat_button.visual(
        Box((0.026, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, -0.007, 0.0)),
        material="button_cream",
        name="heat_button",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.radians(95.0),
            effort=35.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "cabinet_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_rack,
        origin=Origin(xyz=(0.0, LOWER_RACK_Y, LOWER_RACK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=LOWER_RACK_TRAVEL, effort=60.0, velocity=0.40),
    )
    model.articulation(
        "cabinet_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_rack,
        origin=Origin(xyz=(0.0, UPPER_RACK_Y, UPPER_RACK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=UPPER_RACK_TRAVEL, effort=50.0, velocity=0.35),
    )
    model.articulation(
        "cabinet_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, 0.020, LOWER_SPRAY_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=15.0),
    )
    model.articulation(
        "upper_rack_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, 0.0, UPPER_SPRAY_LOCAL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=15.0),
    )
    model.articulation(
        "upper_rack_to_cup_shelf",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=cup_shelf,
        origin=Origin(xyz=(CUP_SHELF_X, CUP_SHELF_Y, CUP_SHELF_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.radians(105.0), effort=5.0, velocity=1.5),
    )
    model.articulation(
        "door_to_program_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=program_dial,
        origin=Origin(xyz=(0.220, -DOOR_T, DOOR_H - 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "door_to_temperature_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=temperature_dial,
        origin=Origin(xyz=(0.145, -DOOR_T, DOOR_H - 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "door_to_start_button",
        ArticulationType.PRISMATIC,
        parent=door,
        child=start_button,
        origin=Origin(xyz=(0.220, -DOOR_T, DOOR_H - 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.008, effort=8.0, velocity=0.10),
    )
    model.articulation(
        "door_to_heat_button",
        ArticulationType.PRISMATIC,
        parent=door,
        child=heat_button,
        origin=Origin(xyz=(0.145, -DOOR_T, DOOR_H - 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.008, effort=8.0, velocity=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("cabinet_to_door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    cup_shelf = object_model.get_part("cup_shelf")
    lower_spray_arm = object_model.get_part("lower_spray_arm")
    upper_spray_arm = object_model.get_part("upper_spray_arm")
    program_dial = object_model.get_part("program_dial")
    temperature_dial = object_model.get_part("temperature_dial")
    start_button = object_model.get_part("start_button")
    heat_button = object_model.get_part("heat_button")
    lower_rack_slide = object_model.get_articulation("cabinet_to_lower_rack")
    upper_rack_slide = object_model.get_articulation("cabinet_to_upper_rack")
    cup_shelf_hinge = object_model.get_articulation("upper_rack_to_cup_shelf")
    lower_spray_joint = object_model.get_articulation("cabinet_to_lower_spray_arm")
    program_dial_joint = object_model.get_articulation("door_to_program_dial")
    start_button_joint = object_model.get_articulation("door_to_start_button")

    ctx.allow_overlap(
        cabinet,
        lower_spray_arm,
        elem_a="lower_spray_mount",
        elem_b="lower_spray_arm",
        reason="The lower spray arm hub is intentionally simplified around the spindle mount.",
    )
    ctx.allow_overlap(
        door,
        program_dial,
        reason="The cycle dial is represented as a through-mounted control on the simplified solid door skin.",
    )
    ctx.allow_overlap(
        door,
        temperature_dial,
        reason="The temperature dial is represented as a through-mounted control on the simplified solid door skin.",
    )
    ctx.allow_overlap(
        door,
        start_button,
        reason="The start button is represented as a panel-mounted push control on the simplified solid door skin.",
    )
    ctx.allow_overlap(
        door,
        heat_button,
        reason="The heat button is represented as a panel-mounted push control on the simplified solid door skin.",
    )

    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.45,
        name="door spans the cabinet front",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    upper = hinge.motion_limits.upper if hinge.motion_limits is not None else None
    if upper is not None:
        with ctx.pose({hinge: upper}):
            open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.check(
            "door drops downward when opened",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] < closed_aabb[1][2] - 0.45
            and open_aabb[0][1] < closed_aabb[0][1] - 0.20,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    lower_rack_rest = ctx.part_world_position(lower_rack)
    if lower_rack_slide.motion_limits is not None and lower_rack_slide.motion_limits.upper is not None:
        with ctx.pose({lower_rack_slide: lower_rack_slide.motion_limits.upper}):
            lower_rack_ext = ctx.part_world_position(lower_rack)
            ctx.expect_overlap(
                lower_rack,
                cabinet,
                axes="y",
                min_overlap=0.18,
                name="lower rack stays retained in the tub when extended",
            )
        ctx.check(
            "lower rack pulls forward",
            lower_rack_rest is not None
            and lower_rack_ext is not None
            and lower_rack_ext[1] < lower_rack_rest[1] - 0.18,
            details=f"rest={lower_rack_rest}, extended={lower_rack_ext}",
        )

    upper_rack_rest = ctx.part_world_position(upper_rack)
    upper_spray_rest = ctx.part_world_position(upper_spray_arm)
    if upper_rack_slide.motion_limits is not None and upper_rack_slide.motion_limits.upper is not None:
        with ctx.pose({upper_rack_slide: upper_rack_slide.motion_limits.upper}):
            upper_rack_ext = ctx.part_world_position(upper_rack)
            upper_spray_ext = ctx.part_world_position(upper_spray_arm)
            ctx.expect_overlap(
                upper_rack,
                cabinet,
                axes="y",
                min_overlap=0.16,
                name="upper rack stays retained in the tub when extended",
            )
        ctx.check(
            "upper rack pulls forward",
            upper_rack_rest is not None
            and upper_rack_ext is not None
            and upper_rack_ext[1] < upper_rack_rest[1] - 0.16,
            details=f"rest={upper_rack_rest}, extended={upper_rack_ext}",
        )
        ctx.check(
            "upper spray arm rides with the upper rack",
            upper_spray_rest is not None
            and upper_spray_ext is not None
            and upper_spray_ext[1] < upper_spray_rest[1] - 0.16,
            details=f"rest={upper_spray_rest}, extended={upper_spray_ext}",
        )

    shelf_rest = ctx.part_element_world_aabb(cup_shelf, elem="cup_shelf")
    if cup_shelf_hinge.motion_limits is not None and cup_shelf_hinge.motion_limits.upper is not None:
        with ctx.pose({cup_shelf_hinge: cup_shelf_hinge.motion_limits.upper}):
            shelf_folded = ctx.part_element_world_aabb(cup_shelf, elem="cup_shelf")
        ctx.check(
            "cup shelf folds upward on its pivots",
            shelf_rest is not None
            and shelf_folded is not None
            and shelf_folded[1][2] > shelf_rest[1][2] + 0.05,
            details=f"rest={shelf_rest}, folded={shelf_folded}",
        )

    spray_rest = ctx.part_element_world_aabb(lower_spray_arm, elem="lower_spray_arm")
    with ctx.pose({lower_spray_joint: 1.2}):
        spray_rotated = ctx.part_element_world_aabb(lower_spray_arm, elem="lower_spray_arm")
    ctx.check(
        "lower spray arm can rotate continuously",
        spray_rest is not None
        and spray_rotated is not None
        and (
            abs(spray_rotated[0][0] - spray_rest[0][0]) > 0.005
            or abs(spray_rotated[1][1] - spray_rest[1][1]) > 0.005
        ),
        details=f"rest={spray_rest}, rotated={spray_rotated}",
    )

    dial_rest = ctx.part_element_world_aabb(program_dial, elem="program_dial")
    with ctx.pose({program_dial_joint: 1.0}):
        dial_rotated = ctx.part_element_world_aabb(program_dial, elem="program_dial")
    ctx.check(
        "program dial rotates about its shaft",
        dial_rest is not None
        and dial_rotated is not None
        and (
            abs(dial_rotated[1][0] - dial_rest[1][0]) > 0.004
            or abs(dial_rotated[1][2] - dial_rest[1][2]) > 0.004
        ),
        details=f"rest={dial_rest}, rotated={dial_rotated}",
    )

    button_rest = ctx.part_world_position(start_button)
    if start_button_joint.motion_limits is not None and start_button_joint.motion_limits.upper is not None:
        with ctx.pose({start_button_joint: start_button_joint.motion_limits.upper}):
            button_pressed = ctx.part_world_position(start_button)
        ctx.check(
            "start button presses inward",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[1] > button_rest[1] + 0.005,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
