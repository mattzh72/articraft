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


DISHWASHER_WIDTH = 0.598
DISHWASHER_DEPTH = 0.570
DISHWASHER_HEIGHT = 0.820
TUB_SIDE_WALL = 0.012
TUB_BACK_WALL = 0.012
TUB_TOP_WALL = 0.012
TUB_BOTTOM_WALL = 0.015
DOOR_THICKNESS = 0.060
DOOR_FRONT_SKIN = 0.004
DOOR_EDGE_WALL = 0.018
CONTROL_STRIP_HEIGHT = 0.070
CONTROL_STRIP_DEPTH = 0.006

BUTTON_X = (-0.175, -0.127, -0.079)
BUTTON_Z = DISHWASHER_HEIGHT - 0.037
BUTTON_WIDTH = 0.034
BUTTON_HEIGHT = 0.022
BUTTON_CAP_DEPTH = 0.006
BUTTON_STEM_DEPTH = 0.019
BUTTON_TRAVEL = 0.0035

KNOB_X = 0.188
KNOB_Z = DISHWASHER_HEIGHT - 0.037
KNOB_RADIUS = 0.020
KNOB_DEPTH = 0.018

LOWER_RAIL_Y = -0.245
LOWER_RAIL_Z = 0.185
LOWER_RACK_TRAVEL = 0.320

UPPER_RAIL_Y = -0.235
UPPER_RAIL_Z = 0.415
UPPER_RACK_TRAVEL = 0.300


def _wp_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _wp_cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate(
        (center[0], center[1], center[2] - length * 0.5)
    )


def _combine(shapes: list[cq.Workplane]) -> cq.Workplane:
    solid = shapes[0]
    for shape in shapes[1:]:
        solid = solid.union(shape)
    return solid


def _build_body_mesh() -> cq.Workplane:
    inner_width = DISHWASHER_WIDTH - 2.0 * TUB_SIDE_WALL
    inner_height = DISHWASHER_HEIGHT - TUB_BOTTOM_WALL - TUB_TOP_WALL

    outer = _wp_box(
        (DISHWASHER_WIDTH, DISHWASHER_DEPTH, DISHWASHER_HEIGHT),
        (0.0, 0.0, DISHWASHER_HEIGHT * 0.5),
    )
    cavity = _wp_box(
        (
            inner_width,
            DISHWASHER_DEPTH - TUB_BACK_WALL + 0.030,
            inner_height,
        ),
        (
            0.0,
            -(TUB_BACK_WALL + 0.030) * 0.5,
            TUB_BOTTOM_WALL + inner_height * 0.5,
        ),
    )
    body = outer.cut(cavity)

    lower_rail_length = 0.495
    upper_rail_length = 0.455
    rail_width = 0.012
    rail_height = 0.008
    rail_x = inner_width * 0.5 - rail_width * 0.5

    for x in (-(rail_x + 0.001), rail_x + 0.001):
        body = body.union(
            _wp_box(
                (rail_width, lower_rail_length, rail_height),
                (x, LOWER_RAIL_Y + lower_rail_length * 0.5, LOWER_RAIL_Z - rail_height * 0.5),
            )
        )
        body = body.union(
            _wp_box(
                (rail_width, upper_rail_length, rail_height),
                (x, UPPER_RAIL_Y + upper_rail_length * 0.5, UPPER_RAIL_Z - rail_height * 0.5),
            )
        )

    body = body.union(_wp_cylinder_z(0.013, 0.018, (0.0, 0.025, 0.078)))
    body = body.union(_wp_cylinder_z(0.011, 0.016, (0.0, 0.040, 0.336)))
    return body


def _build_door_mesh() -> cq.Workplane:
    outer = _wp_box(
        (DISHWASHER_WIDTH, DOOR_THICKNESS, DISHWASHER_HEIGHT),
        (0.0, -DOOR_THICKNESS * 0.5, DISHWASHER_HEIGHT * 0.5),
    )
    cavity = _wp_box(
        (
            DISHWASHER_WIDTH - 2.0 * DOOR_EDGE_WALL,
            DOOR_THICKNESS - DOOR_FRONT_SKIN + 0.010,
            DISHWASHER_HEIGHT - 0.020 - 0.028,
        ),
        (
            0.0,
            (-DOOR_THICKNESS + DOOR_FRONT_SKIN + 0.010) * 0.5,
            0.028 + (DISHWASHER_HEIGHT - 0.020 - 0.028) * 0.5,
        ),
    )
    door = outer.cut(cavity)

    control_strip = _wp_box(
        (DISHWASHER_WIDTH - 0.046, CONTROL_STRIP_DEPTH, CONTROL_STRIP_HEIGHT),
        (0.0, -DOOR_THICKNESS - CONTROL_STRIP_DEPTH * 0.5, DISHWASHER_HEIGHT - CONTROL_STRIP_HEIGHT * 0.5),
    )
    door = door.union(control_strip)

    for button_x in BUTTON_X:
        bezel_cut = _wp_box(
            (BUTTON_WIDTH - 0.012, CONTROL_STRIP_DEPTH + 0.002, BUTTON_HEIGHT - 0.010),
            (
                button_x,
                -DOOR_THICKNESS - CONTROL_STRIP_DEPTH * 0.5,
                BUTTON_Z,
            ),
        )
        door_cut = _wp_box(
            (BUTTON_WIDTH + 0.004, 0.040, BUTTON_HEIGHT + 0.004),
            (button_x, -DOOR_THICKNESS + 0.020, BUTTON_Z),
        )
        door = door.cut(bezel_cut).cut(door_cut)

    knob_clear = _wp_box(
        (0.028, 0.040, 0.028),
        (KNOB_X, -DOOR_THICKNESS + 0.020, KNOB_Z),
    )
    knob_bezel = _wp_box(
        (0.020, CONTROL_STRIP_DEPTH + 0.002, 0.020),
        (KNOB_X, -DOOR_THICKNESS - CONTROL_STRIP_DEPTH * 0.5, KNOB_Z),
    )
    door = door.cut(knob_clear).cut(knob_bezel)

    vent_slot = _wp_box(
        (0.138, 0.010, 0.008),
        (
            0.0,
            -DOOR_THICKNESS + 0.009,
            DISHWASHER_HEIGHT - 0.072,
        ),
    )
    door = door.cut(vent_slot)
    return door


def _build_rack_mesh(
    width: float,
    depth: float,
    height: float,
    *,
    top_z: float,
    floor_z: float,
    tine_rows: tuple[float, ...],
    tine_height: float,
) -> cq.Workplane:
    rod = 0.006
    fuse = 0.002
    side_x = width * 0.5 - rod * 0.5
    runner_x = 0.271
    runner_width = 0.010
    front_y = -fuse
    rear_y = depth - rod

    shapes: list[cq.Workplane] = [
        _wp_box((runner_width, depth + 2.0 * fuse, rod), (-runner_x, depth * 0.5 - fuse, rod * 0.5)),
        _wp_box((runner_width, depth + 2.0 * fuse, rod), (runner_x, depth * 0.5 - fuse, rod * 0.5)),
        _wp_box(
            (rod + fuse, rod + fuse, top_z + rod + fuse),
            (-side_x, front_y + rod * 0.5, top_z * 0.5 + rod * 0.5),
        ),
        _wp_box(
            (rod + fuse, rod + fuse, top_z + rod + fuse),
            (side_x, front_y + rod * 0.5, top_z * 0.5 + rod * 0.5),
        ),
        _wp_box(
            (rod + fuse, rod + fuse, top_z + rod + fuse),
            (-side_x, rear_y + rod * 0.5 + fuse, top_z * 0.5 + rod * 0.5),
        ),
        _wp_box(
            (rod + fuse, rod + fuse, top_z + rod + fuse),
            (side_x, rear_y + rod * 0.5 + fuse, top_z * 0.5 + rod * 0.5),
        ),
        _wp_box((width - 2.0 * rod + 2.0 * fuse, rod + fuse, rod + fuse), (0.0, front_y + rod * 0.5, floor_z)),
        _wp_box(
            (width - 2.0 * rod + 2.0 * fuse, rod + fuse, rod + fuse),
            (0.0, rear_y + rod * 0.5 + fuse, floor_z),
        ),
        _wp_box((rod + fuse, depth + 2.0 * fuse, rod + fuse), (-side_x, depth * 0.5 - fuse, floor_z)),
        _wp_box((rod + fuse, depth + 2.0 * fuse, rod + fuse), (side_x, depth * 0.5 - fuse, floor_z)),
        _wp_box(
            (width - 2.0 * rod + 2.0 * fuse, rod + fuse, rod + fuse),
            (0.0, front_y + rod * 0.5, top_z),
        ),
        _wp_box(
            (width - 2.0 * rod + 2.0 * fuse, rod + fuse, rod + fuse),
            (0.0, rear_y + rod * 0.5 + fuse, top_z),
        ),
        _wp_box((rod + fuse, depth + 2.0 * fuse, rod + fuse), (-side_x, depth * 0.5 - fuse, top_z)),
        _wp_box((rod + fuse, depth + 2.0 * fuse, rod + fuse), (side_x, depth * 0.5 - fuse, top_z)),
        _wp_box((width - 0.040, rod + fuse, rod + fuse), (0.0, depth * 0.5, floor_z)),
        _wp_box((rod + fuse, depth - 0.050, rod + fuse), (0.0, depth * 0.5, floor_z)),
        _wp_box(
            (runner_x - side_x + runner_width, rod + fuse, rod + fuse),
            (-(runner_x + side_x) * 0.5, 0.020, rod * 0.5),
        ),
        _wp_box(
            (runner_x - side_x + runner_width, rod + fuse, rod + fuse),
            ((runner_x + side_x) * 0.5, 0.020, rod * 0.5),
        ),
        _wp_box(
            (runner_x - side_x + runner_width, rod + fuse, rod + fuse),
            (-(runner_x + side_x) * 0.5, depth - 0.020, rod * 0.5),
        ),
        _wp_box(
            (runner_x - side_x + runner_width, rod + fuse, rod + fuse),
            ((runner_x + side_x) * 0.5, depth - 0.020, rod * 0.5),
        ),
    ]

    return _combine(shapes)


def _build_spray_arm_mesh(length: float, branch_length: float) -> cq.Workplane:
    cap = _wp_cylinder_z(0.026, 0.012, (0.0, 0.0, 0.006))
    main_bar = _wp_box((length, 0.018, 0.008), (0.0, 0.0, 0.004))
    branch = _wp_box((0.020, branch_length, 0.008), (0.055, 0.0, 0.004))
    nozzle_0 = _wp_box((0.032, 0.016, 0.008), (-length * 0.5 + 0.024, 0.010, 0.004))
    nozzle_1 = _wp_box((0.032, 0.016, 0.008), (length * 0.5 - 0.024, -0.010, 0.004))
    return _combine([cap, main_bar, branch, nozzle_0, nozzle_1])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_dishwasher")

    steel = model.material("steel", rgba=(0.74, 0.77, 0.79, 1.0))
    liner = model.material("liner", rgba=(0.82, 0.84, 0.86, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    rack_grey = model.material("rack_grey", rgba=(0.65, 0.67, 0.70, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_black = model.material("satin_black", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_mesh(), "dishwasher_body"),
        material=liner,
        name="body_shell",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_build_door_mesh(), "dishwasher_door"),
        material=steel,
        name="door_shell",
    )

    lower_rack = model.part("lower_rack")
    lower_rack.visual(
        mesh_from_cadquery(
            _build_rack_mesh(
                width=0.520,
                depth=0.470,
                height=0.118,
                top_z=0.104,
                floor_z=0.032,
                tine_rows=(0.185, 0.305),
                tine_height=0.058,
            ),
            "lower_rack",
        ),
        material=rack_grey,
        name="rack",
    )

    upper_rack = model.part("upper_rack")
    upper_rack.visual(
        mesh_from_cadquery(
            _build_rack_mesh(
                width=0.505,
                depth=0.435,
                height=0.088,
                top_z=0.074,
                floor_z=0.026,
                tine_rows=(0.160, 0.280),
                tine_height=0.040,
            ),
            "upper_rack",
        ),
        material=rack_grey,
        name="rack",
    )

    lower_spray_arm = model.part("lower_spray_arm")
    lower_spray_arm.visual(
        mesh_from_cadquery(_build_spray_arm_mesh(0.330, 0.145), "lower_spray_arm"),
        material=dark_plastic,
        name="arm",
    )

    upper_spray_arm = model.part("upper_spray_arm")
    upper_spray_arm.visual(
        mesh_from_cadquery(_build_spray_arm_mesh(0.250, 0.110), "upper_spray_arm"),
        material=dark_plastic,
        name="arm",
    )

    vent_flap = model.part("vent_flap")
    vent_flap.visual(
        Box((0.168, 0.003, 0.005)),
        origin=Origin(xyz=(0.0, 0.0015, -0.0025)),
        material=graphite,
        name="hinge_barrel",
    )
    vent_flap.visual(
        Box((0.166, 0.0035, 0.032)),
        origin=Origin(xyz=(0.0, 0.00175, -0.019)),
        material=graphite,
        name="flap_panel",
    )

    cycle_knob = model.part("cycle_knob")
    cycle_knob.visual(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_DEPTH),
        origin=Origin(xyz=(0.0, -KNOB_DEPTH * 0.5, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=satin_black,
        name="knob_body",
    )
    cycle_knob.visual(
        Box((0.014, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=satin_black,
        name="knob_shaft",
    )
    cycle_knob.visual(
        Box((0.004, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, -KNOB_DEPTH + 0.002, KNOB_RADIUS * 0.45)),
        material=graphite,
        name="knob_pointer",
    )

    for index in range(3):
        button = model.part(f"option_button_{index}")
        button.visual(
            Box((BUTTON_WIDTH, BUTTON_CAP_DEPTH, BUTTON_HEIGHT)),
            origin=Origin(xyz=(0.0, -BUTTON_CAP_DEPTH * 0.5, 0.0)),
            material=graphite,
            name="button_cap",
        )
        button.visual(
            Box((BUTTON_WIDTH - 0.016, BUTTON_STEM_DEPTH, BUTTON_HEIGHT - 0.012)),
            origin=Origin(xyz=(0.0, BUTTON_STEM_DEPTH * 0.5, 0.0)),
            material=graphite,
            name="button_stem",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -DISHWASHER_DEPTH * 0.5, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "body_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.0, LOWER_RAIL_Y, LOWER_RAIL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=LOWER_RACK_TRAVEL),
    )
    model.articulation(
        "body_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.0, UPPER_RAIL_Y, UPPER_RAIL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=0.0, upper=UPPER_RACK_TRAVEL),
    )
    model.articulation(
        "body_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, 0.025, 0.087)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=12.0),
    )
    model.articulation(
        "body_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, 0.040, 0.344)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=12.0),
    )
    model.articulation(
        "door_to_vent_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=vent_flap,
        origin=Origin(xyz=(0.0, -DOOR_THICKNESS + DOOR_FRONT_SKIN, DISHWASHER_HEIGHT - 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=0.75),
    )
    model.articulation(
        "door_to_cycle_knob",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=cycle_knob,
        origin=Origin(xyz=(KNOB_X, -DOOR_THICKNESS - CONTROL_STRIP_DEPTH, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    for index, button_x in enumerate(BUTTON_X):
        model.articulation(
            f"door_to_option_button_{index}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=model.get_part(f"option_button_{index}"),
            origin=Origin(xyz=(button_x, -DOOR_THICKNESS - CONTROL_STRIP_DEPTH, BUTTON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
        )

    return model


def _aabb_y_min(aabb):
    return aabb[0][1] if aabb is not None else None


def _aabb_z_max(aabb):
    return aabb[1][2] if aabb is not None else None


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    lower_spray_arm = object_model.get_part("lower_spray_arm")
    upper_spray_arm = object_model.get_part("upper_spray_arm")
    vent_flap = object_model.get_part("vent_flap")
    cycle_knob = object_model.get_part("cycle_knob")
    buttons = [object_model.get_part(f"option_button_{index}") for index in range(3)]

    door_joint = object_model.get_articulation("body_to_door")
    lower_rack_joint = object_model.get_articulation("body_to_lower_rack")
    upper_rack_joint = object_model.get_articulation("body_to_upper_rack")
    lower_arm_joint = object_model.get_articulation("body_to_lower_spray_arm")
    upper_arm_joint = object_model.get_articulation("body_to_upper_spray_arm")
    vent_joint = object_model.get_articulation("door_to_vent_flap")
    knob_joint = object_model.get_articulation("door_to_cycle_knob")
    button_joints = [object_model.get_articulation(f"door_to_option_button_{index}") for index in range(3)]

    ctx.allow_overlap(
        body,
        lower_rack,
        reason="The lower rack runners are intentionally represented as sliding within simplified integrated side-rail ledges on the tub shell.",
    )
    ctx.allow_overlap(
        body,
        upper_rack,
        reason="The upper rack runners are intentionally represented as sliding within simplified integrated side-rail ledges on the tub shell.",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            max_gap=0.0015,
            max_penetration=0.0,
            name="door closes flush to the tub opening",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.55,
            name="door covers the front opening",
        )

    closed_aabb = ctx.part_world_aabb(door)
    door_limits = door_joint.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_joint: door_limits.upper}):
            open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door drops outward when opened",
            closed_aabb is not None
            and open_aabb is not None
            and _aabb_y_min(open_aabb) is not None
            and _aabb_y_min(closed_aabb) is not None
            and _aabb_z_max(open_aabb) is not None
            and _aabb_z_max(closed_aabb) is not None
            and _aabb_y_min(open_aabb) < _aabb_y_min(closed_aabb) - 0.18
            and _aabb_z_max(open_aabb) < _aabb_z_max(closed_aabb) - 0.35,
            details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
        )

    ctx.expect_contact(lower_rack, body, name="lower rack sits on the body rails")
    ctx.expect_contact(upper_rack, body, name="upper rack sits on the body rails")
    ctx.expect_contact(lower_spray_arm, body, name="lower spray arm stays mounted on its hub")
    ctx.expect_contact(upper_spray_arm, body, name="upper spray arm stays mounted on its hub")
    ctx.expect_contact(vent_flap, door, name="vent flap is mounted to the inner door liner")
    ctx.expect_contact(cycle_knob, door, name="cycle knob seats on the control strip")
    for index, button in enumerate(buttons):
        ctx.expect_contact(button, door, name=f"option button {index} seats in the control strip")

    lower_rest = ctx.part_world_position(lower_rack)
    lower_limits = lower_rack_joint.motion_limits
    if lower_limits is not None and lower_limits.upper is not None:
        with ctx.pose({lower_rack_joint: lower_limits.upper}):
            ctx.expect_within(
                lower_rack,
                body,
                axes="xz",
                margin=0.035,
                name="lower rack stays guided between the tub walls when extended",
            )
            ctx.expect_overlap(
                lower_rack,
                body,
                axes="y",
                min_overlap=0.14,
                name="lower rack keeps retained insertion when extended",
            )
            lower_extended = ctx.part_world_position(lower_rack)
        ctx.check(
            "lower rack slides outward",
            lower_rest is not None
            and lower_extended is not None
            and lower_extended[1] < lower_rest[1] - 0.20,
            details=f"rest={lower_rest}, extended={lower_extended}",
        )

    upper_rest = ctx.part_world_position(upper_rack)
    upper_limits = upper_rack_joint.motion_limits
    if upper_limits is not None and upper_limits.upper is not None:
        with ctx.pose({upper_rack_joint: upper_limits.upper}):
            ctx.expect_within(
                upper_rack,
                body,
                axes="xz",
                margin=0.035,
                name="upper rack stays guided between the tub walls when extended",
            )
            ctx.expect_overlap(
                upper_rack,
                body,
                axes="y",
                min_overlap=0.12,
                name="upper rack keeps retained insertion when extended",
            )
            upper_extended = ctx.part_world_position(upper_rack)
        ctx.check(
            "upper rack slides outward",
            upper_rest is not None
            and upper_extended is not None
            and upper_extended[1] < upper_rest[1] - 0.18,
            details=f"rest={upper_rest}, extended={upper_extended}",
        )

    with ctx.pose({lower_arm_joint: math.pi * 0.5, upper_arm_joint: math.pi / 3.0, knob_joint: math.pi * 0.5}):
        ctx.expect_contact(lower_spray_arm, body, name="lower spray arm remains hub-mounted while rotating")
        ctx.expect_contact(upper_spray_arm, body, name="upper spray arm remains hub-mounted while rotating")
        ctx.expect_contact(cycle_knob, door, name="cycle knob stays seated while rotating")

    vent_rest = ctx.part_world_aabb(vent_flap)
    vent_limits = vent_joint.motion_limits
    if vent_limits is not None and vent_limits.upper is not None:
        with ctx.pose({vent_joint: vent_limits.upper}):
            vent_open = ctx.part_world_aabb(vent_flap)
        ctx.check(
            "vent flap opens into the tub",
            vent_rest is not None
            and vent_open is not None
            and vent_open[1][1] > vent_rest[1][1] + 0.008,
            details=f"rest={vent_rest}, open={vent_open}",
        )

    for index, (button, button_joint) in enumerate(zip(buttons, button_joints)):
        rest_positions = [ctx.part_world_position(part) for part in buttons]
        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            posed_positions = [ctx.part_world_position(part) for part in buttons]
        moved = (
            rest_positions[index] is not None
            and posed_positions[index] is not None
            and posed_positions[index][1] > rest_positions[index][1] + 0.0015
        )
        others_still = True
        for other_index in range(3):
            if other_index == index:
                continue
            if rest_positions[other_index] is None or posed_positions[other_index] is None:
                others_still = False
                break
            if abs(posed_positions[other_index][1] - rest_positions[other_index][1]) > 1e-6:
                others_still = False
                break
        ctx.check(
            f"option button {index} moves independently",
            moved and others_still,
            details=f"rest={rest_positions}, posed={posed_positions}",
        )

    return ctx.report()


object_model = build_object_model()
