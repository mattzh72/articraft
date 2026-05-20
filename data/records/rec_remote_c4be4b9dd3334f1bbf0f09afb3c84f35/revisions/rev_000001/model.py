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

BODY_LENGTH = 0.196
BODY_BOTTOM_WIDTH = 0.046
BODY_TOP_WIDTH = 0.056
BODY_THICKNESS = 0.017
BODY_WALL = 0.0022
BODY_CORNER_RADIUS = 0.0048

BUTTON_TRAVEL = 0.0012
BUTTON_HEIGHT = 0.0040
BUTTON_HOLE_CLEARANCE = 0.0008
INNER_BUTTON_HOLE_CLEARANCE = 0.0

DOOR_WIDTH = 0.038
DOOR_LENGTH = 0.047
DOOR_THICKNESS = 0.0026
DOOR_CENTER_Y = -0.070
DOOR_HINGE_Y = DOOR_CENTER_Y - (DOOR_LENGTH * 0.5)
DOOR_OPENING_WIDTH = 0.032
DOOR_OPENING_LENGTH = 0.038
DOOR_OPEN_LIMIT = 1.45
HINGE_RADIUS = 0.0018

INNER_PANEL_WIDTH = 0.030
INNER_PANEL_LENGTH = 0.034
INNER_PANEL_THICKNESS = 0.0024
INNER_PANEL_TOP_Z = 0.0138

OUTER_BUTTON_SPECS = [
    {"name": "power_key", "kind": "round", "x": 0.0, "y": 0.078, "radius": 0.0052, "material": "button_red"},
    {"name": "home_key", "kind": "round", "x": -0.012, "y": 0.057, "radius": 0.0047, "material": "button_dark"},
    {"name": "mute_key", "kind": "round", "x": 0.0, "y": 0.057, "radius": 0.0047, "material": "button_dark"},
    {"name": "guide_key", "kind": "round", "x": 0.012, "y": 0.057, "radius": 0.0047, "material": "button_dark"},
    {"name": "menu_key", "kind": "rect", "x": 0.0, "y": 0.035, "size": (0.014, 0.009), "material": "button_silver"},
    {"name": "up_key", "kind": "rect", "x": 0.0, "y": 0.014, "size": (0.013, 0.008), "material": "button_silver"},
    {"name": "left_key", "kind": "rect", "x": -0.013, "y": -0.004, "size": (0.010, 0.008), "material": "button_silver"},
    {"name": "ok_key", "kind": "round", "x": 0.0, "y": -0.004, "radius": 0.0052, "material": "button_light"},
    {"name": "right_key", "kind": "rect", "x": 0.013, "y": -0.004, "size": (0.010, 0.008), "material": "button_silver"},
    {"name": "down_key", "kind": "rect", "x": 0.0, "y": -0.022, "size": (0.013, 0.008), "material": "button_silver"},
    {"name": "volume_up", "kind": "rect", "x": -0.013, "y": -0.043, "size": (0.011, 0.017), "material": "button_light"},
    {"name": "channel_up", "kind": "rect", "x": 0.013, "y": -0.043, "size": (0.011, 0.017), "material": "button_light"},
]

INNER_BUTTON_SPECS = [
    {"name": "red_key", "kind": "rect", "x": -0.008, "y": 0.008, "size": (0.010, 0.009), "material": "button_red"},
    {"name": "green_key", "kind": "rect", "x": 0.008, "y": 0.008, "size": (0.010, 0.009), "material": "button_green"},
    {"name": "yellow_key", "kind": "rect", "x": -0.008, "y": -0.008, "size": (0.010, 0.009), "material": "button_yellow"},
    {"name": "blue_key", "kind": "rect", "x": 0.008, "y": -0.008, "size": (0.010, 0.009), "material": "button_blue"},
]

ALL_BUTTON_SPECS = [*OUTER_BUTTON_SPECS, *INNER_BUTTON_SPECS]


def _body_profile() -> cq.Workplane:
    half_length = BODY_LENGTH * 0.5
    half_bottom = BODY_BOTTOM_WIDTH * 0.5
    half_top = BODY_TOP_WIDTH * 0.5
    return (
        cq.Workplane("XY")
        .moveTo(-half_bottom, -half_length)
        .lineTo(half_bottom, -half_length)
        .lineTo(half_top, half_length)
        .lineTo(-half_top, half_length)
        .close()
    )


def _collect_cut_specs(button_specs: list[dict[str, object]]) -> tuple[list[dict[str, float]], list[dict[str, float]]]:
    round_specs: list[dict[str, float]] = []
    rect_specs: list[dict[str, float]] = []
    for spec in button_specs:
        if spec["kind"] == "round":
            round_specs.append(
                {
                    "x": float(spec["x"]),
                    "y": float(spec["y"]),
                    "radius": float(spec["radius"]),
                }
            )
        else:
            width, depth = spec["size"]
            rect_specs.append(
                {
                    "x": float(spec["x"]),
                    "y": float(spec["y"]),
                    "width": float(width),
                    "depth": float(depth),
                }
            )
    return round_specs, rect_specs


def _build_body_shell_shape() -> cq.Workplane:
    outer = _body_profile().extrude(BODY_THICKNESS)
    outer = outer.edges("|Z").fillet(BODY_CORNER_RADIUS)
    shell = outer.faces("<Z").shell(-BODY_WALL)

    cut_depth = BODY_THICKNESS + 0.006
    top_plane = cq.Workplane("XY").workplane(offset=BODY_THICKNESS + 0.001)

    door_cut = (
        top_plane.center(0.0, DOOR_CENTER_Y)
        .rect(DOOR_OPENING_WIDTH, DOOR_OPENING_LENGTH)
        .extrude(-cut_depth)
    )
    shell = shell.cut(door_cut)

    round_specs, rect_specs = _collect_cut_specs(OUTER_BUTTON_SPECS)
    for spec in round_specs:
        round_cut = (
            top_plane.center(spec["x"], spec["y"])
            .circle(spec["radius"] + (BUTTON_HOLE_CLEARANCE * 0.5))
            .extrude(-cut_depth)
        )
        shell = shell.cut(round_cut)

    for spec in rect_specs:
        rect_cut = (
            top_plane.center(spec["x"], spec["y"])
            .rect(spec["width"] + BUTTON_HOLE_CLEARANCE, spec["depth"] + BUTTON_HOLE_CLEARANCE)
            .extrude(-cut_depth)
        )
        shell = shell.cut(rect_cut)

    return shell


def _build_back_plate_shape() -> cq.Workplane:
    return _body_profile().extrude(BODY_WALL + 0.0003)


def _build_inner_panel_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .rect(INNER_PANEL_WIDTH, INNER_PANEL_LENGTH)
        .extrude(INNER_PANEL_THICKNESS)
        .translate((0.0, 0.0, -INNER_PANEL_THICKNESS))
    )

    cut_depth = INNER_PANEL_THICKNESS + 0.004
    top_plane = cq.Workplane("XY").workplane(offset=0.001)

    for spec in INNER_BUTTON_SPECS:
        width, depth = spec["size"]
        panel = panel.cut(
            top_plane.center(float(spec["x"]), float(spec["y"]))
            .rect(float(width) + INNER_BUTTON_HOLE_CLEARANCE, float(depth) + INNER_BUTTON_HOLE_CLEARANCE)
            .extrude(-cut_depth)
        )

    return panel


def _add_button(
    model: ArticulatedObject,
    parent,
    *,
    articulation_prefix: str,
    spec: dict[str, object],
) -> None:
    button = model.part(str(spec["name"]))

    protrusion = BUTTON_TRAVEL
    center_z = protrusion - (BUTTON_HEIGHT * 0.5)

    if spec["kind"] == "round":
        radius = float(spec["radius"])
        button.visual(
            Cylinder(radius=radius, length=BUTTON_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, center_z)),
            material=str(spec["material"]),
            name="cap",
        )
    else:
        width, depth = spec["size"]
        button.visual(
            Box((float(width), float(depth), BUTTON_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, center_z)),
            material=str(spec["material"]),
            name="cap",
        )

    model.articulation(
        f"{articulation_prefix}_to_{spec['name']}",
        ArticulationType.PRISMATIC,
        parent=parent,
        child=button,
        origin=Origin(xyz=(float(spec["x"]), float(spec["y"]), 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=0.08,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="universal_remote")

    model.material("housing", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("back_plate", rgba=(0.17, 0.18, 0.19, 1.0))
    model.material("hinge", rgba=(0.22, 0.23, 0.24, 1.0))
    model.material("button_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    model.material("button_light", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("button_silver", rgba=(0.68, 0.69, 0.70, 1.0))
    model.material("button_red", rgba=(0.72, 0.12, 0.12, 1.0))
    model.material("button_green", rgba=(0.16, 0.56, 0.20, 1.0))
    model.material("button_yellow", rgba=(0.76, 0.64, 0.16, 1.0))
    model.material("button_blue", rgba=(0.17, 0.40, 0.78, 1.0))
    model.material("lens", rgba=(0.14, 0.22, 0.26, 0.85))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell_shape(), "remote_body_shell"),
        material="housing",
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_build_back_plate_shape(), "remote_back_plate"),
        material="back_plate",
        name="back_plate",
    )
    body.visual(
        Box((0.016, 0.010, 0.0012)),
        origin=Origin(xyz=(0.0, 0.088, BODY_THICKNESS + 0.0006)),
        material="lens",
        name="emitter_window",
    )

    for x_pos in (-0.011, 0.011):
        body.visual(
            Box((0.006, 0.005, 0.002)),
            origin=Origin(xyz=(x_pos, DOOR_HINGE_Y - 0.002, BODY_THICKNESS - 0.001)),
            material="hinge",
            name=f"hinge_mount_{'l' if x_pos < 0.0 else 'r'}",
        )
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.008),
            origin=Origin(xyz=(x_pos, DOOR_HINGE_Y, BODY_THICKNESS), rpy=(0.0, math.pi * 0.5, 0.0)),
            material="hinge",
            name=f"hinge_barrel_{'l' if x_pos < 0.0 else 'r'}",
        )

    door = model.part("door")
    door.visual(
        Box((DOOR_WIDTH, DOOR_LENGTH, DOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, DOOR_LENGTH * 0.5, DOOR_THICKNESS * 0.5)),
        material="housing",
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="hinge",
        name="door_barrel",
    )
    door.visual(
        Box((0.014, 0.004, 0.0012)),
        origin=Origin(xyz=(0.0, DOOR_LENGTH - 0.006, DOOR_THICKNESS + 0.0006)),
        material="housing",
        name="door_pull",
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, DOOR_HINGE_Y, BODY_THICKNESS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=DOOR_OPEN_LIMIT,
        ),
    )

    inner_panel = model.part("inner_panel")
    inner_panel.visual(
        mesh_from_cadquery(_build_inner_panel_shape(), "remote_inner_panel"),
        material="housing",
        name="panel",
    )
    for x_pos in (-0.017, 0.017):
        inner_panel.visual(
            Box((0.004, 0.040, 0.0010)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0005)),
            material="housing",
            name=f"rail_{'l' if x_pos < 0.0 else 'r'}",
        )
    model.articulation(
        "body_to_inner_panel",
        ArticulationType.FIXED,
        parent=body,
        child=inner_panel,
        origin=Origin(xyz=(0.0, DOOR_CENTER_Y, INNER_PANEL_TOP_Z)),
    )

    for spec in OUTER_BUTTON_SPECS:
        _add_button(model, body, articulation_prefix="body", spec=spec)

    for spec in INNER_BUTTON_SPECS:
        _add_button(model, inner_panel, articulation_prefix="inner_panel", spec=spec)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    inner_panel = object_model.get_part("inner_panel")

    door_hinge = object_model.get_articulation("body_to_door")
    power_joint = object_model.get_articulation("body_to_power_key")
    mute_joint = object_model.get_articulation("body_to_mute_key")
    red_joint = object_model.get_articulation("inner_panel_to_red_key")
    green_joint = object_model.get_articulation("inner_panel_to_green_key")

    power_key = object_model.get_part("power_key")
    mute_key = object_model.get_part("mute_key")
    red_key = object_model.get_part("red_key")
    green_key = object_model.get_part("green_key")

    ctx.expect_overlap(
        door,
        body,
        axes="xy",
        elem_a="door_panel",
        elem_b="shell",
        min_overlap=0.025,
        name="door covers the lower opening footprint",
    )
    ctx.expect_gap(
        door,
        body,
        axis="z",
        positive_elem="door_panel",
        negative_elem="shell",
        min_gap=0.0,
        max_gap=0.004,
        name="door sits proud without intersecting the housing",
    )
    ctx.expect_gap(
        door,
        inner_panel,
        axis="z",
        positive_elem="door_panel",
        negative_elem="panel",
        min_gap=0.002,
        max_gap=0.006,
        name="hidden control panel sits behind the flip door cavity",
    )

    button_joint_names = [
        f"body_to_{spec['name']}" for spec in OUTER_BUTTON_SPECS
    ] + [
        f"inner_panel_to_{spec['name']}" for spec in INNER_BUTTON_SPECS
    ]
    all_prismatic = True
    all_have_travel = True
    for joint_name in button_joint_names:
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        if joint.articulation_type != ArticulationType.PRISMATIC:
            all_prismatic = False
        if limits is None or limits.upper is None or limits.upper < 0.0010:
            all_have_travel = False

    ctx.check(
        "all keys use independent prismatic joints",
        all_prismatic and len(button_joint_names) == len(ALL_BUTTON_SPECS),
        details=f"joint_count={len(button_joint_names)}",
    )
    ctx.check(
        "all keys have realistic push travel",
        all_have_travel,
        details=f"checked={button_joint_names}",
    )

    power_rest = ctx.part_world_position(power_key)
    mute_rest = ctx.part_world_position(mute_key)
    with ctx.pose({power_joint: BUTTON_TRAVEL}):
        power_pressed = ctx.part_world_position(power_key)
        mute_still = ctx.part_world_position(mute_key)
    ctx.check(
        "top keys depress independently",
        power_rest is not None
        and power_pressed is not None
        and mute_rest is not None
        and mute_still is not None
        and power_pressed[2] < power_rest[2] - 0.0010
        and abs(mute_still[2] - mute_rest[2]) < 1e-6,
        details=f"power_rest={power_rest}, power_pressed={power_pressed}, mute_rest={mute_rest}, mute_still={mute_still}",
    )

    red_rest = ctx.part_world_position(red_key)
    green_rest = ctx.part_world_position(green_key)
    with ctx.pose({door_hinge: DOOR_OPEN_LIMIT, red_joint: BUTTON_TRAVEL}):
        red_pressed = ctx.part_world_position(red_key)
        green_still = ctx.part_world_position(green_key)
    ctx.check(
        "hidden keys remain independent behind the door",
        red_rest is not None
        and red_pressed is not None
        and green_rest is not None
        and green_still is not None
        and red_pressed[2] < red_rest[2] - 0.0010
        and abs(green_still[2] - green_rest[2]) < 1e-6,
        details=f"red_rest={red_rest}, red_pressed={red_pressed}, green_rest={green_rest}, green_still={green_still}",
    )

    door_rest_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: DOOR_OPEN_LIMIT}):
        door_open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.expect_gap(
            door,
            inner_panel,
            axis="z",
            positive_elem="door_panel",
            negative_elem="panel",
            min_gap=0.002,
            name="opened door clears the hidden control panel",
        )
    ctx.check(
        "flip door swings upward from the lower hinge",
        door_rest_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[1][2] > door_rest_aabb[1][2] + 0.020,
        details=f"rest={door_rest_aabb}, open={door_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
