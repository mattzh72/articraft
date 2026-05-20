from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

REMOTE_LENGTH = 0.192
REMOTE_WIDTH_REAR = 0.048
REMOTE_WIDTH_NOSE = 0.041
REMOTE_THICKNESS = 0.018

DOOR_WIDTH = 0.037
DOOR_LENGTH = 0.108
DOOR_CENTER_Y = -0.022
DOOR_RECESS_DEPTH = 0.0012
BATTERY_BAY_WIDTH = 0.031
BATTERY_BAY_LENGTH = 0.091
BATTERY_BAY_DEPTH = 0.0052
DOOR_TRAVEL = 0.052

BUTTON_SPECS = (
    {
        "name": "power_button",
        "kind": "round",
        "x": 0.0,
        "y": 0.072,
        "width": 0.011,
        "length": 0.011,
        "travel": 0.0016,
        "material": "power",
    },
    {
        "name": "mute_button",
        "kind": "rect",
        "x": -0.013,
        "y": 0.050,
        "width": 0.010,
        "length": 0.008,
        "travel": 0.0014,
        "material": "dark",
    },
    {
        "name": "home_button",
        "kind": "rect",
        "x": 0.0,
        "y": 0.050,
        "width": 0.010,
        "length": 0.008,
        "travel": 0.0014,
        "material": "dark",
    },
    {
        "name": "back_button",
        "kind": "rect",
        "x": 0.013,
        "y": 0.050,
        "width": 0.010,
        "length": 0.008,
        "travel": 0.0014,
        "material": "dark",
    },
    {
        "name": "nav_up",
        "kind": "rect",
        "x": 0.0,
        "y": 0.018,
        "width": 0.013,
        "length": 0.010,
        "travel": 0.0015,
        "material": "light",
    },
    {
        "name": "nav_left",
        "kind": "rect",
        "x": -0.013,
        "y": 0.004,
        "width": 0.010,
        "length": 0.013,
        "travel": 0.0015,
        "material": "light",
    },
    {
        "name": "ok_button",
        "kind": "round",
        "x": 0.0,
        "y": 0.004,
        "width": 0.0125,
        "length": 0.0125,
        "travel": 0.0017,
        "material": "light",
    },
    {
        "name": "nav_right",
        "kind": "rect",
        "x": 0.013,
        "y": 0.004,
        "width": 0.010,
        "length": 0.013,
        "travel": 0.0015,
        "material": "light",
    },
    {
        "name": "nav_down",
        "kind": "rect",
        "x": 0.0,
        "y": -0.010,
        "width": 0.013,
        "length": 0.010,
        "travel": 0.0015,
        "material": "light",
    },
    {
        "name": "volume_up",
        "kind": "rect",
        "x": -0.013,
        "y": -0.041,
        "width": 0.012,
        "length": 0.010,
        "travel": 0.0014,
        "material": "dark",
    },
    {
        "name": "volume_down",
        "kind": "rect",
        "x": -0.013,
        "y": -0.055,
        "width": 0.012,
        "length": 0.010,
        "travel": 0.0014,
        "material": "dark",
    },
    {
        "name": "channel_up",
        "kind": "rect",
        "x": 0.013,
        "y": -0.041,
        "width": 0.012,
        "length": 0.010,
        "travel": 0.0014,
        "material": "dark",
    },
    {
        "name": "channel_down",
        "kind": "rect",
        "x": 0.013,
        "y": -0.055,
        "width": 0.012,
        "length": 0.010,
        "travel": 0.0014,
        "material": "dark",
    },
    {
        "name": "play_button",
        "kind": "rect",
        "x": 0.0,
        "y": -0.073,
        "width": 0.016,
        "length": 0.010,
        "travel": 0.0014,
        "material": "dark",
    },
)


def _button_mesh_name(kind: str, width: float, length: float) -> str:
    return f"{kind}_button_{int(round(width * 1000.0))}_{int(round(length * 1000.0))}"


def _build_rect_button_shape(width: float, length: float) -> cq.Workplane:
    cap_height = 0.0029
    cap_embed = 0.0012
    stem_height = 0.0016
    stem_width = max(width * 0.62, width - 0.0042)
    stem_length = max(length * 0.62, length - 0.0042)
    corner = min(width, length) * 0.18

    cap = (
        cq.Workplane("XY")
        .box(width, length, cap_height)
        .edges("|Z")
        .fillet(corner)
        .translate((0.0, 0.0, (cap_height * 0.5) - cap_embed))
    )
    stem = (
        cq.Workplane("XY")
        .box(stem_width, stem_length, stem_height)
        .edges("|Z")
        .fillet(min(stem_width, stem_length) * 0.15)
        .translate((0.0, 0.0, -0.00185))
    )
    return cap.union(stem)


def _build_round_button_shape(diameter: float) -> cq.Workplane:
    cap_height = 0.0028
    cap_embed = 0.0011
    stem_height = 0.0017
    cap = (
        cq.Workplane("XY")
        .circle(diameter * 0.5)
        .extrude(cap_height)
        .translate((0.0, 0.0, -cap_embed))
    )
    stem = (
        cq.Workplane("XY")
        .circle(diameter * 0.27)
        .extrude(stem_height)
        .translate((0.0, 0.0, -0.0026))
    )
    return cap.union(stem)


def _build_body_shape() -> cq.Workplane:
    half_rear = REMOTE_WIDTH_REAR * 0.5
    half_nose = REMOTE_WIDTH_NOSE * 0.5
    half_length = REMOTE_LENGTH * 0.5

    body = (
        cq.Workplane("XY")
        .moveTo(-half_rear, -half_length)
        .lineTo(half_rear, -half_length)
        .lineTo(half_nose, half_length)
        .lineTo(-half_nose, half_length)
        .close()
        .extrude(REMOTE_THICKNESS)
        .translate((0.0, 0.0, -REMOTE_THICKNESS * 0.5))
        .edges("|Z")
        .fillet(0.0046)
        .edges(">Z")
        .fillet(0.0030)
        .edges("<Z")
        .fillet(0.0022)
    )

    door_cut = (
        cq.Workplane("XY")
        .box(DOOR_WIDTH + 0.0014, DOOR_LENGTH + 0.0016, DOOR_RECESS_DEPTH)
        .translate(
            (
                0.0,
                DOOR_CENTER_Y,
                (-REMOTE_THICKNESS * 0.5) + (DOOR_RECESS_DEPTH * 0.5),
            )
        )
    )
    battery_bay = (
        cq.Workplane("XY")
        .box(BATTERY_BAY_WIDTH, BATTERY_BAY_LENGTH, BATTERY_BAY_DEPTH)
        .translate(
            (
                0.0,
                DOOR_CENTER_Y,
                (-REMOTE_THICKNESS * 0.5)
                + DOOR_RECESS_DEPTH
                + (BATTERY_BAY_DEPTH * 0.5),
            )
        )
    )
    latch_relief = (
        cq.Workplane("XY")
        .box(0.016, 0.010, 0.0026)
        .translate(
            (
                0.0,
                DOOR_CENTER_Y - (DOOR_LENGTH * 0.5) + 0.008,
                (-REMOTE_THICKNESS * 0.5) + DOOR_RECESS_DEPTH + 0.0013,
            )
        )
    )

    body = body.cut(door_cut).cut(battery_bay).cut(latch_relief)

    for spec in BUTTON_SPECS:
        pocket_depth = spec["travel"] + 0.0031
        if spec["kind"] == "round":
            cutter = (
                cq.Workplane("XY")
                .circle(spec["width"] * 0.5)
                .extrude(pocket_depth)
                .translate(
                    (
                        spec["x"],
                        spec["y"],
                        (REMOTE_THICKNESS * 0.5) - pocket_depth,
                    )
                )
            )
        else:
            cutter = (
                cq.Workplane("XY")
                .rect(spec["width"], spec["length"])
                .extrude(pocket_depth)
                .translate(
                    (
                        spec["x"],
                        spec["y"],
                        (REMOTE_THICKNESS * 0.5) - pocket_depth,
                    )
                )
            )
        body = body.cut(cutter)

    return body


def _build_battery_door_shape() -> cq.Workplane:
    outer_thickness = 0.00095
    guide_height = 0.00135
    guide_width = 0.0045
    guide_offset = (DOOR_WIDTH * 0.5) - 0.0056

    panel = (
        cq.Workplane("XY")
        .box(DOOR_WIDTH, DOOR_LENGTH, outer_thickness)
        .edges("|Z")
        .fillet(0.0017)
        .translate((0.0, 0.0, (-REMOTE_THICKNESS * 0.5) + (outer_thickness * 0.5)))
    )
    left_guide = (
        cq.Workplane("XY")
        .box(guide_width, DOOR_LENGTH - 0.008, guide_height)
        .translate(
            (
                -guide_offset,
                0.0,
                (-REMOTE_THICKNESS * 0.5)
                + outer_thickness
                + (guide_height * 0.5),
            )
        )
    )
    right_guide = (
        cq.Workplane("XY")
        .box(guide_width, DOOR_LENGTH - 0.008, guide_height)
        .translate(
            (
                guide_offset,
                0.0,
                (-REMOTE_THICKNESS * 0.5)
                + outer_thickness
                + (guide_height * 0.5),
            )
        )
    )
    thumb_rib = (
        cq.Workplane("XY")
        .box(0.015, 0.014, 0.00055)
        .edges("|Z")
        .fillet(0.0014)
        .translate(
            (
                0.0,
                -0.030,
                (-REMOTE_THICKNESS * 0.5) - (0.00055 * 0.5),
            )
        )
    )
    return panel.union(left_guide).union(right_guide).union(thumb_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="television_remote")

    shell = model.material("shell", rgba=(0.12, 0.13, 0.15, 1.0))
    door = model.material("door", rgba=(0.18, 0.19, 0.21, 1.0))
    button_dark = model.material("button_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    button_light = model.material("button_light", rgba=(0.47, 0.48, 0.50, 1.0))
    power_red = model.material("power_red", rgba=(0.67, 0.10, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "remote_body"),
        material=shell,
        name="shell",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        mesh_from_cadquery(_build_battery_door_shape(), "battery_door"),
        material=door,
        name="door_panel",
    )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, DOOR_CENTER_Y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.18,
            lower=0.0,
            upper=DOOR_TRAVEL,
        ),
    )

    button_mesh_cache: dict[tuple[str, float, float], object] = {}
    button_materials = {
        "dark": button_dark,
        "light": button_light,
        "power": power_red,
    }
    for spec in BUTTON_SPECS:
        cache_key = (spec["kind"], spec["width"], spec["length"])
        button_mesh = button_mesh_cache.get(cache_key)
        if button_mesh is None:
            if spec["kind"] == "round":
                button_shape = _build_round_button_shape(spec["width"])
            else:
                button_shape = _build_rect_button_shape(spec["width"], spec["length"])
            button_mesh = mesh_from_cadquery(
                button_shape,
                _button_mesh_name(spec["kind"], spec["width"], spec["length"]),
            )
            button_mesh_cache[cache_key] = button_mesh

        button_part = model.part(spec["name"])
        button_part.visual(
            button_mesh,
            material=button_materials[spec["material"]],
            name="button",
        )
        model.articulation(
            f"body_to_{spec['name']}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button_part,
            origin=Origin(xyz=(spec["x"], spec["y"], REMOTE_THICKNESS * 0.5)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=0.04,
                lower=0.0,
                upper=spec["travel"],
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    battery_door = object_model.get_part("battery_door")
    door_slide = object_model.get_articulation("body_to_battery_door")
    power_button = object_model.get_part("power_button")
    ok_button = object_model.get_part("ok_button")
    play_button = object_model.get_part("play_button")
    power_joint = object_model.get_articulation("body_to_power_button")
    ok_joint = object_model.get_articulation("body_to_ok_button")

    ctx.expect_within(
        battery_door,
        body,
        axes="x",
        margin=0.0005,
        name="battery door stays centered on the body width",
    )
    ctx.expect_overlap(
        battery_door,
        body,
        axes="y",
        min_overlap=0.060,
        name="battery door covers the battery bay at rest",
    )

    rest_position = ctx.part_world_position(battery_door)
    with ctx.pose({door_slide: DOOR_TRAVEL}):
        ctx.expect_within(
            battery_door,
            body,
            axes="x",
            margin=0.0005,
            name="battery door remains guided laterally when open",
        )
        ctx.expect_overlap(
            battery_door,
            body,
            axes="y",
            min_overlap=0.040,
            name="battery door remains captured when slid open",
        )
        open_position = ctx.part_world_position(battery_door)

    ctx.check(
        "battery door slides toward the handset base",
        rest_position is not None
        and open_position is not None
        and open_position[1] < rest_position[1] - 0.040,
        details=f"rest={rest_position}, open={open_position}",
    )

    ctx.expect_within(
        power_button,
        body,
        axes="xy",
        margin=0.0005,
        name="power button sits within the tapered handset face",
    )
    ctx.expect_within(
        play_button,
        body,
        axes="xy",
        margin=0.0005,
        name="lower button bank stays within the handset footprint",
    )

    power_rest = ctx.part_world_position(power_button)
    ok_rest = ctx.part_world_position(ok_button)
    with ctx.pose({power_joint: power_joint.motion_limits.upper}):
        power_pressed = ctx.part_world_position(power_button)
        ok_during_power_press = ctx.part_world_position(ok_button)

    ctx.check(
        "power button depresses independently",
        power_rest is not None
        and power_pressed is not None
        and ok_rest is not None
        and ok_during_power_press is not None
        and power_pressed[2] < power_rest[2] - 0.0010
        and abs(ok_during_power_press[2] - ok_rest[2]) < 0.00005,
        details=(
            f"power_rest={power_rest}, power_pressed={power_pressed}, "
            f"ok_rest={ok_rest}, ok_during_power_press={ok_during_power_press}"
        ),
    )

    with ctx.pose({ok_joint: ok_joint.motion_limits.upper}):
        ok_pressed = ctx.part_world_position(ok_button)
        power_during_ok_press = ctx.part_world_position(power_button)

    ctx.check(
        "center button also depresses independently",
        power_rest is not None
        and power_during_ok_press is not None
        and ok_rest is not None
        and ok_pressed is not None
        and ok_pressed[2] < ok_rest[2] - 0.0011
        and abs(power_during_ok_press[2] - power_rest[2]) < 0.00005,
        details=(
            f"ok_rest={ok_rest}, ok_pressed={ok_pressed}, "
            f"power_rest={power_rest}, power_during_ok_press={power_during_ok_press}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
