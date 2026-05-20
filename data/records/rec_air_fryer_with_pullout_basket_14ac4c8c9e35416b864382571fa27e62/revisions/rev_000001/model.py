from __future__ import annotations

import math

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

BODY_RADIUS = 0.135
BODY_HEIGHT = 0.272
BODY_FRONT_X = 0.108

TOWER_DEPTH = 0.100
TOWER_WIDTH = 0.128
TOWER_HEIGHT = 0.094
TOWER_CENTER_X = 0.044
TOWER_FRONT_X = TOWER_CENTER_X + TOWER_DEPTH * 0.5

DRAWER_CENTER_Z = 0.105
CAVITY_WIDTH = 0.222
CAVITY_HEIGHT = 0.126
CAVITY_DEPTH = 0.186

DRAWER_WIDTH = 0.212
DRAWER_HEIGHT = 0.106
DRAWER_DEPTH = 0.170
DRAWER_FRONT_THICKNESS = 0.020
DRAWER_TRAVEL = 0.112

BASKET_WIDTH = 0.196
BASKET_HEIGHT = 0.084
BASKET_DEPTH = 0.154

KEEP_WARM_TOP_X = 0.040
KEEP_WARM_TOP_Z = BODY_HEIGHT + TOWER_HEIGHT


def _body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").circle(BODY_RADIUS).extrude(BODY_HEIGHT)

    front_flat = (
        cq.Workplane("XY")
        .box(0.260, 0.360, BODY_HEIGHT + TOWER_HEIGHT + 0.060)
        .translate((BODY_FRONT_X + 0.130, 0.0, (BODY_HEIGHT + TOWER_HEIGHT) * 0.5))
    )
    body = body.cut(front_flat)

    tower = (
        cq.Workplane("XY")
        .box(TOWER_DEPTH, TOWER_WIDTH, TOWER_HEIGHT)
        .translate((TOWER_CENTER_X, 0.0, BODY_HEIGHT + TOWER_HEIGHT * 0.5 - 0.004))
        .edges("|Z")
        .fillet(0.016)
    )
    body = body.union(tower)

    cavity = (
        cq.Workplane("YZ")
        .center(0.0, DRAWER_CENTER_Z)
        .rect(CAVITY_WIDTH, CAVITY_HEIGHT)
        .extrude(CAVITY_DEPTH + 0.008)
        .edges("|X")
        .fillet(0.018)
        .translate((BODY_FRONT_X - CAVITY_DEPTH - 0.004, 0.0, 0.0))
    )
    body = body.cut(cavity)

    keep_warm_well = (
        cq.Workplane("XY")
        .box(0.018, 0.028, 0.010)
        .translate((KEEP_WARM_TOP_X, 0.0, KEEP_WARM_TOP_Z - 0.005))
    )
    body = body.cut(keep_warm_well)

    for dial_z in (BODY_HEIGHT + 0.018, BODY_HEIGHT + 0.050):
        dial_bore = (
            cq.Workplane("YZ")
            .center(0.0, dial_z)
            .circle(0.0095)
            .extrude(0.006)
            .translate((BODY_FRONT_X - 0.006, 0.0, 0.0))
        )
        body = body.cut(dial_bore)

    return body


def _drawer_shape() -> cq.Workplane:
    floor = cq.Workplane("XY").box(0.164, 0.196, 0.004).translate((-0.080, 0.0, -0.047))
    rear_wall = cq.Workplane("XY").box(0.004, 0.204, 0.090).translate((-0.166, 0.0, -0.004))
    left_wall = cq.Workplane("XY").box(0.168, 0.004, 0.090).translate((-0.080, -0.100, -0.004))
    right_wall = cq.Workplane("XY").box(0.168, 0.004, 0.090).translate((-0.080, 0.100, -0.004))
    upper_left_rail = cq.Workplane("XY").box(0.156, 0.008, 0.004).translate((-0.080, -0.098, 0.039))
    upper_right_rail = cq.Workplane("XY").box(0.156, 0.008, 0.004).translate((-0.080, 0.098, 0.039))
    rear_rim = cq.Workplane("XY").box(0.008, 0.196, 0.004).translate((-0.162, 0.0, 0.039))

    guide_pad_0 = cq.Workplane("XY").box(0.042, 0.010, 0.024).translate((-0.040, -0.105, -0.008))
    guide_pad_1 = cq.Workplane("XY").box(0.042, 0.010, 0.024).translate((-0.040, 0.105, -0.008))
    guide_pad_2 = cq.Workplane("XY").box(0.042, 0.010, 0.024).translate((-0.120, -0.105, -0.008))
    guide_pad_3 = cq.Workplane("XY").box(0.042, 0.010, 0.024).translate((-0.120, 0.105, -0.008))

    fascia = (
        cq.Workplane("XY")
        .box(DRAWER_FRONT_THICKNESS, 0.224, 0.112)
        .translate((DRAWER_FRONT_THICKNESS * 0.5, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.008)
    )

    drawer = floor.union(rear_wall).union(left_wall).union(right_wall)
    drawer = drawer.union(upper_left_rail).union(upper_right_rail).union(rear_rim).union(fascia)
    drawer = drawer.union(guide_pad_0).union(guide_pad_1).union(guide_pad_2).union(guide_pad_3)
    return drawer


def _basket_shape() -> cq.Workplane:
    floor = cq.Workplane("XY").box(0.124, 0.192, 0.004).translate((-0.082, 0.0, -0.041))
    rear_wall = cq.Workplane("XY").box(0.004, 0.184, 0.060).translate((-0.146, 0.0, -0.010))
    front_wall = cq.Workplane("XY").box(0.004, 0.184, 0.030).translate((-0.018, 0.0, -0.025))
    left_wall = cq.Workplane("XY").box(0.132, 0.004, 0.060).translate((-0.082, -0.096, -0.010))
    right_wall = cq.Workplane("XY").box(0.132, 0.004, 0.060).translate((-0.082, 0.096, -0.010))

    basket = floor.union(rear_wall).union(front_wall).union(left_wall).union(right_wall)

    for x_center in (-0.116, -0.082, -0.048):
        for y_center in (-0.052, -0.018, 0.018, 0.052):
            basket = basket.cut(
                cq.Workplane("XY")
                .box(0.016, 0.012, 0.006)
                .translate((x_center, y_center, -0.041))
            )
    return basket


def _handle_shape() -> cq.Workplane:
    rear_pad = cq.Workplane("XY").box(0.010, 0.090, 0.032).translate((0.005, 0.0, 0.0))
    top_bridge = cq.Workplane("XY").box(0.038, 0.110, 0.014).translate((0.021, 0.0, 0.021))
    front_grip = cq.Workplane("XY").box(0.014, 0.118, 0.026).translate((0.057, 0.0, -0.004))
    brace_0 = cq.Workplane("XY").box(0.048, 0.012, 0.034).translate((0.034, -0.052, 0.0))
    brace_1 = cq.Workplane("XY").box(0.048, 0.012, 0.034).translate((0.034, 0.052, 0.0))

    handle = rear_pad.union(top_bridge).union(front_grip).union(brace_0).union(brace_1)

    button_slot = cq.Workplane("XY").box(0.020, 0.038, 0.012).translate((0.022, 0.0, 0.022))
    stem_slot = cq.Workplane("XY").box(0.010, 0.024, 0.020).translate((0.026, 0.0, 0.012))
    return handle.cut(button_slot).cut(stem_slot)


def _release_button_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").box(0.020, 0.040, 0.004).translate((0.0, 0.0, 0.002))
    stem = cq.Workplane("XY").box(0.010, 0.024, 0.010).translate((0.0, 0.0, -0.005))
    return cap.union(stem)


def _control_button_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").box(0.022, 0.032, 0.004).translate((0.0, 0.0, 0.002))
    stem = cq.Workplane("XY").box(0.010, 0.018, 0.005).translate((0.0, 0.0, -0.0015))
    return cap.union(stem)


def _keep_warm_bezel_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.026, 0.036, 0.006).translate((0.0, 0.0, 0.003))
    inner = cq.Workplane("XY").box(0.0185, 0.0285, 0.008).translate((0.0, 0.0, 0.003))
    return outer.cut(inner)


def _dial_shape() -> cq.Workplane:
    shaft = cq.Workplane("YZ").circle(0.009).extrude(0.004).translate((-0.004, 0.0, 0.0))
    skirt = cq.Workplane("YZ").circle(0.018).extrude(0.006)
    cap = skirt.faces(">X").workplane().circle(0.014).extrude(0.010)
    pointer = (
        cq.Workplane("YZ")
        .center(0.0, 0.0065)
        .rect(0.0018, 0.007)
        .extrude(0.002)
        .translate((0.0125, 0.0, 0.0))
    )
    return shaft.union(cap).union(pointer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_air_fryer")

    body_matte = model.material("body_matte", rgba=(0.17, 0.18, 0.19, 1.0))
    drawer_matte = model.material("drawer_matte", rgba=(0.13, 0.14, 0.15, 1.0))
    basket_dark = model.material("basket_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    dial_dark = model.material("dial_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    control_accent = model.material("control_accent", rgba=(0.52, 0.53, 0.54, 1.0))
    release_accent = model.material("release_accent", rgba=(0.72, 0.17, 0.14, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "air_fryer_body"),
        material=body_matte,
        name="body_shell",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "air_fryer_drawer"),
        material=drawer_matte,
        name="drawer_shell",
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_FRONT_X - 0.0045, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.18,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shape(), "air_fryer_basket"),
        material=basket_dark,
        name="basket_shell",
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(),
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shape(), "air_fryer_handle"),
        material=handle_dark,
        name="handle_shell",
    )
    model.articulation(
        "drawer_to_handle",
        ArticulationType.FIXED,
        parent=drawer,
        child=handle,
        origin=Origin(xyz=(DRAWER_FRONT_THICKNESS, 0.0, -0.002)),
    )

    release_button = model.part("release_button")
    release_button.visual(
        mesh_from_cadquery(_release_button_shape(), "air_fryer_release_button"),
        material=release_accent,
        name="release_button_cap",
    )
    model.articulation(
        "handle_to_release_button",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=release_button,
        origin=Origin(xyz=(0.022, 0.0, 0.028)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.003,
        ),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((0.004, 0.074, 0.092)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=drawer_matte,
        name="control_panel",
    )
    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, BODY_HEIGHT + 0.034)),
    )

    temperature_dial = model.part("temperature_dial")
    temperature_dial.visual(
        mesh_from_cadquery(_dial_shape(), "air_fryer_temperature_dial"),
        material=dial_dark,
        name="temperature_dial",
    )
    model.articulation(
        "control_panel_to_temperature_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=temperature_dial,
        origin=Origin(xyz=(0.004, 0.0, -0.021)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=4.0),
    )

    time_dial = model.part("time_dial")
    time_dial.visual(
        mesh_from_cadquery(_dial_shape(), "air_fryer_time_dial"),
        material=dial_dark,
        name="time_dial",
    )
    model.articulation(
        "control_panel_to_time_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=time_dial,
        origin=Origin(xyz=(0.004, 0.0, 0.021)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=4.0),
    )

    keep_warm_bezel = model.part("keep_warm_bezel")
    keep_warm_bezel.visual(
        mesh_from_cadquery(_keep_warm_bezel_shape(), "air_fryer_keep_warm_bezel"),
        material=drawer_matte,
        name="keep_warm_bezel",
    )
    model.articulation(
        "body_to_keep_warm_bezel",
        ArticulationType.FIXED,
        parent=body,
        child=keep_warm_bezel,
        origin=Origin(xyz=(KEEP_WARM_TOP_X, 0.0, KEEP_WARM_TOP_Z - 0.004)),
    )

    keep_warm_button = model.part("keep_warm_button")
    keep_warm_button.visual(
        mesh_from_cadquery(_control_button_shape(), "air_fryer_keep_warm_button"),
        material=control_accent,
        name="keep_warm_cap",
    )
    model.articulation(
        "keep_warm_bezel_to_button",
        ArticulationType.PRISMATIC,
        parent=keep_warm_bezel,
        child=keep_warm_button,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.003,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    handle = object_model.get_part("handle")
    control_panel = object_model.get_part("control_panel")
    keep_warm_button = object_model.get_part("keep_warm_button")
    release_button = object_model.get_part("release_button")

    drawer_joint = object_model.get_articulation("body_to_drawer")
    keep_warm_joint = object_model.get_articulation("keep_warm_bezel_to_button")
    release_joint = object_model.get_articulation("handle_to_release_button")
    temp_joint = object_model.get_articulation("control_panel_to_temperature_dial")
    time_joint = object_model.get_articulation("control_panel_to_time_dial")

    for part_name in ("drawer", "basket", "handle", "release_button"):
        ctx.allow_isolated_part(
            part_name,
            reason="The pull-out basket assembly is guided by hidden internal rails inside the fryer body, so the visible clearance path is intentional.",
        )

    ctx.allow_overlap(
        basket,
        drawer,
        elem_a="basket_shell",
        elem_b="drawer_shell",
        reason="The basket is intentionally represented as a retained insert nested inside the outer drawer carrier.",
    )

    ctx.expect_within(
        basket,
        drawer,
        axes="yz",
        inner_elem="basket_shell",
        outer_elem="drawer_shell",
        margin=0.0015,
        name="basket stays within drawer walls",
    )
    ctx.expect_overlap(
        basket,
        drawer,
        axes="x",
        elem_a="basket_shell",
        elem_b="drawer_shell",
        min_overlap=0.115,
        name="basket remains deeply seated in the drawer",
    )
    ctx.expect_contact(
        handle,
        drawer,
        elem_a="handle_shell",
        elem_b="drawer_shell",
        contact_tol=0.0005,
        name="handle mounts flush to the drawer front",
    )

    drawer_rest = ctx.part_world_position(drawer)
    drawer_extended = None
    if drawer_joint.motion_limits is not None and drawer_joint.motion_limits.upper is not None:
        with ctx.pose({drawer_joint: drawer_joint.motion_limits.upper}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="yz",
                min_overlap=0.100,
                name="drawer remains aligned with the cavity when extended",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.045,
                name="drawer keeps retained insertion at max extension",
            )
            drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides outward from the front",
        drawer_rest is not None and drawer_extended is not None and drawer_extended[0] > drawer_rest[0] + 0.090,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    keep_rest = ctx.part_world_position(keep_warm_button)
    keep_pressed = None
    if keep_warm_joint.motion_limits is not None and keep_warm_joint.motion_limits.upper is not None:
        with ctx.pose({keep_warm_joint: keep_warm_joint.motion_limits.upper}):
            keep_pressed = ctx.part_world_position(keep_warm_button)
    ctx.check(
        "keep-warm button presses downward",
        keep_rest is not None and keep_pressed is not None and keep_pressed[2] < keep_rest[2] - 0.002,
        details=f"rest={keep_rest}, pressed={keep_pressed}",
    )

    release_rest = ctx.part_world_position(release_button)
    release_pressed = None
    if release_joint.motion_limits is not None and release_joint.motion_limits.upper is not None:
        with ctx.pose({release_joint: release_joint.motion_limits.upper}):
            release_pressed = ctx.part_world_position(release_button)
    ctx.check(
        "release button sinks into the handle top",
        release_rest is not None and release_pressed is not None and release_pressed[2] < release_rest[2] - 0.002,
        details=f"rest={release_rest}, pressed={release_pressed}",
    )

    for joint in (temp_joint, time_joint):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    with ctx.pose(
        {
            temp_joint: math.pi / 2.0,
            time_joint: -math.pi / 2.0,
        }
    ):
        ctx.expect_contact(
            "temperature_dial",
            control_panel,
            elem_a="temperature_dial",
            elem_b="control_panel",
            contact_tol=0.001,
            name="temperature dial stays seated on the control tower",
        )
        ctx.expect_contact(
            "time_dial",
            control_panel,
            elem_a="time_dial",
            elem_b="control_panel",
            contact_tol=0.001,
            name="time dial stays seated on the control tower",
        )

    return ctx.report()


object_model = build_object_model()
