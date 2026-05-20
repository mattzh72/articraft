from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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

BODY_W = 0.370
BODY_D = 0.245
BODY_H = 0.590
BODY_CORNER_R = 0.040
TOP_R = 0.022
WALL_T = 0.0032
BASE_T = 0.008
TOP_T = 0.020

CONTROL_POCKET_W = 0.245
CONTROL_POCKET_D = 0.105
CONTROL_POCKET_DEPTH = 0.007
BUTTON_Z = BODY_H
PLINTH_H = 0.004
DIAL_Z = BODY_H + PLINTH_H
CONTROL_Y = -0.010

BUCKET_W = 0.286
BUCKET_H = 0.224
BUCKET_D = 0.180
BUCKET_TRAVEL = 0.120
BUCKET_OPENING_Z0 = 0.050
BUCKET_OPENING_H = 0.230

SERVICE_DOOR_W = 0.122
SERVICE_DOOR_H = 0.192
SERVICE_DOOR_T = 0.0025
SERVICE_DOOR_CENTER_Y = 0.010
SERVICE_DOOR_Z = 0.350


def _body_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER_R)
        .edges(">Z")
        .fillet(TOP_R)
    )

    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BASE_T))
        .box(
            BODY_W - 2.0 * WALL_T,
            BODY_D - 2.0 * WALL_T,
            BODY_H - BASE_T - TOP_T,
            centered=(True, True, False),
        )
    )
    shell = shell.cut(inner)

    front_opening = (
        cq.Workplane("XY")
        .transformed(
            offset=(0.0, -BODY_D * 0.5 - 0.025, BUCKET_OPENING_Z0),
        )
        .box(BUCKET_W + 0.010, 0.060, BUCKET_OPENING_H, centered=(True, True, False))
    )
    shell = shell.cut(front_opening)

    service_opening = (
        cq.Workplane("XY")
        .transformed(
            offset=(BODY_W * 0.5 + 0.025, SERVICE_DOOR_CENTER_Y, SERVICE_DOOR_Z - SERVICE_DOOR_H * 0.5),
        )
        .box(0.060, SERVICE_DOOR_W, SERVICE_DOOR_H, centered=(True, True, False))
    )
    shell = shell.cut(service_opening)

    for x_pos in (-0.070, -0.035, 0.0, 0.035, 0.070):
        top_slot = (
            cq.Workplane("XY")
            .transformed(offset=(x_pos, 0.062, BODY_H - 0.003))
            .box(0.018, 0.050, 0.010, centered=(True, True, False))
        )
        shell = shell.cut(top_slot)

    for x_pos in (-0.120, -0.090, -0.060, -0.030, 0.0, 0.030, 0.060, 0.090, 0.120):
        rear_slot = (
            cq.Workplane("XY")
            .transformed(offset=(x_pos, BODY_D * 0.5 + 0.020, 0.355))
            .box(0.012, 0.050, 0.150, centered=(True, True, False))
        )
        shell = shell.cut(rear_slot)

    return shell


def _bucket_shape() -> cq.Workplane:
    bucket = (
        cq.Workplane("XY")
        .box(BUCKET_W, BUCKET_D, BUCKET_H, centered=(True, False, True))
        .edges("|Z")
        .fillet(0.016)
        .faces(">Z")
        .shell(-0.003)
    )

    handle_recess = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, -0.001, 0.050))
        .box(0.125, 0.016, 0.042, centered=(True, False, True))
    )
    bucket = bucket.cut(handle_recess)

    lip = (
        cq.Workplane("XY")
        .box(BUCKET_W - 0.020, 0.010, 0.010, centered=(True, False, True))
        .translate((0.0, 0.004, BUCKET_H * 0.5 - 0.012))
    )
    return bucket.union(lip)


def _service_door_shape() -> cq.Workplane:
    door_panel = (
        cq.Workplane("XY")
        .box(SERVICE_DOOR_T, SERVICE_DOOR_W, SERVICE_DOOR_H)
        .translate((-SERVICE_DOOR_T * 0.5, -SERVICE_DOOR_W * 0.5, 0.0))
    )

    hinge_barrel = (
        cq.Workplane("XY")
        .circle(0.0035)
        .extrude(SERVICE_DOOR_H)
        .translate((0.001, 0.0, -SERVICE_DOOR_H * 0.5))
    )
    door_panel = door_panel.union(hinge_barrel)

    finger_pull = (
        cq.Workplane("XY")
        .box(0.012, 0.024, 0.055)
        .translate((-0.001, -SERVICE_DOOR_W + 0.018, 0.0))
    )
    door_panel = door_panel.cut(finger_pull)

    for y_pos in (-0.096, -0.078, -0.060, -0.042, -0.024):
        slot = (
            cq.Workplane("XY")
            .box(0.008, 0.008, 0.105)
            .translate((0.0, y_pos, 0.008))
        )
        door_panel = door_panel.cut(slot)

    return door_panel


def _button_shape(width: float, depth: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(min(width, depth) * 0.18)
        .edges(">Z")
        .fillet(min(width, depth) * 0.12)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_dehumidifier")

    body_plastic = model.material("body_plastic", rgba=(0.92, 0.93, 0.94, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    bucket_plastic = model.material("bucket_plastic", rgba=(0.82, 0.85, 0.88, 0.92))
    bucket_handle = model.material("bucket_handle", rgba=(0.65, 0.69, 0.74, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.14, 0.15, 0.17, 1.0))
    button_finish = model.material("button_finish", rgba=(0.86, 0.88, 0.90, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material=body_plastic,
        name="shell",
    )
    body.visual(
        Box((0.296, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + 0.006, BUCKET_OPENING_Z0 + BUCKET_OPENING_H + 0.006)),
        material=trim_dark,
        name="bucket_upper_trim",
    )
    body.visual(
        Box((0.050, 0.145, 0.004)),
        origin=Origin(xyz=(0.0, CONTROL_Y, BODY_H + PLINTH_H * 0.5)),
        material=trim_dark,
        name="dial_plinth",
    )
    body.visual(
        Box((0.280, 0.165, 0.006)),
        origin=Origin(xyz=(0.0, -0.028, BUCKET_OPENING_Z0 + 0.003)),
        material=trim_dark,
        name="bucket_tray",
    )
    body.visual(
        Box((0.090, 0.090, 0.042)),
        origin=Origin(xyz=(0.0, -0.006, 0.029)),
        material=trim_dark,
        name="tray_support",
    )

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_bucket_shape(), "bucket_shell"),
        material=bucket_plastic,
        name="bucket_shell",
    )
    bucket.visual(
        Box((0.120, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.008, BUCKET_H * 0.5 - 0.020)),
        material=bucket_handle,
        name="bucket_grip",
    )

    model.articulation(
        "body_to_bucket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bucket,
        origin=Origin(
            xyz=(0.0, -BODY_D * 0.5, BUCKET_OPENING_Z0 + BUCKET_OPENING_H * 0.5),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=BUCKET_TRAVEL,
        ),
    )

    service_door = model.part("service_door")
    service_door.visual(
        mesh_from_cadquery(_service_door_shape(), "service_door"),
        material=body_plastic,
        name="door_shell",
    )

    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(
            xyz=(
                BODY_W * 0.5,
                SERVICE_DOOR_CENTER_Y + SERVICE_DOOR_W * 0.5,
                SERVICE_DOOR_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.024,
                body_style="skirted",
                top_diameter=0.044,
                skirt=KnobSkirt(0.066, 0.005, flare=0.04),
                grip=KnobGrip(style="fluted", count=22, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "selector_dial",
        ),
        material=dial_finish,
        name="dial_knob",
    )

    model.articulation(
        "body_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_dial,
        origin=Origin(xyz=(0.0, CONTROL_Y, DIAL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    button_mesh = mesh_from_cadquery(_button_shape(0.028, 0.017, 0.008), "top_button")
    button_x_positions = (-0.128, -0.090, -0.052, 0.052, 0.090, 0.128)

    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(button_mesh, material=button_finish, name="button_cap")
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, CONTROL_Y, BUTTON_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0022,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    service_door = object_model.get_part("service_door")
    selector_dial = object_model.get_part("selector_dial")
    bucket_slide = object_model.get_articulation("body_to_bucket")
    door_hinge = object_model.get_articulation("body_to_service_door")
    dial_joint = object_model.get_articulation("body_to_selector_dial")

    ctx.check(
        "selector dial is continuous",
        str(dial_joint.articulation_type).lower().endswith("continuous"),
        details=f"type={dial_joint.articulation_type!r}",
    )

    ctx.expect_within(
        bucket,
        body,
        axes="xz",
        margin=0.045,
        name="bucket stays aligned with the body opening",
    )

    with ctx.pose({bucket_slide: 0.0}):
        ctx.expect_overlap(
            bucket,
            body,
            axes="y",
            min_overlap=0.17,
            name="seated bucket remains deeply inserted",
        )
        bucket_rest = ctx.part_world_position(bucket)

    with ctx.pose({bucket_slide: BUCKET_TRAVEL}):
        ctx.expect_overlap(
            bucket,
            body,
            axes="y",
            min_overlap=0.045,
            name="extended bucket still retains insertion",
        )
        bucket_extended = ctx.part_world_position(bucket)

    ctx.check(
        "bucket extends out the front",
        bucket_rest is not None
        and bucket_extended is not None
        and bucket_extended[1] < bucket_rest[1] - 0.08,
        details=f"rest={bucket_rest}, extended={bucket_extended}",
    )

    door_rest_aabb = ctx.part_world_aabb(service_door)
    with ctx.pose({door_hinge: math.radians(95.0)}):
        door_open_aabb = ctx.part_world_aabb(service_door)

    def _aabb_center_x(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return float(mins[0] + maxs[0]) * 0.5

    ctx.check(
        "service door swings outward",
        _aabb_center_x(door_rest_aabb) is not None
        and _aabb_center_x(door_open_aabb) is not None
        and _aabb_center_x(door_open_aabb) > _aabb_center_x(door_rest_aabb) + 0.020,
        details=f"rest={door_rest_aabb}, open={door_open_aabb}",
    )

    button_names = [f"button_{index}" for index in range(6)]
    ctx.check(
        "six independent top buttons present",
        all(object_model.get_part(name) is not None for name in button_names),
        details=f"buttons={button_names}",
    )

    button_joint = object_model.get_articulation("body_to_button_0")
    button_part = object_model.get_part("button_0")
    button_rest = ctx.part_world_position(button_part)
    with ctx.pose({button_joint: 0.0022}):
        button_pressed = ctx.part_world_position(button_part)

    ctx.check(
        "top button depresses downward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[2] < button_rest[2] - 0.0015,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    dial_rest = ctx.part_world_position(selector_dial)
    with ctx.pose({dial_joint: 1.1}):
        dial_turned = ctx.part_world_position(selector_dial)

    ctx.check(
        "selector dial rotates in place",
        dial_rest is not None
        and dial_turned is not None
        and abs(dial_rest[0] - dial_turned[0]) < 1e-6
        and abs(dial_rest[1] - dial_turned[1]) < 1e-6
        and abs(dial_rest[2] - dial_turned[2]) < 1e-6,
        details=f"rest={dial_rest}, turned={dial_turned}",
    )

    return ctx.report()


object_model = build_object_model()
