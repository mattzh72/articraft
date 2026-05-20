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


BODY_W = 0.414
BODY_D = 0.306
BODY_H = 0.628

BUCKET_W = 0.348
BUCKET_D = 0.214
BUCKET_H = 0.252
BUCKET_WALL = 0.0045
BUCKET_TRAVEL = 0.176
BUCKET_SEAT_Y = -BODY_D / 2.0 + 0.006
BUCKET_SEAT_Z = 0.040
UPPER_SHELL_Z0 = 0.292
SIDE_W = 0.026
FRONT_FRAME_D = 0.018

CAVITY_W = 0.362
CAVITY_D = 0.244
CAVITY_H = 0.272

FILTER_OPEN_W = 0.302
FILTER_OPEN_H = 0.394
FILTER_OPEN_Z = 0.168
FILTER_CUT_D = 0.040
DOOR_W = 0.318
DOOR_H = 0.410
DOOR_T = 0.008

WINDOW_W = 0.028
WINDOW_H = 0.142
WINDOW_X = 0.132
WINDOW_Z = 0.088

PANEL_TILT = math.radians(32.0)
PANEL_CENTER = (0.000, -0.065, 0.600)

KNOB_LOCAL = (-0.060, 0.000)
START_LOCAL = (0.040, 0.018)
HUMIDITY_0_LOCAL = (0.040, -0.020)
HUMIDITY_1_LOCAL = (0.072, -0.020)


def _panel_point(local_x: float, local_y: float, local_z: float = 0.0) -> tuple[float, float, float]:
    cos_t = math.cos(PANEL_TILT)
    sin_t = math.sin(PANEL_TILT)
    return (
        PANEL_CENTER[0] + local_x,
        PANEL_CENTER[1] + local_y * cos_t - local_z * sin_t,
        PANEL_CENTER[2] + local_y * sin_t + local_z * cos_t,
    )


def _panel_frame(local_x: float, local_y: float, local_z: float = 0.0) -> Origin:
    return Origin(xyz=_panel_point(local_x, local_y, local_z), rpy=(PANEL_TILT, 0.0, 0.0))


def _panel_cylinder_cutter(local_x: float, local_y: float, radius: float, depth: float):
    return (
        cq.Workplane("XY")
        .transformed(
            offset=_panel_point(local_x, local_y),
            rotate=(math.degrees(PANEL_TILT), 0.0, 0.0),
        )
        .circle(radius)
        .extrude(-depth)
    )


def _panel_rect_cutter(local_x: float, local_y: float, size_x: float, size_y: float, depth: float):
    return (
        cq.Workplane("XY")
        .transformed(
            offset=_panel_point(local_x, local_y),
            rotate=(math.degrees(PANEL_TILT), 0.0, 0.0),
        )
        .rect(size_x, size_y)
        .extrude(-depth)
    )


def _build_chassis_shape():
    profile = [
        (-BODY_D / 2.0, UPPER_SHELL_Z0),
        (BODY_D / 2.0, UPPER_SHELL_Z0),
        (BODY_D / 2.0, BODY_H),
        (-0.025, BODY_H),
        (-0.105, BODY_H - 0.050),
        (-BODY_D / 2.0, BODY_H - 0.040),
    ]
    body = (
        cq.Workplane("YZ")
        .polyline(profile)
        .close()
        .extrude(BODY_W)
        .translate((-BODY_W / 2.0, 0.0, 0.0))
    )

    filter_cut = (
        cq.Workplane("XY")
        .box(FILTER_OPEN_W, FILTER_CUT_D, FILTER_OPEN_H, centered=(True, True, False))
        .translate((0.000, BODY_D / 2.0 - FILTER_CUT_D / 2.0 + 0.001, FILTER_OPEN_Z))
    )
    body = body.cut(filter_cut)

    return body


def _build_bucket_shape():
    bucket = cq.Workplane("XY").box(BUCKET_W, BUCKET_D, BUCKET_H, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(
            BUCKET_W - 2.0 * BUCKET_WALL,
            BUCKET_D - 2.0 * BUCKET_WALL,
            BUCKET_H - BUCKET_WALL,
            centered=(True, True, False),
        )
        .translate((0.000, 0.000, BUCKET_WALL))
    )
    rim = (
        cq.Workplane("XY")
        .box(0.144, 0.010, 0.020, centered=(True, True, False))
        .translate((0.000, -BUCKET_D / 2.0 + 0.005, BUCKET_H - 0.030))
    )
    return bucket.cut(inner).union(rim)


def _build_filter_door_shape():
    door = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H, centered=(False, False, False))
    vent_centers = [(0.159, 0.090 + index * 0.036) for index in range(7)]
    slot_cut = (
        cq.Workplane("XZ")
        .workplane(offset=DOOR_T / 2.0)
        .pushPoints(vent_centers)
        .box(0.220, DOOR_T * 2.0, 0.010, centered=(True, True, True))
    )
    return door.cut(slot_cut)


def _build_selector_knob_geometry():
    return KnobGeometry(
        0.050,
        0.024,
        body_style="skirted",
        top_diameter=0.036,
        skirt=KnobSkirt(0.058, 0.006, flare=0.06),
        grip=KnobGrip(style="fluted", count=16, depth=0.0011),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
        center=False,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="basement_dehumidifier")

    body_finish = model.material("body_finish", rgba=(0.84, 0.85, 0.86, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    bucket_finish = model.material("bucket_finish", rgba=(0.90, 0.91, 0.92, 1.0))
    window_glass = model.material("window_glass", rgba=(0.60, 0.72, 0.80, 0.35))
    bucket_window = model.material("bucket_window", rgba=(0.64, 0.78, 0.88, 0.55))
    knob_finish = model.material("knob_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    button_finish = model.material("button_finish", rgba=(0.18, 0.20, 0.22, 1.0))
    start_finish = model.material("start_finish", rgba=(0.22, 0.58, 0.34, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        mesh_from_cadquery(_build_chassis_shape(), "dehumidifier_chassis"),
        material=body_finish,
        name="body_shell",
    )
    chassis.visual(
        Box((BODY_W, BODY_D, BUCKET_SEAT_Z)),
        origin=Origin(xyz=(0.000, 0.000, BUCKET_SEAT_Z / 2.0)),
        material=body_finish,
        name="base_plinth",
    )
    chassis.visual(
        Box((SIDE_W, BODY_D, UPPER_SHELL_Z0 - BUCKET_SEAT_Z)),
        origin=Origin(
            xyz=(
                -BODY_W / 2.0 + SIDE_W / 2.0,
                0.000,
                BUCKET_SEAT_Z + (UPPER_SHELL_Z0 - BUCKET_SEAT_Z) / 2.0,
            ),
        ),
        material=body_finish,
        name="left_wall",
    )
    chassis.visual(
        Box((SIDE_W, BODY_D, UPPER_SHELL_Z0 - BUCKET_SEAT_Z)),
        origin=Origin(
            xyz=(
                BODY_W / 2.0 - SIDE_W / 2.0,
                0.000,
                BUCKET_SEAT_Z + (UPPER_SHELL_Z0 - BUCKET_SEAT_Z) / 2.0,
            ),
        ),
        material=body_finish,
        name="right_wall",
    )
    chassis.visual(
        Box((BODY_W - 2.0 * SIDE_W, 0.070, UPPER_SHELL_Z0 - BUCKET_SEAT_Z)),
        origin=Origin(
            xyz=(
                0.000,
                BODY_D / 2.0 - 0.035,
                BUCKET_SEAT_Z + (UPPER_SHELL_Z0 - BUCKET_SEAT_Z) / 2.0,
            ),
        ),
        material=body_finish,
        name="rear_wall",
    )
    chassis.visual(
        Box((BODY_W - 2.0 * SIDE_W, FRONT_FRAME_D, 0.014)),
        origin=Origin(
            xyz=(
                0.000,
                -BODY_D / 2.0 + FRONT_FRAME_D / 2.0,
                UPPER_SHELL_Z0 + 0.007,
            ),
        ),
        material=body_finish,
        name="bucket_lintel",
    )
    chassis.visual(
        Box((0.024, FRONT_FRAME_D, UPPER_SHELL_Z0 - BUCKET_SEAT_Z - 0.016)),
        origin=Origin(
            xyz=(
                -BODY_W / 2.0 + 0.012,
                -BODY_D / 2.0 + FRONT_FRAME_D / 2.0,
                BUCKET_SEAT_Z + (UPPER_SHELL_Z0 - BUCKET_SEAT_Z - 0.016) / 2.0,
            ),
        ),
        material=body_finish,
        name="front_jamb_0",
    )
    chassis.visual(
        Box((0.024, FRONT_FRAME_D, UPPER_SHELL_Z0 - BUCKET_SEAT_Z - 0.016)),
        origin=Origin(
            xyz=(
                BODY_W / 2.0 - 0.012,
                -BODY_D / 2.0 + FRONT_FRAME_D / 2.0,
                BUCKET_SEAT_Z + (UPPER_SHELL_Z0 - BUCKET_SEAT_Z - 0.016) / 2.0,
            ),
        ),
        material=body_finish,
        name="front_jamb_1",
    )
    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_build_bucket_shape(), "dehumidifier_bucket"),
        origin=Origin(xyz=(0.000, BUCKET_D / 2.0, 0.000)),
        material=bucket_finish,
        name="bucket_shell",
    )
    bucket.visual(
        Box((WINDOW_W - 0.004, 0.003, WINDOW_H - 0.012)),
        origin=Origin(xyz=(WINDOW_X, 0.0015, WINDOW_Z + (WINDOW_H - 0.012) / 2.0)),
        material=bucket_window,
        name="bucket_window",
    )

    rear_filter_door = model.part("rear_filter_door")
    rear_filter_door.visual(
        mesh_from_cadquery(_build_filter_door_shape(), "rear_filter_door"),
        material=body_finish,
        name="door_shell",
    )
    rear_filter_door.visual(
        Box((0.024, 0.012, 0.038)),
        origin=Origin(xyz=(DOOR_W - 0.022, DOOR_T + 0.003, DOOR_H * 0.54)),
        material=trim_dark,
        name="door_pull",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(_build_selector_knob_geometry(), "selector_knob"),
        material=knob_finish,
        name="knob_cap",
    )
    selector_knob.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.000, 0.000, -0.012)),
        material=trim_dark,
        name="knob_shaft",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.0105, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=start_finish,
        name="start_cap",
    )
    start_button.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, -0.004)),
        material=trim_dark,
        name="start_stem",
    )

    humidity_button_0 = model.part("humidity_button_0")
    humidity_button_0.visual(
        Cylinder(radius=0.0095, length=0.0055),
        origin=Origin(xyz=(0.000, 0.000, 0.00275)),
        material=button_finish,
        name="humidity_cap",
    )
    humidity_button_0.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, -0.004)),
        material=trim_dark,
        name="humidity_stem",
    )

    humidity_button_1 = model.part("humidity_button_1")
    humidity_button_1.visual(
        Cylinder(radius=0.0095, length=0.0055),
        origin=Origin(xyz=(0.000, 0.000, 0.00275)),
        material=button_finish,
        name="humidity_cap",
    )
    humidity_button_1.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, -0.004)),
        material=trim_dark,
        name="humidity_stem",
    )

    model.articulation(
        "chassis_to_bucket",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=bucket,
        origin=Origin(xyz=(0.000, BUCKET_SEAT_Y, BUCKET_SEAT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.28,
            lower=0.0,
            upper=BUCKET_TRAVEL,
        ),
    )
    model.articulation(
        "chassis_to_rear_filter_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=rear_filter_door,
        origin=Origin(
            xyz=(-DOOR_W / 2.0, BODY_D / 2.0, FILTER_OPEN_Z - (DOOR_H - FILTER_OPEN_H) / 2.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=1.85,
        ),
    )
    model.articulation(
        "chassis_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=selector_knob,
        origin=_panel_frame(*KNOB_LOCAL, local_z=0.020),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=9.0),
    )
    model.articulation(
        "chassis_to_start_button",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=start_button,
        origin=_panel_frame(*START_LOCAL, local_z=0.010),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.10, lower=0.0, upper=0.004),
    )
    model.articulation(
        "chassis_to_humidity_button_0",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=humidity_button_0,
        origin=_panel_frame(*HUMIDITY_0_LOCAL, local_z=0.009),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.10, lower=0.0, upper=0.0035),
    )
    model.articulation(
        "chassis_to_humidity_button_1",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=humidity_button_1,
        origin=_panel_frame(*HUMIDITY_1_LOCAL, local_z=0.009),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.10, lower=0.0, upper=0.0035),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    bucket = object_model.get_part("bucket")
    rear_filter_door = object_model.get_part("rear_filter_door")
    selector_knob = object_model.get_part("selector_knob")
    start_button = object_model.get_part("start_button")
    humidity_button_0 = object_model.get_part("humidity_button_0")
    humidity_button_1 = object_model.get_part("humidity_button_1")

    bucket_joint = object_model.get_articulation("chassis_to_bucket")
    door_joint = object_model.get_articulation("chassis_to_rear_filter_door")
    knob_joint = object_model.get_articulation("chassis_to_selector_knob")
    start_joint = object_model.get_articulation("chassis_to_start_button")
    humidity_joint_0 = object_model.get_articulation("chassis_to_humidity_button_0")
    humidity_joint_1 = object_model.get_articulation("chassis_to_humidity_button_1")

    bucket_upper = bucket_joint.motion_limits.upper if bucket_joint.motion_limits is not None else None
    door_upper = door_joint.motion_limits.upper if door_joint.motion_limits is not None else None
    start_upper = start_joint.motion_limits.upper if start_joint.motion_limits is not None else None
    humidity_upper_0 = humidity_joint_0.motion_limits.upper if humidity_joint_0.motion_limits is not None else None
    humidity_upper_1 = humidity_joint_1.motion_limits.upper if humidity_joint_1.motion_limits is not None else None

    ctx.allow_overlap(
        chassis,
        start_button,
        elem_a="body_shell",
        elem_b="start_stem",
        reason="The start button stem is intentionally represented as passing through the front control panel into the housing cavity.",
    )
    ctx.allow_overlap(
        chassis,
        humidity_button_0,
        elem_a="body_shell",
        elem_b="humidity_stem",
        reason="The humidity button stem is intentionally represented as passing through the front control panel into the housing cavity.",
    )
    ctx.allow_overlap(
        chassis,
        humidity_button_1,
        elem_a="body_shell",
        elem_b="humidity_stem",
        reason="The humidity button stem is intentionally represented as passing through the front control panel into the housing cavity.",
    )
    ctx.allow_overlap(
        chassis,
        selector_knob,
        elem_a="body_shell",
        elem_b="knob_shaft",
        reason="The selector knob shaft is intentionally represented as passing through the front control panel into the housing cavity.",
    )

    with ctx.pose({bucket_joint: 0.0}):
        ctx.expect_within(
            bucket,
            chassis,
            axes="x",
            margin=0.008,
            name="bucket stays laterally within the lower body",
        )
        ctx.expect_overlap(
            bucket,
            chassis,
            axes="y",
            min_overlap=0.18,
            name="closed bucket seats deeply inside the chassis",
        )
        ctx.expect_overlap(
            bucket,
            chassis,
            axes="z",
            min_overlap=0.22,
            name="bucket remains within the lower opening height",
        )

    if bucket_upper is not None:
        closed_bucket_pos = ctx.part_world_position(bucket)
        with ctx.pose({bucket_joint: bucket_upper}):
            ctx.expect_within(
                bucket,
                chassis,
                axes="x",
                margin=0.008,
                name="extended bucket stays centered in the guide opening",
            )
            ctx.expect_overlap(
                bucket,
                chassis,
                axes="y",
                min_overlap=0.03,
                name="extended bucket remains retained in the chassis",
            )
            extended_bucket_pos = ctx.part_world_position(bucket)
        ctx.check(
            "bucket slides forward from the front opening",
            closed_bucket_pos is not None
            and extended_bucket_pos is not None
            and extended_bucket_pos[1] < closed_bucket_pos[1] - 0.12,
            details=f"closed={closed_bucket_pos}, extended={extended_bucket_pos}",
        )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            rear_filter_door,
            chassis,
            axis="y",
            min_gap=0.0,
            max_gap=0.010,
            positive_elem="door_shell",
            negative_elem="body_shell",
            name="rear filter door closes flush to the back of the housing",
        )

    closed_door_aabb = ctx.part_element_world_aabb(rear_filter_door, elem="door_shell")
    if door_upper is not None:
        with ctx.pose({door_joint: door_upper}):
            open_door_aabb = ctx.part_element_world_aabb(rear_filter_door, elem="door_shell")
        ctx.check(
            "rear filter door swings outward at the back",
            closed_door_aabb is not None
            and open_door_aabb is not None
            and float(open_door_aabb[1][1]) > float(closed_door_aabb[1][1]) + 0.10,
            details=f"closed={closed_door_aabb}, open={open_door_aabb}",
        )

    knob_rest = ctx.part_world_position(selector_knob)
    with ctx.pose({knob_joint: 1.3}):
        knob_turned = ctx.part_world_position(selector_knob)
    ctx.check(
        "selector knob rotates in place on the control cluster",
        knob_rest is not None
        and knob_turned is not None
        and all(abs(a - b) <= 1e-6 for a, b in zip(knob_rest, knob_turned)),
        details=f"rest={knob_rest}, turned={knob_turned}",
    )

    def _check_button_press(part, joint, upper, name: str) -> None:
        if upper is None:
            return
        rest_pos = ctx.part_world_position(part)
        with ctx.pose({joint: upper}):
            pressed_pos = ctx.part_world_position(part)
        ctx.check(
            name,
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.002
            and pressed_pos[1] > rest_pos[1] + 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    _check_button_press(start_button, start_joint, start_upper, "start button presses inward along the sloped panel")
    _check_button_press(
        humidity_button_0,
        humidity_joint_0,
        humidity_upper_0,
        "humidity button 0 presses inward along the sloped panel",
    )
    _check_button_press(
        humidity_button_1,
        humidity_joint_1,
        humidity_upper_1,
        "humidity button 1 presses inward along the sloped panel",
    )

    return ctx.report()


object_model = build_object_model()
