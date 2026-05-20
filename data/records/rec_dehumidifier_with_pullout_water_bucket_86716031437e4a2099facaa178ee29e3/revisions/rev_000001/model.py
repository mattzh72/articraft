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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_W = 0.390
BODY_D = 0.265
BODY_H = 0.640
FOOT_H = 0.010
SIDE_T = 0.010
FRONT_T = 0.010
BACK_T = 0.010
FLOOR_T = 0.014
TOP_T = 0.020

GRILLE_W = 0.315
GRILLE_H = 0.190
GRILLE_Z = 0.462

BUCKET_W = 0.298
BUCKET_D = 0.162
BUCKET_H = 0.228
BUCKET_BOTTOM_Z = 0.056
BUCKET_SIDE_T = 0.006
BUCKET_FRONT_T = 0.010
BUCKET_BACK_T = 0.008
BUCKET_FLOOR_T = 0.006
BUCKET_LIP_T = 0.010
BUCKET_TRAVEL = 0.115

FILTER_W = 0.252
FILTER_H = 0.342
FILTER_CENTER_Z = 0.402
FILTER_T = 0.016

POD_W = 0.165
POD_D = 0.115
POD_H = 0.028


def _build_body_mesh() -> object:
    shell_h = BODY_H - FOOT_H
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, shell_h)
        .translate((0.0, 0.0, FOOT_H + shell_h / 2.0))
        .edges("|Z")
        .fillet(0.016)
        .edges(">Z")
        .fillet(0.008)
    )

    inner_h = shell_h - FLOOR_T - TOP_T
    inner = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * SIDE_T, BODY_D - FRONT_T - BACK_T, inner_h)
        .translate((0.0, (BACK_T - FRONT_T) / 2.0, FOOT_H + FLOOR_T + inner_h / 2.0))
    )

    grille_open = (
        cq.Workplane("XY")
        .box(GRILLE_W - 0.028, 0.050, GRILLE_H - 0.028)
        .translate((0.0, -BODY_D / 2.0 + 0.020, GRILLE_Z))
    )

    bucket_open = (
        cq.Workplane("XY")
        .box(BUCKET_W + 0.016, 0.060, BUCKET_H + 0.010)
        .translate((0.0, -BODY_D / 2.0 + 0.022, BUCKET_BOTTOM_Z + BUCKET_H / 2.0))
    )

    filter_open = (
        cq.Workplane("XY")
        .box(FILTER_W + 0.012, 0.050, FILTER_H + 0.012)
        .translate((0.0, BODY_D / 2.0 - 0.020, FILTER_CENTER_Z))
    )

    guide_h = BUCKET_BOTTOM_Z - (FOOT_H + FLOOR_T) - 0.004
    guide_z = FOOT_H + FLOOR_T + guide_h / 2.0
    left_guide = cq.Workplane("XY").box(0.022, 0.146, guide_h).translate((-0.118, -0.018, guide_z))
    right_guide = cq.Workplane("XY").box(0.022, 0.146, guide_h).translate((0.118, -0.018, guide_z))

    housing = outer.cut(inner).cut(grille_open).cut(bucket_open).cut(filter_open)
    housing = housing.union(left_guide).union(right_guide)

    for foot_x in (-0.123, 0.123):
        for foot_y in (-0.090, 0.090):
            foot = cq.Workplane("XY").box(0.055, 0.030, FOOT_H).translate((foot_x, foot_y, FOOT_H / 2.0))
            housing = housing.union(foot)

    return housing


def _build_bucket_mesh() -> object:
    bucket = (
        cq.Workplane("XY")
        .box(BUCKET_W, BUCKET_D, BUCKET_H)
        .translate((0.0, 0.0, BUCKET_H / 2.0))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.005)
    )

    cavity_h = BUCKET_H - BUCKET_FLOOR_T - BUCKET_LIP_T
    cavity = (
        cq.Workplane("XY")
        .box(
            BUCKET_W - 2.0 * BUCKET_SIDE_T,
            BUCKET_D - BUCKET_FRONT_T - BUCKET_BACK_T,
            cavity_h,
        )
        .translate(
            (
                0.0,
                (BUCKET_BACK_T - BUCKET_FRONT_T) / 2.0,
                BUCKET_FLOOR_T + cavity_h / 2.0,
            )
        )
    )

    grip_cut = (
        cq.Workplane("XY")
        .box(0.126, 0.036, 0.030)
        .translate((0.0, -BUCKET_D / 2.0 + 0.015, BUCKET_H * 0.72))
        .edges("|Z")
        .fillet(0.010)
    )

    return bucket.cut(cavity).cut(grip_cut)


def _build_front_panel_mesh() -> object:
    panel_bottom = BUCKET_BOTTOM_Z + BUCKET_H + 0.010
    case_h = BODY_H - TOP_T - panel_bottom
    front = (
        cq.Workplane("XY")
        .box(BODY_W, FRONT_T, case_h)
        .translate((0.0, -BODY_D / 2.0 + FRONT_T / 2.0, panel_bottom + case_h / 2.0))
        .edges("|Z")
        .fillet(0.0035)
    )

    grille_open = (
        cq.Workplane("XY")
        .box(GRILLE_W - 0.028, FRONT_T + 0.006, GRILLE_H - 0.028)
        .translate((0.0, -BODY_D / 2.0 + FRONT_T / 2.0, GRILLE_Z))
    )
    return front.cut(grille_open)


def _build_rear_panel_mesh() -> object:
    case_h = BODY_H - FOOT_H - TOP_T
    rear = (
        cq.Workplane("XY")
        .box(BODY_W, BACK_T, case_h)
        .translate((0.0, BODY_D / 2.0 - BACK_T / 2.0, FOOT_H + case_h / 2.0))
        .edges("|Z")
        .fillet(0.0035)
    )
    filter_open = (
        cq.Workplane("XY")
        .box(FILTER_W - 0.028, BACK_T + 0.006, FILTER_H - 0.040)
        .translate((0.0, BODY_D / 2.0 - BACK_T / 2.0, FILTER_CENTER_Z))
    )
    return rear.cut(filter_open)


def _build_control_pod_mesh() -> object:
    pod = (
        cq.Workplane("XY")
        .box(POD_W, POD_D, POD_H)
        .translate((0.0, 0.0, POD_H / 2.0))
        .edges("|Z")
        .fillet(0.007)
        .edges(">Z")
        .fillet(0.004)
    )

    dial_bezel = cq.Workplane("XY").circle(0.036).extrude(0.003).translate((-0.030, -0.004, POD_H))
    power_bezel = cq.Workplane("XY").circle(0.014).extrude(0.002).translate((0.048, -0.020, POD_H))
    humid_bezel_0 = cq.Workplane("XY").box(0.032, 0.020, 0.002).translate((0.050, 0.014, POD_H + 0.001))
    humid_bezel_1 = cq.Workplane("XY").box(0.032, 0.020, 0.002).translate((0.050, 0.043, POD_H + 0.001))

    return pod.union(dial_bezel).union(power_bezel).union(humid_bezel_0).union(humid_bezel_1)


def _build_filter_door_mesh() -> object:
    frame = cq.Workplane("XY").box(FILTER_W, FILTER_T, FILTER_H)
    frame = frame.cut(cq.Workplane("XY").box(FILTER_W - 0.048, FILTER_T + 0.010, FILTER_H - 0.070))
    frame = frame.edges("|Z").fillet(0.006)

    pull_tab = cq.Workplane("XY").box(0.015, 0.012, 0.055).translate(
        (-FILTER_W / 2.0 + 0.010, FILTER_T / 2.0 + 0.006, 0.0)
    )

    knuckle_span = 0.082
    knuckle_radius = 0.005
    door = frame.union(pull_tab)
    for knuckle_z in (-0.112, 0.0, 0.112):
        knuckle = (
            cq.Workplane("XY")
            .circle(knuckle_radius)
            .extrude(knuckle_span)
            .translate((FILTER_W / 2.0 - 0.004, FILTER_T / 2.0 + 0.003, knuckle_z - knuckle_span / 2.0))
        )
        door = door.union(knuckle)

    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="console_dehumidifier")

    body_white = model.material("body_white", rgba=(0.90, 0.91, 0.89, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.24, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    button_black = model.material("button_black", rgba=(0.10, 0.10, 0.11, 1.0))
    bucket_tint = model.material("bucket_tint", rgba=(0.70, 0.74, 0.78, 0.78))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_front_panel_mesh(), "dehumidifier_front_panel"),
        material=body_white,
        name="front_panel",
    )
    body.visual(
        mesh_from_cadquery(_build_rear_panel_mesh(), "dehumidifier_rear_panel"),
        material=body_white,
        name="rear_panel",
    )
    case_h = BODY_H - FOOT_H - TOP_T
    side_depth = BODY_D - FRONT_T - BACK_T
    body.visual(
        Box((SIDE_T, side_depth, case_h)),
        origin=Origin(xyz=(-BODY_W / 2.0 + SIDE_T / 2.0, 0.0, FOOT_H + case_h / 2.0)),
        material=body_white,
        name="left_side",
    )
    body.visual(
        Box((SIDE_T, side_depth, case_h)),
        origin=Origin(xyz=(BODY_W / 2.0 - SIDE_T / 2.0, 0.0, FOOT_H + case_h / 2.0)),
        material=body_white,
        name="right_side",
    )
    body.visual(
        Box((BODY_W, BODY_D, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - TOP_T / 2.0)),
        material=body_white,
        name="roof",
    )
    body.visual(
        Box((BODY_W - 2.0 * SIDE_T, BODY_D - FRONT_T - BACK_T, FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H + FLOOR_T / 2.0)),
        material=body_white,
        name="floor",
    )
    guide_h = BUCKET_BOTTOM_Z - (FOOT_H + FLOOR_T) - 0.004
    guide_z = FOOT_H + FLOOR_T + guide_h / 2.0
    body.visual(
        Box((0.022, 0.146, guide_h)),
        origin=Origin(xyz=(-0.118, -0.018, guide_z)),
        material=charcoal,
        name="guide_0",
    )
    body.visual(
        Box((0.022, 0.146, guide_h)),
        origin=Origin(xyz=(0.118, -0.018, guide_z)),
        material=charcoal,
        name="guide_1",
    )
    for index, (foot_x, foot_y) in enumerate(
        (
            (-0.123, -0.090),
            (-0.123, 0.090),
            (0.123, -0.090),
            (0.123, 0.090),
        )
    ):
        body.visual(
            Box((0.055, 0.030, FOOT_H)),
            origin=Origin(xyz=(foot_x, foot_y, FOOT_H / 2.0)),
            material=charcoal,
            name=f"foot_{index}",
        )
    body.visual(
        Box((0.060, FRONT_T, 0.020)),
        origin=Origin(
            xyz=(0.0, -BODY_D / 2.0 + FRONT_T / 2.0, BUCKET_BOTTOM_Z + BUCKET_H + 0.020)
        ),
        material=body_white,
        name="mid_band",
    )
    body.visual(
        Box((BODY_W - 0.040, FRONT_T, 0.018)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + FRONT_T / 2.0, BUCKET_BOTTOM_Z - 0.006)),
        material=body_white,
        name="lower_band",
    )
    bucket_jamb_h = 0.235
    bucket_jamb_w = (BODY_W - BUCKET_W) / 2.0
    bucket_jamb_z = 0.1765
    bucket_jamb_x = BUCKET_W / 2.0 + bucket_jamb_w / 2.0
    body.visual(
        Box((bucket_jamb_w, FRONT_T, bucket_jamb_h)),
        origin=Origin(xyz=(-bucket_jamb_x, -BODY_D / 2.0 + FRONT_T / 2.0, bucket_jamb_z)),
        material=body_white,
        name="bucket_jamb_0",
    )
    body.visual(
        Box((bucket_jamb_w, FRONT_T, bucket_jamb_h)),
        origin=Origin(xyz=(bucket_jamb_x, -BODY_D / 2.0 + FRONT_T / 2.0, bucket_jamb_z)),
        material=body_white,
        name="bucket_jamb_1",
    )

    intake_grille = model.part("intake_grille")
    intake_grille.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (GRILLE_W, GRILLE_H),
                0.003,
                slot_size=(GRILLE_W - 0.070, 0.005),
                pitch=(GRILLE_W - 0.060, 0.015),
                frame=0.015,
                corner_radius=0.008,
                center=False,
            ),
            "dehumidifier_intake_grille",
        ),
        material=grille_dark,
        name="grille_face",
    )
    model.articulation(
        "body_to_intake_grille",
        ArticulationType.FIXED,
        parent=body,
        child=intake_grille,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, GRILLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_build_bucket_mesh(), "dehumidifier_bucket"),
        origin=Origin(xyz=(0.0, BUCKET_D / 2.0 - 0.005, 0.0)),
        material=bucket_tint,
        name="bucket_shell",
    )
    model.articulation(
        "body_to_bucket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + 0.005, BUCKET_BOTTOM_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=BUCKET_TRAVEL),
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        mesh_from_cadquery(_build_control_pod_mesh(), "dehumidifier_control_pod"),
        material=charcoal,
        name="pod_shell",
    )
    model.articulation(
        "body_to_control_pod",
        ArticulationType.FIXED,
        parent=body,
        child=control_pod,
        origin=Origin(xyz=(0.086, -0.014, BODY_H)),
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.058,
                0.025,
                body_style="skirted",
                top_diameter=0.048,
                skirt=KnobSkirt(0.070, 0.005, flare=0.06),
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "dehumidifier_timer_dial",
        ),
        material=knob_dark,
        name="dial_knob",
    )
    model.articulation(
        "control_pod_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=control_pod,
        child=timer_dial,
        origin=Origin(xyz=(-0.030, -0.004, POD_H + 0.003)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.20, velocity=8.0),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.0105, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=button_black,
        name="power_cap",
    )
    model.articulation(
        "control_pod_to_power_button",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=power_button,
        origin=Origin(xyz=(0.048, -0.020, POD_H + 0.002)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.002),
    )

    for index, button_y in enumerate((0.014, 0.043)):
        humidity_button = model.part(f"humidity_button_{index}")
        humidity_button.visual(
            Box((0.026, 0.016, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, 0.0025)),
            material=button_black,
            name="humidity_cap",
        )
        model.articulation(
            f"control_pod_to_humidity_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_pod,
            child=humidity_button,
            origin=Origin(xyz=(0.050, button_y, POD_H + 0.002)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=0.08, lower=0.0, upper=0.002),
        )

    filter_door = model.part("filter_door")
    filter_door.visual(
        mesh_from_cadquery(_build_filter_door_mesh(), "dehumidifier_filter_door"),
        origin=Origin(xyz=(-FILTER_W / 2.0, FILTER_T / 2.0, 0.0)),
        material=body_white,
        name="door_frame",
    )
    model.articulation(
        "body_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_door,
        origin=Origin(xyz=(FILTER_W / 2.0, BODY_D / 2.0, FILTER_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    filter_door.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (FILTER_W - 0.048, FILTER_H - 0.070),
                0.0025,
                slot_size=(FILTER_W - 0.098, 0.004),
                pitch=(FILTER_W - 0.088, 0.013),
                frame=0.008,
                corner_radius=0.004,
                center=False,
            ),
            "dehumidifier_filter_insert",
        ),
        origin=Origin(
            xyz=(-FILTER_W / 2.0, 0.003, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_dark,
        name="filter_insert",
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    control_pod = object_model.get_part("control_pod")
    bucket = object_model.get_part("bucket")
    filter_door = object_model.get_part("filter_door")
    timer_dial_joint = object_model.get_articulation("control_pod_to_timer_dial")
    bucket_slide = object_model.get_articulation("body_to_bucket")
    filter_hinge = object_model.get_articulation("body_to_filter_door")
    power_button_joint = object_model.get_articulation("control_pod_to_power_button")
    humidity_joint_0 = object_model.get_articulation("control_pod_to_humidity_button_0")
    humidity_joint_1 = object_model.get_articulation("control_pod_to_humidity_button_1")

    ctx.expect_overlap(
        bucket,
        body,
        axes="xz",
        min_overlap=0.220,
        name="bucket footprint stays within body width and height",
    )
    ctx.expect_gap(
        control_pod,
        body,
        axis="z",
        positive_elem="pod_shell",
        negative_elem="roof",
        max_gap=0.001,
        max_penetration=0.0,
        name="control pod seats on the roof",
    )
    ctx.check(
        "timer dial uses continuous rotation",
        timer_dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={timer_dial_joint.articulation_type!r}",
    )

    bucket_limits = bucket_slide.motion_limits
    if bucket_limits is not None and bucket_limits.upper is not None:
        rest_aabb = ctx.part_world_aabb(bucket)
        with ctx.pose({bucket_slide: bucket_limits.upper}):
            extended_aabb = ctx.part_world_aabb(bucket)
            ctx.expect_overlap(
                bucket,
                body,
                axes="xz",
                min_overlap=0.220,
                name="extended bucket remains aligned with the opening",
            )
        rest_min_y = rest_aabb[0][1] if rest_aabb is not None else None
        extended_min_y = extended_aabb[0][1] if extended_aabb is not None else None
        ctx.check(
            "bucket slides outward",
            rest_min_y is not None and extended_min_y is not None and extended_min_y < rest_min_y - 0.090,
            details=f"rest_min_y={rest_min_y}, extended_min_y={extended_min_y}",
        )

    filter_limits = filter_hinge.motion_limits
    if filter_limits is not None and filter_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(filter_door, elem="door_frame")
        with ctx.pose({filter_hinge: filter_limits.upper}):
            open_aabb = ctx.part_element_world_aabb(filter_door, elem="door_frame")
        closed_max_y = closed_aabb[1][1] if closed_aabb is not None else None
        open_max_y = open_aabb[1][1] if open_aabb is not None else None
        ctx.check(
            "filter door swings outward from the rear",
            closed_max_y is not None and open_max_y is not None and open_max_y > closed_max_y + 0.080,
            details=f"closed_max_y={closed_max_y}, open_max_y={open_max_y}",
        )

    power_rest = ctx.part_element_world_aabb("power_button", elem="power_cap")
    with ctx.pose({power_button_joint: 0.002}):
        power_pressed = ctx.part_element_world_aabb("power_button", elem="power_cap")
    power_rest_max_z = power_rest[1][2] if power_rest is not None else None
    power_pressed_max_z = power_pressed[1][2] if power_pressed is not None else None
    ctx.check(
        "power button depresses downward",
        power_rest_max_z is not None
        and power_pressed_max_z is not None
        and power_pressed_max_z < power_rest_max_z - 0.0015,
        details=f"rest_max_z={power_rest_max_z}, pressed_max_z={power_pressed_max_z}",
    )

    for joint_name, part_name in (
        (humidity_joint_0, "humidity_button_0"),
        (humidity_joint_1, "humidity_button_1"),
    ):
        rest_aabb = ctx.part_element_world_aabb(part_name, elem="humidity_cap")
        with ctx.pose({joint_name: 0.002}):
            pressed_aabb = ctx.part_element_world_aabb(part_name, elem="humidity_cap")
        rest_max_z = rest_aabb[1][2] if rest_aabb is not None else None
        pressed_max_z = pressed_aabb[1][2] if pressed_aabb is not None else None
        ctx.check(
            f"{part_name} depresses downward",
            rest_max_z is not None and pressed_max_z is not None and pressed_max_z < rest_max_z - 0.0015,
            details=f"rest_max_z={rest_max_z}, pressed_max_z={pressed_max_z}",
        )

    return ctx.report()


object_model = build_object_model()
