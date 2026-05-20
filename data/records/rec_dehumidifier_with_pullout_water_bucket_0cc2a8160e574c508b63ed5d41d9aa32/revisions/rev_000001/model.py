from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    WheelGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.39
BODY_DEPTH = 0.29
BODY_BOTTOM = 0.055
BODY_HEIGHT = 0.595
FRONT_Y = -BODY_DEPTH * 0.5
REAR_Y = BODY_DEPTH * 0.5

CONSOLE_FRONT_Z = 0.490
CONSOLE_BREAK_Y = -0.020
CONSOLE_BREAK_Z = BODY_HEIGHT
CONSOLE_SLOPE = math.atan2(CONSOLE_BREAK_Z - CONSOLE_FRONT_Z, CONSOLE_BREAK_Y - FRONT_Y)

BUCKET_WIDTH = 0.316
BUCKET_DEPTH = 0.220
BUCKET_HEIGHT = 0.205
BUCKET_WALL = 0.0038
BUCKET_BOTTOM_Z = 0.082

FILTER_WIDTH = 0.290
FILTER_HEIGHT = 0.338
FILTER_THICKNESS = 0.012
FILTER_BOTTOM_Z = 0.148
FILTER_HINGE_X = FILTER_WIDTH * 0.5

WHEEL_RADIUS = 0.024
WHEEL_WIDTH = 0.016


def _console_z(y: float) -> float:
    if y <= CONSOLE_BREAK_Y:
        return CONSOLE_FRONT_Z + (y - FRONT_Y) * math.tan(CONSOLE_SLOPE)
    return CONSOLE_BREAK_Z


def _build_housing_shape() -> cq.Workplane:
    profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (FRONT_Y, BODY_BOTTOM),
                (REAR_Y, BODY_BOTTOM),
                (REAR_Y, BODY_HEIGHT),
                (CONSOLE_BREAK_Y, BODY_HEIGHT),
                (FRONT_Y, CONSOLE_FRONT_Z),
            ]
        )
        .close()
    )
    housing = profile.extrude(BODY_WIDTH * 0.5, both=True)
    housing = housing.edges("|Z").fillet(0.010)

    bucket_cut = (
        cq.Workplane("XY")
        .box(0.326, 0.228, 0.223, centered=(True, True, False))
        .translate((0.0, FRONT_Y + 0.114, 0.080))
    )
    filter_cut = (
        cq.Workplane("XY")
        .box(0.304, 0.100, 0.352, centered=(True, True, False))
        .translate((0.0, REAR_Y - 0.050, 0.142))
    )
    return housing.cut(bucket_cut.val()).cut(filter_cut.val())


def _build_bucket_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BUCKET_WIDTH, BUCKET_DEPTH, BUCKET_HEIGHT, centered=(True, True, False))
        .translate((0.0, BUCKET_DEPTH * 0.5, 0.0))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Y and |X")
        .fillet(0.006)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            BUCKET_WIDTH - 2.0 * BUCKET_WALL,
            BUCKET_DEPTH - 2.0 * BUCKET_WALL,
            BUCKET_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, BUCKET_DEPTH * 0.5, BUCKET_WALL))
    )
    handle_recess = (
        cq.Workplane("XY")
        .box(0.150, 0.030, 0.040, centered=(True, True, False))
        .translate((0.0, 0.015, 0.133))
        .edges("|Z")
        .fillet(0.008)
    )
    finger_slot = (
        cq.Workplane("XZ")
        .center(0.0, 0.145)
        .rect(0.100, 0.020)
        .extrude(0.022)
        .translate((0.0, 0.002, 0.0))
    )
    return outer.cut(inner).cut(handle_recess).cut(finger_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="basement_dehumidifier")

    cabinet_white = model.material("cabinet_white", rgba=(0.86, 0.87, 0.84, 1.0))
    cabinet_dark = model.material("cabinet_dark", rgba=(0.26, 0.28, 0.29, 1.0))
    bucket_grey = model.material("bucket_grey", rgba=(0.73, 0.75, 0.77, 1.0))
    filter_dark = model.material("filter_dark", rgba=(0.20, 0.22, 0.23, 1.0))
    control_grey = model.material("control_grey", rgba=(0.60, 0.62, 0.64, 1.0))
    button_grey = model.material("button_grey", rgba=(0.90, 0.90, 0.88, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.36, 0.37, 0.38, 1.0))

    body = model.part("body")
    shell_width = BODY_WIDTH - 0.006
    shell_depth = BODY_DEPTH - 0.010
    body.visual(
        Box((0.006, BODY_DEPTH, BODY_HEIGHT - BODY_BOTTOM)),
        origin=Origin(xyz=(-BODY_WIDTH * 0.5 + 0.003, 0.0, BODY_BOTTOM + (BODY_HEIGHT - BODY_BOTTOM) * 0.5)),
        material=cabinet_white,
        name="left_side",
    )
    body.visual(
        Box((0.006, BODY_DEPTH, BODY_HEIGHT - BODY_BOTTOM)),
        origin=Origin(xyz=(BODY_WIDTH * 0.5 - 0.003, 0.0, BODY_BOTTOM + (BODY_HEIGHT - BODY_BOTTOM) * 0.5)),
        material=cabinet_white,
        name="right_side",
    )
    body.visual(
        Box((shell_width, shell_depth, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM + 0.005)),
        material=cabinet_white,
        name="bottom_pan",
    )
    body.visual(
        Box((shell_width, 0.006, 0.026)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.003, 0.068)),
        material=cabinet_white,
        name="front_lower_rail",
    )
    body.visual(
        Box((shell_width, 0.006, 0.184)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.003, 0.398)),
        material=cabinet_white,
        name="front_upper_panel",
    )
    for index, x_pos in enumerate((-0.1775, 0.1775)):
        body.visual(
            Box((0.029, 0.006, 0.224)),
            origin=Origin(xyz=(x_pos, FRONT_Y + 0.003, 0.192)),
            material=cabinet_white,
            name=f"front_jamb_{index}",
        )
    body.visual(
        Box((shell_width, 0.006, 0.094)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.003, 0.102)),
        material=cabinet_white,
        name="rear_lower_panel",
    )
    body.visual(
        Box((shell_width, 0.006, 0.108)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.003, 0.541)),
        material=cabinet_white,
        name="rear_upper_panel",
    )
    for index, x_pos in enumerate((-0.1675, 0.1675)):
        body.visual(
            Box((0.049, 0.006, 0.344)),
            origin=Origin(xyz=(x_pos, REAR_Y - 0.003, 0.320)),
            material=cabinet_white,
            name=f"rear_jamb_{index}",
        )
    body.visual(
        Box((shell_width, 0.162, 0.008)),
        origin=Origin(xyz=(0.0, 0.064, BODY_HEIGHT - 0.004)),
        material=cabinet_white,
        name="top_lid",
    )
    for index, (x_pos, y_pos) in enumerate(((-0.140, -0.100), (0.140, -0.100), (-0.140, 0.100), (0.140, 0.100))):
        body.visual(
            Box((0.010, 0.012, 0.015)),
            origin=Origin(xyz=(x_pos, y_pos, 0.056)),
            material=cabinet_dark,
            name=f"caster_stem_{index}",
        )
        body.visual(
            Box((0.026, 0.012, 0.004)),
            origin=Origin(xyz=(x_pos, y_pos, 0.049)),
            material=cabinet_dark,
            name=f"caster_bridge_{index}",
        )
        for side_index, side_offset in enumerate((-0.010, 0.010)):
            body.visual(
                Box((0.004, 0.012, 0.028)),
                origin=Origin(xyz=(x_pos + side_offset, y_pos, 0.034)),
                material=cabinet_dark,
                name=f"caster_fork_{index}_{side_index}",
            )

    body_knuckles = (
        (0.045, 0.070),
        (0.170, 0.070),
        (0.293, 0.070),
    )
    for index, (center_z_local, length) in enumerate(body_knuckles):
        body.visual(
            Cylinder(radius=0.005, length=length),
            origin=Origin(
                xyz=(FILTER_HINGE_X, REAR_Y, FILTER_BOTTOM_Z + center_z_local),
            ),
            material=hinge_dark,
            name=f"body_hinge_{index}",
        )

    console_panel = model.part("console_panel")
    console_panel.visual(
        Box((shell_width, 0.165, 0.006)),
        material=cabinet_white,
        name="panel",
    )
    model.articulation(
        "body_to_console_panel",
        ArticulationType.FIXED,
        parent=body,
        child=console_panel,
        origin=Origin(
            xyz=(
                0.0,
                -0.0825 + math.sin(CONSOLE_SLOPE) * 0.003,
                0.545 - math.cos(CONSOLE_SLOPE) * 0.003,
            ),
            rpy=(CONSOLE_SLOPE, 0.0, 0.0),
        ),
    )

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_build_bucket_shape(), "dehumidifier_bucket"),
        material=bucket_grey,
        name="bucket_shell",
    )
    bucket.visual(
        Box((BUCKET_WIDTH - 0.020, BUCKET_DEPTH - 0.020, 0.008)),
        origin=Origin(
            xyz=(0.0, BUCKET_DEPTH * 0.5, 0.004),
        ),
        material=bucket_grey,
        name="bucket_floor",
    )
    model.articulation(
        "body_to_bucket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(0.0, FRONT_Y, 0.080)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.115,
        ),
    )

    filter_panel = model.part("filter_panel")
    filter_panel.visual(
        Box((FILTER_WIDTH - 0.008, 0.009, FILTER_HEIGHT)),
        origin=Origin(
            xyz=(-0.147, -0.0045, FILTER_HEIGHT * 0.5),
        ),
        material=control_grey,
        name="panel_frame",
    )
    filter_panel.visual(
        Box((FILTER_WIDTH - 0.038, 0.004, FILTER_HEIGHT - 0.034)),
        origin=Origin(
            xyz=(-0.147, -0.0060, FILTER_HEIGHT * 0.5),
        ),
        material=filter_dark,
        name="filter_media",
    )
    panel_knuckles = (
        (0.107, 0.052),
        (0.235, 0.052),
    )
    for index, (center_z_local, length) in enumerate(panel_knuckles):
        filter_panel.visual(
            Cylinder(radius=0.0048, length=length),
            origin=Origin(
                xyz=(-0.0025, 0.003, center_z_local),
            ),
            material=hinge_dark,
            name=f"panel_hinge_{index}",
        )
    model.articulation(
        "body_to_filter_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_panel,
        origin=Origin(xyz=(FILTER_HINGE_X, REAR_Y, FILTER_BOTTOM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=1.60,
        ),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.024,
                body_style="tapered",
                top_diameter=0.032,
                base_diameter=0.040,
                edge_radius=0.002,
                center=False,
            ),
            "selector_knob",
        ),
        material=cabinet_dark,
        name="knob",
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=console_panel,
        child=selector_knob,
        origin=Origin(
            xyz=(0.020, 0.0115, 0.0030),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=8.0,
        ),
    )

    button_mounts = (
        ("button_0", 0.060),
        ("button_1", 0.097),
    )
    for part_name, x_pos in button_mounts:
        button = model.part(part_name)
        button.visual(
            Box((0.016, 0.012, 0.0055)),
            origin=Origin(xyz=(0.0, 0.0, 0.00275)),
            material=button_grey,
            name="cap",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=console_panel,
            child=button,
            origin=Origin(
                xyz=(x_pos, 0.0195, 0.0030),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0013,
            ),
        )

    wheel_positions = (
        (-0.140, -0.100),
        (0.140, -0.100),
        (-0.140, 0.100),
        (0.140, 0.100),
    )
    for index, (x_pos, y_pos) in enumerate(wheel_positions):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=wheel_black,
            name="wheel",
        )
        caster.visual(
            Cylinder(radius=0.012, length=WHEEL_WIDTH + 0.002),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=control_grey,
            name="hub",
        )
        model.articulation(
            f"body_to_caster_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=caster,
            origin=Origin(xyz=(x_pos, y_pos, WHEEL_RADIUS)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=20.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    console_panel = object_model.get_part("console_panel")
    bucket = object_model.get_part("bucket")
    filter_panel = object_model.get_part("filter_panel")
    selector_knob = object_model.get_part("selector_knob")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    bucket_joint = object_model.get_articulation("body_to_bucket")
    filter_joint = object_model.get_articulation("body_to_filter_panel")
    knob_joint = object_model.get_articulation("body_to_selector_knob")
    button_joint_0 = object_model.get_articulation("body_to_button_0")
    button_joint_1 = object_model.get_articulation("body_to_button_1")

    ctx.allow_overlap(
        console_panel,
        selector_knob,
        elem_a="panel",
        elem_b="knob",
        reason="The selector knob mounts through an unmodeled hole in the sloped console proxy.",
    )
    ctx.allow_overlap(
        console_panel,
        button_0,
        elem_a="panel",
        elem_b="cap",
        reason="Button travel is represented against a simplified solid console panel rather than a modeled plunger bore.",
    )
    ctx.allow_overlap(
        console_panel,
        button_1,
        elem_a="panel",
        elem_b="cap",
        reason="Button travel is represented against a simplified solid console panel rather than a modeled plunger bore.",
    )

    ctx.check(
        "selector knob uses continuous articulation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"joint={knob_joint.articulation_type!r}, limits={knob_joint.motion_limits!r}",
    )

    for index in range(4):
        caster_joint = object_model.get_articulation(f"body_to_caster_{index}")
        ctx.check(
            f"caster_{index} spins continuously",
            caster_joint.articulation_type == ArticulationType.CONTINUOUS
            and caster_joint.motion_limits is not None
            and caster_joint.motion_limits.lower is None
            and caster_joint.motion_limits.upper is None
            and tuple(float(v) for v in caster_joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={caster_joint.articulation_type!r}, axis={caster_joint.axis!r}, limits={caster_joint.motion_limits!r}",
        )

    ctx.expect_within(
        bucket,
        body,
        axes="xz",
        margin=0.012,
        name="bucket stays within body opening footprint at rest",
    )

    bucket_rest = ctx.part_world_position(bucket)
    bucket_upper = bucket_joint.motion_limits.upper if bucket_joint.motion_limits is not None else None
    if bucket_upper is not None:
        with ctx.pose({bucket_joint: bucket_upper}):
            ctx.expect_overlap(
                bucket,
                body,
                axes="y",
                min_overlap=0.090,
                name="bucket retains insertion at full extension",
            )
            bucket_open = ctx.part_world_position(bucket)
        ctx.check(
            "bucket slides forward",
            bucket_rest is not None
            and bucket_open is not None
            and bucket_open[1] < bucket_rest[1] - 0.10,
            details=f"rest={bucket_rest}, open={bucket_open}",
        )

    panel_closed_aabb = ctx.part_element_world_aabb(filter_panel, elem="panel_frame")
    panel_upper = filter_joint.motion_limits.upper if filter_joint.motion_limits is not None else None
    if panel_upper is not None:
        with ctx.pose({filter_joint: panel_upper}):
            panel_open_aabb = ctx.part_element_world_aabb(filter_panel, elem="panel_frame")
        ctx.check(
            "rear filter panel swings outward",
            panel_closed_aabb is not None
            and panel_open_aabb is not None
            and panel_open_aabb[1][1] > panel_closed_aabb[1][1] + 0.10,
            details=f"closed={panel_closed_aabb}, open={panel_open_aabb}",
        )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_0: 0.0013, button_joint_1: 0.0013}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_pressed = ctx.part_world_position(button_1)
    ctx.check(
        "button_0 presses inward",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.0005,
        details=f"rest={button_0_rest}, pressed={button_0_pressed}",
    )
    ctx.check(
        "button_1 presses inward",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[2] < button_1_rest[2] - 0.0005,
        details=f"rest={button_1_rest}, pressed={button_1_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
