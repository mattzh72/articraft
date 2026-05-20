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


CABINET_WIDTH = 0.320
CABINET_DEPTH = 0.225
CABINET_HEIGHT = 0.575
WALL = 0.012
ROOF = 0.016

BUCKET_WIDTH = 0.252
BUCKET_DEPTH = 0.150
BUCKET_HEIGHT = 0.188
BUCKET_WALL = 0.0025
BUCKET_TRAVEL = 0.105

FILTER_DOOR_WIDTH = 0.216
FILTER_DOOR_HEIGHT = 0.266
FILTER_DOOR_THICKNESS = 0.006

CONTROL_STRIP_WIDTH = 0.236
CONTROL_STRIP_DEPTH = 0.076
CONTROL_STRIP_HEIGHT = 0.006

OUTLET_SLOT_WIDTH = 0.138
OUTLET_SLOT_DEPTH = 0.038
OUTLET_FLAP_WIDTH = 0.148
OUTLET_FLAP_DEPTH = 0.048
OUTLET_FLAP_THICKNESS = 0.004


def _cabinet_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        CABINET_WIDTH,
        CABINET_DEPTH,
        CABINET_HEIGHT,
        centered=(True, True, False),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            CABINET_WIDTH - 2.0 * WALL,
            CABINET_DEPTH - 2.0 * WALL,
            CABINET_HEIGHT - WALL - ROOF,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, WALL))
    )

    shell = outer.cut(inner)

    front_bucket_cut = (
        cq.Workplane("XY")
        .box(0.268, 0.050, 0.206, centered=(True, True, False))
        .translate((0.0, -CABINET_DEPTH / 2.0 - 0.010, 0.018))
    )
    rear_filter_cut = (
        cq.Workplane("XY")
        .box(0.224, 0.052, 0.278, centered=(True, True, False))
        .translate((0.0, CABINET_DEPTH / 2.0 + 0.010, 0.242))
    )
    outlet_cut = (
        cq.Workplane("XY")
        .box(OUTLET_SLOT_WIDTH, OUTLET_SLOT_DEPTH, ROOF + 0.010, centered=(True, True, False))
        .translate((0.0, 0.050, CABINET_HEIGHT - ROOF - 0.002))
    )

    return shell.cut(front_bucket_cut).cut(rear_filter_cut).cut(outlet_cut)


def _bucket_shell() -> cq.Workplane:
    tub = (
        cq.Workplane("XY")
        .box(BUCKET_WIDTH, BUCKET_DEPTH, BUCKET_HEIGHT, centered=(True, False, False))
        .translate((0.0, 0.012, 0.0))
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            BUCKET_WIDTH - 2.0 * BUCKET_WALL,
            BUCKET_DEPTH - BUCKET_WALL - 0.006,
            BUCKET_HEIGHT - BUCKET_WALL,
            centered=(True, False, False),
        )
        .translate((0.0, 0.012 + BUCKET_WALL, BUCKET_WALL))
    )
    front_face = (
        cq.Workplane("XY")
        .box(BUCKET_WIDTH - 0.006, 0.018, BUCKET_HEIGHT - 0.010, centered=(True, True, False))
        .translate((0.0, 0.003, 0.005))
    )
    grip_cut = (
        cq.Workplane("XY")
        .box(0.122, 0.010, 0.038, centered=(True, True, True))
        .translate((0.0, -0.003, 0.122))
    )
    return tub.cut(cavity).union(front_face).cut(grip_cut)


def _filter_door() -> cq.Workplane:
    door = (
        cq.Workplane("XY")
        .box(
            FILTER_DOOR_WIDTH,
            FILTER_DOOR_THICKNESS,
            FILTER_DOOR_HEIGHT,
            centered=(False, True, False),
        )
        .translate((-FILTER_DOOR_WIDTH, 0.0, 0.0))
    )

    slot_length = FILTER_DOOR_WIDTH - 0.050
    for z_center in (0.060, 0.088, 0.116, 0.144, 0.172, 0.200):
        slot = (
            cq.Workplane("XY")
            .box(slot_length, FILTER_DOOR_THICKNESS + 0.010, 0.006, centered=(True, True, True))
            .translate((-FILTER_DOOR_WIDTH / 2.0, 0.0, z_center))
        )
        door = door.cut(slot)

    for z_start, length in ((0.000, 0.052), (0.094, 0.078), (0.214, 0.052)):
        knuckle = cq.Workplane("XY").circle(0.004).extrude(length).translate((0.0015, 0.0, z_start))
        door = door.union(knuckle)

    pull_boss = (
        cq.Workplane("XY")
        .box(0.014, 0.010, 0.052, centered=(True, True, True))
        .translate((-FILTER_DOOR_WIDTH + 0.010, 0.000, 0.142))
    )
    return door.union(pull_boss)


def _outlet_flap() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(
            OUTLET_FLAP_WIDTH,
            OUTLET_FLAP_DEPTH,
            OUTLET_FLAP_THICKNESS,
            centered=(True, False, False),
        )
        .translate((0.0, -OUTLET_FLAP_DEPTH, 0.0))
    )
    hinge_rib = (
        cq.Workplane("XY")
        .box(OUTLET_FLAP_WIDTH * 0.72, 0.010, 0.008, centered=(True, False, False))
        .translate((0.0, -0.010, 0.0))
    )
    return plate.union(hinge_rib)


def _aabb_text(aabb) -> str:
    if aabb is None:
        return "None"
    mins, maxs = aabb
    return f"min={tuple(round(v, 4) for v in mins)}, max={tuple(round(v, 4) for v in maxs)}"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_dehumidifier")

    body_light = model.material("body_light", rgba=(0.90, 0.91, 0.89, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.24, 0.26, 0.28, 1.0))
    control_dark = model.material("control_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    bucket_grey = model.material("bucket_grey", rgba=(0.82, 0.83, 0.82, 1.0))
    filter_grey = model.material("filter_grey", rgba=(0.58, 0.61, 0.63, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((WALL, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-(CABINET_WIDTH - WALL) / 2.0, 0.0, CABINET_HEIGHT / 2.0)),
        material=body_light,
        name="left_wall",
    )
    cabinet.visual(
        Box((WALL, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=((CABINET_WIDTH - WALL) / 2.0, 0.0, CABINET_HEIGHT / 2.0)),
        material=body_light,
        name="right_wall",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, WALL)),
        origin=Origin(xyz=(0.000, 0.000, WALL / 2.0)),
        material=body_light,
        name="floor_pan",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, ROOF)),
        origin=Origin(xyz=(0.000, 0.000, CABINET_HEIGHT - ROOF / 2.0)),
        material=body_light,
        name="roof_pan",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, WALL, CABINET_HEIGHT - 0.224)),
        origin=Origin(
            xyz=(
                0.000,
                -CABINET_DEPTH / 2.0 + WALL / 2.0,
                0.224 + (CABINET_HEIGHT - 0.224) / 2.0,
            )
        ),
        material=body_light,
        name="front_upper",
    )
    cabinet.visual(
        Box((0.026, WALL, 0.206)),
        origin=Origin(xyz=(-0.147, -CABINET_DEPTH / 2.0 + WALL / 2.0, 0.121)),
        material=body_light,
        name="front_jamb_0",
    )
    cabinet.visual(
        Box((0.026, WALL, 0.206)),
        origin=Origin(xyz=(0.147, -CABINET_DEPTH / 2.0 + WALL / 2.0, 0.121)),
        material=body_light,
        name="front_jamb_1",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, WALL, 0.242)),
        origin=Origin(xyz=(0.000, CABINET_DEPTH / 2.0 - WALL / 2.0, 0.121)),
        material=body_light,
        name="rear_lower",
    )
    cabinet.visual(
        Box((0.048, WALL, 0.278)),
        origin=Origin(xyz=(-0.136, CABINET_DEPTH / 2.0 - WALL / 2.0, 0.381)),
        material=body_light,
        name="rear_jamb_0",
    )
    cabinet.visual(
        Box((0.048, WALL, 0.278)),
        origin=Origin(xyz=(0.136, CABINET_DEPTH / 2.0 - WALL / 2.0, 0.381)),
        material=body_light,
        name="rear_jamb_1",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, WALL, 0.055)),
        origin=Origin(xyz=(0.000, CABINET_DEPTH / 2.0 - WALL / 2.0, 0.5475)),
        material=body_light,
        name="rear_top",
    )
    cabinet.visual(
        Box((CONTROL_STRIP_WIDTH, CONTROL_STRIP_DEPTH, CONTROL_STRIP_HEIGHT)),
        origin=Origin(xyz=(0.000, -0.026, CABINET_HEIGHT + CONTROL_STRIP_HEIGHT / 2.0)),
        material=panel_dark,
        name="control_strip",
    )
    cabinet.visual(
        Box((0.164, 0.056, 0.004)),
        origin=Origin(xyz=(0.000, 0.052, CABINET_HEIGHT + 0.002)),
        material=panel_dark,
        name="outlet_surround",
    )
    cabinet.visual(
        Box((0.004, 0.006, 0.270)),
        origin=Origin(xyz=(0.1135, CABINET_DEPTH / 2.0 + 0.003, 0.248 + 0.135)),
        material=filter_grey,
        name="door_hinge_spine",
    )

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_bucket_shell(), "dehumidifier_bucket"),
        material=bucket_grey,
        name="bucket_shell",
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        mesh_from_cadquery(_filter_door(), "dehumidifier_filter_door"),
        material=filter_grey,
        name="filter_panel",
    )

    flap = model.part("outlet_flap")
    flap.visual(
        mesh_from_cadquery(_outlet_flap(), "dehumidifier_outlet_flap"),
        material=panel_dark,
        name="flap_panel",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.020,
                body_style="skirted",
                top_diameter=0.032,
                skirt=KnobSkirt(0.050, 0.005, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "dehumidifier_dial",
        ),
        material=control_dark,
        name="dial_knob",
    )

    button_positions = (0.022, 0.058)
    for index, x_pos in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.020, 0.014, 0.005)),
            origin=Origin(xyz=(0.000, 0.000, 0.0025)),
            material=control_dark,
            name="button_cap",
        )
        model.articulation(
            f"cabinet_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, -0.024, CABINET_HEIGHT + CONTROL_STRIP_HEIGHT)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.050, lower=0.0, upper=0.0015),
        )

    model.articulation(
        "cabinet_to_bucket",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=bucket,
        origin=Origin(xyz=(0.000, -CABINET_DEPTH / 2.0 + 0.006, WALL)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.18, lower=0.0, upper=BUCKET_TRAVEL),
    )

    model.articulation(
        "cabinet_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=filter_door,
        origin=Origin(xyz=(0.106, CABINET_DEPTH / 2.0 + 0.003, 0.248)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(118.0),
        ),
    )

    model.articulation(
        "cabinet_to_outlet_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(0.000, 0.074, CABINET_HEIGHT + 0.001)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    model.articulation(
        "cabinet_to_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(-0.060, -0.024, CABINET_HEIGHT + CONTROL_STRIP_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.30, velocity=7.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    bucket = object_model.get_part("bucket")
    filter_door = object_model.get_part("filter_door")
    flap = object_model.get_part("outlet_flap")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    bucket_joint = object_model.get_articulation("cabinet_to_bucket")
    door_joint = object_model.get_articulation("cabinet_to_filter_door")
    flap_joint = object_model.get_articulation("cabinet_to_outlet_flap")
    dial_joint = object_model.get_articulation("cabinet_to_dial")
    button_joint_0 = object_model.get_articulation("cabinet_to_button_0")
    button_joint_1 = object_model.get_articulation("cabinet_to_button_1")

    ctx.check(
        "bucket_joint_is_prismatic",
        bucket_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={bucket_joint.articulation_type!r}",
    )
    ctx.check(
        "filter_door_joint_is_revolute",
        door_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={door_joint.articulation_type!r}",
    )
    ctx.check(
        "outlet_flap_joint_is_revolute",
        flap_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={flap_joint.articulation_type!r}",
    )
    ctx.check(
        "dial_joint_is_continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type!r}",
    )
    ctx.check(
        "button_joints_are_prismatic",
        (
            button_joint_0.articulation_type == ArticulationType.PRISMATIC
            and button_joint_1.articulation_type == ArticulationType.PRISMATIC
        ),
        details=f"types={(button_joint_0.articulation_type, button_joint_1.articulation_type)!r}",
    )

    ctx.expect_within(bucket, cabinet, axes="x", margin=0.0, name="bucket stays within cabinet width")

    bucket_limits = bucket_joint.motion_limits
    if bucket_limits is not None and bucket_limits.upper is not None:
        rest_bucket_aabb = ctx.part_world_aabb(bucket)
        with ctx.pose({bucket_joint: bucket_limits.upper}):
            ctx.expect_within(
                bucket,
                cabinet,
                axes="x",
                margin=0.0,
                name="extended bucket stays laterally aligned",
            )
            ctx.expect_overlap(
                bucket,
                cabinet,
                axes="y",
                min_overlap=0.045,
                name="extended bucket retains insertion",
            )
            extended_bucket_aabb = ctx.part_world_aabb(bucket)

        ctx.check(
            "bucket_slides_forward",
            rest_bucket_aabb is not None
            and extended_bucket_aabb is not None
            and extended_bucket_aabb[0][1] < rest_bucket_aabb[0][1] - 0.080,
            details=f"rest={_aabb_text(rest_bucket_aabb)}, extended={_aabb_text(extended_bucket_aabb)}",
        )

    door_limits = door_joint.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        closed_door_aabb = ctx.part_world_aabb(filter_door)
        with ctx.pose({door_joint: door_limits.upper}):
            opened_door_aabb = ctx.part_world_aabb(filter_door)

        ctx.check(
            "filter_door_swings_outward",
            closed_door_aabb is not None
            and opened_door_aabb is not None
            and opened_door_aabb[1][1] > closed_door_aabb[1][1] + 0.050,
            details=f"closed={_aabb_text(closed_door_aabb)}, open={_aabb_text(opened_door_aabb)}",
        )

    flap_limits = flap_joint.motion_limits
    if flap_limits is not None and flap_limits.upper is not None:
        closed_flap_aabb = ctx.part_world_aabb(flap)
        with ctx.pose({flap_joint: flap_limits.upper}):
            opened_flap_aabb = ctx.part_world_aabb(flap)

        ctx.check(
            "outlet_flap_lifts_up",
            closed_flap_aabb is not None
            and opened_flap_aabb is not None
            and opened_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.028,
            details=f"closed={_aabb_text(closed_flap_aabb)}, open={_aabb_text(opened_flap_aabb)}",
        )

    for button_part, button_joint, label in (
        (button_0, button_joint_0, "button_0"),
        (button_1, button_joint_1, "button_1"),
    ):
        button_limits = button_joint.motion_limits
        if button_limits is None or button_limits.upper is None:
            continue
        rest_button_aabb = ctx.part_world_aabb(button_part)
        with ctx.pose({button_joint: button_limits.upper}):
            pressed_button_aabb = ctx.part_world_aabb(button_part)

        ctx.check(
            f"{label}_presses_down",
            rest_button_aabb is not None
            and pressed_button_aabb is not None
            and pressed_button_aabb[1][2] < rest_button_aabb[1][2] - 0.001,
            details=f"rest={_aabb_text(rest_button_aabb)}, pressed={_aabb_text(pressed_button_aabb)}",
        )

    return ctx.report()


object_model = build_object_model()
