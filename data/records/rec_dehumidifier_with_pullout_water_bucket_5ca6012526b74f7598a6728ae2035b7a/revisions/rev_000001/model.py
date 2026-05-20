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
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_DEPTH = 0.34
BODY_WIDTH = 0.39
BODY_HEIGHT = 0.67
BODY_WALL = 0.004
BODY_BOTTOM = 0.012
BODY_TOP = 0.028
BASE_LIFT = 0.052

BUCKET_DEPTH = 0.265
BUCKET_WIDTH = 0.292
BUCKET_HEIGHT = 0.215
BUCKET_WALL = 0.0035
BUCKET_BOTTOM = 0.0045

DOOR_WIDTH = 0.30
DOOR_HEIGHT = 0.44
DOOR_THICKNESS = 0.010


def _body_shell() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_DEPTH - 2.0 * BODY_WALL,
            BODY_WIDTH - 2.0 * BODY_WALL,
            BODY_HEIGHT - BODY_BOTTOM - BODY_TOP,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BODY_BOTTOM))
    )
    front_bucket_opening = (
        cq.Workplane("XY")
        .box(0.080, 0.308, 0.248, centered=(True, True, False))
        .translate((BODY_DEPTH * 0.5 - 0.040, 0.0, 0.067))
    )
    rear_filter_opening = (
        cq.Workplane("XY")
        .box(0.080, 0.308, 0.446, centered=(True, True, False))
        .translate((-BODY_DEPTH * 0.5 + 0.040, 0.0, 0.183))
    )
    top_recess = (
        cq.Workplane("XY")
        .box(0.218, 0.144, 0.008, centered=(True, True, False))
        .translate((0.008, 0.0, BODY_HEIGHT - 0.008))
    )
    front_window_bezel = (
        cq.Workplane("XY")
        .box(0.012, 0.110, 0.155, centered=(True, True, False))
        .translate((BODY_DEPTH * 0.5 - 0.006, 0.0, 0.095))
    )
    return body.cut(inner).cut(front_bucket_opening).cut(rear_filter_opening).cut(top_recess).cut(front_window_bezel)


def _bucket_shell() -> cq.Workplane:
    bucket_outer = (
        cq.Workplane("XY")
        .box(BUCKET_DEPTH, BUCKET_WIDTH, BUCKET_HEIGHT, centered=(False, True, False))
        .translate((-BUCKET_DEPTH, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.010)
    )
    bucket_inner = (
        cq.Workplane("XY")
        .box(
            BUCKET_DEPTH - 2.0 * BUCKET_WALL,
            BUCKET_WIDTH - 2.0 * BUCKET_WALL,
            BUCKET_HEIGHT - BUCKET_BOTTOM,
            centered=(False, True, False),
        )
        .translate((-BUCKET_DEPTH + BUCKET_WALL, 0.0, BUCKET_BOTTOM))
    )
    level_window = (
        cq.Workplane("XY")
        .box(0.012, 0.082, 0.112, centered=(False, True, False))
        .translate((-0.010, 0.0, 0.060))
    )
    handle_recess = (
        cq.Workplane("XY")
        .box(0.030, 0.160, 0.028, centered=(False, True, False))
        .translate((-0.024, 0.0, BUCKET_HEIGHT - 0.042))
    )
    return bucket_outer.cut(bucket_inner).cut(level_window).cut(handle_recess)


def _filter_door_frame() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(DOOR_THICKNESS, DOOR_WIDTH, DOOR_HEIGHT, centered=(False, False, False))
        .translate((-DOOR_THICKNESS, -DOOR_WIDTH, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(DOOR_THICKNESS + 0.004, DOOR_WIDTH - 0.036, DOOR_HEIGHT - 0.042, centered=(False, False, False))
        .translate((-DOOR_THICKNESS - 0.002, -DOOR_WIDTH + 0.018, 0.021))
    )
    return outer.cut(inner)


def _control_panel_insert() -> cq.Workplane:
    panel = cq.Workplane("XY").box(0.220, 0.146, 0.008, centered=(True, True, False))
    dial_relief = (
        cq.Workplane("XY")
        .cylinder(0.0015, 0.032, centered=(True, True, False))
        .translate((0.050, -0.040, 0.0065))
    )
    button_pockets = [
        (
            cq.Workplane("XY")
            .box(0.026, 0.020, 0.002, centered=(True, True, False))
            .translate((0.000, pocket_y, 0.006))
        )
        for pocket_y in (-0.034, 0.000, 0.034)
    ]
    result = panel.cut(dial_relief)
    for pocket in button_pockets:
        result = result.cut(pocket)
    return result


def _add_caster(
    model: ArticulatedObject,
    body,
    *,
    name_prefix: str,
    mount_xyz: tuple[float, float, float],
    fork_material,
    wheel_material,
    hub_material,
) -> None:
    caster = model.part(f"{name_prefix}_caster")
    caster.visual(
        Box((0.036, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=fork_material,
        name="mount_plate",
    )
    caster.visual(
        Box((0.012, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=fork_material,
        name="swivel_block",
    )
    caster.visual(
        Box((0.018, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=fork_material,
        name="fork_bridge",
    )
    for side_index, side_y in enumerate((-0.009, 0.009)):
        caster.visual(
            Box((0.018, 0.004, 0.024)),
            origin=Origin(xyz=(0.0, side_y, -0.022)),
            material=fork_material,
            name=f"fork_arm_{side_index}",
        )

    wheel = model.part(f"{name_prefix}_wheel")
    wheel.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_material,
        name="wheel_shell",
    )
    wheel.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="wheel_hub",
    )

    model.articulation(
        f"{name_prefix}_caster_mount",
        ArticulationType.FIXED,
        parent=body,
        child=caster,
        origin=Origin(xyz=mount_xyz),
    )
    model.articulation(
        f"{name_prefix}_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=caster,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=12.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_dehumidifier")

    shell_white = model.material("shell_white", rgba=(0.91, 0.92, 0.90, 1.0))
    panel_black = model.material("panel_black", rgba=(0.16, 0.17, 0.18, 1.0))
    bucket_white = model.material("bucket_white", rgba=(0.95, 0.95, 0.94, 1.0))
    water_window = model.material("water_window", rgba=(0.62, 0.77, 0.88, 0.34))
    filter_dark = model.material("filter_dark", rgba=(0.35, 0.38, 0.39, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.70, 0.72, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "dehumidifier_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, BASE_LIFT)),
        material=shell_white,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_control_panel_insert(), "dehumidifier_control_panel"),
        origin=Origin(xyz=(0.008, 0.0, BASE_LIFT + BODY_HEIGHT - 0.008)),
        material=panel_black,
        name="control_panel",
    )
    body.visual(
        Box((0.232, BODY_WIDTH - 2.0 * BODY_WALL, 0.010)),
        origin=Origin(xyz=(0.026, 0.0, BASE_LIFT + 0.069)),
        material=shell_white,
        name="bucket_shelf",
    )

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_bucket_shell(), "dehumidifier_bucket_shell"),
        material=bucket_white,
        name="bucket_shell",
    )
    bucket.visual(
        Box((0.004, 0.086, 0.116)),
        origin=Origin(xyz=(-0.002, 0.0, 0.116)),
        material=water_window,
        name="bucket_window",
    )
    bucket.visual(
        Box((0.010, 0.170, 0.018)),
        origin=Origin(xyz=(-0.005, 0.0, BUCKET_HEIGHT - 0.020)),
        material=bucket_white,
        name="bucket_handle_lip",
    )

    door = model.part("filter_door")
    door.visual(
        mesh_from_cadquery(_filter_door_frame(), "dehumidifier_filter_door_frame"),
        material=shell_white,
        name="door_frame",
    )
    door.visual(
        Box((0.004, DOOR_WIDTH - 0.024, DOOR_HEIGHT - 0.030)),
        origin=Origin(
            xyz=(-DOOR_THICKNESS * 0.5, -DOOR_WIDTH * 0.5, DOOR_HEIGHT * 0.5),
        ),
        material=filter_dark,
        name="filter_mesh",
    )
    door.visual(
        Box((0.016, 0.020, 0.120)),
        origin=Origin(xyz=(-0.008, -DOOR_WIDTH + 0.010, 0.200)),
        material=shell_white,
        name="door_pull",
    )

    model.articulation(
        "bucket_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(BODY_DEPTH * 0.5 - 0.006, 0.0, BASE_LIFT + 0.074)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.25, lower=0.0, upper=0.170),
    )
    model.articulation(
        "rear_filter_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-BODY_DEPTH * 0.5, DOOR_WIDTH * 0.5, BASE_LIFT + 0.183)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.55),
    )

    humidistat_dial = model.part("humidistat_dial")
    humidistat_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.022,
                body_style="skirted",
                top_diameter=0.044,
                base_diameter=0.052,
                edge_radius=0.002,
                crown_radius=0.001,
                center=False,
            ),
            "humidistat_dial",
        ),
        material=panel_black,
        name="dial_knob",
    )
    humidistat_dial.visual(
        Box((0.016, 0.003, 0.002)),
        origin=Origin(xyz=(0.014, 0.0, 0.019)),
        material=shell_white,
        name="dial_indicator",
    )
    model.articulation(
        "humidistat_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=humidistat_dial,
        origin=Origin(xyz=(0.058, -0.040, BASE_LIFT + BODY_HEIGHT - 0.0015)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    for index, button_y in enumerate((-0.034, 0.000, 0.034)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.024, 0.018, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=shell_white,
            name="button_cap",
        )
        model.articulation(
            f"button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.008, button_y, BASE_LIFT + BODY_HEIGHT - 0.002)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.002),
        )

    caster_positions = {
        "front_left": (0.116, 0.145, BASE_LIFT),
        "front_right": (0.116, -0.145, BASE_LIFT),
        "rear_left": (-0.116, 0.145, BASE_LIFT),
        "rear_right": (-0.116, -0.145, BASE_LIFT),
    }
    for name_prefix, mount_xyz in caster_positions.items():
        _add_caster(
            model,
            body,
            name_prefix=name_prefix,
            mount_xyz=mount_xyz,
            fork_material=panel_black,
            wheel_material=wheel_gray,
            hub_material=panel_black,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    door = object_model.get_part("filter_door")
    dial = object_model.get_part("humidistat_dial")
    bucket_slide = object_model.get_articulation("bucket_slide")
    rear_filter_door = object_model.get_articulation("rear_filter_door")
    dial_turn = object_model.get_articulation("humidistat_turn")

    bucket_limits = bucket_slide.motion_limits
    door_limits = rear_filter_door.motion_limits

    ctx.expect_overlap(
        bucket,
        body,
        axes="yz",
        min_overlap=0.18,
        name="bucket aligns with lower body opening",
    )
    ctx.expect_overlap(
        bucket,
        body,
        axes="x",
        min_overlap=0.09,
        name="bucket remains inserted in the cabinet at rest",
    )
    ctx.expect_gap(
        body,
        door,
        axis="x",
        max_gap=0.004,
        max_penetration=0.0,
        name="rear filter door sits nearly flush with the rear shell",
    )

    if bucket_limits is not None and bucket_limits.upper is not None:
        bucket_rest = ctx.part_world_position(bucket)
        with ctx.pose({bucket_slide: bucket_limits.upper}):
            bucket_extended = ctx.part_world_position(bucket)
            ctx.expect_overlap(
                bucket,
                body,
                axes="x",
                min_overlap=0.09,
                name="bucket retains insertion when fully extended",
            )
        ctx.check(
            "bucket extends toward the front",
            bucket_rest is not None
            and bucket_extended is not None
            and bucket_extended[0] > bucket_rest[0] + 0.12,
            details=f"rest={bucket_rest}, extended={bucket_extended}",
        )

    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({rear_filter_door: door_limits.upper}):
            open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "rear filter door swings outward",
            open_aabb is not None and open_aabb[0][0] < -BODY_DEPTH * 0.5 - 0.08,
            details=f"open_aabb={open_aabb}",
        )

    ctx.check(
        "humidistat uses a continuous dial articulation",
        dial_turn.articulation_type == ArticulationType.CONTINUOUS
        and dial_turn.motion_limits is not None
        and dial_turn.motion_limits.lower is None
        and dial_turn.motion_limits.upper is None,
        details=f"type={dial_turn.articulation_type}, limits={dial_turn.motion_limits}",
    )
    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_knob",
        elem_b="control_panel",
        contact_tol=0.001,
        name="humidistat dial sits on the control panel",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"button_{index}_press")
        limits = button_joint.motion_limits
        button_rest = ctx.part_world_position(button)
        button_pressed = None
        if limits is not None and limits.upper is not None:
            with ctx.pose({button_joint: limits.upper}):
                button_pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses downward",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[2] < button_rest[2] - 0.0015,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    for name_prefix in ("front_left", "front_right", "rear_left", "rear_right"):
        wheel = object_model.get_part(f"{name_prefix}_wheel")
        wheel_joint = object_model.get_articulation(f"{name_prefix}_wheel_spin")
        wheel_aabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"{name_prefix}_wheel uses continuous axle spin",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and wheel_joint.motion_limits is not None
            and wheel_joint.motion_limits.lower is None
            and wheel_joint.motion_limits.upper is None,
            details=f"type={wheel_joint.articulation_type}, limits={wheel_joint.motion_limits}",
        )
        ctx.check(
            f"{name_prefix}_wheel sits on the floor plane",
            wheel_aabb is not None and abs(wheel_aabb[0][2]) <= 0.002,
            details=f"aabb={wheel_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
