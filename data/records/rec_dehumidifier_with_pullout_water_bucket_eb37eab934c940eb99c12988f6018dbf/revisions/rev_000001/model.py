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

BODY_DEPTH = 0.245
BODY_WIDTH = 0.355
BODY_SHELL_HEIGHT = 0.575
BODY_BOTTOM = 0.040
BODY_TOP = BODY_BOTTOM + BODY_SHELL_HEIGHT

BUCKET_DEPTH = 0.210
BUCKET_WIDTH = 0.276
BUCKET_HEIGHT = 0.205
BUCKET_WALL = 0.004
BUCKET_BOTTOM = 0.072
BUCKET_TRAVEL = 0.135
BUCKET_CAVITY_DEPTH = 0.218
BUCKET_CAVITY_WIDTH = 0.288
BUCKET_CAVITY_HEIGHT = 0.214

FILTER_DOOR_WIDTH = 0.230
FILTER_DOOR_HEIGHT = 0.315
FILTER_DOOR_THICKNESS = 0.004
FILTER_DOOR_BOTTOM = 0.186

CONTROL_PANEL_DEPTH = 0.112
CONTROL_PANEL_WIDTH = 0.250
CONTROL_PANEL_HEIGHT = 0.016
CONTROL_PANEL_X = 0.060

TOP_GRILLE_DEPTH = 0.104
TOP_GRILLE_WIDTH = 0.286
TOP_GRILLE_HEIGHT = 0.008
TOP_GRILLE_X = -0.058

DIAL_DIAMETER = 0.044
DIAL_HEIGHT = 0.020

BUTTON_DEPTH = 0.026
BUTTON_WIDTH = 0.018
BUTTON_HEIGHT = 0.008
BUTTON_TRAVEL = 0.001
BUTTON_Y_POSITIONS = (0.000, 0.047, 0.094)

WHEEL_RADIUS = 0.016
WHEEL_WIDTH = 0.014
AXLE_Z = 0.018


def _bottom_center(box: cq.Workplane, height: float) -> cq.Workplane:
    return box.translate((0.0, 0.0, height * 0.5))


def _front_bottom_box(depth: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(depth, width, height).translate((-depth * 0.5, 0.0, height * 0.5))


def _build_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_SHELL_HEIGHT)
        .edges("|Z")
        .fillet(0.026)
        .translate((0.0, 0.0, BODY_BOTTOM + BODY_SHELL_HEIGHT * 0.5))
    )

    bucket_cavity = cq.Workplane("XY").box(
        BUCKET_CAVITY_DEPTH,
        BUCKET_CAVITY_WIDTH,
        BUCKET_CAVITY_HEIGHT,
    ).translate(
        (
            BODY_DEPTH * 0.5 - BUCKET_CAVITY_DEPTH * 0.5 + 0.001,
            0.0,
            BUCKET_BOTTOM + BUCKET_CAVITY_HEIGHT * 0.5,
        )
    )
    filter_recess = cq.Workplane("XY").box(
        0.028,
        FILTER_DOOR_WIDTH - 0.022,
        FILTER_DOOR_HEIGHT - 0.034,
    ).translate(
        (
            -BODY_DEPTH * 0.5 + 0.014,
            0.0,
            FILTER_DOOR_BOTTOM + FILTER_DOOR_HEIGHT * 0.5,
        )
    )
    front_status_window = cq.Workplane("XY").box(0.012, 0.124, 0.020).translate(
        (
            BODY_DEPTH * 0.5 - 0.006,
            0.0,
            BODY_BOTTOM + BODY_SHELL_HEIGHT - 0.145,
        )
    )

    return body.cut(bucket_cavity).cut(filter_recess).cut(front_status_window)


def _build_bucket_shape() -> cq.Workplane:
    outer = _front_bottom_box(BUCKET_DEPTH, BUCKET_WIDTH, BUCKET_HEIGHT)
    inner = cq.Workplane("XY").box(
        BUCKET_DEPTH - 2.0 * BUCKET_WALL,
        BUCKET_WIDTH - 2.0 * BUCKET_WALL,
        BUCKET_HEIGHT - BUCKET_WALL,
    ).translate(
        (
            -BUCKET_DEPTH * 0.5,
            0.0,
            (BUCKET_HEIGHT + BUCKET_WALL) * 0.5,
        )
    )
    pull_recess = cq.Workplane("XY").box(0.016, 0.120, 0.032).translate((-0.008, 0.0, 0.138))
    return outer.cut(inner).cut(pull_recess)


def _build_filter_door_shape() -> cq.Workplane:
    door = cq.Workplane("XY").box(
        FILTER_DOOR_THICKNESS,
        FILTER_DOOR_WIDTH,
        FILTER_DOOR_HEIGHT,
    ).translate(
        (
            -FILTER_DOOR_THICKNESS * 0.5,
            -FILTER_DOOR_WIDTH * 0.5,
            FILTER_DOOR_HEIGHT * 0.5,
        )
    )

    for z_pos in (0.068, 0.108, 0.148, 0.188, 0.228):
        slot = cq.Workplane("XY").box(
            FILTER_DOOR_THICKNESS + 0.006,
            FILTER_DOOR_WIDTH * 0.72,
            0.012,
        ).translate(
            (
                -FILTER_DOOR_THICKNESS * 0.5,
                -FILTER_DOOR_WIDTH * 0.5,
                z_pos,
            )
        )
        door = door.cut(slot)

    for start_z, length in ((0.018, 0.058), (0.132, 0.060), (0.250, 0.050)):
        hinge_barrel = (
            cq.Workplane("XY")
            .circle(0.0055)
            .extrude(length)
            .translate((-FILTER_DOOR_THICKNESS * 0.85, 0.0, start_z))
        )
        door = door.union(hinge_barrel)

    latch_tab = cq.Workplane("XY").box(0.006, 0.014, 0.064).translate(
        (
            -FILTER_DOOR_THICKNESS * 0.5,
            -FILTER_DOOR_WIDTH + 0.007,
            FILTER_DOOR_HEIGHT * 0.57,
        )
    )
    return door.union(latch_tab)


def _build_control_panel_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(CONTROL_PANEL_DEPTH, CONTROL_PANEL_WIDTH, CONTROL_PANEL_HEIGHT)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, CONTROL_PANEL_HEIGHT * 0.5))
    )


def _build_top_grille_shape() -> cq.Workplane:
    grille = (
        cq.Workplane("XY")
        .box(TOP_GRILLE_DEPTH, TOP_GRILLE_WIDTH, TOP_GRILLE_HEIGHT)
        .edges("|Z")
        .fillet(0.005)
        .translate((0.0, 0.0, TOP_GRILLE_HEIGHT * 0.5))
    )
    for x_pos in (-0.034, -0.022, -0.010, 0.002, 0.014, 0.026):
        slot = cq.Workplane("XY").box(
            0.010,
            TOP_GRILLE_WIDTH - 0.058,
            TOP_GRILLE_HEIGHT + 0.006,
        ).translate((x_pos, 0.0, TOP_GRILLE_HEIGHT * 0.5))
        grille = grille.cut(slot)
    return grille


def _build_button_cap_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BUTTON_DEPTH, BUTTON_WIDTH, BUTTON_HEIGHT)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, BUTTON_HEIGHT * 0.5))
    )


def _build_caster_bracket_shape() -> cq.Workplane:
    left_cheek = cq.Workplane("XY").box(0.006, 0.003, 0.018).translate((0.0, 0.010, 0.000))
    right_cheek = cq.Workplane("XY").box(0.006, 0.003, 0.018).translate((0.0, -0.010, 0.000))
    bridge = cq.Workplane("XY").box(0.006, 0.023, 0.004).translate((0.0, 0.0, 0.019))
    return left_cheek.union(right_cheek).union(bridge)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_dehumidifier")

    body_finish = model.material("body_finish", rgba=(0.90, 0.91, 0.88, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.24, 0.27, 0.29, 1.0))
    grille_finish = model.material("grille_finish", rgba=(0.18, 0.20, 0.22, 1.0))
    bucket_finish = model.material("bucket_finish", rgba=(0.82, 0.84, 0.86, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.14, 0.16, 0.18, 1.0))
    button_finish = model.material("button_finish", rgba=(0.86, 0.88, 0.90, 1.0))
    caster_finish = model.material("caster_finish", rgba=(0.22, 0.24, 0.26, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.72, 0.74, 0.77, 1.0))
    tire_finish = model.material("tire_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    bucket_mesh = mesh_from_cadquery(_build_bucket_shape(), "dehumidifier_bucket")
    door_mesh = mesh_from_cadquery(_build_filter_door_shape(), "rear_filter_door")
    panel_mesh = mesh_from_cadquery(_build_control_panel_shape(), "control_panel")
    grille_mesh = mesh_from_cadquery(_build_top_grille_shape(), "top_grille")
    button_mesh = mesh_from_cadquery(_build_button_cap_shape(), "button_cap")

    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            DIAL_DIAMETER,
            DIAL_HEIGHT,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.050, 0.005, flare=0.05),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            center=False,
        ),
        "humidistat_dial",
    )

    body = model.part("body")
    upper_shell_bottom = BUCKET_BOTTOM + BUCKET_CAVITY_HEIGHT
    side_wall_width = (BODY_WIDTH - BUCKET_CAVITY_WIDTH) * 0.5
    rear_bridge_depth = BODY_DEPTH - BUCKET_CAVITY_DEPTH

    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, BODY_TOP - upper_shell_bottom)),
        origin=Origin(xyz=(0.0, 0.0, upper_shell_bottom + (BODY_TOP - upper_shell_bottom) * 0.5)),
        material=body_finish,
        name="upper_shell",
    )
    for suffix, y_pos in (
        ("0", -(BUCKET_CAVITY_WIDTH * 0.5 + side_wall_width * 0.5)),
        ("1", BUCKET_CAVITY_WIDTH * 0.5 + side_wall_width * 0.5),
    ):
        body.visual(
            Box((BODY_DEPTH, side_wall_width, upper_shell_bottom - BODY_BOTTOM)),
            origin=Origin(
                xyz=(
                    0.0,
                    y_pos,
                    BODY_BOTTOM + (upper_shell_bottom - BODY_BOTTOM) * 0.5,
                )
            ),
            material=body_finish,
            name=f"side_wall_{suffix}",
        )
    body.visual(
        Box((rear_bridge_depth, BUCKET_CAVITY_WIDTH, upper_shell_bottom - BODY_BOTTOM)),
        origin=Origin(
            xyz=(
                -BODY_DEPTH * 0.5 + rear_bridge_depth * 0.5,
                0.0,
                BODY_BOTTOM + (upper_shell_bottom - BODY_BOTTOM) * 0.5,
            )
        ),
        material=body_finish,
        name="rear_bridge",
    )
    body.visual(
        Box((0.018, BUCKET_CAVITY_WIDTH + 0.010, 0.024)),
        origin=Origin(
            xyz=(
                BODY_DEPTH * 0.5 - 0.009,
                0.0,
                BODY_BOTTOM + 0.012,
            )
        ),
        material=body_finish,
        name="front_sill",
    )
    body.visual(
        Box((0.200, BUCKET_CAVITY_WIDTH - 0.028, 0.006)),
        origin=Origin(
            xyz=(
                0.0045,
                0.0,
                BUCKET_BOTTOM - 0.003,
            )
        ),
        material=body_finish,
        name="bucket_floor",
    )
    body.visual(
        Box((0.006, 0.124, 0.018)),
        origin=Origin(
            xyz=(
                BODY_DEPTH * 0.5 - 0.003,
                0.0,
                BODY_BOTTOM + BODY_SHELL_HEIGHT - 0.145,
            )
        ),
        material=panel_finish,
        name="status_window",
    )

    control_panel = model.part("control_panel")
    control_panel.visual(panel_mesh, material=panel_finish, name="panel_shell")
    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(CONTROL_PANEL_X, 0.0, BODY_TOP)),
    )

    top_grille = model.part("top_grille")
    top_grille.visual(grille_mesh, material=grille_finish, name="grille_shell")
    model.articulation(
        "body_to_top_grille",
        ArticulationType.FIXED,
        parent=body,
        child=top_grille,
        origin=Origin(xyz=(TOP_GRILLE_X, 0.0, BODY_TOP)),
    )

    bucket = model.part("bucket")
    bucket.visual(bucket_mesh, material=bucket_finish, name="bucket_shell")
    model.articulation(
        "body_to_bucket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(BODY_DEPTH * 0.5 - 0.002, 0.0, BUCKET_BOTTOM)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.28, lower=0.0, upper=BUCKET_TRAVEL),
    )

    filter_door = model.part("filter_door")
    filter_door.visual(door_mesh, material=body_finish, name="door_panel")
    model.articulation(
        "body_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_door,
        origin=Origin(
            xyz=(
                -BODY_DEPTH * 0.5 - 0.002,
                FILTER_DOOR_WIDTH * 0.5,
                FILTER_DOOR_BOTTOM,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    humidistat_dial = model.part("humidistat_dial")
    humidistat_dial.visual(dial_mesh, material=dial_finish, name="dial_cap")
    model.articulation(
        "control_panel_to_humidistat_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=humidistat_dial,
        origin=Origin(xyz=(0.000, -0.078, CONTROL_PANEL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.20, velocity=6.0),
    )

    for index, y_pos in enumerate(BUTTON_Y_POSITIONS):
        button = model.part(f"button_{index}")
        button.visual(
            button_mesh,
            material=button_finish,
            name="button_cap",
        )
        model.articulation(
            f"control_panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(0.006, y_pos, CONTROL_PANEL_HEIGHT)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    caster_layout = (
        ("front_caster_0", "front_wheel_0", BODY_DEPTH * 0.5 - 0.044, -BODY_WIDTH * 0.5 + 0.044),
        ("front_caster_1", "front_wheel_1", BODY_DEPTH * 0.5 - 0.044, BODY_WIDTH * 0.5 - 0.044),
        ("rear_caster_0", "rear_wheel_0", -BODY_DEPTH * 0.5 + 0.044, -BODY_WIDTH * 0.5 + 0.044),
        ("rear_caster_1", "rear_wheel_1", -BODY_DEPTH * 0.5 + 0.044, BODY_WIDTH * 0.5 - 0.044),
    )
    for caster_name, wheel_name, x_pos, y_pos in caster_layout:
        caster = model.part(caster_name)
        caster.visual(
            Cylinder(radius=0.0035, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.019)),
            material=caster_finish,
            name="caster_stem",
        )
        for suffix, cheek_y in (("0", -0.009), ("1", 0.009)):
            caster.visual(
                Box((0.006, 0.003, 0.022)),
                origin=Origin(xyz=(0.0, cheek_y, 0.011)),
                material=caster_finish,
                name=f"fork_arm_{suffix}",
            )
        caster.visual(
            Box((0.006, 0.021, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.018)),
            material=caster_finish,
            name="fork_bridge",
        )
        model.articulation(
            f"body_to_{caster_name}",
            ArticulationType.FIXED,
            parent=body,
            child=caster,
            origin=Origin(xyz=(x_pos, y_pos, AXLE_Z)),
        )

        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=tire_finish,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=WHEEL_RADIUS * 0.62, length=WHEEL_WIDTH + 0.001),
            origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=wheel_hub,
            name="rim",
        )
        model.articulation(
            f"{caster_name}_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=14.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    control_panel = object_model.get_part("control_panel")
    filter_door = object_model.get_part("filter_door")

    bucket_joint = object_model.get_articulation("body_to_bucket")
    door_joint = object_model.get_articulation("body_to_filter_door")
    dial_joint = object_model.get_articulation("control_panel_to_humidistat_dial")
    button_joints = [object_model.get_articulation(f"control_panel_to_button_{index}") for index in range(3)]
    button_parts = [object_model.get_part(f"button_{index}") for index in range(3)]
    wheel_joints = [
        object_model.get_articulation(name)
        for name in (
            "front_caster_0_to_front_wheel_0",
            "front_caster_1_to_front_wheel_1",
            "rear_caster_0_to_rear_wheel_0",
            "rear_caster_1_to_rear_wheel_1",
        )
    ]

    ctx.check(
        "primary articulation types are correct",
        bucket_joint.articulation_type == ArticulationType.PRISMATIC
        and door_joint.articulation_type == ArticulationType.REVOLUTE
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"bucket={bucket_joint.articulation_type}, "
            f"door={door_joint.articulation_type}, "
            f"dial={dial_joint.articulation_type}"
        ),
    )
    ctx.check(
        "buttons use independent prismatic joints",
        all(joint.articulation_type == ArticulationType.PRISMATIC for joint in button_joints),
        details=", ".join(f"{joint.name}={joint.articulation_type}" for joint in button_joints),
    )
    ctx.check(
        "caster wheels use continuous joints",
        all(joint.articulation_type == ArticulationType.CONTINUOUS for joint in wheel_joints),
        details=", ".join(f"{joint.name}={joint.articulation_type}" for joint in wheel_joints),
    )

    ctx.expect_within(
        bucket,
        body,
        axes="yz",
        inner_elem="bucket_shell",
        margin=0.004,
        name="bucket stays within body width and height at rest",
    )
    ctx.expect_overlap(
        bucket,
        body,
        axes="x",
        elem_a="bucket_shell",
        min_overlap=0.180,
        name="bucket remains inserted at rest",
    )
    bucket_rest_pos = ctx.part_world_position(bucket)
    bucket_upper = bucket_joint.motion_limits.upper if bucket_joint.motion_limits is not None else None
    if bucket_upper is not None:
        with ctx.pose({bucket_joint: bucket_upper}):
            ctx.expect_within(
                bucket,
                body,
                axes="yz",
                inner_elem="bucket_shell",
                margin=0.004,
                name="bucket stays aligned with the body opening when extended",
            )
            ctx.expect_overlap(
                bucket,
                body,
                axes="x",
                elem_a="bucket_shell",
                min_overlap=0.072,
                name="bucket retains insertion at full travel",
            )
            bucket_extended_pos = ctx.part_world_position(bucket)
        ctx.check(
            "bucket slides outward from the lower front",
            bucket_rest_pos is not None
            and bucket_extended_pos is not None
            and bucket_extended_pos[0] > bucket_rest_pos[0] + 0.10,
            details=f"rest={bucket_rest_pos}, extended={bucket_extended_pos}",
        )

    ctx.expect_gap(
        body,
        filter_door,
        axis="x",
        negative_elem="door_panel",
        max_gap=0.0045,
        max_penetration=0.0002,
        name="rear filter door closes against the rear face",
    )
    ctx.expect_overlap(
        filter_door,
        body,
        axes="yz",
        elem_a="door_panel",
        min_overlap=0.210,
        name="rear filter door covers the rear filter opening",
    )
    door_closed_center = _aabb_center(ctx.part_element_world_aabb(filter_door, elem="door_panel"))
    door_upper = door_joint.motion_limits.upper if door_joint.motion_limits is not None else None
    if door_upper is not None:
        with ctx.pose({door_joint: door_upper}):
            door_open_center = _aabb_center(ctx.part_element_world_aabb(filter_door, elem="door_panel"))
        ctx.check(
            "rear filter door swings outward on its side hinge",
            door_closed_center is not None
            and door_open_center is not None
            and door_open_center[0] > door_closed_center[0] + 0.05,
            details=f"closed={door_closed_center}, open={door_open_center}",
        )

    ctx.expect_gap(
        object_model.get_part("humidistat_dial"),
        control_panel,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="panel_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="humidistat dial seats on the control panel",
    )

    rest_button_positions = {
        part.name: ctx.part_world_position(part)
        for part in button_parts
    }
    for index, (button_part, button_joint) in enumerate(zip(button_parts, button_joints)):
        upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None
        if upper is None:
            continue
        with ctx.pose({button_joint: upper}):
            pressed_pos = ctx.part_world_position(button_part)
            ctx.check(
                f"button_{index} presses downward",
                rest_button_positions[button_part.name] is not None
                and pressed_pos is not None
                and pressed_pos[2] < rest_button_positions[button_part.name][2] - 0.0007,
                details=f"rest={rest_button_positions[button_part.name]}, pressed={pressed_pos}",
            )
            ctx.expect_gap(
                button_part,
                control_panel,
                axis="z",
                positive_elem="button_cap",
                negative_elem="panel_shell",
                max_gap=BUTTON_TRAVEL + 0.0005,
                max_penetration=BUTTON_TRAVEL + 0.0003,
                name=f"button_{index} stays above the control panel",
            )
            for other_part in button_parts:
                if other_part.name == button_part.name:
                    continue
                other_pressed_pos = ctx.part_world_position(other_part)
                other_rest_pos = rest_button_positions[other_part.name]
                ctx.check(
                    f"{button_part.name} does not move {other_part.name}",
                    other_pressed_pos is not None
                    and other_rest_pos is not None
                    and abs(other_pressed_pos[2] - other_rest_pos[2]) <= 1e-6,
                    details=f"{other_part.name}: rest={other_rest_pos}, posed={other_pressed_pos}",
                )

    return ctx.report()


object_model = build_object_model()
