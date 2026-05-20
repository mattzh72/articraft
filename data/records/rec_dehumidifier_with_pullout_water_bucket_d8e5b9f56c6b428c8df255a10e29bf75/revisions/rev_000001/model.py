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

BODY_W = 0.365
BODY_D = 0.245
BODY_H = 0.615

BUCKET_W = 0.290
BUCKET_D = 0.160
BUCKET_H = 0.205
BUCKET_WALL = 0.0035
BUCKET_BOTTOM = 0.0050
BUCKET_FRONT_Y = BODY_D * 0.5 - 0.007
BUCKET_BOTTOM_Z = 0.052
BUCKET_TRAVEL = 0.130

BUCKET_CAVITY_W = 0.310
BUCKET_CAVITY_D = 0.176
BUCKET_CAVITY_H = 0.226
BUCKET_CAVITY_Z = 0.044

WINDOW_W = 0.034
WINDOW_H = 0.112
WINDOW_T = 0.0024
WINDOW_X = 0.096
WINDOW_Z = 0.052


def _body_shape() -> cq.Workplane:
    side_t = 0.028
    back_t = 0.050
    top_t = 0.074
    front_t = 0.026
    opening_w = 0.314
    opening_h = 0.226
    opening_z = 0.044
    pillar_w = (BODY_W - opening_w) * 0.5

    top_cover = cq.Workplane("XY").box(BODY_W, BODY_D, top_t).translate(
        (0.0, 0.0, BODY_H - top_t * 0.5)
    )
    back_wall = cq.Workplane("XY").box(BODY_W, back_t, BODY_H - top_t).translate(
        (0.0, -BODY_D * 0.5 + back_t * 0.5, (BODY_H - top_t) * 0.5)
    )
    side_wall = cq.Workplane("XY").box(side_t, BODY_D - back_t, BODY_H - top_t).translate(
        (-BODY_W * 0.5 + side_t * 0.5, 0.025, (BODY_H - top_t) * 0.5)
    )
    left_wall = side_wall
    right_wall = side_wall.mirror("YZ")
    front_upper = cq.Workplane("XY").box(BODY_W - 2.0 * side_t, front_t, BODY_H - (opening_z + opening_h)).translate(
        (
            0.0,
            BODY_D * 0.5 - front_t * 0.5,
            opening_z + opening_h + (BODY_H - (opening_z + opening_h)) * 0.5,
        )
    )
    front_pillar = cq.Workplane("XY").box(pillar_w, front_t, opening_h + 0.010).translate(
        (
            -(opening_w * 0.5 + pillar_w * 0.5),
            BODY_D * 0.5 - front_t * 0.5,
            opening_z + (opening_h + 0.010) * 0.5,
        )
    )
    lower_floor = cq.Workplane("XY").box(BODY_W - 2.0 * side_t, 0.112, 0.020).translate((0.0, -0.018, 0.010))
    front_sill = cq.Workplane("XY").box(BODY_W - 0.040, front_t, 0.030).translate(
        (0.0, BODY_D * 0.5 - front_t * 0.5, 0.015)
    )

    body = (
        top_cover.union(back_wall)
        .union(left_wall)
        .union(right_wall)
        .union(front_upper)
        .union(front_pillar)
        .union(front_pillar.mirror("YZ"))
        .union(lower_floor)
        .union(front_sill)
    )

    for z_center in (0.320, 0.350, 0.380, 0.410, 0.440, 0.470):
        side_slot = cq.Workplane("XY").box(0.016, 0.116, 0.004).translate(
            (BODY_W * 0.5 - 0.008, 0.0, z_center)
        )
        body = body.cut(side_slot)
        body = body.cut(side_slot.mirror("YZ"))

    for y_center in (-0.046, -0.030, -0.014, 0.002):
        top_slot = cq.Workplane("XY").box(0.198, 0.006, 0.020).translate((0.0, y_center, BODY_H - 0.008))
        body = body.cut(top_slot)

    top_deck_cut = cq.Workplane("XY").box(0.300, 0.150, 0.014).translate((0.0, -0.010, BODY_H - 0.007))
    handle_pocket_cut = cq.Workplane("XY").box(0.238, 0.126, 0.020).translate((0.0, -0.040, BODY_H - 0.010))
    body = body.cut(top_deck_cut)
    body = body.cut(handle_pocket_cut)

    return body


def _bucket_shape() -> cq.Workplane:
    bucket = (
        cq.Workplane("XY")
        .box(BUCKET_W, BUCKET_D, BUCKET_H)
        .translate((0.0, -BUCKET_D * 0.5, BUCKET_H * 0.5))
        .edges("|Z")
        .fillet(0.016)
        .faces(">Z")
        .edges()
        .fillet(0.010)
    )

    inner = cq.Workplane("XY").box(
        BUCKET_W - 2.0 * BUCKET_WALL,
        BUCKET_D - 2.0 * BUCKET_WALL,
        BUCKET_H - BUCKET_BOTTOM,
    ).translate((0.0, -BUCKET_D * 0.5, (BUCKET_H + BUCKET_BOTTOM) * 0.5))
    bucket = bucket.cut(inner)

    front_grip = (
        cq.Workplane("XZ")
        .center(0.0, WINDOW_Z + 0.010)
        .rect(0.120, 0.030)
        .extrude(0.022)
        .translate((0.0, 0.011, 0.0))
    )
    bucket = bucket.cut(front_grip)

    window_cut = cq.Workplane("XY").box(WINDOW_W, 0.010, WINDOW_H).translate(
        (WINDOW_X, -0.002, WINDOW_Z + WINDOW_H * 0.5)
    )
    bucket = bucket.cut(window_cut)

    runner = cq.Workplane("XY").box(0.010, 0.118, 0.010).translate(
        (BUCKET_W * 0.5 - 0.003, -0.060, 0.022)
    )
    bucket = bucket.union(runner).union(runner.mirror("YZ"))

    return bucket


def _handle_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(0.014, 0.098, 0.010).translate((0.096, 0.055, 0.0))
    handle = rail.union(rail.mirror("YZ"))
    handle = handle.union(cq.Workplane("XY").box(0.206, 0.014, 0.010).translate((0.0, 0.104, 0.0)))
    handle = handle.union(
        cq.Workplane("YZ").circle(0.0065).extrude(0.020).translate((0.094, 0.0, 0.0)).translate((-0.010, 0.0, 0.0))
    )
    handle = handle.union(
        cq.Workplane("YZ").circle(0.0065).extrude(0.020).translate((-0.094, 0.0, 0.0)).translate((-0.010, 0.0, 0.0))
    )
    return handle.edges("|Y").fillet(0.004)


def _button_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.030, 0.018, 0.006)
        .translate((0.0, 0.0, 0.003))
        .edges("|Z")
        .fillet(0.003)
        .faces(">Z")
        .edges()
        .fillet(0.0018)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_dehumidifier")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.76, 0.79, 0.82, 1.0))
    control_dark = model.material("control_dark", rgba=(0.21, 0.23, 0.25, 1.0))
    control_black = model.material("control_black", rgba=(0.12, 0.13, 0.14, 1.0))
    tank_clear = model.material("tank_clear", rgba=(0.52, 0.76, 0.92, 0.35))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Box((0.286, 0.128, 0.004)),
        origin=Origin(xyz=(0.0, -0.010, BODY_H - 0.012)),
        material=trim_grey,
        name="top_interface",
    )
    body.visual(
        Box((0.228, 0.118, 0.004)),
        origin=Origin(xyz=(0.0, -0.040, BODY_H - 0.020)),
        material=trim_grey,
        name="handle_floor",
    )
    body.visual(
        Box((0.034, 0.120, 0.040)),
        origin=Origin(xyz=(-0.092, 0.004, 0.032)),
        material=trim_grey,
        name="guide_0",
    )
    body.visual(
        Box((0.034, 0.120, 0.040)),
        origin=Origin(xyz=(0.092, 0.004, 0.032)),
        material=trim_grey,
        name="guide_1",
    )

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_bucket_shape(), "bucket_shell"),
        material=trim_grey,
        name="bucket_shell",
    )
    bucket.visual(
        Box((WINDOW_W - 0.004, WINDOW_T, WINDOW_H - 0.016)),
        origin=Origin(
            xyz=(
                WINDOW_X,
                -WINDOW_T * 0.5,
                WINDOW_Z + (WINDOW_H - 0.016) * 0.5,
            )
        ),
        material=tank_clear,
        name="water_window",
    )

    model.articulation(
        "body_to_bucket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(0.0, BUCKET_FRONT_Y, BUCKET_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.22,
            lower=0.0,
            upper=BUCKET_TRAVEL,
        ),
    )

    handle_mesh = mesh_from_cadquery(_handle_shape(), "top_handle")
    top_handle = model.part("top_handle")
    top_handle.visual(handle_mesh, material=control_dark, name="handle_shell")
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_handle,
        origin=Origin(xyz=(0.0, -0.089, BODY_H - 0.013)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )

    wheel = model.part("selector_wheel")
    wheel.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.020,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.056, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=20, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006, angle_deg=0.0),
                center=False,
            ),
            "selector_wheel",
        ),
        material=control_black,
        name="wheel_shell",
    )
    model.articulation(
        "body_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(0.088, 0.014, BODY_H - 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=8.0,
        ),
    )

    button_mesh = mesh_from_cadquery(_button_shape(), "program_button")
    button_positions = [
        (-0.094, 0.028),
        (-0.056, 0.028),
        (-0.018, 0.028),
        (-0.094, -0.012),
        (-0.056, -0.012),
        (-0.018, -0.012),
    ]
    for index, (x_pos, y_pos) in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(button_mesh, material=control_dark, name="button_cap")
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, y_pos, BODY_H - 0.010)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.10,
                lower=-0.0025,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    bucket_slide = object_model.get_articulation("body_to_bucket")
    top_handle = object_model.get_part("top_handle")
    handle_joint = object_model.get_articulation("body_to_handle")
    wheel_joint = object_model.get_articulation("body_to_wheel")

    ctx.expect_within(
        bucket,
        body,
        axes="x",
        margin=0.004,
        name="bucket stays centered between body side guides",
    )
    ctx.expect_overlap(
        bucket,
        body,
        axes="z",
        min_overlap=0.18,
        name="bucket remains vertically nested in the lower bay",
    )

    rest_pos = ctx.part_world_position(bucket)
    with ctx.pose({bucket_slide: BUCKET_TRAVEL}):
        ctx.expect_within(
            bucket,
            body,
            axes="x",
            margin=0.004,
            name="extended bucket stays laterally aligned",
        )
        ctx.expect_overlap(
            bucket,
            body,
            axes="y",
            min_overlap=0.030,
            name="extended bucket retains insertion on the lower guides",
        )
        extended_pos = ctx.part_world_position(bucket)

    ctx.check(
        "bucket pulls forward along +Y",
        rest_pos is not None and extended_pos is not None and extended_pos[1] > rest_pos[1] + 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    handle_limits = handle_joint.motion_limits
    rest_handle_aabb = ctx.part_world_aabb(top_handle)
    opened_handle_aabb = None
    if handle_limits is not None and handle_limits.upper is not None:
        with ctx.pose({handle_joint: handle_limits.upper}):
            opened_handle_aabb = ctx.part_world_aabb(top_handle)
    ctx.check(
        "top handle folds upward",
        rest_handle_aabb is not None
        and opened_handle_aabb is not None
        and float(opened_handle_aabb[1][2]) > float(rest_handle_aabb[1][2]) + 0.060,
        details=f"rest={rest_handle_aabb}, opened={opened_handle_aabb}",
    )

    wheel_limits = wheel_joint.motion_limits
    ctx.check(
        "selector wheel is continuous",
        wheel_limits is not None and wheel_limits.lower is None and wheel_limits.upper is None,
        details=f"wheel_limits={wheel_limits}",
    )

    for index in range(6):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        button_limits = button_joint.motion_limits
        rest_button_pos = ctx.part_world_position(button)
        pressed_button_pos = None
        if button_limits is not None and button_limits.lower is not None:
            with ctx.pose({button_joint: button_limits.lower}):
                pressed_button_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index}_presses_down",
            rest_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[2] < rest_button_pos[2] - 0.0015,
            details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    return ctx.report()


object_model = build_object_model()
