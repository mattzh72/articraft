from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="waffle_maker")

    body_metal = model.material("body_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    body_shadow = model.material("body_shadow", rgba=(0.34, 0.35, 0.37, 1.0))
    plate_dark = model.material("plate_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    handle_black = model.material("handle_black", rgba=(0.09, 0.09, 0.10, 1.0))
    foot_black = model.material("foot_black", rgba=(0.06, 0.06, 0.06, 1.0))
    dial_face = model.material("dial_face", rgba=(0.20, 0.20, 0.21, 1.0))
    dial_mark = model.material("dial_mark", rgba=(0.82, 0.20, 0.16, 1.0))

    base_w = 0.290
    base_d = 0.270
    foot_h = 0.008
    body_h = 0.055
    plate_w = 0.248
    plate_d = 0.228
    plate_t = 0.012
    grid_w = 0.018
    grid_t = 0.006
    hinge_y = -0.133
    hinge_z = 0.074

    body_shell_shape = cq.Workplane("XY").box(base_w, base_d, body_h).edges().fillet(0.012)

    lower_body = model.part("lower_body")
    lower_body.visual(
        mesh_from_cadquery(body_shell_shape, "waffle_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, foot_h + body_h / 2.0)),
        material=body_metal,
        name="body_shell",
    )
    lower_body.visual(
        Box((plate_w, plate_d, plate_t)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=plate_dark,
        name="base_plate",
    )
    lower_body.visual(
        Box((plate_w * 0.94, grid_w, grid_t)),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=plate_dark,
        name="base_grid_cross_x",
    )
    lower_body.visual(
        Box((grid_w, plate_d * 0.94, grid_t)),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=plate_dark,
        name="base_grid_cross_y",
    )

    foot_x = 0.108
    foot_y = 0.094
    foot_r = 0.011
    for idx, (x_pos, y_pos) in enumerate(
        (
            (-foot_x, -foot_y),
            (foot_x, -foot_y),
            (-foot_x, foot_y),
            (foot_x, foot_y),
        )
    ):
        lower_body.visual(
            Cylinder(radius=foot_r, length=foot_h),
            origin=Origin(xyz=(x_pos, y_pos, foot_h / 2.0)),
            material=foot_black,
            name=f"foot_{idx}",
        )

    lower_body.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(base_w / 2.0 + 0.003, 0.010, 0.040), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_shadow,
        name="dial_boss",
    )

    hinge_support_x = 0.086
    for idx, x_pos in enumerate((-hinge_support_x, hinge_support_x)):
        lower_body.visual(
            Box((0.030, 0.018, 0.020)),
            origin=Origin(xyz=(x_pos, hinge_y + 0.008, 0.064)),
            material=body_shadow,
            name=f"hinge_support_{idx}",
        )
        lower_body.visual(
            Cylinder(radius=0.010, length=0.063),
            origin=Origin(xyz=(x_pos, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=body_shadow,
            name=f"hinge_barrel_{idx}",
        )

    lid_w = 0.286
    lid_d = 0.251
    lid_shell_t = 0.050
    lid_shell_shape = cq.Workplane("XY").box(lid_w, lid_d, lid_shell_t).edges().fillet(0.014)

    top_shell = model.part("top_shell")
    top_shell.visual(
        mesh_from_cadquery(lid_shell_shape, "waffle_lid_shell"),
        origin=Origin(xyz=(0.0, 0.010 + lid_d / 2.0, 0.028)),
        material=body_metal,
        name="lid_shell",
    )
    top_shell.visual(
        Box((0.220, 0.178, 0.014)),
        origin=Origin(xyz=(0.0, 0.142, 0.056)),
        material=body_shadow,
        name="lid_crown",
    )
    top_shell.visual(
        Cylinder(radius=0.009, length=0.106),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_shadow,
        name="lid_hinge_barrel",
    )
    top_shell.visual(
        Box((0.160, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, 0.020, 0.010)),
        material=body_shadow,
        name="hinge_web",
    )
    top_shell.visual(
        Box((plate_w - 0.002, plate_d - 0.002, plate_t)),
        origin=Origin(xyz=(0.0, 0.137, -0.002)),
        material=plate_dark,
        name="lid_plate",
    )
    top_shell.visual(
        Box((0.018, 0.028, 0.024)),
        origin=Origin(xyz=(-0.040, 0.270, 0.005)),
        material=handle_black,
        name="handle_arm_0",
    )
    top_shell.visual(
        Box((0.018, 0.028, 0.024)),
        origin=Origin(xyz=(0.040, 0.270, 0.005)),
        material=handle_black,
        name="handle_arm_1",
    )
    top_shell.visual(
        Cylinder(radius=0.010, length=0.112),
        origin=Origin(xyz=(0.0, 0.284, 0.008), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_black,
        name="handle_grip",
    )

    thermostat_dial = model.part("thermostat_dial")
    thermostat_dial.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=body_shadow,
        name="dial_shaft",
    )
    thermostat_dial.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dial_face,
        name="dial_knob",
    )
    thermostat_dial.visual(
        Box((0.004, 0.003, 0.012)),
        origin=Origin(xyz=(0.024, 0.0, 0.013)),
        material=dial_mark,
        name="dial_indicator",
    )

    model.articulation(
        "body_to_top_shell",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=top_shell,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_thermostat_dial",
        ArticulationType.CONTINUOUS,
        parent=lower_body,
        child=thermostat_dial,
        origin=Origin(xyz=(base_w / 2.0 + 0.006, 0.010, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("lower_body")
    lid = object_model.get_part("top_shell")
    dial = object_model.get_part("thermostat_dial")
    lid_hinge = object_model.get_articulation("body_to_top_shell")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_plate",
            negative_elem="base_plate",
            min_gap=0.0005,
            max_gap=0.004,
            name="closed waffle plates keep a narrow cooking gap",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_plate",
            elem_b="base_plate",
            min_overlap=0.20,
            name="closed lid covers the lower cooking plate",
        )

    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem="handle_grip",
                negative_elem="body_shell",
                min_gap=0.08,
                name="open lid lifts the front handle well above the body",
            )

    ctx.expect_origin_gap(
        dial,
        body,
        axis="x",
        min_gap=0.14,
        max_gap=0.17,
        name="thermostat dial sits out on the side wall",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="yz",
        elem_a="dial_knob",
        elem_b="body_shell",
        min_overlap=0.03,
        name="thermostat dial aligns with the housing side panel",
    )

    return ctx.report()


object_model = build_object_model()
