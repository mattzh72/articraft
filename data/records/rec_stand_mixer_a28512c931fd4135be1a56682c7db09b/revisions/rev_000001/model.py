from __future__ import annotations

import math

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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _body_casting() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.04))
        .box(0.82, 0.64, 0.08)
        .edges("|Z")
        .fillet(0.025)
    )
    pedestal = (
        cq.Workplane("XY")
        .transformed(offset=(-0.15, 0.0, 0.36))
        .box(0.46, 0.44, 0.56)
        .edges("|Z")
        .fillet(0.028)
    )
    column = (
        cq.Workplane("XY")
        .transformed(offset=(-0.03, 0.0, 0.86))
        .box(0.24, 0.32, 0.52)
        .edges("|Z")
        .fillet(0.022)
    )
    neck = (
        cq.Workplane("XY")
        .transformed(offset=(0.03, 0.0, 1.00))
        .box(0.34, 0.32, 0.16)
        .edges("|Z")
        .fillet(0.020)
    )
    head = (
        cq.Workplane("XY")
        .transformed(offset=(0.16, 0.0, 1.18))
        .box(0.62, 0.34, 0.20)
        .edges("|Z")
        .fillet(0.022)
    )
    motor_hump = (
        cq.Workplane("XY")
        .transformed(offset=(0.02, 0.0, 1.33))
        .box(0.28, 0.24, 0.10)
        .edges("|Z")
        .fillet(0.018)
    )
    bowl_clearance_plinth = (
        cq.Workplane("XY")
        .transformed(offset=(0.16, 0.0, 0.14))
        .box(0.18, 0.36, 0.12)
        .edges("|Z")
        .fillet(0.015)
    )
    return (
        base.union(pedestal)
        .union(column)
        .union(neck)
        .union(head)
        .union(motor_hump)
        .union(bowl_clearance_plinth)
    )


def _carriage_frame() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.0))
        .box(0.08, 0.18, 0.38)
        .edges("|Z")
        .fillet(0.010)
    )


def _bowl_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.18))
        .cylinder(0.36, 0.235)
        .edges("<Z")
        .fillet(0.040)
        .faces(">Z")
        .shell(-0.005)
    )


def _handwheel_shape() -> cq.Workplane:
    rim = (
        cq.Workplane("XZ")
        .circle(0.150)
        .extrude(0.014, both=True)
        .cut(cq.Workplane("XZ").circle(0.118).extrude(0.018, both=True))
    )
    hub = cq.Workplane("XZ").circle(0.040).extrude(0.020, both=True)
    spoke_vertical = cq.Workplane("XZ").rect(0.026, 0.244).extrude(0.012, both=True)
    spoke_horizontal = cq.Workplane("XZ").rect(0.244, 0.026).extrude(0.012, both=True)
    return rim.union(hub).union(spoke_vertical).union(spoke_horizontal)


def _dough_hook_mesh():
    hook_path = [
        (0.000, 0.000, -0.090),
        (0.030, 0.010, -0.135),
        (0.075, 0.016, -0.220),
        (0.095, 0.004, -0.300),
        (0.070, -0.008, -0.340),
        (0.018, -0.004, -0.345),
        (-0.030, 0.004, -0.315),
        (-0.022, 0.010, -0.255),
        (0.006, 0.004, -0.220),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            hook_path,
            radius=0.018,
            samples_per_segment=20,
            radial_segments=20,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "dough_hook",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_stand_mixer")

    enamel = model.material("enamel", rgba=(0.92, 0.92, 0.90, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_casting(), "mixer_body"),
        material=enamel,
        name="body_casting",
    )
    body.visual(
        Box((0.108, 0.24, 0.48)),
        origin=Origin(xyz=(0.134, 0.0, 0.58)),
        material=enamel,
        name="guide_tower",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.160),
        origin=Origin(xyz=(0.00, 0.180, 0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="wheel_shaft_boss",
    )
    body.visual(
        Cylinder(radius=0.062, length=0.080),
        origin=Origin(xyz=(0.40, 0.0, 1.10)),
        material=dark_metal,
        name="drive_socket",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_frame(), "lift_carriage"),
        material=enamel,
        name="carriage_frame",
    )
    carriage.visual(
        Box((0.10, 0.18, 0.05)),
        origin=Origin(xyz=(0.05, 0.18, 0.0)),
        material=dark_metal,
        name="root_0",
    )
    carriage.visual(
        Box((0.10, 0.18, 0.05)),
        origin=Origin(xyz=(0.05, -0.18, 0.0)),
        material=dark_metal,
        name="root_1",
    )
    carriage.visual(
        Box((0.32, 0.06, 0.03)),
        origin=Origin(xyz=(0.22, 0.280, 0.0)),
        material=dark_metal,
        name="arm_0",
    )
    carriage.visual(
        Box((0.32, 0.06, 0.03)),
        origin=Origin(xyz=(0.22, -0.280, 0.0)),
        material=dark_metal,
        name="arm_1",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_bowl_shell(), "mixing_bowl"),
        material=stainless,
        name="bowl_shell",
    )
    bowl.visual(
        Box((0.08, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, 0.268, 0.22)),
        material=stainless,
        name="lug_0",
    )
    bowl.visual(
        Box((0.08, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, -0.268, 0.22)),
        material=stainless,
        name="lug_1",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_handwheel_shape(), "lift_handwheel"),
        material=dark_metal,
        name="wheel",
    )
    hook = model.part("hook")
    hook.visual(
        Cylinder(radius=0.022, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=stainless,
        name="shaft",
    )
    hook.visual(
        Cylinder(radius=0.032, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=stainless,
        name="hub",
    )
    hook.visual(
        _dough_hook_mesh(),
        material=stainless,
        name="hook_sweep",
    )

    lift = model.articulation(
        "bowl_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(0.228, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.10,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.35, 0.0, -0.18)),
    )
    wheel_joint = model.articulation(
        "lift_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=handwheel,
        origin=Origin(xyz=(0.00, 0.280, 0.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )
    hook_joint = model.articulation(
        "tool_drive",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=hook,
        origin=Origin(xyz=(0.40, 0.0, 1.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=16.0),
    )

    lift.meta["qc_samples"] = [0.0, 0.09, 0.18]
    wheel_joint.meta["qc_samples"] = [0.0, math.pi / 2.0]
    hook_joint.meta["qc_samples"] = [0.0, math.pi / 2.0]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bowl = object_model.get_part("bowl")
    carriage = object_model.get_part("carriage")
    handwheel = object_model.get_part("handwheel")
    hook = object_model.get_part("hook")

    lift = object_model.get_articulation("bowl_lift")
    wheel_joint = object_model.get_articulation("lift_wheel")
    hook_joint = object_model.get_articulation("tool_drive")

    ctx.expect_contact(
        bowl,
        carriage,
        elem_a="lug_0",
        elem_b="arm_0",
        name="upper bowl lug seats on the right support arm",
    )
    ctx.expect_contact(
        bowl,
        carriage,
        elem_a="lug_1",
        elem_b="arm_1",
        name="upper bowl lug seats on the left support arm",
    )

    with ctx.pose({lift: lift.motion_limits.upper}):
        ctx.expect_within(
            hook,
            bowl,
            axes="xy",
            inner_elem="hook_sweep",
            outer_elem="bowl_shell",
            margin=0.020,
            name="raised bowl stays centered under the dough hook",
        )
        ctx.expect_overlap(
            hook,
            bowl,
            axes="xy",
            elem_a="hook_sweep",
            elem_b="bowl_shell",
            min_overlap=0.060,
            name="dough hook projects over the raised bowl opening",
        )

    bowl_low = ctx.part_world_position(bowl)
    with ctx.pose({lift: lift.motion_limits.upper}):
        bowl_high = ctx.part_world_position(bowl)
    ctx.check(
        "bowl lift raises the bowl vertically",
        bowl_low is not None
        and bowl_high is not None
        and bowl_high[2] > bowl_low[2] + 0.15
        and abs(bowl_high[0] - bowl_low[0]) < 1e-6
        and abs(bowl_high[1] - bowl_low[1]) < 1e-6,
        details=f"low={bowl_low}, high={bowl_high}",
    )

    wheel_rest = ctx.part_world_position(handwheel)
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        wheel_quarter = ctx.part_world_position(handwheel)
    ctx.check(
        "handwheel spins in place on its side shaft",
        wheel_rest is not None
        and wheel_quarter is not None
        and max(abs(a - b) for a, b in zip(wheel_rest, wheel_quarter)) < 1e-6,
        details=f"rest={wheel_rest}, quarter_turn={wheel_quarter}",
    )

    hook_rest = ctx.part_world_position(hook)
    with ctx.pose({hook_joint: math.pi / 2.0}):
        hook_quarter = ctx.part_world_position(hook)
    ctx.check(
        "dough hook spins in place on the vertical drive axis",
        hook_rest is not None
        and hook_quarter is not None
        and max(abs(a - b) for a, b in zip(hook_rest, hook_quarter)) < 1e-6,
        details=f"rest={hook_rest}, quarter_turn={hook_quarter}",
    )

    ctx.expect_gap(
        handwheel,
        body,
        axis="y",
        positive_elem="wheel",
        negative_elem="wheel_shaft_boss",
        min_gap=0.0,
        max_gap=0.004,
        name="handwheel sits tightly against its shaft boss",
    )
    ctx.expect_origin_gap(
        bowl,
        body,
        axis="x",
        min_gap=0.40,
        name="bowl sits forward of the mixer column",
    )

    return ctx.report()


object_model = build_object_model()
