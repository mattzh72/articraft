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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _pedestal_base_mesh():
    pedestal = (
        cq.Workplane("XY")
        .box(0.205, 0.245, 0.016, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
    )
    top_pad = (
        cq.Workplane("XY")
        .box(0.120, 0.108, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, 0.016))
    )
    return pedestal.union(top_pad)


def _neck_sleeve_mesh():
    collar = cq.Workplane("XY").circle(0.031).extrude(0.012)
    outer_tube = (
        cq.Workplane("XY")
        .box(0.036, 0.056, 0.166, centered=(True, True, False))
        .translate((0.0, 0.0, 0.010))
        .edges("|Z")
        .fillet(0.010)
    )
    inner_cut = (
        cq.Workplane("XY")
        .box(0.024, 0.040, 0.190, centered=(True, True, False))
        .translate((0.0, 0.0, 0.004))
    )
    return collar.union(outer_tube.cut(inner_cut))


def _display_shell_mesh():
    housing = (
        cq.Workplane("XY")
        .box(0.040, 0.542, 0.336)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.029, 0.0, 0.040))
    )
    screen_pocket = cq.Workplane("XY").box(0.012, 0.486, 0.274).translate((0.043, 0.0, 0.040))
    rear_bulge = cq.Workplane("XY").box(0.020, 0.184, 0.118).translate((0.015, 0.0, 0.040))
    hinge_barrel = cq.Workplane("XZ").circle(0.0065).extrude(0.013, both=True)
    return housing.cut(screen_pocket).union(rear_bulge).union(hinge_barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="business_monitor")

    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.28, 1.0))
    trim = model.material("trim", rgba=(0.30, 0.31, 0.34, 1.0))
    satin = model.material("satin", rgba=(0.42, 0.43, 0.46, 1.0))
    screen = model.material("screen", rgba=(0.08, 0.11, 0.13, 0.96))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_pedestal_base_mesh(), "monitor_pedestal_base"),
        material=graphite,
        name="pedestal",
    )

    neck_sleeve = model.part("neck_sleeve")
    neck_sleeve.visual(
        mesh_from_cadquery(_neck_sleeve_mesh(), "monitor_neck_sleeve"),
        material=charcoal,
        name="sleeve_body",
    )
    neck_sleeve.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin,
        name="swivel_trim",
    )

    neck_mast = model.part("neck_mast")
    neck_mast.visual(
        Box((0.020, 0.034, 0.290)),
        origin=Origin(),
        material=trim,
        name="mast_bar",
    )
    neck_mast.visual(
        Box((0.030, 0.048, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin,
        name="guide_collar",
    )
    neck_mast.visual(
        Box((0.028, 0.040, 0.020)),
        origin=Origin(xyz=(0.002, 0.0, 0.129)),
        material=charcoal,
        name="head_block",
    )
    neck_mast.visual(
        Box((0.020, 0.008, 0.032)),
        origin=Origin(xyz=(0.010, -0.022, 0.148)),
        material=charcoal,
        name="ear_0",
    )
    neck_mast.visual(
        Box((0.020, 0.008, 0.032)),
        origin=Origin(xyz=(0.010, 0.022, 0.148)),
        material=charcoal,
        name="ear_1",
    )

    display = model.part("display")
    display.visual(
        mesh_from_cadquery(_display_shell_mesh(), "monitor_display_shell"),
        material=charcoal,
        name="housing_shell",
    )
    display.visual(
        Box((0.010, 0.482, 0.270)),
        origin=Origin(xyz=(0.041, 0.0, 0.040)),
        material=screen,
        name="screen_glass",
    )
    display.visual(
        Box((0.010, 0.020, 0.006)),
        origin=Origin(xyz=(0.024, 0.0, -0.131)),
        material=trim,
        name="joystick_socket",
    )
    display.visual(
        Box((0.004, 0.024, 0.020)),
        origin=Origin(xyz=(-0.004, 0.0, -0.001)),
        material=charcoal,
        name="hinge_tongue",
    )

    joystick_yoke = model.part("joystick_yoke")
    joystick_yoke.visual(
        Sphere(radius=0.0025),
        origin=Origin(),
        material=trim,
        name="pivot_ball",
    )

    joystick = model.part("joystick")
    joystick.visual(
        Cylinder(radius=0.0025, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=satin,
        name="joystick_stem",
    )
    joystick.visual(
        Sphere(radius=0.0042),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=trim,
        name="joystick_cap",
    )

    model.articulation(
        "base_to_neck",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=neck_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5),
    )
    model.articulation(
        "sleeve_to_mast",
        ArticulationType.PRISMATIC,
        parent=neck_sleeve,
        child=neck_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.18,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "mast_to_display",
        ArticulationType.REVOLUTE,
        parent=neck_mast,
        child=display,
        origin=Origin(xyz=(0.018, 0.0, 0.148)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-0.18,
            upper=0.30,
        ),
    )
    model.articulation(
        "display_to_joystick_yoke",
        ArticulationType.REVOLUTE,
        parent=display,
        child=joystick_yoke,
        origin=Origin(xyz=(0.024, 0.0, -0.131)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=4.0,
            lower=-0.24,
            upper=0.24,
        ),
    )
    model.articulation(
        "yoke_to_joystick",
        ArticulationType.REVOLUTE,
        parent=joystick_yoke,
        child=joystick,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=4.0,
            lower=-0.24,
            upper=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    neck_sleeve = object_model.get_part("neck_sleeve")
    neck_mast = object_model.get_part("neck_mast")
    display = object_model.get_part("display")
    joystick = object_model.get_part("joystick")

    swivel = object_model.get_articulation("base_to_neck")
    lift = object_model.get_articulation("sleeve_to_mast")
    tilt = object_model.get_articulation("mast_to_display")
    joystick_x = object_model.get_articulation("display_to_joystick_yoke")
    joystick_y = object_model.get_articulation("yoke_to_joystick")

    def _extent(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    def _aabb_center(aabb) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.expect_within(
        neck_mast,
        neck_sleeve,
        axes="xy",
        inner_elem="mast_bar",
        outer_elem="sleeve_body",
        margin=0.0,
        name="mast stays centered in the sleeve at rest",
    )
    ctx.expect_overlap(
        neck_mast,
        neck_sleeve,
        axes="z",
        elem_a="mast_bar",
        elem_b="sleeve_body",
        min_overlap=0.140,
        name="mast remains deeply inserted at rest",
    )

    rest_display_pos = ctx.part_world_position(display)
    rest_display_aabb = ctx.part_element_world_aabb(display, elem="housing_shell")

    lift_limits = lift.motion_limits
    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({lift: lift_limits.upper}):
            ctx.expect_within(
                neck_mast,
                neck_sleeve,
                axes="xy",
                inner_elem="mast_bar",
                outer_elem="sleeve_body",
                margin=0.0,
                name="mast stays centered when raised",
            )
            ctx.expect_overlap(
                neck_mast,
                neck_sleeve,
                axes="z",
                elem_a="mast_bar",
                elem_b="sleeve_body",
                min_overlap=0.060,
                name="mast retains insertion when raised",
            )
            raised_display_pos = ctx.part_world_position(display)
        ctx.check(
            "display rises with height adjustment",
            rest_display_pos is not None
            and raised_display_pos is not None
            and raised_display_pos[2] > rest_display_pos[2] + 0.07,
            details=f"rest={rest_display_pos}, raised={raised_display_pos}",
        )

    with ctx.pose({swivel: math.pi / 2.0}):
        swivel_aabb = ctx.part_world_aabb(display)
    ctx.check(
        "display swivels broadside over the base",
        _extent(swivel_aabb, 0) is not None
        and _extent(swivel_aabb, 1) is not None
        and _extent(swivel_aabb, 0) > 0.45
        and _extent(swivel_aabb, 1) < 0.09,
        details=f"swivel_aabb={swivel_aabb}",
    )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt: tilt_limits.lower}):
            forward_aabb = ctx.part_element_world_aabb(display, elem="housing_shell")
        with ctx.pose({tilt: tilt_limits.upper}):
            backward_aabb = ctx.part_element_world_aabb(display, elem="housing_shell")
        ctx.check(
            "display tilts forward at the lower stop",
            rest_display_aabb is not None
            and forward_aabb is not None
            and forward_aabb[1][0] > rest_display_aabb[1][0] + 0.008,
            details=f"rest={rest_display_aabb}, forward={forward_aabb}",
        )
        ctx.check(
            "display tilts backward at the upper stop",
            rest_display_aabb is not None
            and backward_aabb is not None
            and backward_aabb[0][0] < rest_display_aabb[0][0] - 0.020,
            details=f"rest={rest_display_aabb}, backward={backward_aabb}",
        )

    rest_cap = _aabb_center(ctx.part_element_world_aabb(joystick, elem="joystick_cap"))
    with ctx.pose({joystick_x: 0.18}):
        side_cap = _aabb_center(ctx.part_element_world_aabb(joystick, elem="joystick_cap"))
    with ctx.pose({joystick_y: 0.18}):
        fore_cap = _aabb_center(ctx.part_element_world_aabb(joystick, elem="joystick_cap"))

    ctx.check(
        "joystick can rock sideways",
        rest_cap is not None and side_cap is not None and abs(side_cap[1] - rest_cap[1]) > 0.003,
        details=f"rest={rest_cap}, side={side_cap}",
    )
    ctx.check(
        "joystick can rock fore and aft",
        rest_cap is not None and fore_cap is not None and abs(fore_cap[0] - rest_cap[0]) > 0.003,
        details=f"rest={rest_cap}, fore={fore_cap}",
    )

    return ctx.report()


object_model = build_object_model()
