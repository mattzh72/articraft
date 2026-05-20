from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOUSING_LENGTH = 0.280
HOUSING_DEPTH = 0.160
HOUSING_HEIGHT = 0.045
TRACKBALL_RADIUS = 0.040
TRACKBALL_CENTER = (-0.030, 0.000, 0.052)
SOCKET_CLEARANCE_RADIUS = 0.045
SOCKET_LIP_OUTER_RADIUS = 0.054
SOCKET_LIP_INNER_RADIUS = 0.043
SCROLL_WHEEL_CENTER = (HOUSING_LENGTH / 2.0 + 0.010, -0.045, 0.026)


def _build_housing_shell() -> cq.Workplane:
    """Rounded desktop-controller shell with a real spherical trackball socket."""
    shell = (
        cq.Workplane("XY")
        .box(HOUSING_LENGTH, HOUSING_DEPTH, HOUSING_HEIGHT)
        .translate((0.0, 0.0, HOUSING_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.006)
    )
    socket_cutter = (
        cq.Workplane("XY")
        .sphere(SOCKET_CLEARANCE_RADIUS)
        .translate(TRACKBALL_CENTER)
    )
    return shell.cut(socket_cutter)


def _build_socket_lip() -> cq.Workplane:
    """A dark annular bezel that makes the circular top socket explicit."""
    outer = cq.Workplane("XY").cylinder(0.006, SOCKET_LIP_OUTER_RADIUS)
    inner = cq.Workplane("XY").cylinder(0.008, SOCKET_LIP_INNER_RADIUS)
    return outer.cut(inner).translate(
        (TRACKBALL_CENTER[0], TRACKBALL_CENTER[1], HOUSING_HEIGHT + 0.0025)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trackball_desktop_controller")

    body_mat = model.material("charcoal_plastic", rgba=(0.075, 0.080, 0.088, 1.0))
    socket_mat = model.material("matte_black_socket", rgba=(0.010, 0.011, 0.012, 1.0))
    ball_mat = model.material("deep_red_trackball", rgba=(0.55, 0.035, 0.050, 1.0))
    ball_mark_mat = model.material("trackball_highlight", rgba=(0.95, 0.48, 0.42, 1.0))
    wheel_mat = model.material("ribbed_rubber", rgba=(0.030, 0.031, 0.034, 1.0))
    axle_mat = model.material("brushed_metal", rgba=(0.58, 0.59, 0.56, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shell(), "housing_shell", tolerance=0.0007),
        material=body_mat,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_build_socket_lip(), "socket_lip", tolerance=0.0005),
        material=socket_mat,
        name="socket_lip",
    )
    housing.visual(
        Box((0.003, 0.052, 0.046)),
        origin=Origin(xyz=(HOUSING_LENGTH / 2.0 + 0.0005, -0.045, 0.026)),
        material=socket_mat,
        name="side_bearing_plate",
    )
    housing.visual(
        Box((0.235, 0.006, 0.006)),
        origin=Origin(xyz=(-0.003, -HOUSING_DEPTH / 2.0 - 0.001, 0.018)),
        material=socket_mat,
        name="front_shadow_seam",
    )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_LENGTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        mass=0.78,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_HEIGHT / 2.0)),
    )

    trackball = model.part("trackball")
    trackball.visual(
        Sphere(TRACKBALL_RADIUS),
        origin=Origin(),
        material=ball_mat,
        name="ball",
    )
    # A small raised dot makes rotation legible while remaining part of the ball.
    trackball.visual(
        Sphere(0.006),
        origin=Origin(xyz=(0.010, -0.022, 0.032)),
        material=ball_mark_mat,
        name="surface_dot",
    )
    trackball.inertial = Inertial.from_geometry(
        Sphere(TRACKBALL_RADIUS),
        mass=0.115,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_trackball",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=trackball,
        origin=Origin(xyz=TRACKBALL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=18.0),
    )

    scroll_wheel = model.part("scroll_wheel")
    scroll_wheel.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_mat,
        name="wheel_disk",
    )
    scroll_wheel.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_mat,
        name="center_axle",
    )
    for index in range(14):
        angle = 2.0 * math.pi * index / 14.0
        scroll_wheel.visual(
            Box((0.015, 0.0032, 0.0018)),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0178 * math.cos(angle),
                    0.0178 * math.sin(angle),
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            material=wheel_mat,
            name=f"tread_{index}",
        )
    scroll_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.016),
        mass=0.026,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_scroll_wheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=scroll_wheel,
        origin=Origin(xyz=SCROLL_WHEEL_CENTER),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.04, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    trackball = object_model.get_part("trackball")
    scroll_wheel = object_model.get_part("scroll_wheel")
    trackball_joint = object_model.get_articulation("housing_to_trackball")
    wheel_joint = object_model.get_articulation("housing_to_scroll_wheel")

    ctx.expect_within(
        trackball,
        housing,
        axes="xy",
        inner_elem="ball",
        outer_elem="socket_lip",
        margin=0.001,
        name="trackball sits inside circular socket lip",
    )
    ctx.expect_overlap(
        trackball,
        housing,
        axes="z",
        elem_a="ball",
        elem_b="housing_shell",
        min_overlap=0.025,
        name="trackball is visibly seated into the housing",
    )
    ctx.expect_gap(
        scroll_wheel,
        housing,
        axis="x",
        positive_elem="wheel_disk",
        negative_elem="side_bearing_plate",
        min_gap=0.0005,
        max_gap=0.004,
        name="scroll wheel sits just outside side bearing plate",
    )

    ctx.check(
        "trackball continuous axis is horizontal",
        trackball_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(trackball_joint.axis[2]) < 1e-9,
        details=f"axis={trackball_joint.axis}, type={trackball_joint.articulation_type}",
    )
    ctx.check(
        "scroll wheel continuous axis is horizontal",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(wheel_joint.axis[2]) < 1e-9,
        details=f"axis={wheel_joint.axis}, type={wheel_joint.articulation_type}",
    )

    rest_ball = ctx.part_world_position(trackball)
    rest_wheel = ctx.part_world_position(scroll_wheel)
    with ctx.pose({trackball_joint: math.pi / 2.0, wheel_joint: math.pi}):
        moved_ball = ctx.part_world_position(trackball)
        moved_wheel = ctx.part_world_position(scroll_wheel)
        ctx.check(
            "continuous rotations keep centers captured",
            rest_ball == moved_ball and rest_wheel == moved_wheel,
            details=f"rest_ball={rest_ball}, moved_ball={moved_ball}, rest_wheel={rest_wheel}, moved_wheel={moved_wheel}",
        )

    return ctx.report()


object_model = build_object_model()
