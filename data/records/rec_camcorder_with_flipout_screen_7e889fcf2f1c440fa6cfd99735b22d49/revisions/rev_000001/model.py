from __future__ import annotations

import math

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
import cadquery as cq


BODY_LEN = 0.300
BODY_W = 0.120
BODY_H = 0.140
BODY_SIDE_Y = BODY_W / 2.0
BODY_FRONT_X = BODY_LEN / 2.0

LENS_RING_X = 0.225
SCREEN_HINGE_X = 0.006
SCREEN_HINGE_Y = BODY_SIDE_Y + 0.006
SCREEN_OPEN_YAW = -math.radians(65.0)
SCREEN_CLOSE_Q = -SCREEN_OPEN_YAW
BUTTON_TRAVEL = 0.004


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        try:
            shape = shape.edges().fillet(radius)
        except Exception:
            # Keep the correctly scaled solid if an OCC fillet is too small or
            # has no stable edge set on the current primitive.
            pass
    return shape


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (x * c - y * s, x * s + y * c)


def _screen_origin(
    closed_xyz: tuple[float, float, float],
    *,
    yaw: float = SCREEN_OPEN_YAW,
) -> Origin:
    x, y = _rotate_xy(closed_xyz[0], closed_xyz[1], yaw)
    return Origin(xyz=(x, y, closed_xyz[2]), rpy=(0.0, 0.0, yaw))


def _lens_ring_geometry() -> cq.Workplane:
    """Hollow ribbed rubber sleeve, authored around local +X."""

    length = 0.056
    outer_r = 0.041
    inner_r = 0.0345
    tube = (
        cq.Workplane("YZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(length / 2.0, both=True)
    )

    rib_count = 28
    rib_radial = 0.0035
    rib_tangent = 0.0040
    rib_length = length * 0.92
    radius = outer_r + rib_radial * 0.40
    for index in range(rib_count):
        angle = 360.0 * index / rib_count
        theta = math.radians(angle)
        y = -radius * math.sin(theta)
        z = radius * math.cos(theta)
        rib = (
            cq.Workplane("XY")
            .box(rib_length, rib_tangent, rib_radial)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle)
            .translate((0.0, y, z))
        )
        tube = tube.union(rib)
    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prosumer_camcorder")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.058, 0.064, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.105, 0.112, 1.0))
    rubber = model.material("ribbed_rubber", rgba=(0.008, 0.008, 0.009, 1.0))
    lens_glass = model.material("coated_lens_glass", rgba=(0.02, 0.07, 0.095, 0.82))
    screen_glass = model.material("dark_screen_glass", rgba=(0.0, 0.012, 0.020, 1.0))
    button_blue = model.material("button_blue_gray", rgba=(0.08, 0.11, 0.135, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_box((BODY_LEN, BODY_W, BODY_H), 0.012), "main_shell"),
        material=dark_plastic,
        name="main_shell",
    )
    body.visual(
        mesh_from_cadquery(_rounded_box((0.074, 0.030, 0.116), 0.010), "handgrip"),
        origin=Origin(xyz=(0.072, BODY_SIDE_Y + 0.014, -0.006)),
        material=matte_black,
        name="handgrip",
    )
    body.visual(
        mesh_from_cadquery(_rounded_box((0.145, 0.026, 0.024), 0.006), "top_handle_bar"),
        origin=Origin(xyz=(-0.010, 0.0, BODY_H / 2.0 + 0.052)),
        material=matte_black,
        name="top_handle_bar",
    )
    for x in (-0.075, 0.050):
        body.visual(
            Box((0.020, 0.024, 0.070)),
            origin=Origin(xyz=(x, 0.0, BODY_H / 2.0 + 0.022)),
            material=matte_black,
            name=f"handle_post_{0 if x < 0 else 1}",
        )
    body.visual(
        mesh_from_cadquery(_rounded_box((0.065, 0.055, 0.035), 0.010), "viewfinder"),
        origin=Origin(xyz=(-BODY_LEN / 2.0 - 0.025, 0.0, 0.043)),
        material=matte_black,
        name="viewfinder",
    )
    body.visual(
        Box((0.118, 0.0025, 0.082)),
        origin=Origin(xyz=(-0.067, BODY_SIDE_Y + 0.0008, 0.006)),
        material=screen_glass,
        name="screen_bay",
    )
    body.visual(
        Box((0.128, 0.005, 0.094)),
        origin=Origin(xyz=(-0.067, BODY_SIDE_Y + 0.0013, 0.006)),
        material=charcoal,
        name="screen_recess_frame",
    )
    body.visual(
        Box((0.112, 0.006, 0.078)),
        origin=Origin(xyz=(-0.067, BODY_SIDE_Y + 0.0022, 0.006)),
        material=screen_glass,
        name="screen_opening",
    )

    # Deep, long front optics: static barrel pieces on the body plus a moving focus ring.
    body.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(BODY_FRONT_X + 0.014, 0.0, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="lens_mount",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.178),
        origin=Origin(xyz=(BODY_FRONT_X + 0.092, 0.0, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lens_barrel",
    )
    for name, x in (("rear_ring_shoulder", LENS_RING_X - 0.030), ("front_ring_shoulder", LENS_RING_X + 0.030)):
        body.visual(
            Cylinder(radius=0.042, length=0.004),
            origin=Origin(xyz=(x, 0.0, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name=name,
        )
    body.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(BODY_FRONT_X + 0.174, 0.0, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="front_hood",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(BODY_FRONT_X + 0.191, 0.0, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_glass",
    )

    # Exposed vertical hinge knuckles mounted on the side wall for the flip-out monitor.
    for z in (-0.041, 0.047):
        body.visual(
            Cylinder(radius=0.0048, length=0.025),
            origin=Origin(xyz=(SCREEN_HINGE_X, SCREEN_HINGE_Y, z)),
            material=matte_black,
            name=f"screen_hinge_knuckle_{0 if z < 0 else 1}",
        )
        body.visual(
            Box((0.016, 0.010, 0.026)),
            origin=Origin(xyz=(SCREEN_HINGE_X, BODY_SIDE_Y + 0.0015, z)),
            material=charcoal,
            name=f"screen_hinge_boss_{0 if z < 0 else 1}",
        )

    button_xs = (-0.108, -0.073, -0.038)
    for index, x in enumerate(button_xs):
        body.visual(
            Cylinder(radius=0.0075, length=0.0015),
            origin=Origin(
                xyz=(x, BODY_SIDE_Y - 0.0008, -0.056),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=matte_black,
            name=f"button_socket_{index}",
        )

    ring = model.part("lens_ring")
    ring.visual(
        mesh_from_cadquery(_lens_ring_geometry(), "focus_ring", tolerance=0.0008, angular_tolerance=0.08),
        material=rubber,
        name="focus_ring",
    )
    model.articulation(
        "body_to_lens_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=ring,
        origin=Origin(xyz=(LENS_RING_X, 0.0, 0.016)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    monitor = model.part("monitor")
    monitor.visual(
        Cylinder(radius=0.0040, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=matte_black,
        name="hinge_barrel",
    )
    monitor.visual(
        Box((0.012, 0.006, 0.076)),
        origin=_screen_origin((-0.006, 0.004, 0.004)),
        material=charcoal,
        name="hinge_leaf",
    )
    monitor.visual(
        mesh_from_cadquery(_rounded_box((0.116, 0.010, 0.078), 0.004), "monitor_panel"),
        origin=_screen_origin((-0.068, 0.009, 0.004)),
        material=charcoal,
        name="panel_shell",
    )
    monitor.visual(
        Box((0.094, 0.0020, 0.058)),
        origin=_screen_origin((-0.070, 0.0147, 0.004)),
        material=screen_glass,
        name="screen_glass",
    )
    model.articulation(
        "body_to_monitor",
        ArticulationType.REVOLUTE,
        parent=body,
        child=monitor,
        origin=Origin(xyz=(SCREEN_HINGE_X, SCREEN_HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.45, upper=SCREEN_CLOSE_Q),
    )

    for index, x in enumerate(button_xs):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.0058, length=0.0065),
            origin=Origin(xyz=(0.0, 0.00325, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=button_blue,
            name="cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, BODY_SIDE_Y, -0.056)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.12, lower=0.0, upper=BUTTON_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    ring = object_model.get_part("lens_ring")
    monitor = object_model.get_part("monitor")
    lens_joint = object_model.get_articulation("body_to_lens_ring")
    monitor_joint = object_model.get_articulation("body_to_monitor")

    ctx.check(
        "lens ring is continuous rotary control",
        lens_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={lens_joint.articulation_type}",
    )
    ctx.expect_within(
        body,
        ring,
        axes="yz",
        inner_elem="lens_barrel",
        outer_elem="focus_ring",
        margin=0.002,
        name="static barrel fits through focus ring bore",
    )
    ctx.expect_overlap(
        ring,
        body,
        axes="x",
        elem_a="focus_ring",
        elem_b="lens_barrel",
        min_overlap=0.045,
        name="focus ring wraps a meaningful barrel length",
    )

    open_aabb = ctx.part_element_world_aabb(monitor, elem="panel_shell")
    with ctx.pose({monitor_joint: SCREEN_CLOSE_Q}):
        ctx.expect_gap(
            monitor,
            body,
            axis="y",
            positive_elem="panel_shell",
            negative_elem="screen_bay",
            min_gap=0.002,
            max_gap=0.020,
            name="closed monitor sits outside side screen bay",
        )
        ctx.expect_overlap(
            monitor,
            body,
            axes="xz",
            elem_a="panel_shell",
            elem_b="screen_bay",
            min_overlap=0.050,
            name="closed monitor covers the side screen opening",
        )
        closed_aabb = ctx.part_element_world_aabb(monitor, elem="panel_shell")

    ctx.check(
        "monitor swings on vertical hinge",
        open_aabb is not None
        and closed_aabb is not None
        and ((open_aabb[0][1] + open_aabb[1][1]) * 0.5)
        > ((closed_aabb[0][1] + closed_aabb[1][1]) * 0.5) + 0.030,
        details=f"open_aabb={open_aabb}, closed_aabb={closed_aabb}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"body_to_button_{index}")
        rest = ctx.part_world_position(button)
        ctx.expect_gap(
            button,
            body,
            axis="y",
            positive_elem="cap",
            negative_elem=f"button_socket_{index}",
            min_gap=0.0,
            max_gap=0.002,
            name=f"button {index} cap is seated in body wall",
        )
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button {index} depresses inward independently",
            rest is not None and pressed is not None and pressed[1] < rest[1] - 0.003,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
