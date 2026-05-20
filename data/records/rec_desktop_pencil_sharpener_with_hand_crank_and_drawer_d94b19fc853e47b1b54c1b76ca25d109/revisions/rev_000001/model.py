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
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="schoolhouse_pencil_sharpener")

    enamel = model.material("enamel_red", rgba=(0.66, 0.16, 0.14, 1.0))
    drawer_paint = model.material("drawer_red", rgba=(0.61, 0.15, 0.13, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    knob = model.material("knob_black", rgba=(0.14, 0.14, 0.15, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.65, 0.33, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.088, 0.096, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=graphite,
        name="base",
    )
    body.visual(
        Box((0.009, 0.096, 0.104)),
        origin=Origin(xyz=(-0.0395, 0.000, 0.062)),
        material=enamel,
        name="left_wall",
    )
    body.visual(
        Box((0.009, 0.096, 0.104)),
        origin=Origin(xyz=(0.0395, 0.000, 0.062)),
        material=enamel,
        name="right_wall",
    )
    body.visual(
        Box((0.070, 0.009, 0.104)),
        origin=Origin(xyz=(0.000, 0.0435, 0.062)),
        material=enamel,
        name="rear_wall",
    )
    body.visual(
        Box((0.070, 0.072, 0.006)),
        origin=Origin(xyz=(0.000, 0.004, 0.056)),
        material=graphite,
        name="separator",
    )
    body.visual(
        Box((0.070, 0.074, 0.052)),
        origin=Origin(xyz=(0.000, 0.004, 0.085)),
        material=enamel,
        name="upper_case",
    )
    body.visual(
        Box((0.088, 0.012, 0.048)),
        origin=Origin(xyz=(0.000, -0.042, 0.086)),
        material=enamel,
        name="front_case",
    )
    body.visual(
        Box((0.050, 0.090, 0.012)),
        origin=Origin(xyz=(-0.020, 0.002, 0.126), rpy=(0.0, 0.62, 0.0)),
        material=enamel,
        name="roof_left",
    )
    body.visual(
        Box((0.050, 0.090, 0.012)),
        origin=Origin(xyz=(0.020, 0.002, 0.126), rpy=(0.0, -0.62, 0.0)),
        material=enamel,
        name="roof_right",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.000, -0.047, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="port_bezel",
    )
    body.visual(
        Box((0.040, 0.003, 0.020)),
        origin=Origin(xyz=(0.000, -0.0485, 0.119)),
        material=brass,
        name="badge",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.043, 0.005, 0.082), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft_boss",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.064, 0.070, 0.034)),
        origin=Origin(xyz=(0.000, 0.035, 0.017)),
        material=drawer_paint,
        name="bin",
    )
    drawer.visual(
        Box((0.068, 0.006, 0.040)),
        origin=Origin(xyz=(0.000, -0.003, 0.020)),
        material=drawer_paint,
        name="face",
    )
    drawer.visual(
        Box((0.024, 0.004, 0.010)),
        origin=Origin(xyz=(0.000, -0.008, 0.020)),
        material=black,
        name="pull",
    )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=0.0025, length=0.030),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.032, 0.003, 0.024)),
        origin=Origin(xyz=(0.000, -0.0015, -0.014)),
        material=black,
        name="panel",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.008, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft",
    )
    crank.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.003, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    _add_member(
        crank,
        (0.006, 0.000, 0.000),
        (0.046, 0.000, -0.032),
        radius=0.004,
        material=steel,
        name="arm",
    )
    crank.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(0.056, 0.000, -0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob,
        name="grip",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.000, -0.042, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.20, lower=0.0, upper=0.040),
    )
    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.000, -0.0505, 0.103)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(0.049, 0.005, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    flap = object_model.get_part("flap")
    crank = object_model.get_part("crank")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    flap_joint = object_model.get_articulation("body_to_flap")
    crank_joint = object_model.get_articulation("body_to_crank")

    drawer_limits = drawer_joint.motion_limits
    flap_limits = flap_joint.motion_limits

    if drawer_limits is not None and drawer_limits.lower is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_joint: drawer_limits.lower}):
            ctx.expect_contact(
                drawer,
                body,
                elem_a="bin",
                elem_b="base",
                name="drawer bin is supported on the body base at rest",
            )
            closed_pos = ctx.part_world_position(drawer)

        with ctx.pose({drawer_joint: drawer_limits.upper}):
            ctx.expect_contact(
                drawer,
                body,
                elem_a="bin",
                elem_b="base",
                name="drawer stays supported while extended",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="bin",
                min_overlap=0.030,
                name="drawer retains insertion inside the housing",
            )
            extended_pos = ctx.part_world_position(drawer)

        ctx.check(
            "drawer slides forward from the front opening",
            closed_pos is not None
            and extended_pos is not None
            and extended_pos[1] < closed_pos[1] - 0.020,
            details=f"closed={closed_pos}, extended={extended_pos}",
        )

    if flap_limits is not None and flap_limits.lower is not None and flap_limits.upper is not None:
        with ctx.pose({flap_joint: flap_limits.lower}):
            ctx.expect_gap(
                body,
                flap,
                axis="y",
                positive_elem="front_case",
                negative_elem="panel",
                min_gap=0.001,
                max_gap=0.004,
                name="dust flap sits close over the entry port when closed",
            )
            flap_closed = ctx.part_element_world_aabb(flap, elem="panel")

        with ctx.pose({flap_joint: flap_limits.upper}):
            flap_open = ctx.part_element_world_aabb(flap, elem="panel")

        flap_closed_center = _aabb_center(flap_closed)
        flap_open_center = _aabb_center(flap_open)
        ctx.check(
            "dust flap lifts upward and outward",
            flap_closed_center is not None
            and flap_open_center is not None
            and flap_open_center[2] > flap_closed_center[2] + 0.010
            and flap_open_center[1] < flap_closed_center[1] - 0.008,
            details=f"closed={flap_closed_center}, open={flap_open_center}",
        )

    ctx.expect_contact(
        crank,
        body,
        elem_a="hub",
        elem_b="shaft_boss",
        name="crank hub seats against the side shaft support",
    )
    grip_rest = ctx.part_element_world_aabb(crank, elem="grip")
    with ctx.pose({crank_joint: math.pi / 2.0}):
        ctx.expect_contact(
            crank,
            body,
            elem_a="hub",
            elem_b="shaft_boss",
            name="crank stays supported while turning",
        )
        grip_turned = ctx.part_element_world_aabb(crank, elem="grip")

    grip_rest_center = _aabb_center(grip_rest)
    grip_turned_center = _aabb_center(grip_turned)
    ctx.check(
        "crank grip traces a circular turn about the side shaft",
        grip_rest_center is not None
        and grip_turned_center is not None
        and abs(grip_rest_center[2] - grip_turned_center[2]) > 0.015
        and abs(grip_rest_center[1] - grip_turned_center[1]) > 0.015,
        details=f"rest={grip_rest_center}, turned={grip_turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
