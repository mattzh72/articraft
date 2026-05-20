from __future__ import annotations

import math

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
)


PI = math.pi


def _cyl_x(part, name: str, center, radius: float, length: float, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, PI / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(part, name: str, center, radius: float, length: float, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(-PI / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_z(part, name: str, center, radius: float, length: float, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _cyl_xz_between(part, name: str, start, end, radius: float, material):
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.hypot(dx, dz)
    angle = math.atan2(dx, dz)
    center = (
        (start[0] + end[0]) / 2.0,
        (start[1] + end[1]) / 2.0,
        (start[2] + end[2]) / 2.0,
    )
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, angle, 0.0)),
        material=material,
        name=name,
    )


def _point_along(start, end, t: float):
    return (
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
        start[2] + (end[2] - start[2]) * t,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_vacuum")

    enamel = model.material("aged_burgundy_enamel", rgba=(0.46, 0.06, 0.045, 1.0))
    cream = model.material("warm_ivory_bakelite", rgba=(0.84, 0.74, 0.55, 1.0))
    dark = model.material("black_rubber", rgba=(0.018, 0.017, 0.015, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    chrome = model.material("polished_wand_tube", rgba=(0.82, 0.84, 0.82, 1.0))
    brass = model.material("aged_brass_bolts", rgba=(0.80, 0.56, 0.22, 1.0))

    # Root: a low, horizontal canister with a cylindrical tank on a bolted base.
    body = model.part("body")
    _cyl_x(body, "main_tank", (0.0, 0.0, 0.25), 0.18, 0.62, enamel)
    _cyl_x(body, "front_end_cap", (0.335, 0.0, 0.25), 0.185, 0.055, cream)
    _cyl_x(body, "rear_end_cap", (-0.335, 0.0, 0.25), 0.185, 0.055, cream)
    body.visual(
        Box((0.74, 0.42, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=dark,
        name="rubber_base",
    )
    body.visual(
        Box((0.67, 0.34, 0.035)),
        origin=Origin(xyz=(-0.015, 0.0, 0.098)),
        material=steel,
        name="steel_cradle",
    )

    # Retro banding and pragmatic service access on top.
    for i, x in enumerate((-0.18, 0.15)):
        body.visual(
            Box((0.026, 0.38, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.421)),
            material=steel,
            name=f"tank_band_{i}",
        )
    body.visual(
        Box((0.28, 0.15, 0.025)),
        origin=Origin(xyz=(-0.055, 0.0, 0.431)),
        material=steel,
        name="hatch_saddle",
    )
    body.visual(
        Box((0.036, 0.018, 0.028)),
        origin=Origin(xyz=(-0.205, -0.079, 0.448)),
        material=steel,
        name="hatch_hinge_ear_0",
    )
    body.visual(
        Box((0.036, 0.018, 0.028)),
        origin=Origin(xyz=(-0.205, 0.079, 0.448)),
        material=steel,
        name="hatch_hinge_ear_1",
    )

    # Rear exhaust grille and name plate are mounted into the rear end cap.
    for i, z in enumerate((0.205, 0.235, 0.265, 0.295)):
        body.visual(
            Box((0.009, 0.18, 0.008)),
            origin=Origin(xyz=(-0.365, 0.0, z)),
            material=dark,
            name=f"rear_vent_slat_{i}",
        )
    body.visual(
        Box((0.010, 0.115, 0.030)),
        origin=Origin(xyz=(-0.365, 0.0, 0.155)),
        material=brass,
        name="rear_badge",
    )

    # Wheel-and-runner construction cues; these remain fixed to the body.
    for i, y in enumerate((-0.235, 0.235)):
        _cyl_y(body, f"rear_wheel_{i}", (-0.24, y, 0.085), 0.066, 0.055, dark)
        _cyl_y(body, f"rear_hub_{i}", (-0.24, y, 0.085), 0.028, 0.062, steel)
        body.visual(
            Box((0.055, 0.035, 0.040)),
            origin=Origin(xyz=(-0.24, y * 0.88, 0.118)),
            material=steel,
            name=f"wheel_bracket_{i}",
        )
    _cyl_z(body, "front_caster_wheel", (0.265, 0.0, 0.038), 0.038, 0.040, dark)
    body.visual(
        Box((0.090, 0.070, 0.030)),
        origin=Origin(xyz=(0.265, 0.0, 0.072)),
        material=steel,
        name="caster_fork",
    )

    # Bolted front adapter: flange, socket, fasteners, and gussets.
    _cyl_x(body, "front_adapter_flange", (0.381, 0.0, 0.320), 0.116, 0.026, steel)
    body.visual(
        Cylinder(radius=0.078, length=0.072),
        origin=Origin(xyz=(0.425, 0.0, 0.320), rpy=(0.0, PI / 2.0, 0.0)),
        material=dark,
        name="front_socket",
    )
    body.visual(
        Cylinder(radius=0.086, length=0.012),
        origin=Origin(xyz=(0.462, 0.0, 0.320), rpy=(0.0, PI / 2.0, 0.0)),
        material=brass,
        name="inner_socket_lip",
    )
    for i in range(8):
        theta = 2.0 * PI * i / 8.0
        y = math.cos(theta) * 0.091
        z = 0.320 + math.sin(theta) * 0.091
        _cyl_x(body, f"front_bolt_{i}", (0.399, y, z), 0.0085, 0.012, brass)
    body.visual(
        Box((0.080, 0.024, 0.028)),
        origin=Origin(xyz=(0.368, 0.0, 0.426)),
        material=steel,
        name="adapter_top_gusset",
    )
    for i, y in enumerate((-0.105, 0.105)):
        body.visual(
            Box((0.074, 0.020, 0.045)),
            origin=Origin(xyz=(0.377, y, 0.335)),
            material=steel,
            name=f"adapter_side_gusset_{i}",
        )

    # Articulated service hatch with a visible hinge barrel and latch knob.
    hatch = model.part("service_hatch")
    hatch.visual(
        Box((0.245, 0.135, 0.016)),
        origin=Origin(xyz=(0.1225, 0.0, 0.008)),
        material=cream,
        name="hatch_panel",
    )
    hatch.visual(
        Box((0.155, 0.075, 0.006)),
        origin=Origin(xyz=(0.135, 0.0, 0.019)),
        material=steel,
        name="hatch_reinforcement",
    )
    hatch.visual(
        Cylinder(radius=0.011, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=steel,
        name="hatch_barrel",
    )
    _cyl_z(hatch, "hatch_latch_knob", (0.205, 0.0, 0.025), 0.014, 0.014, brass)

    model.articulation(
        "body_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(-0.205, 0.0, 0.446)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    # Upper wand segment: socket ball, tube, ribbed grip, and an elbow yoke.
    upper = model.part("upper_wand")
    upper.visual(
        Sphere(radius=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark,
        name="swivel_ball",
    )
    upper_start = (0.020, 0.0, -0.003)
    upper_end = (0.500, 0.0, -0.060)
    upper.visual(
        Cylinder(radius=0.024, length=0.4833704583449432),
        origin=Origin(xyz=(0.260, 0.0, -0.0315), rpy=(0.0, 1.6889895826964237, 0.0)),
        material=chrome,
        name="upper_tube",
    )
    _cyl_xz_between(
        upper,
        "socket_collar",
        _point_along(upper_start, upper_end, 0.00),
        _point_along(upper_start, upper_end, 0.15),
        0.037,
        brass,
    )
    _cyl_xz_between(
        upper,
        "upper_end_collar",
        _point_along(upper_start, upper_end, 0.70),
        _point_along(upper_start, upper_end, 0.86),
        0.035,
        brass,
    )
    _cyl_xz_between(
        upper,
        "ribbed_hand_grip",
        _point_along(upper_start, upper_end, 0.22),
        _point_along(upper_start, upper_end, 0.42),
        0.031,
        dark,
    )
    for i, y in enumerate((-0.038, 0.038)):
        upper.visual(
            Box((0.060, 0.028, 0.078)),
            origin=Origin(xyz=(upper_end[0] + 0.006, y, upper_end[2])),
            material=steel,
            name=f"elbow_yoke_side_{i}",
        )
    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper,
        origin=Origin(xyz=(0.455, 0.0, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.65, upper=0.65),
    )

    # Lower wand segment pivots at the elbow and carries a fork for the nozzle.
    lower = model.part("lower_wand")
    lower_start = (0.0, 0.0, 0.0)
    lower_end = (0.580, 0.0, -0.180)
    lower.visual(
        Cylinder(radius=0.023, length=0.5601615838273544),
        origin=Origin(xyz=(0.3125, 0.0, -0.097), rpy=(0.0, 1.8717410688229853, 0.0)),
        material=chrome,
        name="lower_tube",
    )
    lower.visual(
        Cylinder(radius=0.019, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=brass,
        name="elbow_pin",
    )
    _cyl_xz_between(
        lower,
        "lower_start_collar",
        _point_along(lower_start, lower_end, 0.13),
        _point_along(lower_start, lower_end, 0.25),
        0.034,
        brass,
    )
    lower.visual(
        Box((0.052, 0.038, 0.032)),
        origin=Origin(xyz=(0.035, 0.0, -0.010)),
        material=steel,
        name="elbow_lug",
    )
    _cyl_xz_between(
        lower,
        "nozzle_end_collar",
        _point_along(lower_start, lower_end, 0.76),
        _point_along(lower_start, lower_end, 0.92),
        0.034,
        brass,
    )
    for i, y in enumerate((-0.036, 0.036)):
        lower.visual(
            Box((0.058, 0.028, 0.074)),
            origin=Origin(xyz=(lower_end[0] + 0.006, y, lower_end[2])),
            material=steel,
            name=f"nozzle_yoke_side_{i}",
        )

    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=lower,
        origin=Origin(xyz=upper_end),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=1.5, lower=-0.40, upper=0.45),
    )

    # Floor nozzle: broad legacy metal head with pivot pin, neck, rubber sole, brush roller.
    nozzle = model.part("floor_nozzle")
    nozzle.visual(
        Box((0.360, 0.320, 0.070)),
        origin=Origin(xyz=(0.240, 0.0, -0.035)),
        material=cream,
        name="nozzle_shell",
    )
    nozzle.visual(
        Box((0.350, 0.335, 0.012)),
        origin=Origin(xyz=(0.255, 0.0, -0.074)),
        material=dark,
        name="rubber_sole",
    )
    nozzle.visual(
        Cylinder(radius=0.047, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=steel,
        name="nozzle_neck",
    )
    nozzle.visual(
        Cylinder(radius=0.018, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=brass,
        name="nozzle_pivot_pin",
    )
    nozzle.visual(
        Box((0.105, 0.038, 0.050)),
        origin=Origin(xyz=(0.052, 0.0, -0.035)),
        material=steel,
        name="neck_bridge",
    )
    for i, y in enumerate((-0.125, 0.125)):
        nozzle.visual(
            Box((0.130, 0.050, 0.030)),
            origin=Origin(xyz=(0.025, y, -0.020)),
            material=steel,
            name=f"cheek_bridge_{i}",
        )
    _cyl_y(nozzle, "front_brush_roller", (0.345, 0.0, -0.066), 0.016, 0.305, dark)
    for i, y in enumerate((-0.125, 0.125)):
        nozzle.visual(
            Box((0.060, 0.040, 0.018)),
            origin=Origin(xyz=(-0.030, y, -0.008)),
            material=steel,
            name=f"nozzle_pivot_cheek_{i}",
        )

    model.articulation(
        "lower_wand_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=nozzle,
        origin=Origin(xyz=lower_end),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    hatch = object_model.get_part("service_hatch")
    upper = object_model.get_part("upper_wand")
    lower = object_model.get_part("lower_wand")
    nozzle = object_model.get_part("floor_nozzle")

    hatch_joint = object_model.get_articulation("body_to_service_hatch")
    body_swivel = object_model.get_articulation("body_to_upper_wand")
    elbow = object_model.get_articulation("upper_to_lower_wand")
    nozzle_pivot = object_model.get_articulation("lower_wand_to_nozzle")

    # Captured/nested hardware intentionally shares volume at the bearing proxy.
    ctx.allow_overlap(
        body,
        hatch,
        elem_a="hatch_hinge_ear_0",
        elem_b="hatch_barrel",
        reason="The hatch barrel is captured in the service-hatch hinge ear.",
    )
    ctx.allow_overlap(
        body,
        hatch,
        elem_a="hatch_hinge_ear_1",
        elem_b="hatch_barrel",
        reason="The hatch barrel is captured in the service-hatch hinge ear.",
    )
    ctx.allow_overlap(
        body,
        upper,
        elem_a="front_socket",
        elem_b="swivel_ball",
        reason="The spherical wand swivel is seated inside the front socket cup.",
    )
    ctx.allow_overlap(
        body,
        upper,
        elem_a="inner_socket_lip",
        elem_b="swivel_ball",
        reason="The retaining lip is a solid proxy for a ring around the captured wand swivel.",
    )
    for i in (0, 1):
        ctx.allow_overlap(
            upper,
            lower,
            elem_a=f"elbow_yoke_side_{i}",
            elem_b="elbow_pin",
            reason="The elbow pin is captured through the forked wand yoke.",
        )
        ctx.allow_overlap(
            lower,
            nozzle,
            elem_a=f"nozzle_yoke_side_{i}",
            elem_b="nozzle_pivot_pin",
            reason="The nozzle pivot pin is captured through the lower-wand fork.",
        )
    ctx.allow_overlap(
        lower,
        nozzle,
        elem_a="lower_tube",
        elem_b="nozzle_pivot_pin",
        reason="The nozzle pivot pin passes through the reinforced end of the lower tube.",
    )
    ctx.allow_overlap(
        upper,
        lower,
        elem_a="upper_tube",
        elem_b="elbow_pin",
        reason="The elbow pivot pin passes through a lug at the end of the upper wand tube.",
    )
    ctx.allow_overlap(
        lower,
        nozzle,
        elem_a="lower_tube",
        elem_b="nozzle_neck",
        reason="The nozzle neck is a solid socket proxy around the reinforced lower wand end.",
    )
    for i in (0, 1):
        ctx.allow_overlap(
            lower,
            nozzle,
            elem_a=f"nozzle_yoke_side_{i}",
            elem_b="nozzle_neck",
            reason="The fork cheek partially wraps the nozzle neck around the pivot bearing.",
        )

    ctx.expect_within(
        upper,
        body,
        axes="yz",
        inner_elem="swivel_ball",
        outer_elem="front_socket",
        margin=0.0,
        name="wand swivel is centered in socket cup",
    )
    ctx.expect_overlap(
        upper,
        body,
        axes="x",
        elem_a="swivel_ball",
        elem_b="front_socket",
        min_overlap=0.015,
        name="wand swivel remains inserted in socket",
    )
    ctx.expect_within(
        upper,
        body,
        axes="yz",
        inner_elem="swivel_ball",
        outer_elem="inner_socket_lip",
        margin=0.0,
        name="retaining lip surrounds wand swivel",
    )
    for i in (0, 1):
        ctx.expect_overlap(
            lower,
            upper,
            axes="yz",
            elem_a="elbow_pin",
            elem_b=f"elbow_yoke_side_{i}",
            min_overlap=0.006,
            name=f"elbow pin crosses yoke side {i}",
        )
        ctx.expect_overlap(
            nozzle,
            lower,
            axes="yz",
            elem_a="nozzle_pivot_pin",
            elem_b=f"nozzle_yoke_side_{i}",
            min_overlap=0.006,
            name=f"nozzle pin crosses yoke side {i}",
        )
        ctx.expect_overlap(
            nozzle,
            lower,
            axes="xz",
            elem_a="nozzle_neck",
            elem_b=f"nozzle_yoke_side_{i}",
            min_overlap=0.006,
            name=f"nozzle neck is wrapped by yoke side {i}",
        )
    ctx.expect_overlap(
        nozzle,
        lower,
        axes="xz",
        elem_a="nozzle_pivot_pin",
        elem_b="lower_tube",
        min_overlap=0.006,
        name="nozzle pin crosses reinforced tube end",
    )
    ctx.expect_overlap(
        lower,
        upper,
        axes="xz",
        elem_a="elbow_pin",
        elem_b="upper_tube",
        min_overlap=0.006,
        name="elbow pin crosses upper tube lug",
    )
    ctx.expect_overlap(
        nozzle,
        lower,
        axes="xz",
        elem_a="nozzle_neck",
        elem_b="lower_tube",
        min_overlap=0.006,
        name="nozzle neck captures lower wand end",
    )

    with ctx.pose({hatch_joint: 0.0}):
        ctx.expect_gap(
            hatch,
            body,
            axis="z",
            positive_elem="hatch_panel",
            negative_elem="hatch_saddle",
            min_gap=0.0,
            max_gap=0.004,
            name="service hatch seats on saddle",
        )

    closed_hatch_aabb = ctx.part_world_aabb(hatch)
    with ctx.pose({hatch_joint: 0.9}):
        open_hatch_aabb = ctx.part_world_aabb(hatch)
    ctx.check(
        "service hatch opens upward",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][2] > closed_hatch_aabb[1][2] + 0.04,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    rest_nozzle_pos = ctx.part_world_position(nozzle)
    with ctx.pose({body_swivel: 0.45}):
        swept_nozzle_pos = ctx.part_world_position(nozzle)
    ctx.check(
        "body swivel steers wand sideways",
        rest_nozzle_pos is not None
        and swept_nozzle_pos is not None
        and abs(swept_nozzle_pos[1] - rest_nozzle_pos[1]) > 0.20,
        details=f"rest={rest_nozzle_pos}, swept={swept_nozzle_pos}",
    )

    rest_nozzle_aabb = ctx.part_world_aabb(nozzle)
    with ctx.pose({elbow: 0.35, nozzle_pivot: -0.20}):
        raised_nozzle_aabb = ctx.part_world_aabb(nozzle)
    ctx.check(
        "elbow articulation changes nozzle working angle",
        rest_nozzle_aabb is not None
        and raised_nozzle_aabb is not None
        and raised_nozzle_aabb[0][2] > rest_nozzle_aabb[0][2] + 0.04,
        details=f"rest={rest_nozzle_aabb}, raised={raised_nozzle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
