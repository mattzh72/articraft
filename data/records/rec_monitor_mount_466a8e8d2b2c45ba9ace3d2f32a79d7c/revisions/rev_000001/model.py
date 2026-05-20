from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BLACK = Material("matte_black_powder_coat", rgba=(0.015, 0.015, 0.014, 1.0))
GRAPHITE = Material("dark_graphite_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
RUBBER = Material("compressed_black_rubber", rgba=(0.003, 0.003, 0.003, 1.0))
STEEL = Material("brushed_steel", rgba=(0.55, 0.56, 0.54, 1.0))
HOLE = Material("black_recesses", rgba=(0.0, 0.0, 0.0, 1.0))
WOOD = Material("desk_edge_wood", rgba=(0.58, 0.36, 0.18, 1.0))


def _cylinder_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """URDF cylinders are local-Z; rotate one onto the local Y axis."""

    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _cylinder_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """URDF cylinders are local-Z; rotate one onto the local X axis."""

    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_clamped_monitor_mount")

    clamp_base = model.part("clamp_base")

    # A short desk edge is included as context so the C-clamp reads as clamped,
    # not merely as a freestanding post.
    clamp_base.visual(
        Box((0.24, 0.30, 0.040)),
        origin=Origin(xyz=(-0.080, 0.0, -0.440)),
        material=WOOD,
        name="desk_edge",
    )
    clamp_base.visual(
        Box((0.200, 0.130, 0.012)),
        origin=Origin(xyz=(-0.065, 0.0, -0.406)),
        material=BLACK,
        name="top_jaw",
    )
    clamp_base.visual(
        Box((0.026, 0.130, 0.165)),
        origin=Origin(xyz=(-0.158, 0.0, -0.485)),
        material=BLACK,
        name="clamp_spine",
    )
    clamp_base.visual(
        Box((0.150, 0.110, 0.012)),
        origin=Origin(xyz=(-0.090, 0.0, -0.562)),
        material=BLACK,
        name="lower_jaw",
    )
    clamp_base.visual(
        Cylinder(radius=0.009, length=0.088),
        origin=Origin(xyz=(-0.050, 0.0, -0.514)),
        material=STEEL,
        name="clamp_screw",
    )
    knob_geom, knob_rot = _cylinder_y(radius=0.024, length=0.080)
    clamp_base.visual(
        knob_geom,
        origin=Origin(xyz=(-0.050, 0.0, -0.592), rpy=knob_rot.rpy),
        material=GRAPHITE,
        name="screw_knob",
    )
    clamp_base.visual(
        Cylinder(radius=0.025, length=0.008),
        origin=Origin(xyz=(-0.050, 0.0, -0.464)),
        material=RUBBER,
        name="lower_pad",
    )
    clamp_base.visual(
        Box((0.130, 0.095, 0.008)),
        origin=Origin(xyz=(-0.052, 0.0, -0.416)),
        material=RUBBER,
        name="upper_pad",
    )
    clamp_base.visual(
        Cylinder(radius=0.022, length=0.335),
        origin=Origin(xyz=(0.0, 0.0, -0.2295)),
        material=BLACK,
        name="mast",
    )
    clamp_base.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.397)),
        material=BLACK,
        name="base_collar",
    )
    clamp_base.visual(
        Cylinder(radius=0.030, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.053)),
        material=BLACK,
        name="shoulder_hub",
    )
    clamp_base.visual(
        Box((0.082, 0.145, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=BLACK,
        name="shoulder_block",
    )
    clamp_base.visual(
        Box((0.072, 0.014, 0.084)),
        origin=Origin(xyz=(0.0, -0.067, 0.010)),
        material=BLACK,
        name="shoulder_cheek_0",
    )
    clamp_base.visual(
        Box((0.072, 0.014, 0.084)),
        origin=Origin(xyz=(0.0, 0.067, 0.010)),
        material=BLACK,
        name="shoulder_cheek_1",
    )
    shoulder_pin_geom, shoulder_pin_rot = _cylinder_y(radius=0.010, length=0.164)
    clamp_base.visual(
        shoulder_pin_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=shoulder_pin_rot.rpy),
        material=STEEL,
        name="shoulder_pin",
    )

    lower_arm = model.part("lower_arm")
    lower_shoulder_geom, lower_shoulder_rot = _cylinder_y(radius=0.026, length=0.120)
    lower_arm.visual(
        lower_shoulder_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=lower_shoulder_rot.rpy),
        material=GRAPHITE,
        name="shoulder_barrel",
    )
    for side, y in enumerate((-0.052, 0.052)):
        lower_arm.visual(
            Box((0.290, 0.016, 0.022)),
            origin=Origin(xyz=(0.175, y, 0.055)),
            material=BLACK,
            name=f"lower_rail_{side}",
        )
        lower_arm.visual(
            Box((0.044, 0.020, 0.034)),
            origin=Origin(xyz=(0.026, y, 0.032)),
            material=BLACK,
            name=f"shoulder_web_{side}",
        )
    lower_arm.visual(
        Box((0.064, 0.014, 0.086)),
        origin=Origin(xyz=(0.340, -0.057, 0.080)),
        material=BLACK,
        name="elbow_fork_0",
    )
    lower_arm.visual(
        Box((0.064, 0.014, 0.086)),
        origin=Origin(xyz=(0.340, 0.057, 0.080)),
        material=BLACK,
        name="elbow_fork_1",
    )
    lower_arm.visual(
        Box((0.040, 0.120, 0.014)),
        origin=Origin(xyz=(0.330, 0.0, 0.122)),
        material=BLACK,
        name="elbow_bridge",
    )
    lower_arm.visual(
        Cylinder(radius=0.010, length=0.142),
        origin=Origin(xyz=(0.340, 0.0, 0.080), rpy=lower_shoulder_rot.rpy),
        material=STEEL,
        name="elbow_pin",
    )

    upper_arm = model.part("upper_arm")
    upper_elbow_geom, upper_elbow_rot = _cylinder_y(radius=0.025, length=0.100)
    upper_arm.visual(
        upper_elbow_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=upper_elbow_rot.rpy),
        material=GRAPHITE,
        name="elbow_barrel",
    )
    for side, y in enumerate((-0.037, 0.037)):
        upper_arm.visual(
            Box((0.292, 0.015, 0.020)),
            origin=Origin(xyz=(0.166, y, 0.028)),
            material=BLACK,
            name=f"upper_rail_{side}",
        )
        upper_arm.visual(
            Box((0.035, 0.017, 0.030)),
            origin=Origin(xyz=(0.040, y, 0.017)),
            material=BLACK,
            name=f"elbow_web_{side}",
        )
    upper_arm.visual(
        Box((0.060, 0.094, 0.026)),
        origin=Origin(xyz=(0.318, 0.0, 0.028)),
        material=BLACK,
        name="wrist_block",
    )
    upper_arm.visual(
        Cylinder(radius=0.039, length=0.012),
        origin=Origin(xyz=(0.340, 0.0, 0.034)),
        material=GRAPHITE,
        name="pan_socket",
    )

    pan_plate = model.part("pan_plate")
    pan_plate.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=GRAPHITE,
        name="pan_disc",
    )
    pan_plate.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=BLACK,
        name="pan_stem",
    )
    pan_plate.visual(
        Box((0.075, 0.120, 0.018)),
        origin=Origin(xyz=(0.040, 0.0, 0.031)),
        material=BLACK,
        name="tilt_neck",
    )
    pan_plate.visual(
        Box((0.052, 0.014, 0.072)),
        origin=Origin(xyz=(0.090, -0.053, 0.064)),
        material=BLACK,
        name="tilt_cheek_0",
    )
    pan_plate.visual(
        Box((0.052, 0.014, 0.072)),
        origin=Origin(xyz=(0.090, 0.053, 0.064)),
        material=BLACK,
        name="tilt_cheek_1",
    )
    tilt_pin_geom, tilt_pin_rot = _cylinder_y(radius=0.008, length=0.122)
    pan_plate.visual(
        tilt_pin_geom,
        origin=Origin(xyz=(0.090, 0.0, 0.064), rpy=tilt_pin_rot.rpy),
        material=STEEL,
        name="tilt_pin",
    )

    head_plate = model.part("head_plate")
    head_barrel_geom, head_barrel_rot = _cylinder_y(radius=0.022, length=0.092)
    head_plate.visual(
        head_barrel_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=head_barrel_rot.rpy),
        material=GRAPHITE,
        name="tilt_barrel",
    )
    head_plate.visual(
        Box((0.042, 0.050, 0.030)),
        origin=Origin(xyz=(0.029, 0.0, 0.0)),
        material=BLACK,
        name="plate_neck",
    )
    head_plate.visual(
        Box((0.008, 0.160, 0.160)),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material=BLACK,
        name="vesa_plate",
    )
    head_plate.visual(
        Box((0.012, 0.132, 0.020)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=GRAPHITE,
        name="cross_slot",
    )
    head_plate.visual(
        Box((0.012, 0.020, 0.132)),
        origin=Origin(xyz=(0.059, 0.0, 0.0)),
        material=GRAPHITE,
        name="upright_slot",
    )
    washer_geom, washer_rot = _cylinder_x(radius=0.012, length=0.003)
    hole_geom, hole_rot = _cylinder_x(radius=0.0065, length=0.004)
    for row, z in enumerate((-0.050, 0.050)):
        for col, y in enumerate((-0.050, 0.050)):
            idx = row * 2 + col
            head_plate.visual(
                washer_geom,
                origin=Origin(xyz=(0.0575, y, z), rpy=washer_rot.rpy),
                material=STEEL,
                name=f"vesa_boss_{idx}",
            )
            head_plate.visual(
                hole_geom,
                origin=Origin(xyz=(0.060, y, z), rpy=hole_rot.rpy),
                material=HOLE,
                name=f"vesa_hole_{idx}",
            )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.4, lower=-2.094, upper=2.094),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.340, 0.0, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=-2.094, upper=2.094),
    )
    model.articulation(
        "head_pan",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=pan_plate,
        origin=Origin(xyz=(0.340, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.785, upper=0.785),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_plate,
        child=head_plate,
        origin=Origin(xyz=(0.090, 0.0, 0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.2, lower=-0.349, upper=0.349),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    pan = object_model.get_articulation("head_pan")
    tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "parallel arm hinge axes",
        shoulder.axis == elbow.axis == (0.0, 1.0, 0.0),
        details=f"shoulder={shoulder.axis}, elbow={elbow.axis}",
    )

    for joint, span in ((shoulder, 2.094), (elbow, 2.094), (tilt, 0.349), (pan, 0.785)):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} symmetric travel",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower + span) < 0.002
            and abs(limits.upper - span) < 0.002,
            details=f"limits={limits}",
        )

    clamp = object_model.get_part("clamp_base")
    lower = object_model.get_part("lower_arm")
    upper = object_model.get_part("upper_arm")
    pan_plate = object_model.get_part("pan_plate")
    head = object_model.get_part("head_plate")

    ctx.allow_overlap(
        clamp,
        lower,
        elem_a="shoulder_pin",
        elem_b="shoulder_barrel",
        reason="The steel shoulder pin is intentionally modeled captured inside the solid barrel proxy.",
    )
    ctx.allow_overlap(
        lower,
        upper,
        elem_a="elbow_pin",
        elem_b="elbow_barrel",
        reason="The elbow pivot pin intentionally passes through the barrel knuckle.",
    )
    ctx.allow_overlap(
        pan_plate,
        head,
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        reason="The tilt pin is intentionally captured through the head barrel knuckle.",
    )

    ctx.expect_overlap(
        lower,
        clamp,
        axes="yz",
        elem_a="shoulder_barrel",
        elem_b="shoulder_pin",
        min_overlap=0.018,
        name="shoulder pin passes through barrel",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="yz",
        elem_a="elbow_barrel",
        elem_b="elbow_pin",
        min_overlap=0.018,
        name="elbow pin passes through barrel",
    )
    ctx.expect_contact(
        lower,
        clamp,
        elem_a="shoulder_barrel",
        elem_b="shoulder_cheek_1",
        contact_tol=0.001,
        name="shoulder knuckle captured by yoke",
    )
    ctx.expect_contact(
        upper,
        lower,
        elem_a="elbow_barrel",
        elem_b="elbow_fork_1",
        contact_tol=0.001,
        name="elbow knuckle captured by fork",
    )
    ctx.expect_contact(
        pan_plate,
        upper,
        elem_a="pan_disc",
        elem_b="pan_socket",
        contact_tol=0.001,
        name="pan plate seated on wrist socket",
    )
    ctx.expect_contact(
        head,
        pan_plate,
        elem_a="tilt_barrel",
        elem_b="tilt_cheek_1",
        contact_tol=0.001,
        name="tilt barrel captured by head yoke",
    )

    ctx.expect_overlap(
        head,
        pan_plate,
        axes="yz",
        elem_a="tilt_barrel",
        elem_b="tilt_pin",
        min_overlap=0.015,
        name="tilt pin passes through barrel centerline",
    )

    with ctx.pose({pan: pan.motion_limits.upper, tilt: tilt.motion_limits.upper}):
        posed_head_aabb = ctx.part_element_world_aabb(head, elem="vesa_plate")
        posed_pan_aabb = ctx.part_world_aabb(pan_plate)
    ctx.check(
        "posed head remains mounted at pan plate",
        posed_head_aabb is not None
        and posed_pan_aabb is not None
        and posed_head_aabb[0][0] > posed_pan_aabb[0][0],
        details=f"head={posed_head_aabb}, pan_plate={posed_pan_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
