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
)


def _tube_x(outer_radius: float, inner_radius: float, length: float):
    """Hollow cylindrical tube centered on the local X axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length * 0.5, both=True)
    )


def _stabilizer_prism():
    """Small trapezoidal glass prism block, centered at the local origin."""
    return (
        cq.Workplane("YZ")
        .polyline(
            [
                (-0.011, -0.009),
                (0.011, -0.009),
                (0.008, 0.009),
                (-0.008, 0.009),
            ]
        )
        .close()
        .extrude(0.034, both=True)
    )


def _cylinder_x_origin(xyz):
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _cylinder_y_origin(xyz):
    return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))


def _add_optical_barrel(
    part,
    *,
    center_y: float,
    mount_side: float,
    main_tube,
    objective_ring,
    eyepiece_ring,
    grip_band,
    rubber,
    black,
    glass,
    name_prefix: str,
) -> None:
    """Add one connected binocular barrel, with hollow front rings and lenses."""
    part.visual(
        main_tube,
        origin=Origin(xyz=(0.000, center_y, 0.0)),
        material=rubber,
        name=f"{name_prefix}_tube",
    )
    part.visual(
        objective_ring,
        origin=Origin(xyz=(0.090, center_y, 0.0)),
        material=black,
        name=f"{name_prefix}_objective_ring",
    )
    part.visual(
        eyepiece_ring,
        origin=Origin(xyz=(-0.092, center_y, 0.0)),
        material=black,
        name=f"{name_prefix}_eyepiece_ring",
    )

    for i, x in enumerate((-0.048, -0.030, -0.012)):
        part.visual(
            grip_band,
            origin=Origin(xyz=(x, center_y, 0.0)),
            material=black,
            name=f"{name_prefix}_grip_band_{i}",
        )

    part.visual(
        Cylinder(radius=0.027, length=0.004),
        origin=_cylinder_x_origin((0.099, center_y, 0.0)),
        material=glass,
        name=f"{name_prefix}_objective_lens",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=_cylinder_x_origin((-0.104, center_y, 0.0)),
        material=glass,
        name=f"{name_prefix}_eyepiece_lens",
    )

    # A molded lug on the inner side of the barrel gives the bridge/hinge a
    # real surface to carry, instead of leaving the tube visually floating.
    lug_center_y = center_y + mount_side * 0.036
    part.visual(
        Box((0.052, 0.012, 0.014)),
        origin=Origin(xyz=(0.000, lug_center_y, 0.0)),
        material=black,
        name=f"{name_prefix}_mount_lug",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="image_stabilized_binoculars")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    rubber = model.material("pebbled_rubber", rgba=(0.035, 0.036, 0.038, 1.0))
    graphite = model.material("graphite_bridge", rgba=(0.12, 0.13, 0.14, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.26, 0.27, 0.28, 1.0))
    coated_glass = model.material("green_coated_glass", rgba=(0.18, 0.44, 0.38, 0.55))
    prism_glass = model.material("stabilizer_glass", rgba=(0.42, 0.72, 0.92, 0.45))

    main_tube = mesh_from_cadquery(
        _tube_x(0.031, 0.021, 0.170),
        "barrel_main_tube",
        tolerance=0.0007,
        angular_tolerance=0.05,
    )
    objective_ring = mesh_from_cadquery(
        _tube_x(0.039, 0.026, 0.020),
        "objective_barrel_ring",
        tolerance=0.0007,
        angular_tolerance=0.05,
    )
    eyepiece_ring = mesh_from_cadquery(
        _tube_x(0.024, 0.015, 0.026),
        "eyepiece_ring",
        tolerance=0.0007,
        angular_tolerance=0.05,
    )
    grip_band = mesh_from_cadquery(
        _tube_x(0.033, 0.030, 0.004),
        "barrel_grip_band",
        tolerance=0.0007,
        angular_tolerance=0.05,
    )
    prism_mesh = mesh_from_cadquery(
        _stabilizer_prism(),
        "stabilizing_prism_block",
        tolerance=0.0005,
        angular_tolerance=0.05,
    )

    bridge = model.part("bridge")
    bridge.visual(
        Cylinder(radius=0.012, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=gunmetal,
        name="center_hinge_knuckle",
    )
    bridge.visual(
        Box((0.060, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
        material=graphite,
        name="fixed_bridge_arm",
    )
    bridge.visual(
        Box((0.018, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material=gunmetal,
        name="hinge_root_strap",
    )
    bridge.visual(
        Box((0.012, 0.008, 0.040)),
        origin=Origin(xyz=(0.0, -0.020, 0.025)),
        material=gunmetal,
        name="upper_bridge_post",
    )
    bridge.visual(
        Box((0.018, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, -0.010, 0.045)),
        material=gunmetal,
        name="post_cap",
    )
    bridge.visual(
        Box((0.120, 0.012, 0.004)),
        origin=Origin(xyz=(0.000, 0.0, 0.045)),
        material=graphite,
        name="upper_bridge_plate",
    )
    bridge.visual(
        Box((0.052, 0.056, 0.006)),
        origin=Origin(xyz=(0.060, 0.0, 0.018)),
        material=graphite,
        name="prism_base",
    )
    bridge.visual(
        Box((0.016, 0.052, 0.004)),
        origin=Origin(xyz=(0.060, 0.0, 0.045)),
        material=gunmetal,
        name="prism_crossbar",
    )
    for suffix, y in (("neg", -0.023), ("pos", 0.023)):
        bridge.visual(
            Box((0.012, 0.006, 0.026)),
            origin=Origin(xyz=(0.060, y, 0.033)),
            material=gunmetal,
            name=f"prism_yoke_{suffix}",
        )
    bridge.visual(
        Box((0.025, 0.014, 0.026)),
        origin=Origin(xyz=(-0.012, 0.0, 0.057)),
        material=graphite,
        name="focus_web",
    )
    bridge.visual(
        Box((0.070, 0.018, 0.006)),
        origin=Origin(xyz=(-0.030, 0.0, 0.070)),
        material=graphite,
        name="focus_spine",
    )
    bridge.visual(
        Box((0.024, 0.050, 0.008)),
        origin=Origin(xyz=(-0.055, 0.0, 0.069)),
        material=graphite,
        name="focus_yoke_base",
    )
    for suffix, y in (("neg", -0.021), ("pos", 0.021)):
        bridge.visual(
            Box((0.012, 0.008, 0.036)),
            origin=Origin(xyz=(-0.055, y, 0.091)),
            material=gunmetal,
            name=f"focus_yoke_{suffix}",
        )

    fixed_barrel = model.part("fixed_barrel")
    _add_optical_barrel(
        fixed_barrel,
        center_y=0.0,
        mount_side=1.0,
        main_tube=main_tube,
        objective_ring=objective_ring,
        eyepiece_ring=eyepiece_ring,
        grip_band=grip_band,
        rubber=rubber,
        black=matte_black,
        glass=coated_glass,
        name_prefix="fixed",
    )

    swing_barrel = model.part("swing_barrel")
    swing_barrel.visual(
        Cylinder(radius=0.014, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0295)),
        material=gunmetal,
        name="upper_hinge_knuckle",
    )
    swing_barrel.visual(
        Cylinder(radius=0.014, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, -0.0295)),
        material=gunmetal,
        name="lower_hinge_knuckle",
    )
    swing_barrel.visual(
        Box((0.018, 0.004, 0.084)),
        origin=Origin(xyz=(0.0, 0.0155, 0.0)),
        material=gunmetal,
        name="hinge_strap",
    )
    swing_barrel.visual(
        Box((0.052, 0.034, 0.012)),
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
        material=graphite,
        name="swing_bridge_arm",
    )
    _add_optical_barrel(
        swing_barrel,
        center_y=0.060,
        mount_side=-1.0,
        main_tube=main_tube,
        objective_ring=objective_ring,
        eyepiece_ring=eyepiece_ring,
        grip_band=grip_band,
        rubber=rubber,
        black=matte_black,
        glass=coated_glass,
        name_prefix="swing",
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=_cylinder_y_origin((0.0, 0.0, 0.0)),
        material=matte_black,
        name="wheel_body",
    )
    focus_wheel.visual(
        Cylinder(radius=0.004, length=0.034),
        origin=_cylinder_y_origin((0.0, 0.0, 0.0)),
        material=gunmetal,
        name="focus_axle",
    )
    for i in range(16):
        theta = 2.0 * math.pi * i / 16.0
        radial = 0.0195
        focus_wheel.visual(
            Box((0.0030, 0.030, 0.0050)),
            origin=Origin(
                xyz=(radial * math.cos(theta), 0.0, radial * math.sin(theta)),
                rpy=(0.0, -theta, 0.0),
            ),
            material=rubber,
            name=f"grip_rib_{i}",
        )

    prism_block = model.part("prism_block")
    prism_block.visual(
        prism_mesh,
        origin=Origin(),
        material=prism_glass,
        name="glass_prism",
    )
    prism_block.visual(
        Cylinder(radius=0.003, length=0.040),
        origin=_cylinder_y_origin((0.0, 0.0, 0.0)),
        material=gunmetal,
        name="gimbal_pin",
    )

    model.articulation(
        "bridge_to_fixed_barrel",
        ArticulationType.FIXED,
        parent=bridge,
        child=fixed_barrel,
        origin=Origin(xyz=(0.0, -0.060, 0.0)),
    )
    model.articulation(
        "ipd_hinge",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=swing_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.6, lower=0.0, upper=0.26),
    )
    model.articulation(
        "focus_spin",
        ArticulationType.CONTINUOUS,
        parent=bridge,
        child=focus_wheel,
        origin=Origin(xyz=(-0.055, 0.0, 0.091)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )
    model.articulation(
        "prism_stabilizer",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=prism_block,
        origin=Origin(xyz=(0.060, 0.0, 0.033)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.1, velocity=1.0, lower=-0.08, upper=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge")
    fixed_barrel = object_model.get_part("fixed_barrel")
    swing_barrel = object_model.get_part("swing_barrel")
    focus_wheel = object_model.get_part("focus_wheel")
    prism_block = object_model.get_part("prism_block")
    ipd_hinge = object_model.get_articulation("ipd_hinge")
    focus_spin = object_model.get_articulation("focus_spin")
    prism_stabilizer = object_model.get_articulation("prism_stabilizer")

    ctx.check(
        "ipd hinge is a limited revolute joint",
        ipd_hinge.articulation_type == ArticulationType.REVOLUTE
        and ipd_hinge.motion_limits is not None
        and ipd_hinge.motion_limits.upper == 0.26,
    )
    ctx.check(
        "focus wheel is continuous",
        focus_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(focus_spin.axis) == (0.0, 1.0, 0.0),
    )
    ctx.check(
        "stabilizing prism has small tilt travel",
        prism_stabilizer.articulation_type == ArticulationType.REVOLUTE
        and prism_stabilizer.motion_limits is not None
        and prism_stabilizer.motion_limits.lower < 0.0
        and prism_stabilizer.motion_limits.upper > 0.0,
    )

    ctx.expect_contact(
        fixed_barrel,
        bridge,
        elem_a="fixed_mount_lug",
        elem_b="fixed_bridge_arm",
        contact_tol=0.001,
        name="fixed barrel lug seats on bridge",
    )
    ctx.expect_contact(
        focus_wheel,
        bridge,
        elem_a="focus_axle",
        elem_b="focus_yoke_pos",
        contact_tol=0.001,
        name="focus axle is carried by yoke",
    )
    ctx.expect_contact(
        prism_block,
        bridge,
        elem_a="gimbal_pin",
        elem_b="prism_yoke_pos",
        contact_tol=0.001,
        name="prism gimbal pin is captured by yoke",
    )
    ctx.expect_overlap(
        fixed_barrel,
        swing_barrel,
        axes="xz",
        min_overlap=0.018,
        elem_a="fixed_objective_ring",
        elem_b="swing_objective_ring",
        name="paired objective barrels align forward",
    )

    rest_eye = ctx.part_element_world_aabb(swing_barrel, elem="swing_eyepiece_ring")
    with ctx.pose({ipd_hinge: 0.26}):
        folded_eye = ctx.part_element_world_aabb(swing_barrel, elem="swing_eyepiece_ring")
    ctx.check(
        "ipd hinge draws eyepiece inward",
        rest_eye is not None
        and folded_eye is not None
        and folded_eye[1][1] < rest_eye[1][1] - 0.015,
        details=f"rest={rest_eye}, folded={folded_eye}",
    )

    with ctx.pose({focus_spin: 1.7, prism_stabilizer: 0.06}):
        ctx.expect_within(
            prism_block,
            bridge,
            axes="y",
            inner_elem="glass_prism",
            outer_elem="prism_base",
            margin=0.0,
            name="stabilizing prism stays inside bridge window",
        )

    return ctx.report()


object_model = build_object_model()
