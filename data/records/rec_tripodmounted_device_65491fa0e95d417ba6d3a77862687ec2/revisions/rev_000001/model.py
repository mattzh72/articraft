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


PAINT = Material("wrinkle black powder coat", rgba=(0.035, 0.038, 0.036, 1.0))
OLIVE = Material("molded olive housing", rgba=(0.25, 0.29, 0.18, 1.0))
RUBBER = Material("matte black rubber", rgba=(0.006, 0.006, 0.005, 1.0))
STEEL = Material("brushed exposed steel", rgba=(0.55, 0.56, 0.52, 1.0))
PAD = Material("dark anti slip pad", rgba=(0.015, 0.016, 0.014, 1.0))
GLASS = Material("smoked instrument glass", rgba=(0.02, 0.04, 0.045, 1.0))


def _cylinder_between(part, name, p0, p1, radius, material):
    """Add a cylinder whose local Z axis is aligned between two local points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("zero-length cylinder")
    pitch = math.acos(max(-1.0, min(1.0, dz / length)))
    yaw = math.atan2(dy, dx)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_tripodmounted_device")

    hub = model.part("hub")

    # Tripod crown, central post, and rigid head plate.
    hub.visual(Cylinder(radius=0.115, length=0.16), origin=Origin(xyz=(0, 0, 0.78)), material=PAINT, name="lower_collar")
    hub.visual(Cylinder(radius=0.045, length=0.36), origin=Origin(xyz=(0, 0, 0.96)), material=PAINT, name="center_post")
    hub.visual(Cylinder(radius=0.065, length=0.11), origin=Origin(xyz=(0, 0, 1.105)), material=PAINT, name="upper_collar")
    hub.visual(Box((0.48, 0.36, 0.04)), origin=Origin(xyz=(0, 0, 1.15)), material=PAINT, name="head_plate")
    hub.visual(Box((0.38, 0.26, 0.012)), origin=Origin(xyz=(0, 0, 1.176)), material=PAD, name="deck_pad")
    hub.visual(Box((0.30, 0.16, 0.05)), origin=Origin(xyz=(0, 0, 1.195)), material=PAINT, name="yoke_base")

    pivot_z = 1.30
    hub.visual(Box((0.040, 0.15, 0.24)), origin=Origin(xyz=(-0.205, 0.0, 1.30)), material=PAINT, name="yoke_cheek_0")
    hub.visual(Box((0.040, 0.15, 0.24)), origin=Origin(xyz=(0.205, 0.0, 1.30)), material=PAINT, name="yoke_cheek_1")
    hub.visual(Cylinder(radius=0.044, length=0.012), origin=Origin(xyz=(-0.231, 0.0, pivot_z), rpy=(0.0, math.pi / 2, 0.0)), material=STEEL, name="left_washer")
    hub.visual(Cylinder(radius=0.044, length=0.012), origin=Origin(xyz=(0.231, 0.0, pivot_z), rpy=(0.0, math.pi / 2, 0.0)), material=STEEL, name="right_washer")

    # Head-plate exposed service fasteners.
    for i, (x, y) in enumerate(((-0.18, -0.13), (0.18, -0.13), (-0.18, 0.13), (0.18, 0.13))):
        hub.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(x, y, 1.178), rpy=(0.0, 0.0, 0.0)),
            material=STEEL,
            name=f"deck_bolt_{i}",
        )

    # Three reinforced leg clevises around the lower crown.
    leg_pivot_radius = 0.165
    leg_pivot_z = 0.78
    for i, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        r = (math.cos(theta), math.sin(theta), 0.0)
        t = (-math.sin(theta), math.cos(theta), 0.0)
        pivot = (leg_pivot_radius * r[0], leg_pivot_radius * r[1], leg_pivot_z)
        # Radial bridge to the cast crown, below the rotating barrel.
        hub.visual(
            Box((0.12, 0.13, 0.035)),
            origin=Origin(
                xyz=(pivot[0] - 0.060 * r[0], pivot[1] - 0.060 * r[1], pivot[2] - 0.075),
                rpy=(0.0, 0.0, theta),
            ),
            material=PAINT,
            name=f"leg_bridge_{i}",
        )
        for side, sgn in enumerate((-1.0, 1.0)):
            hub.visual(
                Box((0.095, 0.018, 0.115)),
                origin=Origin(
                    xyz=(pivot[0] + sgn * 0.060 * t[0], pivot[1] + sgn * 0.060 * t[1], pivot[2] - 0.005),
                    rpy=(0.0, 0.0, theta),
                ),
                material=PAINT,
                name=f"leg_clevis_{i}_{side}",
            )
        # Exposed pin heads outside the fork cheeks.
        for side, sgn in enumerate((-1.0, 1.0)):
            hub.visual(
                Cylinder(radius=0.020, length=0.016),
                origin=Origin(
                    xyz=(pivot[0] + sgn * 0.076 * t[0], pivot[1] + sgn * 0.076 * t[1], pivot[2]),
                    rpy=(-math.pi / 2.0, 0.0, theta),
                ),
                material=STEEL,
                name=f"leg_pin_head_{i}_{side}",
            )

    # Articulated tripod legs, each built as a rugged twin-tube assembly.
    for i, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{i}")
        leg.visual(Cylinder(radius=0.032, length=0.102), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=STEEL, name="hinge_barrel")
        leg.visual(Box((0.110, 0.090, 0.055)), origin=Origin(xyz=(0.045, 0.0, -0.025)), material=PAINT, name="upper_knuckle")

        for side, y in enumerate((-0.030, 0.030)):
            _cylinder_between(leg, f"upper_tube_{side}", (0.065, y, -0.045), (0.485, y, -0.690), 0.018, PAINT)
            leg.visual(
                Cylinder(radius=0.025, length=0.050),
                origin=Origin(xyz=(0.250, y, -0.335), rpy=(0.0, 2.56, 0.0)),
                material=STEEL,
                name=f"clamp_collar_{side}",
            )
        leg.visual(Box((0.12, 0.082, 0.050)), origin=Origin(xyz=(0.250, 0.0, -0.335)), material=PAINT, name="middle_clamp")
        leg.visual(Box((0.18, 0.105, 0.038)), origin=Origin(xyz=(0.515, 0.0, -0.728)), material=RUBBER, name="foot_pad")
        leg.visual(Cylinder(radius=0.016, length=0.096), origin=Origin(xyz=(0.505, 0.0, -0.695), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=STEEL, name="foot_axle")
        leg.visual(Box((0.10, 0.028, 0.030)), origin=Origin(xyz=(0.430, 0.0, -0.600)), material=STEEL, name="wear_guard")

        model.articulation(
            f"leg_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=leg,
            origin=Origin(xyz=(leg_pivot_radius * math.cos(theta), leg_pivot_radius * math.sin(theta), leg_pivot_z), rpy=(0.0, 0.0, theta)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.25, upper=0.30),
        )

    cradle = model.part("tilt_cradle")
    cradle.visual(Cylinder(radius=0.028, length=0.370), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=STEEL, name="trunnion_bar")
    cradle.visual(Box((0.325, 0.034, 0.290)), origin=Origin(xyz=(0.0, 0.075, 0.155)), material=PAINT, name="mount_plate")
    cradle.visual(Box((0.055, 0.060, 0.285)), origin=Origin(xyz=(-0.160, 0.058, 0.140)), material=PAINT, name="side_arm_0")
    cradle.visual(Box((0.055, 0.060, 0.285)), origin=Origin(xyz=(0.160, 0.058, 0.140)), material=PAINT, name="side_arm_1")
    cradle.visual(Box((0.315, 0.050, 0.035)), origin=Origin(xyz=(0.0, 0.070, 0.290)), material=PAINT, name="top_rail")
    cradle.visual(Box((0.315, 0.050, 0.030)), origin=Origin(xyz=(0.0, 0.070, 0.028)), material=PAINT, name="lower_rail")
    standoff_specs = (
        (-0.125, 0.150, "standoff_0", "bracket_bolt_0"),
        (0.125, 0.150, "standoff_1", "bracket_bolt_1"),
        (-0.125, 0.275, "standoff_2", "bracket_bolt_2"),
        (0.125, 0.275, "standoff_3", "bracket_bolt_3"),
    )
    for x, z, standoff_name, bolt_name in standoff_specs:
        cradle.visual(Cylinder(radius=0.014, length=0.013), origin=Origin(xyz=(x, 0.0985, z), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=STEEL, name=standoff_name)
        cradle.visual(Cylinder(radius=0.010, length=0.004), origin=Origin(xyz=(x, 0.102, z), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=STEEL, name=bolt_name)

    model.articulation(
        "tilt_pivot",
        ArticulationType.REVOLUTE,
        parent=hub,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-0.45, upper=0.65),
    )

    device = model.part("device")
    device.visual(Box((0.380, 0.110, 0.260)), material=OLIVE, name="body_shell")
    device.visual(Box((0.250, 0.012, 0.135)), origin=Origin(xyz=(0.0, 0.061, 0.015)), material=GLASS, name="front_window")
    device.visual(Box((0.335, 0.010, 0.205)), origin=Origin(xyz=(0.0, 0.058, 0.0)), material=RUBBER, name="front_gasket")
    for i, (x, z) in enumerate(((-0.170, -0.115), (0.170, -0.115), (-0.170, 0.115), (0.170, 0.115))):
        device.visual(Box((0.055, 0.126, 0.055)), origin=Origin(xyz=(x, 0.0, z)), material=RUBBER, name=f"corner_bumper_{i}")
        device.visual(Cylinder(radius=0.010, length=0.008), origin=Origin(xyz=(x * 0.94, 0.065, z * 0.82), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=STEEL, name=f"face_screw_{i}")
    device.visual(Box((0.030, 0.038, 0.075)), origin=Origin(xyz=(-0.115, -0.010, 0.166)), material=RUBBER, name="handle_post_0")
    device.visual(Box((0.030, 0.038, 0.075)), origin=Origin(xyz=(0.115, -0.010, 0.166)), material=RUBBER, name="handle_post_1")
    device.visual(Cylinder(radius=0.016, length=0.260), origin=Origin(xyz=(0.0, -0.010, 0.212), rpy=(0.0, math.pi / 2.0, 0.0)), material=RUBBER, name="top_handle")
    device.visual(Box((0.090, 0.014, 0.024)), origin=Origin(xyz=(-0.068, 0.067, -0.074)), material=STEEL, name="label_plate")
    device.visual(Box((0.060, 0.018, 0.034)), origin=Origin(xyz=(0.105, 0.070, -0.075)), material=RUBBER, name="sealed_button")

    model.articulation(
        "cradle_to_device",
        ArticulationType.FIXED,
        parent=cradle,
        child=device,
        origin=Origin(xyz=(0.0, 0.160, 0.270)),
    )

    knob = model.part("tilt_knob")
    knob.visual(Cylinder(radius=0.050, length=0.055), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=RUBBER, name="knob_body")
    knob.visual(Cylinder(radius=0.024, length=0.010), origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=STEEL, name="center_cap")
    knob.visual(Box((0.010, 0.086, 0.014)), origin=Origin(xyz=(0.026, 0.0, 0.0)), material=STEEL, name="driver_slot")

    model.articulation(
        "tilt_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=hub,
        child=knob,
        origin=Origin(xyz=(0.2645, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hub = object_model.get_part("hub")
    cradle = object_model.get_part("tilt_cradle")
    device = object_model.get_part("device")
    tilt = object_model.get_articulation("tilt_pivot")

    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        aabb = ctx.part_world_aabb(leg)
        ctx.check(
            f"leg {i} reaches ground",
            aabb is not None and aabb[0][2] < 0.06 and aabb[1][2] > 0.75,
            details=f"aabb={aabb}",
        )
        for side in range(2):
            ctx.allow_overlap(
                hub,
                leg,
                elem_a=f"leg_clevis_{i}_{side}",
                elem_b="hinge_barrel",
                reason="The hinge barrel is intentionally captured in the reinforced leg clevis cheeks.",
            )
            ctx.expect_contact(
                hub,
                leg,
                elem_a=f"leg_clevis_{i}_{side}",
                elem_b="hinge_barrel",
                contact_tol=0.001,
                name=f"leg {i} clevis {side} captures hinge barrel",
            )

    ctx.expect_gap(device, hub, axis="z", min_gap=0.004, name="device clears fixed yoke")
    ctx.expect_contact(device, cradle, elem_a="body_shell", elem_b="standoff_0", contact_tol=0.003, name="device bears on bracket standoff")

    rest_aabb = ctx.part_world_aabb(device)
    with ctx.pose({tilt: 0.60}):
        tilted_aabb = ctx.part_world_aabb(device)
    ctx.check(
        "tilt pivot raises device",
        rest_aabb is not None
        and tilted_aabb is not None
        and (tilted_aabb[0][2] + tilted_aabb[1][2]) * 0.5 > (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5 + 0.015,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    leg0 = object_model.get_part("leg_0")
    leg0_joint = object_model.get_articulation("leg_pivot_0")
    rest_leg = ctx.part_world_aabb(leg0)
    with ctx.pose({leg0_joint: -0.20}):
        spread_leg = ctx.part_world_aabb(leg0)
    ctx.check(
        "leg pivot increases spread",
        rest_leg is not None and spread_leg is not None and spread_leg[1][0] > rest_leg[1][0] + 0.08,
        details=f"rest={rest_leg}, spread={spread_leg}",
    )

    return ctx.report()


object_model = build_object_model()
