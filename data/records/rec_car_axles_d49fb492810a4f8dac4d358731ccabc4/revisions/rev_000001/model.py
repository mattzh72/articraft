from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twist_beam_rear_axle")

    painted_steel = Material("satin_black_painted_steel", rgba=(0.02, 0.022, 0.024, 1.0))
    raw_steel = Material("machined_steel", rgba=(0.56, 0.56, 0.52, 1.0))
    dark_rubber = Material("black_rubber_bushing", rgba=(0.005, 0.005, 0.004, 1.0))
    weld_blue = Material("heat_tinted_welds", rgba=(0.08, 0.10, 0.12, 1.0))

    chassis = model.part("chassis_bracket")
    chassis.visual(
        Box((0.18, 1.86, 0.08)),
        origin=Origin(xyz=(-0.08, 0.0, 0.64)),
        material=painted_steel,
        name="cross_rail",
    )

    pivot_y = 0.72
    pivot_z = 0.42
    for index, y0 in enumerate((-pivot_y, pivot_y)):
        for ear_sign in (-1.0, 1.0):
            ear_y = y0 + ear_sign * 0.085
            chassis.visual(
                Box((0.13, 0.038, 0.25)),
                origin=Origin(xyz=(0.0, ear_y, pivot_z)),
                material=painted_steel,
                name=f"bracket_{index}_{'outer' if ear_sign == (1.0 if y0 > 0 else -1.0) else 'inner'}_ear",
            )
            chassis.visual(
                Box((0.09, 0.038, 0.24)),
                origin=Origin(xyz=(-0.055, ear_y, 0.525)),
                material=painted_steel,
                name=f"hanger_{index}_{ear_sign:+.0f}",
            )
            chassis.visual(
                Cylinder(radius=0.052, length=0.026),
                origin=Origin(
                    xyz=(0.0, y0 + ear_sign * 0.111, pivot_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=raw_steel,
                name=f"bolt_head_{index}_{ear_sign:+.0f}",
            )

    def add_trailing_arm(name: str, *, side_sign: float):
        """Create one trailing arm half with the inward half of the C-channel beam."""
        inward = -side_sign
        arm = model.part(name)
        arm.visual(
            Cylinder(radius=0.064, length=0.122),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_rubber,
            name="pivot_bushing",
        )
        arm.visual(
            Cylinder(radius=0.040, length=0.138),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=raw_steel,
            name="bushing_sleeve",
        )
        arm.visual(
            Box((1.08, 0.075, 0.105)),
            origin=Origin(xyz=(0.54, 0.0, 0.0)),
            material=painted_steel,
            name="trailing_arm_tube",
        )
        arm.visual(
            Box((0.16, 0.08, 0.23)),
            origin=Origin(xyz=(1.12, 0.0, 0.0)),
            material=painted_steel,
            name="hub_carrier",
        )
        arm.visual(
            Box((0.12, 0.09, 0.20)),
            origin=Origin(xyz=(0.72, inward * 0.035, 0.0)),
            material=weld_blue,
            name="beam_weld_pad",
        )
        arm.visual(
            Box((0.045, 0.72, 0.18)),
            origin=Origin(xyz=(0.775, inward * 0.36, 0.015)),
            material=painted_steel,
            name="beam_web",
        )
        arm.visual(
            Box((0.17, 0.72, 0.035)),
            origin=Origin(xyz=(0.705, inward * 0.36, 0.105)),
            material=painted_steel,
            name="beam_top_flange",
        )
        arm.visual(
            Box((0.17, 0.72, 0.035)),
            origin=Origin(xyz=(0.705, inward * 0.36, -0.075)),
            material=painted_steel,
            name="beam_bottom_flange",
        )
        arm.visual(
            Box((0.055, 0.72, 0.045)),
            origin=Origin(xyz=(0.61, inward * 0.36, 0.015)),
            material=weld_blue,
            name="open_lip",
        )
        return arm

    arm_0 = add_trailing_arm("trailing_arm_0", side_sign=-1.0)
    arm_1 = add_trailing_arm("trailing_arm_1", side_sign=1.0)

    mount_limits = MotionLimits(effort=4500.0, velocity=2.0, lower=-0.24, upper=0.24)
    model.articulation(
        "arm_mount_0",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=arm_0,
        origin=Origin(xyz=(0.0, -pivot_y, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=mount_limits,
    )
    model.articulation(
        "arm_mount_1",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=arm_1,
        origin=Origin(xyz=(0.0, pivot_y, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=mount_limits,
        mimic=Mimic(joint="arm_mount_0", multiplier=1.0, offset=0.0),
    )

    def add_hub(name: str, *, side_sign: float):
        outward = side_sign
        hub = model.part(name)
        hub.visual(
            Cylinder(radius=0.135, length=0.080),
            origin=Origin(xyz=(0.0, outward * 0.086, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=raw_steel,
            name="hub_flange",
        )
        hub.visual(
            Cylinder(radius=0.070, length=0.070),
            origin=Origin(xyz=(0.0, outward * 0.138, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=raw_steel,
            name="bearing_cap",
        )
        hub.visual(
            Cylinder(radius=0.032, length=0.060),
            origin=Origin(xyz=(0.0, outward * 0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=raw_steel,
            name="inner_spigot",
        )
        for stud_index in range(5):
            angle = 2.0 * math.pi * stud_index / 5.0
            hub.visual(
                Cylinder(radius=0.013, length=0.045),
                origin=Origin(
                    xyz=(0.088 * math.cos(angle), outward * 0.145, 0.088 * math.sin(angle)),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=raw_steel,
                name=f"wheel_stud_{stud_index}",
            )
        return hub

    hub_0 = add_hub("hub_0", side_sign=-1.0)
    hub_1 = add_hub("hub_1", side_sign=1.0)

    model.articulation(
        "hub_axle_0",
        ArticulationType.CONTINUOUS,
        parent=arm_0,
        child=hub_0,
        origin=Origin(xyz=(1.12, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=60.0),
    )
    model.articulation(
        "hub_axle_1",
        ArticulationType.CONTINUOUS,
        parent=arm_1,
        child=hub_1,
        origin=Origin(xyz=(1.12, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=60.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    arm_0 = object_model.get_part("trailing_arm_0")
    arm_1 = object_model.get_part("trailing_arm_1")
    hub_0 = object_model.get_part("hub_0")
    hub_1 = object_model.get_part("hub_1")
    mount_0 = object_model.get_articulation("arm_mount_0")
    mount_1 = object_model.get_articulation("arm_mount_1")
    hub_axle_0 = object_model.get_articulation("hub_axle_0")
    hub_axle_1 = object_model.get_articulation("hub_axle_1")

    ctx.check(
        "trailing arm mounts are horizontal revolutes",
        mount_0.articulation_type == ArticulationType.REVOLUTE
        and mount_1.articulation_type == ArticulationType.REVOLUTE
        and tuple(mount_0.axis) == (0.0, 1.0, 0.0)
        and tuple(mount_1.axis) == (0.0, 1.0, 0.0),
        details=f"mount_0={mount_0.articulation_type} axis={mount_0.axis}; mount_1={mount_1.articulation_type} axis={mount_1.axis}",
    )
    ctx.check(
        "wheel hubs spin on axle axes",
        hub_axle_0.articulation_type == ArticulationType.CONTINUOUS
        and hub_axle_1.articulation_type == ArticulationType.CONTINUOUS
        and tuple(hub_axle_0.axis) == (0.0, 1.0, 0.0)
        and tuple(hub_axle_1.axis) == (0.0, 1.0, 0.0),
        details=f"hub_axle_0={hub_axle_0.articulation_type} axis={hub_axle_0.axis}; hub_axle_1={hub_axle_1.articulation_type} axis={hub_axle_1.axis}",
    )
    for hub, arm, label in ((hub_0, arm_0, "0"), (hub_1, arm_1, "1")):
        ctx.allow_overlap(
            hub,
            arm,
            elem_a="inner_spigot",
            elem_b="hub_carrier",
            reason="The hub's inner spigot is intentionally seated inside the trailing-arm bearing carrier proxy.",
        )
        ctx.expect_within(
            hub,
            arm,
            axes="xz",
            inner_elem="inner_spigot",
            outer_elem="hub_carrier",
            margin=0.002,
            name=f"hub spigot {label} is centered in carrier",
        )
        ctx.expect_overlap(
            hub,
            arm,
            axes="y",
            elem_a="inner_spigot",
            elem_b="hub_carrier",
            min_overlap=0.035,
            name=f"hub spigot {label} remains inserted in carrier",
        )
    ctx.expect_contact(
        arm_0,
        arm_1,
        elem_a="beam_web",
        elem_b="beam_web",
        contact_tol=0.001,
        name="C-section torsion beam halves meet at center",
    )
    ctx.expect_overlap(
        arm_0,
        arm_1,
        axes="xz",
        elem_a="beam_top_flange",
        elem_b="beam_top_flange",
        min_overlap=0.03,
        name="C-section top flanges align across the axle",
    )

    rest_hub = ctx.part_world_position(hub_0)
    with ctx.pose({mount_0: -0.16}):
        bumped_hub_0 = ctx.part_world_position(hub_0)
        bumped_hub_1 = ctx.part_world_position(hub_1)
    ctx.check(
        "paired trailing arms bump upward together",
        rest_hub is not None
        and bumped_hub_0 is not None
        and bumped_hub_1 is not None
        and bumped_hub_0[2] > rest_hub[2] + 0.12
        and abs(bumped_hub_0[2] - bumped_hub_1[2]) < 0.002,
        details=f"rest={rest_hub}, bumped_0={bumped_hub_0}, bumped_1={bumped_hub_1}",
    )

    return ctx.report()


object_model = build_object_model()
