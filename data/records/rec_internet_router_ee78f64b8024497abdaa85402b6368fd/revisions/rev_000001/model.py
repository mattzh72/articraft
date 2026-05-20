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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rack_router_folding_handles")

    graphite = Material("slightly textured graphite powder coat", rgba=(0.055, 0.060, 0.065, 1.0))
    dark_panel = Material("dark anodized front panel", rgba=(0.018, 0.021, 0.024, 1.0))
    black = Material("matte black plastic", rgba=(0.002, 0.002, 0.002, 1.0))
    gunmetal = Material("blackened folded steel", rgba=(0.010, 0.012, 0.014, 1.0))
    rubber = Material("soft black handle grip", rgba=(0.005, 0.005, 0.004, 1.0))
    green = Material("green status lens", rgba=(0.0, 0.65, 0.17, 1.0))
    amber = Material("amber status lens", rgba=(0.95, 0.55, 0.05, 1.0))

    chassis = model.part("chassis")

    # Shallow 19-inch rack appliance proportions: a 1U-height router body set
    # behind a darker front panel and bolt-on rack ears.
    chassis.visual(
        Box((0.235, 0.430, 0.044)),
        origin=Origin(xyz=(0.1175, 0.0, 0.022)),
        material=graphite,
        name="body_shell",
    )
    chassis.visual(
        Box((0.010, 0.450, 0.048)),
        origin=Origin(xyz=(-0.001, 0.0, 0.024)),
        material=dark_panel,
        name="front_panel",
    )
    chassis.visual(
        Box((0.012, 0.026, 0.048)),
        origin=Origin(xyz=(-0.002, -0.238, 0.024)),
        material=dark_panel,
        name="rack_ear_0",
    )
    chassis.visual(
        Box((0.012, 0.026, 0.048)),
        origin=Origin(xyz=(-0.002, 0.238, 0.024)),
        material=dark_panel,
        name="rack_ear_1",
    )

    # Thin top and bottom lips make the enclosure read like sheet-metal panels
    # wrapped around the shallower case.
    chassis.visual(
        Box((0.225, 0.426, 0.003)),
        origin=Origin(xyz=(0.121, 0.0, 0.0455)),
        material=graphite,
        name="top_lip",
    )
    chassis.visual(
        Box((0.225, 0.426, 0.003)),
        origin=Origin(xyz=(0.121, 0.0, -0.0015)),
        material=graphite,
        name="bottom_lip",
    )

    # Rack screw holes are black recessed disks embedded in the rack ears.
    for i, y in enumerate((-0.238, 0.238)):
        for j, z in enumerate((0.011, 0.037)):
            chassis.visual(
                Cylinder(radius=0.0042, length=0.0014),
                origin=Origin(xyz=(-0.0085, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=black,
                name=f"ear_hole_{i}_{j}",
            )

    # Front vent slots and port recesses provide recognizable router detailing.
    for bank, y0 in enumerate((-0.142, 0.103)):
        for slot in range(7):
            chassis.visual(
                Box((0.002, 0.012, 0.0032)),
                origin=Origin(xyz=(-0.007, y0 + slot * 0.017, 0.031)),
                material=black,
                name=f"vent_slot_{bank}_{slot}",
            )
    for port in range(4):
        y = -0.048 + port * 0.030
        chassis.visual(
            Box((0.003, 0.022, 0.014)),
            origin=Origin(xyz=(-0.0075, y, 0.020)),
            material=black,
            name=f"rj45_port_{port}",
        )
        chassis.visual(
            Box((0.002, 0.004, 0.003)),
            origin=Origin(xyz=(-0.0067, y - 0.006, 0.030)),
            material=green if port % 2 == 0 else amber,
            name=f"port_led_{port}",
        )

    chassis.visual(
        Box((0.003, 0.044, 0.027)),
        origin=Origin(xyz=(-0.0075, 0.0, 0.024)),
        material=black,
        name="switch_bezel",
    )

    # Small fixed hinge pads on the rack ears.  The moving handle sleeves sit
    # just in front of these pads with a narrow running clearance.
    for i, y in enumerate((-0.224, 0.224)):
        chassis.visual(
            Box((0.006, 0.012, 0.040)),
            origin=Origin(xyz=(-0.0088, y, 0.022)),
            material=gunmetal,
            name=f"handle_mount_{i}",
        )

    handle_limits = MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.15)
    pivot_x = -0.0157
    arm_length = 0.058

    for i, (y, direction, axis) in enumerate(((-0.224, 1.0, (0.0, 0.0, 1.0)), (0.224, -1.0, (0.0, 0.0, -1.0)))):
        handle = model.part(f"handle_{i}")
        handle.visual(
            Cylinder(radius=0.0044, length=0.038),
            origin=Origin(),
            material=gunmetal,
            name="hinge_sleeve",
        )
        for z in (-0.0135, 0.0135):
            handle.visual(
                Cylinder(radius=0.0031, length=arm_length),
                origin=Origin(xyz=(-0.001, direction * arm_length / 2.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=gunmetal,
                name=f"side_rod_{'lower' if z < 0 else 'upper'}",
            )
        handle.visual(
            Cylinder(radius=0.0036, length=0.034),
            origin=Origin(xyz=(-0.001, direction * arm_length, 0.0)),
            material=rubber,
            name="grip",
        )
        model.articulation(
            f"chassis_to_handle_{i}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=handle,
            origin=Origin(xyz=(pivot_x, y, 0.022)),
            axis=axis,
            motion_limits=handle_limits,
        )

    switch = model.part("power_switch")
    switch.visual(
        Box((0.006, 0.030, 0.018)),
        origin=Origin(),
        material=black,
        name="rocker_plate",
    )
    switch.visual(
        Box((0.0015, 0.018, 0.003)),
        origin=Origin(xyz=(-0.0037, 0.0, 0.0058)),
        material=Material("subtle switch highlight", rgba=(0.13, 0.13, 0.12, 1.0)),
        name="top_mark",
    )
    model.articulation(
        "chassis_to_power_switch",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=switch,
        origin=Origin(xyz=(-0.0115, 0.0, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.7, velocity=5.0, lower=-0.24, upper=0.24),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    switch = object_model.get_part("power_switch")

    ctx.check(
        "two independent folding handle joints",
        all(object_model.get_articulation(f"chassis_to_handle_{i}") is not None for i in range(2)),
    )
    ctx.check(
        "rocker switch is separately articulated",
        object_model.get_articulation("chassis_to_power_switch") is not None,
    )

    for i in range(2):
        handle = object_model.get_part(f"handle_{i}")
        ctx.allow_overlap(
            chassis,
            handle,
            elem_a=f"handle_mount_{i}",
            elem_b="hinge_sleeve",
            reason="The handle sleeve is intentionally captured by the fixed rack-ear hinge pad with a tiny hidden running overlap.",
        )
        ctx.expect_gap(
            chassis,
            handle,
            axis="x",
            positive_elem=f"handle_mount_{i}",
            negative_elem="hinge_sleeve",
            max_penetration=0.001,
            max_gap=0.003,
            name=f"handle_{i} sleeve is captured at fixed hinge pad",
        )
        ctx.expect_overlap(
            handle,
            chassis,
            axes="z",
            elem_a="hinge_sleeve",
            elem_b=f"handle_mount_{i}",
            min_overlap=0.030,
            name=f"handle_{i} hinge pad spans sleeve height",
        )

    ctx.expect_within(
        switch,
        chassis,
        axes="yz",
        inner_elem="rocker_plate",
        outer_elem="front_panel",
        margin=0.002,
        name="rocker sits near the center of the front panel",
    )
    ctx.allow_overlap(
        chassis,
        switch,
        elem_a="switch_bezel",
        elem_b="rocker_plate",
        reason="The rocker plate is intentionally seated in the front-panel bezel around its transverse pivot.",
    )
    ctx.expect_gap(
        chassis,
        switch,
        axis="x",
        positive_elem="switch_bezel",
        negative_elem="rocker_plate",
        max_penetration=0.001,
        max_gap=0.004,
        name="rocker plate is seated proud in the front bezel",
    )

    for i in range(2):
        handle = object_model.get_part(f"handle_{i}")
        joint = object_model.get_articulation(f"chassis_to_handle_{i}")
        rest_aabb = ctx.part_element_world_aabb(handle, elem="grip")
        with ctx.pose({joint: 1.0}):
            open_aabb = ctx.part_element_world_aabb(handle, elem="grip")
        ctx.check(
            f"handle_{i} folds outward from rack ear",
            rest_aabb is not None
            and open_aabb is not None
            and open_aabb[0][0] < rest_aabb[0][0] - 0.030,
            details=f"rest={rest_aabb}, open={open_aabb}",
        )

    switch_joint = object_model.get_articulation("chassis_to_power_switch")
    rest_mark = ctx.part_element_world_aabb(switch, elem="top_mark")
    with ctx.pose({switch_joint: 0.22}):
        pressed_mark = ctx.part_element_world_aabb(switch, elem="top_mark")
    rest_x = (rest_mark[0][0] + rest_mark[1][0]) / 2.0 if rest_mark is not None else None
    pressed_x = (pressed_mark[0][0] + pressed_mark[1][0]) / 2.0 if pressed_mark is not None else None
    ctx.check(
        "rocker top moves inward on its transverse pivot",
        rest_x is not None and pressed_x is not None and pressed_x > rest_x + 0.001,
        details=f"rest_top_mark_x={rest_x}, pressed_top_mark_x={pressed_x}",
    )

    return ctx.report()


object_model = build_object_model()
