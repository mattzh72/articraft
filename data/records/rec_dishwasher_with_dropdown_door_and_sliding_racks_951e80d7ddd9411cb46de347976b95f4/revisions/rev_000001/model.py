from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


WIDTH = 0.60
DEPTH = 0.62
HEIGHT = 0.82
WALL = 0.025


def _add_wash_arm(
    part,
    *,
    span: float,
    width: float,
    thickness: float,
    material,
    hub_center_z: float,
    blade_center_z: float,
    shaft_length: float = 0.0,
    shaft_center_z: float = 0.0,
) -> None:
    if shaft_length > 0.0:
        part.visual(
            Cylinder(radius=0.008, length=shaft_length),
            origin=Origin(xyz=(0.0, 0.0, shaft_center_z)),
            material=material,
            name="spindle",
        )
    part.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, hub_center_z)),
        material=material,
        name="hub",
    )
    arm_half = span * 0.5
    for suffix, x_sign in (("a", -1.0), ("b", 1.0)):
        part.visual(
            Box((arm_half, width, thickness)),
            origin=Origin(
                xyz=(x_sign * arm_half * 0.5, 0.0, blade_center_z),
            ),
            material=material,
            name=f"arm_{suffix}",
        )
        part.visual(
            Box((0.034, width * 0.65, thickness * 0.85)),
            origin=Origin(
                xyz=(x_sign * (arm_half - 0.017), 0.0, blade_center_z),
            ),
            material=material,
            name=f"tip_{suffix}",
        )


def _add_lower_rack(part, *, material, runner_material) -> None:
    runner_z = 0.007
    part.visual(
        Box((0.014, 0.48, 0.014)),
        origin=Origin(xyz=(-0.267, 0.24, runner_z)),
        material=runner_material,
        name="runner_left",
    )
    part.visual(
        Box((0.014, 0.48, 0.014)),
        origin=Origin(xyz=(0.267, 0.24, runner_z)),
        material=runner_material,
        name="runner_right",
    )

    for x_value in (-0.223, 0.223):
        part.visual(
            Box((0.008, 0.454, 0.008)),
            origin=Origin(xyz=(x_value, 0.235, 0.045)),
            material=material,
            name=f"base_side_{'left' if x_value < 0.0 else 'right'}",
        )
        part.visual(
            Box((0.008, 0.454, 0.008)),
            origin=Origin(xyz=(x_value, 0.235, 0.120)),
            material=material,
            name=f"top_side_{'left' if x_value < 0.0 else 'right'}",
        )

    for y_value, tag in ((0.008, "front"), (0.462, "rear")):
        part.visual(
            Box((0.446, 0.008, 0.008)),
            origin=Origin(xyz=(0.0, y_value, 0.045)),
            material=material,
            name=f"base_{tag}",
        )
        part.visual(
            Box((0.446, 0.008, 0.008)),
            origin=Origin(xyz=(0.0, y_value, 0.120)),
            material=material,
            name=f"top_{tag}",
        )

    for x_value in (-0.223, 0.223):
        for y_value, tag in ((0.008, "front"), (0.462, "rear")):
            part.visual(
                Box((0.008, 0.008, 0.075)),
                origin=Origin(xyz=(x_value, y_value, 0.0825)),
                material=material,
                name=f"post_{'left' if x_value < 0.0 else 'right'}_{tag}",
            )

    for x_value, side_name in ((-0.243, "left"), (0.243, "right")):
        for y_value, tag in ((0.055, "front"), (0.415, "rear")):
            part.visual(
                Box((0.034, 0.030, 0.035)),
                origin=Origin(xyz=(x_value, y_value, 0.0245)),
                material=material,
                name=f"runner_mount_{side_name}_{tag}",
            )

    for index, y_value in enumerate((0.105, 0.205, 0.305, 0.405)):
        part.visual(
            Box((0.438, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, y_value, 0.038)),
            material=material,
            name=f"wire_{index}",
        )

    part.visual(
        Box((0.180, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.000, 0.120)),
        material=material,
        name="handle",
    )


def _add_upper_rack(part, *, material, runner_material) -> None:
    part.visual(
        Box((0.014, 0.44, 0.014)),
        origin=Origin(xyz=(-0.255, 0.22, 0.0)),
        material=runner_material,
        name="runner_left",
    )
    part.visual(
        Box((0.014, 0.44, 0.014)),
        origin=Origin(xyz=(0.255, 0.22, 0.0)),
        material=runner_material,
        name="runner_right",
    )

    for x_value in (-0.210, 0.210):
        part.visual(
            Box((0.008, 0.400, 0.008)),
            origin=Origin(xyz=(x_value, 0.22, -0.020)),
            material=material,
            name=f"top_side_{'left' if x_value < 0.0 else 'right'}",
        )
        part.visual(
            Box((0.008, 0.400, 0.008)),
            origin=Origin(xyz=(x_value, 0.22, -0.072)),
            material=material,
            name=f"base_side_{'left' if x_value < 0.0 else 'right'}",
        )

    for y_value, tag in ((0.020, "front"), (0.420, "rear")):
        part.visual(
            Box((0.420, 0.008, 0.008)),
            origin=Origin(xyz=(0.0, y_value, -0.020)),
            material=material,
            name=f"top_{tag}",
        )
        part.visual(
            Box((0.420, 0.008, 0.008)),
            origin=Origin(xyz=(0.0, y_value, -0.072)),
            material=material,
            name=f"base_{tag}",
        )

    for x_value in (-0.210, 0.210):
        for y_value, tag in ((0.020, "front"), (0.420, "rear")):
            part.visual(
                Box((0.008, 0.008, 0.052)),
                origin=Origin(xyz=(x_value, y_value, -0.046)),
                material=material,
                name=f"post_{'left' if x_value < 0.0 else 'right'}_{tag}",
            )

    for x_value, side_name in ((-0.230, "left"), (0.230, "right")):
        for y_value, tag in ((0.080, "front"), (0.360, "rear")):
            part.visual(
                Box((0.036, 0.030, 0.075)),
                origin=Origin(xyz=(x_value, y_value, -0.0305)),
                material=material,
                name=f"runner_mount_{side_name}_{tag}",
            )

    for index, y_value in enumerate((0.095, 0.180, 0.265, 0.350)):
        part.visual(
            Box((0.412, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, y_value, -0.079)),
            material=material,
            name=f"wire_{index}",
        )

    part.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=Origin(xyz=(0.0, 0.220, -0.046)),
        material=runner_material,
        name="spray_mount",
    )
    part.visual(
        Box((0.412, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.220, -0.020)),
        material=material,
        name="center_bar",
    )

    part.visual(
        Box((0.160, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.010, -0.020)),
        material=material,
        name="handle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_dishwasher")

    stainless = model.material("stainless", rgba=(0.72, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.40, 0.42, 0.45, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    control_trim = model.material("control_trim", rgba=(0.25, 0.27, 0.30, 1.0))

    body = model.part("body")
    body.visual(
        Box((WALL, DEPTH, HEIGHT)),
        origin=Origin(xyz=(-(WIDTH - WALL) * 0.5, 0.0, HEIGHT * 0.5)),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((WALL, DEPTH, HEIGHT)),
        origin=Origin(xyz=((WIDTH - WALL) * 0.5, 0.0, HEIGHT * 0.5)),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL, DEPTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT - WALL * 0.5)),
        material=stainless,
        name="top_wall",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL, DEPTH, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=stainless,
        name="bottom_wall",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL, WALL, HEIGHT - WALL - 0.030)),
        origin=Origin(xyz=(0.0, (DEPTH - WALL) * 0.5, 0.4125)),
        material=stainless,
        name="back_wall",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL, 0.040, 0.120)),
        origin=Origin(xyz=(0.0, -0.290, 0.760)),
        material=control_trim,
        name="control_panel",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL, 0.040, 0.090)),
        origin=Origin(xyz=(0.0, -0.290, 0.075)),
        material=control_trim,
        name="threshold",
    )

    for name, x_value, z_value in (
        ("lower_guide_left", -0.267, 0.177),
        ("lower_guide_right", 0.267, 0.177),
        ("upper_guide_left", -0.255, 0.494),
        ("upper_guide_right", 0.255, 0.494),
    ):
        body.visual(
            Box((0.018, 0.480 if "lower" in name else 0.440, 0.012)),
            origin=Origin(xyz=(x_value, -0.040, z_value)),
            material=dark_steel,
            name=name,
        )
    for x_value, side_name in ((-0.271, "left"), (0.271, "right")):
        for y_value, z_value, tag in (
            (-0.190, 0.177, "lower_rear"),
            (0.120, 0.177, "lower_front"),
            (-0.180, 0.494, "upper_rear"),
            (0.110, 0.494, "upper_front"),
        ):
            body.visual(
                Box((0.022, 0.040, 0.018)),
                origin=Origin(xyz=(x_value, y_value, z_value)),
                material=dark_steel,
                name=f"guide_bracket_{side_name}_{tag}",
            )

    body.visual(
        Cylinder(radius=0.037, length=0.002),
        origin=Origin(
            xyz=(-0.165, -0.309, 0.760),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=graphite,
        name="dial_bezel",
    )
    for index, x_value in enumerate((0.040, 0.090, 0.140)):
        body.visual(
            Box((0.032, 0.002, 0.044)),
            origin=Origin(xyz=(x_value, -0.309, 0.760)),
            material=graphite,
            name=f"switch_bezel_{index}",
        )

    door = model.part("door")
    door.visual(
        Box((0.580, 0.030, 0.620)),
        origin=Origin(xyz=(0.0, -0.015, 0.310)),
        material=stainless,
        name="door_shell",
    )
    door.visual(
        Box((0.520, 0.012, 0.540)),
        origin=Origin(xyz=(0.0, 0.006, 0.315)),
        material=dark_steel,
        name="door_liner",
    )
    door.visual(
        Box((0.300, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.045, 0.555)),
        material=graphite,
        name="door_handle",
    )
    door.visual(
        Box((0.180, 0.035, 0.110)),
        origin=Origin(xyz=(0.0, 0.024, 0.340)),
        material=dark_steel,
        name="detergent_pocket",
    )

    lower_rack = model.part("lower_rack")
    _add_lower_rack(lower_rack, material=dark_steel, runner_material=graphite)

    upper_rack = model.part("upper_rack")
    _add_upper_rack(upper_rack, material=dark_steel, runner_material=graphite)

    lower_arm = model.part("lower_arm")
    _add_wash_arm(
        lower_arm,
        span=0.340,
        width=0.030,
        thickness=0.012,
        material=black,
        hub_center_z=0.008,
        blade_center_z=0.006,
        shaft_length=0.110,
        shaft_center_z=-0.055,
    )

    upper_arm = model.part("upper_arm")
    _add_wash_arm(
        upper_arm,
        span=0.300,
        width=0.026,
        thickness=0.010,
        material=black,
        hub_center_z=-0.008,
        blade_center_z=-0.005,
        shaft_length=0.085,
        shaft_center_z=-0.0425,
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.060,
                0.026,
                body_style="skirted",
                top_diameter=0.049,
                edge_radius=0.002,
                skirt=KnobSkirt(0.066, 0.006, flare=0.06),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "dishwasher_timer_dial",
        ),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=graphite,
        name="dial_knob",
    )

    for index, x_value in enumerate((0.040, 0.090, 0.140)):
        switch = model.part(f"switch_{index}")
        switch.visual(
            Box((0.026, 0.008, 0.040)),
            origin=Origin(xyz=(0.0, -0.010, 0.0)),
            material=black,
            name="rocker",
        )
        switch.visual(
            Box((0.014, 0.006, 0.014)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material=graphite,
            name="pivot",
        )
        model.articulation(
            f"body_to_switch_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=switch,
            origin=Origin(xyz=(x_value, -DEPTH * 0.5, 0.760)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=3.0,
                lower=-0.30,
                upper=0.30,
            ),
        )

    detergent_flap = model.part("detergent_flap")
    detergent_flap.visual(
        Box((0.145, 0.003, 0.095)),
        origin=Origin(xyz=(0.0, 0.0015, -0.0475)),
        material=graphite,
        name="flap_panel",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -DEPTH * 0.5, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "body_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.0, -0.280, 0.183)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=0.240,
        ),
    )
    model.articulation(
        "body_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.0, -0.280, 0.507)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.28,
            lower=0.0,
            upper=0.220,
        ),
    )
    model.articulation(
        "body_to_lower_arm",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.020, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=16.0),
    )
    model.articulation(
        "body_to_upper_arm",
        ArticulationType.CONTINUOUS,
        parent=upper_rack,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.220, -0.076)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=16.0),
    )
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(-0.165, -DEPTH * 0.5, 0.760)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )
    model.articulation(
        "door_to_detergent_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_flap,
        origin=Origin(xyz=(0.0, 0.0415, 0.395)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    detergent_flap = object_model.get_part("detergent_flap")

    door_joint = object_model.get_articulation("body_to_door")
    lower_joint = object_model.get_articulation("body_to_lower_rack")
    upper_joint = object_model.get_articulation("body_to_upper_rack")
    flap_joint = object_model.get_articulation("door_to_detergent_flap")

    ctx.expect_origin_gap(
        upper_rack,
        lower_rack,
        axis="z",
        min_gap=0.300,
        name="upper rack sits above lower rack",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    lower_upper = lower_joint.motion_limits.upper if lower_joint.motion_limits is not None else None
    if lower_upper is not None:
        with ctx.pose({lower_joint: lower_upper}):
            lower_extended = ctx.part_world_position(lower_rack)
            ctx.expect_within(
                lower_rack,
                "body",
                axes="x",
                inner_elem="runner_left",
                outer_elem="lower_guide_left",
                margin=0.002,
                name="lower rack runner stays on left guide",
            )
            ctx.expect_overlap(
                lower_rack,
                "body",
                axes="y",
                elem_a="runner_left",
                elem_b="lower_guide_left",
                min_overlap=0.190,
                name="lower rack remains inserted on left guide",
            )
        ctx.check(
            "lower rack extends outward",
            lower_rest is not None
            and lower_extended is not None
            and lower_extended[1] < lower_rest[1] - 0.18,
            details=f"rest={lower_rest!r}, extended={lower_extended!r}",
        )

    upper_rest = ctx.part_world_position(upper_rack)
    upper_upper = upper_joint.motion_limits.upper if upper_joint.motion_limits is not None else None
    if upper_upper is not None:
        with ctx.pose({upper_joint: upper_upper}):
            upper_extended = ctx.part_world_position(upper_rack)
            ctx.expect_within(
                upper_rack,
                "body",
                axes="x",
                inner_elem="runner_left",
                outer_elem="upper_guide_left",
                margin=0.002,
                name="upper rack runner stays on left guide",
            )
            ctx.expect_overlap(
                upper_rack,
                "body",
                axes="y",
                elem_a="runner_left",
                elem_b="upper_guide_left",
                min_overlap=0.170,
                name="upper rack remains inserted on left guide",
            )
        ctx.check(
            "upper rack extends outward",
            upper_rest is not None
            and upper_extended is not None
            and upper_extended[1] < upper_rest[1] - 0.16,
            details=f"rest={upper_rest!r}, extended={upper_extended!r}",
        )

    door_closed = ctx.part_element_world_aabb(door, elem="door_shell")
    door_upper = door_joint.motion_limits.upper if door_joint.motion_limits is not None else None
    if door_upper is not None:
        with ctx.pose({door_joint: door_upper}):
            door_open = ctx.part_element_world_aabb(door, elem="door_shell")
        ctx.check(
            "door drops downward when opened",
            door_closed is not None
            and door_open is not None
            and door_open[1][2] < door_closed[1][2] - 0.45
            and door_open[0][1] < door_closed[0][1] - 0.20,
            details=f"closed={door_closed!r}, open={door_open!r}",
        )

    flap_closed = ctx.part_element_world_aabb(detergent_flap, elem="flap_panel")
    flap_upper = flap_joint.motion_limits.upper if flap_joint.motion_limits is not None else None
    if flap_upper is not None:
        with ctx.pose({flap_joint: flap_upper}):
            flap_open = ctx.part_element_world_aabb(detergent_flap, elem="flap_panel")
        ctx.check(
            "detergent flap opens toward the chamber",
            flap_closed is not None
            and flap_open is not None
            and flap_open[1][1] > flap_closed[1][1] + 0.04,
            details=f"closed={flap_closed!r}, open={flap_open!r}",
        )

    return ctx.report()


object_model = build_object_model()
