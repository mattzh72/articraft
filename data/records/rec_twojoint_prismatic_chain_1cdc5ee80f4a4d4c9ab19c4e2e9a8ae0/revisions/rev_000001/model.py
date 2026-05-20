from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="compact_two_stage_service_axis")

    dark_cast = Material("dark_cast_body", rgba=(0.08, 0.09, 0.10, 1.0))
    black = Material("black_bearing", rgba=(0.015, 0.016, 0.018, 1.0))
    rail_steel = Material("ground_steel_rails", rgba=(0.66, 0.68, 0.66, 1.0))
    screw_steel = Material("polished_drive_screw", rgba=(0.78, 0.78, 0.74, 1.0))
    carriage_aluminum = Material("satin_aluminum_carriage", rgba=(0.58, 0.61, 0.63, 1.0))
    terminal_orange = Material("orange_terminal_plate", rgba=(0.95, 0.43, 0.10, 1.0))
    rubber = Material("black_rubber_feet", rgba=(0.01, 0.01, 0.01, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.62, 0.30, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_cast,
        name="ground_plate",
    )
    body.visual(
        Box((0.56, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -0.086, 0.064)),
        material=rail_steel,
        name="rail_0",
    )
    body.visual(
        Box((0.56, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.086, 0.064)),
        material=rail_steel,
        name="rail_1",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.54),
        origin=Origin(xyz=(0.0, 0.0, 0.094), rpy=(0.0, pi / 2.0, 0.0)),
        material=screw_steel,
        name="drive_screw",
    )
    for x in (-0.270, 0.270):
        body.visual(
            Box((0.034, 0.062, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.075)),
            material=dark_cast,
            name=f"end_bearing_{0 if x < 0 else 1}",
        )
    for x in (-0.245, 0.245):
        body.visual(
            Box((0.035, 0.045, 0.018)),
            origin=Origin(xyz=(x, -0.118, -0.008)),
            material=rubber,
            name=f"foot_{0 if x < 0 else 1}",
        )
        body.visual(
            Box((0.035, 0.045, 0.018)),
            origin=Origin(xyz=(x, 0.118, -0.008)),
            material=rubber,
            name=f"foot_{2 if x < 0 else 3}",
        )
    for x in (-0.210, 0.0, 0.210):
        for y, suffix in ((-0.086, "a"), (0.086, "b")):
            body.visual(
                Cylinder(radius=0.0075, length=0.004),
                origin=Origin(xyz=(x, y, 0.080)),
                material=black,
                name=f"rail_bolt_{suffix}_{int((x + 0.21) * 1000)}",
            )

    main_carriage = model.part("main_carriage")
    main_carriage.visual(
        Box((0.190, 0.048, 0.028)),
        origin=Origin(xyz=(0.0, -0.086, 0.014)),
        material=black,
        name="bearing_0",
    )
    main_carriage.visual(
        Box((0.190, 0.048, 0.028)),
        origin=Origin(xyz=(0.0, 0.086, 0.014)),
        material=black,
        name="bearing_1",
    )
    main_carriage.visual(
        Box((0.240, 0.205, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.0455)),
        material=carriage_aluminum,
        name="main_saddle",
    )
    main_carriage.visual(
        Box((0.190, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.055, 0.0725)),
        material=rail_steel,
        name="secondary_rail_0",
    )
    main_carriage.visual(
        Box((0.190, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.055, 0.0725)),
        material=rail_steel,
        name="secondary_rail_1",
    )
    main_carriage.visual(
        Cylinder(radius=0.006, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.082), rpy=(0.0, pi / 2.0, 0.0)),
        material=screw_steel,
        name="second_drive_screw",
    )
    main_carriage.visual(
        Box((0.026, 0.070, 0.030)),
        origin=Origin(xyz=(-0.105, 0.0, 0.077)),
        material=carriage_aluminum,
        name="rear_stop",
    )
    main_carriage.visual(
        Box((0.026, 0.070, 0.030)),
        origin=Origin(xyz=(0.105, 0.0, 0.077)),
        material=carriage_aluminum,
        name="front_stop",
    )

    terminal_carriage = model.part("terminal_carriage")
    terminal_carriage.visual(
        Box((0.095, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, -0.055, 0.010)),
        material=black,
        name="terminal_bearing_0",
    )
    terminal_carriage.visual(
        Box((0.095, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.055, 0.010)),
        material=black,
        name="terminal_bearing_1",
    )
    terminal_carriage.visual(
        Box((0.126, 0.126, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=terminal_orange,
        name="tool_plate",
    )
    terminal_carriage.visual(
        Box((0.034, 0.090, 0.050)),
        origin=Origin(xyz=(0.055, 0.0, 0.070)),
        material=terminal_orange,
        name="nose_mount",
    )
    terminal_carriage.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.030, 0.035, 0.049)),
        material=black,
        name="socket_bolt_0",
    )
    terminal_carriage.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.030, -0.035, 0.049)),
        material=black,
        name="socket_bolt_1",
    )

    model.articulation(
        "body_to_main",
        ArticulationType.PRISMATIC,
        parent=body,
        child=main_carriage,
        origin=Origin(xyz=(-0.085, 0.0, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.35, lower=0.0, upper=0.170),
    )
    model.articulation(
        "main_to_terminal",
        ArticulationType.PRISMATIC,
        parent=main_carriage,
        child=terminal_carriage,
        origin=Origin(xyz=(-0.040, 0.0, 0.0815)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.28, lower=0.0, upper=0.080),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    main_carriage = object_model.get_part("main_carriage")
    terminal_carriage = object_model.get_part("terminal_carriage")
    body_to_main = object_model.get_articulation("body_to_main")
    main_to_terminal = object_model.get_articulation("main_to_terminal")

    ctx.expect_gap(
        main_carriage,
        body,
        axis="z",
        positive_elem="bearing_0",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="main lower bearing sits on base rail",
    )
    ctx.expect_overlap(
        main_carriage,
        body,
        axes="xy",
        elem_a="bearing_0",
        elem_b="rail_0",
        min_overlap=0.015,
        name="main carriage is engaged with base guide",
    )
    ctx.expect_gap(
        terminal_carriage,
        main_carriage,
        axis="z",
        positive_elem="terminal_bearing_0",
        negative_elem="secondary_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="terminal bearing sits on secondary rail",
    )
    ctx.expect_overlap(
        terminal_carriage,
        main_carriage,
        axes="xy",
        elem_a="terminal_bearing_0",
        elem_b="secondary_rail_0",
        min_overlap=0.015,
        name="terminal carriage is engaged with secondary guide",
    )

    rest_main = ctx.part_world_position(main_carriage)
    rest_terminal = ctx.part_world_position(terminal_carriage)
    with ctx.pose({body_to_main: 0.170}):
        extended_main = ctx.part_world_position(main_carriage)
        ctx.expect_overlap(
            main_carriage,
            body,
            axes="x",
            elem_a="bearing_0",
            elem_b="rail_0",
            min_overlap=0.06,
            name="main stage retains rail engagement at travel limit",
        )
    with ctx.pose({main_to_terminal: 0.080}):
        extended_terminal = ctx.part_world_position(terminal_carriage)
        ctx.expect_overlap(
            terminal_carriage,
            main_carriage,
            axes="x",
            elem_a="terminal_bearing_0",
            elem_b="secondary_rail_0",
            min_overlap=0.04,
            name="terminal stage retains rail engagement at travel limit",
        )

    ctx.check(
        "main prismatic stage moves along +x",
        rest_main is not None
        and extended_main is not None
        and extended_main[0] > rest_main[0] + 0.16,
        details=f"rest={rest_main}, extended={extended_main}",
    )
    ctx.check(
        "terminal prismatic stage moves along +x",
        rest_terminal is not None
        and extended_terminal is not None
        and extended_terminal[0] > rest_terminal[0] + 0.075,
        details=f"rest={rest_terminal}, extended={extended_terminal}",
    )

    return ctx.report()


object_model = build_object_model()
