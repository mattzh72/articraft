from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_automation_gantry")

    anodized = model.material("black_anodized_aluminum", rgba=(0.05, 0.055, 0.06, 1.0))
    blue = model.material("blue_gantry_carriage", rgba=(0.04, 0.19, 0.42, 1.0))
    shuttle_red = model.material("red_center_shuttle", rgba=(0.75, 0.08, 0.05, 1.0))
    rail_steel = model.material("brushed_linear_rail_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    screw_dark = model.material("dark_fasteners", rgba=(0.02, 0.02, 0.025, 1.0))

    base = model.part("base_frame")
    # Grounded rectangular bench frame: two long side rails tied by end rails and
    # a middle tie, with the working linear guides sitting proud on top.
    for y, name in ((-0.32, "side_beam_0"), (0.32, "side_beam_1")):
        base.visual(
            Box((1.60, 0.075, 0.080)),
            origin=Origin(xyz=(0.0, y, 0.040)),
            material=anodized,
            name=name,
        )
    for x, name in ((-0.765, "end_beam_0"), (0.765, "end_beam_1")):
        base.visual(
            Box((0.070, 0.715, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.040)),
            material=anodized,
            name=name,
        )
    base.visual(
        Box((0.090, 0.715, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=anodized,
        name="center_tie",
    )
    base.visual(
        Box((1.48, 0.032, 0.028)),
        origin=Origin(xyz=(0.0, -0.32, 0.094)),
        material=rail_steel,
        name="linear_rail_0",
    )
    base.visual(
        Box((1.48, 0.032, 0.028)),
        origin=Origin(xyz=(0.0, 0.32, 0.094)),
        material=rail_steel,
        name="linear_rail_1",
    )
    for x, y, name in (
        (-0.70, -0.32, "foot_0"),
        (-0.70, 0.32, "foot_1"),
        (0.70, -0.32, "foot_2"),
        (0.70, 0.32, "foot_3"),
    ):
        base.visual(
            Box((0.120, 0.105, 0.022)),
            origin=Origin(xyz=(x, y, -0.011)),
            material=rubber,
            name=name,
        )

    bridge = model.part("bridge_carriage")
    # The moving portal carriage sits on the two base rails.  Its local origin
    # is at the base center so the X prismatic joint directly reads as travel.
    bridge.visual(
        Box((0.170, 0.085, 0.052)),
        origin=Origin(xyz=(0.0, -0.32, 0.134)),
        material=blue,
        name="rail_shoe_0",
    )
    bridge.visual(
        Box((0.105, 0.065, 0.520)),
        origin=Origin(xyz=(0.0, -0.32, 0.420)),
        material=blue,
        name="upright_0",
    )
    bridge.visual(
        Box((0.170, 0.085, 0.052)),
        origin=Origin(xyz=(0.0, 0.32, 0.134)),
        material=blue,
        name="rail_shoe_1",
    )
    bridge.visual(
        Box((0.105, 0.065, 0.520)),
        origin=Origin(xyz=(0.0, 0.32, 0.420)),
        material=blue,
        name="upright_1",
    )
    bridge.visual(
        Box((0.140, 0.815, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, 0.728)),
        material=blue,
        name="top_crossbeam",
    )
    bridge.visual(
        Box((0.110, 0.720, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.666)),
        material=rail_steel,
        name="y_linear_rail",
    )
    # Small end blocks make the second-axis rail read as a captured slide rather
    # than a floating strip on the beam underside.
    for y, name in ((-0.375, "y_end_stop_0"), (0.375, "y_end_stop_1")):
        bridge.visual(
            Box((0.145, 0.030, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.675)),
            material=screw_dark,
            name=name,
        )

    shuttle = model.part("center_shuttle")
    # Short nested shuttle: the main block runs just below the Y rail, while the
    # side cheeks wrap up around the crossbeam sides to sell the captured guide.
    shuttle.visual(
        Box((0.155, 0.160, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=shuttle_red,
        name="shuttle_core",
    )
    for x, name in ((-0.092, "side_cheek_0"), (0.092, "side_cheek_1")):
        shuttle.visual(
            Box((0.032, 0.160, 0.096)),
            origin=Origin(xyz=(x, 0.0, 0.042)),
            material=shuttle_red,
            name=name,
        )
    shuttle.visual(
        Box((0.122, 0.108, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
        material=screw_dark,
        name="tool_plate",
    )
    for y, z, name in (
        (-0.032, -0.060, "front_bolt_0"),
        (0.032, -0.060, "front_bolt_1"),
        (-0.032, -0.112, "front_bolt_2"),
        (0.032, -0.112, "front_bolt_3"),
    ):
        shuttle.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(-0.064, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=rail_steel,
            name=name,
        )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=-0.46, upper=0.46),
    )
    model.articulation(
        "bridge_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, 0.615)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=-0.255, upper=0.255),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    bridge = object_model.get_part("bridge_carriage")
    shuttle = object_model.get_part("center_shuttle")
    bridge_slide = object_model.get_articulation("base_to_bridge")
    shuttle_slide = object_model.get_articulation("bridge_to_shuttle")

    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        positive_elem="rail_shoe_0",
        negative_elem="linear_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="bridge shoe sits on first base rail",
    )
    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        positive_elem="rail_shoe_1",
        negative_elem="linear_rail_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="bridge shoe sits on second base rail",
    )
    ctx.expect_overlap(
        bridge,
        base,
        axes="xy",
        elem_a="rail_shoe_0",
        elem_b="linear_rail_0",
        min_overlap=0.030,
        name="first bridge shoe is engaged on rail footprint",
    )
    ctx.expect_gap(
        bridge,
        shuttle,
        axis="z",
        positive_elem="y_linear_rail",
        negative_elem="shuttle_core",
        max_gap=0.006,
        max_penetration=0.0,
        name="shuttle is tucked just below bridge rail",
    )
    ctx.expect_overlap(
        shuttle,
        bridge,
        axes="y",
        elem_a="shuttle_core",
        elem_b="y_linear_rail",
        min_overlap=0.140,
        name="center shuttle remains nested along bridge rail at center",
    )

    bridge_rest = ctx.part_world_position(bridge)
    with ctx.pose({bridge_slide: 0.46}):
        ctx.expect_overlap(
            bridge,
            base,
            axes="x",
            elem_a="rail_shoe_0",
            elem_b="linear_rail_0",
            min_overlap=0.120,
            name="bridge keeps rail engagement at forward travel",
        )
        bridge_extended = ctx.part_world_position(bridge)
    ctx.check(
        "bridge prismatic axis travels along base rail direction",
        bridge_rest is not None
        and bridge_extended is not None
        and bridge_extended[0] > bridge_rest[0] + 0.40,
        details=f"rest={bridge_rest}, extended={bridge_extended}",
    )

    shuttle_rest = ctx.part_world_position(shuttle)
    with ctx.pose({shuttle_slide: 0.255}):
        ctx.expect_overlap(
            shuttle,
            bridge,
            axes="y",
            elem_a="shuttle_core",
            elem_b="y_linear_rail",
            min_overlap=0.140,
            name="shuttle remains captured at bridge-axis travel",
        )
        shuttle_extended = ctx.part_world_position(shuttle)
    ctx.check(
        "shuttle prismatic axis travels across the bridge",
        shuttle_rest is not None
        and shuttle_extended is not None
        and shuttle_extended[1] > shuttle_rest[1] + 0.22,
        details=f"rest={shuttle_rest}, extended={shuttle_extended}",
    )

    return ctx.report()


object_model = build_object_model()
