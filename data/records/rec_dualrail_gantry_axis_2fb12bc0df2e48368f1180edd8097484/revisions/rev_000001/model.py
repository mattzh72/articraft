from __future__ import annotations

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
    model = ArticulatedObject(name="tabletop_gantry_axis")

    dark_aluminum = Material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_steel = Material("brushed_linear_rail", rgba=(0.70, 0.72, 0.72, 1.0))
    blue_bridge = Material("blue_powder_coated_bridge", rgba=(0.05, 0.20, 0.55, 1.0))
    red_sled = Material("red_tool_sled", rgba=(0.72, 0.08, 0.05, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    tool_dark = Material("dark_tooling", rgba=(0.03, 0.03, 0.035, 1.0))

    base = model.part("base")
    for y in (-0.36, 0.36):
        base.visual(
            Box((1.20, 0.055, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.095)),
            material=dark_aluminum,
            name="side_rail_neg" if y < 0 else "side_rail_pos",
        )
        base.visual(
            Box((1.08, 0.020, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.134)),
            material=brushed_steel,
            name=f"linear_rail_{'neg' if y < 0 else 'pos'}",
        )
    for x in (-0.575, 0.575):
        base.visual(
            Box((0.090, 0.800, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.070)),
            material=dark_aluminum,
            name="end_tie_neg" if x < 0 else "end_tie_pos",
        )
    for x in (-0.48, 0.48):
        for y in (-0.36, 0.36):
            base.visual(
                Box((0.105, 0.090, 0.030)),
                origin=Origin(xyz=(x, y, 0.015)),
                material=black_rubber,
                name=f"foot_{'neg' if x < 0 else 'pos'}_{'neg' if y < 0 else 'pos'}",
            )
            base.visual(
                Box((0.065, 0.050, 0.055)),
                origin=Origin(xyz=(x, y, 0.055)),
                material=dark_aluminum,
                name=f"foot_post_{'neg' if x < 0 else 'pos'}_{'neg' if y < 0 else 'pos'}",
            )

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.140, 0.780, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue_bridge,
        name="bridge_beam",
    )
    bridge.visual(
        Box((0.050, 0.690, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=brushed_steel,
        name="sled_guide",
    )
    for y in (-0.36, 0.36):
        bridge.visual(
            Box((0.110, 0.035, 0.160)),
            origin=Origin(xyz=(0.0, y, -0.090)),
            material=blue_bridge,
            name=f"end_cheek_{'neg' if y < 0 else 'pos'}",
        )
        bridge.visual(
            Box((0.160, 0.095, 0.035)),
            origin=Origin(xyz=(0.0, y, -0.185)),
            material=brushed_steel,
            name=f"rail_shoe_{'neg' if y < 0 else 'pos'}",
        )

    sled = model.part("sled")
    sled.visual(
        Box((0.120, 0.105, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=red_sled,
        name="saddle_block",
    )
    for x in (-0.047, 0.047):
        sled.visual(
            Box((0.018, 0.105, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.015)),
            material=red_sled,
            name=f"guide_cheek_{'neg' if x < 0 else 'pos'}",
        )
    sled.visual(
        Box((0.078, 0.078, 0.125)),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=red_sled,
        name="tool_mount",
    )
    sled.visual(
        Cylinder(radius=0.018, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, -0.2225)),
        material=tool_dark,
        name="spindle_body",
    )
    sled.visual(
        Cylinder(radius=0.005, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.3425)),
        material=brushed_steel,
        name="tool_bit",
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(-0.390, 0.0, 0.3455)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.780),
    )
    model.articulation(
        "bridge_to_sled",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=sled,
        origin=Origin(xyz=(0.0, -0.240, -0.0775)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.30, lower=0.0, upper=0.480),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    sled = object_model.get_part("sled")
    bridge_axis = object_model.get_articulation("base_to_bridge")
    sled_axis = object_model.get_articulation("bridge_to_sled")

    ctx.expect_overlap(
        bridge,
        base,
        axes="y",
        min_overlap=0.70,
        elem_a="bridge_beam",
        elem_b="end_tie_neg",
        name="bridge spans both fixed side rails",
    )
    ctx.expect_within(
        sled,
        bridge,
        axes="x",
        margin=0.005,
        inner_elem="saddle_block",
        outer_elem="bridge_beam",
        name="small sled stays under beam width",
    )
    ctx.expect_gap(
        bridge,
        sled,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="sled_guide",
        negative_elem="saddle_block",
        name="sled saddle touches underside guide",
    )

    bridge_rest = ctx.part_world_position(bridge)
    with ctx.pose({bridge_axis: 0.780}):
        bridge_extended = ctx.part_world_position(bridge)
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            margin=0.15,
            inner_elem="bridge_beam",
            outer_elem="side_rail_pos",
            name="bridge remains on rail travel at end stroke",
        )
    ctx.check(
        "bridge carriage moves along rail direction",
        bridge_rest is not None
        and bridge_extended is not None
        and bridge_extended[0] > bridge_rest[0] + 0.70
        and abs(bridge_extended[1] - bridge_rest[1]) < 1e-6,
        details=f"rest={bridge_rest}, extended={bridge_extended}",
    )

    sled_rest = ctx.part_world_position(sled)
    with ctx.pose({sled_axis: 0.480}):
        sled_extended = ctx.part_world_position(sled)
        ctx.expect_within(
            sled,
            bridge,
            axes="y",
            margin=0.02,
            inner_elem="saddle_block",
            outer_elem="sled_guide",
            name="sled remains on bridge guide at end stroke",
        )
    ctx.check(
        "tool sled moves along bridge direction",
        sled_rest is not None
        and sled_extended is not None
        and sled_extended[1] > sled_rest[1] + 0.40
        and abs(sled_extended[0] - sled_rest[0]) < 1e-6,
        details=f"rest={sled_rest}, extended={sled_extended}",
    )

    return ctx.report()


object_model = build_object_model()
