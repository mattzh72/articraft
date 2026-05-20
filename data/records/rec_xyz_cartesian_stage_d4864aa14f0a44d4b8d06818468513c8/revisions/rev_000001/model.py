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
    model = ArticulatedObject(name="laboratory_axis_stage")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.62, 0.64, 0.65, 1.0))
    dark_rail = model.material("dark_burnished_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.74, 0.76, 0.77, 1.0))
    black_cover = model.material("black_anodized_cover", rgba=(0.015, 0.017, 0.02, 1.0))
    blue_label = model.material("muted_axis_label", rgba=(0.06, 0.16, 0.32, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.08, 0.46, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cast_aluminum,
        name="base_plate",
    )
    base.visual(
        Box((0.98, 0.035, 0.022)),
        origin=Origin(xyz=(0.0, -0.16, 0.066)),
        material=dark_rail,
        name="base_rail_0",
    )
    base.visual(
        Box((0.98, 0.035, 0.022)),
        origin=Origin(xyz=(0.0, 0.16, 0.066)),
        material=dark_rail,
        name="base_rail_1",
    )
    for i, x in enumerate((-0.505, 0.505)):
        base.visual(
            Box((0.045, 0.115, 0.050)),
            origin=Origin(xyz=(x, -0.16, 0.080)),
            material=black_cover,
            name=f"x_stop_0_{i}",
        )
        base.visual(
            Box((0.045, 0.115, 0.050)),
            origin=Origin(xyz=(x, 0.16, 0.080)),
            material=black_cover,
            name=f"x_stop_1_{i}",
        )
    base.visual(
        Box((0.86, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=ground_steel,
        name="machined_center_face",
    )

    bridge_saddle = model.part("bridge_saddle")
    bridge_saddle.visual(
        Box((0.180, 0.080, 0.045)),
        origin=Origin(xyz=(0.0, -0.16, 0.0225)),
        material=cast_aluminum,
        name="x_bearing_0",
    )
    bridge_saddle.visual(
        Box((0.155, 0.052, 0.008)),
        origin=Origin(xyz=(0.0, -0.16, 0.049)),
        material=ground_steel,
        name="x_bearing_cap_0",
    )
    bridge_saddle.visual(
        Box((0.180, 0.080, 0.045)),
        origin=Origin(xyz=(0.0, 0.16, 0.0225)),
        material=cast_aluminum,
        name="x_bearing_1",
    )
    bridge_saddle.visual(
        Box((0.155, 0.052, 0.008)),
        origin=Origin(xyz=(0.0, 0.16, 0.049)),
        material=ground_steel,
        name="x_bearing_cap_1",
    )
    bridge_saddle.visual(
        Box((0.280, 0.410, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
        material=cast_aluminum,
        name="bridge_bed",
    )
    bridge_saddle.visual(
        Box((0.220, 0.360, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=cast_aluminum,
        name="raised_y_deck",
    )
    bridge_saddle.visual(
        Box((0.022, 0.465, 0.022)),
        origin=Origin(xyz=(-0.105, 0.0, 0.111)),
        material=dark_rail,
        name="y_rail_0",
    )
    bridge_saddle.visual(
        Box((0.022, 0.465, 0.022)),
        origin=Origin(xyz=(0.105, 0.0, 0.111)),
        material=dark_rail,
        name="y_rail_1",
    )
    for i, y in enumerate((-0.245, 0.245)):
        bridge_saddle.visual(
            Box((0.210, 0.025, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.1275)),
            material=black_cover,
            name=f"y_stop_{i}",
        )
    bridge_saddle.visual(
        Box((0.055, 0.300, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        material=blue_label,
        name="y_axis_scale",
    )

    cross_carriage = model.part("cross_carriage")
    cross_carriage.visual(
        Box((0.065, 0.130, 0.040)),
        origin=Origin(xyz=(-0.105, 0.0, 0.020)),
        material=cast_aluminum,
        name="y_bearing_0",
    )
    cross_carriage.visual(
        Box((0.050, 0.100, 0.006)),
        origin=Origin(xyz=(-0.105, 0.0, 0.043)),
        material=ground_steel,
        name="y_bearing_cap_0",
    )
    cross_carriage.visual(
        Box((0.075, 0.190, 0.030)),
        origin=Origin(xyz=(-0.105, 0.0, 0.055)),
        material=cast_aluminum,
        name="z_saddle_side_0",
    )
    cross_carriage.visual(
        Box((0.065, 0.130, 0.040)),
        origin=Origin(xyz=(0.105, 0.0, 0.020)),
        material=cast_aluminum,
        name="y_bearing_1",
    )
    cross_carriage.visual(
        Box((0.050, 0.100, 0.006)),
        origin=Origin(xyz=(0.105, 0.0, 0.043)),
        material=ground_steel,
        name="y_bearing_cap_1",
    )
    cross_carriage.visual(
        Box((0.075, 0.190, 0.030)),
        origin=Origin(xyz=(0.105, 0.0, 0.055)),
        material=cast_aluminum,
        name="z_saddle_side_1",
    )
    for i, y in enumerate((-0.080, 0.080)):
        cross_carriage.visual(
            Box((0.205, 0.035, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.055)),
            material=cast_aluminum,
            name=f"z_saddle_crossbar_{i}",
        )
    for i, x in enumerate((-0.062, 0.062)):
        cross_carriage.visual(
            Box((0.032, 0.110, 0.300)),
            origin=Origin(xyz=(x, 0.0, 0.220)),
            material=cast_aluminum,
            name=f"z_guide_column_{i}",
        )
    cross_carriage.visual(
        Box((0.150, 0.025, 0.300)),
        origin=Origin(xyz=(0.0, 0.060, 0.220)),
        material=black_cover,
        name="z_back_cover",
    )
    cross_carriage.visual(
        Box((0.006, 0.080, 0.260)),
        origin=Origin(xyz=(-0.043, 0.0, 0.220)),
        material=ground_steel,
        name="left_guide_face",
    )
    cross_carriage.visual(
        Box((0.006, 0.080, 0.260)),
        origin=Origin(xyz=(0.043, 0.0, 0.220)),
        material=ground_steel,
        name="right_guide_face",
    )
    cross_carriage.visual(
        Box((0.170, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, -0.065, 0.3875)),
        material=black_cover,
        name="front_ram_cover",
    )
    cross_carriage.visual(
        Box((0.170, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, 0.065, 0.3875)),
        material=black_cover,
        name="rear_ram_cover",
    )

    z_ram = model.part("z_ram")
    z_ram.visual(
        Box((0.080, 0.055, 0.325)),
        origin=Origin(xyz=(0.0, 0.0, -0.1625)),
        material=ground_steel,
        name="ram_body",
    )
    z_ram.visual(
        Box((0.052, 0.006, 0.300)),
        origin=Origin(xyz=(0.0, -0.033, -0.175)),
        material=black_cover,
        name="ram_front_cover",
    )
    z_ram.visual(
        Box((0.130, 0.100, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.3375)),
        material=cast_aluminum,
        name="mounting_flange",
    )
    for ix, x in enumerate((-0.043, 0.043)):
        for iy, y in enumerate((-0.030, 0.030)):
            z_ram.visual(
                Cylinder(radius=0.008, length=0.005),
                origin=Origin(xyz=(x, y, -0.3515)),
                material=dark_rail,
                name=f"flange_bolt_{ix}_{iy}",
            )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge_saddle,
        origin=Origin(xyz=(-0.280, 0.0, 0.077)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.45, lower=0.0, upper=0.560),
    )
    model.articulation(
        "bridge_to_cross",
        ArticulationType.PRISMATIC,
        parent=bridge_saddle,
        child=cross_carriage,
        origin=Origin(xyz=(0.0, -0.135, 0.122)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.270),
    )
    model.articulation(
        "cross_to_ram",
        ArticulationType.PRISMATIC,
        parent=cross_carriage,
        child=z_ram,
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.120),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge_saddle")
    cross = object_model.get_part("cross_carriage")
    ram = object_model.get_part("z_ram")
    x_axis = object_model.get_articulation("base_to_bridge")
    y_axis = object_model.get_articulation("bridge_to_cross")
    z_axis = object_model.get_articulation("cross_to_ram")

    bridge_rest = ctx.part_world_position(bridge)
    with ctx.pose({x_axis: 0.560}):
        bridge_extended = ctx.part_world_position(bridge)
    ctx.check(
        "bridge saddle travels along X",
        bridge_rest is not None
        and bridge_extended is not None
        and bridge_extended[0] > bridge_rest[0] + 0.55
        and abs(bridge_extended[1] - bridge_rest[1]) < 1e-6
        and abs(bridge_extended[2] - bridge_rest[2]) < 1e-6,
        details=f"rest={bridge_rest}, extended={bridge_extended}",
    )

    cross_rest = ctx.part_world_position(cross)
    with ctx.pose({y_axis: 0.270}):
        cross_extended = ctx.part_world_position(cross)
    ctx.check(
        "cross carriage travels along Y",
        cross_rest is not None
        and cross_extended is not None
        and cross_extended[1] > cross_rest[1] + 0.26
        and abs(cross_extended[0] - cross_rest[0]) < 1e-6
        and abs(cross_extended[2] - cross_rest[2]) < 1e-6,
        details=f"rest={cross_rest}, extended={cross_extended}",
    )

    ram_rest = ctx.part_world_position(ram)
    with ctx.pose({z_axis: 0.120}):
        ram_lowered = ctx.part_world_position(ram)
    ctx.check(
        "Z ram descends vertically",
        ram_rest is not None
        and ram_lowered is not None
        and ram_lowered[2] < ram_rest[2] - 0.11
        and abs(ram_lowered[0] - ram_rest[0]) < 1e-6
        and abs(ram_lowered[1] - ram_rest[1]) < 1e-6,
        details=f"rest={ram_rest}, lowered={ram_lowered}",
    )

    for x_pose in (0.0, 0.560):
        with ctx.pose({x_axis: x_pose}):
            ctx.expect_contact(
                bridge,
                base,
                elem_a="x_bearing_0",
                elem_b="base_rail_0",
                name=f"lower X bearing rests on rail at {x_pose:.2f} m",
            )
            ctx.expect_contact(
                bridge,
                base,
                elem_a="x_bearing_1",
                elem_b="base_rail_1",
                name=f"upper X bearing rests on rail at {x_pose:.2f} m",
            )
            ctx.expect_overlap(
                bridge,
                base,
                axes="xy",
                elem_a="x_bearing_0",
                elem_b="base_rail_0",
                min_overlap=0.030,
                name=f"lower X bearing remains captured at {x_pose:.2f} m",
            )
            ctx.expect_overlap(
                bridge,
                base,
                axes="xy",
                elem_a="x_bearing_1",
                elem_b="base_rail_1",
                min_overlap=0.030,
                name=f"upper X bearing remains captured at {x_pose:.2f} m",
            )

    for y_pose in (0.0, 0.270):
        with ctx.pose({y_axis: y_pose}):
            ctx.expect_contact(
                cross,
                bridge,
                elem_a="y_bearing_0",
                elem_b="y_rail_0",
                name=f"first Y bearing rests on rail at {y_pose:.2f} m",
            )
            ctx.expect_contact(
                cross,
                bridge,
                elem_a="y_bearing_1",
                elem_b="y_rail_1",
                name=f"second Y bearing rests on rail at {y_pose:.2f} m",
            )
            ctx.expect_overlap(
                cross,
                bridge,
                axes="xy",
                elem_a="y_bearing_0",
                elem_b="y_rail_0",
                min_overlap=0.018,
                name=f"first Y bearing remains captured at {y_pose:.2f} m",
            )
            ctx.expect_overlap(
                cross,
                bridge,
                axes="xy",
                elem_a="y_bearing_1",
                elem_b="y_rail_1",
                min_overlap=0.018,
                name=f"second Y bearing remains captured at {y_pose:.2f} m",
            )

    for z_pose in (0.0, 0.120):
        with ctx.pose({z_axis: z_pose}):
            ctx.expect_gap(
                ram,
                cross,
                axis="x",
                positive_elem="ram_body",
                negative_elem="left_guide_face",
                max_gap=0.001,
                max_penetration=0.000001,
                name=f"left Z guide face remains clear at {z_pose:.2f} m",
            )
            ctx.expect_gap(
                cross,
                ram,
                axis="x",
                positive_elem="right_guide_face",
                negative_elem="ram_body",
                max_gap=0.001,
                max_penetration=0.000001,
                name=f"right Z guide face remains clear at {z_pose:.2f} m",
            )

    with ctx.pose({y_axis: 0.0, z_axis: 0.120}):
        ctx.expect_gap(
            ram,
            base,
            axis="z",
            positive_elem="mounting_flange",
            negative_elem="base_rail_0",
            min_gap=0.010,
            name="lowered flange clears the near base rail",
        )
    with ctx.pose({y_axis: 0.270, z_axis: 0.120}):
        ctx.expect_gap(
            ram,
            base,
            axis="z",
            positive_elem="mounting_flange",
            negative_elem="base_rail_1",
            min_gap=0.010,
            name="lowered flange clears the far base rail",
        )

    return ctx.report()


object_model = build_object_model()
