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
    model = ArticulatedObject(name="three_axis_cartesian_stage")

    aluminum = Material("satin_anodized_aluminum", color=(0.68, 0.70, 0.72, 1.0))
    dark_aluminum = Material("dark_anodized_aluminum", color=(0.08, 0.09, 0.10, 1.0))
    steel = Material("polished_steel_rails", color=(0.78, 0.80, 0.82, 1.0))
    black = Material("black_bearing_blocks", color=(0.015, 0.015, 0.018, 1.0))
    rubber = Material("matte_rubber_feet", color=(0.02, 0.018, 0.016, 1.0))
    brass = Material("brass_stop_screws", color=(0.92, 0.68, 0.28, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.86, 0.58, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_aluminum,
        name="base_plate",
    )
    # Two supported X-axis linear guide rails mounted to the base top surface.
    for y, name in ((-0.20, "x_rail_0"), (0.20, "x_rail_1")):
        base.visual(
            Box((0.76, 0.035, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.095)),
            material=steel,
            name=name,
        )
        for x, stop_name in ((-0.405, f"{name}_stop_0"), (0.405, f"{name}_stop_1")):
            base.visual(
                Box((0.030, 0.070, 0.060)),
                origin=Origin(xyz=(x, y, 0.110)),
                material=aluminum,
                name=stop_name,
            )
    for x in (-0.34, 0.34):
        for y in (-0.23, 0.23):
            base.visual(
                Cylinder(radius=0.035, length=0.025),
                origin=Origin(xyz=(x, y, -0.0125)),
                material=rubber,
                name=f"foot_{'p' if x > 0 else 'n'}_{'p' if y > 0 else 'n'}",
            )

    x_bridge = model.part("x_bridge")
    # Bearing shoes ride on the two X rails.  Their lower faces touch the rail tops.
    for y, name in ((-0.20, "x_bearing_0"), (0.20, "x_bearing_1")):
        x_bridge.visual(
            Box((0.125, 0.090, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.1275)),
            material=black,
            name=name,
        )
    for y, name in ((-0.245, "bridge_post_0"), (0.245, "bridge_post_1")):
        x_bridge.visual(
            Box((0.095, 0.050, 0.280)),
            origin=Origin(xyz=(0.0, y, 0.285)),
            material=aluminum,
            name=name,
        )
    x_bridge.visual(
        Box((0.130, 0.590, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        material=aluminum,
        name="bridge_beam",
    )
    # The bridge carries the Y-axis rail pair on its front face.
    for z, name in ((0.430, "y_rail_0"), (0.500, "y_rail_1")):
        x_bridge.visual(
            Box((0.025, 0.500, 0.024)),
            origin=Origin(xyz=(-0.0775, 0.0, z)),
            material=steel,
            name=name,
        )
    x_bridge.visual(
        Box((0.018, 0.510, 0.018)),
        origin=Origin(xyz=(-0.074, 0.0, 0.465)),
        material=brass,
        name="y_scale_strip",
    )

    y_carriage = model.part("y_carriage")
    # Four Y-axis trucks clamp the two bridge rails from the front side.
    for y in (-0.065, 0.065):
        for z, name in ((-0.035, "y_bearing_lower"), (0.035, "y_bearing_upper")):
            y_carriage.visual(
                Box((0.035, 0.080, 0.038)),
                origin=Origin(xyz=(-0.005, y, z)),
                material=black,
                name=f"{name}_{'p' if y > 0 else 'n'}",
            )
    y_carriage.visual(
        Box((0.030, 0.240, 0.190)),
        origin=Origin(xyz=(-0.0325, 0.0, 0.0)),
        material=aluminum,
        name="carriage_plate",
    )
    # Vertical Z-axis guide rails mounted on the carriage plate.
    for y, name in ((-0.055, "z_rail_0"), (0.055, "z_rail_1")):
        y_carriage.visual(
            Box((0.020, 0.018, 0.240)),
            origin=Origin(xyz=(-0.0575, y, 0.0)),
            material=steel,
            name=name,
        )
    y_carriage.visual(
        Box((0.012, 0.150, 0.006)),
        origin=Origin(xyz=(-0.065, 0.0, -0.115)),
        material=brass,
        name="z_lower_stop",
    )
    y_carriage.visual(
        Box((0.012, 0.150, 0.006)),
        origin=Origin(xyz=(-0.065, 0.0, 0.115)),
        material=brass,
        name="z_upper_stop",
    )

    z_slide = model.part("z_slide")
    for y in (-0.055, 0.055):
        for z, name in ((-0.040, "z_bearing_low"), (0.040, "z_bearing_high")):
            z_slide.visual(
                Box((0.025, 0.042, 0.048)),
                origin=Origin(xyz=(0.0, y, z)),
                material=black,
                name=f"{name}_{'p' if y > 0 else 'n'}",
            )
    z_slide.visual(
        Box((0.030, 0.170, 0.180)),
        origin=Origin(xyz=(-0.0275, 0.0, 0.0)),
        material=dark_aluminum,
        name="slide_plate",
    )
    z_slide.visual(
        Box((0.020, 0.130, 0.075)),
        origin=Origin(xyz=(-0.0525, 0.0, -0.075)),
        material=aluminum,
        name="tool_plate",
    )
    # Small front-facing screw heads make the tool plate read as a real bolted plate.
    for y in (-0.045, 0.045):
        for z in (-0.095, -0.055):
            z_slide.visual(
                Cylinder(radius=0.007, length=0.004),
                origin=Origin(xyz=(-0.0645, y, z), rpy=(0.0, 1.57079632679, 0.0)),
                material=steel,
                name=f"tool_screw_{'p' if y > 0 else 'n'}_{'h' if z > -0.075 else 'l'}",
            )

    model.articulation(
        "base_to_x_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_bridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=-0.22, upper=0.22),
    )
    model.articulation(
        "x_bridge_to_y_carriage",
        ArticulationType.PRISMATIC,
        parent=x_bridge,
        child=y_carriage,
        origin=Origin(xyz=(-0.1025, 0.0, 0.465)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=-0.12, upper=0.12),
    )
    model.articulation(
        "y_carriage_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_slide,
        origin=Origin(xyz=(-0.0800, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=-0.05, upper=0.06),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_bridge = object_model.get_part("x_bridge")
    y_carriage = object_model.get_part("y_carriage")
    z_slide = object_model.get_part("z_slide")
    x_joint = object_model.get_articulation("base_to_x_bridge")
    y_joint = object_model.get_articulation("x_bridge_to_y_carriage")
    z_joint = object_model.get_articulation("y_carriage_to_z_slide")

    ctx.check(
        "three orthogonal prismatic joints",
        x_joint.articulation_type == ArticulationType.PRISMATIC
        and y_joint.articulation_type == ArticulationType.PRISMATIC
        and z_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(y_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(z_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axes={x_joint.axis}, {y_joint.axis}, {z_joint.axis}",
    )

    ctx.expect_contact(
        x_bridge,
        base,
        elem_a="x_bearing_0",
        elem_b="x_rail_0",
        name="x bridge is supported on one x rail",
    )
    ctx.expect_contact(
        x_bridge,
        base,
        elem_a="x_bearing_1",
        elem_b="x_rail_1",
        name="x bridge is supported on second x rail",
    )
    ctx.expect_contact(
        y_carriage,
        x_bridge,
        elem_a="y_bearing_lower_n",
        elem_b="y_rail_0",
        name="y carriage lower truck contacts y rail",
    )
    ctx.expect_contact(
        y_carriage,
        x_bridge,
        elem_a="y_bearing_upper_p",
        elem_b="y_rail_1",
        name="y carriage upper truck contacts y rail",
    )
    ctx.expect_contact(
        z_slide,
        y_carriage,
        elem_a="z_bearing_low_n",
        elem_b="z_rail_0",
        name="z slide truck contacts vertical rail",
    )
    ctx.expect_contact(
        z_slide,
        y_carriage,
        elem_a="z_bearing_high_p",
        elem_b="z_rail_1",
        name="z slide second truck contacts vertical rail",
    )

    rest_x = ctx.part_world_position(x_bridge)
    rest_y = ctx.part_world_position(y_carriage)
    rest_z = ctx.part_world_position(z_slide)
    with ctx.pose({x_joint: 0.18, y_joint: 0.10, z_joint: 0.05}):
        moved_x = ctx.part_world_position(x_bridge)
        moved_y = ctx.part_world_position(y_carriage)
        moved_z = ctx.part_world_position(z_slide)
        ctx.expect_overlap(
            x_bridge,
            base,
            axes="x",
            min_overlap=0.10,
            elem_a="x_bearing_0",
            elem_b="x_rail_0",
            name="extended x bridge remains on rail",
        )
        ctx.expect_overlap(
            y_carriage,
            x_bridge,
            axes="y",
            min_overlap=0.04,
            elem_a="y_bearing_upper_p",
            elem_b="y_rail_1",
            name="extended y carriage remains on rail",
        )
        ctx.expect_overlap(
            z_slide,
            y_carriage,
            axes="z",
            min_overlap=0.03,
            elem_a="z_bearing_high_p",
            elem_b="z_rail_1",
            name="raised z slide remains on rail",
        )

    ctx.check(
        "positive commands move along x y z",
        rest_x is not None
        and rest_y is not None
        and rest_z is not None
        and moved_x is not None
        and moved_y is not None
        and moved_z is not None
        and moved_x[0] > rest_x[0] + 0.15
        and moved_y[1] > rest_y[1] + 0.08
        and moved_z[2] > rest_z[2] + 0.04,
        details=f"rest={rest_x, rest_y, rest_z}, moved={moved_x, moved_y, moved_z}",
    )

    return ctx.report()


object_model = build_object_model()
