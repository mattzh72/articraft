from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_plate_xyz_transfer_unit")

    aluminum = model.material("brushed_aluminum", color=(0.72, 0.74, 0.73, 1.0))
    dark = model.material("black_oxide", color=(0.015, 0.016, 0.018, 1.0))
    rail = model.material("hardened_steel", color=(0.43, 0.46, 0.48, 1.0))
    carriage = model.material("blue_anodized_carriage", color=(0.05, 0.18, 0.55, 1.0))
    cross_color = model.material("silver_cross_slide", color=(0.62, 0.65, 0.66, 1.0))
    output_color = model.material("orange_output_plate", color=(0.9, 0.38, 0.06, 1.0))
    rubber = model.material("dark_wiper_blocks", color=(0.03, 0.03, 0.028, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.20, 0.52, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark,
        name="base_plate",
    )
    # Long X-axis guide rails mounted on the under-plate frame.
    for rail_name, y in (("x_rail_0", -0.155), ("x_rail_1", 0.155)):
        base.visual(
            Box((1.10, 0.036, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0755)),
            material=rail,
            name=rail_name,
        )
    for i, x in enumerate((-0.555, 0.555)):
        base.visual(
            Box((0.030, 0.45, 0.065)),
            origin=Origin(xyz=(x, 0.0, 0.0925)),
            material=dark,
            name=f"x_end_stop_{i}",
        )
    for i, (x, y) in enumerate(
        (
            (-0.48, -0.21),
            (-0.16, -0.21),
            (0.16, -0.21),
            (0.48, -0.21),
            (-0.48, 0.21),
            (-0.16, 0.21),
            (0.16, 0.21),
            (0.48, 0.21),
        )
    ):
        base.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(x, y, 0.0625)),
            material=aluminum,
            name=f"base_bolt_{i}",
        )

    lower = model.part("lower_carriage")
    for bearing_name, y in (("x_bearing_0", -0.155), ("x_bearing_1", 0.155)):
        lower.visual(
            Box((0.235, 0.082, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.0175)),
            material=carriage,
            name=bearing_name,
        )
    lower.visual(
        Box((0.720, 0.380, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0555)),
        material=carriage,
        name="lower_bed",
    )
    for rail_name, x in (("y_rail_0", -0.205), ("y_rail_1", 0.205)):
        lower.visual(
            Box((0.038, 0.480, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.091)),
            material=rail,
            name=rail_name,
        )
    for i, x in enumerate((-0.205, 0.205)):
        lower.visual(
            Box((0.052, 0.040, 0.040)),
            origin=Origin(xyz=(x, -0.235, 0.098)),
            material=dark,
            name=f"y_stop_low_{i}",
        )
        lower.visual(
            Box((0.052, 0.040, 0.040)),
            origin=Origin(xyz=(x, 0.235, 0.098)),
            material=dark,
            name=f"y_stop_high_{i}",
        )

    cross = model.part("cross_slide")
    for bearing_name, x in (("y_bearing_0", -0.205), ("y_bearing_1", 0.205)):
        cross.visual(
            Box((0.086, 0.145, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.0175)),
            material=cross_color,
            name=bearing_name,
        )
    cross.visual(
        Box((0.540, 0.270, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=cross_color,
        name="cross_table",
    )
    # Twin vertical guide rails fixed to the crosswise slide.
    for rail_name, foot_name, x in (("z_rail_0", "z_rail_foot_0", -0.090), ("z_rail_1", "z_rail_foot_1", 0.090)):
        cross.visual(
            Box((0.035, 0.035, 0.320)),
            origin=Origin(xyz=(x, 0.0, 0.233)),
            material=rail,
            name=rail_name,
        )
        cross.visual(
            Box((0.075, 0.070, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.0955)),
            material=dark,
            name=foot_name,
        )
    cross.visual(
        Box((0.230, 0.026, 0.050)),
        origin=Origin(xyz=(0.0, 0.035, 0.143)),
        material=dark,
        name="rear_stiffener",
    )

    vertical = model.part("vertical_slide")
    for bearing_name, wiper_name, x in (("z_bearing_0", "z_wiper_0", -0.090), ("z_bearing_1", "z_wiper_1", 0.090)):
        vertical.visual(
            Box((0.055, 0.035, 0.145)),
            origin=Origin(xyz=(x, -0.035, 0.155)),
            material=output_color,
            name=bearing_name,
        )
        vertical.visual(
            Box((0.052, 0.010, 0.110)),
            origin=Origin(xyz=(x, -0.0575, 0.155)),
            material=rubber,
            name=wiper_name,
        )
    vertical.visual(
        Box((0.260, 0.024, 0.155)),
        origin=Origin(xyz=(0.0, -0.052, 0.155)),
        material=output_color,
        name="vertical_carrier",
    )
    vertical.visual(
        Box((0.090, 0.088, 0.185)),
        origin=Origin(xyz=(0.0, -0.030, 0.255)),
        material=output_color,
        name="lift_riser",
    )
    vertical.visual(
        Box((0.400, 0.300, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.348)),
        material=output_color,
        name="output_plate",
    )
    for i, (x, y) in enumerate(((-0.145, -0.105), (0.145, -0.105), (-0.145, 0.105), (0.145, 0.105))):
        vertical.visual(
            Cylinder(radius=0.014, length=0.007),
            origin=Origin(xyz=(x, y, 0.3685)),
            material=aluminum,
            name=f"output_bolt_{i}",
        )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower,
        origin=Origin(xyz=(-0.220, 0.0, 0.093)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.45, lower=0.0, upper=0.440),
        motion_properties=MotionProperties(damping=8.0, friction=1.2),
    )
    model.articulation(
        "lower_to_cross",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=cross,
        origin=Origin(xyz=(0.0, -0.100, 0.106)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.200),
        motion_properties=MotionProperties(damping=6.0, friction=0.8),
    )
    model.articulation(
        "cross_to_vertical",
        ArticulationType.PRISMATIC,
        parent=cross,
        child=vertical,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.22, lower=0.0, upper=0.160),
        motion_properties=MotionProperties(damping=5.0, friction=0.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base")
    lower = object_model.get_part("lower_carriage")
    cross = object_model.get_part("cross_slide")
    vertical = object_model.get_part("vertical_slide")

    x_joint = object_model.get_articulation("base_to_lower")
    y_joint = object_model.get_articulation("lower_to_cross")
    z_joint = object_model.get_articulation("cross_to_vertical")

    ctx.expect_contact(
        lower,
        base,
        elem_a="x_bearing_0",
        elem_b="x_rail_0",
        contact_tol=1e-6,
        name="lower carriage rides on x rail",
    )
    ctx.expect_contact(
        cross,
        lower,
        elem_a="y_bearing_0",
        elem_b="y_rail_0",
        contact_tol=1e-6,
        name="cross slide rides on y rail",
    )
    ctx.expect_contact(
        vertical,
        cross,
        elem_a="z_bearing_0",
        elem_b="z_rail_0",
        contact_tol=1e-6,
        name="vertical slide bears on z rail",
    )

    ctx.expect_overlap(
        lower,
        base,
        axes="x",
        elem_a="x_bearing_0",
        elem_b="x_rail_0",
        min_overlap=0.20,
        name="x bearing has retained rail engagement",
    )
    ctx.expect_overlap(
        cross,
        lower,
        axes="y",
        elem_a="y_bearing_0",
        elem_b="y_rail_0",
        min_overlap=0.12,
        name="y bearing has retained rail engagement",
    )
    ctx.expect_overlap(
        vertical,
        cross,
        axes="z",
        elem_a="z_bearing_0",
        elem_b="z_rail_0",
        min_overlap=0.07,
        name="z bearing has retained rail engagement",
    )

    rest_lower = ctx.part_world_position(lower)
    rest_cross = ctx.part_world_position(cross)
    rest_vertical = ctx.part_world_position(vertical)
    with ctx.pose({x_joint: 0.440, y_joint: 0.200, z_joint: 0.160}):
        moved_lower = ctx.part_world_position(lower)
        moved_cross = ctx.part_world_position(cross)
        moved_vertical = ctx.part_world_position(vertical)
        ctx.expect_overlap(
            lower,
            base,
            axes="x",
            elem_a="x_bearing_0",
            elem_b="x_rail_0",
            min_overlap=0.20,
            name="extended x carriage remains on rail",
        )
        ctx.expect_overlap(
            cross,
            lower,
            axes="y",
            elem_a="y_bearing_0",
            elem_b="y_rail_0",
            min_overlap=0.12,
            name="extended y carriage remains on rail",
        )
        ctx.expect_overlap(
            vertical,
            cross,
            axes="z",
            elem_a="z_bearing_0",
            elem_b="z_rail_0",
            min_overlap=0.07,
            name="raised z carriage remains on guide",
        )

    ctx.check(
        "stacked xyz joints move along x y z",
        rest_lower is not None
        and rest_cross is not None
        and rest_vertical is not None
        and moved_lower is not None
        and moved_cross is not None
        and moved_vertical is not None
        and moved_lower[0] > rest_lower[0] + 0.40
        and moved_cross[1] > rest_cross[1] + 0.18
        and moved_vertical[2] > rest_vertical[2] + 0.14,
        details=(
            f"rest lower/cross/vertical={rest_lower}, {rest_cross}, {rest_vertical}; "
            f"moved={moved_lower}, {moved_cross}, {moved_vertical}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
