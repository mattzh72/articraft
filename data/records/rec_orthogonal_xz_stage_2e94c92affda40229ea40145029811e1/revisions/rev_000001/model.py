from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xz_transfer_axis")

    # Base: under-plate with X rails
    base = model.part("base")
    base.visual(
        Box((0.6, 0.4, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_plate",
    )
    base.visual(
        Box((0.5, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.1, 0.07)),
        name="base_rail_0",
    )
    base.visual(
        Box((0.5, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, -0.1, 0.07)),
        name="base_rail_1",
    )

    # Carriage: moves along X, carries Z rails
    carriage = model.part("carriage")
    # Sleeves for X rails (solid proxies)
    carriage.visual(
        Box((0.2, 0.04, 0.035)),
        origin=Origin(xyz=(0.0, 0.1, -0.0175)),
        name="carriage_x_sleeve_0",
    )
    carriage.visual(
        Box((0.2, 0.04, 0.035)),
        origin=Origin(xyz=(0.0, -0.1, -0.0175)),
        name="carriage_x_sleeve_1",
    )
    # Carriage base plate
    carriage.visual(
        Box((0.2, 0.3, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="carriage_base",
    )
    # Upright pillar
    carriage.visual(
        Box((0.1, 0.2, 0.4)),
        origin=Origin(xyz=(-0.05, 0.0, 0.22)),
        name="carriage_pillar",
    )
    # Z rails on the pillar
    carriage.visual(
        Box((0.02, 0.02, 0.4)),
        origin=Origin(xyz=(0.01, 0.05, 0.22)),
        name="carriage_z_rail_0",
    )
    carriage.visual(
        Box((0.02, 0.02, 0.4)),
        origin=Origin(xyz=(0.01, -0.05, 0.22)),
        name="carriage_z_rail_1",
    )

    # Pad: moves along Z
    pad = model.part("pad")
    # Sleeves for Z rails (solid proxies)
    pad.visual(
        Box((0.02, 0.04, 0.1)),
        origin=Origin(xyz=(0.0, 0.05, -0.05)),
        name="pad_sleeve_0",
    )
    pad.visual(
        Box((0.02, 0.04, 0.1)),
        origin=Origin(xyz=(0.0, -0.05, -0.05)),
        name="pad_sleeve_1",
    )
    # Mount connecting sleeves to the plate
    pad.visual(
        Box((0.02, 0.14, 0.1)),
        origin=Origin(xyz=(0.02, 0.0, -0.05)),
        name="pad_mount",
    )
    # Top output pad
    pad.visual(
        Box((0.15, 0.2, 0.02)),
        origin=Origin(xyz=(0.105, 0.0, -0.01)),
        name="pad_plate",
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.15, upper=0.15),
    )

    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=pad,
        origin=Origin(xyz=(0.01, 0.0, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.3, upper=0.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    pad = object_model.get_part("pad")

    # Carriage sleeves are solid proxies sliding on X rails
    ctx.allow_overlap(carriage, base, reason="Carriage sleeves are solid proxies sliding on X rails", elem_a="carriage_x_sleeve_0", elem_b="base_rail_0")
    ctx.allow_overlap(carriage, base, reason="Carriage sleeves are solid proxies sliding on X rails", elem_a="carriage_x_sleeve_1", elem_b="base_rail_1")

    # Pad sleeves are solid proxies sliding on Z rails
    ctx.allow_overlap(pad, carriage, reason="Pad sleeves are solid proxies sliding on Z rails", elem_a="pad_sleeve_0", elem_b="carriage_z_rail_0")
    ctx.allow_overlap(pad, carriage, reason="Pad sleeves are solid proxies sliding on Z rails", elem_a="pad_sleeve_1", elem_b="carriage_z_rail_1")

    # X sleeves stay centered on the X rails
    ctx.expect_within(base, carriage, axes="y", inner_elem="base_rail_0", outer_elem="carriage_x_sleeve_0", margin=0.0)
    ctx.expect_within(base, carriage, axes="y", inner_elem="base_rail_1", outer_elem="carriage_x_sleeve_1", margin=0.0)

    # Z sleeves stay centered on the Z rails
    ctx.expect_within(carriage, pad, axes="xy", inner_elem="carriage_z_rail_0", outer_elem="pad_sleeve_0", margin=0.0)
    ctx.expect_within(carriage, pad, axes="xy", inner_elem="carriage_z_rail_1", outer_elem="pad_sleeve_1", margin=0.0)

    # X and Z sleeves retain overlap with their rails at rest
    ctx.expect_overlap(carriage, base, axes="x", elem_a="carriage_x_sleeve_0", elem_b="base_rail_0", min_overlap=0.1)
    ctx.expect_overlap(pad, carriage, axes="z", elem_a="pad_sleeve_0", elem_b="carriage_z_rail_0", min_overlap=0.05)

    x_axis = object_model.get_articulation("x_axis")
    z_axis = object_model.get_articulation("z_axis")

    with ctx.pose({x_axis: 0.15}):
        ctx.expect_overlap(carriage, base, axes="x", elem_a="carriage_x_sleeve_0", elem_b="base_rail_0", min_overlap=0.05)

    with ctx.pose({z_axis: -0.3}):
        ctx.expect_overlap(pad, carriage, axes="z", elem_a="pad_sleeve_0", elem_b="carriage_z_rail_0", min_overlap=0.05)

    return ctx.report()

object_model = build_object_model()