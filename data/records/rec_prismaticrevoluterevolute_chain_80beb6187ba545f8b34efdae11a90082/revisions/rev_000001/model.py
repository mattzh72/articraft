from __future__ import annotations

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
    model = ArticulatedObject(name="mast_slide_side_arm")

    painted = model.material("warm_grey_paint", color=(0.55, 0.58, 0.57, 1.0))
    rail_steel = model.material("ground_steel", color=(0.78, 0.80, 0.78, 1.0))
    dark_steel = model.material("dark_oxide_steel", color=(0.12, 0.13, 0.14, 1.0))
    black = model.material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))
    blue = model.material("blue_arm_paint", color=(0.08, 0.22, 0.42, 1.0))
    orange = model.material("orange_forearm_paint", color=(0.88, 0.36, 0.08, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.42, 0.32, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_steel,
        name="floor_plate",
    )
    mast.visual(
        Box((0.12, 0.16, 1.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.69)),
        material=painted,
        name="upright_tube",
    )
    mast.visual(
        Box((0.030, 0.022, 0.96)),
        origin=Origin(xyz=(0.075, 0.065, 0.76)),
        material=rail_steel,
        name="rail_0",
    )
    mast.visual(
        Box((0.030, 0.022, 0.96)),
        origin=Origin(xyz=(0.075, -0.065, 0.76)),
        material=rail_steel,
        name="rail_1",
    )
    mast.visual(
        Box((0.15, 0.22, 0.035)),
        origin=Origin(xyz=(0.045, 0.0, 0.235)),
        material=dark_steel,
        name="lower_rail_tie",
    )
    mast.visual(
        Box((0.15, 0.22, 0.035)),
        origin=Origin(xyz=(0.045, 0.0, 1.285)),
        material=dark_steel,
        name="upper_rail_tie",
    )
    mast.visual(
        Box((0.055, 0.16, 0.040)),
        origin=Origin(xyz=(0.0875, 0.0, 0.350)),
        material=black,
        name="lower_stop",
    )
    mast.visual(
        Box((0.055, 0.16, 0.040)),
        origin=Origin(xyz=(0.0875, 0.0, 1.250)),
        material=black,
        name="top_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.050, 0.190, 0.280)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
        material=dark_steel,
        name="slide_plate",
    )
    carriage.visual(
        Box((0.030, 0.044, 0.090)),
        origin=Origin(xyz=(-0.075, 0.065, -0.075)),
        material=rail_steel,
        name="guide_shoe_lower_0",
    )
    carriage.visual(
        Box((0.030, 0.044, 0.090)),
        origin=Origin(xyz=(-0.075, -0.065, -0.075)),
        material=rail_steel,
        name="guide_shoe_lower_1",
    )
    carriage.visual(
        Box((0.030, 0.044, 0.090)),
        origin=Origin(xyz=(-0.075, 0.065, 0.075)),
        material=rail_steel,
        name="guide_shoe_upper_0",
    )
    carriage.visual(
        Box((0.030, 0.044, 0.090)),
        origin=Origin(xyz=(-0.075, -0.065, 0.075)),
        material=rail_steel,
        name="guide_shoe_upper_1",
    )
    carriage.visual(
        Box((0.050, 0.065, 0.030)),
        origin=Origin(xyz=(-0.085, 0.0, -0.150)),
        material=black,
        name="bottom_striker",
    )
    carriage.visual(
        Box((0.050, 0.065, 0.030)),
        origin=Origin(xyz=(-0.085, 0.0, 0.150)),
        material=black,
        name="top_striker",
    )
    carriage.visual(
        Box((0.160, 0.195, 0.040)),
        origin=Origin(xyz=(-0.010, 0.0, 0.020)),
        material=dark_steel,
        name="bearing_shelf",
    )
    for y, name in ((0.086, "root_cheek_0"), (-0.086, "root_cheek_1")):
        carriage.visual(
            Box((0.085, 0.014, 0.150)),
            origin=Origin(xyz=(0.0, y, 0.075)),
            material=dark_steel,
            name=name,
        )
    carriage.visual(
        Cylinder(radius=0.070, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=rail_steel,
        name="shoulder_bearing",
    )

    upper_link = model.part("upper_link")
    upper_length = 0.36
    for y, name in ((0.030, "upper_bar_0"), (-0.030, "upper_bar_1")):
        upper_link.visual(
            Box((0.285, 0.020, 0.038)),
            origin=Origin(xyz=(upper_length / 2.0, y, 0.180)),
            material=blue,
            name=name,
        )
    upper_link.visual(
        Cylinder(radius=0.058, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=rail_steel,
        name="shoulder_collar",
    )
    upper_link.visual(
        Cylinder(radius=0.052, length=0.060),
        origin=Origin(xyz=(upper_length, 0.0, 0.180)),
        material=rail_steel,
        name="elbow_collar",
    )
    upper_link.visual(
        Box((0.055, 0.132, 0.030)),
        origin=Origin(xyz=(upper_length, 0.0, 0.142)),
        material=dark_steel,
        name="elbow_clevis_base",
    )
    for y, name in ((0.058, "elbow_cheek_0"), (-0.058, "elbow_cheek_1")):
        upper_link.visual(
            Box((0.065, 0.012, 0.078)),
            origin=Origin(xyz=(upper_length, y, 0.168)),
            material=dark_steel,
            name=name,
        )

    forearm = model.part("forearm")
    forearm_length = 0.32
    for y, name in ((0.026, "forearm_bar_0"), (-0.026, "forearm_bar_1")):
        forearm.visual(
            Box((0.300, 0.018, 0.034)),
            origin=Origin(xyz=(forearm_length / 2.0, y, 0.240)),
            material=orange,
            name=name,
        )
    forearm.visual(
        Cylinder(radius=0.046, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=rail_steel,
        name="forearm_elbow_collar",
    )
    forearm.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(forearm_length, 0.0, 0.240)),
        material=rail_steel,
        name="wrist_collar",
    )
    forearm.visual(
        Box((0.070, 0.070, 0.032)),
        origin=Origin(xyz=(forearm_length + 0.020, 0.0, 0.240)),
        material=dark_steel,
        name="tool_pad",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.180, 0.0, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.500),
    )
    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.2, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(upper_length, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-1.20, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    slide = object_model.get_articulation("mast_slide")
    root_pivot = object_model.get_articulation("root_pivot")
    elbow_pivot = object_model.get_articulation("elbow_pivot")

    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="guide_shoe_lower_0",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower guide shoe rides on rail face",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="guide_shoe_upper_1",
        negative_elem="rail_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper guide shoe rides on opposite rail face",
    )
    ctx.expect_gap(
        upper_link,
        carriage,
        axis="z",
        positive_elem="shoulder_collar",
        negative_elem="shoulder_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="root bearing stack is seated",
    )
    ctx.expect_gap(
        forearm,
        upper_link,
        axis="z",
        positive_elem="forearm_elbow_collar",
        negative_elem="elbow_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="elbow bearing stack is seated",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="z",
        positive_elem="bottom_striker",
        negative_elem="lower_stop",
        min_gap=0.005,
        max_gap=0.030,
        name="lower slide stop has small bumper clearance",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.500}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem="top_stop",
            negative_elem="top_striker",
            min_gap=0.005,
            max_gap=0.030,
            name="upper slide stop has small bumper clearance",
        )

    ctx.check(
        "vertical carriage moves upward on mast",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.49,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({slide: 0.500, root_pivot: 0.95, elbow_pivot: 1.20}):
        ctx.expect_gap(
            upper_link,
            mast,
            axis="x",
            positive_elem="shoulder_collar",
            negative_elem="rail_0",
            min_gap=0.025,
            name="raised shoulder clears mast rail at root limit",
        )
        ctx.expect_gap(
            forearm,
            mast,
            axis="x",
            positive_elem="wrist_collar",
            negative_elem="rail_0",
            min_gap=0.050,
            name="folded forearm clears mast rail at arm limit",
        )

    return ctx.report()


object_model = build_object_model()
