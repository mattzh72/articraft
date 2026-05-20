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
    model = ArticulatedObject(name="low_profile_xy_positioning_module")

    anodized_black = Material("matte_black_anodized", color=(0.03, 0.035, 0.04, 1.0))
    dark_gray = Material("dark_machined_aluminum", color=(0.18, 0.19, 0.20, 1.0))
    blue_gray = Material("blue_gray_carriage", color=(0.18, 0.24, 0.32, 1.0))
    bright_steel = Material("ground_steel", color=(0.72, 0.72, 0.70, 1.0))
    screw_black = Material("blackened_fasteners", color=(0.01, 0.01, 0.012, 1.0))

    # Absolute dimensions are in meters.  This is a compact industrial XY table:
    # broad base, shallow lower X carriage, and a narrower upper Y carriage.
    plate_len = 0.64
    plate_w = 0.40
    plate_t = 0.020
    x_rail_h = 0.014
    x_rail_top = plate_t + x_rail_h
    lower_t = 0.028
    y_rail_h = 0.012
    y_rail_top = lower_t + y_rail_h

    mounting_plate = model.part("mounting_plate")
    mounting_plate.visual(
        Box((plate_len, plate_w, plate_t)),
        origin=Origin(xyz=(0.0, 0.0, plate_t / 2.0)),
        material=anodized_black,
        name="plate",
    )
    for rail_name, y in (("x_rail_0", -0.125), ("x_rail_1", 0.125)):
        mounting_plate.visual(
            Box((0.56, 0.026, x_rail_h)),
            origin=Origin(xyz=(0.0, y, plate_t + x_rail_h / 2.0)),
            material=bright_steel,
            name=rail_name,
        )
    for idx, x in enumerate((-0.306, 0.306)):
        mounting_plate.visual(
            Box((0.020, 0.32, 0.018)),
            origin=Origin(xyz=(x, 0.0, plate_t + 0.009)),
            material=dark_gray,
            name=f"x_end_stop_{idx}",
        )
    for ix, x in enumerate((-0.255, 0.255)):
        for iy, y in enumerate((-0.160, 0.160)):
            mounting_plate.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(x, y, plate_t + 0.002)),
                material=screw_black,
                name=f"mount_screw_{ix}_{iy}",
            )

    x_stage = model.part("x_stage")
    x_stage.visual(
        Box((0.36, 0.31, lower_t)),
        origin=Origin(xyz=(0.0, 0.0, lower_t / 2.0)),
        material=dark_gray,
        name="lower_saddle",
    )
    for rail_name, x in (("y_rail_0", -0.095), ("y_rail_1", 0.095)):
        x_stage.visual(
            Box((0.028, 0.25, y_rail_h)),
            origin=Origin(xyz=(x, 0.0, lower_t + y_rail_h / 2.0)),
            material=bright_steel,
            name=rail_name,
        )
    x_stage.visual(
        Box((0.22, 0.030, y_rail_h)),
        origin=Origin(xyz=(0.0, 0.0, lower_t + y_rail_h / 2.0)),
        material=dark_gray,
        name="center_bridge",
    )
    x_stage.visual(
        Box((0.28, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.158, 0.017)),
        material=screw_black,
        name="x_scale_strip",
    )
    for idx, x in enumerate((-0.145, 0.145)):
        x_stage.visual(
            Cylinder(radius=0.007, length=0.003),
            origin=Origin(xyz=(x, 0.0, lower_t + 0.0015)),
            material=screw_black,
            name=f"saddle_screw_{idx}",
        )

    y_stage = model.part("y_stage")
    y_stage.visual(
        Box((0.24, 0.18, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=blue_gray,
        name="upper_table",
    )
    y_stage.visual(
        Box((0.20, 0.12, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=dark_gray,
        name="work_top",
    )
    y_stage.visual(
        Box((0.006, 0.14, 0.006)),
        origin=Origin(xyz=(-0.123, 0.0, 0.015)),
        material=screw_black,
        name="y_scale_strip",
    )
    for ix, x in enumerate((-0.075, 0.075)):
        for iy, y in enumerate((-0.045, 0.045)):
            y_stage.visual(
                Cylinder(radius=0.007, length=0.003),
                origin=Origin(xyz=(x, y, 0.0315)),
                material=screw_black,
                name=f"table_screw_{ix}_{iy}",
            )

    model.articulation(
        "plate_to_x_stage",
        ArticulationType.PRISMATIC,
        parent=mounting_plate,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, x_rail_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=-0.10, upper=0.10),
    )
    model.articulation(
        "x_stage_to_y_stage",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, y_rail_top)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=-0.055, upper=0.055),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mounting_plate = object_model.get_part("mounting_plate")
    x_stage = object_model.get_part("x_stage")
    y_stage = object_model.get_part("y_stage")
    x_joint = object_model.get_articulation("plate_to_x_stage")
    y_joint = object_model.get_articulation("x_stage_to_y_stage")

    ctx.check(
        "two perpendicular prismatic stages",
        x_joint.articulation_type == ArticulationType.PRISMATIC
        and y_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(y_joint.axis) == (0.0, 1.0, 0.0),
        details=f"x_axis={x_joint.axis}, y_axis={y_joint.axis}",
    )

    def _dims(part):
        bounds = ctx.part_world_aabb(part)
        if bounds is None:
            return None
        lo, hi = bounds
        return (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])

    base_dims = _dims(mounting_plate)
    lower_dims = _dims(x_stage)
    upper_dims = _dims(y_stage)
    ctx.check(
        "stack narrows at each carriage level",
        base_dims is not None
        and lower_dims is not None
        and upper_dims is not None
        and base_dims[1] > lower_dims[1] > upper_dims[1],
        details=f"base={base_dims}, lower={lower_dims}, upper={upper_dims}",
    )

    ctx.expect_gap(
        x_stage,
        mounting_plate,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lower_saddle",
        negative_elem="x_rail_0",
        name="lower saddle sits on X guide rail",
    )
    ctx.expect_gap(
        y_stage,
        x_stage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="upper_table",
        negative_elem="y_rail_0",
        name="upper cross slide sits on Y guide rail",
    )
    ctx.expect_within(
        x_stage,
        mounting_plate,
        axes="xy",
        inner_elem="lower_saddle",
        outer_elem="plate",
        margin=0.0,
        name="lower saddle remains within broad mounting plate",
    )
    ctx.expect_within(
        y_stage,
        x_stage,
        axes="xy",
        inner_elem="upper_table",
        outer_elem="lower_saddle",
        margin=0.0,
        name="upper table is smaller than lower saddle",
    )

    x_rest = ctx.part_world_position(x_stage)
    with ctx.pose({x_joint: 0.08}):
        x_shifted = ctx.part_world_position(x_stage)
        ctx.expect_within(
            x_stage,
            mounting_plate,
            axes="xy",
            inner_elem="lower_saddle",
            outer_elem="plate",
            margin=0.0,
            name="X travel remains carried by base",
        )
    ctx.check(
        "X joint moves lower carriage horizontally",
        x_rest is not None
        and x_shifted is not None
        and x_shifted[0] > x_rest[0] + 0.075
        and abs(x_shifted[1] - x_rest[1]) < 1e-6
        and abs(x_shifted[2] - x_rest[2]) < 1e-6,
        details=f"rest={x_rest}, shifted={x_shifted}",
    )

    y_rest = ctx.part_world_position(y_stage)
    with ctx.pose({y_joint: 0.045}):
        y_shifted = ctx.part_world_position(y_stage)
        ctx.expect_within(
            y_stage,
            x_stage,
            axes="xy",
            inner_elem="upper_table",
            outer_elem="lower_saddle",
            margin=0.0,
            name="Y travel remains within lower carriage footprint",
        )
    ctx.check(
        "Y joint moves upper cross-slide horizontally",
        y_rest is not None
        and y_shifted is not None
        and y_shifted[1] > y_rest[1] + 0.040
        and abs(y_shifted[0] - y_rest[0]) < 1e-6
        and abs(y_shifted[2] - y_rest[2]) < 1e-6,
        details=f"rest={y_rest}, shifted={y_shifted}",
    )

    return ctx.report()


object_model = build_object_model()
