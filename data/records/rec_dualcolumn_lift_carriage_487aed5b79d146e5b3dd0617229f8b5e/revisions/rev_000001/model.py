from __future__ import annotations

import math

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
    model = ArticulatedObject(name="dual_column_lift_carriage")

    painted_steel = Material("painted_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    bright_steel = Material("bright_ground_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    carriage_blue = Material("blue_powder_coat", rgba=(0.05, 0.22, 0.65, 1.0))
    wear_pad = Material("black_polymer_wear_pad", rgba=(0.02, 0.02, 0.018, 1.0))
    fastener = Material("zinc_fasteners", rgba=(0.78, 0.76, 0.68, 1.0))

    column_x = 0.33
    column_radius = 0.035

    frame = model.part("frame")
    frame.visual(
        Box((0.92, 0.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_steel,
        name="base_plate",
    )
    frame.visual(
        Box((0.92, 0.32, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.53)),
        material=painted_steel,
        name="top_crosshead",
    )
    for index, x in enumerate((-column_x, column_x)):
        if index == 0:
            frame.visual(
                Cylinder(radius=column_radius, length=1.44),
                origin=Origin(xyz=(x, 0.0, 0.78)),
                material=bright_steel,
                name="column_0",
            )
        else:
            frame.visual(
                Cylinder(radius=column_radius, length=1.44),
                origin=Origin(xyz=(x, 0.0, 0.78)),
                material=bright_steel,
                name="column_1",
            )
        frame.visual(
            Cylinder(radius=0.058, length=0.045),
            origin=Origin(xyz=(x, 0.0, 0.105)),
            material=painted_steel,
            name=f"lower_clamp_{index}",
        )
        frame.visual(
            Cylinder(radius=0.058, length=0.045),
            origin=Origin(xyz=(x, 0.0, 1.455)),
            material=painted_steel,
            name=f"upper_clamp_{index}",
        )
    for index, (x, y) in enumerate(
        ((-0.36, -0.16), (-0.36, 0.16), (0.36, -0.16), (0.36, 0.16))
    ):
        frame.visual(
            Box((0.13, 0.08, 0.03)),
            origin=Origin(xyz=(x, y, -0.015)),
            material=painted_steel,
            name=f"mount_foot_{index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.535, 0.120, 0.125)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_blue,
        name="cross_plate",
    )
    carriage.visual(
        Box((0.500, 0.035, 0.195)),
        origin=Origin(xyz=(0.0, -0.087, 0.0)),
        material=carriage_blue,
        name="front_web",
    )

    # Four bars around each column make each guide block read as an actual sleeve
    # with a clear square window for the polished round guide column.
    sleeve_outer_x = 0.082
    sleeve_rail_t = 0.027
    sleeve_h = 0.245
    sleeve_half_gap = 0.052
    for side_index, x in enumerate((-column_x, column_x)):
        if side_index == 0:
            carriage.visual(
                Box((sleeve_outer_x * 2.0, sleeve_rail_t, sleeve_h)),
                origin=Origin(xyz=(x, -sleeve_half_gap - sleeve_rail_t / 2.0, 0.0)),
                material=carriage_blue,
                name="sleeve_0_front",
            )
            carriage.visual(
                Box((sleeve_outer_x * 2.0, sleeve_rail_t, sleeve_h)),
                origin=Origin(xyz=(x, sleeve_half_gap + sleeve_rail_t / 2.0, 0.0)),
                material=carriage_blue,
                name="sleeve_0_rear",
            )
        else:
            carriage.visual(
                Box((sleeve_outer_x * 2.0, sleeve_rail_t, sleeve_h)),
                origin=Origin(xyz=(x, -sleeve_half_gap - sleeve_rail_t / 2.0, 0.0)),
                material=carriage_blue,
                name="sleeve_1_front",
            )
            carriage.visual(
                Box((sleeve_outer_x * 2.0, sleeve_rail_t, sleeve_h)),
                origin=Origin(xyz=(x, sleeve_half_gap + sleeve_rail_t / 2.0, 0.0)),
                material=carriage_blue,
                name="sleeve_1_rear",
            )
        for rail_index, rail_sign in enumerate((-1.0, 1.0)):
            carriage.visual(
                Box((sleeve_rail_t, 0.132, sleeve_h)),
                origin=Origin(
                    xyz=(
                        x + rail_sign * (sleeve_half_gap + sleeve_rail_t / 2.0),
                        0.0,
                        0.0,
                    )
                ),
                material=carriage_blue,
                name=f"sleeve_{side_index}_side_{rail_index}",
            )

        # Replace the unrealistic frictionless clearance with visible guide
        # shoes that just meet the round column and are backed by the sleeve bars.
        pad_outer = 0.0525
        pad_inner = column_radius
        pad_thick = pad_outer - pad_inner
        pad_center = (pad_outer + pad_inner) / 2.0
        for pad_index, pad_sign in enumerate((-1.0, 1.0)):
            carriage.visual(
                Box((pad_thick, 0.055, 0.205)),
                origin=Origin(xyz=(x + pad_sign * pad_center, 0.0, 0.0)),
                material=wear_pad,
                name=f"wear_x_{side_index}_{pad_index}",
            )
            carriage.visual(
                Box((0.055, pad_thick, 0.205)),
                origin=Origin(xyz=(x, pad_sign * pad_center, 0.0)),
                material=wear_pad,
                name=f"wear_y_{side_index}_{pad_index}",
            )

        for bolt_index, (bx, bz) in enumerate(
            ((-0.042, -0.070), (0.042, -0.070), (-0.042, 0.070), (0.042, 0.070))
        ):
            carriage.visual(
                Cylinder(radius=0.012, length=0.012),
                origin=Origin(
                    xyz=(x + bx, -0.084, bz),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=fastener,
                name=f"bolt_{side_index}_{bolt_index}",
            )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.75, effort=800.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("carriage_slide")

    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="sleeve_0_front",
        elem_b="column_0",
        min_overlap=0.20,
        name="lower carriage sleeve surrounds a guide column",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="x",
        elem_a="cross_plate",
        elem_b="top_crosshead",
        min_overlap=0.45,
        name="carriage width matches the crosshead span",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.75}):
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="sleeve_1_rear",
            elem_b="column_1",
            min_overlap=0.20,
            name="raised carriage remains captured on guide column",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "prismatic carriage travels upward",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.70,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
