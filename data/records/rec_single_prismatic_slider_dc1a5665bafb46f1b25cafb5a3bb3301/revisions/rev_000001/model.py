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


BASE_LENGTH = 1.05
BASE_WIDTH = 0.22
BASE_HEIGHT = 0.028

RAIL_LENGTH = 0.92
RAIL_WIDTH = 0.052
RAIL_HEIGHT = 0.052
RAIL_CENTER_Z = BASE_HEIGHT + RAIL_HEIGHT / 2.0
RAIL_TOP_Z = BASE_HEIGHT + RAIL_HEIGHT

CARRIAGE_LENGTH = 0.20
CARRIAGE_WIDTH = 0.142
CARRIAGE_TOP_THICKNESS = 0.038
CARRIAGE_UNDERSIDE_Z = RAIL_TOP_Z + 0.012
CARRIAGE_TOP_Z = CARRIAGE_UNDERSIDE_Z + CARRIAGE_TOP_THICKNESS
CARRIAGE_SIDE_HEIGHT = CARRIAGE_UNDERSIDE_Z - (BASE_HEIGHT + 0.006)
CARRIAGE_SIDE_CENTER_Z = BASE_HEIGHT + 0.006 + CARRIAGE_SIDE_HEIGHT / 2.0
CARRIAGE_SIDE_THICKNESS = 0.018
CARRIAGE_SIDE_CENTER_Y = 0.046

BEARING_PAD_THICKNESS = CARRIAGE_SIDE_CENTER_Y - CARRIAGE_SIDE_THICKNESS / 2.0 - RAIL_WIDTH / 2.0
BEARING_PAD_WIDTH_Z = 0.040
BEARING_PAD_CENTER_Y = RAIL_WIDTH / 2.0 + BEARING_PAD_THICKNESS / 2.0

WIPER_THICKNESS = 0.012
TRAVEL = 0.28


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_tool_linear_stage")

    model.material("ground_steel", rgba=(0.62, 0.65, 0.68, 1.0))
    model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("anodized_aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material("black_polymer", rgba=(0.025, 0.025, 0.03, 1.0))
    model.material("brass_bearing", rgba=(0.86, 0.66, 0.30, 1.0))
    model.material("rubber", rgba=(0.04, 0.035, 0.03, 1.0))

    bed = model.part("bed")
    bed.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material="dark_steel",
        name="base_plate",
    )
    bed.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_CENTER_Z)),
        material="ground_steel",
        name="rail_body",
    )

    for x in (-0.34, -0.18, 0.0, 0.18, 0.34):
        bed.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(x, 0.0, RAIL_TOP_Z + 0.002)),
            material="dark_steel",
            name=f"rail_screw_{x:+.2f}",
        )

    for x in (-0.49, 0.49):
        bed.visual(
            Box((0.035, 0.160, 0.080)),
            origin=Origin(xyz=(x, 0.0, BASE_HEIGHT + 0.040)),
            material="dark_steel",
            name=f"end_stop_{'neg' if x < 0.0 else 'pos'}",
        )
        bed.visual(
            Box((0.008, 0.045, 0.026)),
            origin=Origin(xyz=(x - 0.021 if x > 0.0 else x + 0.021, 0.0, RAIL_CENTER_Z + 0.010)),
            material="rubber",
            name=f"bumper_{'neg' if x < 0.0 else 'pos'}",
        )

    for x in (-0.40, -0.20, 0.20, 0.40):
        for y in (-0.080, 0.080):
            bed.visual(
                Cylinder(radius=0.012, length=0.003),
                origin=Origin(xyz=(x, y, BASE_HEIGHT + 0.0015)),
                material="black_polymer",
                name=f"base_bolt_{x:+.2f}_{y:+.2f}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (CARRIAGE_UNDERSIDE_Z + CARRIAGE_TOP_THICKNESS / 2.0) - RAIL_CENTER_Z,
            )
        ),
        material="anodized_aluminum",
        name="top_saddle",
    )
    for sign, suffix, cheek_name in (
        (-1.0, "neg", "side_cheek_neg"),
        (1.0, "pos", "side_cheek_pos"),
    ):
        carriage.visual(
            Box((CARRIAGE_LENGTH, CARRIAGE_SIDE_THICKNESS, CARRIAGE_SIDE_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    sign * CARRIAGE_SIDE_CENTER_Y,
                    CARRIAGE_SIDE_CENTER_Z - RAIL_CENTER_Z,
                )
            ),
            material="anodized_aluminum",
            name=cheek_name,
        )
        carriage.visual(
            Box((CARRIAGE_LENGTH * 0.86, BEARING_PAD_THICKNESS, BEARING_PAD_WIDTH_Z)),
            origin=Origin(
                xyz=(
                    0.0,
                    sign * BEARING_PAD_CENTER_Y,
                    (RAIL_CENTER_Z + 0.002) - RAIL_CENTER_Z,
                )
            ),
            material="brass_bearing",
            name=f"bearing_pad_{suffix}",
        )

    for sign, suffix in ((-1.0, "neg"), (1.0, "pos")):
        x = sign * (CARRIAGE_LENGTH / 2.0 + WIPER_THICKNESS / 2.0 - 0.001)
        carriage.visual(
            Box((WIPER_THICKNESS, CARRIAGE_WIDTH * 0.92, 0.016)),
            origin=Origin(
                xyz=(
                    x,
                    0.0,
                    (CARRIAGE_UNDERSIDE_Z + 0.008) - RAIL_CENTER_Z,
                )
            ),
            material="black_polymer",
            name=f"wiper_bridge_{suffix}",
        )
        for y_sign, y_suffix in ((-1.0, "neg"), (1.0, "pos")):
            carriage.visual(
                Box((WIPER_THICKNESS, 0.018, CARRIAGE_SIDE_HEIGHT * 0.88)),
                origin=Origin(
                    xyz=(
                        x,
                        y_sign * CARRIAGE_SIDE_CENTER_Y,
                        CARRIAGE_SIDE_CENTER_Z - RAIL_CENTER_Z,
                    )
                ),
                material="black_polymer",
                name=f"wiper_side_{suffix}_{y_suffix}",
            )

    for y in (-0.040, 0.040):
        carriage.visual(
            Box((CARRIAGE_LENGTH * 0.70, 0.010, 0.003)),
            origin=Origin(
                xyz=(
                    0.0,
                    y,
                    CARRIAGE_TOP_Z + 0.0015 - RAIL_CENTER_Z,
                )
            ),
            material="black_polymer",
            name=f"table_slot_{y:+.2f}",
        )

    model.articulation(
        "bed_to_carriage",
        ArticulationType.PRISMATIC,
        parent=bed,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-TRAVEL, upper=TRAVEL, effort=1200.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("bed_to_carriage")

    ctx.expect_origin_distance(
        bed,
        carriage,
        axes="y",
        max_dist=0.001,
        name="carriage is centered laterally on the rail",
    )
    ctx.expect_gap(
        carriage,
        bed,
        axis="z",
        positive_elem="top_saddle",
        negative_elem="rail_body",
        min_gap=0.010,
        name="top saddle clears the rail crown",
    )
    ctx.expect_gap(
        carriage,
        bed,
        axis="y",
        positive_elem="side_cheek_pos",
        negative_elem="rail_body",
        min_gap=0.006,
        name="positive cheek clears rail side",
    )
    ctx.expect_gap(
        bed,
        carriage,
        axis="y",
        positive_elem="rail_body",
        negative_elem="side_cheek_neg",
        min_gap=0.006,
        name="negative cheek clears rail side",
    )
    ctx.expect_overlap(
        carriage,
        bed,
        axes="x",
        elem_a="top_saddle",
        elem_b="rail_body",
        min_overlap=0.18,
        name="carriage covers rail at home",
    )

    home_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: TRAVEL}):
        ctx.expect_overlap(
            carriage,
            bed,
            axes="x",
            elem_a="top_saddle",
            elem_b="rail_body",
            min_overlap=0.18,
            name="carriage remains on rail at positive travel",
        )
        extended_position = ctx.part_world_position(carriage)

    ctx.check(
        "positive joint travel moves carriage along +X",
        home_position is not None
        and extended_position is not None
        and extended_position[0] > home_position[0] + 0.25,
        details=f"home={home_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
