from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


POST_X = 0.105
POST_RADIUS = 0.0105
SLEEVE_INNER_RADIUS = 0.0145
SLEEVE_OUTER_RADIUS = 0.023
SLEEVE_HEIGHT = 0.120
BEARING_BALL_RADIUS = 0.0022
BEARING_BALL_OFFSET = POST_RADIUS + BEARING_BALL_RADIUS
LOWER_CARRIAGE_Z = 0.182
LIFT_TRAVEL = 0.300


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _hollow_sleeve() -> cq.Workplane:
    outer = cq.Workplane("XY").cylinder(SLEEVE_HEIGHT, SLEEVE_OUTER_RADIUS)
    bore = cq.Workplane("XY").cylinder(SLEEVE_HEIGHT + 0.006, SLEEVE_INNER_RADIUS)
    return outer.cut(bore)


def _carriage_frame() -> cq.Workplane:
    rear_block = _cq_box((0.285, 0.050, 0.086), (0.0, 0.045, 0.000))
    central_spine = _cq_box((0.080, 0.120, 0.036), (0.0, -0.020, 0.010))
    table_riser = _cq_box((0.105, 0.070, 0.047), (0.0, -0.083, 0.044))
    front_web = _cq_box((0.165, 0.018, 0.026), (0.0, -0.055, 0.026))
    return rear_block.union(central_spine).union(table_riser).union(front_web)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clean_room_lift_axis")

    machined = model.material("satin_machined_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    polished = model.material("polished_guide_steel", rgba=(0.90, 0.93, 0.95, 1.0))
    dark = model.material("black_hard_stop_polymer", rgba=(0.02, 0.023, 0.026, 1.0))
    white = model.material("white_clean_room_cover", rgba=(0.92, 0.94, 0.91, 1.0))
    ceramic = model.material("matte_tooling_table", rgba=(0.78, 0.80, 0.78, 1.0))
    bronze = model.material("bronze_bearing_liner", rgba=(0.68, 0.48, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.155, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=machined,
        name="base_foot",
    )
    base.visual(
        Cylinder(radius=0.098, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=white,
        name="base_can",
    )
    base.visual(
        Cylinder(radius=0.101, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=dark,
        name="can_trim",
    )
    base.visual(
        Box((0.325, 0.104, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=machined,
        name="lower_plate",
    )

    for index, x in enumerate((-POST_X, POST_X)):
        base.visual(
            Cylinder(radius=POST_RADIUS, length=0.562),
            origin=Origin(xyz=(x, 0.0, 0.365)),
            material=polished,
            name=f"guide_post_{index}",
        )
        base.visual(
            Cylinder(radius=0.0185, length=0.020),
            origin=Origin(xyz=(x, 0.0, 0.112)),
            material=dark,
            name=f"bottom_stop_{index}",
        )
        base.visual(
            Cylinder(radius=0.0185, length=0.020),
            origin=Origin(xyz=(x, 0.0, 0.565)),
            material=dark,
            name=f"top_stop_{index}",
        )

    base.visual(
        Box((0.325, 0.060, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.643)),
        material=machined,
        name="top_bridge",
    )
    base.visual(
        Box((0.250, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.663)),
        material=dark,
        name="top_trim",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_frame(), "carriage_frame", tolerance=0.0006),
        material=white,
        name="carriage_frame",
    )
    for index, x in enumerate((-POST_X, POST_X)):
        carriage.visual(
            mesh_from_cadquery(_hollow_sleeve(), f"bearing_sleeve_{index}", tolerance=0.0005),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=bronze,
            name=f"bearing_{index}",
        )
        inward = 1.0 if x < 0.0 else -1.0
        ball_positions = (
            ("front", (0.0, -BEARING_BALL_OFFSET)),
            ("rear", (0.0, BEARING_BALL_OFFSET)),
            ("inner", (inward * BEARING_BALL_OFFSET, 0.0)),
            ("outer", (-inward * BEARING_BALL_OFFSET, 0.0)),
        )
        for side, (dx, dy) in ball_positions:
            for level, z in (("low", -0.038), ("high", 0.038)):
                carriage.visual(
                    Sphere(radius=BEARING_BALL_RADIUS),
                    origin=Origin(xyz=(x + dx, dy, z)),
                    material=polished,
                    name=f"ball_{index}_{side}_{level}",
                )
    carriage.visual(
        Box((0.220, 0.130, 0.022)),
        origin=Origin(xyz=(0.0, -0.105, 0.073)),
        material=ceramic,
        name="table_plate",
    )
    carriage.visual(
        Box((0.170, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.173, 0.087)),
        material=dark,
        name="front_table_trim",
    )

    model.articulation(
        "lift_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_CARRIAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=LIFT_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("lift_slide")

    for index in (0, 1):
        ctx.expect_overlap(
            carriage,
            base,
            axes="xy",
            elem_a=f"bearing_{index}",
            elem_b=f"guide_post_{index}",
            min_overlap=0.018,
            name=f"bearing {index} surrounds its guide post in plan",
        )
        ctx.expect_contact(
            carriage,
            base,
            elem_a=f"ball_{index}_front_low",
            elem_b=f"guide_post_{index}",
            contact_tol=1e-5,
            name=f"bearing {index} has rolling contact with its post",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            positive_elem=f"bearing_{index}",
            negative_elem=f"bottom_stop_{index}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"bearing {index} rests on the lower stop",
        )

    with ctx.pose({lift: LIFT_TRAVEL}):
        for index in (0, 1):
            ctx.expect_gap(
                base,
                carriage,
                axis="z",
                positive_elem=f"top_stop_{index}",
                negative_elem=f"bearing_{index}",
                min_gap=0.003,
                max_gap=0.016,
                name=f"bearing {index} clears the upper stop at full lift",
            )
            ctx.expect_contact(
                carriage,
                base,
                elem_a=f"ball_{index}_front_high",
                elem_b=f"guide_post_{index}",
                contact_tol=1e-5,
                name=f"bearing {index} remains guided at full lift",
            )
        ctx.expect_gap(
            base,
            carriage,
            axis="z",
            positive_elem="top_bridge",
            negative_elem="table_plate",
            min_gap=0.035,
            name="raised table clears the top bridge",
        )

    return ctx.report()


object_model = build_object_model()
