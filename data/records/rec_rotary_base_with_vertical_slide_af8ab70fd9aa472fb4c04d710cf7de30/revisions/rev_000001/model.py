from __future__ import annotations

import math

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
    model = ArticulatedObject(name="side_wall_rotary_column")

    dark_steel = model.material("dark_steel", color=(0.08, 0.085, 0.09, 1.0))
    parkerized = model.material("parkerized_plate", color=(0.18, 0.20, 0.22, 1.0))
    black = model.material("black_anodized", color=(0.015, 0.016, 0.018, 1.0))
    brushed = model.material("brushed_rail", color=(0.72, 0.72, 0.68, 1.0))
    safety_yellow = model.material("safety_yellow_face", color=(0.95, 0.68, 0.12, 1.0))
    label_white = model.material("etched_scale", color=(0.88, 0.88, 0.80, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.035, 0.36, 0.78)),
        origin=Origin(xyz=(-0.0175, 0.0, 0.25)),
        material=parkerized,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.17, 0.18, 0.035)),
        origin=Origin(xyz=(0.085, 0.0, -0.030)),
        material=parkerized,
        name="support_shelf",
    )
    side_plate.visual(
        Cylinder(radius=0.062, length=0.025),
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_socket",
    )
    for idx, y in enumerate((-0.073, 0.073)):
        side_plate.visual(
            Box((0.120, 0.014, 0.130)),
            origin=Origin(xyz=(0.060, y, -0.085)),
            material=parkerized,
            name=f"shelf_gusset_{idx}",
        )
    for idx, y in enumerate((-0.145, 0.145)):
        side_plate.visual(
            Box((0.012, 0.018, 0.68)),
            origin=Origin(xyz=(0.006, y, 0.25)),
            material=dark_steel,
            name=f"edge_rib_{idx}",
        )
    for row, z in enumerate((-0.030, 0.530)):
        for col, y in enumerate((-0.130, 0.130)):
            side_plate.visual(
                Cylinder(radius=0.014, length=0.010),
                origin=Origin(xyz=(0.005, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_steel,
                name=f"wall_bolt_{row}_{col}",
            )

    rotary_base = model.part("rotary_base")
    rotary_base.visual(
        Cylinder(radius=0.066, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0285)),
        material=black,
        name="rotating_puck",
    )
    rotary_base.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0545)),
        material=dark_steel,
        name="top_collar",
    )
    rotary_base.visual(
        Box((0.095, 0.150, 0.045)),
        origin=Origin(xyz=(0.028, 0.0, 0.082)),
        material=black,
        name="lower_clamp",
    )
    rotary_base.visual(
        Box((0.026, 0.120, 0.480)),
        origin=Origin(xyz=(-0.012, 0.0, 0.340)),
        material=black,
        name="upright_spine",
    )
    rotary_base.visual(
        Cylinder(radius=0.009, length=0.500),
        origin=Origin(xyz=(0.028, -0.045, 0.340)),
        material=brushed,
        name="guide_rail_0",
    )
    rotary_base.visual(
        Cylinder(radius=0.009, length=0.500),
        origin=Origin(xyz=(0.028, 0.045, 0.340)),
        material=brushed,
        name="guide_rail_1",
    )
    rotary_base.visual(
        Box((0.085, 0.150, 0.035)),
        origin=Origin(xyz=(0.028, 0.0, 0.598)),
        material=black,
        name="top_bridge",
    )
    rotary_base.visual(
        Box((0.004, 0.006, 0.420)),
        origin=Origin(xyz=(-0.026, 0.063, 0.350)),
        material=label_white,
        name="travel_scale",
    )
    for idx, z in enumerate((0.190, 0.270, 0.350, 0.430, 0.510)):
        rotary_base.visual(
            Box((0.005, 0.012, 0.004)),
            origin=Origin(xyz=(-0.026, 0.067, z)),
            material=label_white,
            name=f"scale_tick_{idx}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.018, 0.160, 0.190)),
        origin=Origin(xyz=(0.103, 0.0, 0.0)),
        material=safety_yellow,
        name="faceplate",
    )
    for idx, z in enumerate((-0.055, 0.055)):
        carriage.visual(
            Box((0.038, 0.132, 0.032)),
            origin=Origin(xyz=(0.076, 0.0, z)),
            material=dark_steel,
            name=f"bearing_bridge_{idx}",
        )
    carriage.visual(
        Box((0.020, 0.023, 0.150)),
        origin=Origin(xyz=(0.047, -0.045, 0.0)),
        material=dark_steel,
        name="front_bearing_0",
    )
    carriage.visual(
        Box((0.020, 0.023, 0.150)),
        origin=Origin(xyz=(0.047, 0.045, 0.0)),
        material=dark_steel,
        name="front_bearing_1",
    )
    for row, z in enumerate((-0.060, 0.060)):
        for col, y in enumerate((-0.055, 0.055)):
            carriage.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(0.115, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_steel,
                name=f"face_bolt_{row}_{col}",
            )

    model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=rotary_base,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=rotary_base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.250),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    rotary_base = object_model.get_part("rotary_base")
    carriage = object_model.get_part("carriage")
    base_swivel = object_model.get_articulation("base_swivel")
    carriage_slide = object_model.get_articulation("carriage_slide")

    ctx.expect_gap(
        rotary_base,
        side_plate,
        axis="z",
        positive_elem="rotating_puck",
        negative_elem="bearing_socket",
        max_gap=0.0005,
        max_penetration=0.000001,
        name="rotary puck seats on fixed bearing socket",
    )
    ctx.expect_gap(
        carriage,
        rotary_base,
        axis="x",
        positive_elem="front_bearing_0",
        negative_elem="guide_rail_0",
        max_gap=0.0005,
        max_penetration=0.000001,
        name="carriage shoe rides first rail",
    )
    ctx.expect_gap(
        carriage,
        rotary_base,
        axis="x",
        positive_elem="front_bearing_1",
        negative_elem="guide_rail_1",
        max_gap=0.0005,
        max_penetration=0.000001,
        name="carriage shoe rides second rail",
    )
    ctx.expect_within(
        carriage,
        rotary_base,
        axes="z",
        inner_elem="faceplate",
        outer_elem="guide_rail_0",
        margin=0.010,
        name="lowered faceplate stays inside guide travel",
    )

    lowered_pos = ctx.part_world_position(carriage)
    lowered_aabb = ctx.part_element_world_aabb(carriage, elem="faceplate")
    with ctx.pose({carriage_slide: 0.250}):
        ctx.expect_within(
            carriage,
            rotary_base,
            axes="z",
            inner_elem="faceplate",
            outer_elem="guide_rail_0",
            margin=0.010,
            name="raised faceplate remains retained on guide",
        )
        raised_pos = ctx.part_world_position(carriage)

    with ctx.pose({base_swivel: 0.75}):
        yawed_aabb = ctx.part_element_world_aabb(carriage, elem="faceplate")

    if lowered_aabb is not None and yawed_aabb is not None:
        lowered_y = (lowered_aabb[0][1] + lowered_aabb[1][1]) / 2.0
        yawed_y = (yawed_aabb[0][1] + yawed_aabb[1][1]) / 2.0
    else:
        lowered_y = yawed_y = None

    ctx.check(
        "carriage slide moves upward",
        lowered_pos is not None
        and raised_pos is not None
        and raised_pos[2] > lowered_pos[2] + 0.240,
        details=f"lowered={lowered_pos}, raised={raised_pos}",
    )
    ctx.check(
        "swivel visibly yaws faceplate",
        lowered_y is not None and yawed_y is not None and abs(yawed_y - lowered_y) > 0.050,
        details=f"lowered_y={lowered_y}, yawed_y={yawed_y}",
    )
    return ctx.report()


object_model = build_object_model()
