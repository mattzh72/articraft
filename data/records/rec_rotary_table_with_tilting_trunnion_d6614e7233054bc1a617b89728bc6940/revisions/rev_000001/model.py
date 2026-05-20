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
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_trunnion_stage")

    cast_iron = model.material("dark_cast_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.02, 0.025, 0.03, 1.0))
    blue_steel = model.material("blue_steel", rgba=(0.18, 0.34, 0.58, 1.0))
    brass = model.material("brass_index", rgba=(0.80, 0.62, 0.28, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.35, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.18, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=ground_steel,
        name="fixed_bearing_race",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=black_oxide,
        name="center_spigot",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.315, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=ground_steel,
        name="rotary_platter",
    )
    platter.visual(
        Cylinder(radius=0.205, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=cast_iron,
        name="raised_turntable",
    )
    platter.visual(
        Cylinder(radius=0.322, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=brass,
        name="index_ring",
    )

    side_supports = TrunnionYokeGeometry(
        (0.600, 0.155, 0.168),
        span_width=0.420,
        trunnion_diameter=0.070,
        trunnion_center_z=0.118,
        base_thickness=0.030,
        corner_radius=0.010,
        center=False,
    )
    platter.visual(
        mesh_from_geometry(side_supports, "side_supports"),
        origin=Origin(xyz=(0.0, 0.0, 0.0345)),
        material=cast_iron,
        name="side_supports",
    )

    work_table = model.part("work_table")
    work_table.visual(
        Cylinder(radius=0.023, length=0.568),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="trunnion_shaft",
    )
    work_table.visual(
        Box((0.335, 0.070, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=ground_steel,
        name="center_saddle",
    )
    work_table.visual(
        Box((0.360, 0.210, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=ground_steel,
        name="work_table_plate",
    )
    work_table.visual(
        Cylinder(radius=0.035, length=0.040),
        origin=Origin(xyz=(-0.190, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue_steel,
        name="end_hub_0",
    )
    work_table.visual(
        Cylinder(radius=0.035, length=0.040),
        origin=Origin(xyz=(0.190, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue_steel,
        name="end_hub_1",
    )
    for y in (-0.065, 0.0, 0.065):
        work_table.visual(
            Box((0.315, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.073)),
            material=black_oxide,
            name=f"t_slot_{int(round((y + 0.065) / 0.065))}",
        )

    model.articulation(
        "base_to_platter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "platter_to_work_table",
        ArticulationType.REVOLUTE,
        parent=platter,
        child=work_table,
        origin=Origin(xyz=(0.0, 0.0, 0.1525)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.6, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platter = object_model.get_part("platter")
    work_table = object_model.get_part("work_table")
    yaw = object_model.get_articulation("base_to_platter")
    tilt = object_model.get_articulation("platter_to_work_table")

    ctx.allow_overlap(
        platter,
        work_table,
        elem_a="side_supports",
        elem_b="end_hub_0",
        reason="The visible end hub is intentionally captured in the trunnion yoke bearing bore.",
    )
    ctx.allow_overlap(
        platter,
        work_table,
        elem_a="side_supports",
        elem_b="end_hub_1",
        reason="The visible end hub is intentionally captured in the trunnion yoke bearing bore.",
    )

    ctx.expect_gap(
        platter,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem="rotary_platter",
        negative_elem="fixed_bearing_race",
        name="platter rides just above fixed bearing race",
    )
    ctx.expect_overlap(
        "work_table",
        "platter",
        axes="x",
        min_overlap=0.20,
        elem_a="trunnion_shaft",
        elem_b="side_supports",
        name="horizontal shaft spans the side supports",
    )
    ctx.expect_contact(
        work_table,
        platter,
        elem_a="end_hub_0",
        elem_b="side_supports",
        name="first end hub seats in its yoke bore",
    )
    ctx.expect_contact(
        work_table,
        platter,
        elem_a="end_hub_1",
        elem_b="side_supports",
        name="second end hub seats in its yoke bore",
    )
    ctx.expect_within(
        "work_table",
        "platter",
        axes="x",
        margin=0.010,
        inner_elem="work_table_plate",
        outer_elem="side_supports",
        name="upper table fits inside the trunnion yoke span",
    )

    ctx.check(
        "platter joint is vertical revolute",
        yaw.axis == (0.0, 0.0, 1.0) and yaw.motion_limits.lower < 0.0 < yaw.motion_limits.upper,
        details=f"axis={yaw.axis}, limits={yaw.motion_limits}",
    )
    ctx.check(
        "work table joint is horizontal revolute",
        tilt.axis == (1.0, 0.0, 0.0) and tilt.motion_limits.lower < 0.0 < tilt.motion_limits.upper,
        details=f"axis={tilt.axis}, limits={tilt.motion_limits}",
    )

    rest_aabb = ctx.part_element_world_aabb(work_table, elem="work_table_plate")
    with ctx.pose({tilt: 0.55}):
        tilted_aabb = ctx.part_element_world_aabb(work_table, elem="work_table_plate")
    ctx.check(
        "upper work table visibly tilts about the hub axis",
        rest_aabb is not None
        and tilted_aabb is not None
        and (tilted_aabb[1][2] - tilted_aabb[0][2]) > (rest_aabb[1][2] - rest_aabb[0][2]) + 0.04,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
