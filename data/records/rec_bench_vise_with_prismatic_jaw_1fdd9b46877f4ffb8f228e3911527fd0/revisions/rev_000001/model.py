from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


JOIN_EPS = 0.0008

BASE_WAY_TOP_Z = 0.060
X_SADDLE_WAY_TOP_Z = 0.036

X_TRAVEL = 0.045
Y_TRAVEL = 0.030


def _box_on_base(
    size_x: float,
    size_y: float,
    size_z: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z0: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(
        size_x,
        size_y,
        size_z,
        centered=(True, True, False),
    ).translate((x, y, z0))


def _base_body_shape() -> cq.Workplane:
    foot = _box_on_base(0.300, 0.180, 0.018)
    pedestal = _box_on_base(0.250, 0.140, 0.030 + JOIN_EPS, z0=0.018 - JOIN_EPS / 2.0)
    ways = _box_on_base(0.240, 0.160, 0.012 + JOIN_EPS, z0=0.048 - JOIN_EPS)
    center_trough = _box_on_base(0.200, 0.052, 0.020, z0=0.043)

    return foot.union(pedestal).union(ways.cut(center_trough))


def _x_boss_shape() -> cq.Workplane:
    boss = cq.Workplane("YZ", origin=(0.125, 0.0, 0.050)).circle(0.016).extrude(0.040)
    collar = cq.Workplane("YZ", origin=(0.148, 0.0, 0.050)).circle(0.010).extrude(0.010)
    return boss.union(collar)


def _x_saddle_body_shape() -> cq.Workplane:
    runner_pos = 0.048
    runner_left = _box_on_base(0.260, 0.032, 0.010, y=-runner_pos)
    runner_right = _box_on_base(0.260, 0.032, 0.010, y=runner_pos)
    bridge = _box_on_base(0.240, 0.170, 0.016, z0=0.010 - JOIN_EPS / 2.0)

    top_way_x = 0.055
    top_way_left = _box_on_base(
        0.032,
        0.150,
        0.010 + JOIN_EPS,
        x=-top_way_x,
        z0=0.026 - JOIN_EPS,
    )
    top_way_right = _box_on_base(
        0.032,
        0.150,
        0.010 + JOIN_EPS,
        x=top_way_x,
        z0=0.026 - JOIN_EPS,
    )
    center_rib = _box_on_base(0.056, 0.110, 0.007, z0=0.024)

    body = (
        runner_left.union(runner_right)
        .union(bridge)
        .union(top_way_left)
        .union(top_way_right)
        .union(center_rib)
    )

    return body


def _y_boss_shape() -> cq.Workplane:
    return cq.Workplane("XZ", origin=(0.0, 0.085, 0.028)).circle(0.014).extrude(-0.040)


def _y_carriage_body_shape() -> cq.Workplane:
    rail_x = 0.055
    runner_left = _box_on_base(0.032, 0.160, 0.010, x=-rail_x)
    runner_right = _box_on_base(0.032, 0.160, 0.010, x=rail_x)
    bridge = _box_on_base(0.180, 0.160, 0.014, z0=0.010 - JOIN_EPS / 2.0)

    vise_bed = _box_on_base(0.170, 0.110, 0.018, z0=0.024 - JOIN_EPS / 2.0)
    rear_jaw = _box_on_base(0.130, 0.018, 0.032, y=-0.035, z0=0.042 - JOIN_EPS / 2.0)
    front_jaw = _box_on_base(0.130, 0.018, 0.024, y=0.028, z0=0.042 - JOIN_EPS / 2.0)
    rear_buttress = _box_on_base(0.160, 0.018, 0.012, y=-0.052, z0=0.024)
    front_guide = _box_on_base(0.160, 0.014, 0.010, y=0.050, z0=0.024)
    screw_housing = _box_on_base(0.024, 0.034, 0.020, y=-0.002, z0=0.042 - JOIN_EPS / 2.0)

    carriage = (
        runner_left.union(runner_right)
        .union(bridge)
        .union(vise_bed)
        .union(rear_jaw)
        .union(front_jaw)
        .union(rear_buttress)
        .union(front_guide)
        .union(screw_housing)
    )

    return carriage


def _x_crank_shape() -> cq.Workplane:
    hub = cq.Workplane("YZ").circle(0.010).extrude(0.016)
    arm = cq.Workplane("XY").box(0.009, 0.056, 0.007).translate((0.0195, -0.028, 0.0))
    knob_stem = cq.Workplane("XY").circle(0.0038).extrude(0.022).translate((0.0195, -0.056, 0.0015))
    knob = cq.Workplane("XY").circle(0.006).extrude(0.024).translate((0.0195, -0.056, 0.0215))
    counterweight = cq.Workplane("XY").box(0.006, 0.014, 0.008).translate((0.018, 0.010, 0.0))
    return hub.union(arm).union(knob_stem).union(knob).union(counterweight)


def _y_crank_shape() -> cq.Workplane:
    return _x_crank_shape().rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_drill_vise")

    cast_iron = model.material("cast_iron", rgba=(0.28, 0.30, 0.32, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "base_body"),
        material=cast_iron,
        name="body",
    )
    base.visual(
        mesh_from_cadquery(_x_boss_shape(), "base_x_boss"),
        material=cast_iron,
        name="x_boss",
    )

    x_saddle = model.part("x_saddle")
    x_saddle.visual(
        mesh_from_cadquery(_x_saddle_body_shape(), "x_saddle_body"),
        material=cast_iron,
        name="body",
    )
    x_saddle.visual(
        mesh_from_cadquery(_y_boss_shape(), "x_saddle_y_boss"),
        material=cast_iron,
        name="y_boss",
    )

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        mesh_from_cadquery(_y_carriage_body_shape(), "y_carriage_body"),
        material=cast_iron,
        name="body",
    )

    x_crank = model.part("x_crank")
    x_crank.visual(
        mesh_from_cadquery(_x_crank_shape(), "x_crank"),
        material=steel,
        name="crank",
    )

    y_crank = model.part("y_crank")
    y_crank.visual(
        mesh_from_cadquery(_y_crank_shape(), "y_crank"),
        material=steel,
        name="crank",
    )

    model.articulation(
        "base_to_x_saddle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_saddle,
        origin=Origin(xyz=(0.0, 0.0, BASE_WAY_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.06,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )
    model.articulation(
        "x_saddle_to_y_carriage",
        ArticulationType.PRISMATIC,
        parent=x_saddle,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_SADDLE_WAY_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.05,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )
    model.articulation(
        "base_to_x_crank",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=x_crank,
        origin=Origin(xyz=(0.165, 0.0, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
    )
    model.articulation(
        "x_saddle_to_y_crank",
        ArticulationType.CONTINUOUS,
        parent=x_saddle,
        child=y_crank,
        origin=Origin(xyz=(0.0, 0.125, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=12.0),
    )

    return model


def _aabb_span(aabb, axis_index: int) -> float | None:
    if aabb is None:
        return None
    return aabb[1][axis_index] - aabb[0][axis_index]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    x_saddle = object_model.get_part("x_saddle")
    y_carriage = object_model.get_part("y_carriage")
    x_crank = object_model.get_part("x_crank")
    y_crank = object_model.get_part("y_crank")

    x_slide = object_model.get_articulation("base_to_x_saddle")
    y_slide = object_model.get_articulation("x_saddle_to_y_carriage")
    x_crank_joint = object_model.get_articulation("base_to_x_crank")
    y_crank_joint = object_model.get_articulation("x_saddle_to_y_crank")

    ctx.expect_gap(
        x_saddle,
        base,
        axis="z",
        positive_elem="body",
        negative_elem="body",
        max_gap=0.0005,
        max_penetration=0.0,
        name="x saddle sits on the base ways",
    )
    ctx.expect_gap(
        y_carriage,
        x_saddle,
        axis="z",
        positive_elem="body",
        negative_elem="body",
        max_gap=0.0005,
        max_penetration=0.0,
        name="y carriage sits on the top ways",
    )
    ctx.expect_overlap(
        x_saddle,
        base,
        axes="y",
        elem_a="body",
        elem_b="body",
        min_overlap=0.120,
        name="x saddle spans the base laterally",
    )
    ctx.expect_overlap(
        y_carriage,
        x_saddle,
        axes="x",
        elem_a="body",
        elem_b="body",
        min_overlap=0.120,
        name="y carriage remains captured on the upper ways",
    )

    x_rest = ctx.part_world_position(x_saddle)
    with ctx.pose({x_slide: X_TRAVEL}):
        x_upper = ctx.part_world_position(x_saddle)
        ctx.expect_overlap(
            x_saddle,
            base,
            axes="x",
            elem_a="body",
            elem_b="body",
            min_overlap=0.200,
            name="x saddle keeps substantial engagement at right travel",
        )
    with ctx.pose({x_slide: -X_TRAVEL}):
        x_lower = ctx.part_world_position(x_saddle)
        ctx.expect_overlap(
            x_saddle,
            base,
            axes="x",
            elem_a="body",
            elem_b="body",
            min_overlap=0.200,
            name="x saddle keeps substantial engagement at left travel",
        )
    ctx.check(
        "x saddle moves in +x at upper travel",
        x_rest is not None and x_upper is not None and x_upper[0] > x_rest[0] + 0.03,
        details=f"rest={x_rest}, upper={x_upper}",
    )
    ctx.check(
        "x saddle moves in -x at lower travel",
        x_rest is not None and x_lower is not None and x_lower[0] < x_rest[0] - 0.03,
        details=f"rest={x_rest}, lower={x_lower}",
    )

    y_rest = ctx.part_world_position(y_carriage)
    with ctx.pose({y_slide: Y_TRAVEL}):
        y_upper = ctx.part_world_position(y_carriage)
        ctx.expect_overlap(
            y_carriage,
            x_saddle,
            axes="y",
            elem_a="body",
            elem_b="body",
            min_overlap=0.120,
            name="y carriage keeps substantial engagement at front travel",
        )
    with ctx.pose({y_slide: -Y_TRAVEL}):
        y_lower = ctx.part_world_position(y_carriage)
        ctx.expect_overlap(
            y_carriage,
            x_saddle,
            axes="y",
            elem_a="body",
            elem_b="body",
            min_overlap=0.120,
            name="y carriage keeps substantial engagement at rear travel",
        )
    ctx.check(
        "y carriage moves toward the front at upper travel",
        y_rest is not None and y_upper is not None and y_upper[1] > y_rest[1] + 0.02,
        details=f"rest={y_rest}, upper={y_upper}",
    )
    ctx.check(
        "y carriage moves toward the rear at lower travel",
        y_rest is not None and y_lower is not None and y_lower[1] < y_rest[1] - 0.02,
        details=f"rest={y_rest}, lower={y_lower}",
    )

    x_crank_rest = ctx.part_world_aabb(x_crank)
    with ctx.pose({x_crank_joint: math.pi / 2.0}):
        x_crank_quarter = ctx.part_world_aabb(x_crank)
    x_crank_rest_z = _aabb_span(x_crank_rest, 2)
    x_crank_quarter_z = _aabb_span(x_crank_quarter, 2)
    ctx.check(
        "x crank visibly rotates about the screw axis",
        x_crank_rest_z is not None
        and x_crank_quarter_z is not None
        and x_crank_quarter_z > x_crank_rest_z + 0.015,
        details=f"rest_z_span={x_crank_rest_z}, quarter_z_span={x_crank_quarter_z}",
    )

    y_crank_rest = ctx.part_world_aabb(y_crank)
    with ctx.pose({y_crank_joint: math.pi / 2.0}):
        y_crank_quarter = ctx.part_world_aabb(y_crank)
    y_crank_rest_z = _aabb_span(y_crank_rest, 2)
    y_crank_quarter_z = _aabb_span(y_crank_quarter, 2)
    ctx.check(
        "y crank visibly rotates about the screw axis",
        y_crank_rest_z is not None
        and y_crank_quarter_z is not None
        and y_crank_quarter_z > y_crank_rest_z + 0.015,
        details=f"rest_z_span={y_crank_rest_z}, quarter_z_span={y_crank_quarter_z}",
    )

    return ctx.report()


object_model = build_object_model()
