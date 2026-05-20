from __future__ import annotations

import math

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
)


def _segment_length(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _segment_midpoint(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )


def _segment_rpy(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_box_member(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    width: float,
    depth: float,
    material,
    name: str,
) -> None:
    part.visual(
        Box((width, depth, _segment_length(start, end))),
        origin=Origin(xyz=_segment_midpoint(start, end), rpy=_segment_rpy(start, end)),
        material=material,
        name=name,
    )


def _add_rod(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_segment_length(start, end)),
        origin=Origin(xyz=_segment_midpoint(start, end), rpy=_segment_rpy(start, end)),
        material=material,
        name=name,
    )


def _aabb_center(bounds) -> tuple[float, float, float] | None:
    if bounds is None:
        return None
    min_corner, max_corner = bounds
    return (
        0.5 * (min_corner[0] + max_corner[0]),
        0.5 * (min_corner[1] + max_corner[1]),
        0.5 * (min_corner[2] + max_corner[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_tripod_instrument")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    survey_yellow = model.material("survey_yellow", rgba=(0.86, 0.75, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.63, 0.80, 0.88, 0.40))

    crown = model.part("crown")
    crown.visual(
        Box((0.090, 0.018, 0.016)),
        origin=Origin(xyz=(0.070, 0.0, 0.006)),
        material=graphite,
        name="top_arm_0",
    )
    lower_outer = 0.086
    lower_inner = 0.054
    lower_wall = 0.016
    lower_offset = 0.5 * (lower_inner + lower_wall)
    crown.visual(
        Box((lower_wall, lower_outer, 0.050)),
        origin=Origin(xyz=(lower_offset, 0.0, -0.024)),
        material=dark_graphite,
        name="hub_wall_x_pos",
    )
    crown.visual(
        Box((lower_wall, lower_outer, 0.050)),
        origin=Origin(xyz=(-lower_offset, 0.0, -0.024)),
        material=dark_graphite,
        name="hub_wall_x_neg",
    )
    crown.visual(
        Box((lower_inner, lower_wall, 0.050)),
        origin=Origin(xyz=(0.0, lower_offset, -0.024)),
        material=dark_graphite,
        name="hub_wall_y_pos",
    )
    crown.visual(
        Box((lower_inner, lower_wall, 0.050)),
        origin=Origin(xyz=(0.0, -lower_offset, -0.024)),
        material=dark_graphite,
        name="hub_wall_y_neg",
    )
    crown.visual(
        Box((0.010, 0.056, 0.018)),
        origin=Origin(xyz=(0.027, 0.0, 0.100)),
        material=graphite,
        name="top_cap_x_pos",
    )
    crown.visual(
        Box((0.010, 0.056, 0.018)),
        origin=Origin(xyz=(-0.027, 0.0, 0.100)),
        material=graphite,
        name="top_cap_x_neg",
    )
    crown.visual(
        Box((0.044, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.027, 0.100)),
        material=graphite,
        name="top_cap_y_pos",
    )
    crown.visual(
        Box((0.044, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.027, 0.100)),
        material=graphite,
        name="top_cap_y_neg",
    )

    sleeve_outer = 0.064
    sleeve_inner = 0.044
    sleeve_wall = 0.010
    sleeve_height = 0.180
    sleeve_center_z = 0.030
    wall_offset = 0.5 * (sleeve_inner + sleeve_wall)
    crown.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(xyz=(wall_offset, 0.0, sleeve_center_z)),
        material=graphite,
        name="sleeve_wall_x_pos",
    )
    crown.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(xyz=(-wall_offset, 0.0, sleeve_center_z)),
        material=graphite,
        name="sleeve_wall_x_neg",
    )
    crown.visual(
        Box((sleeve_inner, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, wall_offset, sleeve_center_z)),
        material=graphite,
        name="sleeve_wall_y_pos",
    )
    crown.visual(
        Box((sleeve_inner, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, -wall_offset, sleeve_center_z)),
        material=graphite,
        name="sleeve_wall_y_neg",
    )

    crown.visual(
        Box((0.050, 0.070, 0.045)),
        origin=Origin(xyz=(0.0, 0.080, -0.002)),
        material=dark_graphite,
        name="gearbox",
    )
    crown.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.0, 0.116, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crank_bushing",
    )

    hinge_radius = 0.082
    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        crown.visual(
            Box((0.062, 0.018, 0.012)),
            origin=Origin(
                xyz=(0.053 * c, 0.053 * s, -0.046),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_graphite,
            name=f"leg_arm_{index}",
        )
        if index > 0:
            crown.visual(
                Box((0.090, 0.018, 0.016)),
                origin=Origin(
                    xyz=(0.070 * c, 0.070 * s, 0.006),
                    rpy=(0.0, 0.0, angle),
                ),
                material=graphite,
                name=f"top_arm_{index}",
            )
        crown.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(hinge_radius * c, hinge_radius * s, -0.040)),
            material=dark_graphite,
            name=f"leg_post_{index}",
        )

    column = model.part("column")
    column.visual(
        Box((0.038, 0.038, 0.660)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=aluminum,
        name="mast",
    )
    column.visual(
        Box((0.006, 0.002, 0.220)),
        origin=Origin(xyz=(0.020, 0.0, -0.010)),
        material=dark_graphite,
        name="scale_strip",
    )
    column.visual(
        Box((0.082, 0.082, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.458)),
        material=graphite,
        name="top_stage",
    )
    column.visual(
        Cylinder(radius=0.021, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.478)),
        material=steel,
        name="pan_spigot",
    )

    model.articulation(
        "crown_to_column",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=0.0,
            upper=0.180,
        ),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.045, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=graphite,
        name="pan_base",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=steel,
        name="pan_bearing",
    )
    head.visual(
        Box((0.090, 0.120, 0.030)),
        origin=Origin(xyz=(0.000, 0.0, 0.055)),
        material=graphite,
        name="bridge",
    )
    head.visual(
        Box((0.034, 0.016, 0.150)),
        origin=Origin(xyz=(0.0, 0.064, 0.120)),
        material=graphite,
        name="yoke_cheek_pos",
    )
    head.visual(
        Box((0.034, 0.016, 0.150)),
        origin=Origin(xyz=(0.0, -0.064, 0.120)),
        material=graphite,
        name="yoke_cheek_neg",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, 0.064, 0.130), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bearing_cap_pos",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, -0.064, 0.130), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bearing_cap_neg",
    )

    model.articulation(
        "column_to_head",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.490)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5),
    )

    instrument = model.part("instrument")
    instrument.visual(
        Cylinder(radius=0.014, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion",
    )
    instrument.visual(
        Box((0.094, 0.082, 0.102)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=survey_yellow,
        name="body",
    )
    instrument.visual(
        Box((0.060, 0.070, 0.035)),
        origin=Origin(xyz=(0.004, 0.0, 0.060)),
        material=survey_yellow,
        name="upper_body",
    )
    instrument.visual(
        Cylinder(radius=0.019, length=0.110),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="objective",
    )
    instrument.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.130, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="hood",
    )
    instrument.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(-0.062, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="eyepiece",
    )
    instrument.visual(
        Box((0.040, 0.002, 0.050)),
        origin=Origin(xyz=(-0.002, 0.042, 0.002)),
        material=glass,
        name="window_pos",
    )
    instrument.visual(
        Box((0.040, 0.002, 0.050)),
        origin=Origin(xyz=(-0.002, -0.042, 0.002)),
        material=glass,
        name="window_neg",
    )
    instrument.visual(
        Box((0.010, 0.010, 0.040)),
        origin=Origin(xyz=(-0.012, 0.022, 0.085)),
        material=dark_graphite,
        name="handle_post_pos",
    )
    instrument.visual(
        Box((0.010, 0.010, 0.040)),
        origin=Origin(xyz=(-0.012, -0.022, 0.085)),
        material=dark_graphite,
        name="handle_post_neg",
    )
    instrument.visual(
        Box((0.050, 0.054, 0.010)),
        origin=Origin(xyz=(-0.012, 0.0, 0.105)),
        material=dark_graphite,
        name="handle_bridge",
    )

    model.articulation(
        "head_to_instrument",
        ArticulationType.REVOLUTE,
        parent=head,
        child=instrument,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=-0.45,
            upper=1.10,
        ),
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shaft",
    )
    _add_rod(
        crank,
        start=(0.0, 0.024, 0.0),
        end=(0.0, 0.036, -0.055),
        radius=0.0045,
        material=steel,
        name="arm",
    )
    crank.visual(
        Cylinder(radius=0.007, length=0.036),
        origin=Origin(xyz=(0.016, 0.036, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="knob",
    )

    model.articulation(
        "crown_to_crank",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=crank,
        origin=Origin(xyz=(0.0, 0.124, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    for index, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.009, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.020, 0.024, 0.010)),
            origin=Origin(xyz=(0.010, 0.0, -0.012)),
            material=graphite,
            name="hinge_socket",
        )
        _add_box_member(
            leg,
            start=(0.012, 0.0, -0.016),
            end=(0.090, 0.0, -0.103),
            width=0.026,
            depth=0.024,
            material=graphite,
            name="hinge_bracket",
        )
        _add_box_member(
            leg,
            start=(0.090, 0.0, -0.105),
            end=(0.214, 0.0, -0.500),
            width=0.036,
            depth=0.026,
            material=graphite,
            name="upper_shank",
        )
        _add_box_member(
            leg,
            start=(0.198, 0.0, -0.460),
            end=(0.398, 0.0, -0.930),
            width=0.028,
            depth=0.022,
            material=graphite,
            name="lower_shank",
        )
        leg.visual(
            Box((0.026, 0.020, 0.036)),
            origin=Origin(xyz=(0.285, 0.0, -0.665)),
            material=dark_graphite,
            name="clamp_band",
        )
        leg.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.398, 0.0, -0.930)),
            material=rubber,
            name="foot",
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), -0.022), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=1.2,
                lower=0.0,
                upper=math.radians(55.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    column = object_model.get_part("column")
    crank = object_model.get_part("crank")
    instrument = object_model.get_part("instrument")
    leg_0 = object_model.get_part("leg_0")
    leg_1 = object_model.get_part("leg_1")
    leg_2 = object_model.get_part("leg_2")

    column_joint = object_model.get_articulation("crown_to_column")
    pan_joint = object_model.get_articulation("column_to_head")
    tilt_joint = object_model.get_articulation("head_to_instrument")
    crank_joint = object_model.get_articulation("crown_to_crank")
    leg_joint = object_model.get_articulation("crown_to_leg_0")

    ctx.allow_overlap(
        crown,
        leg_0,
        elem_a="leg_post_0",
        elem_b="hinge_bracket",
        reason="The crown hinge post is intentionally simplified as nesting into the deployed leg bracket at the folding crown joint.",
    )
    ctx.allow_overlap(
        crown,
        leg_0,
        elem_a="leg_post_0",
        elem_b="hinge_socket",
        reason="The crown hinge post is intentionally simplified as nesting into the deployed leg bracket at the folding crown joint.",
    )
    ctx.allow_overlap(
        crown,
        leg_1,
        elem_a="leg_post_1",
        elem_b="hinge_bracket",
        reason="The crown hinge post is intentionally simplified as nesting into the deployed leg bracket at the folding crown joint.",
    )
    ctx.allow_overlap(
        crown,
        leg_1,
        elem_a="leg_post_1",
        elem_b="hinge_socket",
        reason="The crown hinge post is intentionally simplified as nesting into the deployed leg bracket at the folding crown joint.",
    )
    ctx.allow_overlap(
        crown,
        leg_2,
        elem_a="leg_post_2",
        elem_b="hinge_bracket",
        reason="The crown hinge post is intentionally simplified as nesting into the deployed leg bracket at the folding crown joint.",
    )
    ctx.allow_overlap(
        crown,
        leg_2,
        elem_a="leg_post_2",
        elem_b="hinge_socket",
        reason="The crown hinge post is intentionally simplified as nesting into the deployed leg bracket at the folding crown joint.",
    )

    ctx.expect_origin_distance(
        column,
        crown,
        axes="xy",
        max_dist=0.001,
        name="column stays centered over the crown",
    )
    ctx.expect_overlap(
        column,
        crown,
        axes="z",
        min_overlap=0.12,
        name="column remains inserted in the sleeve at rest",
    )

    rest_column_pos = ctx.part_world_position(column)
    column_upper = column_joint.motion_limits.upper if column_joint.motion_limits is not None else 0.18
    with ctx.pose({column_joint: column_upper}):
        ctx.expect_overlap(
            column,
            crown,
            axes="z",
            min_overlap=0.02,
            name="column retains insertion at maximum lift",
        )
        extended_column_pos = ctx.part_world_position(column)

    ctx.check(
        "column extends upward",
        rest_column_pos is not None
        and extended_column_pos is not None
        and extended_column_pos[2] > rest_column_pos[2] + 0.15,
        details=f"rest={rest_column_pos}, extended={extended_column_pos}",
    )

    objective_rest = _aabb_center(ctx.part_element_world_aabb(instrument, elem="objective"))
    with ctx.pose({pan_joint: math.pi / 2.0}):
        objective_panned = _aabb_center(ctx.part_element_world_aabb(instrument, elem="objective"))
    ctx.check(
        "head pans around the vertical axis",
        objective_rest is not None
        and objective_panned is not None
        and objective_rest[0] > 0.05
        and abs(objective_panned[0]) < 0.035
        and objective_panned[1] > 0.05,
        details=f"rest={objective_rest}, panned={objective_panned}",
    )

    with ctx.pose({tilt_joint: 0.75}):
        objective_tilted = _aabb_center(ctx.part_element_world_aabb(instrument, elem="objective"))
    ctx.check(
        "instrument tilts upward on the yoke",
        objective_rest is not None
        and objective_tilted is not None
        and objective_tilted[2] > objective_rest[2] + 0.05,
        details=f"rest={objective_rest}, tilted={objective_tilted}",
    )

    crank_rest = _aabb_center(ctx.part_element_world_aabb(crank, elem="knob"))
    with ctx.pose({crank_joint: math.pi / 2.0}):
        crank_turned = _aabb_center(ctx.part_element_world_aabb(crank, elem="knob"))
    ctx.check(
        "crank handle rotates around its side shaft",
        crank_rest is not None
        and crank_turned is not None
        and crank_turned[2] > crank_rest[2] + 0.03,
        details=f"rest={crank_rest}, turned={crank_turned}",
    )

    foot_rest = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot"))
    leg_upper = leg_joint.motion_limits.upper if leg_joint.motion_limits is not None else math.radians(55.0)
    with ctx.pose({leg_joint: leg_upper}):
        foot_folded = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot"))
    ctx.check(
        "tripod leg folds upward from the crown hinge",
        foot_rest is not None
        and foot_folded is not None
        and foot_folded[2] > foot_rest[2] + 0.45,
        details=f"rest={foot_rest}, folded={foot_folded}",
    )

    return ctx.report()


object_model = build_object_model()
