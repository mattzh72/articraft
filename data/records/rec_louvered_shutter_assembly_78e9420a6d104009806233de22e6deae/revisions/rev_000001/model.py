from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="plantation_shutter")

    painted_wood = Material("warm_white_painted_wood", rgba=(0.91, 0.88, 0.80, 1.0))
    raised_edge = Material("slightly_worn_edge_paint", rgba=(0.96, 0.94, 0.86, 1.0))
    brass = Material("aged_brass_pivots", rgba=(0.72, 0.52, 0.23, 1.0))

    outer_width = 0.90
    outer_height = 1.40
    frame_depth = 0.060
    side_width = 0.070
    rail_height = 0.080

    inner_width = outer_width - 2.0 * side_width
    inner_height = outer_height - 2.0 * rail_height

    frame = model.part("frame")
    frame.visual(
        Box((side_width, frame_depth, outer_height)),
        origin=Origin(xyz=(-(outer_width / 2.0 - side_width / 2.0), 0.0, 0.0)),
        material=painted_wood,
        name="side_member_0",
    )
    frame.visual(
        Box((side_width, frame_depth, outer_height)),
        origin=Origin(xyz=(outer_width / 2.0 - side_width / 2.0, 0.0, 0.0)),
        material=painted_wood,
        name="side_member_1",
    )
    frame.visual(
        Box((outer_width, frame_depth, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, outer_height / 2.0 - rail_height / 2.0)),
        material=painted_wood,
        name="top_rail",
    )
    frame.visual(
        Box((outer_width, frame_depth, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, -(outer_height / 2.0 - rail_height / 2.0))),
        material=painted_wood,
        name="bottom_rail",
    )

    # A shallow raised stop on the front face gives the frame a wood shutter
    # reveal while leaving the central opening clear for the moving blades.
    front_y = -frame_depth / 2.0 - 0.006
    trim_depth = 0.012
    frame.visual(
        Box((0.014, trim_depth, inner_height)),
        origin=Origin(xyz=(-(inner_width / 2.0 + 0.007), front_y, 0.0)),
        material=raised_edge,
        name="inner_stop_0",
    )
    frame.visual(
        Box((0.014, trim_depth, inner_height)),
        origin=Origin(xyz=(inner_width / 2.0 + 0.007, front_y, 0.0)),
        material=raised_edge,
        name="inner_stop_1",
    )
    frame.visual(
        Box((inner_width, trim_depth, 0.014)),
        origin=Origin(xyz=(0.0, front_y, inner_height / 2.0 + 0.007)),
        material=raised_edge,
        name="top_inner_stop",
    )
    frame.visual(
        Box((inner_width, trim_depth, 0.014)),
        origin=Origin(xyz=(0.0, front_y, -(inner_height / 2.0 + 0.007))),
        material=raised_edge,
        name="bottom_inner_stop",
    )

    louver_count = 7
    louver_pitch = 0.165
    louver_chord = 0.145
    louver_thickness = 0.026
    louver_length = 0.690
    pin_length = 0.040
    pin_radius = 0.012
    edge_radius = louver_thickness / 2.0
    cylinder_to_x = Origin(rpy=(0.0, pi / 2.0, 0.0))
    cylinder_to_y = Origin(rpy=(pi / 2.0, 0.0, 0.0))

    louver_zs = [
        (i - (louver_count - 1) / 2.0) * louver_pitch for i in range(louver_count)
    ]

    for i, z in enumerate(louver_zs):
        # Brass pivot heads on the front of both stiles make the individual
        # louver pivots visible without mechanically tying the blades together.
        for side, x in enumerate((-(inner_width / 2.0 + 0.010), inner_width / 2.0 + 0.010)):
            frame.visual(
                Cylinder(radius=0.016, length=0.012),
                origin=Origin(xyz=(x, -frame_depth / 2.0 - 0.006, z), rpy=cylinder_to_y.rpy),
                material=brass,
                name=f"pivot_cap_{i}_{side}",
            )

        louver = model.part(f"louver_{i}")
        louver.visual(
            Box((louver_length, louver_thickness, louver_chord - louver_thickness + 0.002)),
            material=painted_wood,
            name="blade_face",
        )
        for edge_name, edge_z in (
            ("upper_rounded_edge", louver_chord / 2.0 - edge_radius),
            ("lower_rounded_edge", -(louver_chord / 2.0 - edge_radius)),
        ):
            louver.visual(
                Cylinder(radius=edge_radius, length=louver_length),
                origin=Origin(xyz=(0.0, 0.0, edge_z), rpy=cylinder_to_x.rpy),
                material=painted_wood,
                name=edge_name,
            )
        for pin_name, pin_x in (
            ("pin_0", -(louver_length / 2.0 + pin_length / 2.0 - 0.005)),
            ("pin_1", louver_length / 2.0 + pin_length / 2.0 - 0.005),
        ):
            louver.visual(
                Cylinder(radius=pin_radius, length=pin_length),
                origin=Origin(xyz=(pin_x, 0.0, 0.0), rpy=cylinder_to_x.rpy),
                material=brass,
                name=pin_name,
            )

        model.articulation(
            f"frame_to_louver_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=louver,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.05, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    louvers = [object_model.get_part(f"louver_{i}") for i in range(7)]
    joints = [object_model.get_articulation(f"frame_to_louver_{i}") for i in range(7)]

    ctx.check(
        "seven independent louver revolutes",
        len(louvers) == 7
        and len(joints) == 7
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and len({j.name for j in joints}) == 7,
        details=f"louvers={len(louvers)}, joints={[j.name for j in joints]}",
    )

    for lower_index in range(6):
        ctx.expect_gap(
            louvers[lower_index + 1],
            louvers[lower_index],
            axis="z",
            min_gap=0.015,
            max_gap=0.030,
            name=f"small gap between louver {lower_index} and {lower_index + 1}",
        )

    ctx.expect_contact(
        louvers[3],
        frame,
        elem_a="pin_0",
        elem_b="side_member_0",
        name="center louver pivot touches side member 0",
    )
    ctx.expect_contact(
        louvers[3],
        frame,
        elem_a="pin_1",
        elem_b="side_member_1",
        name="center louver pivot touches side member 1",
    )

    rest_aabb = ctx.part_world_aabb(louvers[3])
    neighbor_rest_aabb = ctx.part_world_aabb(louvers[2])
    with ctx.pose({joints[3]: 0.75}):
        tilted_aabb = ctx.part_world_aabb(louvers[3])
        neighbor_tilt_aabb = ctx.part_world_aabb(louvers[2])

    if rest_aabb is not None and tilted_aabb is not None:
        rest_depth = rest_aabb[1][1] - rest_aabb[0][1]
        tilted_depth = tilted_aabb[1][1] - tilted_aabb[0][1]
    else:
        rest_depth = tilted_depth = None
    ctx.check(
        "one louver tilts about its long axis",
        rest_depth is not None and tilted_depth is not None and tilted_depth > rest_depth + 0.060,
        details=f"rest_depth={rest_depth}, tilted_depth={tilted_depth}",
    )

    if neighbor_rest_aabb is not None and neighbor_tilt_aabb is not None:
        before = neighbor_rest_aabb[1][1] - neighbor_rest_aabb[0][1]
        after = neighbor_tilt_aabb[1][1] - neighbor_tilt_aabb[0][1]
    else:
        before = after = None
    ctx.check(
        "tilting one louver leaves adjacent louver independent",
        before is not None and after is not None and abs(before - after) < 1.0e-6,
        details=f"neighbor_depth_before={before}, neighbor_depth_after={after}",
    )

    return ctx.report()


object_model = build_object_model()
