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
    model = ArticulatedObject(name="compact_overbed_table")

    aluminum = model.material("satin_aluminum", rgba=(0.66, 0.68, 0.68, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.06, 0.07, 0.075, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    laminate = model.material("warm_laminate", rgba=(0.74, 0.55, 0.34, 1.0))
    brake_red = model.material("brake_red", rgba=(0.78, 0.12, 0.05, 1.0))

    # Low, narrow rolling base frame.  The two side rails are tied together by
    # real cross members, including the cross member that supports the sleeve.
    base = model.part("base_frame")
    rail_z = 0.085
    side_x = 0.18
    for idx, x in enumerate((-side_x, side_x)):
        base.visual(
            Box((0.035, 0.58, 0.035)),
            origin=Origin(xyz=(x, 0.0, rail_z)),
            material=aluminum,
            name=f"side_rail_{idx}",
        )

    base.visual(
        Box((0.395, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.25, rail_z)),
        material=aluminum,
        name="front_cross_rail",
    )
    base.visual(
        Box((0.395, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.25, rail_z)),
        material=aluminum,
        name="rear_cross_rail",
    )
    base.visual(
        Box((0.395, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, rail_z)),
        material=aluminum,
        name="center_cross_rail",
    )
    base.visual(
        Box((0.115, 0.095, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.1118)),
        material=aluminum,
        name="socket_plate",
    )

    # A true square-tube outer sleeve: four connected walls leave clearance for
    # the inner sliding column instead of hiding it inside a solid proxy.
    sleeve_bottom = 0.121
    sleeve_len = 0.360
    sleeve_z = sleeve_bottom + sleeve_len / 2.0
    sleeve_outer = 0.056
    wall = 0.006
    for name, x in (("sleeve_side_0", -sleeve_outer / 2.0 + wall / 2.0),
                    ("sleeve_side_1", sleeve_outer / 2.0 - wall / 2.0)):
        base.visual(
            Box((wall, sleeve_outer, sleeve_len)),
            origin=Origin(xyz=(x, 0.0, sleeve_z)),
            material=aluminum,
            name=name,
        )
    base.visual(
        Box((sleeve_outer - 2.0 * wall, wall, sleeve_len)),
        origin=Origin(xyz=(0.0, -sleeve_outer / 2.0 + wall / 2.0, sleeve_z)),
        material=aluminum,
        name="sleeve_front_wall",
    )
    base.visual(
        Box((sleeve_outer - 2.0 * wall, wall, sleeve_len)),
        origin=Origin(xyz=(0.0, sleeve_outer / 2.0 - wall / 2.0, sleeve_z)),
        material=aluminum,
        name="sleeve_rear_wall",
    )

    # Brake pivots are small supported tabs on the side rails.  The rotating bar
    # spans between them rather than floating in front of the frame.
    brake_y = -0.170
    brake_z = 0.118
    for idx, x in enumerate((-side_x, side_x)):
        base.visual(
            Box((0.018, 0.032, 0.055)),
            origin=Origin(xyz=(x, brake_y, 0.104)),
            material=dark_plastic,
            name=f"brake_pivot_{idx}",
        )

    # Caster forks are part of the base frame and overlap the rails slightly at
    # their top edges, making every fork visibly supported by the lower rails.
    caster_positions = [
        (-side_x, -0.250),
        (side_x, -0.250),
        (-side_x, 0.250),
        (side_x, 0.250),
    ]
    for idx, (x, y) in enumerate(caster_positions):
        for side, offset in (("outer", -0.019), ("inner", 0.019)):
            base.visual(
                Box((0.006, 0.024, 0.047)),
                origin=Origin(xyz=(x + offset, y, 0.055)),
                material=dark_plastic,
                name=f"caster_fork_{idx}_{side}",
            )
        base.visual(
            Cylinder(radius=0.007, length=0.030),
            origin=Origin(xyz=(x, y, 0.075), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_plastic,
            name=f"caster_stem_{idx}",
        )

    # Sliding inner mast and its centered support head.  The top hinge sits on
    # this head directly over the column, so the tabletop is not cantilevered
    # from a hidden side arm.
    column = model.part("column")
    column.visual(
        Box((0.044, 0.044, 0.560)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=aluminum,
        name="inner_mast",
    )
    column.visual(
        Box((0.210, 0.090, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.327)),
        material=aluminum,
        name="support_head",
    )
    for idx, x in enumerate((-0.105, 0.105)):
        column.visual(
            Box((0.024, 0.060, 0.026)),
            origin=Origin(xyz=(x, 0.0, 0.360)),
            material=aluminum,
            name=f"hinge_cheek_{idx}",
        )
    column.visual(
        Cylinder(radius=0.012, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.365), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="tilt_pin",
    )

    # Rectangular laptop-sized work surface with a raised retaining lip along
    # the front edge for a laptop or notebook.
    top = model.part("top")
    top.visual(
        Cylinder(radius=0.016, length=0.176),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="hinge_barrel",
    )
    top.visual(
        Box((0.176, 0.040, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_plastic,
        name="hinge_saddle",
    )
    top.visual(
        Box((0.620, 0.380, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=laminate,
        name="top_panel",
    )
    top.visual(
        Box((0.620, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.199, 0.060)),
        material=dark_plastic,
        name="front_lip",
    )

    brake_bar = model.part("brake_bar")
    brake_bar.visual(
        Cylinder(radius=0.011, length=0.342),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brake_red,
        name="brake_tube",
    )
    brake_bar.visual(
        Box((0.160, 0.035, 0.014)),
        origin=Origin(xyz=(0.0, -0.020, 0.002)),
        material=brake_red,
        name="foot_pad",
    )

    for idx, (x, y) in enumerate(caster_positions):
        caster = model.part(f"caster_{idx}")
        caster.visual(
            Cylinder(radius=0.032, length=0.032),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="tire",
        )
        caster.visual(
            Cylinder(radius=0.017, length=0.030),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name="hub",
        )
        model.articulation(
            f"frame_to_caster_{idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x, y, 0.035)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        )

    sleeve_top = sleeve_bottom + sleeve_len
    model.articulation(
        "frame_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, sleeve_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.160),
    )
    model.articulation(
        "column_to_top",
        ArticulationType.REVOLUTE,
        parent=column,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.30, upper=0.30),
    )
    model.articulation(
        "frame_to_brake_bar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=brake_bar,
        origin=Origin(xyz=(0.0, brake_y, brake_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.45, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    column = object_model.get_part("column")
    top = object_model.get_part("top")
    brake = object_model.get_part("brake_bar")
    column_slide = object_model.get_articulation("frame_to_column")
    top_tilt = object_model.get_articulation("column_to_top")
    brake_pivot = object_model.get_articulation("frame_to_brake_bar")

    ctx.allow_overlap(
        column,
        top,
        elem_a="tilt_pin",
        elem_b="hinge_barrel",
        reason="The top hinge barrel is intentionally modeled around the support-head pin as a captured rotating sleeve.",
    )
    ctx.expect_overlap(
        column,
        top,
        axes="x",
        elem_a="tilt_pin",
        elem_b="hinge_barrel",
        min_overlap=0.160,
        name="tilt pin runs through the hinge barrel",
    )
    ctx.expect_contact(
        column,
        top,
        elem_a="tilt_pin",
        elem_b="hinge_barrel",
        name="hinge barrel is seated on the tilt pin",
    )

    ctx.expect_origin_distance(
        top,
        column,
        axes="xy",
        max_dist=0.001,
        name="top hinge is centered over the column",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="sleeve_front_wall",
        min_overlap=0.150,
        name="collapsed column remains inside sleeve",
    )
    ctx.expect_overlap(
        brake,
        base,
        axes="x",
        elem_a="brake_tube",
        elem_b="front_cross_rail",
        min_overlap=0.320,
        name="brake bar spans the lower frame width",
    )

    rest_pos = ctx.part_world_position(column)
    with ctx.pose({column_slide: 0.160}):
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="sleeve_front_wall",
            min_overlap=0.060,
            name="extended column retains sleeve insertion",
        )
        raised_pos = ctx.part_world_position(column)
    ctx.check(
        "column slide raises the tabletop support",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.150,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    top_rest_aabb = ctx.part_element_world_aabb(top, elem="front_lip")
    with ctx.pose({top_tilt: 0.30}):
        top_tilted_aabb = ctx.part_element_world_aabb(top, elem="front_lip")
    if top_rest_aabb is not None and top_tilted_aabb is not None:
        rest_front_z = (top_rest_aabb[0][2] + top_rest_aabb[1][2]) / 2.0
        tilted_front_z = (top_tilted_aabb[0][2] + top_tilted_aabb[1][2]) / 2.0
        moved = abs(tilted_front_z - rest_front_z) > 0.030
    else:
        moved = False
        rest_front_z = tilted_front_z = None
    ctx.check(
        "top tilt moves the retaining lip",
        moved,
        details=f"rest_z={rest_front_z}, tilted_z={tilted_front_z}",
    )

    brake_rest_aabb = ctx.part_element_world_aabb(brake, elem="foot_pad")
    with ctx.pose({brake_pivot: -0.35}):
        brake_rotated_aabb = ctx.part_element_world_aabb(brake, elem="foot_pad")
    if brake_rest_aabb is not None and brake_rotated_aabb is not None:
        rest_pad_z = (brake_rest_aabb[0][2] + brake_rest_aabb[1][2]) / 2.0
        rotated_pad_z = (brake_rotated_aabb[0][2] + brake_rotated_aabb[1][2]) / 2.0
        brake_moved = abs(rotated_pad_z - rest_pad_z) > 0.006
    else:
        brake_moved = False
        rest_pad_z = rotated_pad_z = None
    ctx.check(
        "brake bar pivots on its axle",
        brake_moved,
        details=f"rest_z={rest_pad_z}, rotated_z={rotated_pad_z}",
    )

    return ctx.report()


object_model = build_object_model()
