from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehearsal_music_stand")

    cast_black = model.material("cast_black", rgba=(0.13, 0.13, 0.14, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.24, 0.26, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    plate_thickness = 0.012
    socket_height = 0.022
    outer_tube_radius = 0.0165
    outer_tube_length = 0.560
    outer_tube_bottom = plate_thickness + socket_height
    outer_tube_top = outer_tube_bottom + outer_tube_length
    base_plate_shape = (
        cq.Workplane("XY")
        .rect(0.360, 0.280)
        .extrude(plate_thickness * 0.5, both=True)
        .edges("|Z")
        .fillet(0.028)
        .edges(">Z")
        .chamfer(0.0015)
    )

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_plate_shape, "music_stand_base_plate"),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness * 0.5)),
        material=cast_black,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.038, length=socket_height),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness + socket_height * 0.5)),
        material=cast_black,
        name="socket_base",
    )
    base.visual(
        Cylinder(radius=outer_tube_radius, length=outer_tube_length),
        origin=Origin(xyz=(0.0, 0.0, outer_tube_bottom + outer_tube_length * 0.5)),
        material=satin_black,
        name="outer_tube",
    )
    base.visual(
        Cylinder(radius=0.0245, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, outer_tube_top - 0.050)),
        material=cast_black,
        name="height_collar",
    )
    base.visual(
        Box((0.018, 0.044, 0.024)),
        origin=Origin(xyz=(0.020, 0.0, outer_tube_top - 0.050)),
        material=cast_black,
        name="collar_lug",
    )
    base.visual(
        Cylinder(radius=0.0035, length=0.026),
        origin=Origin(
            xyz=(0.032, 0.0, outer_tube_top - 0.050),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=graphite,
        name="collar_stud",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(
            xyz=(0.053, 0.0, outer_tube_top - 0.050),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=knob_black,
        name="collar_knob",
    )

    inner_tube_radius = 0.0135
    inner_tube_length = 0.800
    head_base_z = 0.388
    yoke_height = 0.068
    yoke_width = 0.092
    yoke_span = 0.050
    trunnion_diameter = 0.010

    upper_post = model.part("upper_post")
    upper_post.visual(
        Cylinder(radius=inner_tube_radius, length=inner_tube_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=graphite,
        name="inner_tube",
    )
    upper_post.visual(
        Cylinder(radius=0.019, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, head_base_z - 0.003)),
        material=satin_black,
        name="head_collar",
    )
    upper_post.visual(
        Box((0.042, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, head_base_z + 0.012)),
        material=satin_black,
        name="head_block",
    )
    upper_post.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (yoke_width, 0.036, yoke_height),
                span_width=yoke_span,
                trunnion_diameter=trunnion_diameter,
                trunnion_center_z=0.050,
                base_thickness=0.016,
                corner_radius=0.004,
                center=False,
            ),
            "music_stand_tilt_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, head_base_z)),
        material=satin_black,
        name="tilt_yoke",
    )
    upper_post.visual(
        Cylinder(radius=0.006, length=0.007),
        origin=Origin(
            xyz=(-0.049, 0.0, head_base_z + 0.050),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=graphite,
        name="pivot_cap",
    )

    post_slide = model.articulation(
        "base_to_upper_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_post,
        origin=Origin(xyz=(0.0, 0.0, outer_tube_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=0.220,
        ),
    )

    desk = model.part("desk")
    desk_rest_angle = 0.18
    desk_panel = PerforatedPanelGeometry(
        (0.460, 0.330),
        0.0014,
        hole_diameter=0.010,
        pitch=(0.022, 0.022),
        frame=0.022,
        corner_radius=0.016,
        stagger=True,
    )
    desk.visual(
        Cylinder(radius=0.005, length=0.092),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=graphite,
        name="pivot_axle",
    )
    desk.visual(
        Box((0.018, 0.030, 0.102)),
        origin=Origin(xyz=(0.0, 0.004, 0.050)),
        material=satin_black,
        name="rear_spine",
    )
    desk.visual(
        Box((0.070, 0.170, 0.032)),
        origin=Origin(
            xyz=(0.0, 0.004, 0.112),
            rpy=(math.pi * 0.5 + desk_rest_angle, 0.0, 0.0),
        ),
        material=satin_black,
        name="center_channel",
    )
    desk.visual(
        mesh_from_geometry(desk_panel, "music_stand_desk_panel"),
        origin=Origin(
            xyz=(0.0, 0.010, 0.170),
            rpy=(math.pi * 0.5 + desk_rest_angle, 0.0, 0.0),
        ),
        material=graphite,
        name="desk_panel",
    )
    desk.visual(
        Box((0.014, 0.330, 0.016)),
        origin=Origin(
            xyz=(-0.223, 0.006, 0.170),
            rpy=(math.pi * 0.5 + desk_rest_angle, 0.0, 0.0),
        ),
        material=graphite,
        name="side_flange_0",
    )
    desk.visual(
        Box((0.014, 0.330, 0.016)),
        origin=Origin(
            xyz=(0.223, 0.006, 0.170),
            rpy=(math.pi * 0.5 + desk_rest_angle, 0.0, 0.0),
        ),
        material=graphite,
        name="side_flange_1",
    )
    desk.visual(
        Box((0.430, 0.044, 0.012)),
        origin=Origin(
            xyz=(0.0, 0.053, 0.028),
            rpy=(desk_rest_angle, 0.0, 0.0),
        ),
        material=graphite,
        name="music_shelf",
    )
    desk.visual(
        Box((0.430, 0.006, 0.018)),
        origin=Origin(
            xyz=(0.0, 0.073, 0.032),
            rpy=(desk_rest_angle, 0.0, 0.0),
        ),
        material=graphite,
        name="front_fence",
    )
    desk.visual(
        Box((0.452, 0.014, 0.014)),
        origin=Origin(
            xyz=(0.0, -0.019, 0.324),
            rpy=(math.pi * 0.5 + desk_rest_angle, 0.0, 0.0),
        ),
        material=graphite,
        name="top_return",
    )

    desk_tilt = model.articulation(
        "upper_post_to_desk",
        ArticulationType.REVOLUTE,
        parent=upper_post,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, head_base_z + 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.50,
            upper=0.90,
        ),
    )

    knob = model.part("friction_knob")
    knob.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(
            xyz=(0.006, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=graphite,
        name="knob_stub",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.018,
                body_style="cylindrical",
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                center=False,
            ),
            "music_stand_friction_knob",
        ),
        origin=Origin(
            xyz=(0.012, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=knob_black,
        name="knob_body",
    )

    model.articulation(
        "upper_post_to_friction_knob",
        ArticulationType.CONTINUOUS,
        parent=upper_post,
        child=knob,
        origin=Origin(xyz=(yoke_width * 0.5, 0.0, head_base_z + 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_post = object_model.get_part("upper_post")
    desk = object_model.get_part("desk")
    knob = object_model.get_part("friction_knob")

    post_slide = object_model.get_articulation("base_to_upper_post")
    desk_tilt = object_model.get_articulation("upper_post_to_desk")

    ctx.expect_within(
        upper_post,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_tube",
        margin=0.0,
        name="inner post stays centered in outer tube at rest",
    )
    ctx.allow_overlap(
        base,
        upper_post,
        elem_a="outer_tube",
        elem_b="inner_tube",
        reason="The lower post is intentionally simplified as a solid sleeve proxy around the telescoping inner tube.",
    )
    ctx.allow_overlap(
        base,
        upper_post,
        elem_a="height_collar",
        elem_b="inner_tube",
        reason="The height collar is modeled as a solid clamp sleeve around the telescoping inner tube path.",
    )
    ctx.expect_overlap(
        upper_post,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_tube",
        min_overlap=0.390,
        name="collapsed post remains deeply inserted in lower tube",
    )

    rest_post_pos = ctx.part_world_position(upper_post)
    slide_upper = post_slide.motion_limits.upper if post_slide.motion_limits is not None else None
    if slide_upper is not None:
        with ctx.pose({post_slide: slide_upper}):
            ctx.expect_within(
                upper_post,
                base,
                axes="xy",
                inner_elem="inner_tube",
                outer_elem="outer_tube",
                margin=0.0,
                name="extended post stays centered in outer tube",
            )
            ctx.expect_overlap(
                upper_post,
                base,
                axes="z",
                elem_a="inner_tube",
                elem_b="outer_tube",
                min_overlap=0.180,
                name="extended post still retains insertion in lower tube",
            )
            extended_post_pos = ctx.part_world_position(upper_post)
        ctx.check(
            "post extends upward",
            rest_post_pos is not None
            and extended_post_pos is not None
            and extended_post_pos[2] > rest_post_pos[2] + 0.15,
            details=f"rest={rest_post_pos}, extended={extended_post_pos}",
        )

    ctx.expect_gap(
        knob,
        upper_post,
        axis="x",
        positive_elem="knob_stub",
        negative_elem="tilt_yoke",
        max_gap=0.001,
        max_penetration=0.0015,
        name="friction knob mounts against the tilt yoke",
    )
    ctx.expect_within(
        desk,
        upper_post,
        axes="x",
        inner_elem="pivot_axle",
        outer_elem="tilt_yoke",
        margin=0.0015,
        name="desk pivot axle stays carried by the tilt yoke",
    )
    ctx.allow_overlap(
        desk,
        upper_post,
        elem_a="pivot_axle",
        elem_b="tilt_yoke",
        reason="The desk pivot axle is intentionally captured inside the simplified tilt-yoke support mesh.",
    )

    tilt_lower = desk_tilt.motion_limits.lower if desk_tilt.motion_limits is not None else None
    tilt_upper = desk_tilt.motion_limits.upper if desk_tilt.motion_limits is not None else None
    if tilt_lower is not None and tilt_upper is not None:
        with ctx.pose({desk_tilt: tilt_lower}):
            low_panel_center = _aabb_center(ctx.part_element_world_aabb(desk, elem="desk_panel"))
        with ctx.pose({desk_tilt: tilt_upper}):
            high_panel_center = _aabb_center(ctx.part_element_world_aabb(desk, elem="desk_panel"))
        ctx.check(
            "desk tilts back with positive rotation",
            low_panel_center is not None
            and high_panel_center is not None
            and high_panel_center[1] < low_panel_center[1] - 0.10,
            details=f"low={low_panel_center}, high={high_panel_center}",
        )

    return ctx.report()


object_model = build_object_model()
