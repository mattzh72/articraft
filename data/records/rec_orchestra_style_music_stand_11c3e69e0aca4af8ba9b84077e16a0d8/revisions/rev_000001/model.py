from __future__ import annotations

import math

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
    mesh_from_geometry,
    tube_from_spline_points,
)


DESK_REST_TILT = math.radians(18.0)


def hollow_tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def make_desk_shell() -> cq.Workplane:
    width = 0.520
    height = 0.340
    sheet = 0.0018
    side_depth = 0.018
    lip_depth = 0.060
    lip_height = 0.042
    top_return = 0.012
    header_width = 0.180
    header_height = 0.042
    header_depth = 0.018

    panel = cq.Workplane("XY").box(width, sheet, height).translate((0.0, sheet / 2.0, -height / 2.0))
    left_flange = cq.Workplane("XY").box(sheet, side_depth, height).translate(
        (-width / 2.0 + sheet / 2.0, side_depth / 2.0, -height / 2.0)
    )
    right_flange = cq.Workplane("XY").box(sheet, side_depth, height).translate(
        (width / 2.0 - sheet / 2.0, side_depth / 2.0, -height / 2.0)
    )
    lower_shelf = cq.Workplane("XY").box(width - 2.0 * sheet, lip_depth, sheet).translate(
        (0.0, lip_depth / 2.0, -height + sheet / 2.0)
    )
    lower_fence = cq.Workplane("XY").box(width - 2.0 * sheet, sheet, lip_height).translate(
        (0.0, lip_depth - sheet / 2.0, -height + lip_height / 2.0)
    )
    top_return_flange = cq.Workplane("XY").box(width, top_return, sheet).translate(
        (0.0, -(top_return - sheet) / 2.0, -sheet / 2.0)
    )
    rear_header = cq.Workplane("XY").box(header_width, header_depth, header_height).translate(
        (0.0, header_depth / 2.0, -header_height / 2.0 - 0.006)
    )

    return panel.union(left_flange).union(right_flange).union(lower_shelf).union(lower_fence).union(
        top_return_flange
    ).union(rear_header)


def build_leg_mesh() -> object:
    return tube_from_spline_points(
        [
            (0.010, 0.0, 0.000),
            (0.095, 0.0, -0.014),
            (0.220, 0.0, -0.082),
            (0.325, 0.0, -0.160),
        ],
        radius=0.009,
        samples_per_segment=20,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_tripod_orchestra_stand")

    powder_black = model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.056, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=powder_black,
        name="hub_shell",
    )
    base.visual(
        mesh_from_cadquery(hollow_tube(0.016, 0.0125, 0.520), "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=powder_black,
        name="outer_sleeve",
    )
    collar_shape = hollow_tube(0.027, 0.0146, 0.060).union(
        cq.Workplane("YZ").circle(0.010).extrude(0.022).translate((0.013, 0.0, 0.0))
    )
    base.visual(
        mesh_from_cadquery(collar_shape, "clamp_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=graphite,
        name="clamp_collar",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(xyz=(0.028, 0.0, 0.530), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="knob_boss",
    )
    for index, (x, y) in enumerate(
        (
            (0.0124, 0.0),
            (-0.0062, 0.0107),
            (-0.0062, -0.0107),
        )
    ):
        base.visual(
            Box((0.004, 0.010, 0.028)),
            origin=Origin(xyz=(x, y, 0.706)),
            material=graphite,
            name=f"guide_pad_{index}",
        )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        base.visual(
            Box((0.024, 0.020, 0.024)),
            origin=Origin(
                xyz=(0.056 * c, 0.056 * s, 0.160),
                rpy=(0.0, 0.0, angle),
            ),
            material=powder_black,
            name=f"leg_lug_{index}",
        )

    leg_joint_origin_radius = 0.068
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        leg = model.part(f"leg_{index}")
        leg.visual(
            mesh_from_geometry(build_leg_mesh(), f"tripod_leg_{index}"),
            material=powder_black,
            name="leg_tube",
        )
        leg.visual(
            Sphere(radius=0.007),
            origin=Origin(xyz=(0.007, 0.0, -0.001)),
            material=powder_black,
            name="leg_socket",
        )
        leg.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(0.325, 0.0, -0.160)),
            material=rubber,
            name="foot",
        )
        model.articulation(
            f"base_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leg,
            origin=Origin(
                xyz=(leg_joint_origin_radius * c, leg_joint_origin_radius * s, 0.160),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=1.2,
                lower=0.0,
                upper=1.18,
            ),
        )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="threaded_stem",
    )
    clamp_knob.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="knob_hub",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        clamp_knob.visual(
            Cylinder(radius=0.0065, length=0.020),
            origin=Origin(
                xyz=(0.030, 0.0125 * math.cos(angle), 0.0125 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=powder_black,
            name=f"lobe_{index}",
        )
    model.articulation(
        "base_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=clamp_knob,
        origin=Origin(xyz=(0.048, 0.0, 0.530)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )

    upper_mast = model.part("upper_mast")
    upper_mast.visual(
        Cylinder(radius=0.0104, length=0.900),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=graphite,
        name="inner_tube",
    )
    upper_mast.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=powder_black,
        name="top_cap",
    )
    model.articulation(
        "base_to_upper_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.18,
            lower=0.0,
            upper=0.320,
        ),
    )

    head = model.part("head")
    head.visual(
        Box((0.050, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=powder_black,
        name="head_block",
    )
    head.visual(
        Box((0.030, 0.066, 0.018)),
        origin=Origin(xyz=(0.0, 0.020, 0.048)),
        material=powder_black,
        name="support_spine",
    )
    head.visual(
        Box((0.028, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.018, 0.038), rpy=(-0.78, 0.0, 0.0)),
        material=powder_black,
        name="support_gusset",
    )
    head.visual(
        Box((0.132, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.036, 0.062)),
        material=powder_black,
        name="yoke_bridge",
    )
    head.visual(
        Box((0.012, 0.016, 0.044)),
        origin=Origin(xyz=(-0.058, 0.050, 0.050)),
        material=powder_black,
        name="yoke_0",
    )
    head.visual(
        Box((0.012, 0.016, 0.044)),
        origin=Origin(xyz=(0.058, 0.050, 0.050)),
        material=powder_black,
        name="yoke_1",
    )
    model.articulation(
        "upper_mast_to_head",
        ArticulationType.FIXED,
        parent=upper_mast,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.490)),
    )

    desk = model.part("desk")
    desk.visual(
        mesh_from_cadquery(make_desk_shell(), "music_stand_desk"),
        origin=Origin(xyz=(0.0, 0.014, -0.010), rpy=(DESK_REST_TILT, 0.0, 0.0)),
        material=graphite,
        name="desk_shell",
    )
    desk.visual(
        Box((0.010, 0.014, 0.028)),
        origin=Origin(xyz=(-0.044, 0.005, -0.014)),
        material=powder_black,
        name="pivot_tab_0",
    )
    desk.visual(
        Box((0.010, 0.014, 0.028)),
        origin=Origin(xyz=(0.044, 0.005, -0.014)),
        material=powder_black,
        name="pivot_tab_1",
    )
    model.articulation(
        "head_to_desk",
        ArticulationType.REVOLUTE,
        parent=head,
        child=desk,
        origin=Origin(xyz=(0.0, 0.050, 0.072)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=-0.22,
            upper=0.36,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_mast = object_model.get_part("upper_mast")
    head = object_model.get_part("head")
    desk = object_model.get_part("desk")
    clamp_knob = object_model.get_part("clamp_knob")
    leg_0 = object_model.get_part("leg_0")

    mast_slide = object_model.get_articulation("base_to_upper_mast")
    desk_tilt = object_model.get_articulation("head_to_desk")
    knob_spin = object_model.get_articulation("base_to_clamp_knob")
    leg_fold = object_model.get_articulation("base_to_leg_0")

    mast_limits = mast_slide.motion_limits
    desk_limits = desk_tilt.motion_limits
    leg_limits = leg_fold.motion_limits

    if mast_limits is not None and mast_limits.upper is not None:
        with ctx.pose({mast_slide: 0.0}):
            ctx.expect_within(
                upper_mast,
                base,
                axes="xy",
                inner_elem="inner_tube",
                outer_elem="outer_sleeve",
                margin=0.0025,
                name="collapsed mast stays centered in sleeve",
            )
            ctx.expect_overlap(
                upper_mast,
                base,
                axes="z",
                elem_a="inner_tube",
                elem_b="outer_sleeve",
                min_overlap=0.420,
                name="collapsed mast remains deeply inserted",
            )
            rest_cap_aabb = ctx.part_element_world_aabb(upper_mast, elem="top_cap")

        with ctx.pose({mast_slide: mast_limits.upper}):
            ctx.expect_within(
                upper_mast,
                base,
                axes="xy",
                inner_elem="inner_tube",
                outer_elem="outer_sleeve",
                margin=0.0025,
                name="extended mast stays centered in sleeve",
            )
            ctx.expect_overlap(
                upper_mast,
                base,
                axes="z",
                elem_a="inner_tube",
                elem_b="outer_sleeve",
                min_overlap=0.100,
                name="extended mast retains insertion",
            )
            extended_cap_aabb = ctx.part_element_world_aabb(upper_mast, elem="top_cap")

        ctx.check(
            "mast extends upward",
            rest_cap_aabb is not None
            and extended_cap_aabb is not None
            and extended_cap_aabb[1][2] > rest_cap_aabb[1][2] + 0.250,
            details=f"rest_cap_aabb={rest_cap_aabb}, extended_cap_aabb={extended_cap_aabb}",
        )

    if desk_limits is not None and desk_limits.lower is not None and desk_limits.upper is not None:
        with ctx.pose({desk_tilt: desk_limits.lower}):
            low_shell_aabb = ctx.part_element_world_aabb(desk, elem="desk_shell")
        with ctx.pose({desk_tilt: desk_limits.upper}):
            high_shell_aabb = ctx.part_element_world_aabb(desk, elem="desk_shell")

        ctx.check(
            "desk tilt raises and advances the lip",
            low_shell_aabb is not None
            and high_shell_aabb is not None
            and high_shell_aabb[1][1] > low_shell_aabb[1][1] + 0.035
            and high_shell_aabb[0][2] > low_shell_aabb[0][2] + 0.020,
            details=f"low_shell_aabb={low_shell_aabb}, high_shell_aabb={high_shell_aabb}",
        )

    with ctx.pose({knob_spin: 0.0}):
        knob_rest = ctx.part_element_world_aabb(clamp_knob, elem="lobe_0")
    with ctx.pose({knob_spin: 1.0}):
        knob_turned = ctx.part_element_world_aabb(clamp_knob, elem="lobe_0")
    ctx.check(
        "clamp knob visibly rotates about its threaded axis",
        knob_rest is not None
        and knob_turned is not None
        and (
            abs((knob_turned[0][1] + knob_turned[1][1]) - (knob_rest[0][1] + knob_rest[1][1])) > 0.008
            or abs((knob_turned[0][2] + knob_turned[1][2]) - (knob_rest[0][2] + knob_rest[1][2])) > 0.008
        ),
        details=f"knob_rest={knob_rest}, knob_turned={knob_turned}",
    )

    if leg_limits is not None and leg_limits.upper is not None:
        with ctx.pose({leg_fold: 0.0}):
            foot_deployed = ctx.part_element_world_aabb(leg_0, elem="foot")
        with ctx.pose({leg_fold: leg_limits.upper}):
            foot_folded = ctx.part_element_world_aabb(leg_0, elem="foot")
        ctx.check(
            "tripod leg folds upward toward storage",
            foot_deployed is not None
            and foot_folded is not None
            and foot_folded[1][2] > foot_deployed[1][2] + 0.120,
            details=f"foot_deployed={foot_deployed}, foot_folded={foot_folded}",
        )

    return ctx.report()


object_model = build_object_model()
