from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shade_shell_mesh():
    """A thin, open-bottom straight cylindrical shade with a small top cable hole."""
    outer_radius = 0.115
    inner_radius = 0.107
    top_z = -0.035
    bottom_z = -0.285
    cap_inner_hole = 0.022
    cap_thickness = 0.014
    profile = [
        (inner_radius, bottom_z),
        (outer_radius, bottom_z),
        (outer_radius, top_z),
        (cap_inner_hole, top_z),
        (cap_inner_hole, top_z - cap_thickness),
        (inner_radius, top_z - cap_thickness),
        (inner_radius, bottom_z),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=64), "shade_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="factory_articulating_wall_lamp")

    plate_steel = model.material("blued_steel", rgba=(0.24, 0.26, 0.28, 1.0))
    bolt_steel = model.material("dark_bolt_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    pivot_steel = model.material("brushed_pivot_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    tube_black = model.material("black_square_tube", rgba=(0.025, 0.027, 0.028, 1.0))
    matte_black = model.material("matte_black_shade", rgba=(0.01, 0.011, 0.012, 1.0))
    backplate = model.part("backplate")
    backplate.visual(
        Box((0.026, 0.180, 0.420)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=plate_steel,
        name="rect_plate",
    )
    backplate.visual(
        Box((0.030, 0.285, 0.082)),
        origin=Origin(xyz=(0.002, 0.0, 0.170)),
        material=plate_steel,
        name="upper_bolt_flange",
    )
    backplate.visual(
        Box((0.030, 0.285, 0.082)),
        origin=Origin(xyz=(0.002, 0.0, -0.170)),
        material=plate_steel,
        name="lower_bolt_flange",
    )
    for z in (-0.170, 0.170):
        for y in (-0.088, 0.088):
            backplate.visual(
                Cylinder(radius=0.018, length=0.006),
                origin=Origin(xyz=(0.016, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=pivot_steel,
                name=f"bolt_washer_{z:+.2f}_{y:+.2f}",
            )
            backplate.visual(
                Cylinder(radius=0.011, length=0.010),
                origin=Origin(xyz=(0.024, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bolt_steel,
                name=f"bolt_head_{z:+.2f}_{y:+.2f}",
            )

    backplate.visual(
        Box((0.082, 0.074, 0.052)),
        origin=Origin(xyz=(0.038, 0.0, 0.120)),
        material=pivot_steel,
        name="upper_post_lug",
    )
    backplate.visual(
        Box((0.082, 0.074, 0.052)),
        origin=Origin(xyz=(0.038, 0.0, -0.085)),
        material=pivot_steel,
        name="lower_post_lug",
    )
    backplate.visual(
        Cylinder(radius=0.027, length=0.300),
        origin=Origin(xyz=(0.065, 0.0, 0.025)),
        material=pivot_steel,
        name="pivot_post",
    )
    backplate.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.065, 0.0, 0.178)),
        material=bolt_steel,
        name="post_top_cap",
    )
    backplate.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.065, 0.0, -0.128)),
        material=bolt_steel,
        name="post_bottom_cap",
    )

    inner_arm = model.part("inner_arm")
    inner_arm.visual(
        Cylinder(radius=0.039, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tube_black,
        name="pivot_sleeve",
    )
    inner_arm.visual(
        Box((0.465, 0.036, 0.036)),
        origin=Origin(xyz=(0.2575, 0.0, 0.0)),
        material=tube_black,
        name="inner_square_tube",
    )
    inner_arm.visual(
        Box((0.090, 0.060, 0.058)),
        origin=Origin(xyz=(0.525, 0.0, 0.0)),
        material=tube_black,
        name="elbow_block",
    )
    inner_arm.visual(
        Cylinder(radius=0.026, length=0.092),
        origin=Origin(xyz=(0.540, 0.0, 0.0)),
        material=pivot_steel,
        name="elbow_pin",
    )
    inner_arm.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.540, 0.0, 0.052)),
        material=bolt_steel,
        name="elbow_top_bolt",
    )
    inner_arm.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.540, 0.0, -0.052)),
        material=bolt_steel,
        name="elbow_bottom_bolt",
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        Cylinder(radius=0.037, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tube_black,
        name="elbow_sleeve",
    )
    outer_arm.visual(
        Box((0.500, 0.036, 0.036)),
        origin=Origin(xyz=(0.285, 0.0, 0.0)),
        material=tube_black,
        name="outer_square_tube",
    )
    outer_arm.visual(
        Box((0.076, 0.076, 0.225)),
        origin=Origin(xyz=(0.560, 0.0, 0.060)),
        material=tube_black,
        name="drop_socket",
    )
    drop_rod = model.part("drop_rod")
    drop_rod.visual(
        Cylinder(radius=0.012, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=pivot_steel,
        name="threaded_rod",
    )
    for index, z in enumerate((-0.170, -0.190, -0.210, -0.240, -0.260, -0.280)):
        drop_rod.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=bolt_steel,
            name=f"thread_crest_{index}",
        )
    drop_rod.visual(
        Cylinder(radius=0.033, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=bolt_steel,
        name="locking_ring",
    )
    drop_rod.visual(
        Cylinder(radius=0.022, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.414)),
        material=pivot_steel,
        name="lower_collar",
    )
    drop_rod.visual(
        Box((0.048, 0.128, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.412)),
        material=pivot_steel,
        name="tilt_yoke_bridge",
    )
    drop_rod.visual(
        Box((0.048, 0.016, 0.075)),
        origin=Origin(xyz=(0.0, 0.060, -0.450)),
        material=pivot_steel,
        name="tilt_yoke_0",
    )
    drop_rod.visual(
        Box((0.048, 0.016, 0.075)),
        origin=Origin(xyz=(0.0, -0.060, -0.450)),
        material=pivot_steel,
        name="tilt_yoke_1",
    )
    drop_rod.visual(
        Cylinder(radius=0.007, length=0.138),
        origin=Origin(xyz=(0.0, 0.0, -0.450), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="tilt_pin",
    )

    shade_head = model.part("shade_head")
    shade_head.visual(
        Cylinder(radius=0.019, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pivot_steel,
        name="tilt_barrel",
    )
    shade_head.visual(
        Box((0.052, 0.052, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=matte_black,
        name="shade_neck",
    )
    shade_head.visual(
        _shade_shell_mesh(),
        origin=Origin(),
        material=matte_black,
        name="shade_shell",
    )
    model.articulation(
        "wall_pivot",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=inner_arm,
        origin=Origin(xyz=(0.065, 0.0, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=outer_arm,
        origin=Origin(xyz=(0.540, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.4, lower=-2.55, upper=2.55),
    )
    model.articulation(
        "drop_height",
        ArticulationType.PRISMATIC,
        parent=outer_arm,
        child=drop_rod,
        origin=Origin(xyz=(0.560, 0.0, 0.070)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.16, lower=0.0, upper=0.125),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=drop_rod,
        child=shade_head,
        origin=Origin(xyz=(0.0, 0.0, -0.450)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=-0.85, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    inner_arm = object_model.get_part("inner_arm")
    outer_arm = object_model.get_part("outer_arm")
    drop_rod = object_model.get_part("drop_rod")
    shade_head = object_model.get_part("shade_head")

    wall_pivot = object_model.get_articulation("wall_pivot")
    elbow = object_model.get_articulation("elbow")
    drop_height = object_model.get_articulation("drop_height")
    shade_tilt = object_model.get_articulation("shade_tilt")

    ctx.allow_overlap(
        backplate,
        inner_arm,
        elem_a="pivot_post",
        elem_b="pivot_sleeve",
        reason="The boom sleeve is intentionally captured around the vertical pivot post.",
    )
    ctx.expect_within(
        backplate,
        inner_arm,
        axes="xy",
        inner_elem="pivot_post",
        outer_elem="pivot_sleeve",
        margin=0.001,
        name="pivot post sits inside boom sleeve footprint",
    )
    ctx.expect_overlap(
        backplate,
        inner_arm,
        axes="z",
        elem_a="pivot_post",
        elem_b="pivot_sleeve",
        min_overlap=0.070,
        name="pivot sleeve has vertical bearing engagement",
    )

    ctx.allow_overlap(
        inner_arm,
        outer_arm,
        elem_a="elbow_pin",
        elem_b="elbow_sleeve",
        reason="The mid-arm elbow sleeve rotates around a solid hinge pin proxy.",
    )
    ctx.allow_overlap(
        inner_arm,
        outer_arm,
        elem_a="elbow_block",
        elem_b="elbow_sleeve",
        reason="The elbow sleeve is locally embedded in the blocky hinge knuckle for a captured factory-arm bearing.",
    )
    ctx.expect_within(
        inner_arm,
        outer_arm,
        axes="xy",
        inner_elem="elbow_pin",
        outer_elem="elbow_sleeve",
        margin=0.001,
        name="elbow pin is centered in elbow sleeve",
    )
    ctx.expect_overlap(
        inner_arm,
        outer_arm,
        axes="z",
        elem_a="elbow_pin",
        elem_b="elbow_sleeve",
        min_overlap=0.060,
        name="elbow hinge has bearing depth",
    )
    ctx.expect_overlap(
        inner_arm,
        outer_arm,
        axes="xy",
        elem_a="elbow_block",
        elem_b="elbow_sleeve",
        min_overlap=0.050,
        name="elbow sleeve is seated in the hinge knuckle",
    )

    ctx.allow_overlap(
        outer_arm,
        drop_rod,
        elem_a="drop_socket",
        elem_b="threaded_rod",
        reason="The threaded drop rod is represented as sliding inside the square end socket.",
    )
    ctx.expect_within(
        drop_rod,
        outer_arm,
        axes="xy",
        inner_elem="threaded_rod",
        outer_elem="drop_socket",
        margin=0.002,
        name="drop rod stays centered in square socket",
    )
    ctx.expect_overlap(
        outer_arm,
        drop_rod,
        axes="z",
        elem_a="drop_socket",
        elem_b="threaded_rod",
        min_overlap=0.070,
        name="drop rod retained in socket at short setting",
    )
    with ctx.pose({drop_height: 0.125}):
        ctx.expect_within(
            drop_rod,
            outer_arm,
            axes="xy",
            inner_elem="threaded_rod",
            outer_elem="drop_socket",
            margin=0.002,
            name="extended drop rod remains centered in socket",
        )
        ctx.expect_overlap(
            outer_arm,
            drop_rod,
            axes="z",
            elem_a="drop_socket",
            elem_b="threaded_rod",
            min_overlap=0.025,
            name="extended drop rod retains threaded insertion",
        )

    ctx.allow_overlap(
        drop_rod,
        shade_head,
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        reason="The shade tilt barrel is intentionally captured by the yoke pin.",
    )
    ctx.expect_overlap(
        drop_rod,
        shade_head,
        axes="y",
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        min_overlap=0.070,
        name="tilt pin spans shade barrel",
    )
    ctx.expect_overlap(
        drop_rod,
        shade_head,
        axes="xz",
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        min_overlap=0.012,
        name="tilt pin passes through the shade barrel center",
    )

    rest_drop_pos = ctx.part_world_position(drop_rod)
    with ctx.pose({elbow: 2.25}):
        folded_drop_pos = ctx.part_world_position(drop_rod)
    ctx.check(
        "elbow folds the outer arm closer to the wall",
        rest_drop_pos is not None
        and folded_drop_pos is not None
        and folded_drop_pos[0] < rest_drop_pos[0] - 0.25,
        details=f"rest={rest_drop_pos}, folded={folded_drop_pos}",
    )

    with ctx.pose({wall_pivot: math.pi / 2.0}):
        swung_aabb = ctx.part_world_aabb(inner_arm)
    ctx.check(
        "wall pivot swings boom horizontally",
        swung_aabb is not None and swung_aabb[1][1] > 0.45,
        details=f"swung_aabb={swung_aabb}",
    )

    high_drop = ctx.part_world_position(shade_head)
    with ctx.pose({drop_height: 0.125}):
        low_drop = ctx.part_world_position(shade_head)
    ctx.check(
        "prismatic drop lowers the shade head",
        high_drop is not None and low_drop is not None and low_drop[2] < high_drop[2] - 0.10,
        details=f"high={high_drop}, low={low_drop}",
    )

    shade_closed_aabb = ctx.part_world_aabb(shade_head)
    with ctx.pose({shade_tilt: 0.70}):
        shade_tilted_aabb = ctx.part_world_aabb(shade_head)
    ctx.check(
        "shade tilt joint changes head angle",
        shade_closed_aabb is not None
        and shade_tilted_aabb is not None
        and shade_tilted_aabb[0][0] < shade_closed_aabb[0][0] - 0.08,
        details=f"closed={shade_closed_aabb}, tilted={shade_tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
