from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


DESK_WIDTH = 0.50
DESK_HEIGHT = 0.34
DESK_DEPTH = 0.018
DESK_SIDE_AXIS_X = DESK_WIDTH / 2.0 + 0.005
DESK_SIDE_AXIS_Y = 0.024
DESK_SIDE_AXIS_Z = -0.205
DESK_TILT_REST = 0.26
WING_WIDTH = 0.132
WING_HEIGHT = 0.250
WING_DEPTH = 0.014
WING_KNUCKLE_LENGTH = 0.104
DESK_KNUCKLE_LENGTH = 0.052
DESK_KNUCKLE_OFFSET = 0.078


def hollow_tube(outer_radius: float, inner_radius: float, length: float):
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def desk_shell_shape():
    thickness = 0.0012
    return (
        cq.Workplane("XY")
        .box(DESK_WIDTH, DESK_DEPTH, DESK_HEIGHT)
        .translate((0.0, 0.024, -0.205))
        .faces("<Y")
        .shell(-thickness)
    )


def wing_shell_shape():
    thickness = 0.0011
    return (
        cq.Workplane("XY")
        .box(WING_WIDTH, WING_DEPTH, WING_HEIGHT)
        .translate((0.070, 0.0, -0.015))
        .faces("<Y")
        .shell(-thickness)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_orchestra_stand")

    powder_black = model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.58, 0.59, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(hollow_tube(0.060, 0.0150, 0.044), "tripod_hub"),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=powder_black,
        name="tripod_hub",
    )
    base.visual(
        mesh_from_cadquery(hollow_tube(0.016, 0.0132, 0.420), "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=powder_black,
        name="outer_sleeve",
    )
    base.visual(
        mesh_from_cadquery(hollow_tube(0.024, 0.0150, 0.058), "clamp_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        material=powder_black,
        name="clamp_collar",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.032, 0.0, 0.548), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="height_lock_stem",
    )
    base.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.052, 0.0, 0.548)),
        material=rubber,
        name="height_lock_knob",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.030 * c, 0.030 * s, 0.115),
                (0.170 * c, 0.170 * s, 0.075),
                (0.390 * c, 0.390 * s, 0.016),
            ],
            radius=0.0105,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        base.visual(
            mesh_from_geometry(leg_mesh, f"tripod_leg_{index}"),
            material=powder_black,
            name=f"leg_{index}",
        )
        base.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(0.390 * c, 0.390 * s, 0.016)),
            material=rubber,
            name=f"foot_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.82, 0.82, 0.64)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
    )

    upper_post = model.part("upper_post")
    upper_post.visual(
        Cylinder(radius=0.0113, length=1.160),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=graphite,
        name="inner_tube",
    )
    upper_post.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.648)),
        material=powder_black,
        name="head_collar",
    )
    upper_post.visual(
        Cylinder(radius=0.0146, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_metal,
        name="stop_collar",
    )
    upper_post.visual(
        Box((0.062, 0.028, 0.064)),
        origin=Origin(xyz=(0.0, -0.024, 0.704)),
        material=powder_black,
        name="yoke_spine",
    )
    for index, side in enumerate((-1.0, 1.0)):
        upper_post.visual(
            Box((0.008, 0.020, 0.050)),
            origin=Origin(xyz=(side * 0.030, -0.012, 0.730)),
            material=powder_black,
            name=f"yoke_cheek_{index}",
        )
    upper_post.visual(
        Cylinder(radius=0.004, length=0.026),
        origin=Origin(xyz=(0.047, 0.0, 0.730), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="tilt_lock_stem",
    )
    upper_post.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.062, 0.0, 0.730)),
        material=rubber,
        name="tilt_lock_knob",
    )
    upper_post.inertial = Inertial.from_geometry(
        Box((0.080, 0.080, 1.180)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
    )

    model.articulation(
        "base_to_upper_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_post,
        origin=Origin(xyz=(0.0, 0.0, 0.545)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.18,
            lower=0.0,
            upper=0.320,
        ),
    )

    desk = model.part("desk")
    desk.visual(
        mesh_from_cadquery(desk_shell_shape(), "desk_shell"),
        material=graphite,
        name="desk_shell",
    )
    for index, side in enumerate((-1.0, 1.0)):
        desk.visual(
            Box((0.012, 0.020, 0.060)),
            origin=Origin(xyz=(side * 0.020, 0.018, -0.030)),
            material=graphite,
            name=f"tilt_strap_{index}",
        )
        desk.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(
                xyz=(side * 0.020, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_metal,
            name=f"tilt_trunnion_{index}",
        )
    desk.visual(
        Box((0.455, 0.045, 0.004)),
        origin=Origin(xyz=(0.0, 0.045, -0.351)),
        material=graphite,
        name="retaining_shelf",
    )
    desk.visual(
        Box((0.448, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, 0.065, -0.342)),
        material=graphite,
        name="retaining_fence",
    )
    for index, side in enumerate((-1.0, 1.0)):
        desk.visual(
            Box((0.008, 0.012, 0.230)),
            origin=Origin(
                xyz=(side * (DESK_SIDE_AXIS_X - 0.009), DESK_SIDE_AXIS_Y, DESK_SIDE_AXIS_Z)
            ),
            material=graphite,
            name=f"desk_leaf_{index}",
        )
        for barrel_index, barrel_z in enumerate(
            (DESK_SIDE_AXIS_Z + DESK_KNUCKLE_OFFSET, DESK_SIDE_AXIS_Z - DESK_KNUCKLE_OFFSET)
        ):
            desk.visual(
                Cylinder(radius=0.005, length=DESK_KNUCKLE_LENGTH),
                origin=Origin(
                    xyz=(side * DESK_SIDE_AXIS_X, DESK_SIDE_AXIS_Y, barrel_z),
                ),
                material=satin_metal,
                name=f"desk_hinge_{index}_{barrel_index}",
            )
    desk.inertial = Inertial.from_geometry(
        Box((0.560, 0.140, 0.420)),
        mass=1.8,
        origin=Origin(xyz=(0.0, -0.010, -0.175)),
    )

    model.articulation(
        "upper_post_to_desk",
        ArticulationType.REVOLUTE,
        parent=upper_post,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.730), rpy=(DESK_TILT_REST, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.5,
            lower=-0.40,
            upper=0.45,
        ),
    )

    for index, side in enumerate((-1.0, 1.0)):
        wing = model.part(f"wing_{index}")
        wing.visual(
            mesh_from_cadquery(wing_shell_shape(), f"wing_shell_{index}"),
            material=graphite,
            name="wing_shell",
        )
        wing.visual(
            Box((0.008, 0.012, 0.230)),
            origin=Origin(xyz=(0.0075, 0.0, 0.0)),
            material=graphite,
            name="wing_leaf",
        )
        wing.visual(
            Cylinder(radius=0.005, length=WING_KNUCKLE_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=satin_metal,
            name="wing_barrel",
        )
        wing.visual(
            Box((0.102, 0.028, 0.004)),
            origin=Origin(xyz=(0.072, 0.016, -0.137)),
            material=graphite,
            name="wing_lip",
        )
        wing.inertial = Inertial.from_geometry(
            Box((0.150, 0.070, 0.280)),
            mass=0.35,
            origin=Origin(xyz=(0.070, 0.0, -0.015)),
        )

        model.articulation(
            f"desk_to_wing_{index}",
            ArticulationType.REVOLUTE,
            parent=desk,
            child=wing,
            origin=Origin(
                xyz=(side * DESK_SIDE_AXIS_X, DESK_SIDE_AXIS_Y, DESK_SIDE_AXIS_Z),
                rpy=(0.0, 0.0, math.pi if side < 0.0 else 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=2.0,
                lower=-1.55,
                upper=0.10,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_post = object_model.get_part("upper_post")
    desk = object_model.get_part("desk")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    lift = object_model.get_articulation("base_to_upper_post")
    tilt = object_model.get_articulation("upper_post_to_desk")
    wing_joint_0 = object_model.get_articulation("desk_to_wing_0")
    wing_joint_1 = object_model.get_articulation("desk_to_wing_1")

    lift_limits = lift.motion_limits
    if lift_limits is not None and lift_limits.upper is not None:
        ctx.expect_within(
            upper_post,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.0025,
            name="telescoping post stays centered in the sleeve",
        )
        ctx.expect_overlap(
            upper_post,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.180,
            name="telescoping post remains inserted at rest",
        )
        rest_pos = ctx.part_world_position(upper_post)
        with ctx.pose({lift: lift_limits.upper}):
            ctx.expect_within(
                upper_post,
                base,
                axes="xy",
                inner_elem="inner_tube",
                outer_elem="outer_sleeve",
                margin=0.0025,
                name="extended post stays centered in the sleeve",
            )
            ctx.expect_overlap(
                upper_post,
                base,
                axes="z",
                elem_a="inner_tube",
                elem_b="outer_sleeve",
                min_overlap=0.090,
                name="extended post keeps retained insertion",
            )
            extended_pos = ctx.part_world_position(upper_post)
        ctx.check(
            "telescoping post extends upward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.20,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    tilt_limits = tilt.motion_limits
    if (
        tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
    ):
        with ctx.pose({tilt: tilt_limits.lower}):
            lower_fence = ctx.part_element_world_aabb(desk, elem="retaining_fence")
        with ctx.pose({tilt: tilt_limits.upper}):
            upper_fence = ctx.part_element_world_aabb(desk, elem="retaining_fence")
        ctx.check(
            "desk tilts upward through the head hinge",
            lower_fence is not None
            and upper_fence is not None
            and upper_fence[0][2] > lower_fence[0][2] + 0.03,
            details=f"lower={lower_fence}, upper={upper_fence}",
        )

    ctx.expect_gap(
        desk,
        wing_0,
        axis="x",
        positive_elem="desk_leaf_0",
        negative_elem="wing_leaf",
        min_gap=0.0075,
        max_gap=0.0095,
        name="left wing sits directly off the desk edge",
    )
    ctx.expect_gap(
        wing_1,
        desk,
        axis="x",
        positive_elem="wing_leaf",
        negative_elem="desk_leaf_1",
        min_gap=0.0075,
        max_gap=0.0095,
        name="right wing sits directly off the desk edge",
    )
    ctx.expect_overlap(
        wing_0,
        desk,
        axes="z",
        elem_a="wing_leaf",
        elem_b="desk_leaf_0",
        min_overlap=0.180,
        name="left wing is carried by a tall desk edge leaf",
    )
    ctx.expect_overlap(
        wing_1,
        desk,
        axes="z",
        elem_a="wing_leaf",
        elem_b="desk_leaf_1",
        min_overlap=0.180,
        name="right wing is carried by a tall desk edge leaf",
    )

    wing_0_open = ctx.part_element_world_aabb(wing_0, elem="wing_shell")
    wing_1_open = ctx.part_element_world_aabb(wing_1, elem="wing_shell")
    with ctx.pose({wing_joint_0: wing_joint_0.motion_limits.lower, wing_joint_1: wing_joint_1.motion_limits.lower}):
        wing_0_folded = ctx.part_element_world_aabb(wing_0, elem="wing_shell")
        wing_1_folded = ctx.part_element_world_aabb(wing_1, elem="wing_shell")
    ctx.check(
        "side wings fold inward from the score width",
        wing_0_open is not None
        and wing_1_open is not None
        and wing_0_folded is not None
        and wing_1_folded is not None
        and wing_0_folded[0][0] > wing_0_open[0][0] + 0.045
        and wing_1_folded[1][0] < wing_1_open[1][0] - 0.045,
        details=(
            f"left_open={wing_0_open}, left_folded={wing_0_folded}, "
            f"right_open={wing_1_open}, right_folded={wing_1_folded}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
