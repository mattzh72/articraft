from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[index] + high[index]) * 0.5 for index in range(3))


def _planar_radius(aabb) -> float | None:
    center = _aabb_center(aabb)
    if center is None:
        return None
    return math.hypot(center[0], center[1])


def _build_crown_shell():
    bore_radius = 0.0215
    base_ring = cq.Workplane("XY").circle(0.057).circle(bore_radius).extrude(0.060)
    sleeve = cq.Workplane("XY").circle(0.031).circle(bore_radius).extrude(0.335)
    top_collar = (
        cq.Workplane("XY")
        .circle(0.039)
        .circle(bore_radius)
        .extrude(0.030)
        .translate((0.0, 0.0, 0.335))
    )
    lower_flange = (
        cq.Workplane("XY")
        .circle(0.058)
        .circle(bore_radius)
        .extrude(0.016)
        .translate((0.0, 0.0, -0.010))
    )
    return base_ring.union(sleeve).union(top_collar).union(lower_flange)


def _build_projector_shell():
    body = (
        cq.Workplane("XY")
        .box(0.220, 0.160, 0.082)
        .edges("|Z")
        .fillet(0.013)
        .edges("%Line and (<Z or >Z)")
        .fillet(0.005)
    )
    lens_recess = (
        cq.Workplane("YZ")
        .workplane(offset=0.110)
        .circle(0.030)
        .extrude(0.014)
    )
    top_relief = (
        cq.Workplane("XY")
        .box(0.120, 0.080, 0.020)
        .translate((0.015, 0.0, 0.041))
    )
    return body.cut(lens_recess).cut(top_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="projector_tripod_stand")

    powder_black = model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.28, 0.29, 0.31, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.71, 0.72, 0.74, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.26, 0.40, 0.48, 0.40))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    crown = model.part("crown")
    crown.visual(
        mesh_from_cadquery(_build_crown_shell(), "projector_tripod_crown_shell"),
        material=powder_black,
        name="crown_shell",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        lug_x, lug_y = _polar_xy(0.046, angle)
        crown.visual(
            Box((0.032, 0.026, 0.018)),
            origin=Origin(xyz=(lug_x, lug_y, 0.001), rpy=(0.0, 0.0, angle)),
            material=powder_black,
            name=f"lug_{index}",
        )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.010, length=0.020),
            origin=Origin(xyz=(0.022, 0.0, -0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.028, 0.020, 0.022)),
            origin=Origin(xyz=(0.026, 0.0, -0.016)),
            material=graphite,
            name="hinge_block",
        )
        leg.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.028, 0.0, -0.024),
                        (0.120, 0.0, -0.220),
                        (0.250, 0.0, -0.460),
                        (0.348, 0.0, -0.635),
                    ],
                    radius=0.012,
                    samples_per_segment=18,
                    radial_segments=18,
                    cap_ends=True,
                    up_hint=(0.0, 1.0, 0.0),
                ),
                f"projector_tripod_leg_{index}",
            ),
            material=graphite,
            name="leg_tube",
        )
        leg.visual(
            Sphere(radius=0.016),
            origin=Origin(xyz=(0.352, 0.0, -0.646)),
            material=rubber,
            name="foot",
        )

        hinge_x, hinge_y = _polar_xy(0.050, angle)
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=(hinge_x, hinge_y, 0.0), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=35.0,
                velocity=1.2,
                lower=-0.25,
                upper=0.58,
            ),
        )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.0185, length=0.780),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=satin_silver,
        name="inner_post",
    )
    column.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=powder_black,
        name="stop_collar",
    )
    column.visual(
        Cylinder(radius=0.029, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.484)),
        material=powder_black,
        name="top_collar",
    )
    column.visual(
        Cylinder(radius=0.021, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.526)),
        material=satin_silver,
        name="pan_spigot",
    )

    model.articulation(
        "crown_to_column",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.22,
            lower=0.0,
            upper=0.240,
        ),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=powder_black,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=powder_black,
        name="pan_neck",
    )
    pan_head.visual(
        Box((0.050, 0.070, 0.022)),
        origin=Origin(xyz=(-0.012, 0.0, 0.060)),
        material=powder_black,
        name="yoke_base",
    )
    pan_head.visual(
        Box((0.016, 0.064, 0.040)),
        origin=Origin(xyz=(-0.032, 0.0, 0.080)),
        material=powder_black,
        name="rear_brace",
    )
    for side, y_pos in enumerate((-0.038, 0.038)):
        pan_head.visual(
            Box((0.040, 0.012, 0.088)),
            origin=Origin(xyz=(-0.002, y_pos, 0.104)),
            material=powder_black,
            name=f"side_cheek_{side}",
        )

    model.articulation(
        "column_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.6),
    )

    tilt_plate = model.part("tilt_plate")
    tilt_plate.visual(
        Box((0.042, 0.064, 0.020)),
        origin=Origin(xyz=(0.000, 0.0, 0.000)),
        material=graphite,
        name="trunnion_block",
    )
    tilt_plate.visual(
        Box((0.036, 0.052, 0.052)),
        origin=Origin(xyz=(0.030, 0.0, 0.032)),
        material=graphite,
        name="tilt_riser",
    )
    tilt_plate.visual(
        Box((0.200, 0.120, 0.008)),
        origin=Origin(xyz=(0.094, 0.0, 0.060)),
        material=dark_grey,
        name="top_plate",
    )
    tilt_plate.visual(
        mesh_from_cadquery(_build_projector_shell(), "projector_body_shell"),
        origin=Origin(xyz=(0.110, 0.0, 0.103)),
        material=dark_grey,
        name="projector_shell",
    )
    tilt_plate.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(xyz=(0.219, 0.0, 0.103), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_silver,
        name="lens_bezel",
    )
    tilt_plate.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.232, 0.0, 0.103), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_glass",
    )

    model.articulation(
        "pan_head_to_tilt_plate",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_plate,
        origin=Origin(xyz=(0.000, 0.0, 0.104)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-0.35,
            upper=0.72,
        ),
    )

    side_knob = model.part("side_knob")
    side_knob.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="shaft",
    )
    side_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.022,
                body_style="mushroom",
                crown_radius=0.004,
                edge_radius=0.0015,
                center=False,
            ),
            "projector_tripod_side_knob",
        ),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="knob_body",
    )

    model.articulation(
        "pan_head_to_side_knob",
        ArticulationType.CONTINUOUS,
        parent=pan_head,
        child=side_knob,
        origin=Origin(xyz=(0.000, 0.0455, 0.104)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=9.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    column = object_model.get_part("column")
    pan_head = object_model.get_part("pan_head")
    tilt_plate = object_model.get_part("tilt_plate")
    leg_0 = object_model.get_part("leg_0")
    side_knob = object_model.get_part("side_knob")

    column_slide = object_model.get_articulation("crown_to_column")
    pan_joint = object_model.get_articulation("column_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_tilt_plate")
    leg_joint = object_model.get_articulation("crown_to_leg_0")
    knob_joint = object_model.get_articulation("pan_head_to_side_knob")

    slide_upper = 0.240
    leg_fold = 0.58
    pan_turn = math.pi / 2.0
    tilt_up = 0.55

    ctx.expect_within(
        column,
        crown,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="crown_shell",
        margin=0.0,
        name="column stays centered within the crown sleeve",
    )
    ctx.expect_overlap(
        column,
        crown,
        axes="z",
        elem_a="inner_post",
        elem_b="crown_shell",
        min_overlap=0.300,
        name="collapsed column remains deeply inserted in the sleeve",
    )

    rest_pan_pos = ctx.part_world_position(pan_head)
    with ctx.pose({column_slide: slide_upper}):
        ctx.expect_within(
            column,
            crown,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="crown_shell",
            margin=0.0,
            name="extended column stays centered within the crown sleeve",
        )
        ctx.expect_overlap(
            column,
            crown,
            axes="z",
            elem_a="inner_post",
            elem_b="crown_shell",
            min_overlap=0.060,
            name="extended column retains insertion in the sleeve",
        )
        extended_pan_pos = ctx.part_world_position(pan_head)

    ctx.check(
        "column extends upward",
        rest_pan_pos is not None
        and extended_pan_pos is not None
        and extended_pan_pos[2] > rest_pan_pos[2] + 0.18,
        details=f"rest={rest_pan_pos}, extended={extended_pan_pos}",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(tilt_plate, elem="lens_bezel")
    with ctx.pose({pan_joint: pan_turn}):
        turned_lens_aabb = ctx.part_element_world_aabb(tilt_plate, elem="lens_bezel")

    rest_lens_center = _aabb_center(rest_lens_aabb)
    turned_lens_center = _aabb_center(turned_lens_aabb)
    ctx.check(
        "pan joint sweeps the projector around the column",
        rest_lens_center is not None
        and turned_lens_center is not None
        and math.hypot(
            turned_lens_center[0] - rest_lens_center[0],
            turned_lens_center[1] - rest_lens_center[1],
        )
        > 0.20
        and abs(turned_lens_center[2] - rest_lens_center[2]) < 0.03,
        details=f"rest={rest_lens_center}, turned={turned_lens_center}",
    )

    with ctx.pose({tilt_joint: tilt_up}):
        tilted_lens_aabb = ctx.part_element_world_aabb(tilt_plate, elem="lens_bezel")

    tilted_lens_center = _aabb_center(tilted_lens_aabb)
    ctx.check(
        "tilt plate raises the projector nose",
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] > rest_lens_center[2] + 0.06,
        details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
    )

    rest_foot_radius = _planar_radius(ctx.part_element_world_aabb(leg_0, elem="foot"))
    with ctx.pose({leg_joint: leg_fold}):
        folded_foot_radius = _planar_radius(ctx.part_element_world_aabb(leg_0, elem="foot"))

    ctx.check(
        "tripod leg folds inward toward the column",
        rest_foot_radius is not None
        and folded_foot_radius is not None
        and folded_foot_radius < rest_foot_radius - 0.12,
        details=f"rest_radius={rest_foot_radius}, folded_radius={folded_foot_radius}",
    )

    ctx.expect_origin_gap(
        side_knob,
        pan_head,
        axis="y",
        min_gap=0.040,
        name="side knob sits outboard of the tilt yoke",
    )
    ctx.check(
        "side knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"type={knob_joint.articulation_type}, limits={knob_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
