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
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


SLEEVE_TOP_Z = 0.90
COLUMN_TRAVEL = 0.30
PAN_HEAD_Z = 1.03
YOKE_TRUNNION_Z = 0.28
GEL_TRAVEL = 0.055


def _tube_shell_shape(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )


def _can_shell_shape() -> cq.Workplane:
    body_radius = 0.115
    shell_thickness = 0.004
    body_length = 0.28
    rear_wall = 0.008

    shell = cq.Workplane("XZ").circle(body_radius).extrude(body_length / 2.0, both=True)
    inner_void = (
        cq.Workplane("XZ")
        .circle(body_radius - shell_thickness)
        .extrude((body_length - rear_wall) / 2.0, both=True)
        .translate((0.0, rear_wall / 2.0, 0.0))
    )
    shell = shell.cut(inner_void)

    front_ring = (
        cq.Workplane("XZ")
        .circle(0.129)
        .circle(0.102)
        .extrude(0.050 / 2.0, both=True)
        .translate((0.0, 0.140, 0.0))
    )
    rear_housing = (
        cq.Workplane("XZ")
        .circle(0.092)
        .extrude(0.060 / 2.0, both=True)
        .translate((0.0, -0.170, 0.0))
    )
    rear_cap = (
        cq.Workplane("XZ")
        .circle(0.056)
        .extrude(0.030 / 2.0, both=True)
        .translate((0.0, -0.215, 0.0))
    )

    shell = shell.union(front_ring).union(rear_housing).union(rear_cap)

    for side in (-1.0, 1.0):
        trunnion = (
            cq.Workplane("YZ")
            .circle(0.017)
            .extrude(0.034 / 2.0, both=True)
            .translate((side * 0.120, 0.0, 0.0))
        )
        shell = shell.union(trunnion)

        guide_slot = (
            cq.Workplane("XY")
            .box(0.036, 0.118, 0.022)
            .translate((side * 0.112, 0.128, 0.0))
        )
        shell = shell.cut(guide_slot)

    return shell


def _gel_frame_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .rect(0.238, 0.238)
        .rect(0.206, 0.206)
        .extrude(0.006 / 2.0, both=True)
    )


def _center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((a + b) / 2.0 for a, b in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="theatre_spotlight_on_stand")

    matte_black = model.material("matte_black", rgba=(0.09, 0.10, 0.11, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.59, 0.62, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    gel_tint = model.material("gel_tint", rgba=(0.67, 0.33, 0.16, 0.90))

    base = model.part("stand")
    base.visual(
        Cylinder(radius=0.060, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=matte_black,
        name="tripod_hub",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=matte_black,
        name="lower_collar",
    )
    base.visual(
        mesh_from_cadquery(_tube_shell_shape(0.032, 0.026, 0.720), "stand_lower_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        material=matte_black,
        name="lower_sleeve",
    )
    base.visual(
        mesh_from_cadquery(_tube_shell_shape(0.040, 0.026, 0.050), "stand_sleeve_head"),
        origin=Origin(xyz=(0.0, 0.0, 0.925)),
        material=matte_black,
        name="sleeve_head",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.046),
        origin=Origin(xyz=(0.045, 0.0, 0.625), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="height_lock_stem",
    )
    base.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.061, 0.0, 0.625)),
        material=rubber,
        name="height_lock_knob",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.032 * c, 0.032 * s, 0.108),
                (0.185 * c, 0.185 * s, 0.080),
                (0.520 * c, 0.520 * s, 0.020),
            ],
            radius=0.013,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        base.visual(
            mesh_from_geometry(leg_mesh, f"spotlight_leg_{index}"),
            material=matte_black,
            name=f"leg_{index}",
        )
        base.visual(
            Sphere(radius=0.017),
            origin=Origin(xyz=(0.520 * c, 0.520 * s, 0.020)),
            material=rubber,
            name=f"foot_{index}",
        )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.022, length=1.440),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=graphite,
        name="inner_tube",
    )
    column.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.960)),
        material=matte_black,
        name="head_collar",
    )
    column.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.010)),
        material=steel,
        name="pan_spigot",
    )

    model.articulation(
        "stand_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=0.0,
            upper=COLUMN_TRAVEL,
        ),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.038, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=matte_black,
        name="pan_hub",
    )
    yoke.visual(
        Box((0.120, 0.100, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=matte_black,
        name="pan_block",
    )
    yoke.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.340, 0.100, 0.280),
                span_width=0.280,
                trunnion_diameter=0.022,
                trunnion_center_z=0.160,
                base_thickness=0.030,
                corner_radius=0.010,
                center=False,
            ),
            "spotlight_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=matte_black,
        name="yoke_frame",
    )
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        yoke.visual(
            Cylinder(radius=0.017, length=0.016),
            origin=Origin(
                xyz=(side * 0.145, 0.0, YOKE_TRUNNION_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"tilt_boss_{suffix}",
        )

    model.articulation(
        "column_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_HEAD_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.20),
    )

    can = model.part("can")
    can.visual(
        mesh_from_cadquery(_can_shell_shape(), "spotlight_can_shell"),
        material=graphite,
        name="can_shell",
    )
    handle_mesh = tube_from_spline_points(
        [
            (0.0, -0.050, 0.102),
            (0.0, -0.010, 0.145),
            (0.0, 0.050, 0.150),
            (0.0, 0.108, 0.118),
        ],
        radius=0.008,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    can.visual(
        mesh_from_geometry(handle_mesh, "spotlight_top_handle"),
        material=steel,
        name="top_handle",
    )
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        can.visual(
            Box((0.010, 0.100, 0.014)),
            origin=Origin(xyz=(side * 0.129, 0.118, 0.0)),
            material=steel,
            name=f"guide_rail_{suffix}",
        )
        for bridge_index, bridge_z in enumerate((-0.009, 0.009)):
            can.visual(
                Box((0.012, 0.028, 0.012)),
                origin=Origin(xyz=(side * 0.128, 0.118, bridge_z)),
                material=steel,
                name=f"rail_bridge_{suffix}_{bridge_index}",
            )

    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, YOKE_TRUNNION_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.50,
            lower=-1.00,
            upper=0.90,
        ),
    )

    gel_frame = model.part("gel_frame")
    gel_frame.visual(
        mesh_from_cadquery(_gel_frame_shape(), "spotlight_gel_frame"),
        material=steel,
        name="frame",
    )
    gel_frame.visual(
        Box((0.208, 0.0015, 0.208)),
        origin=Origin(),
        material=gel_tint,
        name="gel",
    )
    gel_frame.visual(
        Box((0.080, 0.012, 0.015)),
        origin=Origin(xyz=(0.0, 0.009, 0.124)),
        material=steel,
        name="pull_tab",
    )
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        gel_frame.visual(
            Box((0.010, 0.080, 0.012)),
            origin=Origin(xyz=(side * 0.119, -0.032, 0.0)),
            material=steel,
            name=f"guide_tab_{suffix}",
        )

    model.articulation(
        "can_to_gel_frame",
        ArticulationType.PRISMATIC,
        parent=can,
        child=gel_frame,
        origin=Origin(xyz=(0.0, 0.171, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.15,
            lower=0.0,
            upper=GEL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    column = object_model.get_part("column")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    gel_frame = object_model.get_part("gel_frame")

    stand_to_column = object_model.get_articulation("stand_to_column")
    column_to_yoke = object_model.get_articulation("column_to_yoke")
    yoke_to_can = object_model.get_articulation("yoke_to_can")
    can_to_gel_frame = object_model.get_articulation("can_to_gel_frame")

    ctx.expect_within(
        column,
        stand,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="lower_sleeve",
        margin=0.002,
        name="column stays centered in sleeve at rest",
    )
    ctx.expect_overlap(
        column,
        stand,
        axes="z",
        elem_a="inner_tube",
        elem_b="lower_sleeve",
        min_overlap=0.48,
        name="column remains deeply inserted when collapsed",
    )

    rest_column_pos = ctx.part_world_position(column)
    with ctx.pose({stand_to_column: COLUMN_TRAVEL}):
        ctx.expect_within(
            column,
            stand,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="lower_sleeve",
            margin=0.002,
            name="column stays centered in sleeve when extended",
        )
        ctx.expect_overlap(
            column,
            stand,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_sleeve",
            min_overlap=0.18,
            name="column retains insertion at full height",
        )
        extended_column_pos = ctx.part_world_position(column)

    ctx.check(
        "column extends upward",
        rest_column_pos is not None
        and extended_column_pos is not None
        and extended_column_pos[2] > rest_column_pos[2] + 0.20,
        details=f"rest={rest_column_pos}, extended={extended_column_pos}",
    )

    ctx.expect_within(
        gel_frame,
        can,
        axes="xz",
        inner_elem="frame",
        outer_elem="can_shell",
        margin=0.020,
        name="gel frame stays aligned to can opening",
    )
    ctx.expect_overlap(
        gel_frame,
        can,
        axes="y",
        elem_a="guide_tab_0",
        elem_b="guide_rail_0",
        min_overlap=0.030,
        name="left guide tab remains engaged when closed",
    )
    ctx.expect_overlap(
        gel_frame,
        can,
        axes="y",
        elem_a="guide_tab_1",
        elem_b="guide_rail_1",
        min_overlap=0.030,
        name="right guide tab remains engaged when closed",
    )

    rest_gel_pos = ctx.part_world_position(gel_frame)
    with ctx.pose({can_to_gel_frame: GEL_TRAVEL}):
        ctx.expect_within(
            gel_frame,
            can,
            axes="xz",
            inner_elem="frame",
            outer_elem="can_shell",
            margin=0.020,
            name="gel frame stays aligned when pulled out",
        )
        ctx.expect_overlap(
            gel_frame,
            can,
            axes="y",
            elem_a="guide_tab_0",
            elem_b="guide_rail_0",
            min_overlap=0.010,
            name="left guide tab retains insertion when extended",
        )
        ctx.expect_overlap(
            gel_frame,
            can,
            axes="y",
            elem_a="guide_tab_1",
            elem_b="guide_rail_1",
            min_overlap=0.010,
            name="right guide tab retains insertion when extended",
        )
        extended_gel_pos = ctx.part_world_position(gel_frame)

    ctx.check(
        "gel frame pulls forward",
        rest_gel_pos is not None
        and extended_gel_pos is not None
        and extended_gel_pos[1] > rest_gel_pos[1] + 0.04,
        details=f"rest={rest_gel_pos}, extended={extended_gel_pos}",
    )

    rest_gel_center = ctx.part_world_position(gel_frame)
    with ctx.pose({yoke_to_can: 0.65}):
        tilted_gel_center = ctx.part_world_position(gel_frame)

    ctx.check(
        "positive tilt raises the can nose",
        rest_gel_center is not None
        and tilted_gel_center is not None
        and tilted_gel_center[2] > rest_gel_center[2] + 0.08,
        details=f"rest={rest_gel_center}, tilted={tilted_gel_center}",
    )

    rest_front_pos = ctx.part_world_position(gel_frame)
    with ctx.pose({column_to_yoke: 0.70}):
        panned_front_pos = ctx.part_world_position(gel_frame)

    ctx.check(
        "yoke pans about the stand head",
        rest_front_pos is not None
        and panned_front_pos is not None
        and abs(panned_front_pos[0] - rest_front_pos[0]) > 0.08,
        details=f"rest={rest_front_pos}, panned={panned_front_pos}",
    )

    ctx.expect_gap(
        yoke,
        column,
        axis="z",
        positive_elem="pan_hub",
        negative_elem="pan_spigot",
        max_gap=0.015,
        max_penetration=0.0,
        name="pan head seats tightly on spigot",
    )

    return ctx.report()


object_model = build_object_model()
