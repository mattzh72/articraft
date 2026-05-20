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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PAN_HEIGHT = 0.340
TILT_CENTER = (0.020, 0.0, 0.110)
CAN_REAR_X = -0.070
CAN_FRONT_X = 0.100
FILTER_HINGE_X = 0.111
FILTER_HINGE_Z = -0.060


def _annular_cylinder_x(
    length: float,
    outer_radius: float,
    inner_radius: float,
    *,
    center_x: float,
) -> cq.Workplane:
    """A hollow cylinder whose axis is local X."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((center_x - length / 2.0, 0.0, 0.0))
    )


def _can_shell_shape() -> cq.Workplane:
    """Short studio spotlight can with an open front lip and rear service ring."""
    body = _annular_cylinder_x(
        0.152,
        0.064,
        0.055,
        center_x=0.008,
    )
    front_bezel = _annular_cylinder_x(
        0.020,
        0.072,
        0.050,
        center_x=0.090,
    )
    rear_ring = _annular_cylinder_x(
        0.010,
        0.062,
        0.018,
        center_x=-0.067,
    )
    return body.union(front_bezel).union(rear_ring)


def _front_lug_shape(y: float) -> cq.Workplane:
    """One outer knuckle and its welded tab for the filter-holder hinge."""
    knuckle = (
        cq.Workplane("XZ")
        .circle(0.0055)
        .extrude(0.028)
        .translate((FILTER_HINGE_X, y + 0.014, FILTER_HINGE_Z))
    )
    tab = (
        cq.Workplane("XY")
        .box(0.018, 0.018, 0.014)
        .translate((0.101, y, FILTER_HINGE_Z + 0.007))
    )
    return tab.union(knuckle)


def _filter_frame_shape() -> cq.Workplane:
    """Connected rectangular filter holder frame, hinged along its bottom edge."""
    rail_thickness_x = 0.006
    frame = cq.Workplane("XY").box(rail_thickness_x, 0.128, 0.010).translate((0.004, 0.0, 0.006))
    frame = frame.union(
        cq.Workplane("XY").box(rail_thickness_x, 0.010, 0.116).translate((0.004, -0.064, 0.060))
    )
    frame = frame.union(
        cq.Workplane("XY").box(rail_thickness_x, 0.010, 0.116).translate((0.004, 0.064, 0.060))
    )
    frame = frame.union(
        cq.Workplane("XY").box(rail_thickness_x, 0.128, 0.010).translate((0.004, 0.0, 0.114))
    )
    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_studio_spotlight")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.12, 0.125, 0.135, 1.0))
    worn_edge = model.material("worn_edge", rgba=(0.33, 0.34, 0.35, 1.0))
    warm_reflector = model.material("warm_reflector", rgba=(0.92, 0.78, 0.48, 1.0))
    cool_filter = model.material("cool_filter", rgba=(0.25, 0.55, 0.95, 0.45))
    rubber = model.material("rubber", rgba=(0.025, 0.025, 0.026, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.092, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=satin_black,
        name="weighted_base",
    )
    stand.visual(
        Cylinder(radius=0.080, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber,
        name="rubber_foot",
    )
    stand.visual(
        Cylinder(radius=0.013, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
        material=dark_graphite,
        name="upright_post",
    )
    stand.visual(
        Cylinder(radius=0.026, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.309)),
        material=satin_black,
        name="head_socket",
    )
    stand.visual(
        Cylinder(radius=0.044, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.334)),
        material=worn_edge,
        name="pan_bearing_plate",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.043, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=worn_edge,
        name="turntable",
    )
    yoke.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin_black,
        name="pan_neck",
    )
    yoke.visual(
        Box((0.070, 0.186, 0.020)),
        origin=Origin(xyz=(0.010, 0.0, 0.027)),
        material=satin_black,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.040, 0.018, 0.150)),
        origin=Origin(xyz=(0.020, -0.088, 0.112)),
        material=satin_black,
        name="arm_0",
    )
    yoke.visual(
        Box((0.040, 0.018, 0.150)),
        origin=Origin(xyz=(0.020, 0.088, 0.112)),
        material=satin_black,
        name="arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.021, length=0.012),
        origin=Origin(xyz=(TILT_CENTER[0], -0.073, TILT_CENTER[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_edge,
        name="tilt_boss_0",
    )
    yoke.visual(
        Cylinder(radius=0.021, length=0.012),
        origin=Origin(xyz=(TILT_CENTER[0], 0.073, TILT_CENTER[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_edge,
        name="tilt_boss_1",
    )
    yoke.visual(
        Box((0.086, 0.020, 0.014)),
        origin=Origin(xyz=(-0.020, 0.0, 0.030)),
        material=satin_black,
        name="rear_stiffener",
    )

    can = model.part("can")
    can.visual(
        mesh_from_cadquery(_can_shell_shape(), "spotlight_can_shell", tolerance=0.0007),
        material=satin_black,
        name="can_shell",
    )
    can.visual(
        Cylinder(radius=0.011, length=0.134),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_edge,
        name="tilt_pin",
    )
    can.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(0.083, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_reflector,
        name="warm_lens",
    )
    can.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(-0.079, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="rear_cable_gland",
    )
    can.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(-0.071, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="rear_cable_collar",
    )
    for index, y in enumerate((-0.058, 0.058)):
        can.visual(
            mesh_from_cadquery(_front_lug_shape(y), f"front_hinge_lug_{index}", tolerance=0.0005),
            material=worn_edge,
            name=f"front_lug_{index}",
        )

    filter_holder = model.part("filter_holder")
    filter_holder.visual(
        mesh_from_cadquery(_filter_frame_shape(), "filter_frame", tolerance=0.0005),
        material=satin_black,
        name="filter_frame",
    )
    filter_holder.visual(
        Cylinder(radius=0.0053, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_edge,
        name="hinge_knuckle",
    )
    filter_holder.visual(
        Box((0.0025, 0.118, 0.104)),
        origin=Origin(xyz=(0.002, 0.0, 0.060)),
        material=cool_filter,
        name="blue_filter",
    )

    model.articulation(
        "stand_to_yoke",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=TILT_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "can_to_filter_holder",
        ArticulationType.REVOLUTE,
        parent=can,
        child=filter_holder,
        origin=Origin(xyz=(FILTER_HINGE_X, 0.0, FILTER_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    holder = object_model.get_part("filter_holder")
    pan = object_model.get_articulation("stand_to_yoke")
    tilt = object_model.get_articulation("yoke_to_can")
    filter_hinge = object_model.get_articulation("can_to_filter_holder")

    ctx.expect_contact(
        stand,
        yoke,
        elem_a="pan_bearing_plate",
        elem_b="turntable",
        contact_tol=0.001,
        name="yoke turntable sits on stand bearing",
    )
    ctx.expect_contact(
        yoke,
        can,
        elem_a="tilt_boss_0",
        elem_b="tilt_pin",
        contact_tol=0.002,
        name="can tilt pin reaches yoke boss",
    )
    ctx.expect_gap(
        holder,
        can,
        axis="x",
        positive_elem="filter_frame",
        negative_elem="can_shell",
        min_gap=0.004,
        max_gap=0.025,
        name="filter frame sits just in front of front ring",
    )

    rest_holder_aabb = ctx.part_element_world_aabb(holder, elem="filter_frame")
    with ctx.pose({filter_hinge: 1.20}):
        lowered_holder_aabb = ctx.part_element_world_aabb(holder, elem="filter_frame")
        ctx.expect_gap(
            holder,
            can,
            axis="x",
            positive_elem="filter_frame",
            negative_elem="can_shell",
            min_gap=0.0,
            name="lowered filter clears can face",
        )

    with ctx.pose({pan: 0.65, tilt: 0.55}):
        ctx.expect_origin_distance(can, stand, axes="xy", min_dist=0.005, name="panned can is offset from stand")

    ctx.check(
        "filter holder hinge lowers the frame",
        rest_holder_aabb is not None
        and lowered_holder_aabb is not None
        and lowered_holder_aabb[1][2] < rest_holder_aabb[1][2] - 0.020,
        details=f"rest={rest_holder_aabb}, lowered={lowered_holder_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
