from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_base(width: float, depth: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Z")
        .fillet(0.025)
    )


def _hollow_tube(
    length: float,
    outer_radius: float,
    inner_radius: float,
    *,
    feed_window: bool = False,
) -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length, both=True)
    inner = cq.Workplane("YZ").circle(inner_radius).extrude(length + 0.030, both=True)
    tube = outer.cut(inner)
    if feed_window:
        port = (
            cq.Workplane("XY")
            .box(0.150, 0.120, outer_radius * 1.7)
            .translate((0.0, 0.0, outer_radius * 0.80))
        )
        tube = tube.cut(port)
    return tube


def _hopper_funnel(
    bottom_size: tuple[float, float],
    top_size: tuple[float, float],
    height: float,
    wall: float,
) -> cq.Workplane:
    bottom_x, bottom_y = bottom_size
    top_x, top_y = top_size
    outer = (
        cq.Workplane("XY")
        .rect(bottom_x, bottom_y)
        .workplane(offset=height)
        .rect(top_x, top_y)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .rect(bottom_x - 2.0 * wall, bottom_y - 2.0 * wall)
        .workplane(offset=height + 0.012)
        .rect(top_x - 2.0 * wall, top_y - 2.0 * wall)
        .loft(combine=True)
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cold_press_slow_juicer")

    white_plastic = model.material("warm_white_plastic", rgba=(0.92, 0.91, 0.86, 1.0))
    dark_plastic = model.material("matte_black_plastic", rgba=(0.025, 0.025, 0.028, 1.0))
    smoked_clear = model.material("smoked_clear_plastic", rgba=(0.50, 0.68, 0.74, 0.38))
    auger_black = model.material("satin_black_auger", rgba=(0.015, 0.018, 0.020, 1.0))
    brushed_metal = model.material("brushed_steel", rgba=(0.58, 0.58, 0.55, 1.0))
    amber = model.material("amber_juice", rgba=(0.95, 0.58, 0.12, 0.70))

    body = model.part("body")

    body.visual(
        mesh_from_cadquery(_rounded_base(0.54, 0.32, 0.18), "rounded_motor_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=white_plastic,
        name="motor_base",
    )
    body.visual(
        Box((0.42, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.165, 0.160)),
        material=dark_plastic,
        name="front_trim",
    )
    body.visual(
        Box((0.125, 0.018, 0.080)),
        origin=Origin(xyz=(-0.185, -0.165, 0.095)),
        material=dark_plastic,
        name="control_panel",
    )

    chamber_length = 0.46
    chamber_outer = 0.085
    chamber_inner = 0.074
    chamber_center = (0.0, -0.020, 0.255)
    body.visual(
        mesh_from_cadquery(
            _hollow_tube(chamber_length, chamber_outer, chamber_inner, feed_window=True),
            "clear_auger_chamber",
            tolerance=0.0008,
        ),
        origin=Origin(xyz=chamber_center),
        material=smoked_clear,
        name="chamber_shell",
    )
    body.visual(
        mesh_from_cadquery(_hollow_tube(0.030, 0.092, chamber_inner), "front_chamber_band"),
        origin=Origin(xyz=(-0.245, chamber_center[1], chamber_center[2])),
        material=dark_plastic,
        name="chamber_band_0",
    )
    body.visual(
        mesh_from_cadquery(_hollow_tube(0.030, 0.092, chamber_inner), "rear_chamber_band"),
        origin=Origin(xyz=(0.245, chamber_center[1], chamber_center[2])),
        material=dark_plastic,
        name="chamber_band_1",
    )
    for i, x in enumerate((-0.232, 0.232)):
        body.visual(
            Cylinder(radius=0.028, length=0.020),
            origin=Origin(xyz=(x, chamber_center[1], chamber_center[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_metal,
            name=f"bearing_socket_{i}",
        )
        body.visual(
            Box((0.018, 0.014, 0.060)),
            origin=Origin(xyz=(x, chamber_center[1], chamber_center[2] + 0.052)),
            material=brushed_metal,
            name=f"bearing_web_{i}",
        )

    body.visual(
        Box((0.070, 0.100, 0.032)),
        origin=Origin(xyz=(-0.125, -0.112, 0.205), rpy=(0.0, 0.0, -0.15)),
        material=dark_plastic,
        name="juice_spout",
    )
    body.visual(
        Box((0.058, 0.082, 0.010)),
        origin=Origin(xyz=(-0.125, -0.162, 0.180), rpy=(0.0, 0.0, -0.15)),
        material=amber,
        name="spout_opening",
    )

    body.visual(
        mesh_from_cadquery(
            _hopper_funnel((0.120, 0.085), (0.290, 0.225), 0.200, 0.010),
            "wide_hopper_funnel",
            tolerance=0.0008,
        ),
        origin=Origin(xyz=(0.0, -0.020, 0.335)),
        material=smoked_clear,
        name="hopper_shell",
    )
    body.visual(
        Box((0.150, 0.012, 0.080)),
        origin=Origin(xyz=(0.0, -0.072, 0.373)),
        material=smoked_clear,
        name="feed_neck_front",
    )
    body.visual(
        Box((0.150, 0.012, 0.080)),
        origin=Origin(xyz=(0.0, 0.032, 0.373)),
        material=smoked_clear,
        name="feed_neck_back",
    )
    body.visual(
        Box((0.012, 0.104, 0.080)),
        origin=Origin(xyz=(-0.075, -0.020, 0.373)),
        material=smoked_clear,
        name="feed_neck_side_0",
    )
    body.visual(
        Box((0.012, 0.104, 0.080)),
        origin=Origin(xyz=(0.075, -0.020, 0.373)),
        material=smoked_clear,
        name="feed_neck_side_1",
    )
    body.visual(
        Box((0.305, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.100, 0.532)),
        material=dark_plastic,
        name="back_hopper_rim",
    )

    hinge_y = 0.100
    hinge_z = 0.556
    for i, x in enumerate((-0.108, 0.108)):
        body.visual(
            Box((0.060, 0.020, 0.036)),
            origin=Origin(xyz=(x, hinge_y + 0.006, 0.529)),
            material=dark_plastic,
            name=f"hinge_leaf_{i}",
        )
        body.visual(
            Cylinder(radius=0.014, length=0.055),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_plastic,
            name=f"hinge_barrel_{i}",
        )

    auger = model.part("auger")
    auger.visual(
        Cylinder(radius=0.018, length=0.480),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="auger_shaft",
    )
    flight_radius_center = 0.034
    for i in range(13):
        x = -0.180 + i * 0.030
        angle = i * 0.82
        auger.visual(
            Box((0.032, 0.010, 0.054)),
            origin=Origin(
                xyz=(
                    x,
                    -flight_radius_center * math.sin(angle),
                    flight_radius_center * math.cos(angle),
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            material=auger_black,
            name=f"auger_flight_{i}",
        )
    model.articulation(
        "body_to_auger",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=auger,
        origin=Origin(xyz=chamber_center),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.8),
        motion_properties=MotionProperties(damping=0.15, friction=0.04),
    )

    lid = model.part("hopper_lid")
    lid.visual(
        Box((0.310, 0.245, 0.014)),
        origin=Origin(xyz=(0.0, -0.1425, -0.011)),
        material=smoked_clear,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="lid_barrel",
    )
    lid.visual(
        Box((0.120, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.010, -0.008)),
        material=dark_plastic,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((0.130, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.224, 0.003)),
        material=dark_plastic,
        name="front_lip",
    )
    model.articulation(
        "body_to_hopper_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_plastic,
        name="knob_cap",
    )
    control_knob.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=dark_plastic,
        name="knob_stem",
    )
    control_knob.visual(
        Box((0.007, 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=white_plastic,
        name="pointer_mark",
    )
    model.articulation(
        "body_to_control_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=control_knob,
        origin=Origin(xyz=(-0.185, -0.174, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    auger = object_model.get_part("auger")
    lid = object_model.get_part("hopper_lid")
    control_knob = object_model.get_part("control_knob")
    lid_hinge = object_model.get_articulation("body_to_hopper_lid")
    auger_joint = object_model.get_articulation("body_to_auger")

    ctx.allow_overlap(
        control_knob,
        body,
        elem_a="knob_stem",
        elem_b="control_panel",
        reason="The small knob stem is intentionally seated into the front control panel.",
    )
    ctx.expect_within(
        control_knob,
        body,
        axes="xz",
        inner_elem="knob_stem",
        outer_elem="control_panel",
        margin=0.001,
        name="control knob stem is centered in its panel boss",
    )
    ctx.expect_overlap(
        control_knob,
        body,
        axes="y",
        elem_a="knob_stem",
        elem_b="control_panel",
        min_overlap=0.004,
        name="control knob stem is retained in the panel",
    )
    for socket_name in ("bearing_socket_0", "bearing_socket_1"):
        ctx.allow_overlap(
            auger,
            body,
            elem_a="auger_shaft",
            elem_b=socket_name,
            reason="The slow auger shaft is intentionally captured inside a fixed end bearing socket.",
        )
        ctx.expect_within(
            auger,
            body,
            axes="yz",
            inner_elem="auger_shaft",
            outer_elem=socket_name,
            margin=0.001,
            name=f"auger shaft centered in {socket_name}",
        )
        ctx.expect_overlap(
            auger,
            body,
            axes="x",
            elem_a="auger_shaft",
            elem_b=socket_name,
            min_overlap=0.008,
            name=f"auger shaft retained by {socket_name}",
        )

    ctx.check(
        "auger uses a continuous horizontal joint",
        auger_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in auger_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={auger_joint.articulation_type}, axis={auger_joint.axis}",
    )

    ctx.expect_within(
        auger,
        body,
        axes="yz",
        inner_elem="auger_shaft",
        outer_elem="chamber_shell",
        margin=0.004,
        name="auger shaft sits inside the chamber bore",
    )
    ctx.expect_overlap(
        auger,
        body,
        axes="x",
        elem_a="auger_shaft",
        elem_b="chamber_shell",
        min_overlap=0.36,
        name="auger spans the chamber length",
    )

    with ctx.pose({lid_hinge: 0.0}):
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="hopper_shell",
            min_gap=0.0,
            max_gap=0.010,
            name="closed lid rests just above the hopper rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="hopper_shell",
            min_overlap=0.18,
            name="closed lid covers the wide rectangular hopper opening",
        )
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "hopper lid opens upward on its rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
