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
)


BODY_WIDTH = 0.145
BODY_DEPTH = 0.120
BODY_HEIGHT = 0.180
BODY_BOTTOM_Z = 0.012
CRANK_AXLE = (BODY_WIDTH / 2 + 0.018, 0.012, BODY_BOTTOM_Z + 0.112)


def _cylinder_along_y(radius: float, length: float) -> cq.Workplane:
    """CadQuery cylinder centered at the origin with its axis along world Y."""
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _make_port_liner() -> cq.Workplane:
    tube = _cylinder_along_y(0.019, 0.060).cut(_cylinder_along_y(0.0125, 0.064))
    tube = tube.translate((0.0, -BODY_DEPTH / 2 + 0.030, BODY_BOTTOM_Z + 0.118))

    flange = _cylinder_along_y(0.029, 0.006).cut(_cylinder_along_y(0.0135, 0.008))
    flange = flange.translate((0.0, -BODY_DEPTH / 2 - 0.003, BODY_BOTTOM_Z + 0.118))
    return tube.union(flange)


def _make_drawer_tray() -> cq.Workplane:
    # Local drawer frame: center of the sliding tray in its fully inserted pose.
    floor = cq.Workplane("XY").box(0.094, 0.070, 0.004).translate((0.0, 0.0, -0.020))
    side_a = cq.Workplane("XY").box(0.004, 0.070, 0.034).translate((0.049, 0.0, -0.001))
    side_b = cq.Workplane("XY").box(0.004, 0.070, 0.034).translate((-0.049, 0.0, -0.001))
    back = cq.Workplane("XY").box(0.098, 0.004, 0.034).translate((0.0, 0.033, -0.001))
    front = cq.Workplane("XY").box(0.106, 0.006, 0.042).translate((0.0, -0.038, 0.000))
    return floor.union(side_a).union(side_b).union(back).union(front)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_desktop_pencil_sharpener")

    cast_green = model.material("aged_green_cast_metal", rgba=(0.20, 0.33, 0.27, 1.0))
    dark_iron = model.material("japanned_black_iron", rgba=(0.015, 0.014, 0.012, 1.0))
    warm_steel = model.material("worn_bright_steel", rgba=(0.62, 0.60, 0.54, 1.0))
    brass = model.material("aged_brass", rgba=(0.78, 0.57, 0.25, 1.0))
    bakelite = model.material("dark_bakelite", rgba=(0.055, 0.036, 0.023, 1.0))
    rubber = model.material("old_black_rubber", rgba=(0.02, 0.018, 0.016, 1.0))

    housing = model.part("housing")
    cast_boxes = [
        ("lower_side_0", (0.019, BODY_DEPTH, 0.054), (-0.064, 0.0, BODY_BOTTOM_Z + 0.027)),
        ("lower_side_1", (0.019, BODY_DEPTH, 0.054), (0.064, 0.0, BODY_BOTTOM_Z + 0.027)),
        ("lower_back", (BODY_WIDTH, 0.019, 0.054), (0.0, BODY_DEPTH / 2 - 0.0095, BODY_BOTTOM_Z + 0.027)),
        ("lower_top_rail", (BODY_WIDTH, BODY_DEPTH, 0.012), (0.0, 0.0, BODY_BOTTOM_Z + 0.060)),
        ("upper_side_0", (0.020, BODY_DEPTH, 0.114), (-0.0625, 0.0, BODY_BOTTOM_Z + 0.123)),
        ("upper_side_1", (0.020, BODY_DEPTH, 0.114), (0.0625, 0.0, BODY_BOTTOM_Z + 0.123)),
        ("upper_back", (BODY_WIDTH, 0.020, 0.114), (0.0, BODY_DEPTH / 2 - 0.010, BODY_BOTTOM_Z + 0.123)),
        ("front_panel_0", (0.045, 0.006, 0.089), (-0.050, -BODY_DEPTH / 2 - 0.003, BODY_BOTTOM_Z + 0.1105)),
        ("front_panel_1", (0.045, 0.006, 0.089), (0.050, -BODY_DEPTH / 2 - 0.003, BODY_BOTTOM_Z + 0.1105)),
        ("front_panel_lower", (0.145, 0.006, 0.022), (0.0, -BODY_DEPTH / 2 - 0.003, BODY_BOTTOM_Z + 0.077)),
        ("front_panel_upper", (0.145, 0.006, 0.014), (0.0, -BODY_DEPTH / 2 - 0.003, BODY_BOTTOM_Z + 0.173)),
        ("rounded_top_cap", (BODY_WIDTH, BODY_DEPTH, 0.026), (0.0, 0.0, BODY_BOTTOM_Z + BODY_HEIGHT - 0.013)),
        ("drawer_rail_0", (0.014, 0.116, 0.008), (-0.036, 0.002, 0.016)),
        ("drawer_rail_1", (0.014, 0.116, 0.008), (0.036, 0.002, 0.016)),
    ]
    for name, size, xyz in cast_boxes:
        housing.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=cast_green,
            name=name,
        )
    housing.visual(
        Box((0.190, 0.145, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_iron,
        name="desk_base",
    )
    for index, (x, y) in enumerate(
        ((-0.072, -0.052), (-0.072, 0.052), (0.072, -0.052), (0.072, 0.052))
    ):
        housing.visual(
            Cylinder(radius=0.012, length=0.005),
            origin=Origin(xyz=(x, y, 0.0025)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )
    housing.visual(
        mesh_from_cadquery(_make_port_liner(), "pencil_port_liner"),
        material=brass,
        name="pencil_port_liner",
    )
    housing.visual(
        Box((0.070, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2 - 0.0015, BODY_BOTTOM_Z + 0.077)),
        material=brass,
        name="front_nameplate",
    )
    housing.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(BODY_WIDTH / 2 + 0.009, CRANK_AXLE[1], CRANK_AXLE[2]), rpy=(0.0, math.pi / 2, 0.0)),
        material=warm_steel,
        name="side_bearing",
    )
    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_make_drawer_tray(), "shavings_drawer"),
        material=dark_iron,
        name="shavings_drawer",
    )
    drawer.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, -0.046, 0.002), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brass,
        name="drawer_pull",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=warm_steel,
        name="crank_hub",
    )
    handle_dy = 0.055
    handle_dz = -0.065
    handle_len = math.hypot(handle_dy, handle_dz)
    handle_angle = math.atan2(handle_dz, handle_dy)
    crank.visual(
        Box((0.010, handle_len, 0.010)),
        origin=Origin(
            xyz=(0.029, handle_dy / 2, handle_dz / 2),
            rpy=(handle_angle, 0.0, 0.0),
        ),
        material=warm_steel,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.029, handle_dy, handle_dz), rpy=(0.0, math.pi / 2, 0.0)),
        material=warm_steel,
        name="knob_mount",
    )
    crank.visual(
        Cylinder(radius=0.005, length=0.007),
        origin=Origin(xyz=(0.0375, handle_dy, handle_dz), rpy=(0.0, math.pi / 2, 0.0)),
        material=brass,
        name="knob_pin",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=bakelite,
        name="knob_barrel",
    )
    knob.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
        material=bakelite,
        name="knob_inner_round",
    )
    knob.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=bakelite,
        name="knob_outer_round",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.022, BODY_BOTTOM_Z + 0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.18, lower=0.0, upper=0.055),
    )
    model.articulation(
        "crank_axle",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crank,
        origin=Origin(xyz=CRANK_AXLE),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "knob_axle",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=knob,
        origin=Origin(xyz=(0.070, handle_dy, handle_dz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    knob = object_model.get_part("knob")
    drawer_slide = object_model.get_articulation("drawer_slide")
    crank_axle = object_model.get_articulation("crank_axle")
    knob_axle = object_model.get_articulation("knob_axle")

    ctx.check(
        "primary mechanisms articulated",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC
        and crank_axle.articulation_type == ArticulationType.CONTINUOUS
        and knob_axle.articulation_type == ArticulationType.CONTINUOUS,
        details="Expected a prismatic drawer slide plus continuous crank and knob axles.",
    )
    ctx.expect_contact(
        knob,
        crank,
        elem_a="knob_inner_round",
        elem_b="knob_pin",
        contact_tol=0.002,
        name="free knob is carried by the handle pin",
    )
    ctx.expect_within(
        drawer,
        housing,
        axes="xz",
        inner_elem="shavings_drawer",
        margin=0.004,
        name="drawer fits in the front opening",
    )
    ctx.expect_overlap(
        drawer,
        housing,
        axes="y",
        elem_a="shavings_drawer",
        min_overlap=0.040,
        name="closed drawer remains deeply inserted",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.055}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            housing,
            axes="y",
            elem_a="shavings_drawer",
            min_overlap=0.015,
            name="extended drawer retains rear insertion",
        )
    ctx.check(
        "drawer slides out toward the front",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < closed_drawer_pos[1] - 0.045,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    crank_rest = ctx.part_world_position(knob)
    with ctx.pose({crank_axle: math.pi}):
        crank_half_turn = ctx.part_world_position(knob)
    ctx.check(
        "crank carries knob around side axle",
        crank_rest is not None
        and crank_half_turn is not None
        and abs(crank_half_turn[2] - crank_rest[2]) > 0.09,
        details=f"rest={crank_rest}, half_turn={crank_half_turn}",
    )

    port_aabb = ctx.part_element_world_aabb(housing, elem="pencil_port_liner")
    ctx.check(
        "pencil port has visible tunnel depth",
        port_aabb is not None and (port_aabb[1][1] - port_aabb[0][1]) > 0.055,
        details=f"port_aabb={port_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
