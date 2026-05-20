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


def _trumpet_base_shape() -> cq.Workplane:
    """Chrome saucer/trumpet base with a real hollow bore for the gas-lift ram."""

    foot = cq.Workplane("XY").circle(0.29).extrude(0.035)
    flare = (
        cq.Workplane("XY")
        .circle(0.23)
        .workplane(offset=0.22)
        .circle(0.065)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.025))
    )
    sleeve = cq.Workplane("XY").circle(0.050).extrude(0.39).translate((0.0, 0.0, 0.235))
    top_lip = cq.Workplane("XY").circle(0.062).extrude(0.025).translate((0.0, 0.0, 0.600))

    solid = foot.union(flare).union(sleeve).union(top_lip)
    bore = cq.Workplane("XY").circle(0.036).extrude(0.68).translate((0.0, 0.0, 0.060))
    return solid.cut(bore)


def _rubber_foot_ring_shape() -> cq.Workplane:
    ring = cq.Workplane("XY").circle(0.292).extrude(0.012)
    center_cut = cq.Workplane("XY").circle(0.245).extrude(0.016).translate((0.0, 0.0, -0.002))
    return ring.cut(center_cut)


def _seat_cushion_shape() -> cq.Workplane:
    cushion = cq.Workplane("XY").circle(0.245).extrude(0.092)
    return cushion.edges().fillet(0.018).translate((0.0, 0.0, 0.018))


def _backrest_pad_shape() -> cq.Workplane:
    pad = cq.Workplane("XY").box(0.405, 0.045, 0.220)
    return pad.edges().fillet(0.018).translate((0.0, 0.035, 0.165))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_trumpet_bar_stool")

    chrome = model.material("polished_chrome", rgba=(0.72, 0.75, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    black_vinyl = model.material("black_vinyl", rgba=(0.015, 0.014, 0.012, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_trumpet_base_shape(), "trumpet_base"),
        material=chrome,
        name="chrome_shell",
    )
    base.visual(
        mesh_from_cadquery(_rubber_foot_ring_shape(), "rubber_floor_ring"),
        material=rubber,
        name="floor_ring",
    )

    lift_column = model.part("lift_column")
    lift_column.visual(
        Cylinder(radius=0.027, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, -0.200)),
        material=chrome,
        name="inner_piston",
    )
    lift_column.visual(
        Cylinder(radius=0.039, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=chrome,
        name="guide_bushing",
    )
    lift_column.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=chrome,
        name="gas_collar",
    )
    lift_column.visual(
        Cylinder(radius=0.128, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
        material=dark_steel,
        name="swivel_plate",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_seat_cushion_shape(), "rounded_seat_cushion"),
        material=black_vinyl,
        name="cushion",
    )
    seat.visual(
        Cylinder(radius=0.145, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="bearing_plate",
    )
    seat.visual(
        Box((0.110, 0.045, 0.025)),
        origin=Origin(xyz=(0.195, 0.0, -0.0125)),
        material=dark_steel,
        name="lever_stem",
    )
    seat.visual(
        Box((0.036, 0.012, 0.055)),
        origin=Origin(xyz=(0.235, 0.022, -0.035)),
        material=dark_steel,
        name="lever_yoke_0",
    )
    seat.visual(
        Box((0.036, 0.012, 0.055)),
        origin=Origin(xyz=(0.235, -0.022, -0.035)),
        material=dark_steel,
        name="lever_yoke_1",
    )
    for x, name in ((-0.150, "hinge_support_0"), (0.150, "hinge_support_1")):
        seat.visual(
            Box((0.052, 0.185, 0.028)),
            origin=Origin(xyz=(x, 0.155, 0.020)),
            material=dark_steel,
            name=f"rear_rail_{0 if x < 0 else 1}",
        )
        seat.visual(
            Box((0.052, 0.035, 0.088)),
            origin=Origin(xyz=(x, 0.240, 0.055)),
            material=dark_steel,
            name=name,
        )
        seat.visual(
            Cylinder(radius=0.014, length=0.080),
            origin=Origin(xyz=(x, 0.255, 0.096), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"hinge_knuckle_{0 if x < 0 else 1}",
        )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_pin",
    )
    lever.visual(
        Box((0.180, 0.022, 0.018)),
        origin=Origin(xyz=(0.095, 0.0, -0.012), rpy=(0.0, 0.18, 0.0)),
        material=dark_steel,
        name="pull_handle",
    )
    lever.visual(
        Box((0.042, 0.050, 0.018)),
        origin=Origin(xyz=(0.188, 0.0, -0.030), rpy=(0.0, 0.18, 0.0)),
        material=rubber,
        name="finger_paddle",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.013, length=0.170),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    backrest.visual(
        Box((0.190, 0.026, 0.085)),
        origin=Origin(xyz=(0.0, 0.018, 0.042)),
        material=dark_steel,
        name="hinge_leaf",
    )
    backrest.visual(
        mesh_from_cadquery(_backrest_pad_shape(), "short_backrest_pad"),
        material=black_vinyl,
        name="pad",
    )

    model.articulation(
        "base_to_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_column,
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.180, effort=250.0, velocity=0.12),
        motion_properties=MotionProperties(damping=18.0, friction=4.0),
    )
    model.articulation(
        "lift_to_seat",
        ArticulationType.CONTINUOUS,
        parent=lift_column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5),
        motion_properties=MotionProperties(damping=0.35, friction=0.08),
    )
    model.articulation(
        "seat_to_lever",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lever,
        origin=Origin(xyz=(0.235, 0.0, -0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.25, upper=0.45, effort=8.0, velocity=2.0),
        motion_properties=MotionProperties(damping=1.6, friction=0.25),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(0.0, 0.255, 0.096)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.12, upper=0.38, effort=18.0, velocity=0.9),
        motion_properties=MotionProperties(damping=0.9, friction=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lift_column = object_model.get_part("lift_column")
    seat = object_model.get_part("seat")
    lever = object_model.get_part("lever")
    backrest = object_model.get_part("backrest")
    lift = object_model.get_articulation("base_to_lift")
    swivel = object_model.get_articulation("lift_to_seat")
    lever_joint = object_model.get_articulation("seat_to_lever")
    back_hinge = object_model.get_articulation("seat_to_backrest")

    ctx.allow_overlap(
        base,
        lift_column,
        elem_a="chrome_shell",
        elem_b="guide_bushing",
        reason="The sliding guide bushing is intentionally captured inside the sleeve lip of the gas-lift column.",
    )
    for yoke in ("lever_yoke_0", "lever_yoke_1"):
        ctx.allow_overlap(
            seat,
            lever,
            elem_a=yoke,
            elem_b="pivot_pin",
            reason="The lever pivot pin is intentionally seated through the short yoke ears under the seat.",
        )

    ctx.expect_contact(
        seat,
        lift_column,
        elem_a="bearing_plate",
        elem_b="swivel_plate",
        contact_tol=0.001,
        name="seat bearing rests on lift plate",
    )
    ctx.expect_within(
        lift_column,
        base,
        axes="xy",
        inner_elem="inner_piston",
        outer_elem="chrome_shell",
        margin=0.0,
        name="piston centered in trumpet base footprint",
    )
    ctx.expect_overlap(
        lift_column,
        base,
        axes="z",
        elem_a="inner_piston",
        elem_b="chrome_shell",
        min_overlap=0.250,
        name="collapsed gas lift remains inserted",
    )
    ctx.expect_overlap(
        lift_column,
        base,
        axes="z",
        elem_a="guide_bushing",
        elem_b="chrome_shell",
        min_overlap=0.010,
        name="guide bushing is retained by sleeve lip",
    )
    ctx.expect_gap(
        seat,
        lever,
        axis="y",
        positive_elem="lever_yoke_0",
        negative_elem="pivot_pin",
        max_penetration=0.003,
        name="positive yoke lightly captures pivot pin",
    )
    ctx.expect_gap(
        lever,
        seat,
        axis="y",
        positive_elem="pivot_pin",
        negative_elem="lever_yoke_1",
        max_penetration=0.003,
        name="negative yoke lightly captures pivot pin",
    )

    low_pos = ctx.part_world_position(seat)
    with ctx.pose({lift: 0.180}):
        high_pos = ctx.part_world_position(seat)
        ctx.expect_overlap(
            lift_column,
            base,
            axes="z",
            elem_a="inner_piston",
            elem_b="chrome_shell",
            min_overlap=0.250,
            name="extended gas lift remains inserted",
        )
    ctx.check(
        "height slider raises the seat",
        low_pos is not None and high_pos is not None and high_pos[2] > low_pos[2] + 0.150,
        details=f"low={low_pos}, high={high_pos}",
    )

    rest_back = ctx.part_world_position(backrest)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_back = ctx.part_world_position(backrest)
    ctx.check(
        "seat swivel carries the backrest around the pedestal",
        rest_back is not None
        and turned_back is not None
        and abs(turned_back[0] + rest_back[1]) < 0.020
        and abs(turned_back[1] - rest_back[0]) < 0.020,
        details=f"rest={rest_back}, turned={turned_back}",
    )

    rest_handle = ctx.part_element_world_aabb(lever, elem="pull_handle")
    with ctx.pose({lever_joint: 0.40}):
        pulled_handle = ctx.part_element_world_aabb(lever, elem="pull_handle")
    ctx.check(
        "damped lever pivots downward under the seat",
        rest_handle is not None
        and pulled_handle is not None
        and pulled_handle[0][2] < rest_handle[0][2] - 0.030,
        details=f"rest={rest_handle}, pulled={pulled_handle}",
    )

    rest_pad = ctx.part_element_world_aabb(backrest, elem="pad")
    with ctx.pose({back_hinge: 0.32}):
        tilted_pad = ctx.part_element_world_aabb(backrest, elem="pad")
    ctx.check(
        "short backrest tilts rearward on its hinge",
        rest_pad is not None and tilted_pad is not None and tilted_pad[1][1] > rest_pad[1][1] + 0.030,
        details=f"rest={rest_pad}, tilted={tilted_pad}",
    )

    return ctx.report()


object_model = build_object_model()
