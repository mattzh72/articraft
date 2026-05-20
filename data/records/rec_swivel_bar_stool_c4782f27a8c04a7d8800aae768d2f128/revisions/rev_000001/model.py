from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_bar_stool")

    steel = model.material("steel", rgba=(0.66, 0.68, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.24, 1.0))
    cushion_vinyl = model.material("cushion_vinyl", rgba=(0.13, 0.13, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.23, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_steel,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.038, length=0.57),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=steel,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=steel,
        name="ring_collar",
    )
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.16, tube=0.01), "foot_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=steel,
        name="foot_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.007, length=0.11),
        origin=Origin(xyz=(0.105, 0.0, 0.26), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="ring_strut_x_pos",
    )
    pedestal.visual(
        Cylinder(radius=0.007, length=0.11),
        origin=Origin(xyz=(-0.105, 0.0, 0.26), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="ring_strut_x_neg",
    )
    pedestal.visual(
        Cylinder(radius=0.007, length=0.11),
        origin=Origin(xyz=(0.0, 0.105, 0.26), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="ring_strut_y_pos",
    )
    pedestal.visual(
        Cylinder(radius=0.007, length=0.11),
        origin=Origin(xyz=(0.0, -0.105, 0.26), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="ring_strut_y_neg",
    )
    pedestal.visual(
        Cylinder(radius=0.05, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.606)),
        material=steel,
        name="top_collar",
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.18, 0.18, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="mount_plate",
    )
    seat.visual(
        Box((0.34, 0.34, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="seat_pan",
    )
    seat.visual(
        Box((0.40, 0.40, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0495)),
        material=cushion_vinyl,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.026, 0.018, 0.012)),
        origin=Origin(xyz=(0.041, 0.149, 0.002)),
        material=dark_steel,
        name="hinge_bracket_0",
    )
    seat.visual(
        Box((0.026, 0.018, 0.012)),
        origin=Origin(xyz=(-0.041, 0.149, 0.002)),
        material=dark_steel,
        name="hinge_bracket_1",
    )
    seat.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.041, 0.155, 0.002), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_knuckle_0",
    )
    seat.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(-0.041, 0.155, 0.002), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_knuckle_1",
    )

    footrest_tab = model.part("footrest_tab")
    footrest_tab.visual(
        Cylinder(radius=0.0055, length=0.05),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    footrest_tab.visual(
        Box((0.05, 0.014, 0.011)),
        origin=Origin(xyz=(0.0, 0.004, -0.0055)),
        material=steel,
        name="hinge_leaf",
    )
    footrest_tab.visual(
        Box((0.14, 0.104, 0.01)),
        origin=Origin(xyz=(0.0, 0.048, -0.011)),
        material=rubber,
        name="tab_plate",
    )
    footrest_tab.visual(
        Box((0.14, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.095, -0.017)),
        material=dark_steel,
        name="tab_lip",
    )

    model.articulation(
        "pedestal_to_seat",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.617)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
    )
    model.articulation(
        "seat_to_footrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=footrest_tab,
        origin=Origin(xyz=(0.0, 0.155, 0.002)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    footrest_tab = object_model.get_part("footrest_tab")
    seat_swivel = object_model.get_articulation("pedestal_to_seat")
    footrest_hinge = object_model.get_articulation("seat_to_footrest")

    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="mount_plate",
        negative_elem="top_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="seat mount plate sits on the pedestal collar",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="mount_plate",
        elem_b="top_collar",
        min_overlap=0.09,
        name="seat mount stays centered over the pedestal axis",
    )

    seat_cushion_aabb = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    ctx.check(
        "counter stool seat height",
        seat_cushion_aabb is not None and 0.65 <= seat_cushion_aabb[1][2] <= 0.72,
        details=f"seat_cushion_aabb={seat_cushion_aabb}",
    )

    rest_seat_pos = ctx.part_world_position(seat)
    with ctx.pose({seat_swivel: 1.2}):
        rotated_seat_pos = ctx.part_world_position(seat)
    ctx.check(
        "seat rotates about a fixed vertical pedestal axis",
        rest_seat_pos is not None
        and rotated_seat_pos is not None
        and abs(rotated_seat_pos[0] - rest_seat_pos[0]) <= 1e-6
        and abs(rotated_seat_pos[1] - rest_seat_pos[1]) <= 1e-6
        and abs(rotated_seat_pos[2] - rest_seat_pos[2]) <= 1e-6,
        details=f"rest={rest_seat_pos}, rotated={rotated_seat_pos}",
    )

    rest_tab_aabb = ctx.part_world_aabb(footrest_tab)
    with ctx.pose({footrest_hinge: 1.15}):
        deployed_tab_aabb = ctx.part_world_aabb(footrest_tab)
        ctx.expect_gap(
            seat,
            footrest_tab,
            axis="z",
            positive_elem="seat_pan",
            negative_elem="tab_lip",
            min_gap=0.08,
            name="deployed footrest tab hangs below the seat frame",
        )
    ctx.check(
        "footrest tab rotates downward",
        rest_tab_aabb is not None
        and deployed_tab_aabb is not None
        and deployed_tab_aabb[0][2] < rest_tab_aabb[0][2] - 0.05,
        details=f"rest={rest_tab_aabb}, deployed={deployed_tab_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
