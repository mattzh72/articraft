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


def _front_panel_shape(
    width: float,
    height: float,
    thickness: float,
    *,
    porthole_radius: float,
    porthole_z: float,
    slot_x: float,
    slot_z: float,
    slot_width: float,
    slot_height: float,
) -> cq.Workplane:
    """One continuous front fascia with the washer opening and dispenser slot cut through it."""
    plate = cq.Workplane("XY").box(width, thickness, height)
    porthole_cut = (
        cq.Workplane("XZ")
        .circle(porthole_radius)
        .extrude(thickness * 3.0)
        .translate((0.0, -thickness * 1.5, porthole_z))
    )
    slot_cut = (
        cq.Workplane("XY")
        .box(slot_width, thickness * 3.0, slot_height)
        .translate((slot_x, 0.0, slot_z))
    )
    return plate.cut(porthole_cut).cut(slot_cut).edges("|Z").fillet(0.004)


def _control_panel_shape(
    width: float,
    height: float,
    thickness: float,
    *,
    slot_x: float,
    slot_width: float,
    slot_height: float,
) -> cq.Workplane:
    """Raised control fascia with a real through-slot for the soap drawer."""
    panel = cq.Workplane("XY").box(width, thickness, height)
    slot_cut = (
        cq.Workplane("XY")
        .box(slot_width, thickness * 3.0, slot_height)
        .translate((slot_x, 0.0, 0.0))
    )
    return panel.cut(slot_cut).edges("|Z").fillet(0.003)


def _round_ring_shape(
    *,
    center_x: float,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
) -> cq.Workplane:
    """A circular frame extruded along local -Y; useful for a porthole door ring."""
    return (
        cq.Workplane("XZ")
        .center(center_x, 0.0)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .edges()
        .fillet(0.004)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="combined_washer_dryer")

    white = model.material("white_enamel", rgba=(0.92, 0.94, 0.93, 1.0))
    light_gray = model.material("light_gray_plastic", rgba=(0.70, 0.72, 0.72, 1.0))
    dark = model.material("dark_recess", rgba=(0.015, 0.017, 0.018, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.10, 0.16, 0.19, 0.48))
    display_glass = model.material("display_glass", rgba=(0.01, 0.025, 0.04, 1.0))
    blue = model.material("translucent_blue_plastic", rgba=(0.25, 0.45, 0.72, 0.75))

    width = 0.66
    depth = 0.62
    height = 0.86
    side_t = 0.035
    front_t = 0.024
    front_y = -depth / 2.0 - front_t / 2.0
    panel_front_y = front_y - front_t / 2.0

    porthole_z = 0.38
    porthole_z_local = porthole_z - height / 2.0
    slot_x = -0.15
    slot_z = 0.765
    slot_z_local = slot_z - height / 2.0

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((width, depth, side_t)),
        origin=Origin(xyz=(0.0, 0.0, side_t / 2.0)),
        material=white,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((width, depth, side_t)),
        origin=Origin(xyz=(0.0, 0.0, height - side_t / 2.0)),
        material=white,
        name="top_panel",
    )
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + side_t / 2.0, 0.0, height / 2.0)),
        material=white,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(width / 2.0 - side_t / 2.0, 0.0, height / 2.0)),
        material=white,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((width, side_t, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - side_t / 2.0, height / 2.0)),
        material=white,
        name="rear_panel",
    )
    cabinet.visual(
        mesh_from_cadquery(
            _front_panel_shape(
                width,
                height,
                front_t,
                porthole_radius=0.214,
                porthole_z=porthole_z_local,
                slot_x=slot_x,
                slot_z=slot_z_local,
                slot_width=0.300,
                slot_height=0.090,
            ),
            "front_fascia",
            tolerance=0.0012,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, front_y, height / 2.0)),
        material=white,
        name="front_panel",
    )
    cabinet.visual(
        mesh_from_cadquery(
            _control_panel_shape(
                width + 0.018,
                0.118,
                0.016,
                slot_x=slot_x,
                slot_width=0.300,
                slot_height=0.090,
            ),
            "control_fascia",
            tolerance=0.001,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, panel_front_y - 0.008, 0.765)),
        material=light_gray,
        name="control_fascia",
    )
    cabinet.visual(
        Box((0.160, 0.006, 0.048)),
        origin=Origin(xyz=(0.075, panel_front_y - 0.003, 0.766)),
        material=display_glass,
        name="display_window",
    )
    cabinet.visual(
        Cylinder(radius=0.198, length=0.018),
        origin=Origin(xyz=(0.0, panel_front_y + 0.030, porthole_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="dark_drum",
    )
    cabinet.visual(
        Cylinder(radius=0.017, length=0.080),
        origin=Origin(xyz=(-0.270, panel_front_y - 0.026, porthole_z + 0.230)),
        material=chrome,
        name="upper_hinge_knuckle",
    )
    cabinet.visual(
        Cylinder(radius=0.017, length=0.080),
        origin=Origin(xyz=(-0.270, panel_front_y - 0.026, porthole_z - 0.230)),
        material=chrome,
        name="lower_hinge_knuckle",
    )
    cabinet.visual(
        Box((0.050, 0.022, 0.082)),
        origin=Origin(xyz=(-0.288, panel_front_y - 0.010, porthole_z + 0.230)),
        material=chrome,
        name="upper_hinge_mount",
    )
    cabinet.visual(
        Box((0.050, 0.022, 0.082)),
        origin=Origin(xyz=(-0.288, panel_front_y - 0.010, porthole_z - 0.230)),
        material=chrome,
        name="lower_hinge_mount",
    )

    door = model.part("porthole_door")
    door_center_x = 0.270
    door.visual(
        mesh_from_cadquery(
            _round_ring_shape(
                center_x=door_center_x,
                outer_radius=0.252,
                inner_radius=0.166,
                thickness=0.046,
            ),
            "door_outer_ring",
            tolerance=0.001,
            angular_tolerance=0.06,
        ),
        material=chrome,
        name="door_ring",
    )
    door.visual(
        mesh_from_cadquery(
            _round_ring_shape(
                center_x=door_center_x,
                outer_radius=0.182,
                inner_radius=0.148,
                thickness=0.052,
            ),
            "door_gasket_ring",
            tolerance=0.001,
            angular_tolerance=0.06,
        ),
        material=black,
        name="gasket_ring",
    )
    door.visual(
        Cylinder(radius=0.157, length=0.014),
        origin=Origin(xyz=(door_center_x, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=smoked_glass,
        name="glass_bowl",
    )
    door.visual(
        Cylinder(radius=0.017, length=0.380),
        origin=Origin(xyz=(0.0, -0.023, 0.0)),
        material=chrome,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.132, 0.024, 0.060)),
        origin=Origin(xyz=(0.065, -0.023, 0.125)),
        material=chrome,
        name="upper_hinge_leaf",
    )
    door.visual(
        Box((0.132, 0.024, 0.060)),
        origin=Origin(xyz=(0.065, -0.023, -0.125)),
        material=chrome,
        name="lower_hinge_leaf",
    )
    door.visual(
        Box((0.036, 0.032, 0.176)),
        origin=Origin(xyz=(door_center_x + 0.222, -0.040, 0.0)),
        material=black,
        name="door_handle",
    )

    model.articulation(
        "cabinet_to_porthole_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.270, panel_front_y - 0.003, porthole_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.85, effort=18.0, velocity=1.4),
    )

    soap_drawer = model.part("soap_drawer")
    soap_drawer.visual(
        Box((0.326, 0.022, 0.108)),
        origin=Origin(xyz=(0.0, -0.011, 0.0)),
        material=white,
        name="drawer_face",
    )
    soap_drawer.visual(
        Box((0.214, 0.300, 0.008)),
        origin=Origin(xyz=(0.0, 0.148, -0.038)),
        material=blue,
        name="drawer_tray",
    )
    soap_drawer.visual(
        Box((0.008, 0.296, 0.046)),
        origin=Origin(xyz=(-0.111, 0.146, -0.014)),
        material=blue,
        name="tray_side_0",
    )
    soap_drawer.visual(
        Box((0.008, 0.296, 0.046)),
        origin=Origin(xyz=(0.111, 0.146, -0.014)),
        material=blue,
        name="tray_side_1",
    )
    soap_drawer.visual(
        Box((0.214, 0.008, 0.044)),
        origin=Origin(xyz=(0.0, 0.296, -0.014)),
        material=blue,
        name="tray_back_lip",
    )
    soap_drawer.visual(
        Box((0.008, 0.270, 0.044)),
        origin=Origin(xyz=(-0.035, 0.145, -0.014)),
        material=blue,
        name="tray_divider_0",
    )
    soap_drawer.visual(
        Box((0.008, 0.270, 0.044)),
        origin=Origin(xyz=(0.040, 0.145, -0.014)),
        material=blue,
        name="tray_divider_1",
    )

    model.articulation(
        "cabinet_to_soap_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=soap_drawer,
        origin=Origin(xyz=(slot_x, panel_front_y - 0.016, slot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.220, effort=45.0, velocity=0.35),
    )

    program_dial = model.part("program_dial")
    program_dial.visual(
        Cylinder(radius=0.041, length=0.032),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_gray,
        name="dial_cap",
    )
    program_dial.visual(
        Box((0.010, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, -0.034, 0.027)),
        material=black,
        name="dial_pointer",
    )
    model.articulation(
        "cabinet_to_program_dial",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=program_dial,
        origin=Origin(xyz=(0.255, panel_front_y - 0.016, 0.766)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.7, upper=2.7, effort=1.0, velocity=2.0),
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.045, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=chrome,
        name="button_cap",
    )
    model.articulation(
        "cabinet_to_start_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=start_button,
        origin=Origin(xyz=(0.155, panel_front_y - 0.016, 0.716)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.007, effort=2.0, velocity=0.04),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("porthole_door")
    drawer = object_model.get_part("soap_drawer")
    dial = object_model.get_part("program_dial")
    button = object_model.get_part("start_button")
    door_hinge = object_model.get_articulation("cabinet_to_porthole_door")
    drawer_slide = object_model.get_articulation("cabinet_to_soap_drawer")

    with ctx.pose({door_hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            positive_elem="front_panel",
            negative_elem="door_ring",
            min_gap=0.001,
            max_gap=0.008,
            name="closed porthole door sits just proud of the front fascia",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            elem_a="door_ring",
            elem_b="front_panel",
            min_overlap=0.38,
            name="porthole door covers the round front opening",
        )
        ctx.expect_within(
            drawer,
            cabinet,
            axes="xz",
            inner_elem="drawer_tray",
            outer_elem="front_panel",
            margin=0.0,
            name="soap tray passes through the dispenser opening footprint",
        )
        ctx.expect_gap(
            cabinet,
            dial,
            axis="y",
            positive_elem="control_fascia",
            negative_elem="dial_cap",
            min_gap=0.0,
            max_gap=0.006,
            name="program dial is seated on the control fascia",
        )
        ctx.expect_gap(
            cabinet,
            button,
            axis="y",
            positive_elem="control_fascia",
            negative_elem="button_cap",
            min_gap=0.0,
            max_gap=0.006,
            name="start button is proud of the control fascia",
        )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "porthole door swings outward on its vertical edge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10,
        details=f"closed_aabb={closed_door_aabb}, open_aabb={open_door_aabb}",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.220}):
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="drawer_tray",
            elem_b="front_panel",
            min_overlap=0.010,
            name="extended soap tray retains insertion through the housing",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)
    ctx.check(
        "soap drawer slides outward from the top control panel",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < closed_drawer_pos[1] - 0.18,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
