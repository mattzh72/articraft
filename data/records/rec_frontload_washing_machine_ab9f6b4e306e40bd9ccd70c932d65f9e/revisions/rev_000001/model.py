from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


BODY_W = 0.62
BODY_D = 0.68
BODY_H = 0.85
PANEL_T = 0.022
FRONT_Y = -BODY_D / 2.0
REAR_Y = BODY_D / 2.0
PORT_Z = 0.43
HINGE_X = -0.265
DOOR_Y = FRONT_Y - 0.057
TOP_HINGE_Y = REAR_Y - 0.025
TOP_HINGE_Z = BODY_H + 0.020


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cabinet_shell() -> cq.Workplane:
    """Thin rectangular washer cabinet with an open top and a round front port."""
    t = PANEL_T
    front = _cq_box((BODY_W, t, BODY_H), (0.0, FRONT_Y + t / 2.0, BODY_H / 2.0))
    front_cut = (
        cq.Workplane("XZ")
        .center(0.0, PORT_Z)
        .circle(0.205)
        .extrude(t + 0.08, both=True)
        .translate((0.0, FRONT_Y + t / 2.0, 0.0))
    )
    front = front.cut(front_cut)

    panels = [
        front,
        _cq_box((BODY_W, t, BODY_H), (0.0, REAR_Y - t / 2.0, BODY_H / 2.0)),
        _cq_box((t, BODY_D, BODY_H), (-BODY_W / 2.0 + t / 2.0, 0.0, BODY_H / 2.0)),
        _cq_box((t, BODY_D, BODY_H), (BODY_W / 2.0 - t / 2.0, 0.0, BODY_H / 2.0)),
        _cq_box((BODY_W, BODY_D, t), (0.0, 0.0, t / 2.0)),
    ]
    shell = panels[0]
    for panel in panels[1:]:
        shell = shell.union(panel)
    return shell


def _drum_basket() -> cq.Workplane:
    """Open stainless basket, centered on a front-to-back axle."""
    length = 0.46
    outer_r = 0.232
    inner_r = 0.198
    tube = (
        cq.Workplane("XZ")
        .circle(outer_r)
        .extrude(length / 2.0, both=True)
        .cut(cq.Workplane("XZ").circle(inner_r).extrude(length / 2.0 + 0.04, both=True))
    )
    rear_disc = (
        cq.Workplane("XZ")
        .circle(inner_r)
        .extrude(0.014)
        .translate((0.0, length / 2.0 - 0.014, 0.0))
    )
    basket = tube.union(rear_disc)
    for angle in (0.0, 120.0, 240.0):
        lifter = (
            cq.Workplane("XY")
            .box(0.050, length * 0.78, 0.020)
            .translate((0.0, 0.0, inner_r - 0.006))
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
        )
        basket = basket.union(lifter)
    return basket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_access_washer")

    white = Material("white_enamel", rgba=(0.94, 0.95, 0.93, 1.0))
    dark = Material("dark_rubber", rgba=(0.02, 0.022, 0.024, 1.0))
    glass = Material("smoked_glass", rgba=(0.48, 0.70, 0.86, 0.42))
    steel = Material("brushed_stainless", rgba=(0.62, 0.64, 0.63, 1.0))
    hinge_metal = Material("hinge_pin_steel", rgba=(0.36, 0.37, 0.36, 1.0))
    pcb = Material("green_pcb", rgba=(0.04, 0.38, 0.15, 1.0))
    black = Material("control_black", rgba=(0.005, 0.006, 0.007, 1.0))
    amber = Material("capacitor_gold", rgba=(0.95, 0.68, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        Box((PANEL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + PANEL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=white,
        name="left_side_panel",
    )
    body.visual(
        Box((PANEL_T, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - PANEL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=white,
        name="right_side_panel",
    )
    body.visual(
        Box((BODY_W, PANEL_T, BODY_H)),
        origin=Origin(xyz=(0.0, REAR_Y - PANEL_T / 2.0, BODY_H / 2.0)),
        material=white,
        name="rear_panel",
    )
    body.visual(
        Box((BODY_W, BODY_D, PANEL_T)),
        origin=Origin(xyz=(0.0, 0.0, PANEL_T / 2.0)),
        material=white,
        name="bottom_panel",
    )
    stile_w = (BODY_W - 0.410) / 2.0
    for name, x in (
        ("front_left_stile", -BODY_W / 2.0 + stile_w / 2.0),
        ("front_right_stile", BODY_W / 2.0 - stile_w / 2.0),
    ):
        body.visual(
            Box((stile_w + 0.010, PANEL_T, BODY_H)),
            origin=Origin(xyz=(x, FRONT_Y + PANEL_T / 2.0, BODY_H / 2.0)),
            material=white,
            name=name,
        )
    body.visual(
        Box((BODY_W, PANEL_T, PORT_Z - 0.205)),
        origin=Origin(xyz=(0.0, FRONT_Y + PANEL_T / 2.0, (PORT_Z - 0.205) / 2.0)),
        material=white,
        name="front_bottom_rail",
    )
    body.visual(
        Box((BODY_W, PANEL_T, BODY_H - (PORT_Z + 0.205))),
        origin=Origin(xyz=(0.0, FRONT_Y + PANEL_T / 2.0, (BODY_H + PORT_Z + 0.205) / 2.0)),
        material=white,
        name="front_top_rail",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.355, 0.355),
                (0.438, 0.438),
                0.026,
                opening_shape="circle",
                outer_shape="circle",
            ),
            "front_port_gasket",
        ),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.011, PORT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="front_port_gasket",
    )
    body.visual(
        Box((0.46, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.090, BODY_H - 0.085)),
        material=black,
        name="control_tray",
    )
    body.visual(
        Box((0.40, 0.135, 0.010)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.115, BODY_H - 0.068)),
        material=pcb,
        name="control_board",
    )
    for i, x in enumerate((-0.135, -0.045, 0.045, 0.135)):
        body.visual(
            Box((0.038, 0.018, 0.020)),
            origin=Origin(xyz=(x, REAR_Y - 0.115, BODY_H - 0.053)),
            material=amber if i % 2 == 0 else hinge_metal,
            name=f"board_component_{i}",
        )

    body.visual(
        Cylinder(radius=0.055, length=0.080),
        origin=Origin(xyz=(0.0, REAR_Y - 0.045, PORT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="rear_bearing_hub",
    )

    # Two visible fixed knuckle segments for the front porthole door hinges.
    for suffix, local_z in (("upper", 0.165), ("lower", -0.165)):
        body.visual(
            Cylinder(radius=0.017, length=0.037),
            origin=Origin(xyz=(HINGE_X, DOOR_Y - 0.016, PORT_Z + local_z), rpy=(0.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"{suffix}_fixed_knuckle",
        )
        body.visual(
            Cylinder(radius=0.009, length=0.145),
            origin=Origin(xyz=(HINGE_X, DOOR_Y - 0.016, PORT_Z + local_z), rpy=(0.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"{suffix}_hinge_pin",
        )
        body.visual(
            Box((0.058, 0.090, 0.046)),
            origin=Origin(xyz=(HINGE_X - 0.027, FRONT_Y - 0.034, PORT_Z + local_z)),
            material=white,
            name=f"{suffix}_fixed_leaf",
        )

    # Rear top hinge sockets mounted to the cabinet back wall.
    for i, x in enumerate((-0.205, 0.205)):
        body.visual(
            Box((0.110, 0.030, 0.034)),
            origin=Origin(xyz=(x, REAR_Y - 0.010, TOP_HINGE_Z - 0.004)),
            material=white,
            name=f"rear_hinge_socket_{i}",
        )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(_drum_basket(), "drum_basket", tolerance=0.0015),
        material=steel,
        name="drum_basket",
    )
    drum.visual(
        Cylinder(radius=0.032, length=0.62),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="drum_axle",
    )

    door = model.part("porthole_door")
    door.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.330, 0.330),
                (0.470, 0.470),
                0.055,
                opening_shape="circle",
                outer_shape="circle",
            ),
            "door_outer_ring",
        ),
        origin=Origin(xyz=(-HINGE_X, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="door_outer_ring",
    )
    door.visual(
        Cylinder(radius=0.168, length=0.028),
        origin=Origin(xyz=(-HINGE_X, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=0.013, length=0.125),
        origin=Origin(xyz=(-HINGE_X + 0.215, -0.040, 0.0), rpy=(0.0, 0.0, 0.0)),
        material=dark,
        name="door_handle",
    )
    for suffix, zc in (("upper", 0.165), ("lower", -0.165)):
        for j, dz in enumerate((-0.036, 0.036)):
            door.visual(
                Cylinder(radius=0.017, length=0.032),
                origin=Origin(xyz=(0.0, -0.016, zc + dz), rpy=(0.0, 0.0, 0.0)),
                material=hinge_metal,
                name=f"{suffix}_door_knuckle_{j}",
            )
            door.visual(
                Box((0.160, 0.020, 0.034)),
                origin=Origin(xyz=(0.077, -0.032, zc + dz)),
                material=white,
                name=f"{suffix}_door_leaf_{j}",
            )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((BODY_W - 0.070, 0.600, 0.030)),
        origin=Origin(xyz=(0.0, -0.300, 0.0)),
        material=white,
        name="panel_slab",
    )
    service_panel.visual(
        Box((0.140, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, -0.585, 0.018)),
        material=dark,
        name="front_lift_recess",
    )
    for i, x in enumerate((-0.205, 0.205)):
        service_panel.visual(
            Cylinder(radius=0.012, length=0.085),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"rear_hinge_pin_{i}",
        )
        service_panel.visual(
            Box((0.105, 0.025, 0.018)),
            origin=Origin(xyz=(x, -0.018, -0.018)),
            material=white,
            name=f"rear_hinge_leaf_{i}",
        )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, PORT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=12.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, DOOR_Y, PORT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.85),
    )
    model.articulation(
        "body_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_panel,
        origin=Origin(xyz=(0.0, TOP_HINGE_Y, TOP_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.0, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("porthole_door")
    panel = object_model.get_part("service_panel")
    door_joint = object_model.get_articulation("body_to_door")
    panel_joint = object_model.get_articulation("body_to_service_panel")

    ctx.allow_overlap(
        body,
        drum,
        elem_a="rear_bearing_hub",
        elem_b="drum_axle",
        reason="The drum axle is intentionally captured inside the rear bearing hub.",
    )
    for i in (0, 1):
        ctx.allow_overlap(
            body,
            panel,
            elem_a=f"rear_hinge_socket_{i}",
            elem_b=f"rear_hinge_pin_{i}",
            reason="Each top service-panel hinge pin is intentionally seated in its rear socket.",
        )
    for hinge in ("upper", "lower"):
        for i in (0, 1):
            ctx.allow_overlap(
                body,
                door,
                elem_a=f"{hinge}_hinge_pin",
                elem_b=f"{hinge}_door_knuckle_{i}",
                reason="The vertical door hinge pin passes through the rotating barrel knuckle.",
            )

    ctx.expect_within(
        drum,
        body,
        axes="xz",
        margin=0.035,
        name="drum is contained in cabinet width and height",
    )
    ctx.expect_within(
        drum,
        body,
        axes="xz",
        inner_elem="drum_axle",
        outer_elem="rear_bearing_hub",
        margin=0.0,
        name="drum axle is centered inside rear bearing",
    )
    ctx.expect_overlap(
        drum,
        body,
        axes="y",
        elem_a="drum_axle",
        elem_b="rear_bearing_hub",
        min_overlap=0.035,
        name="rear bearing retains the drum axle",
    )
    for i in (0, 1):
        ctx.expect_within(
            panel,
            body,
            axes="xz",
            inner_elem=f"rear_hinge_pin_{i}",
            outer_elem=f"rear_hinge_socket_{i}",
            margin=0.003,
            name=f"service hinge pin {i} sits in socket",
        )
        ctx.expect_overlap(
            panel,
            body,
            axes="y",
            elem_a=f"rear_hinge_pin_{i}",
            elem_b=f"rear_hinge_socket_{i}",
            min_overlap=0.010,
            name=f"service hinge pin {i} is captured in depth",
        )
    for hinge in ("upper", "lower"):
        for i in (0, 1):
            ctx.expect_within(
                body,
                door,
                axes="xy",
                inner_elem=f"{hinge}_hinge_pin",
                outer_elem=f"{hinge}_door_knuckle_{i}",
                margin=0.002,
                name=f"{hinge} door hinge pin {i} is coaxial",
            )
            ctx.expect_overlap(
                body,
                door,
                axes="z",
                elem_a=f"{hinge}_hinge_pin",
                elem_b=f"{hinge}_door_knuckle_{i}",
                min_overlap=0.020,
                name=f"{hinge} door hinge pin {i} passes through knuckle",
            )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_port_gasket",
        negative_elem="door_outer_ring",
        min_gap=0.002,
        max_gap=0.035,
        name="closed porthole door sits just proud of the front gasket",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.28,
        name="circular door covers the front porthole",
    )
    ctx.expect_gap(
        panel,
        body,
        axis="z",
        positive_elem="panel_slab",
        negative_elem="front_top_rail",
        min_gap=0.002,
        max_gap=0.012,
        name="service panel rests above the open top rim",
    )

    closed_handle = ctx.part_element_world_aabb(door, elem="door_handle")
    with ctx.pose({door_joint: 1.20}):
        open_handle = ctx.part_element_world_aabb(door, elem="door_handle")
    ctx.check(
        "porthole door swings outward on left hinge",
        closed_handle is not None
        and open_handle is not None
        and open_handle[0][1] < closed_handle[0][1] - 0.12,
        details=f"closed={closed_handle}, open={open_handle}",
    )

    closed_lip = ctx.part_element_world_aabb(panel, elem="front_lift_recess")
    with ctx.pose({panel_joint: 1.0}):
        lifted_lip = ctx.part_element_world_aabb(panel, elem="front_lift_recess")
    ctx.check(
        "top service panel lifts upward from front edge",
        closed_lip is not None
        and lifted_lip is not None
        and lifted_lip[1][2] > closed_lip[1][2] + 0.18,
        details=f"closed={closed_lip}, lifted={lifted_lip}",
    )

    return ctx.report()


object_model = build_object_model()
