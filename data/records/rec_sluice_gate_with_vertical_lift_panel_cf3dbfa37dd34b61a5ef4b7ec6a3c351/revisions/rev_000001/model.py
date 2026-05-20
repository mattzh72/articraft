from __future__ import annotations

from math import pi

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


OPENING_W = 2.40
OPENING_H = 1.82
PIER_W = 0.56
FRAME_DEPTH = 0.60
SILL_H = 0.26
BRIDGE_H = 0.48
GUIDE_EXT = 1.60

FRAME_WALL_H = SILL_H + OPENING_H + BRIDGE_H
TOTAL_H = FRAME_WALL_H + GUIDE_EXT
OUTER_W = OPENING_W + (2.0 * PIER_W)
TOWER_W = 0.38

BRIDGE_DECK_W = 2.44
BRIDGE_DECK_D = 0.96
BRIDGE_DECK_H = 0.18

TRACK_BOTTOM = 0.18
TRACK_TOP = TOTAL_H - 0.10
TRACK_H = TRACK_TOP - TRACK_BOTTOM
BACK_TRACK_W = 0.10
BACK_TRACK_D = 0.18
BACK_TRACK_Y = -0.065
LIP_W = 0.06
LIP_D = 0.08
LIP_Y = 0.17

PANEL_W = 2.14
PANEL_T = 0.05
PANEL_H = 2.28
PANEL_RIB_T = 0.018
PANEL_CLOSED_BOTTOM = 0.26
PANEL_Y = 0.08
PANEL_TRAVEL = 1.48

PEDESTAL_W = 0.82
PEDESTAL_D = 0.58
PEDESTAL_H = 0.20
GEARBOX_W = 0.62
GEARBOX_D = 0.46
GEARBOX_H = 0.48
TOPCAP_W = 0.40
TOPCAP_D = 0.34
TOPCAP_H = 0.12
HOUSING_MOUNT_Z = FRAME_WALL_H + BRIDGE_DECK_H

AXLE_BOSS_R = 0.11
AXLE_BOSS_L = 0.12
AXLE_BOSS_Y = 0.285
AXLE_BOSS_Z = 0.48

WHEEL_R_OUT = 0.36
WHEEL_R_IN = 0.30
WHEEL_THICK = 0.04
HUB_R = 0.08
HUB_THICK = 0.06
SPOKE_W = 0.05
SPOKE_T = 0.02
SPINNER_R = 0.018
SPINNER_L = 0.09
SPINNER_X = 0.22
SPINNER_Y = 0.05
SPINNER_Z = 0.22
WHEEL_CENTER_Y = 0.44
WHEEL_CENTER_Z = AXLE_BOSS_Z

FLAP_T = 0.025
FLAP_W = 0.18
FLAP_H = 0.34
FLAP_Z0 = 0.27


def _frame_body_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(OUTER_W, FRAME_DEPTH, FRAME_WALL_H)
        .translate((0.0, 0.0, FRAME_WALL_H / 2.0))
    )
    opening = (
        cq.Workplane("XY")
        .box(OPENING_W, FRAME_DEPTH + 0.04, OPENING_H)
        .translate((0.0, 0.0, SILL_H + (OPENING_H / 2.0)))
    )
    body = outer.cut(opening)

    lift_slot = (
        cq.Workplane("XY")
        .box(PANEL_W + 0.10, FRAME_DEPTH + 0.04, TOTAL_H - (SILL_H + OPENING_H))
        .translate(
            (
                0.0,
                0.0,
                (SILL_H + OPENING_H) + ((TOTAL_H - (SILL_H + OPENING_H)) / 2.0),
            )
        )
    )
    body = body.cut(lift_slot)

    tower_center_x = (OPENING_W / 2.0) + (PIER_W / 2.0)
    for sign in (-1.0, 1.0):
        tower = (
            cq.Workplane("XY")
            .box(TOWER_W, FRAME_DEPTH, GUIDE_EXT)
            .translate((sign * tower_center_x, 0.0, FRAME_WALL_H + (GUIDE_EXT / 2.0)))
        )
        body = body.union(tower)

    return body


def _panel_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_T, PANEL_H)
        .translate((0.0, 0.0, PANEL_H / 2.0))
    )

    rib_y = (PANEL_T + PANEL_RIB_T) / 2.0
    rib_specs = (
        (PANEL_W * 0.84, 0.16, PANEL_H - 0.22),
        (PANEL_W * 0.80, 0.12, PANEL_H * 0.72),
        (PANEL_W * 0.78, 0.12, PANEL_H * 0.50),
        (PANEL_W * 0.76, 0.12, PANEL_H * 0.28),
    )
    for rib_w, rib_h, rib_z in rib_specs:
        panel = panel.union(
            cq.Workplane("XY").box(rib_w, PANEL_RIB_T, rib_h).translate((0.0, rib_y, rib_z))
        )

    edge_stiffener_x = (PANEL_W / 2.0) - 0.09
    edge_stiffener_z = PANEL_H / 2.0
    edge_stiffener_h = PANEL_H * 0.88
    for sign in (-1.0, 1.0):
        panel = panel.union(
            cq.Workplane("XY")
            .box(0.12, PANEL_RIB_T, edge_stiffener_h)
            .translate((sign * edge_stiffener_x, rib_y, edge_stiffener_z))
        )

    center_stiffener_h = PANEL_H * 0.82
    panel = panel.union(
        cq.Workplane("XY")
        .box(0.10, PANEL_RIB_T, center_stiffener_h)
        .translate((0.0, rib_y, PANEL_H * 0.50))
    )

    return panel


def _housing_shape() -> cq.Workplane:
    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_W, PEDESTAL_D, PEDESTAL_H)
        .translate((0.0, 0.0, PEDESTAL_H / 2.0))
    )
    gear_box = (
        cq.Workplane("XY")
        .box(GEARBOX_W, GEARBOX_D, GEARBOX_H)
        .translate((0.0, 0.0, PEDESTAL_H + (GEARBOX_H / 2.0)))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(TOPCAP_W, TOPCAP_D, TOPCAP_H)
        .translate((0.0, 0.0, PEDESTAL_H + GEARBOX_H + (TOPCAP_H / 2.0)))
    )
    side_pad = (
        cq.Workplane("XY")
        .box(0.10, 0.20, 0.38)
        .translate((GEARBOX_W / 2.0 + 0.05, 0.0, PEDESTAL_H + 0.27))
    )

    return pedestal.union(gear_box).union(top_cap).union(side_pad)


def _wheel_shape() -> cq.Workplane:
    rim = cq.Workplane("XZ").circle(WHEEL_R_OUT).circle(WHEEL_R_IN).extrude(WHEEL_THICK, both=True)
    hub = cq.Workplane("XZ").circle(HUB_R).extrude(HUB_THICK, both=True)
    vertical_spoke = cq.Workplane("XZ").rect(SPOKE_W, WHEEL_R_OUT * 1.82).extrude(SPOKE_T, both=True)
    horizontal_spoke = cq.Workplane("XZ").rect(WHEEL_R_OUT * 1.82, SPOKE_W).extrude(SPOKE_T, both=True)
    return rim.union(hub).union(vertical_spoke).union(horizontal_spoke)


def _flap_shape() -> cq.Workplane:
    flap = (
        cq.Workplane("XY")
        .box(FLAP_T, FLAP_W, FLAP_H)
        .translate((FLAP_T / 2.0, FLAP_W / 2.0, FLAP_H / 2.0))
    )
    latch = (
        cq.Workplane("XY")
        .box(0.016, 0.045, 0.09)
        .translate((FLAP_T + 0.008, FLAP_W * 0.74, FLAP_H * 0.56))
    )
    return flap.union(latch)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        (min_x + max_x) / 2.0,
        (min_y + max_y) / 2.0,
        (min_z + max_z) / 2.0,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="water_control_gate")

    model.material("concrete", rgba=(0.63, 0.64, 0.65, 1.0))
    model.material("weathered_steel", rgba=(0.34, 0.37, 0.40, 1.0))
    model.material("gate_red", rgba=(0.62, 0.21, 0.15, 1.0))
    model.material("machinery_green", rgba=(0.28, 0.38, 0.28, 1.0))
    model.material("bright_steel", rgba=(0.74, 0.76, 0.79, 1.0))

    frame = model.part("frame")
    pier_center_x = (OPENING_W / 2.0) + (PIER_W / 2.0)
    frame.visual(
        Box((PIER_W, FRAME_DEPTH, FRAME_WALL_H)),
        origin=Origin(xyz=(-pier_center_x, 0.0, FRAME_WALL_H / 2.0)),
        material="concrete",
        name="left_pier",
    )
    frame.visual(
        Box((PIER_W, FRAME_DEPTH, FRAME_WALL_H)),
        origin=Origin(xyz=(pier_center_x, 0.0, FRAME_WALL_H / 2.0)),
        material="concrete",
        name="right_pier",
    )
    frame.visual(
        Box((OUTER_W, FRAME_DEPTH, SILL_H)),
        origin=Origin(xyz=(0.0, 0.0, SILL_H / 2.0)),
        material="concrete",
        name="sill",
    )
    tower_center_z = FRAME_WALL_H + (GUIDE_EXT / 2.0)
    frame.visual(
        Box((TOWER_W, FRAME_DEPTH, GUIDE_EXT)),
        origin=Origin(xyz=(-pier_center_x, 0.0, tower_center_z)),
        material="concrete",
        name="left_tower",
    )
    frame.visual(
        Box((TOWER_W, FRAME_DEPTH, GUIDE_EXT)),
        origin=Origin(xyz=(pier_center_x, 0.0, tower_center_z)),
        material="concrete",
        name="right_tower",
    )
    frame.visual(
        Box((BRIDGE_DECK_W, BRIDGE_DECK_D, BRIDGE_DECK_H)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_WALL_H + (BRIDGE_DECK_H / 2.0))),
        material="weathered_steel",
        name="bridge_deck",
    )

    track_center_z = TRACK_BOTTOM + (TRACK_H / 2.0)
    left_track_x = -((OPENING_W / 2.0) - (BACK_TRACK_W / 2.0))
    right_track_x = -left_track_x
    left_lip_x = -((OPENING_W / 2.0) - (LIP_W / 2.0))
    right_lip_x = -left_lip_x

    frame.visual(
        Box((BACK_TRACK_W, BACK_TRACK_D, TRACK_H)),
        origin=Origin(xyz=(left_track_x, BACK_TRACK_Y, track_center_z)),
        material="weathered_steel",
        name="left_track_back",
    )
    frame.visual(
        Box((BACK_TRACK_W, BACK_TRACK_D, TRACK_H)),
        origin=Origin(xyz=(right_track_x, BACK_TRACK_Y, track_center_z)),
        material="weathered_steel",
        name="right_track_back",
    )
    frame.visual(
        Box((LIP_W, LIP_D, TRACK_H)),
        origin=Origin(xyz=(left_lip_x, LIP_Y, track_center_z)),
        material="weathered_steel",
        name="left_track_lip",
    )
    frame.visual(
        Box((LIP_W, LIP_D, TRACK_H)),
        origin=Origin(xyz=(right_lip_x, LIP_Y, track_center_z)),
        material="weathered_steel",
        name="right_track_lip",
    )

    panel = model.part("panel")
    panel.visual(
        mesh_from_cadquery(_panel_shape(), "gate_panel"),
        material="gate_red",
        name="gate_panel",
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "gear_housing"),
        material="machinery_green",
        name="gear_housing",
    )
    housing.visual(
        Cylinder(radius=AXLE_BOSS_R, length=AXLE_BOSS_L),
        origin=Origin(xyz=(0.0, AXLE_BOSS_Y, AXLE_BOSS_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="machinery_green",
        name="axle_boss",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_wheel_shape(), "handwheel_wheel"),
        material="bright_steel",
        name="wheel_rim",
    )
    handwheel.visual(
        Cylinder(radius=SPINNER_R, length=SPINNER_L),
        origin=Origin(xyz=(SPINNER_X, SPINNER_Y, SPINNER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="spinner_knob",
    )
    handwheel.visual(
        Cylinder(radius=0.028, length=0.10),
        origin=Origin(xyz=(0.0, -0.045, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="axle_stub",
    )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_flap_shape(), "inspection_flap"),
        material="weathered_steel",
        name="inspection_flap",
    )

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, PANEL_Y, PANEL_CLOSED_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=PANEL_TRAVEL, effort=22000.0, velocity=0.20),
    )
    model.articulation(
        "housing_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_MOUNT_Z)),
    )
    model.articulation(
        "handwheel_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=handwheel,
        origin=Origin(xyz=(0.0, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=4.0),
    )
    model.articulation(
        "inspection_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(PEDESTAL_W / 2.0, -(FLAP_W / 2.0), FLAP_Z0)),
        # Closed flap extends along local +Y from the hinge line.
        # Using -Z makes positive q swing the free edge outward toward +X.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=12.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    housing = object_model.get_part("housing")
    handwheel = object_model.get_part("handwheel")
    flap = object_model.get_part("flap")

    gate_slide = object_model.get_articulation("gate_slide")
    handwheel_spin = object_model.get_articulation("handwheel_spin")
    flap_hinge = object_model.get_articulation("inspection_flap_hinge")

    with ctx.pose({gate_slide: 0.0, handwheel_spin: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            panel,
            frame,
            axis="x",
            positive_elem="gate_panel",
            negative_elem="left_track_back",
            min_gap=0.020,
            max_gap=0.040,
            name="panel clears left track back",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="x",
            positive_elem="right_track_back",
            negative_elem="gate_panel",
            min_gap=0.020,
            max_gap=0.040,
            name="panel clears right track back",
        )
        ctx.expect_gap(
            panel,
            frame,
            axis="y",
            positive_elem="gate_panel",
            negative_elem="left_track_back",
            min_gap=0.020,
            max_gap=0.045,
            name="panel clears the rear guide face",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="y",
            positive_elem="left_track_lip",
            negative_elem="gate_panel",
            min_gap=0.006,
            max_gap=0.025,
            name="panel stays behind the retaining lip",
        )
        ctx.expect_contact(
            housing,
            frame,
            elem_a="gear_housing",
            elem_b="bridge_deck",
            contact_tol=0.003,
            name="gear housing is seated on the bridge deck",
        )

    rest_panel_pos = ctx.part_world_position(panel)
    with ctx.pose({gate_slide: PANEL_TRAVEL}):
        raised_panel_pos = ctx.part_world_position(panel)
        ctx.expect_overlap(
            panel,
            frame,
            axes="z",
            elem_a="gate_panel",
            elem_b="left_track_back",
            min_overlap=0.80,
            name="raised panel remains captured in the left guide",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="z",
            elem_a="gate_panel",
            elem_b="right_track_back",
            min_overlap=0.80,
            name="raised panel remains captured in the right guide",
        )
    ctx.check(
        "panel lifts upward through the opening",
        rest_panel_pos is not None
        and raised_panel_pos is not None
        and raised_panel_pos[2] > rest_panel_pos[2] + 1.40,
        details=f"rest={rest_panel_pos}, raised={raised_panel_pos}",
    )

    rest_knob_center = _aabb_center(ctx.part_element_world_aabb(handwheel, elem="spinner_knob"))
    with ctx.pose({handwheel_spin: pi / 2.0}):
        quarter_turn_knob_center = _aabb_center(ctx.part_element_world_aabb(handwheel, elem="spinner_knob"))
    ctx.check(
        "handwheel rotation carries the spinner knob around the axle",
        rest_knob_center is not None
        and quarter_turn_knob_center is not None
        and quarter_turn_knob_center[2] < rest_knob_center[2] - 0.15,
        details=f"rest={rest_knob_center}, quarter_turn={quarter_turn_knob_center}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="inspection_flap")
    with ctx.pose({flap_hinge: 1.0}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="inspection_flap")
    ctx.check(
        "inspection flap swings outward from the housing side",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][0] > closed_flap_aabb[1][0] + 0.10,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
