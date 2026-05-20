from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


DESK_W = 0.72
DESK_D = 0.52
DESK_H = 0.74
TOP_T = 0.024
TOP_Z = DESK_H - (TOP_T / 2.0)

LEG_S = 0.03
LEG_TOP = DESK_H - TOP_T
LEG_Z = LEG_TOP / 2.0

DRAWER_CENTER_X = -0.17
DRAWER_FRONT_Y = -0.205
DRAWER_TRAVEL = 0.20
DRAWER_BODY_W = 0.246
DRAWER_BODY_D = 0.310
DRAWER_BODY_H = 0.092
DRAWER_FRONT_W = 0.274
DRAWER_FRONT_H = 0.108
DRAWER_Z = 0.650

LID_AXIS_X = 0.201
LID_AXIS_Y = 0.230
LID_AXIS_Z = TOP_Z + 0.003
LID_W = 0.198
LID_D = 0.140
LID_T = 0.018


def _interval_overlap(aabb_a, aabb_b, axis_index: int) -> float:
    return min(aabb_a[1][axis_index], aabb_b[1][axis_index]) - max(aabb_a[0][axis_index], aabb_b[0][axis_index])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="student_desk")

    laminate = model.material("laminate", rgba=(0.73, 0.62, 0.44, 1.0))
    drawer_laminate = model.material("drawer_laminate", rgba=(0.68, 0.57, 0.40, 1.0))
    powder_steel = model.material("powder_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    pull_dark = model.material("pull_dark", rgba=(0.12, 0.13, 0.14, 1.0))

    frame = model.part("frame")

    frame.visual(
        Box((DESK_W, 0.350, TOP_T)),
        origin=Origin(xyz=(0.0, -0.085, TOP_Z)),
        material=laminate,
        name="top_front",
    )
    frame.visual(
        Box((0.460, 0.170, TOP_T)),
        origin=Origin(xyz=(-0.130, 0.175, TOP_Z)),
        material=laminate,
        name="top_left_rear",
    )
    frame.visual(
        Box((0.058, 0.170, TOP_T)),
        origin=Origin(xyz=(0.331, 0.175, TOP_Z)),
        material=laminate,
        name="top_right_border",
    )
    frame.visual(
        Box((0.200, 0.028, TOP_T)),
        origin=Origin(xyz=(0.200, 0.246, TOP_Z)),
        material=laminate,
        name="top_rear_strip",
    )

    frame.visual(
        Box((LEG_S, LEG_S, LEG_TOP)),
        origin=Origin(xyz=(-0.320, -0.205, LEG_Z)),
        material=powder_steel,
        name="leg_0",
    )
    frame.visual(
        Box((LEG_S, LEG_S, LEG_TOP)),
        origin=Origin(xyz=(-0.320, 0.205, LEG_Z)),
        material=powder_steel,
        name="leg_1",
    )
    frame.visual(
        Box((LEG_S, LEG_S, LEG_TOP)),
        origin=Origin(xyz=(0.320, -0.205, LEG_Z)),
        material=powder_steel,
        name="leg_2",
    )
    frame.visual(
        Box((LEG_S, LEG_S, LEG_TOP)),
        origin=Origin(xyz=(0.320, 0.205, LEG_Z)),
        material=powder_steel,
        name="leg_3",
    )

    frame.visual(
        Box((0.390, 0.018, 0.080)),
        origin=Origin(xyz=(-0.110, 0.212, 0.676)),
        material=powder_steel,
        name="rear_apron_left",
    )
    frame.visual(
        Box((0.327, 0.018, 0.080)),
        origin=Origin(xyz=(0.1415, -0.231, 0.676)),
        material=powder_steel,
        name="front_apron",
    )
    frame.visual(
        Box((0.018, 0.380, 0.050)),
        origin=Origin(xyz=(0.305, 0.0, 0.160)),
        material=powder_steel,
        name="right_stretcher",
    )
    frame.visual(
        Box((0.018, 0.380, 0.050)),
        origin=Origin(xyz=(-0.305, 0.0, 0.160)),
        material=powder_steel,
        name="left_stretcher",
    )

    frame.visual(
        Box((0.016, 0.340, 0.126)),
        origin=Origin(xyz=(-0.310, -0.050, 0.653)),
        material=powder_steel,
        name="outer_hanger",
    )
    frame.visual(
        Box((0.016, 0.340, 0.126)),
        origin=Origin(xyz=(-0.030, -0.050, 0.653)),
        material=powder_steel,
        name="inner_hanger",
    )
    frame.visual(
        Box((0.264, 0.018, 0.074)),
        origin=Origin(xyz=(-0.170, 0.121, 0.679)),
        material=powder_steel,
        name="guide_bridge",
    )
    frame.visual(
        Box((0.006, 0.290, 0.014)),
        origin=Origin(xyz=(-0.296, -0.005, 0.666)),
        material=rail_steel,
        name="guide_outer",
    )
    frame.visual(
        Box((0.006, 0.290, 0.014)),
        origin=Origin(xyz=(-0.044, -0.005, 0.666)),
        material=rail_steel,
        name="guide_inner",
    )

    frame.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(LID_AXIS_X - 0.055, LID_AXIS_Y + 0.010, LID_AXIS_Z, ), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="hinge_knuckle_0",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(LID_AXIS_X + 0.055, LID_AXIS_Y + 0.010, LID_AXIS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="hinge_knuckle_1",
    )
    frame.visual(
        Box((0.040, 0.012, 0.018)),
        origin=Origin(xyz=(LID_AXIS_X - 0.055, LID_AXIS_Y + 0.016, TOP_Z)),
        material=rail_steel,
        name="hinge_leaf_0",
    )
    frame.visual(
        Box((0.040, 0.012, 0.018)),
        origin=Origin(xyz=(LID_AXIS_X + 0.055, LID_AXIS_Y + 0.016, TOP_Z)),
        material=rail_steel,
        name="hinge_leaf_1",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.258, 0.018, DRAWER_FRONT_H)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=drawer_laminate,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.012, DRAWER_BODY_D - 0.012, DRAWER_BODY_H)),
        origin=Origin(xyz=(-(DRAWER_BODY_W / 2.0) + 0.006, 0.149, 0.0)),
        material=drawer_laminate,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((0.012, DRAWER_BODY_D - 0.012, DRAWER_BODY_H)),
        origin=Origin(xyz=((DRAWER_BODY_W / 2.0) - 0.006, 0.149, 0.0)),
        material=drawer_laminate,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((DRAWER_BODY_W, DRAWER_BODY_D - 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.149, -(DRAWER_BODY_H / 2.0) + 0.006)),
        material=drawer_laminate,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((DRAWER_BODY_W, 0.012, DRAWER_BODY_H)),
        origin=Origin(xyz=(0.0, DRAWER_BODY_D - 0.006, 0.0)),
        material=drawer_laminate,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.100, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=pull_dark,
        name="pull_bar",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, LID_T)),
        origin=Origin(xyz=(0.0, -0.074, 0.0)),
        material=laminate,
        name="lid_panel",
    )
    lid.visual(
        Box((0.184, 0.126, 0.018)),
        origin=Origin(xyz=(0.0, -0.072, -0.018)),
        material=drawer_laminate,
        name="lid_plug",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.160, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -0.008, -0.005)),
        material=rail_steel,
        name="hinge_leaf",
    )

    model.articulation(
        "frame_to_drawer",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=drawer,
        origin=Origin(xyz=(DRAWER_CENTER_X, DRAWER_FRONT_Y, DRAWER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=DRAWER_TRAVEL, effort=80.0, velocity=0.35),
    )
    model.articulation(
        "frame_to_lid",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lid,
        origin=Origin(xyz=(LID_AXIS_X, LID_AXIS_Y, LID_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=12.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    drawer = object_model.get_part("drawer")
    lid = object_model.get_part("lid")
    drawer_slide = object_model.get_articulation("frame_to_drawer")
    lid_hinge = object_model.get_articulation("frame_to_lid")

    top_front_aabb = ctx.part_element_world_aabb(frame, elem="top_front")
    lid_panel_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    if top_front_aabb is not None and lid_panel_aabb is not None:
        lid_top_delta = abs(lid_panel_aabb[1][2] - top_front_aabb[1][2])
        front_clearance = top_front_aabb[1][1] - lid_panel_aabb[0][1]
        ctx.check(
            "lid sits flush in desktop at rest",
            lid_top_delta <= 0.001 and 0.001 <= front_clearance <= 0.008,
            details=f"lid_top_delta={lid_top_delta:.4f}, front_clearance={front_clearance:.4f}",
        )
    else:
        ctx.fail("lid sits flush in desktop at rest", "Missing AABB for desktop or lid panel.")

    drawer_front_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_front")
    if top_front_aabb is not None and drawer_front_aabb is not None:
        overhang = top_front_aabb[0][1] - drawer_front_aabb[0][1]
        ctx.check(
            "drawer front nests below top overhang",
            0.035 <= -overhang <= 0.070,
            details=f"top_front_y={top_front_aabb[0][1]:.4f}, drawer_front_y={drawer_front_aabb[0][1]:.4f}, overhang={overhang:.4f}",
        )
    else:
        ctx.fail("drawer front nests below top overhang", "Missing AABB for desktop or drawer front.")

    closed_drawer_body = ctx.part_element_world_aabb(drawer, elem="drawer_bottom")
    guide_outer = ctx.part_element_world_aabb(frame, elem="guide_outer")
    guide_inner = ctx.part_element_world_aabb(frame, elem="guide_inner")

    closed_drawer_pos = ctx.part_world_position(drawer)
    closed_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({drawer_slide: DRAWER_TRAVEL, lid_hinge: 1.05}):
        open_drawer_pos = ctx.part_world_position(drawer)
        open_drawer_body = ctx.part_element_world_aabb(drawer, elem="drawer_bottom")
        open_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")

        ctx.check(
            "drawer extends toward the front",
            closed_drawer_pos is not None
            and open_drawer_pos is not None
            and open_drawer_pos[1] < closed_drawer_pos[1] - 0.18,
            details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
        )

        if open_drawer_body is not None and guide_outer is not None and guide_inner is not None:
            outer_overlap = _interval_overlap(open_drawer_body, guide_outer, 1)
            inner_overlap = _interval_overlap(open_drawer_body, guide_inner, 1)
            ctx.check(
                "drawer remains engaged with both guide rails at full travel",
                outer_overlap >= 0.040 and inner_overlap >= 0.040,
                details=f"outer_overlap={outer_overlap:.4f}, inner_overlap={inner_overlap:.4f}",
            )
        else:
            ctx.fail("drawer remains engaged with both guide rails at full travel", "Missing AABB for drawer body or guides.")

        ctx.check(
            "lid opens upward from the rear hinge",
            closed_lid_panel is not None
            and open_lid_panel is not None
            and open_lid_panel[1][2] > closed_lid_panel[1][2] + 0.10
            and open_lid_panel[1][1] > closed_lid_panel[1][1] + 0.005,
            details=f"closed={closed_lid_panel}, open={open_lid_panel}",
        )

    return ctx.report()


object_model = build_object_model()
