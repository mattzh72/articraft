from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_L = 1.24
BODY_W = 0.72
BODY_H = 0.90
BODY_CORNER_R = 0.035

INNER_L = 1.06
INNER_W = 0.54
FLOOR_Z = 0.12

RAIL_WIDTH = 0.03
RAIL_HEIGHT = 0.035
RUNNER_WIDTH = 0.010
RUNNER_HEIGHT = 0.001
RUNNER_Y = 0.274
RAIL_Y = 0.295
RAIL_LENGTH = 1.12
END_STOP_Y = 0.28

LID_L = 0.62
LID_W = 0.558
LID_T = 0.014
LID_FRAME_BAND = 0.03
LID_GLASS_T = 0.004
LID_TRAVEL = 0.26
LOWER_LID_X = -0.23
UPPER_LID_X = 0.23
LOWER_LID_Z = BODY_H + 0.008
UPPER_LID_Z = BODY_H + 0.034

DRAIN_X = 0.39
HINGE_Y = -0.255
FLAP_W = 0.068
FLAP_D = 0.060
FLAP_T = 0.003
FLAP_HINGE_BLOCK_L = 0.036
FLAP_HINGE_BLOCK_D = 0.006
FLAP_HINGE_BLOCK_T = 0.006
FLAP_AXIS_Z = FLOOR_Z + 0.007

KNOB_X = BODY_L / 2.0 + 0.024
KNOB_Z = 0.62
KNOB_SHAFT_L = 0.012


def _build_body_shell() -> object:
    outer = (
        cq.Workplane("XY")
        .box(BODY_L, BODY_W, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER_R)
    )
    inner = cq.Workplane("XY").box(
        INNER_L,
        INNER_W,
        BODY_H - FLOOR_Z + 0.03,
        centered=(True, True, False),
    ).translate((0.0, 0.0, FLOOR_Z))
    drain_cut = (
        cq.Workplane("XY")
        .center(DRAIN_X, HINGE_Y + (FLAP_D * 0.5))
        .circle(0.015)
        .extrude(FLOOR_Z + 0.01)
    )
    return outer.cut(inner).cut(drain_cut)


def _build_lid_frame(*, handle_side: float) -> object:
    frame = cq.Workplane("XY").box(LID_L, LID_W, LID_T)
    frame = frame.cut(
        cq.Workplane("XY").box(
            LID_L - (2.0 * LID_FRAME_BAND),
            LID_W - (2.0 * LID_FRAME_BAND),
            LID_T + 0.002,
        )
    )
    handle = cq.Workplane("XY").box(0.16, 0.024, 0.010).translate(
        (handle_side * (LID_L * 0.5 - 0.07), 0.0, LID_T * 0.5 + 0.004)
    )
    return frame.union(handle)


def _build_drain_ring() -> object:
    return (
        cq.Workplane("XY")
        .center(DRAIN_X, HINGE_Y + (FLAP_D * 0.5))
        .circle(0.026)
        .circle(0.016)
        .extrude(0.004)
        .translate((0.0, 0.0, FLOOR_Z))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_freezer")

    model.material("shell_white", rgba=(0.93, 0.95, 0.96, 1.0))
    model.material("trim_gray", rgba=(0.55, 0.58, 0.61, 1.0))
    model.material("liner_white", rgba=(0.98, 0.99, 0.99, 1.0))
    model.material("frame_gray", rgba=(0.42, 0.45, 0.49, 1.0))
    model.material("glass_blue", rgba=(0.60, 0.77, 0.86, 0.35))
    model.material("rubber_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("metal_dark", rgba=(0.30, 0.32, 0.35, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_build_body_shell(), "freezer_shell"),
        material="shell_white",
        name="shell",
    )
    cabinet.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -RAIL_Y, BODY_H + (RAIL_HEIGHT * 0.5))),
        material="trim_gray",
        name="rail_front",
    )
    cabinet.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, RAIL_Y, BODY_H + (RAIL_HEIGHT * 0.5))),
        material="trim_gray",
        name="rail_rear",
    )
    cabinet.visual(
        Box((0.008, END_STOP_Y * 2.0, 0.020)),
        origin=Origin(xyz=((BODY_L * 0.5) - 0.004, 0.0, BODY_H + 0.010)),
        material="trim_gray",
        name="stop_end",
    )
    cabinet.visual(
        Box((0.008, END_STOP_Y * 2.0, 0.020)),
        origin=Origin(xyz=(-(BODY_L * 0.5) + 0.004, 0.0, BODY_H + 0.010)),
        material="trim_gray",
        name="stop_end_1",
    )
    cabinet.visual(
        Box((RAIL_LENGTH - 0.02, RUNNER_WIDTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(0.0, -RUNNER_Y, BODY_H + (RUNNER_HEIGHT * 0.5))),
        material="metal_dark",
        name="runner_front",
    )
    cabinet.visual(
        Box((RAIL_LENGTH - 0.02, RUNNER_WIDTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(0.0, RUNNER_Y, BODY_H + (RUNNER_HEIGHT * 0.5))),
        material="metal_dark",
        name="runner_rear",
    )
    cabinet.visual(
        Box((RAIL_LENGTH - 0.04, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.277, UPPER_LID_Z - (LID_T * 0.5) - 0.004)),
        material="metal_dark",
        name="guide_front",
    )
    cabinet.visual(
        Box((RAIL_LENGTH - 0.04, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.277, UPPER_LID_Z - (LID_T * 0.5) - 0.004)),
        material="metal_dark",
        name="guide_rear",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_drain_ring(), "drain_ring"),
        material="liner_white",
        name="drain_seat",
    )
    cabinet.visual(
        Box((0.024, 0.120, 0.110)),
        origin=Origin(xyz=(BODY_L * 0.5 + 0.012, 0.0, KNOB_Z)),
        material="trim_gray",
        name="control_bezel",
    )
    for x_offset, name in ((-0.027, "hinge_ear_a"), (0.027, "hinge_ear_b")):
        cabinet.visual(
            Box((0.014, FLAP_HINGE_BLOCK_D, 0.014)),
            origin=Origin(
                xyz=(
                    DRAIN_X + x_offset,
                    HINGE_Y - (FLAP_HINGE_BLOCK_D * 0.5),
                    FLOOR_Z + 0.007,
                )
            ),
            material="liner_white",
            name=name,
        )

    lower_lid = model.part("lower_lid")
    lower_lid.visual(
        mesh_from_cadquery(_build_lid_frame(handle_side=-1.0), "lower_lid_frame"),
        material="frame_gray",
        name="frame",
    )
    lower_lid.visual(
        Box((LID_L - (2.0 * LID_FRAME_BAND), LID_W - (2.0 * LID_FRAME_BAND), LID_GLASS_T)),
        material="glass_blue",
        name="glass",
    )

    upper_lid = model.part("upper_lid")
    upper_lid.visual(
        mesh_from_cadquery(_build_lid_frame(handle_side=1.0), "upper_lid_frame"),
        material="frame_gray",
        name="frame",
    )
    upper_lid.visual(
        Box((LID_L - (2.0 * LID_FRAME_BAND), LID_W - (2.0 * LID_FRAME_BAND), LID_GLASS_T)),
        material="glass_blue",
        name="glass",
    )

    drain_flap = model.part("drain_flap")
    drain_flap.visual(
        Box((FLAP_W, FLAP_D, FLAP_T)),
        origin=Origin(xyz=(0.0, FLAP_D * 0.5, -(FLAP_T * 0.5))),
        material="liner_white",
        name="cover",
    )
    drain_flap.visual(
        Box((FLAP_HINGE_BLOCK_L, FLAP_HINGE_BLOCK_D, FLAP_HINGE_BLOCK_T)),
        origin=Origin(xyz=(0.0, -(FLAP_HINGE_BLOCK_D * 0.5), 0.0)),
        material="liner_white",
        name="hinge_block",
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        Cylinder(radius=0.005, length=KNOB_SHAFT_L),
        origin=Origin(xyz=(0.0, 0.0, KNOB_SHAFT_L * 0.5)),
        material="metal_dark",
        name="shaft",
    )
    control_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.042,
                0.024,
                body_style="skirted",
                top_diameter=0.034,
                crown_radius=0.004,
                edge_radius=0.0015,
                center=False,
            ),
            "control_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, KNOB_SHAFT_L)),
        material="rubber_dark",
        name="cap",
    )

    model.articulation(
        "lower_lid_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_lid,
        origin=Origin(xyz=(LOWER_LID_X, 0.0, LOWER_LID_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=LID_TRAVEL),
    )
    model.articulation(
        "upper_lid_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_lid,
        origin=Origin(xyz=(UPPER_LID_X, 0.0, UPPER_LID_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=LID_TRAVEL),
    )
    model.articulation(
        "drain_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=drain_flap,
        origin=Origin(xyz=(DRAIN_X, HINGE_Y, FLAP_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=control_knob,
        origin=Origin(xyz=(KNOB_X, 0.0, KNOB_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lower_lid = object_model.get_part("lower_lid")
    upper_lid = object_model.get_part("upper_lid")
    drain_flap = object_model.get_part("drain_flap")
    control_knob = object_model.get_part("control_knob")

    lower_slide = object_model.get_articulation("lower_lid_slide")
    upper_slide = object_model.get_articulation("upper_lid_slide")
    flap_hinge = object_model.get_articulation("drain_flap_hinge")
    knob_spin = object_model.get_articulation("knob_spin")

    ctx.expect_contact(
        lower_lid,
        cabinet,
        elem_a="frame",
        elem_b="runner_front",
        name="lower lid rides on the front runner",
    )
    ctx.expect_contact(
        lower_lid,
        cabinet,
        elem_a="frame",
        elem_b="runner_rear",
        name="lower lid rides on the rear runner",
    )
    ctx.expect_gap(
        upper_lid,
        lower_lid,
        axis="z",
        positive_elem="frame",
        negative_elem="frame",
        min_gap=0.0005,
        max_gap=0.004,
        name="upper lid clears the lower lid on the stepped track",
    )

    if lower_slide.motion_limits is not None and lower_slide.motion_limits.upper is not None:
        lower_rest = ctx.part_world_position(lower_lid)
        with ctx.pose({lower_slide: lower_slide.motion_limits.upper}):
            lower_open = ctx.part_world_position(lower_lid)
            ctx.expect_overlap(
                lower_lid,
                cabinet,
                axes="y",
                elem_a="frame",
                min_overlap=0.54,
                name="lower lid stays centered between the side rails",
            )
        ctx.check(
            "lower lid slides toward the knob end",
            lower_rest is not None
            and lower_open is not None
            and lower_open[0] > lower_rest[0] + 0.20,
            details=f"rest={lower_rest}, open={lower_open}",
        )

    if upper_slide.motion_limits is not None and upper_slide.motion_limits.upper is not None:
        upper_rest = ctx.part_world_position(upper_lid)
        with ctx.pose({upper_slide: upper_slide.motion_limits.upper}):
            upper_open = ctx.part_world_position(upper_lid)
            ctx.expect_overlap(
                upper_lid,
                cabinet,
                axes="y",
                elem_a="frame",
                min_overlap=0.54,
                name="upper lid stays centered between the side rails",
            )
        ctx.check(
            "upper lid slides away from the knob end",
            upper_rest is not None
            and upper_open is not None
            and upper_open[0] < upper_rest[0] - 0.20,
            details=f"rest={upper_rest}, open={upper_open}",
        )

    ctx.expect_gap(
        drain_flap,
        cabinet,
        axis="z",
        positive_elem="cover",
        negative_elem="drain_seat",
        max_gap=0.002,
        max_penetration=0.0,
        name="drain flap sits down over the drain seat",
    )
    if flap_hinge.motion_limits is not None and flap_hinge.motion_limits.upper is not None:
        closed_cover = ctx.part_element_world_aabb(drain_flap, elem="cover")
        with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
            open_cover = ctx.part_element_world_aabb(drain_flap, elem="cover")
        ctx.check(
            "drain flap lifts clear when opened",
            closed_cover is not None
            and open_cover is not None
            and open_cover[1][2] > closed_cover[1][2] + 0.03,
            details=f"closed={closed_cover}, open={open_cover}",
        )

    ctx.expect_contact(
        control_knob,
        cabinet,
        elem_a="shaft",
        elem_b="control_bezel",
        name="control knob shaft seats into the end-wall bezel",
    )
    ctx.check(
        "control knob uses continuous rotation",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
