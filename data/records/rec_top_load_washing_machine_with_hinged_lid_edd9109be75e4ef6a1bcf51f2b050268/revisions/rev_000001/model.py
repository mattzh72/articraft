from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_WIDTH = 0.68
BODY_DEPTH = 0.72
BODY_HEIGHT = 1.08
BODY_CORNER_RADIUS = 0.04

OPENING_WIDTH = 0.48
OPENING_DEPTH = 0.42
OPENING_CENTER_Y = -0.055
OPENING_CORNER_RADIUS = 0.055
CAVITY_DEPTH = 0.60
WELL_DEPTH = 0.63
WELL_RADIUS = 0.235

LID_WIDTH = 0.54
LID_DEPTH = 0.44
LID_THICKNESS = 0.026
LID_HINGE_Y = OPENING_CENTER_Y + LID_DEPTH / 2.0
LID_GLASS_WIDTH = 0.48
LID_GLASS_DEPTH = 0.38
LID_GLASS_SHIFT_Y = -0.018

DRUM_RADIUS = 0.205
DRUM_INNER_RADIUS = 0.188
DRUM_HEIGHT = 0.58
DRUM_BOTTOM_THICKNESS = 0.018
DRUM_TOP_Z = BODY_HEIGHT - 0.050

DISPENSER_WIDTH = 0.145
DISPENSER_DEPTH = 0.110
DISPENSER_THICKNESS = 0.014
DISPENSER_RECESS_DEPTH = 0.0
DISPENSER_CENTER_Y = 0.285
DISPENSER_OFFSET_X = 0.220

BUTTON_WIDTH = 0.044
BUTTON_DEPTH = 0.030
BUTTON_HEIGHT = 0.014
BUTTON_RECESS_DEPTH = 0.0
BUTTON_TRAVEL = 0.0025
BUTTON_X = 0.108
BUTTON_Y = 0.225

KNOB_X = -0.052
KNOB_Y = 0.225
KNOB_DIAMETER = 0.057
KNOB_HEIGHT = 0.031


def _rounded_rect_prism(width: float, depth: float, height: float, radius: float):
    return (
        cq.Workplane("XY")
        .placeSketch(cq.Sketch().rect(width, depth).vertices().fillet(radius))
        .extrude(height)
    )


def _build_body_shape():
    shell = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER_RADIUS)
    )

    opening_cut = _rounded_rect_prism(
        OPENING_WIDTH,
        OPENING_DEPTH,
        CAVITY_DEPTH,
        OPENING_CORNER_RADIUS,
    ).translate((0.0, OPENING_CENTER_Y, BODY_HEIGHT - CAVITY_DEPTH))
    shell = shell.cut(opening_cut)

    well_cut = (
        cq.Workplane("XY")
        .circle(WELL_RADIUS)
        .extrude(WELL_DEPTH)
        .translate((0.0, OPENING_CENTER_Y, BODY_HEIGHT - WELL_DEPTH))
    )
    shell = shell.cut(well_cut)

    return shell


def _build_lid_frame_shape(width: float, depth: float, thickness: float):
    frame = _rounded_rect_prism(width, depth, thickness, 0.028)
    window_cut = _rounded_rect_prism(
        width - 0.080,
        depth - 0.090,
        thickness + 0.004,
        0.020,
    ).translate((0.0, LID_GLASS_SHIFT_Y, -0.002))
    frame = frame.cut(window_cut)
    front_grip = _rounded_rect_prism(0.160, 0.024, 0.010, 0.008).translate(
        (0.0, -depth / 2.0 + 0.004, 0.0)
    )
    frame = frame.union(front_grip)
    return frame.translate((0.0, -depth / 2.0, 0.0))


def _build_dispenser_lid_shape(width: float, depth: float, thickness: float):
    lid = _rounded_rect_prism(width, depth, thickness, 0.016)
    inset = _rounded_rect_prism(
        width - 0.022,
        depth - 0.022,
        0.0025,
        0.012,
    ).translate((0.0, 0.0, thickness - 0.0025))
    lid = lid.cut(inset)
    front_tab = _rounded_rect_prism(width * 0.36, 0.016, 0.005, 0.005).translate(
        (0.0, -depth / 2.0 + 0.006, thickness - 0.0015)
    )
    lid = lid.union(front_tab)
    return lid.translate((0.0, -depth / 2.0, 0.0))


def _build_drum_shape():
    basket_outer = (
        cq.Workplane("XY").circle(DRUM_RADIUS).extrude(DRUM_HEIGHT).translate((0.0, 0.0, -DRUM_HEIGHT))
    )
    basket_inner = (
        cq.Workplane("XY")
        .circle(DRUM_INNER_RADIUS)
        .extrude(DRUM_HEIGHT - DRUM_BOTTOM_THICKNESS)
        .translate((0.0, 0.0, -DRUM_HEIGHT + DRUM_BOTTOM_THICKNESS))
    )
    basket = basket_outer.cut(basket_inner)

    rim = (
        cq.Workplane("XY")
        .circle(DRUM_RADIUS + 0.010)
        .circle(DRUM_INNER_RADIUS - 0.004)
        .extrude(0.010)
        .translate((0.0, 0.0, -0.010))
    )
    basket = basket.union(rim)

    agitator_column = (
        cq.Workplane("XY")
        .circle(0.058)
        .extrude(0.190)
        .translate((0.0, 0.0, -DRUM_HEIGHT + DRUM_BOTTOM_THICKNESS))
    )
    agitator_cap = (
        cq.Workplane("XY")
        .circle(0.040)
        .extrude(0.060)
        .translate((0.0, 0.0, -DRUM_HEIGHT + DRUM_BOTTOM_THICKNESS + 0.186))
    )
    basket = basket.union(agitator_column).union(agitator_cap)

    fin_center_z = -DRUM_HEIGHT + 0.128
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        fin = (
            cq.Workplane("XY")
            .box(0.014, 0.080, 0.110, centered=(True, True, True))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
            .translate((0.0, 0.0, fin_center_z))
        )
        basket = basket.union(fin)

    return basket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.94, 0.95, 0.96, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.76, 0.78, 0.80, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.28, 0.34, 0.37, 0.35))
    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.77, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.11, 0.12, 1.0))
    button_silver = model.material("button_silver", rgba=(0.73, 0.75, 0.78, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "washer_body"),
        material=cabinet_white,
        name="cabinet_shell",
    )
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_frame_shape(LID_WIDTH, LID_DEPTH, LID_THICKNESS), "washer_lid_frame"),
        material=trim_silver,
        name="lid_frame",
    )
    lid.visual(
        Box((LID_GLASS_WIDTH, LID_GLASS_DEPTH, 0.006)),
        origin=Origin(
            xyz=(0.0, -LID_DEPTH / 2.0 + LID_GLASS_SHIFT_Y, 0.011),
        ),
        material=glass_tint,
        name="lid_glass",
    )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(_build_drum_shape(), "washer_drum"),
        material=stainless,
        name="drum_shell",
    )

    for index, offset_x in enumerate((-DISPENSER_OFFSET_X, DISPENSER_OFFSET_X)):
        dispenser = model.part(f"dispenser_{index}")
        dispenser.visual(
            mesh_from_cadquery(
                _build_dispenser_lid_shape(DISPENSER_WIDTH, DISPENSER_DEPTH, DISPENSER_THICKNESS),
                f"washer_dispenser_{index}",
            ),
            material=trim_silver,
            name="dispenser_shell",
        )
        model.articulation(
            f"body_to_dispenser_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=dispenser,
            origin=Origin(
                xyz=(
                    offset_x,
                    DISPENSER_CENTER_Y + DISPENSER_DEPTH / 2.0,
                    BODY_HEIGHT,
                )
            ),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.0,
                lower=0.0,
                upper=1.35,
            ),
        )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                KNOB_DIAMETER,
                KNOB_HEIGHT,
                body_style="skirted",
                top_diameter=0.041,
                skirt=KnobSkirt(0.067, 0.006, flare=0.12),
                grip=KnobGrip(style="fluted", count=20, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                center=False,
            ),
            "washer_selector_knob",
        ),
        material=control_black,
        name="knob_shell",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_HEIGHT / 2.0)),
        material=button_silver,
        name="button_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, BODY_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.3,
            lower=0.0,
            upper=1.32,
        ),
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, DRUM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=18.0,
        ),
    )

    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(KNOB_X, KNOB_Y, BODY_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
        ),
    )

    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(BUTTON_X, BUTTON_Y, BODY_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    drum = object_model.get_part("drum")
    dispenser_0 = object_model.get_part("dispenser_0")
    dispenser_1 = object_model.get_part("dispenser_1")
    selector_knob = object_model.get_part("selector_knob")
    start_button = object_model.get_part("start_button")

    lid_joint = object_model.get_articulation("body_to_lid")
    dispenser_joint_0 = object_model.get_articulation("body_to_dispenser_0")
    dispenser_joint_1 = object_model.get_articulation("body_to_dispenser_1")
    button_joint = object_model.get_articulation("body_to_start_button")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) / 2.0 for i in range(3))

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Expected a body AABB.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "washer_scale",
            0.65 <= size[0] <= 0.71 and 0.69 <= size[1] <= 0.75 and 1.06 <= size[2] <= 1.10,
            details=f"size={size!r}",
        )

    drum_aabb = ctx.part_world_aabb(drum)
    ctx.check("drum_aabb_present", drum_aabb is not None, "Expected a drum AABB.")
    if drum_aabb is not None and body_aabb is not None:
        drum_mins, drum_maxs = drum_aabb
        body_mins, body_maxs = body_aabb
        drum_height = float(drum_maxs[2] - drum_mins[2])
        top_drop = float(body_maxs[2] - drum_maxs[2])
        ctx.check(
            "drum_reads_deep_and_hollow",
            drum_height >= 0.56 and 0.030 <= top_drop <= 0.070 and float(drum_mins[2]) < 0.48,
            details=f"drum_height={drum_height}, top_drop={top_drop}, drum_min_z={float(drum_mins[2])}",
        )

    ctx.expect_overlap(
        lid,
        drum,
        axes="xy",
        elem_a="lid_glass",
        elem_b="drum_shell",
        min_overlap=0.33,
        name="glass_lid_spans_drum_opening",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_frame",
        negative_elem="cabinet_shell",
        min_gap=0.0,
        max_gap=0.008,
        name="lid_sits_just_above_top_deck",
    )

    body_pos = ctx.part_world_position(body)
    drum_pos = ctx.part_world_position(drum)
    ctx.check(
        "drum_centered_under_opening",
        body_pos is not None
        and drum_pos is not None
        and abs(float(drum_pos[0]) - float(body_pos[0])) <= 0.003
        and abs((float(drum_pos[1]) - float(body_pos[1])) - OPENING_CENTER_Y) <= 0.004,
        details=f"body_pos={body_pos!r}, drum_pos={drum_pos!r}",
    )

    knob_pos = ctx.part_world_position(selector_knob)
    button_pos = ctx.part_world_position(start_button)
    dispenser_pos_0 = ctx.part_world_position(dispenser_0)
    dispenser_pos_1 = ctx.part_world_position(dispenser_1)
    ctx.check(
        "dispenser_lids_behind_controls",
        knob_pos is not None
        and button_pos is not None
        and dispenser_pos_0 is not None
        and dispenser_pos_1 is not None
        and float(dispenser_pos_0[1]) > float(knob_pos[1]) + 0.07
        and float(dispenser_pos_1[1]) > float(button_pos[1]) + 0.07
        and abs(float(dispenser_pos_0[0])) > 0.18
        and abs(float(dispenser_pos_1[0])) > 0.18,
        details=(
            f"knob_pos={knob_pos!r}, button_pos={button_pos!r}, "
            f"dispenser_pos_0={dispenser_pos_0!r}, dispenser_pos_1={dispenser_pos_1!r}"
        ),
    )

    lid_rest_center = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_glass"))
    lid_upper = lid_joint.motion_limits.upper if lid_joint.motion_limits is not None else None
    if lid_upper is not None:
        with ctx.pose({lid_joint: lid_upper}):
            lid_open_center = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_glass"))
        ctx.check(
            "main_lid_opens_upward",
            lid_rest_center is not None
            and lid_open_center is not None
            and lid_open_center[2] > lid_rest_center[2] + 0.12
            and lid_open_center[1] > lid_rest_center[1] + 0.08,
            details=f"rest={lid_rest_center!r}, open={lid_open_center!r}",
        )

    button_rest = ctx.part_world_position(start_button)
    button_upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None
    if button_upper is not None:
        with ctx.pose({button_joint: button_upper}):
            button_pressed = ctx.part_world_position(start_button)
        ctx.check(
            "start_button_presses_down",
            button_rest is not None
            and button_pressed is not None
            and float(button_pressed[2]) < float(button_rest[2]) - 0.002,
            details=f"rest={button_rest!r}, pressed={button_pressed!r}",
        )

    for index, joint in enumerate((dispenser_joint_0, dispenser_joint_1)):
        rest_center = _aabb_center(
            ctx.part_element_world_aabb(object_model.get_part(f"dispenser_{index}"), elem="dispenser_shell")
        )
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        if upper is None:
            continue
        with ctx.pose({joint: upper}):
            open_center = _aabb_center(
                ctx.part_element_world_aabb(object_model.get_part(f"dispenser_{index}"), elem="dispenser_shell")
            )
        ctx.check(
            f"dispenser_{index}_flips_up",
            rest_center is not None
            and open_center is not None
            and open_center[2] > rest_center[2] + 0.04
            and open_center[1] > rest_center[1] + 0.02,
            details=f"rest={rest_center!r}, open={open_center!r}",
        )

    return ctx.report()


object_model = build_object_model()
