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

BODY_W = 0.390
BODY_D = 0.320
BODY_H = 0.205
CHAMBER_Y = 0.040
CHAMBER_OPEN_W = 0.202
CHAMBER_OPEN_D = 0.148
CHAMBER_DEPTH = 0.158

HINGE_Y = 0.145
HINGE_Z = 0.216
HINGE_ROD_R = 0.0038
HINGE_ROD_L = 0.250

PAN_BODY_W = 0.187
PAN_BODY_D = 0.127
PAN_H = 0.148
PAN_WALL = 0.0022
PAN_BOTTOM = 0.0040
PAN_FLANGE_W = 0.204
PAN_FLANGE_D = 0.144
PAN_FLANGE_T = 0.0020

LID_W = 0.276
LID_D = 0.205
LID_H = 0.032
LID_BOTTOM_Z = -0.003
LID_TUBE_OUTER = 0.0066
LID_TUBE_INNER = 0.0048
LID_TUBE_L = 0.180
WINDOW_W = 0.155
WINDOW_D = 0.102
WINDOW_Y = -0.108

VENT_W = 0.056
VENT_D = 0.028
VENT_H = 0.010
VENT_HINGE_Y = -0.030
VENT_HINGE_Z = 0.029
VENT_SLOT_Y = -0.044

PANEL_TOP_Z = BODY_H + 0.005
DIAL_Y = -0.108
DIAL_OUTER_R = 0.036
DIAL_INNER_R = 0.022
BUTTON_CAP_W = 0.028
BUTTON_CAP_D = 0.018
BUTTON_CAP_H = 0.005
BUTTON_STEM_W = 0.013
BUTTON_STEM_D = 0.008
BUTTON_STEM_H = 0.010
BUTTON_PROUD = 0.0015
BUTTON_LAYOUT = (
    (-0.053, -0.082),
    (0.053, -0.082),
    (-0.067, -0.112),
    (0.067, -0.112),
    (0.000, -0.150),
)


def _rounded_box(
    width: float,
    depth: float,
    height: float,
    *,
    corner: float,
    top_fillet: float,
):
    solid = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    solid = solid.edges("|Z").fillet(corner)
    solid = solid.edges(">Z").fillet(top_fillet)
    return solid


def _body_shell_shape():
    shell = _rounded_box(
        BODY_W,
        BODY_D,
        BODY_H,
        corner=0.028,
        top_fillet=0.012,
    )
    chamber_cut = (
        cq.Workplane("XY")
        .box(
            CHAMBER_OPEN_W,
            CHAMBER_OPEN_D,
            CHAMBER_DEPTH + 0.006,
            centered=(True, True, False),
        )
        .translate((0.0, CHAMBER_Y, BODY_H - CHAMBER_DEPTH))
    )
    shell = shell.cut(chamber_cut)

    control_rise = _rounded_box(
        0.244,
        0.092,
        0.006,
        corner=0.010,
        top_fillet=0.003,
    ).translate((0.0, -0.106, BODY_H - 0.001))
    shell = shell.union(control_rise)

    for offset_x in (-0.122, 0.122):
        rear_mount = _rounded_box(
            0.028,
            0.018,
            0.022,
            corner=0.004,
            top_fillet=0.003,
        ).translate((offset_x, HINGE_Y, BODY_H - 0.001))
        shell = shell.union(rear_mount)

    for button_x, button_y in BUTTON_LAYOUT:
        button_cut = (
            cq.Workplane("XY")
            .box(
                BUTTON_STEM_W + 0.002,
                BUTTON_STEM_D + 0.002,
                0.018,
                centered=(True, True, False),
            )
            .translate((button_x, button_y, PANEL_TOP_Z - 0.016))
        )
        shell = shell.cut(button_cut)

    return shell


def _pan_shape():
    bucket = (
        cq.Workplane("XY")
        .box(PAN_BODY_W, PAN_BODY_D, PAN_H, centered=(True, True, False))
        .translate((0.0, 0.0, -PAN_H))
    )
    bucket = bucket.edges("|Z").fillet(0.010)
    inner_cut = (
        cq.Workplane("XY")
        .box(
            PAN_BODY_W - (2.0 * PAN_WALL),
            PAN_BODY_D - (2.0 * PAN_WALL),
            PAN_H,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -PAN_H + PAN_BOTTOM))
    )
    bucket = bucket.cut(inner_cut)

    flange = cq.Workplane("XY").box(
        PAN_FLANGE_W,
        PAN_FLANGE_D,
        PAN_FLANGE_T,
        centered=(True, True, False),
    )
    flange_cut = (
        cq.Workplane("XY")
        .box(
            PAN_BODY_W - (2.0 * PAN_WALL),
            PAN_BODY_D - (2.0 * PAN_WALL),
            PAN_FLANGE_T + 0.003,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -0.001))
    )
    flange = flange.cut(flange_cut)
    return bucket.union(flange)


def _lid_shell_shape():
    lid = _rounded_box(
        LID_W,
        LID_D,
        LID_H,
        corner=0.014,
        top_fillet=0.008,
    ).translate((0.0, -LID_D / 2.0, LID_BOTTOM_Z))

    cavity_cut = (
        cq.Workplane("XY")
        .box(
            LID_W - 0.022,
            LID_D - 0.028,
            LID_H,
            centered=(True, True, False),
        )
        .translate((0.0, -(LID_D / 2.0) - 0.004, LID_BOTTOM_Z - 0.001))
    )
    lid = lid.cut(cavity_cut)

    window_cut = (
        cq.Workplane("XY")
        .box(WINDOW_W, WINDOW_D, 0.024, centered=(True, True, False))
        .translate((0.0, WINDOW_Y, 0.012))
    )
    lid = lid.cut(window_cut)

    for offset_x in (-0.015, 0.0, 0.015):
        slot_cut = (
            cq.Workplane("XY")
            .box(0.006, 0.018, 0.008, centered=(True, True, False))
            .translate((offset_x, VENT_SLOT_Y, 0.021))
        )
        lid = lid.cut(slot_cut)

    for offset_x in (-0.122, 0.122):
        rear_relief = (
            cq.Workplane("XY")
            .box(0.038, 0.030, 0.026, centered=(True, True, False))
            .translate((offset_x, -0.006, -0.004))
        )
        lid = lid.cut(rear_relief)

    hinge_tube = (
        cq.Workplane("YZ")
        .circle(LID_TUBE_OUTER)
        .circle(LID_TUBE_INNER)
        .extrude(LID_TUBE_L / 2.0, both=True)
    )
    return lid.union(hinge_tube)


def _vent_cap_shape():
    cap = _rounded_box(
        VENT_W,
        VENT_D,
        VENT_H,
        corner=0.004,
        top_fillet=0.0025,
    ).translate((0.0, -VENT_D / 2.0, 0.0))
    return cap


def _dial_ring_shape():
    ring = (
        cq.Workplane("XY")
        .circle(DIAL_OUTER_R)
        .circle(DIAL_INNER_R)
        .extrude(0.008)
    )
    ring = ring.edges(">Z").fillet(0.0015)
    return ring


def _button_shape():
    cap = _rounded_box(
        BUTTON_CAP_W,
        BUTTON_CAP_D,
        BUTTON_CAP_H,
        corner=0.003,
        top_fillet=0.0015,
    ).translate((0.0, 0.0, BUTTON_PROUD))
    stem = (
        cq.Workplane("XY")
        .box(
            BUTTON_STEM_W,
            BUTTON_STEM_D,
            BUTTON_STEM_H + BUTTON_PROUD + 0.0004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -BUTTON_STEM_H))
    )
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_bread_maker")

    body_plastic = model.material("body_plastic", rgba=(0.20, 0.22, 0.24, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.55, 0.70, 0.78, 0.32))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    button_satin = model.material("button_satin", rgba=(0.77, 0.79, 0.82, 1.0))
    dial_graphite = model.material("dial_graphite", rgba=(0.14, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "bread_maker_body_shell"),
        material=body_plastic,
        name="housing_shell",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, CHAMBER_Y, BODY_H - PAN_H + PAN_BOTTOM - 0.007)),
        material=steel,
        name="spindle_boss",
    )

    body.visual(
        mesh_from_cadquery(_pan_shape(), "bread_maker_pan"),
        origin=Origin(xyz=(0.0, CHAMBER_Y, BODY_H)),
        material=brushed_metal,
        name="pan_shell",
    )
    body.visual(
        Cylinder(radius=0.019, length=0.004),
        origin=Origin(xyz=(0.0, DIAL_Y, PANEL_TOP_Z + 0.002)),
        material=dark_trim,
        name="dial_hub",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "bread_maker_lid_shell"),
        material=lid_plastic,
        name="lid_shell",
    )
    lid.visual(
        Box((0.171, 0.118, 0.004)),
        origin=Origin(xyz=(0.0, WINDOW_Y, 0.0110)),
        material=glass,
        name="window_glass",
    )

    vent_cap = model.part("vent_cap")
    vent_cap.visual(
        mesh_from_cadquery(_vent_cap_shape(), "bread_maker_vent_cap"),
        material=dark_trim,
        name="vent_cap_shell",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=steel,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.0075, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=steel,
        name="collar",
    )
    spindle.visual(
        Box((0.011, 0.004, 0.006)),
        origin=Origin(xyz=(0.0065, 0.0, 0.014)),
        material=steel,
        name="drive_tab",
    )

    dial_mesh = mesh_from_cadquery(_dial_ring_shape(), "bread_maker_dial_ring")
    dial = model.part("dial")
    dial.visual(
        dial_mesh,
        material=dial_graphite,
        name="dial_ring",
    )
    dial.visual(
        Box((0.006, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, DIAL_OUTER_R - 0.004, 0.0095)),
        material=button_satin,
        name="dial_marker",
    )

    button_mesh = mesh_from_cadquery(_button_shape(), "bread_maker_button")
    for button_index, _ in enumerate(BUTTON_LAYOUT):
        button_part = model.part(f"button_{button_index}")
        button_part.visual(
            button_mesh,
            material=button_satin,
            name="button_shell",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.32),
    )
    model.articulation(
        "lid_to_vent_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(0.0, VENT_HINGE_Y, VENT_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=0.62),
    )
    model.articulation(
        "pan_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spindle,
        origin=Origin(xyz=(0.0, CHAMBER_Y, BODY_H - PAN_H + PAN_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, DIAL_Y, PANEL_TOP_Z + 0.001)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )
    for button_index, (button_x, button_y) in enumerate(BUTTON_LAYOUT):
        model.articulation(
            f"body_to_button_{button_index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=f"button_{button_index}",
            origin=Origin(xyz=(button_x, button_y, PANEL_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0015,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    vent_cap = object_model.get_part("vent_cap")
    spindle = object_model.get_part("spindle")
    dial = object_model.get_part("dial")
    buttons = [object_model.get_part(f"button_{index}") for index in range(5)]

    lid_hinge = object_model.get_articulation("body_to_lid")
    vent_hinge = object_model.get_articulation("lid_to_vent_cap")
    spindle_joint = object_model.get_articulation("pan_to_spindle")
    dial_joint = object_model.get_articulation("body_to_dial")
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(5)]

    ctx.allow_isolated_part(
        lid,
        reason="The lid is intentionally carried on a rear hinge with clearance instead of fused contact to the housing.",
    )
    ctx.allow_isolated_part(
        vent_cap,
        reason="The vent cap is a small hinged flap carried by the lid with hinge clearance.",
    )
    ctx.allow_isolated_part(
        dial,
        reason="The ring dial is carried on a hidden rotary bearing around the panel hub with a small visible running clearance.",
    )
    for button_part in buttons:
        ctx.allow_isolated_part(
            button_part,
            reason="Each push button is represented with a small visible running clearance above its guided panel aperture.",
        )
    ctx.allow_overlap(
        body,
        spindle,
        elem_a="pan_shell",
        elem_b="shaft",
        reason="The kneading spindle is intentionally represented as passing through the pan-floor bearing region of the thin-wall pan shell.",
    )

    def _center(aabb):
        if aabb is None:
            return None
        min_pt, max_pt = aabb
        return tuple((float(min_pt[i]) + float(max_pt[i])) * 0.5 for i in range(3))

    ctx.check(
        "lid articulation is revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={lid_hinge.articulation_type}",
    )
    ctx.check(
        "vent articulation is revolute",
        vent_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={vent_hinge.articulation_type}",
    )
    ctx.check(
        "spindle articulation is continuous",
        spindle_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spindle_joint.articulation_type}",
    )
    ctx.check(
        "dial articulation is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )
    for button_index, button_joint in enumerate(button_joints):
        ctx.check(
            f"button_{button_index} articulation is prismatic",
            button_joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={button_joint.articulation_type}",
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="pan_shell",
            min_gap=0.0005,
            max_gap=0.010,
            name="closed lid clears pan rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="pan_shell",
            min_overlap=0.120,
            name="lid covers the baking chamber footprint",
        )
        closed_lid = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 1.20}):
        open_lid = ctx.part_world_aabb(lid)

    if closed_lid is not None and open_lid is not None:
        ctx.check(
            "lid opens upward",
            float(open_lid[1][2]) > float(closed_lid[1][2]) + 0.110,
            details=f"closed_max_z={closed_lid[1][2]}, open_max_z={open_lid[1][2]}",
        )

    with ctx.pose({vent_hinge: 0.0}):
        vent_closed = ctx.part_world_aabb(vent_cap)
    with ctx.pose({vent_hinge: 0.50}):
        vent_open = ctx.part_world_aabb(vent_cap)
    if vent_closed is not None and vent_open is not None:
        ctx.check(
            "vent cap opens upward",
            float(vent_open[1][2]) > float(vent_closed[1][2]) + 0.010,
            details=f"closed_max_z={vent_closed[1][2]}, open_max_z={vent_open[1][2]}",
        )

    with ctx.pose({spindle_joint: 0.0}):
        tab_a = _center(ctx.part_element_world_aabb(spindle, elem="drive_tab"))
    with ctx.pose({spindle_joint: math.pi / 2.0}):
        tab_b = _center(ctx.part_element_world_aabb(spindle, elem="drive_tab"))
    if tab_a is not None and tab_b is not None:
        planar_shift = math.hypot(tab_b[0] - tab_a[0], tab_b[1] - tab_a[1])
        ctx.check(
            "spindle rotation visibly moves the drive tab",
            planar_shift > 0.007,
            details=f"tab_a={tab_a}, tab_b={tab_b}, planar_shift={planar_shift}",
        )

    with ctx.pose({dial_joint: 0.0}):
        marker_a = _center(ctx.part_element_world_aabb(dial, elem="dial_marker"))
    with ctx.pose({dial_joint: math.pi / 2.0}):
        marker_b = _center(ctx.part_element_world_aabb(dial, elem="dial_marker"))
    if marker_a is not None and marker_b is not None:
        marker_shift = math.hypot(marker_b[0] - marker_a[0], marker_b[1] - marker_a[1])
        ctx.check(
            "dial rotation visibly moves the marker",
            marker_shift > 0.040,
            details=f"marker_a={marker_a}, marker_b={marker_b}, marker_shift={marker_shift}",
        )

    for button_index, (button_part, button_joint) in enumerate(zip(buttons, button_joints)):
        limits = button_joint.motion_limits
        upper = 0.0 if limits is None or limits.upper is None else float(limits.upper)
        rest_pos = ctx.part_world_position(button_part)
        with ctx.pose({button_joint: upper}):
            pressed_pos = ctx.part_world_position(button_part)
        ctx.check(
            f"button_{button_index} presses downward",
            rest_pos is not None
            and pressed_pos is not None
            and float(pressed_pos[2]) < float(rest_pos[2]) - 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
