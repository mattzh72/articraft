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


BODY_L = 0.380
BODY_W = 0.130
BODY_H = 0.070
FRONT_X = -BODY_L / 2.0
REAR_X = BODY_L / 2.0

SLOT_W = 0.108
SLOT_H = 0.014
SLOT_Z = 0.024

HINGE_X = REAR_X + 0.004
HINGE_Z = BODY_H + 0.008
HINGE_R = 0.004
HINGE_LEN = 0.132

LID_L = 0.365
LID_W = 0.136
LID_T = 0.018
LID_REAR_GAP = 0.010

TRAY_L = 0.270
TRAY_W = SLOT_W
TRAY_T = 0.006
TRAY_REST_PROTRUSION = 0.035
TRAY_TRAVEL = 0.160


def _build_body_shell() -> cq.Workplane:
    """Scanner base with a real through slot for the film carrier."""
    body = cq.Workplane("XY").box(BODY_L, BODY_W, BODY_H)
    body = body.edges("|Z").fillet(0.010)
    body = body.translate((0.0, 0.0, BODY_H / 2.0))

    slot_cutter = cq.Workplane("XY").box(BODY_L + 0.030, SLOT_W, SLOT_H + 0.001)
    slot_cutter = slot_cutter.translate((0.0, 0.0, SLOT_Z))
    body = body.cut(slot_cutter)
    return body


def _build_lid_shell() -> cq.Workplane:
    """Closed lid geometry is authored in the lid hinge frame."""
    lid_center_x = -LID_REAR_GAP - LID_L / 2.0
    lid_center_z = 0.002
    lid = cq.Workplane("XY").box(LID_L, LID_W, LID_T)
    lid = lid.edges("|Z").fillet(0.010)
    lid = lid.translate((lid_center_x, 0.0, lid_center_z))

    raised_panel = cq.Workplane("XY").box(LID_L - 0.060, LID_W - 0.036, 0.002)
    raised_panel = raised_panel.edges("|Z").fillet(0.006)
    raised_panel = raised_panel.translate((lid_center_x - 0.004, 0.0, lid_center_z + LID_T / 2.0 + 0.001))
    return lid.union(raised_panel)


def _build_tray_frame() -> cq.Workplane:
    """Thin 35 mm film carrier frame with open frame apertures."""
    tray_center_x = TRAY_L / 2.0 - TRAY_REST_PROTRUSION
    tray = cq.Workplane("XY").box(TRAY_L, TRAY_W, TRAY_T)
    tray = tray.edges("|Z").fillet(0.003)
    tray = tray.translate((tray_center_x, 0.0, 0.0))

    frame_pitch = 0.048
    frame_w = 0.032
    frame_h = 0.040
    first_x = 0.046
    for i in range(4):
        aperture = cq.Workplane("XY").box(frame_w, frame_h, TRAY_T + 0.004)
        aperture = aperture.translate((first_x + i * frame_pitch, 0.0, 0.0))
        tray = tray.cut(aperture)

    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="negative_film_scanner")

    body_plastic = model.material("satin_charcoal", color=(0.035, 0.037, 0.040, 1.0))
    lid_plastic = model.material("warm_black_plastic", color=(0.012, 0.012, 0.014, 1.0))
    dark_trim = model.material("matte_black_trim", color=(0.002, 0.002, 0.003, 1.0))
    tray_plastic = model.material("tray_black_plastic", color=(0.018, 0.018, 0.020, 1.0))
    metal = model.material("brushed_steel", color=(0.68, 0.69, 0.66, 1.0))
    film_brown = model.material("amber_negative_film", color=(0.55, 0.25, 0.07, 0.62))
    frame_orange = model.material("exposed_negative_frames", color=(0.95, 0.52, 0.13, 0.72))
    pale_clear = model.material("clear_sprocket_holes", color=(1.0, 0.82, 0.55, 0.72))

    body = model.part("body")
    slot_bottom = SLOT_Z - SLOT_H / 2.0
    slot_top = SLOT_Z + SLOT_H / 2.0
    side_wall_w = (BODY_W - SLOT_W) / 2.0
    body.visual(
        Box((BODY_L, BODY_W, slot_bottom)),
        origin=Origin(xyz=(0.0, 0.0, slot_bottom / 2.0)),
        material=body_plastic,
        name="bottom_deck",
    )
    body.visual(
        Box((BODY_L, BODY_W, BODY_H - slot_top)),
        origin=Origin(xyz=(0.0, 0.0, slot_top + (BODY_H - slot_top) / 2.0)),
        material=body_plastic,
        name="top_deck",
    )
    for idx, sy in enumerate((-1.0, 1.0)):
        body.visual(
            Box((BODY_L, side_wall_w, BODY_H)),
            origin=Origin(xyz=(0.0, sy * (SLOT_W / 2.0 + side_wall_w / 2.0), BODY_H / 2.0)),
            material=body_plastic,
            name=f"side_wall_{idx}",
        )
    body.visual(
        Box((0.060, BODY_W, SLOT_H)),
        origin=Origin(xyz=(REAR_X - 0.030, 0.0, SLOT_Z)),
        material=dark_trim,
        name="rear_slot_stop",
    )

    body.visual(
        Box((0.008, HINGE_LEN, 0.004)),
        origin=Origin(xyz=(REAR_X, 0.0, BODY_H + 0.002)),
        material=metal,
        name="hinge_leaf",
    )

    # Alternating piano-hinge knuckles span the rear width.  Body knuckles are
    # fixed here; lid knuckles live on the lid link and share the same axis.
    knuckle_count = 9
    gap = 0.0015
    knuckle_len = (HINGE_LEN - (knuckle_count - 1) * gap) / knuckle_count
    start_y = -HINGE_LEN / 2.0 + knuckle_len / 2.0
    for i in range(knuckle_count):
        y = start_y + i * (knuckle_len + gap)
        if i % 2 == 0:
            body.visual(
                Cylinder(radius=HINGE_R, length=knuckle_len),
                origin=Origin(
                    xyz=(HINGE_X, y, HINGE_Z),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=metal,
                name=f"hinge_knuckle_{i}",
            )

    # Lift-assist end pins and cheek brackets at both ends of the hinge.
    for idx, sy in enumerate((-1.0, 1.0)):
        body.visual(
            Box((0.012, 0.006, 0.016)),
            origin=Origin(xyz=(REAR_X - 0.002, sy * (BODY_W / 2.0 + 0.002), BODY_H + 0.006)),
            material=metal,
            name=f"assist_bracket_{idx}",
        )
        body.visual(
            Cylinder(radius=0.0026, length=0.016),
            origin=Origin(
                xyz=(HINGE_X, sy * (HINGE_LEN / 2.0 + 0.006), HINGE_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal,
            name=f"assist_pin_{idx}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "lid_shell", tolerance=0.0008),
        material=lid_plastic,
        name="lid_shell",
    )
    lid.visual(
        Box((LID_L - 0.095, LID_W - 0.065, 0.0016)),
        origin=Origin(xyz=(-LID_REAR_GAP - LID_L / 2.0 - 0.006, 0.0, 0.0132)),
        material=dark_trim,
        name="lid_recess",
    )

    for i in range(knuckle_count):
        y = start_y + i * (knuckle_len + gap)
        if i % 2 == 1:
            lid.visual(
                Cylinder(radius=HINGE_R, length=knuckle_len),
                origin=Origin(
                    xyz=(0.0, y, 0.0),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=metal,
                name=f"hinge_knuckle_{i}",
            )
            lid.visual(
                Box((0.010, knuckle_len * 0.84, 0.006)),
                origin=Origin(xyz=(-0.006, y, -0.001)),
                material=metal,
                name=f"hinge_tab_{i}",
            )

    tray = model.part("film_tray")
    tray.visual(
        mesh_from_cadquery(_build_tray_frame(), "tray_frame", tolerance=0.0006),
        origin=Origin(xyz=(0.0, 0.0, SLOT_Z)),
        material=tray_plastic,
        name="tray_frame",
    )
    tray.visual(
        Box((0.040, 0.118, 0.012)),
        origin=Origin(xyz=(-0.025, 0.0, SLOT_Z)),
        material=tray_plastic,
        name="front_pull",
    )
    tray.visual(
        Box((0.210, 0.052, 0.0018)),
        origin=Origin(xyz=(0.104, 0.0, SLOT_Z + 0.0033)),
        material=film_brown,
        name="negative_strip",
    )
    for i in range(4):
        tray.visual(
            Box((0.024, 0.026, 0.0014)),
            origin=Origin(xyz=(0.046 + i * 0.048, 0.0, SLOT_Z + 0.0042)),
            material=frame_orange,
            name=f"film_frame_{i}",
        )
    for i in range(9):
        x = 0.006 + i * 0.024
        for j, y in enumerate((-0.0215, 0.0215)):
            tray.visual(
                Box((0.006, 0.0035, 0.0015)),
                origin=Origin(xyz=(x, y, SLOT_Z + 0.0046)),
                material=pale_clear,
                name=f"sprocket_{i}_{j}",
            )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    model.articulation(
        "body_to_film_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(FRONT_X, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=TRAY_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("film_tray")
    lid_hinge = object_model.get_articulation("body_to_lid")
    tray_slide = object_model.get_articulation("body_to_film_tray")

    with ctx.pose({lid_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="top_deck",
            min_gap=0.0005,
            max_gap=0.004,
            name="closed lid rests just above scanner body",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="top_deck",
            min_overlap=0.10,
            name="lid covers the narrow scanner footprint",
        )
        ctx.expect_within(
            tray,
            body,
            axes="y",
            inner_elem="tray_frame",
            outer_elem="bottom_deck",
            margin=0.006,
            name="film tray fits between slot side walls",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            elem_a="tray_frame",
            elem_b="bottom_deck",
            min_overlap=0.20,
            name="inserted tray is mostly inside the scanner body",
        )
        rest_pos = ctx.part_world_position(tray)
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    with ctx.pose({lid_hinge: 1.0}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "lid hinge lifts the front edge upward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and float(open_lid_aabb[1][2]) > float(closed_lid_aabb[1][2]) + 0.12,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            elem_a="tray_frame",
            elem_b="bottom_deck",
            min_overlap=0.05,
            name="extended tray remains retained in the front slot",
        )
        extended_pos = ctx.part_world_position(tray)

    ctx.check(
        "tray slide pulls out through the front face",
        rest_pos is not None and extended_pos is not None and extended_pos[0] < rest_pos[0] - 0.12,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
