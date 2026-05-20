from __future__ import annotations

import cadquery as cq
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
    mesh_from_cadquery,
)


BODY_L = 0.420
BODY_W = 0.130
BODY_H = 0.075
FRONT_X = -BODY_L / 2.0
REAR_X = BODY_L / 2.0
TOP_Z = BODY_H / 2.0
HINGE_X = REAR_X + 0.008
HINGE_Z = TOP_Z + 0.0115
TRAY_SLOT_Z = -0.014


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="film_strip_slide_scanner")

    graphite = model.material("satin_graphite", rgba=(0.035, 0.038, 0.040, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.010, 0.011, 0.012, 1.0))
    soft_black = model.material("soft_black", rgba=(0.0, 0.0, 0.0, 1.0))
    smoked = model.material("smoked_lid", rgba=(0.055, 0.060, 0.066, 1.0))
    glass = model.material("scanner_glass", rgba=(0.015, 0.025, 0.030, 1.0))
    amber_film = model.material("amber_film", rgba=(0.58, 0.31, 0.055, 0.72))
    metal = model.material("brushed_steel", rgba=(0.62, 0.63, 0.60, 1.0))

    body = model.part("body")

    body_shell = (
        cq.Workplane("XY")
        .box(BODY_L, BODY_W, BODY_H)
        .edges("|Z")
        .fillet(0.010)
    )
    tray_channel = (
        cq.Workplane("XY")
        .box(0.390, 0.104, 0.020)
        .translate((FRONT_X + 0.165, 0.0, TRAY_SLOT_Z))
    )
    body_shell = body_shell.cut(tray_channel)
    body.visual(
        mesh_from_cadquery(body_shell, "body_shell"),
        material=graphite,
        name="body_shell",
    )

    # Black four-sided front trim makes the real tray slot readable without
    # blocking the hollow guide channel cut into the casing.
    body.visual(
        Box((0.003, 0.113, 0.004)),
        origin=Origin(xyz=(FRONT_X - 0.0015, 0.0, TRAY_SLOT_Z + 0.012)),
        material=soft_black,
        name="slot_upper_lip",
    )
    body.visual(
        Box((0.003, 0.113, 0.004)),
        origin=Origin(xyz=(FRONT_X - 0.0015, 0.0, TRAY_SLOT_Z - 0.012)),
        material=soft_black,
        name="slot_lower_lip",
    )
    for y, name in ((-0.0565, "slot_side_0"), (0.0565, "slot_side_1")):
        body.visual(
            Box((0.003, 0.004, 0.024)),
            origin=Origin(xyz=(FRONT_X - 0.0015, y, TRAY_SLOT_Z)),
            material=soft_black,
            name=name,
        )

    body.visual(
        Box((0.255, 0.054, 0.0022)),
        origin=Origin(xyz=(-0.018, 0.0, TOP_Z + 0.0011)),
        material=glass,
        name="scan_window",
    )
    body.visual(
        Box((0.285, 0.072, 0.0030)),
        origin=Origin(xyz=(-0.018, 0.0, TOP_Z + 0.0005)),
        material=soft_black,
        name="window_recess",
    )

    hinge_centers = (-0.043, 0.043)
    for hinge_index, hinge_y in enumerate(hinge_centers):
        body.visual(
            Box((0.034, 0.038, 0.0030)),
            origin=Origin(xyz=(REAR_X - 0.010, hinge_y, TOP_Z + 0.0015)),
            material=metal,
            name=f"hinge_leaf_{hinge_index}",
        )
        for segment_index, segment_y in enumerate((hinge_y - 0.011, hinge_y + 0.011)):
            body.visual(
                Box((0.012, 0.009, 0.006)),
                origin=Origin(xyz=(REAR_X + 0.002, segment_y, TOP_Z + 0.0055)),
                material=metal,
                name=f"hinge_knuckle_support_{hinge_index}_{segment_index}",
            )
            body.visual(
                Cylinder(radius=0.006, length=0.0085),
                origin=Origin(
                    xyz=(HINGE_X, segment_y, HINGE_Z),
                    rpy=(pi / 2.0, 0.0, 0.0),
                ),
                material=metal,
                name=f"fixed_barrel_{hinge_index}_{segment_index}",
            )

    lid = model.part("lid")
    lid_panel = (
        cq.Workplane("XY")
        .box(0.395, 0.124, 0.014)
        .edges("|Z")
        .fillet(0.006)
    )
    lid.visual(
        mesh_from_cadquery(lid_panel, "lid_panel"),
        origin=Origin(xyz=(-0.2075, 0.0, 0.0)),
        material=smoked,
        name="lid_panel",
    )
    lid.visual(
        Box((0.250, 0.046, 0.002)),
        origin=Origin(xyz=(-0.188, 0.0, 0.0080)),
        material=charcoal,
        name="pressure_pad",
    )
    lid.visual(
        Box((0.060, 0.075, 0.006)),
        origin=Origin(xyz=(-0.380, 0.0, -0.002)),
        material=charcoal,
        name="front_grip",
    )
    for hinge_index, hinge_y in enumerate(hinge_centers):
        lid.visual(
            Box((0.032, 0.026, 0.003)),
            origin=Origin(xyz=(-0.016, hinge_y, -0.005)),
            material=metal,
            name=f"moving_hinge_leaf_{hinge_index}",
        )
        lid.visual(
            Cylinder(radius=0.0055, length=0.0095),
            origin=Origin(xyz=(0.0, hinge_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"moving_barrel_{hinge_index}",
        )

    tray = model.part("film_tray")
    tray_solids = [
        ((0.255, 0.014, 0.006), (0.105, -0.039, TRAY_SLOT_Z)),
        ((0.255, 0.014, 0.006), (0.105, 0.039, TRAY_SLOT_Z)),
        ((0.025, 0.100, 0.006), (-0.020, 0.0, TRAY_SLOT_Z)),
        ((0.024, 0.100, 0.006), (0.230, 0.0, TRAY_SLOT_Z)),
        ((0.006, 0.100, 0.006), (0.035, 0.0, TRAY_SLOT_Z)),
        ((0.006, 0.100, 0.006), (0.083, 0.0, TRAY_SLOT_Z)),
        ((0.006, 0.100, 0.006), (0.131, 0.0, TRAY_SLOT_Z)),
        ((0.006, 0.100, 0.006), (0.179, 0.0, TRAY_SLOT_Z)),
        ((0.255, 0.010, 0.006), (0.105, -0.045, TRAY_SLOT_Z - 0.0035)),
        ((0.255, 0.010, 0.006), (0.105, 0.045, TRAY_SLOT_Z - 0.0035)),
        ((0.018, 0.084, 0.006), (-0.040, 0.0, TRAY_SLOT_Z)),
        ((0.025, 0.108, 0.018), (-0.060, 0.0, TRAY_SLOT_Z)),
    ]
    tray_frame = cq.Workplane("XY").box(*tray_solids[0][0]).translate(tray_solids[0][1])
    for size, center in tray_solids[1:]:
        tray_frame = tray_frame.union(cq.Workplane("XY").box(*size).translate(center))
    tray.visual(
        mesh_from_cadquery(tray_frame, "tray_frame"),
        material=charcoal,
        name="tray_frame",
    )
    tray.visual(
        Box((0.016, 0.122, 0.028)),
        origin=Origin(xyz=(-0.004, 0.0, TRAY_SLOT_Z)),
        material=charcoal,
        name="front_stop",
    )
    tray.visual(
        Box((0.205, 0.031, 0.0012)),
        origin=Origin(xyz=(0.116, 0.0, TRAY_SLOT_Z + 0.0030)),
        material=amber_film,
        name="film_strip",
    )
    for index, x in enumerate((0.011, 0.059, 0.107, 0.155, 0.203)):
        tray.visual(
            Box((0.020, 0.026, 0.0014)),
            origin=Origin(xyz=(x, 0.0, TRAY_SLOT_Z + 0.0040)),
            material=glass,
            name=f"film_frame_{index}",
        )
        tray.visual(
            Box((0.004, 0.010, 0.0020)),
            origin=Origin(xyz=(x, 0.043, TRAY_SLOT_Z + 0.0025)),
            material=metal,
            name=f"index_tick_{index}",
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
        origin=Origin(xyz=(FRONT_X - 0.004, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=0.130),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("film_tray")
    lid_joint = object_model.get_articulation("body_to_lid")
    tray_joint = object_model.get_articulation("body_to_film_tray")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="body_shell",
        min_gap=0.002,
        max_gap=0.008,
        name="closed lid sits just above scanner body",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="scan_window",
        min_overlap=0.045,
        name="lid covers the scanning window footprint",
    )
    ctx.expect_within(
        tray,
        body,
        axes="yz",
        inner_elem="tray_frame",
        outer_elem="body_shell",
        margin=0.002,
        name="film tray fits inside the front guide slot",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="x",
        elem_a="tray_frame",
        elem_b="body_shell",
        min_overlap=0.090,
        name="closed tray remains deeply inserted",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_joint: 1.10}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid opens upward on rear barrel hinges",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: 0.130}):
        extended_tray_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            elem_a="tray_frame",
            elem_b="body_shell",
            min_overlap=0.045,
            name="extended tray keeps retained insertion in scanner",
        )
    ctx.check(
        "tray slides outward from the front face",
        closed_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] < closed_tray_pos[0] - 0.12,
        details=f"closed={closed_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
