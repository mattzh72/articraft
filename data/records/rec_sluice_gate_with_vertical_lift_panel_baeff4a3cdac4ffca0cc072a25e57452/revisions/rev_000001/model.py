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


OUTER_WIDTH = 3.40
FRAME_HEIGHT = 4.20
FRAME_THICKNESS = 0.48
JAMB_WIDTH = 0.50
SILL_HEIGHT = 0.40
TOP_BEAM_HEIGHT = 0.75

OPENING_WIDTH = OUTER_WIDTH - 2.0 * JAMB_WIDTH
OPENING_HEIGHT = FRAME_HEIGHT - SILL_HEIGHT - TOP_BEAM_HEIGHT

PANEL_WIDTH = 2.24
PANEL_HEIGHT = 3.00
PANEL_THICKNESS = 0.16
PANEL_REST_BOTTOM = SILL_HEIGHT
PANEL_REST_Y = -0.06
PANEL_TRAVEL = 2.30


def _build_handwheel_mesh() -> object:
    rim = (
        cq.Workplane("XZ")
        .circle(0.31)
        .circle(0.245)
        .extrude(0.015, both=True)
    )
    hub = cq.Workplane("XZ").circle(0.08).extrude(0.018, both=True)
    spoke_x = cq.Workplane("XZ").rect(0.53, 0.045).extrude(0.012, both=True)
    spoke_z = cq.Workplane("XZ").rect(0.045, 0.53).extrude(0.012, both=True)
    return rim.union(hub).union(spoke_x).union(spoke_z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sluice_gate")

    steel = model.material("steel", rgba=(0.28, 0.31, 0.33, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.42, 0.34, 0.26, 1.0))
    machinery = model.material("machinery", rgba=(0.54, 0.56, 0.58, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((OUTER_WIDTH, FRAME_THICKNESS, SILL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, SILL_HEIGHT * 0.5)),
        material=steel,
        name="sill",
    )
    frame.visual(
        Box((OUTER_WIDTH, FRAME_THICKNESS, TOP_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT - TOP_BEAM_HEIGHT * 0.5)),
        material=steel,
        name="top_beam",
    )
    frame.visual(
        Box((JAMB_WIDTH, FRAME_THICKNESS, FRAME_HEIGHT)),
        origin=Origin(xyz=(-(OUTER_WIDTH - JAMB_WIDTH) * 0.5, 0.0, FRAME_HEIGHT * 0.5)),
        material=steel,
        name="left_guide",
    )
    frame.visual(
        Box((JAMB_WIDTH, FRAME_THICKNESS, FRAME_HEIGHT)),
        origin=Origin(xyz=((OUTER_WIDTH - JAMB_WIDTH) * 0.5, 0.0, FRAME_HEIGHT * 0.5)),
        material=steel,
        name="right_guide",
    )
    frame.visual(
        Box((0.12, 0.16, OPENING_HEIGHT + 0.18)),
        origin=Origin(
            xyz=(
                -OPENING_WIDTH * 0.5 - 0.06,
                FRAME_THICKNESS * 0.5 - 0.08,
                SILL_HEIGHT + OPENING_HEIGHT * 0.5 + 0.04,
            )
        ),
        material=machinery,
        name="left_channel",
    )
    frame.visual(
        Box((0.12, 0.16, OPENING_HEIGHT + 0.18)),
        origin=Origin(
            xyz=(
                OPENING_WIDTH * 0.5 + 0.06,
                FRAME_THICKNESS * 0.5 - 0.08,
                SILL_HEIGHT + OPENING_HEIGHT * 0.5 + 0.04,
            )
        ),
        material=machinery,
        name="right_channel",
    )

    lift_panel = model.part("lift_panel")
    lift_panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(xyz=(0.0, PANEL_REST_Y, PANEL_HEIGHT * 0.5)),
        material=weathered_steel,
        name="panel_plate",
    )
    for rib_index, local_z in enumerate((0.42, 0.98, 1.54, 2.10, 2.66)):
        lift_panel.visual(
            Box((PANEL_WIDTH - 0.10, PANEL_THICKNESS + 0.04, 0.10)),
            origin=Origin(xyz=(0.0, PANEL_REST_Y + 0.01, local_z)),
            material=steel,
            name=f"rib_{rib_index}",
        )
    for stiffener_index, local_x in enumerate((-0.72, 0.0, 0.72)):
        lift_panel.visual(
            Box((0.14, PANEL_THICKNESS + 0.05, PANEL_HEIGHT - 0.24)),
            origin=Origin(xyz=(local_x, PANEL_REST_Y + 0.01, (PANEL_HEIGHT - 0.24) * 0.5 + 0.12)),
            material=steel,
            name=f"stiffener_{stiffener_index}",
        )
    lift_panel.visual(
        Box((0.60, PANEL_THICKNESS + 0.06, 0.20)),
        origin=Origin(xyz=(0.0, PANEL_REST_Y + 0.01, PANEL_HEIGHT - 0.10)),
        material=steel,
        name="head_stiffener",
    )

    model.articulation(
        "frame_to_lift_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=lift_panel,
        origin=Origin(xyz=(0.0, 0.0, PANEL_REST_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25000.0,
            velocity=0.15,
            lower=0.0,
            upper=PANEL_TRAVEL,
        ),
    )

    operator_box = model.part("operator_box")
    operator_box.visual(
        Box((0.98, 0.24, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=machinery,
        name="mount_base",
    )
    operator_box.visual(
        Box((0.18, 0.18, 0.34)),
        origin=Origin(xyz=(-0.26, 0.02, 0.27)),
        material=machinery,
        name="left_post",
    )
    operator_box.visual(
        Box((0.18, 0.18, 0.34)),
        origin=Origin(xyz=(0.26, 0.02, 0.27)),
        material=machinery,
        name="right_post",
    )
    operator_box.visual(
        Box((0.82, 0.44, 0.58)),
        origin=Origin(xyz=(0.0, 0.08, 0.73)),
        material=machinery,
        name="housing_body",
    )
    operator_box.visual(
        Box((0.92, 0.50, 0.06)),
        origin=Origin(xyz=(0.0, 0.08, 1.05)),
        material=steel,
        name="roof_cap",
    )
    operator_box.visual(
        Cylinder(radius=0.11, length=0.12),
        origin=Origin(xyz=(0.0, 0.36, 0.74), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="front_boss",
    )
    operator_box.visual(
        Box((0.12, 0.12, 0.16)),
        origin=Origin(xyz=(0.43, 0.30, 0.82)),
        material=steel,
        name="pawl_mount",
    )

    model.articulation(
        "frame_to_operator_box",
        ArticulationType.FIXED,
        parent=frame,
        child=operator_box,
        origin=Origin(xyz=(0.0, 0.18, FRAME_HEIGHT)),
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_build_handwheel_mesh(), "sluice_gate_handwheel"),
        material=machinery,
        name="wheel_rim",
    )
    handwheel.visual(
        Cylinder(radius=0.03, length=0.10),
        origin=Origin(xyz=(0.0, -0.05, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="shaft",
    )

    model.articulation(
        "operator_box_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=operator_box,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.46, 0.74)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=2.5,
        ),
    )

    locking_pawl = model.part("locking_pawl")
    locking_pawl.visual(
        Cylinder(radius=0.028, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="pivot_hub",
    )
    locking_pawl.visual(
        Box((0.06, 0.05, 0.22)),
        origin=Origin(xyz=(-0.015, 0.0, -0.11)),
        material=steel,
        name="pawl_arm",
    )
    locking_pawl.visual(
        Box((0.08, 0.05, 0.06)),
        origin=Origin(xyz=(-0.055, 0.0, -0.22)),
        material=weathered_steel,
        name="pawl_tooth",
    )

    model.articulation(
        "operator_box_to_locking_pawl",
        ArticulationType.REVOLUTE,
        parent=operator_box,
        child=locking_pawl,
        origin=Origin(xyz=(0.43, 0.40, 0.82)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-0.9,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    lift_panel = object_model.get_part("lift_panel")
    operator_box = object_model.get_part("operator_box")
    handwheel = object_model.get_part("handwheel")
    locking_pawl = object_model.get_part("locking_pawl")
    slide = object_model.get_articulation("frame_to_lift_panel")
    pawl_joint = object_model.get_articulation("operator_box_to_locking_pawl")

    ctx.allow_overlap(
        operator_box,
        handwheel,
        elem_a="front_boss",
        elem_b="shaft",
        reason="The handwheel shaft intentionally runs inside the front bearing boss of the operator housing.",
    )
    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            lift_panel,
            frame,
            axis="x",
            positive_elem="panel_plate",
            negative_elem="left_guide",
            min_gap=0.03,
            max_gap=0.16,
            name="panel clears left guide at rest",
        )
        ctx.expect_gap(
            frame,
            lift_panel,
            axis="x",
            positive_elem="right_guide",
            negative_elem="panel_plate",
            min_gap=0.03,
            max_gap=0.16,
            name="panel clears right guide at rest",
        )
        ctx.expect_gap(
            lift_panel,
            frame,
            axis="z",
            positive_elem="panel_plate",
            negative_elem="sill",
            max_gap=0.03,
            max_penetration=0.001,
            name="panel seats just above sill",
        )
        ctx.expect_within(
            lift_panel,
            frame,
            axes="y",
            inner_elem="panel_plate",
            outer_elem="left_guide",
            margin=0.40,
            name="panel stays within frame depth",
        )
        ctx.expect_gap(
            operator_box,
            frame,
            axis="z",
            positive_elem="mount_base",
            negative_elem="top_beam",
            max_gap=0.001,
            max_penetration=0.001,
            name="operator box is seated on the top beam",
        )
        ctx.expect_gap(
            handwheel,
            operator_box,
            axis="y",
            positive_elem="wheel_rim",
            negative_elem="housing_body",
            min_gap=0.02,
            max_gap=0.15,
            name="handwheel stands proud of the housing front",
        )
        ctx.expect_origin_distance(
            locking_pawl,
            handwheel,
            axes="x",
            min_dist=0.20,
            max_dist=0.45,
            name="pawl sits beside the handwheel",
        )
        ctx.expect_overlap(
            locking_pawl,
            handwheel,
            axes="z",
            min_overlap=0.12,
            name="pawl shares the handwheel working height",
        )

    closed_pos = ctx.part_world_position(lift_panel)
    with ctx.pose({slide: PANEL_TRAVEL}):
        ctx.expect_overlap(
            lift_panel,
            frame,
            axes="x",
            elem_a="panel_plate",
            elem_b="top_beam",
            min_overlap=0.50,
            name="raised panel remains laterally aligned with frame",
        )
        extended_pos = ctx.part_world_position(lift_panel)

    with ctx.pose({pawl_joint: pawl_joint.motion_limits.lower}):
        engaged_tooth = ctx.part_element_world_aabb(locking_pawl, elem="pawl_tooth")
    with ctx.pose({pawl_joint: pawl_joint.motion_limits.upper}):
        released_tooth = ctx.part_element_world_aabb(locking_pawl, elem="pawl_tooth")

    ctx.check(
        "panel raises upward",
        closed_pos is not None and extended_pos is not None and extended_pos[2] > closed_pos[2] + 2.0,
        details=f"closed={closed_pos}, extended={extended_pos}",
    )
    ctx.check(
        "pawl swings beside wheel",
        engaged_tooth is not None
        and released_tooth is not None
        and abs(
            ((released_tooth[0][0] + released_tooth[1][0]) * 0.5)
            - ((engaged_tooth[0][0] + engaged_tooth[1][0]) * 0.5)
        )
        > 0.04,
        details=f"engaged={engaged_tooth}, released={released_tooth}",
    )

    return ctx.report()


object_model = build_object_model()
