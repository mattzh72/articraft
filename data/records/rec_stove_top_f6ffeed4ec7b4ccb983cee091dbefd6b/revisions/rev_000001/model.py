from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_WIDTH = 0.54
BODY_DEPTH = 0.32
BODY_HEIGHT = 0.086
WALL_THICKNESS = 0.012
DECK_THICKNESS = 0.004
DECK_TOP_Z = 0.084
HINGE_Y = BODY_DEPTH * 0.5 - WALL_THICKNESS
HINGE_Z = DECK_TOP_Z + 0.006


def _add_burner(model: ArticulatedObject, body, index: int, x_pos: float) -> None:
    iron = "grate_iron"
    steel = "burner_steel"

    burner = model.part(f"burner_{index}")
    burner.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="tray",
    )
    burner.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=iron,
        name="cap",
    )
    burner.visual(
        Box((0.112, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=iron,
        name="grate_bar_x",
    )
    burner.visual(
        Box((0.018, 0.112, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=iron,
        name="grate_bar_y",
    )

    model.articulation(
        f"body_to_burner_{index}",
        ArticulationType.FIXED,
        parent=body,
        child=burner,
        origin=Origin(xyz=(x_pos, 0.014, DECK_TOP_Z)),
    )


def _add_control_knob(model: ArticulatedObject, body, index: int, x_pos: float) -> None:
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.038,
            0.024,
            body_style="skirted",
            top_diameter=0.031,
            skirt=KnobSkirt(0.046, 0.004, flare=0.08),
            grip=KnobGrip(style="fluted", count=14, depth=0.0012),
            indicator=KnobIndicator(
                style="line",
                mode="engraved",
                depth=0.0008,
                angle_deg=0.0,
            ),
            center=False,
        ),
        f"control_knob_{index}",
    )

    knob = model.part(f"knob_{index}")
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material="knob_black",
        name="knob_shell",
    )
    knob.visual(
        Cylinder(radius=0.007, length=0.003),
        origin=Origin(xyz=(0.0, -0.0015, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material="bezel_dark",
        name="bezel",
    )

    model.articulation(
        f"body_to_knob_{index}",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(x_pos, -BODY_DEPTH * 0.5, 0.038)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camping_stove_top")

    model.material("body_green", rgba=(0.18, 0.27, 0.18, 1.0))
    model.material("lid_green", rgba=(0.20, 0.30, 0.20, 1.0))
    model.material("grate_iron", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("burner_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("bezel_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    model.material("handle_metal", rgba=(0.70, 0.72, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="body_green",
        name="base_pan",
    )
    body.visual(
        Box((BODY_WIDTH, WALL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 + WALL_THICKNESS * 0.5, BODY_HEIGHT * 0.5)),
        material="body_green",
        name="front_panel",
    )
    body.visual(
        Box((BODY_WIDTH, WALL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 - WALL_THICKNESS * 0.5, BODY_HEIGHT * 0.5)),
        material="body_green",
        name="rear_panel",
    )
    body.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-BODY_WIDTH * 0.5 + WALL_THICKNESS * 0.5, 0.0, BODY_HEIGHT * 0.5)),
        material="body_green",
        name="side_panel_0",
    )
    body.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(BODY_WIDTH * 0.5 - WALL_THICKNESS * 0.5, 0.0, BODY_HEIGHT * 0.5)),
        material="body_green",
        name="side_panel_1",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * WALL_THICKNESS, BODY_DEPTH - 2.0 * WALL_THICKNESS, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP_Z - DECK_THICKNESS * 0.5)),
        material="body_green",
        name="deck",
    )
    body.visual(
        Box((BODY_WIDTH - 0.060, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, HINGE_Y - 0.010, HINGE_Z - 0.005)),
        material="body_green",
        name="hinge_rail",
    )
    body.visual(
        Box((0.090, 0.010, 0.010)),
        origin=Origin(xyz=(-0.046, -BODY_DEPTH * 0.5 + 0.011, 0.040)),
        material="bezel_dark",
        name="knob_plinth_0",
    )
    body.visual(
        Box((0.090, 0.010, 0.010)),
        origin=Origin(xyz=(0.046, -BODY_DEPTH * 0.5 + 0.011, 0.040)),
        material="bezel_dark",
        name="knob_plinth_1",
    )
    body.visual(
        Box((0.032, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 + 0.011, 0.040)),
        material="handle_metal",
        name="front_latch",
    )

    _add_burner(model, body, 0, -0.118)
    _add_burner(model, body, 1, 0.118)
    _add_control_knob(model, body, 0, -0.046)
    _add_control_knob(model, body, 1, 0.046)

    lid = model.part("lid")
    lid.visual(
        Box((BODY_WIDTH - 0.018, 0.012, 0.290)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material="lid_green",
        name="panel",
    )
    lid.visual(
        Box((0.012, 0.060, 0.290)),
        origin=Origin(xyz=(-BODY_WIDTH * 0.5 + 0.021, 0.024, 0.145)),
        material="lid_green",
        name="wind_flange_0",
    )
    lid.visual(
        Box((0.012, 0.060, 0.290)),
        origin=Origin(xyz=(BODY_WIDTH * 0.5 - 0.021, 0.024, 0.145)),
        material="lid_green",
        name="wind_flange_1",
    )
    lid.visual(
        Box((BODY_WIDTH - 0.050, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.006, 0.007)),
        material="lid_green",
        name="lower_hem",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-1.38,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")

    lid_hinge = object_model.get_articulation("body_to_lid")
    knob_joint_0 = object_model.get_articulation("body_to_knob_0")
    knob_joint_1 = object_model.get_articulation("body_to_knob_1")

    ctx.expect_overlap(
        knob_0,
        knob_1,
        axes="z",
        min_overlap=0.015,
        name="control knobs share a common control height band",
    )
    ctx.expect_origin_distance(
        knob_0,
        knob_1,
        axes="x",
        min_dist=0.07,
        max_dist=0.11,
        name="control knobs sit side by side across the front",
    )
    ctx.expect_gap(
        body,
        knob_0,
        axis="y",
        positive_elem="front_panel",
        negative_elem="knob_shell",
        min_gap=0.0,
        max_gap=0.004,
        name="left knob seats against the front panel",
    )
    ctx.expect_gap(
        body,
        knob_1,
        axis="y",
        positive_elem="front_panel",
        negative_elem="knob_shell",
        min_gap=0.0,
        max_gap=0.004,
        name="right knob seats against the front panel",
    )

    ctx.check(
        "control joints are continuous front-shaft rotations",
        (
            knob_joint_0.articulation_type == ArticulationType.CONTINUOUS
            and knob_joint_1.articulation_type == ArticulationType.CONTINUOUS
            and knob_joint_0.axis == (0.0, -1.0, 0.0)
            and knob_joint_1.axis == (0.0, -1.0, 0.0)
            and knob_joint_0.motion_limits is not None
            and knob_joint_1.motion_limits is not None
            and knob_joint_0.motion_limits.lower is None
            and knob_joint_0.motion_limits.upper is None
            and knob_joint_1.motion_limits.lower is None
            and knob_joint_1.motion_limits.upper is None
        ),
        details=(
            f"joint_0={knob_joint_0.articulation_type!r}/{knob_joint_0.axis!r}, "
            f"joint_1={knob_joint_1.articulation_type!r}/{knob_joint_1.axis!r}"
        ),
    )

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.lower}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.24,
            elem_a="panel",
            elem_b="deck",
            name="closed lid covers the cook surface",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="panel",
            negative_elem="deck",
            min_gap=0.0,
            max_gap=0.020,
            name="closed lid rests just above the deck",
        )

    open_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.lower}):
        closed_lid_aabb = ctx.part_world_aabb(lid)

    lid_rises = False
    if open_lid_aabb is not None and closed_lid_aabb is not None:
        (_, open_maxs) = open_lid_aabb
        (_, closed_maxs) = closed_lid_aabb
        lid_rises = open_maxs[2] > closed_maxs[2] + 0.12

    ctx.check(
        "lid opens upward into a wind guard pose",
        lid_rises,
        details=f"open_aabb={open_lid_aabb}, closed_aabb={closed_lid_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
