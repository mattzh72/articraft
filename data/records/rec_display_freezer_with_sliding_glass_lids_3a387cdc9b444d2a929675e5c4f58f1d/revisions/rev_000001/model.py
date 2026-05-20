from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_L = 2.10
BODY_W = 0.95
BODY_H = 0.85
WALL_T = 0.12
BOTTOM_T = 0.17

LID_L = 0.60
LID_W = 0.78
LID_H = 0.030
LID_BAR = 0.045
LID_X = (-0.64, 0.0, 0.64)
LID_ZS = (0.885, 0.935, 0.885)

CONTROL_Y = BODY_W / 2.0 - 0.17
CONTROL_Z = 0.43
END_X = -BODY_L / 2.0


def _freezer_body_shell() -> cq.Workplane:
    """Deep, open insulated chest with a real top cavity."""
    outer = cq.Workplane("XY").box(BODY_L, BODY_W, BODY_H).translate((0.0, 0.0, BODY_H / 2.0))
    cavity = (
        cq.Workplane("XY")
        .box(BODY_L - 2.0 * WALL_T, BODY_W - 2.0 * WALL_T, BODY_H)
        .translate((0.0, 0.0, BOTTOM_T + BODY_H / 2.0))
    )
    shell = outer.cut(cavity)
    return shell


def _lid_frame_mesh(length: float, width: float, bar: float, height: float) -> cq.Workplane:
    """One connected rectangular aluminium frame around the glass pane."""
    outer = cq.Workplane("XY").box(length, width, height)
    opening = cq.Workplane("XY").box(length - 2.0 * bar, width - 2.0 * bar, height * 1.6)
    return outer.cut(opening)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_island_freezer")

    insulation = Material("warm_white_insulation", rgba=(0.90, 0.88, 0.78, 1.0))
    liner = Material("cool_white_liner", rgba=(0.84, 0.88, 0.88, 1.0))
    rail_mat = Material("black_top_rails", rgba=(0.015, 0.017, 0.018, 1.0))
    glass_mat = Material("blue_tinted_glass", rgba=(0.45, 0.78, 0.95, 0.34))
    frame_mat = Material("brushed_aluminium", rgba=(0.70, 0.73, 0.74, 1.0))
    dark_mat = Material("recessed_black_panel", rgba=(0.02, 0.025, 0.03, 1.0))
    label_mat = Material("white_control_marks", rgba=(0.93, 0.95, 0.90, 1.0))
    knob_mat = Material("charcoal_knob", rgba=(0.03, 0.033, 0.035, 1.0))

    body = model.part("body")
    # Five overlapping insulated slabs form a visibly hollow, open-top cabinet.
    body.visual(
        Box((BODY_L, BODY_W, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_T / 2.0)),
        material=insulation,
        name="bottom_insulation",
    )
    body.visual(
        Box((BODY_L, WALL_T, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_W / 2.0 + WALL_T / 2.0, BODY_H / 2.0)),
        material=insulation,
        name="front_wall",
    )
    body.visual(
        Box((BODY_L, WALL_T, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_W / 2.0 - WALL_T / 2.0, BODY_H / 2.0)),
        material=insulation,
        name="rear_wall",
    )
    body.visual(
        Box((WALL_T, BODY_W - 2.0 * WALL_T + 0.020, BODY_H)),
        origin=Origin(xyz=(-BODY_L / 2.0 + WALL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=insulation,
        name="end_wall_0",
    )
    body.visual(
        Box((WALL_T, BODY_W - 2.0 * WALL_T + 0.020, BODY_H)),
        origin=Origin(xyz=(BODY_L / 2.0 - WALL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=insulation,
        name="end_wall_1",
    )
    body.visual(
        Box((BODY_L - 2.0 * WALL_T - 0.02, BODY_W - 2.0 * WALL_T - 0.02, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_T + 0.006)),
        material=liner,
        name="inner_liner_floor",
    )

    # Black rails and stops sit on the thick top rim and define three sliding lanes.
    rail_z = BODY_H + 0.010
    body.visual(
        Box((BODY_L - 0.10, 0.035, 0.022)),
        origin=Origin(xyz=(0.0, -0.410, rail_z - 0.001)),
        material=rail_mat,
        name="front_rail",
    )
    body.visual(
        Box((BODY_L - 0.10, 0.030, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, rail_z - 0.001)),
        material=rail_mat,
        name="center_rail",
    )
    body.visual(
        Box((BODY_L - 0.10, 0.035, 0.022)),
        origin=Origin(xyz=(0.0, 0.410, rail_z - 0.001)),
        material=rail_mat,
        name="rear_rail",
    )
    body.visual(
        Box((BODY_L - 0.10, 0.020, 0.070)),
        origin=Origin(xyz=(0.0, -0.400, BODY_H + 0.035)),
        material=rail_mat,
        name="upper_front_rail",
    )
    body.visual(
        Box((BODY_L - 0.10, 0.020, 0.070)),
        origin=Origin(xyz=(0.0, 0.400, BODY_H + 0.035)),
        material=rail_mat,
        name="upper_rear_rail",
    )
    for name, x in (("end_stop_0", -BODY_L / 2.0 + 0.035), ("end_stop_1", BODY_L / 2.0 - 0.035)):
        body.visual(
            Box((0.035, BODY_W - 0.10, 0.026)),
            origin=Origin(xyz=(x, 0.0, BODY_H + 0.011)),
            material=rail_mat,
            name=name,
        )

    # Recessed end-corner control pocket: dark panel sits behind a proud body-colored rim.
    panel_x = END_X - 0.003
    body.visual(
        Box((0.008, 0.29, 0.18)),
        origin=Origin(xyz=(panel_x, CONTROL_Y, CONTROL_Z)),
        material=dark_mat,
        name="control_panel",
    )
    rim_x = END_X - 0.005
    body.visual(
        Box((0.014, 0.335, 0.025)),
        origin=Origin(xyz=(rim_x, CONTROL_Y, CONTROL_Z + 0.102)),
        material=insulation,
        name="control_top_lip",
    )
    body.visual(
        Box((0.014, 0.335, 0.025)),
        origin=Origin(xyz=(rim_x, CONTROL_Y, CONTROL_Z - 0.102)),
        material=insulation,
        name="control_bottom_lip",
    )
    body.visual(
        Box((0.014, 0.025, 0.205)),
        origin=Origin(xyz=(rim_x, CONTROL_Y - 0.167, CONTROL_Z)),
        material=insulation,
        name="control_side_lip_0",
    )
    body.visual(
        Box((0.014, 0.025, 0.205)),
        origin=Origin(xyz=(rim_x, CONTROL_Y + 0.167, CONTROL_Z)),
        material=insulation,
        name="control_side_lip_1",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(END_X - 0.007, CONTROL_Y, CONTROL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_mat,
        name="knob_bushing",
    )
    # Simple temperature tick marks bonded to the recessed panel around the knob.
    for i, (dy, dz, sy, sz) in enumerate(
        (
            (-0.070, 0.000, 0.030, 0.006),
            (-0.048, 0.048, 0.022, 0.006),
            (0.000, 0.068, 0.006, 0.026),
            (0.048, 0.048, 0.022, 0.006),
            (0.070, 0.000, 0.030, 0.006),
        )
    ):
        body.visual(
            Box((0.004, sy, sz)),
            origin=Origin(xyz=(END_X - 0.008, CONTROL_Y + dy, CONTROL_Z + dz)),
            material=label_mat,
            name=f"temperature_tick_{i}",
        )

    frame_mesh = mesh_from_cadquery(
        _lid_frame_mesh(LID_L, LID_W, LID_BAR, LID_H),
        "sliding_lid_frame",
        tolerance=0.0015,
    )
    for i, x in enumerate(LID_X):
        lid = model.part(f"lid_{i}")
        lid.visual(frame_mesh, material=frame_mat, name="aluminium_frame")
        lid.visual(
            Box((LID_L - 2.0 * LID_BAR + 0.018, LID_W - 2.0 * LID_BAR + 0.018, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=glass_mat,
            name="glass_pane",
        )
        lid.visual(
            Box((0.16, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, -LID_W / 2.0 + 0.030, LID_H / 2.0 + 0.006)),
            material=rail_mat,
            name="finger_pull",
        )
        model.articulation(
            f"body_to_lid_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=lid,
            origin=Origin(xyz=(x, 0.0, LID_ZS[i])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=55.0, velocity=0.35, lower=-0.18, upper=0.18),
        )

    knob = model.part("temperature_knob")
    knob.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=frame_mat,
        name="shaft",
    )
    knob_shape = KnobGeometry(
        0.074,
        0.035,
        body_style="skirted",
        top_diameter=0.060,
        skirt=KnobSkirt(0.082, 0.006, flare=0.05, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=22, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_shape, "temperature_knob_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=knob_mat,
        name="cap",
    )
    model.articulation(
        "body_to_temperature_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(END_X - 0.002, CONTROL_Y, CONTROL_Z), rpy=(0.0, -math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    body = object_model.get_part("body")
    lids = [object_model.get_part(f"lid_{i}") for i in range(3)]
    knob = object_model.get_part("temperature_knob")
    lid_joints = [object_model.get_articulation(f"body_to_lid_{i}") for i in range(3)]
    knob_joint = object_model.get_articulation("body_to_temperature_knob")

    ctx.check(
        "three independent prismatic glass lids",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in lid_joints),
        details="Each framed glass lid should have its own prismatic slide joint.",
    )
    ctx.check(
        "temperature knob rotates continuously",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="The knob should be a separate continuously rotating control.",
    )

    ctx.allow_overlap(
        body,
        knob,
        elem_a="knob_bushing",
        elem_b="shaft",
        reason="The knob shaft is intentionally captured in the fixed metal bushing on the recessed end panel.",
    )
    ctx.expect_within(
        knob,
        body,
        axes="yz",
        inner_elem="shaft",
        outer_elem="knob_bushing",
        margin=0.001,
        name="knob shaft is centered in the bushing",
    )
    ctx.expect_overlap(
        knob,
        body,
        axes="x",
        elem_a="shaft",
        elem_b="knob_bushing",
        min_overlap=0.008,
        name="knob shaft remains inserted through the bushing",
    )

    for i, lid in enumerate(lids):
        rail_name = "upper_front_rail" if i == 1 else "front_rail"
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="aluminium_frame",
            negative_elem=rail_name,
            min_gap=0.0,
            max_gap=0.003,
            name=f"lid_{i} rides on the rails",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.45,
            name=f"lid_{i} covers the freezer opening",
        )

    ctx.expect_gap(
        lids[1],
        lids[0],
        axis="x",
        min_gap=0.008,
        max_gap=0.060,
        name="first two lid frames remain distinct",
    )
    ctx.expect_gap(
        lids[2],
        lids[1],
        axis="x",
        min_gap=0.008,
        max_gap=0.060,
        name="last two lid frames remain distinct",
    )

    rest_positions = [ctx.part_world_position(lid) for lid in lids]
    with ctx.pose({lid_joints[0]: 0.16, lid_joints[1]: -0.16, lid_joints[2]: 0.16}):
        moved_positions = [ctx.part_world_position(lid) for lid in lids]
    ctx.check(
        "lid prismatic joints translate along the long rails",
        all(
            rest is not None and moved is not None and abs(moved[0] - rest[0]) > 0.12
            for rest, moved in zip(rest_positions, moved_positions)
        ),
        details=f"rest={rest_positions}, moved={moved_positions}",
    )

    return ctx.report()


object_model = build_object_model()
