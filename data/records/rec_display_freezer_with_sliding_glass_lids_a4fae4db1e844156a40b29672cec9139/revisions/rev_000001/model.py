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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CABINET_L = 1.36
CABINET_W = 0.86
PLINTH_H = 0.08
BODY_H = 0.74
TOP_Z = PLINTH_H + BODY_H

INNER_L = 1.18
INNER_W = 0.66
CAVITY_BOTTOM_Z = 0.18
CAVITY_H = TOP_Z - CAVITY_BOTTOM_Z

TRACK_L = 1.16
TRACK_Y = (INNER_W * 0.5) + 0.02
LOWER_TRACK_H = 0.008
UPPER_TRACK_H = 0.008
LOWER_TRACK_Z = TOP_Z + (LOWER_TRACK_H * 0.5)
UPPER_TRACK_Z = TOP_Z + LOWER_TRACK_H + (UPPER_TRACK_H * 0.5)

LID_LEN = 0.68
LID_W = 0.74
LID_T = 0.018
CONTROL_LID_Z = UPPER_TRACK_Z + UPPER_TRACK_H * 0.5 + LID_T * 0.5
FAR_LID_Z = LOWER_TRACK_Z + LOWER_TRACK_H * 0.5 + LID_T * 0.5
CONTROL_LID_Z = FAR_LID_Z + 0.018
CONTROL_LID_X = -0.26
FAR_LID_X = 0.26
LID_TRAVEL = 0.24

PANEL_D = 0.016
PANEL_W = 0.22
PANEL_H = 0.12
PANEL_X = -(CABINET_L * 0.5)
PANEL_Z = 0.60
KNOB_X_ON_PANEL = -PANEL_D


def _cabinet_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(CABINET_L, CABINET_W, BODY_H)
        .translate((0.0, 0.0, PLINTH_H + (BODY_H * 0.5)))
        .edges("|Z")
        .fillet(0.035)
    )
    cavity = cq.Workplane("XY").box(INNER_L, INNER_W, CAVITY_H).translate(
        (0.0, 0.0, CAVITY_BOTTOM_Z + (CAVITY_H * 0.5))
    )
    return shell.cut(cavity)


def _lid_frame_shape(*, grip_sign: float) -> cq.Workplane:
    frame = cq.Workplane("XY").box(LID_LEN, LID_W, LID_T)
    frame = frame.cut(cq.Workplane("XY").box(LID_LEN - 0.09, LID_W - 0.11, LID_T + 0.004))
    frame = frame.union(
        cq.Workplane("XY")
        .box(0.045, LID_W - 0.16, 0.008)
        .translate((grip_sign * ((LID_LEN * 0.5) - 0.0225), 0.0, 0.004))
    )
    return frame.edges("|Z").fillet(0.003)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="end_cap_display_freezer")

    model.material("shell_white", rgba=(0.92, 0.93, 0.94, 1.0))
    model.material("plinth_grey", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("rail_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("panel_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("panel_face", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("seal_grey", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("glass", rgba=(0.70, 0.84, 0.92, 0.32))
    model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell_shape(), "freezer_cabinet_shell"),
        material="shell_white",
        name="shell",
    )
    cabinet.visual(
        Box((1.20, 0.72, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H * 0.5)),
        material="plinth_grey",
        name="plinth",
    )
    for rail_index, rail_y in enumerate((-TRACK_Y, TRACK_Y)):
        cabinet.visual(
            Box((TRACK_L, 0.032, LOWER_TRACK_H)),
            origin=Origin(xyz=(0.0, rail_y, LOWER_TRACK_Z)),
            material="rail_aluminum",
            name=f"rail_{rail_index}_lower",
        )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((PANEL_D, PANEL_W, PANEL_H)),
        origin=Origin(xyz=(-PANEL_D * 0.5, 0.0, 0.0)),
        material="panel_graphite",
        name="housing",
    )
    control_panel.visual(
        Box((0.002, PANEL_W - 0.02, PANEL_H - 0.02)),
        origin=Origin(xyz=(-PANEL_D + 0.001, 0.0, 0.0)),
        material="panel_face",
        name="faceplate",
    )

    control_lid = model.part("control_lid")
    control_lid.visual(
        mesh_from_cadquery(_lid_frame_shape(grip_sign=-1.0), "control_lid_frame"),
        material="rail_aluminum",
        name="frame",
    )
    control_lid.visual(
        Box((LID_LEN - 0.04, LID_W - 0.06, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material="glass",
        name="glass",
    )
    control_lid.visual(
        Box((LID_LEN - 0.02, LID_W - 0.02, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material="seal_grey",
        name="seal",
    )

    far_lid = model.part("far_lid")
    far_lid.visual(
        mesh_from_cadquery(_lid_frame_shape(grip_sign=1.0), "far_lid_frame"),
        material="rail_aluminum",
        name="frame",
    )
    far_lid.visual(
        Box((LID_LEN - 0.04, LID_W - 0.06, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material="glass",
        name="glass",
    )
    far_lid.visual(
        Box((LID_LEN - 0.02, LID_W - 0.02, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material="seal_grey",
        name="seal",
    )

    temperature_knob = model.part("temperature_knob")
    knob_cap = mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.020,
            body_style="skirted",
            top_diameter=0.032,
            skirt=KnobSkirt(0.050, 0.004, flare=0.06),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "temperature_knob_cap",
    )
    temperature_knob.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, -math.pi * 0.5, 0.0)),
        material="knob_black",
        name="shaft",
    )
    temperature_knob.visual(
        knob_cap,
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, -math.pi * 0.5, 0.0)),
        material="knob_black",
        name="cap",
    )

    model.articulation(
        "cabinet_to_control_panel",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_panel,
        origin=Origin(xyz=(PANEL_X, 0.0, PANEL_Z)),
    )
    model.articulation(
        "cabinet_to_control_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=control_lid,
        origin=Origin(xyz=(CONTROL_LID_X, 0.0, CONTROL_LID_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.35, lower=0.0, upper=LID_TRAVEL),
    )
    model.articulation(
        "cabinet_to_far_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=far_lid,
        origin=Origin(xyz=(FAR_LID_X, 0.0, FAR_LID_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.35, lower=0.0, upper=LID_TRAVEL),
    )
    model.articulation(
        "control_panel_to_temperature_knob",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=temperature_knob,
        origin=Origin(xyz=(KNOB_X_ON_PANEL, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    control_panel = object_model.get_part("control_panel")
    control_lid = object_model.get_part("control_lid")
    far_lid = object_model.get_part("far_lid")
    temperature_knob = object_model.get_part("temperature_knob")

    control_slide = object_model.get_articulation("cabinet_to_control_lid")
    far_slide = object_model.get_articulation("cabinet_to_far_lid")
    knob_joint = object_model.get_articulation("control_panel_to_temperature_knob")

    ctx.expect_gap(
        cabinet,
        control_panel,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        name="control panel mounts flush to the control end wall",
    )
    ctx.expect_gap(
        control_lid,
        cabinet,
        axis="z",
        min_gap=0.0,
        max_gap=0.06,
        name="control lid rides above the cabinet rails",
    )
    ctx.expect_gap(
        far_lid,
        cabinet,
        axis="z",
        min_gap=0.0,
        max_gap=0.06,
        name="far lid rides above the cabinet rails",
    )
    ctx.expect_overlap(
        control_lid,
        far_lid,
        axes="xy",
        min_overlap=0.12,
        name="closed lids overlap in plan on stepped tracks",
    )
    ctx.expect_gap(
        control_panel,
        temperature_knob,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        name="knob stands on a short shaft proud of the panel face",
    )

    control_lid_rest = ctx.part_world_position(control_lid)
    far_lid_rest = ctx.part_world_position(far_lid)
    with ctx.pose({control_slide: LID_TRAVEL}):
        ctx.expect_overlap(
            control_lid,
            cabinet,
            axes="y",
            min_overlap=0.60,
            name="control lid stays captured by the side rails when opened",
        )
        control_lid_open = ctx.part_world_position(control_lid)
    with ctx.pose({far_slide: LID_TRAVEL}):
        ctx.expect_overlap(
            far_lid,
            cabinet,
            axes="y",
            min_overlap=0.60,
            name="far lid stays captured by the side rails when opened",
        )
        far_lid_open = ctx.part_world_position(far_lid)
    with ctx.pose({knob_joint: math.pi * 0.75}):
        ctx.expect_gap(
            control_panel,
            temperature_knob,
            axis="x",
            min_gap=0.0,
            max_gap=0.001,
            name="rotated knob remains seated on the control panel shaft",
        )

    ctx.check(
        "control lid slides toward the center",
        control_lid_rest is not None
        and control_lid_open is not None
        and control_lid_open[0] > control_lid_rest[0] + 0.18,
        details=f"rest={control_lid_rest}, open={control_lid_open}",
    )
    ctx.check(
        "far lid slides toward the center",
        far_lid_rest is not None and far_lid_open is not None and far_lid_open[0] < far_lid_rest[0] - 0.18,
        details=f"rest={far_lid_rest}, open={far_lid_open}",
    )

    return ctx.report()


object_model = build_object_model()
