from __future__ import annotations

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


OPENING_WIDTH = 1.70
OPENING_HEIGHT = 1.55
PIER_WIDTH = 0.58
WALL_THICKNESS = 0.60
SILL_HEIGHT = 0.35
BEAM_HEIGHT = 0.24
FRAME_HEIGHT = 3.60
STRUCTURE_WIDTH = OPENING_WIDTH + 2.0 * PIER_WIDTH

GUIDE_HEIGHT = 3.00
GUIDE_BOTTOM_Z = 0.25
GUIDE_CENTER_Z = GUIDE_BOTTOM_Z + GUIDE_HEIGHT / 2.0
GUIDE_WEB_X = 0.815
GUIDE_WEB_THICKNESS = 0.05
GUIDE_RETAINER_X = 0.760
GUIDE_RETAINER_WIDTH = 0.06

PANEL_Y = 0.10
PANEL_WIDTH = 1.50
PANEL_PLATE_WIDTH = 1.38
PANEL_HEIGHT = 2.45
PANEL_THICKNESS = 0.04
RUNNER_WIDTH = 0.06
RUNNER_DEPTH = 0.08
RUNNER_CENTER_X = 0.72
PANEL_BOTTOM_Z = SILL_HEIGHT
PANEL_CENTER_Z = PANEL_BOTTOM_Z + PANEL_HEIGHT / 2.0
PANEL_TRAVEL = 1.55

BEAM_BOTTOM_Z = SILL_HEIGHT + OPENING_HEIGHT
BEAM_TOP_Z = BEAM_BOTTOM_Z + BEAM_HEIGHT

HOUSING_WIDTH = 0.56
HOUSING_DEPTH = 0.32
HOUSING_HEIGHT = 0.38
GEARBOX_CENTER = (0.0, -0.36, BEAM_TOP_Z + 0.39)


def _handwheel_shape():
    rim = cq.Workplane("XZ").circle(0.26).circle(0.21).extrude(0.009, both=True)
    spoke = cq.Workplane("XZ").rect(0.44, 0.028).extrude(0.007, both=True)

    wheel = rim
    for angle_deg in (0.0, 45.0, 90.0, 135.0):
        wheel = wheel.union(spoke.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg))

    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sluice_gate")

    masonry = model.material("masonry", rgba=(0.62, 0.60, 0.56, 1.0))
    steel = model.material("steel", rgba=(0.36, 0.39, 0.42, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.25, 0.34, 0.42, 1.0))
    gearbox_finish = model.material("gearbox_finish", rgba=(0.24, 0.29, 0.33, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.18, 0.19, 0.20, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((PIER_WIDTH, WALL_THICKNESS, FRAME_HEIGHT)),
        origin=Origin(xyz=(-(OPENING_WIDTH / 2.0 + PIER_WIDTH / 2.0), 0.0, FRAME_HEIGHT / 2.0)),
        material=masonry,
        name="left_pier",
    )
    frame.visual(
        Box((PIER_WIDTH, WALL_THICKNESS, FRAME_HEIGHT)),
        origin=Origin(xyz=((OPENING_WIDTH / 2.0 + PIER_WIDTH / 2.0), 0.0, FRAME_HEIGHT / 2.0)),
        material=masonry,
        name="right_pier",
    )
    frame.visual(
        Box((STRUCTURE_WIDTH, WALL_THICKNESS, SILL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, SILL_HEIGHT / 2.0)),
        material=masonry,
        name="sill",
    )
    frame.visual(
        Box((OPENING_WIDTH + 0.34, 0.24, BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.12, BEAM_BOTTOM_Z + BEAM_HEIGHT / 2.0)),
        material=steel,
        name="lintel_beam",
    )

    frame.visual(
        Box((GUIDE_WEB_THICKNESS, 0.16, GUIDE_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_WEB_X, PANEL_Y, GUIDE_CENTER_Z)),
        material=steel,
        name="left_guide_web",
    )
    frame.visual(
        Box((GUIDE_RETAINER_WIDTH, 0.02, GUIDE_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_RETAINER_X, PANEL_Y + 0.06, GUIDE_CENTER_Z)),
        material=steel,
        name="left_front_keeper",
    )
    frame.visual(
        Box((GUIDE_RETAINER_WIDTH, 0.02, GUIDE_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_RETAINER_X, PANEL_Y - 0.06, GUIDE_CENTER_Z)),
        material=steel,
        name="left_back_keeper",
    )
    frame.visual(
        Box((GUIDE_WEB_THICKNESS, 0.16, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_WEB_X, PANEL_Y, GUIDE_CENTER_Z)),
        material=steel,
        name="right_guide_web",
    )
    frame.visual(
        Box((GUIDE_RETAINER_WIDTH, 0.02, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_RETAINER_X, PANEL_Y + 0.06, GUIDE_CENTER_Z)),
        material=steel,
        name="right_front_keeper",
    )
    frame.visual(
        Box((GUIDE_RETAINER_WIDTH, 0.02, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_RETAINER_X, PANEL_Y - 0.06, GUIDE_CENTER_Z)),
        material=steel,
        name="right_back_keeper",
    )

    panel = model.part("panel")
    panel.visual(
        Box((PANEL_PLATE_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)),
        material=panel_finish,
        name="panel_plate",
    )
    panel.visual(
        Box((RUNNER_WIDTH, RUNNER_DEPTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(-RUNNER_CENTER_X, 0.0, 0.0)),
        material=steel,
        name="left_runner",
    )
    panel.visual(
        Box((RUNNER_WIDTH, RUNNER_DEPTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(RUNNER_CENTER_X, 0.0, 0.0)),
        material=steel,
        name="right_runner",
    )
    for z_pos, name in ((-0.68, "stiffener_low"), (0.0, "stiffener_mid"), (0.68, "stiffener_high")):
        panel.visual(
            Box((1.24, 0.07, 0.10)),
            origin=Origin(xyz=(0.0, 0.015, z_pos)),
            material=steel,
            name=name,
        )
    panel.visual(
        Box((PANEL_WIDTH, 0.07, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -PANEL_HEIGHT / 2.0 + 0.06)),
        material=steel,
        name="bottom_beam",
    )

    gearbox = model.part("gearbox")
    gearbox.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        material=gearbox_finish,
        name="housing_shell",
    )
    gearbox.visual(
        Box((0.30, 0.24, 0.20)),
        origin=Origin(xyz=(0.0, 0.16, -0.29)),
        material=steel,
        name="mount_pedestal",
    )
    gearbox.visual(
        Box((0.30, 0.24, 0.10)),
        origin=Origin(xyz=(0.0, 0.00, 0.24)),
        material=gearbox_finish,
        name="top_cover",
    )
    gearbox.visual(
        Cylinder(radius=0.03, length=0.16),
        origin=Origin(xyz=(-0.16, 0.24, 0.02), rpy=(-1.57079632679, 0.0, 0.0)),
        material=steel,
        name="input_shaft",
    )
    gearbox.visual(
        Box((0.022, 0.012, 0.28)),
        origin=Origin(xyz=(0.019, HOUSING_DEPTH / 2.0 + 0.006, -0.05)),
        material=steel,
        name="door_hinge_leaf",
    )
    gearbox.visual(
        Cylinder(radius=0.007, length=0.04),
        origin=Origin(xyz=(0.025, HOUSING_DEPTH / 2.0 + 0.009, -0.05)),
        material=steel,
        name="hinge_knuckle_low",
    )
    gearbox.visual(
        Cylinder(radius=0.007, length=0.04),
        origin=Origin(xyz=(0.025, HOUSING_DEPTH / 2.0 + 0.009, 0.05)),
        material=steel,
        name="hinge_knuckle_high",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_handwheel_shape(), "sluice_handwheel"),
        material=wheel_finish,
        name="wheel",
    )
    handwheel.visual(
        Cylinder(radius=0.05, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-1.57079632679, 0.0, 0.0)),
        material=steel,
        name="hub",
    )

    inspection_door = model.part("inspection_door")
    inspection_door.visual(
        Box((0.20, 0.018, 0.14)),
        origin=Origin(xyz=(0.10, 0.009, 0.0)),
        material=steel,
        name="door_panel",
    )
    inspection_door.visual(
        Cylinder(radius=0.007, length=0.06),
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
        material=steel,
        name="hinge_knuckle_mid",
    )
    inspection_door.visual(
        Box((0.03, 0.03, 0.03)),
        origin=Origin(xyz=(0.15, 0.024, 0.0)),
        material=wheel_finish,
        name="door_latch",
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, PANEL_Y, PANEL_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=PANEL_TRAVEL, effort=3000.0, velocity=0.25),
    )
    model.articulation(
        "frame_to_gearbox",
        ArticulationType.FIXED,
        parent=frame,
        child=gearbox,
        origin=Origin(xyz=GEARBOX_CENTER),
    )
    model.articulation(
        "gearbox_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=gearbox,
        child=handwheel,
        origin=Origin(xyz=(-0.16, 0.36, 0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0),
    )
    model.articulation(
        "gearbox_to_inspection_door",
        ArticulationType.REVOLUTE,
        parent=gearbox,
        child=inspection_door,
        origin=Origin(xyz=(0.04, HOUSING_DEPTH / 2.0, -0.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=5.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    gearbox = object_model.get_part("gearbox")
    handwheel = object_model.get_part("handwheel")
    inspection_door = object_model.get_part("inspection_door")

    panel_slide = object_model.get_articulation("frame_to_panel")
    door_hinge = object_model.get_articulation("gearbox_to_inspection_door")

    ctx.expect_gap(
        panel,
        frame,
        axis="x",
        positive_elem="left_runner",
        negative_elem="left_guide_web",
        max_gap=0.05,
        max_penetration=0.0,
        name="left panel runner stays inside the left guide",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="x",
        positive_elem="right_guide_web",
        negative_elem="right_runner",
        max_gap=0.05,
        max_penetration=0.0,
        name="right panel runner stays inside the right guide",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="y",
        positive_elem="left_front_keeper",
        negative_elem="left_runner",
        max_gap=0.02,
        max_penetration=0.0,
        name="front keeper captures the panel edge",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="y",
        positive_elem="left_runner",
        negative_elem="left_back_keeper",
        max_gap=0.02,
        max_penetration=0.0,
        name="back keeper captures the panel edge",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="z",
        positive_elem="bottom_beam",
        negative_elem="sill",
        max_gap=0.003,
        max_penetration=0.0,
        name="closed panel seats on the sill",
    )

    panel_limits = panel_slide.motion_limits
    if panel_limits is not None and panel_limits.upper is not None:
        rest_pos = ctx.part_world_position(panel)
        with ctx.pose({panel_slide: panel_limits.upper}):
            ctx.expect_overlap(
                panel,
                frame,
                axes="z",
                elem_a="left_runner",
                elem_b="left_guide_web",
                min_overlap=1.20,
                name="lifted panel remains engaged in the left guide",
            )
            ctx.expect_gap(
                panel,
                frame,
                axis="z",
                positive_elem="bottom_beam",
                negative_elem="sill",
                min_gap=1.50,
                name="raised panel clears the opening",
            )
            raised_pos = ctx.part_world_position(panel)
        ctx.check(
            "panel lifts upward",
            rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 1.50,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    ctx.expect_contact(
        handwheel,
        gearbox,
        elem_a="hub",
        elem_b="input_shaft",
        name="handwheel hub is supported by the gearbox shaft",
    )
    ctx.expect_gap(
        inspection_door,
        gearbox,
        axis="y",
        positive_elem="door_panel",
        negative_elem="housing_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="inspection door sits flush on the housing face",
    )
    ctx.expect_overlap(
        inspection_door,
        gearbox,
        axes="xz",
        elem_a="door_panel",
        elem_b="housing_shell",
        min_overlap=0.10,
        name="inspection door stays mounted on the housing front",
    )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(inspection_door, elem="door_panel")
        with ctx.pose({door_hinge: door_limits.upper}):
            open_aabb = ctx.part_element_world_aabb(inspection_door, elem="door_panel")
        ctx.check(
            "inspection door opens outward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][1] > closed_aabb[1][1] + 0.08,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
