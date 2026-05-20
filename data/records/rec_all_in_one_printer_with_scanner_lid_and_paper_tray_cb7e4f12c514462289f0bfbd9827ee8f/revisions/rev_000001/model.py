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

CHASSIS_W = 0.380
CHASSIS_D = 0.360
CHASSIS_H = 0.270

LID_W = 0.304
LID_D = 0.236
LID_T = 0.020

TRAY_W = 0.286
TRAY_D = 0.140
TRAY_H = 0.032

DOOR_W = 0.266
DOOR_H = 0.056
DOOR_T = 0.006

PANEL_W = 0.182
PANEL_H = 0.068
PANEL_T = 0.008


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _build_chassis_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(CHASSIS_W, CHASSIS_D, CHASSIS_H).translate((0.0, 0.0, CHASSIS_H / 2.0))
    shell = shell.edges("|Z").fillet(0.012)
    shell = shell.edges(">Z").fillet(0.006)

    scanner_pocket = _cq_box((0.286, 0.214, 0.018), (0.0, 0.010, CHASSIS_H - 0.009))
    tray_tunnel = _cq_box((0.324, 0.252, 0.056), (0.0, -0.054, 0.043))
    output_cavity = _cq_box((0.274, 0.036, 0.056), (0.0, -(CHASSIS_D / 2.0) + 0.018, 0.118))
    control_recess = _cq_box((0.182, 0.010, 0.068), (0.0, -(CHASSIS_D / 2.0) + 0.005, 0.192))
    bottom_relief = _cq_box((0.220, 0.140, 0.010), (0.0, 0.006, 0.005))

    shell = shell.cut(scanner_pocket)
    shell = shell.cut(tray_tunnel)
    shell = shell.cut(output_cavity)
    shell = shell.cut(control_recess)
    shell = shell.cut(bottom_relief)

    return shell


def _build_lid_shell() -> cq.Workplane:
    lid = _cq_box((LID_W, LID_D, LID_T), (0.0, -LID_D / 2.0, LID_T / 2.0))
    lid = lid.edges("|X and >Z").fillet(0.004)

    inner_pocket = _cq_box((LID_W - 0.016, LID_D - 0.020, LID_T - 0.003), (0.0, -LID_D / 2.0, (LID_T - 0.003) / 2.0))
    cable_notch = _cq_box((0.070, 0.014, 0.010), (0.0, -0.010, 0.005))

    lid = lid.cut(inner_pocket)
    lid = lid.cut(cable_notch)
    return lid


def _build_tray_shell() -> cq.Workplane:
    tray = _cq_box((TRAY_W, TRAY_D, TRAY_H), (0.0, (TRAY_D / 2.0) - 0.004, TRAY_H / 2.0))
    cavity = _cq_box(
        (TRAY_W - 0.014, TRAY_D - 0.022, TRAY_H - 0.004),
        (0.0, 0.074, (TRAY_H - 0.004) / 2.0 + 0.004),
    )
    finger_scoop = (
        cq.Workplane("XZ")
        .center(0.0, 0.018)
        .circle(0.036)
        .extrude(0.012)
        .translate((0.0, -0.012, 0.0))
    )

    tray = tray.cut(cavity)
    tray = tray.cut(finger_scoop)
    tray = tray.union(_cq_box((0.010, 0.094, 0.003), (-(TRAY_W / 2.0) + 0.015, 0.060, -0.0015)))
    tray = tray.union(_cq_box((0.010, 0.094, 0.003), ((TRAY_W / 2.0) - 0.015, 0.060, -0.0015)))
    return tray


def _build_output_door() -> cq.Workplane:
    panel = _cq_box((DOOR_W, DOOR_T, DOOR_H), (0.0, -DOOR_T / 2.0, DOOR_H / 2.0))
    lip = _cq_box((0.110, 0.010, 0.010), (0.0, -0.003, DOOR_H - 0.010))
    inner_relief = _cq_box((DOOR_W - 0.018, 0.003, DOOR_H - 0.016), (0.0, -DOOR_T + 0.0015, DOOR_H / 2.0 + 0.001))
    return panel.union(lip).cut(inner_relief)


def _build_control_panel() -> cq.Workplane:
    panel = cq.Workplane("XY").box(PANEL_W, PANEL_T, PANEL_H)
    button_slot = lambda x: _cq_box((0.016, PANEL_T + 0.002, 0.010), (x, 0.0, -0.010))
    panel = panel.cut(button_slot(0.028))
    panel = panel.cut(button_slot(0.054))
    panel = panel.cut(button_slot(0.080))
    return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_all_in_one_printer")

    body_finish = model.material("body_graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    lid_finish = model.material("lid_dark", rgba=(0.30, 0.31, 0.33, 1.0))
    tray_finish = model.material("tray_black", rgba=(0.16, 0.17, 0.18, 1.0))
    door_finish = model.material("door_smoke", rgba=(0.24, 0.25, 0.27, 1.0))
    panel_finish = model.material("panel_black", rgba=(0.10, 0.11, 0.12, 1.0))
    knob_finish = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    button_finish = model.material("button_grey", rgba=(0.62, 0.64, 0.67, 1.0))
    glass_finish = model.material("scanner_glass", rgba=(0.43, 0.56, 0.61, 0.35))

    chassis = model.part("chassis")
    chassis.visual(
        mesh_from_cadquery(_build_chassis_shell(), "printer_chassis"),
        material=body_finish,
        name="shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "printer_lid"),
        material=lid_finish,
        name="shell",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_build_tray_shell(), "printer_tray"),
        material=tray_finish,
        name="drawer",
    )

    output_door = model.part("output_door")
    output_door.visual(
        mesh_from_cadquery(_build_output_door(), "printer_output_door"),
        material=door_finish,
        name="panel",
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        mesh_from_cadquery(_build_control_panel(), "printer_control_panel"),
        material=panel_finish,
        name="panel",
    )

    scanner_bed = model.part("scanner_bed")
    scanner_bed.visual(
        Box((0.272, 0.200, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=glass_finish,
        name="glass",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="skirt",
    )
    selector_knob.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="cap",
    )
    selector_knob.visual(
        Box((0.003, 0.0015, 0.010)),
        origin=Origin(xyz=(0.010, -0.019, 0.0)),
        material=button_finish,
        name="pointer",
    )

    for index, local_x in enumerate((0.028, 0.054, 0.080)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.016, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=button_finish,
            name="cap",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=lid,
        origin=Origin(xyz=(0.0, 0.128, CHASSIS_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.6, lower=0.0, upper=1.30),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=tray,
        origin=Origin(xyz=(0.0, -(CHASSIS_D / 2.0), 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.30, lower=0.0, upper=0.108),
    )
    model.articulation(
        "output_door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=output_door,
        origin=Origin(xyz=(0.0, -(CHASSIS_D / 2.0), 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "chassis_to_control_panel",
        ArticulationType.FIXED,
        parent=chassis,
        child=control_panel,
        origin=Origin(xyz=(0.0, -0.176, 0.192)),
    )
    model.articulation(
        "chassis_to_scanner_bed",
        ArticulationType.FIXED,
        parent=chassis,
        child=scanner_bed,
        origin=Origin(xyz=(0.0, 0.010, 0.252)),
    )
    model.articulation(
        "selector_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=selector_knob,
        origin=Origin(xyz=(-0.040, 0.0, 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=6.0),
    )
    for index, local_x in enumerate((0.028, 0.054, 0.080)):
        model.articulation(
            f"button_{index}_push",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=model.get_part(f"button_{index}"),
            origin=Origin(xyz=(local_x, 0.0, -0.010)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=0.0, upper=0.003),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")
    output_door = object_model.get_part("output_door")
    control_panel = object_model.get_part("control_panel")
    selector_knob = object_model.get_part("selector_knob")
    lid_hinge = object_model.get_articulation("lid_hinge")
    tray_slide = object_model.get_articulation("tray_slide")
    output_door_hinge = object_model.get_articulation("output_door_hinge")
    selector_knob_spin = object_model.get_articulation("selector_knob_spin")

    ctx.allow_overlap(
        chassis,
        tray,
        elem_a="shell",
        elem_b="drawer",
        reason="The lower paper tray is intentionally represented as a retained cassette nested inside the simplified lower chassis shell.",
    )
    ctx.allow_overlap(
        chassis,
        control_panel,
        elem_a="shell",
        elem_b="panel",
        reason="The front control panel is intentionally represented as an inset panel nested into the simplified chassis front recess.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            chassis,
            axis="z",
            max_gap=0.004,
            max_penetration=0.0,
            name="lid seats on chassis top",
        )
        ctx.expect_overlap(
            lid,
            chassis,
            axes="xy",
            min_overlap=0.220,
            name="lid covers the flatbed opening",
        )

    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        closed_lid_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.090,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    tray_limits = tray_slide.motion_limits
    if tray_limits is not None and tray_limits.upper is not None:
        closed_tray_pos = ctx.part_world_position(tray)
        with ctx.pose({tray_slide: tray_limits.upper}):
            open_tray_pos = ctx.part_world_position(tray)
            open_tray_aabb = ctx.part_world_aabb(tray)
        ctx.check(
            "tray slides out forward",
            closed_tray_pos is not None
            and open_tray_pos is not None
            and open_tray_pos[1] < closed_tray_pos[1] - 0.080,
            details=f"closed={closed_tray_pos}, open={open_tray_pos}",
        )
        ctx.check(
            "tray retains insertion at full extension",
            open_tray_aabb is not None and open_tray_aabb[1][1] > -0.165,
            details=f"open={open_tray_aabb}",
        )

    door_limits = output_door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        closed_door_aabb = ctx.part_world_aabb(output_door)
        with ctx.pose({output_door_hinge: door_limits.upper}):
            open_door_aabb = ctx.part_world_aabb(output_door)
        ctx.check(
            "output door flips down",
            closed_door_aabb is not None
            and open_door_aabb is not None
            and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.030
            and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.020,
            details=f"closed={closed_door_aabb}, open={open_door_aabb}",
        )

    panel_aabb = ctx.part_world_aabb(control_panel)
    knob_aabb = ctx.part_world_aabb(selector_knob)
    ctx.check(
        "selector knob sits on the front panel",
        panel_aabb is not None
        and knob_aabb is not None
        and knob_aabb[0][1] < panel_aabb[0][1] - 0.008
        and knob_aabb[1][2] <= panel_aabb[1][2] + 0.010,
        details=f"panel={panel_aabb}, knob={knob_aabb}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"button_{index}_push")
        button_limits = button_joint.motion_limits
        if button_limits is None or button_limits.upper is None:
            continue
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} pushes inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.002,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
