from __future__ import annotations

from typing import Iterable

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.060
BODY_FRONT_Y = BODY_DEPTH * 0.5
BODY_BACK_Y = -BODY_FRONT_Y
JAW_CENTER_Z = 0.156
JAW_OUTER_RADIUS = 0.046
JAW_INNER_RADIUS = 0.034


def _build_body_shape() -> cq.Workplane:
    handle = (
        cq.Workplane("XY")
        .box(0.076, BODY_DEPTH, 0.170)
        .edges("|Z")
        .fillet(0.009)
        .translate((0.0, 0.0, -0.030))
    )
    shoulder = (
        cq.Workplane("XY")
        .box(0.092, BODY_DEPTH, 0.060)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, 0.050))
    )
    head = (
        cq.Workplane("XY")
        .box(0.112, BODY_DEPTH, 0.108)
        .edges("|Z")
        .fillet(0.009)
        .translate((0.0, 0.0, 0.124))
    )

    shell = handle.union(shoulder).union(head)

    fixed_jaw = (
        cq.Workplane("XZ")
        .circle(JAW_OUTER_RADIUS)
        .extrude(BODY_DEPTH * 0.5, both=True)
        .translate((0.0, 0.0, JAW_CENTER_Z))
    )
    shell = shell.union(fixed_jaw)
    shell = shell.cut(
        cq.Workplane("XZ")
        .circle(JAW_INNER_RADIUS)
        .extrude(BODY_DEPTH * 0.5 + 0.005, both=True)
        .translate((0.0, 0.0, JAW_CENTER_Z))
    )
    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.150, BODY_DEPTH + 0.020, 0.120)
        .translate((0.0, 0.0, JAW_CENTER_Z + 0.060))
    )

    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.070, 0.006, 0.048)
        .translate((0.0, 0.027, 0.060))
    )
    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.012, 0.008, 0.008)
        .translate((-0.018, 0.026, 0.014))
    )
    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.012, 0.008, 0.008)
        .translate((0.018, 0.026, 0.014))
    )
    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.054, 0.004, 0.112)
        .translate((0.0, -0.028, -0.014))
    )

    hinge_saddle = cq.Workplane("XY").box(0.024, BODY_DEPTH, 0.020).translate((-0.034, 0.0, JAW_CENTER_Z - 0.018))
    shell = shell.union(hinge_saddle)

    return shell


def _build_jaw_shape() -> cq.Workplane:
    upper_arc = (
        cq.Workplane("XZ")
        .circle(JAW_OUTER_RADIUS)
        .extrude(BODY_DEPTH * 0.5, both=True)
        .translate((JAW_OUTER_RADIUS, 0.0, 0.0))
    )
    upper_arc = upper_arc.cut(
        cq.Workplane("XZ")
        .circle(JAW_INNER_RADIUS)
        .extrude(BODY_DEPTH * 0.5 + 0.005, both=True)
        .translate((JAW_OUTER_RADIUS, 0.0, 0.0))
    )
    upper_arc = upper_arc.cut(
        cq.Workplane("XY")
        .box(0.140, BODY_DEPTH + 0.020, 0.090)
        .translate((JAW_OUTER_RADIUS, 0.0, -0.043))
    )

    hinge_bridge = cq.Workplane("XY").box(0.018, BODY_DEPTH, 0.020).translate((0.010, 0.0, 0.011))
    end_tip = cq.Workplane("XY").box(0.014, BODY_DEPTH, 0.016).translate((0.091, 0.0, 0.014))
    return upper_arc.union(hinge_bridge).union(end_tip)


def _build_selector_knob_shape() -> cq.Workplane:
    skirt = cq.Workplane("XY").circle(0.028).extrude(0.006)
    body = cq.Workplane("XY").circle(0.0225).workplane(offset=0.006).extrude(0.018)
    knob = skirt.union(body)
    return knob.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)


def _build_mode_button_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").box(0.016, 0.006, 0.010).translate((0.0, 0.003, 0.0))
    stem = cq.Workplane("XY").box(0.010, 0.006, 0.006).translate((0.0, -0.003, 0.0))
    return cap.union(stem)


def _build_kickstand_shape() -> cq.Workplane:
    panel_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.016, 0.004),
                (0.016, 0.004),
                (0.026, 0.112),
                (-0.026, 0.112),
            ]
        )
        .close()
    )
    panel = panel_profile.extrude(0.004).translate((0.0, -0.004, 0.0))

    slot_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.008, 0.020),
                (0.008, 0.020),
                (0.015, 0.092),
                (-0.015, 0.092),
            ]
        )
        .close()
    )
    panel = panel.cut(slot_profile.extrude(0.006).translate((0.0, -0.005, 0.0)))

    hinge_barrel = (
        cq.Workplane("YZ")
        .circle(0.005)
        .extrude(0.038, both=True)
        .translate((0.0, -0.005, 0.0))
    )
    return panel.union(hinge_barrel)


def _center_from_aabb(aabb: tuple[Iterable[float], Iterable[float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    mins = tuple(mins)
    maxs = tuple(maxs)
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="power_clamp_meter")

    shell_mat = model.material("shell_charcoal", rgba=(0.19, 0.20, 0.21, 1.0))
    jaw_mat = model.material("jaw_black", rgba=(0.11, 0.12, 0.13, 1.0))
    trim_mat = model.material("trim_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    button_mat = model.material("button_grey", rgba=(0.28, 0.29, 0.31, 1.0))
    glass_mat = model.material("display_glass", rgba=(0.20, 0.32, 0.36, 0.45))
    stand_mat = model.material("stand_dark", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "clamp_meter_body"),
        material=shell_mat,
        name="shell",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_build_jaw_shape(), "clamp_meter_jaw"),
        material=jaw_mat,
        name="jaw_arc",
    )
    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(-JAW_OUTER_RADIUS, 0.0, JAW_CENTER_Z - 0.001)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.2, lower=0.0, upper=1.18),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_cadquery(_build_selector_knob_shape(), "clamp_meter_selector_knob"),
        material=trim_mat,
        name="knob",
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, -0.038)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    for index, x_offset in enumerate((-0.018, 0.018)):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.016, 0.0025, 0.010)),
            origin=Origin(xyz=(0.0, 0.00125, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material=button_mat,
            name="button_stem",
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_offset, BODY_FRONT_Y, 0.014)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.0018),
        )

    display = model.part("display")
    display.visual(
        Box((0.062, 0.004, 0.042)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=glass_mat,
        name="display_glass",
    )
    model.articulation(
        "body_to_display",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.006, 0.060)),
    )

    kickstand = model.part("kickstand")
    kickstand.visual(
        mesh_from_cadquery(_build_kickstand_shape(), "clamp_meter_kickstand"),
        material=stand_mat,
        name="stand_panel",
    )
    model.articulation(
        "body_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kickstand,
        origin=Origin(xyz=(0.0, BODY_BACK_Y, -0.072)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    selector_knob = object_model.get_part("selector_knob")
    display = object_model.get_part("display")
    kickstand = object_model.get_part("kickstand")
    jaw_joint = object_model.get_articulation("body_to_jaw")
    selector_joint = object_model.get_articulation("body_to_selector_knob")
    stand_joint = object_model.get_articulation("body_to_kickstand")

    ctx.check(
        "selector knob uses continuous rotation",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={selector_joint.articulation_type}",
    )
    ctx.expect_gap(
        jaw,
        display,
        axis="z",
        elem_a="jaw_arc",
        elem_b="display_glass",
        min_gap=0.028,
        name="jaw opening sits above the display",
    )

    jaw_limits = jaw_joint.motion_limits
    if jaw_limits is not None and jaw_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(jaw, elem="jaw_arc")
        with ctx.pose({jaw_joint: jaw_limits.upper}):
            open_aabb = ctx.part_element_world_aabb(jaw, elem="jaw_arc")
        ctx.check(
            "jaw opens upward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.020,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    stand_limits = stand_joint.motion_limits
    if stand_limits is not None and stand_limits.upper is not None:
        closed_center = _center_from_aabb(ctx.part_element_world_aabb(kickstand, elem="stand_panel"))
        with ctx.pose({stand_joint: stand_limits.upper}):
            open_center = _center_from_aabb(ctx.part_element_world_aabb(kickstand, elem="stand_panel"))
        ctx.check(
            "kickstand swings rearward",
            closed_center is not None
            and open_center is not None
            and open_center[1] < closed_center[1] - 0.030,
            details=f"closed={closed_center}, open={open_center}",
        )

    for index in range(2):
        button = object_model.get_part(f"mode_button_{index}")
        button_joint = object_model.get_articulation(f"body_to_mode_button_{index}")
        ctx.check(
            f"mode button {index} is prismatic",
            button_joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={button_joint.articulation_type}",
        )
        ctx.expect_gap(
            button,
            body,
            axis="y",
            elem_a="button_cap",
            elem_b="shell",
            min_gap=0.0,
            max_gap=0.001,
            name=f"mode button {index} sits proud of the face",
        )
        limits = button_joint.motion_limits
        if limits is not None and limits.upper is not None:
            rest_pos = ctx.part_world_position(button)
            with ctx.pose({button_joint: limits.upper}):
                pressed_pos = ctx.part_world_position(button)
            ctx.check(
                f"mode button {index} presses inward",
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[1] < rest_pos[1] - 0.001,
                details=f"rest={rest_pos}, pressed={pressed_pos}",
            )

    return ctx.report()


object_model = build_object_model()
