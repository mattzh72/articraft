from __future__ import annotations

from math import cos, pi, sin

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


def _box(size: tuple[float, float, float], center: tuple[float, float, float], fillet: float | None = None):
    shape = cq.Workplane("XY").box(*size)
    if fillet is not None:
        shape = shape.edges("|Z").fillet(fillet)
    return shape.translate(center)


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]):
    return cq.Workplane("XY").cylinder(length, radius).translate(center)


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]):
    return cq.Workplane("XY").cylinder(length, radius).rotate((0, 0, 0), (1, 0, 0), 90).translate(center)


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]):
    return cq.Workplane("XY").cylinder(length, radius).rotate((0, 0, 0), (0, 1, 0), 90).translate(center)


def _frame_shape():
    base_lower = _box((0.240, 0.165, 0.018), (0.0, 0.0, 0.009), fillet=0.013)
    base_upper = _box((0.180, 0.118, 0.020), (-0.018, 0.0, 0.028), fillet=0.009)
    pedestal = _box((0.102, 0.084, 0.050), (-0.032, 0.0, 0.055), fillet=0.007)
    rear_column = _box((0.056, 0.048, 0.150), (-0.040, 0.0, 0.140), fillet=0.005)
    bridge = (
        cq.Workplane("XY")
        .box(0.072, 0.048, 0.034)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -22.0)
        .translate((-0.030, 0.0, 0.224))
    )

    return (
        base_lower.union(base_upper)
        .union(pedestal)
        .union(rear_column)
        .union(bridge)
    )


def _substage_shape():
    stage_arm = _box((0.105, 0.044, 0.018), (0.035, 0.0, 0.118), fillet=0.004)
    stage_pad = _box((0.090, 0.054, 0.004), (0.055, 0.0, 0.127))
    support_web = _box((0.040, 0.042, 0.070), (0.015, 0.0, 0.097))
    condenser_body = _cyl_z(0.022, 0.028, (0.055, 0.0, 0.110))
    condenser_ring = _cyl_z(0.030, 0.010, (0.055, 0.0, 0.094))
    pivot_mount = _box((0.018, 0.020, 0.018), (0.065, 0.021, 0.104))

    return (
        stage_arm.union(stage_pad)
        .union(support_web)
        .union(condenser_body)
        .union(condenser_ring)
        .union(pivot_mount)
    )


def _body_carriage_shape():
    sleeve = _box((0.072, 0.064, 0.082), (0.0, 0.0, 0.0), fillet=0.005)
    objective_tube = _cyl_z(0.020, 0.094, (0.0, 0.0, -0.001))
    lower_collar = _cyl_z(0.026, 0.012, (0.0, 0.0, -0.040))
    guide_block = _box((0.014, 0.046, 0.080), (-0.033, 0.0, 0.010), fillet=0.002)
    prism_block = (
        cq.Workplane("XY")
        .box(0.050, 0.048, 0.030)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 16.0)
        .translate((-0.010, 0.0, 0.057))
    )
    return sleeve.union(objective_tube).union(lower_collar).union(guide_block).union(prism_block)


def _body_knobs_shape():
    shaft = _cyl_y(0.0055, 0.072, (-0.010, 0.0, 0.004))
    knob_a = _cyl_y(0.018, 0.010, (-0.010, 0.041, 0.004))
    knob_b = _cyl_y(0.018, 0.010, (-0.010, -0.041, 0.004))
    return shaft.union(knob_a).union(knob_b)


def _eyepiece_shape():
    eyepiece_tube = (
        cq.Workplane("XY")
        .cylinder(0.082, 0.015)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -34.0)
        .translate((-0.012, 0.0, 0.108))
    )
    eye_cup = (
        cq.Workplane("XY")
        .cylinder(0.036, 0.012)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -34.0)
        .translate((-0.036, 0.0, 0.140))
    )
    lens_ring = (
        cq.Workplane("XY")
        .cylinder(0.008, 0.014)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -34.0)
        .translate((-0.048, 0.0, 0.151))
    )
    return eyepiece_tube.union(eye_cup).union(lens_ring)


def _nosepiece_shape():
    turret = _cyl_z(0.026, 0.012, (0.0, 0.0, -0.006))
    hub = _cyl_z(0.013, 0.010, (0.0, 0.0, 0.003))
    collar = _cyl_z(0.018, 0.006, (0.0, 0.0, -0.015))

    barrels = None
    for angle_deg, radius, length in (
        (0.0, 0.0085, 0.034),
        (120.0, 0.0070, 0.026),
        (240.0, 0.0070, 0.022),
    ):
        angle = angle_deg * pi / 180.0
        x_pos = 0.017 * cos(angle)
        y_pos = 0.017 * sin(angle)
        barrel = _cyl_z(radius, length, (x_pos, y_pos, -0.012 - length / 2.0))
        barrels = barrel if barrels is None else barrels.union(barrel)

    return turret.union(hub).union(collar).union(barrels)


def _stage_shape():
    plate = cq.Workplane("XY").box(0.120, 0.084, 0.008)
    plate = plate.faces(">Z").workplane().rect(0.028, 0.022).cutThruAll()
    stage = plate
    stage = stage.union(_box((0.090, 0.042, 0.010), (0.0, 0.0, -0.006)))
    stage = stage.union(_box((0.060, 0.004, 0.006), (0.0, -0.024, 0.005)))
    stage = stage.union(_box((0.034, 0.004, 0.009), (0.018, 0.024, 0.006)))
    stage = stage.union(_box((0.012, 0.018, 0.020), (0.008, 0.048, -0.004)))
    return stage


def _stage_knobs_shape():
    shaft = _cyl_y(0.004, 0.020, (0.012, 0.060, -0.004))
    knob_large = _cyl_y(0.013, 0.008, (0.012, 0.072, -0.004))
    knob_small = _cyl_y(0.009, 0.006, (0.012, 0.066, -0.004))
    return shaft.union(knob_large).union(knob_small)


def _lever_shape():
    pivot_barrel = _cyl_x(0.004, 0.008, (0.004, 0.0, 0.0))
    root_bridge = _box((0.010, 0.010, 0.004), (0.008, 0.007, -0.002))
    arm = (
        cq.Workplane("XY")
        .box(0.018, 0.032, 0.004)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -18.0)
        .translate((0.011, 0.020, -0.004))
    )
    connector = _box((0.007, 0.010, 0.004), (0.018, 0.029, -0.005))
    tip = _cyl_y(0.003, 0.012, (0.022, 0.034, -0.006))
    return pivot_barrel.union(root_bridge).union(arm).union(connector).union(tip)


def _add_mesh_visual(part, shape, name: str, material: str, visual_name: str):
    part.visual(mesh_from_cadquery(shape, name), material=material, name=visual_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monocular_microscope")

    model.material("cast_metal", rgba=(0.64, 0.66, 0.69, 1.0))
    model.material("dark_enamel", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("black_rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    model.material("brass", rgba=(0.73, 0.61, 0.26, 1.0))

    frame = model.part("frame")
    _add_mesh_visual(frame, _frame_shape(), "frame_body", "cast_metal", "frame_body")

    focus_guide = model.part("focus_guide")
    focus_guide.visual(
        Box((0.026, 0.046, 0.120)),
        origin=Origin(xyz=(0.002, 0.0, 0.252)),
        material="dark_enamel",
        name="guide_rail",
    )

    substage = model.part("substage")
    _add_mesh_visual(substage, _substage_shape(), "substage_body", "cast_metal", "substage_body")

    body_carriage = model.part("body_carriage")
    _add_mesh_visual(body_carriage, _body_carriage_shape(), "body_carriage_shell", "cast_metal", "carriage_shell")
    _add_mesh_visual(body_carriage, _body_knobs_shape(), "body_focus_knobs", "dark_enamel", "focus_knobs")
    _add_mesh_visual(body_carriage, _eyepiece_shape(), "body_eyepiece", "black_rubber", "eyepiece")

    nosepiece = model.part("nosepiece")
    _add_mesh_visual(nosepiece, _nosepiece_shape(), "nosepiece_body", "brass", "nosepiece_body")

    stage_carriage = model.part("stage_carriage")
    stage_carriage.visual(
        mesh_from_cadquery(_stage_shape(), "stage_carriage_body"),
        material="dark_enamel",
        name="stage_body",
    )
    stage_carriage.visual(
        mesh_from_cadquery(_stage_knobs_shape(), "stage_controls"),
        material="dark_enamel",
        name="guide_controls",
    )

    diaphragm_lever = model.part("diaphragm_lever")
    _add_mesh_visual(diaphragm_lever, _lever_shape(), "diaphragm_lever", "dark_enamel", "lever_body")

    model.articulation(
        "frame_to_focus_guide",
        ArticulationType.FIXED,
        parent=frame,
        child=focus_guide,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_substage",
        ArticulationType.FIXED,
        parent=frame,
        child=substage,
        origin=Origin(),
    )

    model.articulation(
        "focus_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=body_carriage,
        origin=Origin(xyz=(0.055, 0.0, 0.265)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.055, effort=18.0, velocity=0.060),
    )
    model.articulation(
        "nosepiece_spin",
        ArticulationType.CONTINUOUS,
        parent=body_carriage,
        child=nosepiece,
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=stage_carriage,
        origin=Origin(xyz=(0.055, 0.0, 0.143)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.020, upper=0.020, effort=10.0, velocity=0.030),
    )
    model.articulation(
        "diaphragm_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=diaphragm_lever,
        origin=Origin(xyz=(0.074, 0.030, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=0.45, effort=1.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    focus_guide = object_model.get_part("focus_guide")
    substage = object_model.get_part("substage")
    body_carriage = object_model.get_part("body_carriage")
    nosepiece = object_model.get_part("nosepiece")
    stage_carriage = object_model.get_part("stage_carriage")
    diaphragm_lever = object_model.get_part("diaphragm_lever")

    focus_slide = object_model.get_articulation("focus_slide")
    nosepiece_spin = object_model.get_articulation("nosepiece_spin")
    stage_slide = object_model.get_articulation("stage_slide")
    diaphragm_pivot = object_model.get_articulation("diaphragm_pivot")

    focus_limits = focus_slide.motion_limits
    stage_limits = stage_slide.motion_limits
    lever_limits = diaphragm_pivot.motion_limits

    ctx.allow_overlap(
        frame,
        focus_guide,
        reason="The focus guide is split into a fixed helper part but represents the same cast sliding support blended into the microscope arm.",
    )
    ctx.allow_overlap(
        frame,
        substage,
        reason="The substage support is authored as a fixed helper part to keep the stage and diaphragm support explicit, but it represents the same cast microscope body.",
    )

    ctx.expect_overlap(
        nosepiece,
        stage_carriage,
        axes="xy",
        elem_b="stage_body",
        min_overlap=0.020,
        name="nosepiece stays centered over the slide stage",
    )
    ctx.expect_gap(
        nosepiece,
        stage_carriage,
        axis="z",
        negative_elem="stage_body",
        min_gap=0.004,
        max_gap=0.030,
        name="objectives clear the stage at the low focus stop",
    )
    ctx.expect_contact(
        body_carriage,
        focus_guide,
        name="focus carriage stays guided by the upright rail",
    )
    ctx.expect_contact(
        stage_carriage,
        substage,
        name="stage carriage stays seated on the substage support",
    )
    ctx.expect_contact(
        diaphragm_lever,
        substage,
        name="diaphragm lever stays supported by the condenser housing",
    )
    ctx.expect_gap(
        stage_carriage,
        diaphragm_lever,
        axis="z",
        positive_elem="stage_body",
        min_gap=0.014,
        name="diaphragm lever remains under the stage",
    )

    rest_body_pos = ctx.part_world_position(body_carriage)
    if focus_limits is not None and focus_limits.upper is not None:
        with ctx.pose({focus_slide: focus_limits.upper}):
            upper_body_pos = ctx.part_world_position(body_carriage)
            ctx.expect_gap(
                nosepiece,
                stage_carriage,
                axis="z",
                negative_elem="stage_body",
                min_gap=0.050,
                name="focus slide lifts the objective cluster away from the stage",
            )
        ctx.check(
            "focus slide raises the body carriage",
            rest_body_pos is not None
            and upper_body_pos is not None
            and upper_body_pos[2] > rest_body_pos[2] + 0.045,
            details=f"rest={rest_body_pos}, upper={upper_body_pos}",
        )

    rest_stage_pos = ctx.part_world_position(stage_carriage)
    if stage_limits is not None and stage_limits.upper is not None:
        with ctx.pose({stage_slide: stage_limits.upper}):
            upper_stage_pos = ctx.part_world_position(stage_carriage)
        ctx.check(
            "stage carriage travels laterally across the saddle",
            rest_stage_pos is not None
            and upper_stage_pos is not None
            and upper_stage_pos[1] > rest_stage_pos[1] + 0.018,
            details=f"rest={rest_stage_pos}, upper={upper_stage_pos}",
        )

    with ctx.pose({nosepiece_spin: 1.2}):
        ctx.expect_gap(
            nosepiece,
            stage_carriage,
            axis="z",
            negative_elem="stage_body",
            min_gap=0.004,
            name="rotated nosepiece still clears the stage",
        )
        ctx.expect_overlap(
            nosepiece,
            stage_carriage,
            axes="xy",
            elem_b="stage_body",
            min_overlap=0.018,
            name="rotated nosepiece remains over the stage opening",
        )

    if lever_limits is not None and lever_limits.upper is not None:
        with ctx.pose({diaphragm_pivot: lever_limits.upper}):
            ctx.expect_contact(
                diaphragm_lever,
                substage,
                name="lever pivot stays seated at its upper stop",
            )
            ctx.expect_gap(
                stage_carriage,
                diaphragm_lever,
                axis="z",
                positive_elem="stage_body",
                min_gap=0.008,
                name="lever upper stop remains below the stage",
            )

    return ctx.report()


object_model = build_object_model()
