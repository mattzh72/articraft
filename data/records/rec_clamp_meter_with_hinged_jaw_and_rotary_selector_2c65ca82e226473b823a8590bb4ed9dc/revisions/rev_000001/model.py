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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_THICKNESS = 0.038
BODY_FRONT_Y = BODY_THICKNESS / 2.0
JAW_THICKNESS = 0.024
JAW_HINGE_X = 0.016
JAW_HINGE_Z = 0.216


def _jaw_shape():
    hook = (
        cq.Workplane("XZ")
        .moveTo(0.004, 0.002)
        .threePointArc((-0.018, 0.014), (-0.040, -0.002))
        .threePointArc((-0.048, -0.020), (-0.034, -0.050))
        .lineTo(-0.022, -0.041)
        .threePointArc((-0.035, -0.024), (-0.010, -0.010))
        .lineTo(0.003, -0.006)
        .close()
        .extrude(JAW_THICKNESS / 2.0, both=True)
    )
    barrel = cq.Workplane("XZ").circle(0.0048).extrude(0.009, both=True)
    return hook.union(barrel).val()


def _lower_housing_shape():
    return (
        cq.Workplane("XY")
        .rect(0.044, BODY_THICKNESS)
        .workplane(offset=0.084)
        .rect(0.046, BODY_THICKNESS * 0.96)
        .workplane(offset=0.040)
        .rect(0.056, BODY_THICKNESS + 0.002)
        .workplane(offset=0.024)
        .rect(0.060, BODY_THICKNESS + 0.004)
        .loft(combine=True)
        .val()
    )


def _trigger_shape():
    return (
        cq.Workplane("XZ")
        .moveTo(-0.006, 0.000)
        .lineTo(0.006, 0.000)
        .lineTo(0.006, -0.010)
        .lineTo(0.010, -0.014)
        .lineTo(0.010, -0.026)
        .lineTo(-0.010, -0.026)
        .lineTo(-0.010, -0.014)
        .lineTo(-0.006, -0.010)
        .close()
        .extrude(0.008)
    ).val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_clamp_meter")

    housing = model.material("housing_yellow", rgba=(0.88, 0.78, 0.18, 1.0))
    face = model.material("face_charcoal", rgba=(0.13, 0.14, 0.16, 1.0))
    jaw_dark = model.material("jaw_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    control_dark = model.material("control_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    control_gray = model.material("control_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    glass = model.material("glass", rgba=(0.32, 0.46, 0.50, 0.45))
    accent = model.material("accent_red", rgba=(0.73, 0.12, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_lower_housing_shape(), "clamp_meter_lower_housing"),
        material=housing,
        name="housing_shell",
    )
    body.visual(
        Box((0.068, BODY_THICKNESS + 0.004, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=housing,
        name="head_shell",
    )
    body.visual(
        Box((0.026, JAW_THICKNESS, 0.026)),
        origin=Origin(xyz=(0.014, 0.0, 0.182)),
        material=housing,
        name="jaw_support",
    )
    body.visual(
        Cylinder(radius=0.010, length=JAW_THICKNESS),
        origin=Origin(xyz=(0.018, 0.0, 0.194), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=housing,
        name="jaw_corner",
    )
    body.visual(
        Box((0.008, JAW_THICKNESS, 0.046)),
        origin=Origin(xyz=(0.026, 0.0, 0.188)),
        material=housing,
        name="jaw_leg",
    )
    body.visual(
        Cylinder(radius=0.010, length=JAW_THICKNESS),
        origin=Origin(xyz=(0.000, 0.0, 0.167), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=housing,
        name="jaw_base_curve",
    )
    body.visual(
        Box((0.038, JAW_THICKNESS, 0.006)),
        origin=Origin(xyz=(0.000, 0.0, 0.159)),
        material=housing,
        name="jaw_base",
    )
    body.visual(
        Box((0.056, 0.0015, 0.124)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.00075, 0.098)),
        material=face,
        name="face_panel",
    )
    body.visual(
        Box((0.030, 0.0015, 0.024)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.00155, 0.124)),
        material=glass,
        name="display_window",
    )
    body.visual(
        Box((0.004, JAW_THICKNESS * 0.60, 0.010)),
        origin=Origin(xyz=(-0.010, 0.0, 0.170)),
        material=jaw_dark,
        name="fixed_land",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_jaw_shape(), "clamp_meter_jaw"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=jaw_dark,
        name="jaw_arc",
    )
    jaw.visual(
        Box((0.004, JAW_THICKNESS * 0.55, 0.010)),
        origin=Origin(xyz=(-0.032, 0.0, -0.044)),
        material=jaw_dark,
        name="jaw_tip",
    )

    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_cadquery(_trigger_shape(), "clamp_meter_trigger"),
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
        material=control_dark,
        name="trigger_cap",
    )

    selector_knob = mesh_from_geometry(
        KnobGeometry(
            0.032,
            0.012,
            body_style="cylindrical",
            top_diameter=0.026,
            edge_radius=0.0014,
            grip=KnobGrip(style="fluted", count=14, depth=0.0011),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            center=False,
        ),
        "clamp_meter_selector",
    )
    selector = model.part("selector")
    selector.visual(
        selector_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=control_dark,
        name="selector_knob",
    )

    ncv_button = model.part("ncv_button")
    ncv_button.visual(
        Cylinder(radius=0.0046, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="ncv_cap",
    )

    range_button = model.part("range_button")
    range_button.visual(
        Box((0.014, 0.0030, 0.010)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        material=control_gray,
        name="range_cap",
    )

    model.articulation(
        "jaw_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(JAW_HINGE_X, 0.0, JAW_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=0.95),
    )
    model.articulation(
        "trigger_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.0015, 0.156)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.010),
    )
    model.articulation(
        "selector_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.0015, 0.089)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0),
    )
    model.articulation(
        "ncv_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=ncv_button,
        origin=Origin(xyz=(-0.018, BODY_FRONT_Y + 0.0015, 0.148)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.0016),
    )
    model.articulation(
        "range_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=range_button,
        origin=Origin(xyz=(0.027, BODY_FRONT_Y + 0.0015, 0.088)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.0015),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    trigger = object_model.get_part("trigger")
    selector = object_model.get_part("selector")
    ncv_button = object_model.get_part("ncv_button")
    range_button = object_model.get_part("range_button")

    jaw_hinge = object_model.get_articulation("jaw_hinge")
    trigger_slide = object_model.get_articulation("trigger_slide")
    ncv_press = object_model.get_articulation("ncv_press")
    range_press = object_model.get_articulation("range_press")

    ctx.expect_gap(
        selector,
        body,
        axis="y",
        positive_elem="selector_knob",
        negative_elem="face_panel",
        max_gap=0.0020,
        max_penetration=0.0,
        name="selector sits on the front control face",
    )
    ctx.expect_gap(
        ncv_button,
        body,
        axis="y",
        positive_elem="ncv_cap",
        negative_elem="face_panel",
        max_gap=0.0015,
        max_penetration=0.0,
        name="NCV button sits proud on the shoulder",
    )
    ctx.expect_gap(
        range_button,
        body,
        axis="y",
        positive_elem="range_cap",
        negative_elem="face_panel",
        max_gap=0.0015,
        max_penetration=0.0,
        name="range button sits proud beside the dial",
    )
    ctx.expect_origin_gap(
        ncv_button,
        selector,
        axis="z",
        min_gap=0.045,
        name="NCV button sits above the selector",
    )
    ctx.expect_origin_gap(
        range_button,
        selector,
        axis="x",
        min_gap=0.018,
        name="range button sits beside the selector",
    )

    jaw_limits = jaw_hinge.motion_limits
    if jaw_limits is not None and jaw_limits.upper is not None:
        with ctx.pose({jaw_hinge: 0.0}):
            ctx.expect_gap(
                body,
                jaw,
                axis="x",
                positive_elem="fixed_land",
                negative_elem="jaw_tip",
                max_gap=0.0020,
                max_penetration=0.0,
                name="jaw closes to a narrow gap at the fixed land",
            )

        closed_tip = ctx.part_element_world_aabb(jaw, elem="jaw_tip")
        with ctx.pose({jaw_hinge: jaw_limits.upper}):
            open_tip = ctx.part_element_world_aabb(jaw, elem="jaw_tip")

        open_clear = False
        if closed_tip is not None and open_tip is not None:
            open_clear = open_tip[0][0] > closed_tip[0][0] + 0.020 and open_tip[0][2] < closed_tip[0][2] - 0.006
        ctx.check(
            "jaw swings open from the head hinge",
            open_clear,
            details=f"closed_tip={closed_tip}, open_tip={open_tip}",
        )

    trigger_limits = trigger_slide.motion_limits
    if trigger_limits is not None and trigger_limits.upper is not None:
        trigger_rest = ctx.part_world_position(trigger)
        with ctx.pose({trigger_slide: trigger_limits.upper}):
            trigger_pulled = ctx.part_world_position(trigger)
        trigger_ok = False
        if trigger_rest is not None and trigger_pulled is not None:
            trigger_ok = trigger_pulled[2] < trigger_rest[2] - 0.007
        ctx.check(
            "trigger slides downward under the head",
            trigger_ok,
            details=f"rest={trigger_rest}, pulled={trigger_pulled}",
        )

    ncv_limits = ncv_press.motion_limits
    if ncv_limits is not None and ncv_limits.upper is not None:
        ncv_rest = ctx.part_world_position(ncv_button)
        range_rest = ctx.part_world_position(range_button)
        with ctx.pose({ncv_press: ncv_limits.upper}):
            ncv_pressed = ctx.part_world_position(ncv_button)
            range_steady = ctx.part_world_position(range_button)
        ncv_ok = False
        if (
            ncv_rest is not None
            and ncv_pressed is not None
            and range_rest is not None
            and range_steady is not None
        ):
            ncv_ok = ncv_pressed[1] < ncv_rest[1] - 0.001 and abs(range_steady[1] - range_rest[1]) < 1e-6
        ctx.check(
            "NCV button presses independently",
            ncv_ok,
            details=f"rest={ncv_rest}, pressed={ncv_pressed}, range_rest={range_rest}, range_during_ncv={range_steady}",
        )

    range_limits = range_press.motion_limits
    if range_limits is not None and range_limits.upper is not None:
        range_rest = ctx.part_world_position(range_button)
        ncv_rest = ctx.part_world_position(ncv_button)
        with ctx.pose({range_press: range_limits.upper}):
            range_pressed = ctx.part_world_position(range_button)
            ncv_steady = ctx.part_world_position(ncv_button)
        range_ok = False
        if (
            range_rest is not None
            and range_pressed is not None
            and ncv_rest is not None
            and ncv_steady is not None
        ):
            range_ok = range_pressed[1] < range_rest[1] - 0.001 and abs(ncv_steady[1] - ncv_rest[1]) < 1e-6
        ctx.check(
            "range button presses independently",
            range_ok,
            details=f"rest={range_rest}, pressed={range_pressed}, ncv_rest={ncv_rest}, ncv_during_range={ncv_steady}",
        )

    return ctx.report()


object_model = build_object_model()
