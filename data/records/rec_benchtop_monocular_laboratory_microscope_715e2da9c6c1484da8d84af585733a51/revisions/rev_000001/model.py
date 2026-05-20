from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery box with gently rounded vertical edges, already centered at origin."""

    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _swept_arm() -> cq.Workplane:
    """A broad, swept microscope arm extruded through the width of the stand."""

    profile = [
        (-0.116, 0.030),
        (-0.074, 0.030),
        (-0.049, 0.105),
        (-0.047, 0.294),
        (-0.080, 0.320),
        (-0.112, 0.276),
        (-0.130, 0.120),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(0.055, both=True)
        .edges("|Y")
        .fillet(0.004)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_monocular_microscope")

    body_white = Material("warm_white_enamel", rgba=(0.86, 0.84, 0.78, 1.0))
    dark_metal = Material("charcoal_metal", rgba=(0.05, 0.055, 0.06, 1.0))
    brushed = Material("brushed_steel", rgba=(0.66, 0.68, 0.66, 1.0))
    black = Material("black_rubber", rgba=(0.006, 0.006, 0.007, 1.0))
    glass = Material("slide_glass", rgba=(0.72, 0.90, 1.0, 0.38))

    for material in (body_white, dark_metal, brushed, black, glass):
        model.material(material)

    frame = model.part("frame")

    base_mesh = _rounded_box((0.240, 0.160, 0.032), 0.007).translate((0.0, 0.0, 0.016))
    frame.visual(
        mesh_from_cadquery(base_mesh, "rounded_rectangular_base", tolerance=0.0008),
        material=dark_metal,
        name="base",
    )
    frame.visual(
        mesh_from_cadquery(_swept_arm(), "swept_arm", tolerance=0.0008),
        material=body_white,
        name="swept_arm",
    )

    # Fixed stage and its single pedestal.
    frame.visual(
        Box((0.050, 0.070, 0.116)),
        origin=Origin(xyz=(0.018, 0.0, 0.088)),
        material=body_white,
        name="stage_pedestal",
    )
    frame.visual(
        Box((0.145, 0.125, 0.014)),
        origin=Origin(xyz=(0.032, 0.0, 0.145)),
        material=dark_metal,
        name="stage_plate",
    )
    frame.visual(
        Box((0.132, 0.010, 0.003)),
        origin=Origin(xyz=(0.032, -0.055, 0.1505)),
        material=brushed,
        name="stage_rail_0",
    )
    frame.visual(
        Box((0.132, 0.010, 0.003)),
        origin=Origin(xyz=(0.032, 0.055, 0.1505)),
        material=brushed,
        name="stage_rail_1",
    )

    # Twin vertical guide rails on the front of the swept arm.
    frame.visual(
        Box((0.018, 0.014, 0.205)),
        origin=Origin(xyz=(-0.052, -0.034, 0.198)),
        material=brushed,
        name="guide_rail_0",
    )
    frame.visual(
        Box((0.018, 0.014, 0.205)),
        origin=Origin(xyz=(-0.052, 0.034, 0.198)),
        material=brushed,
        name="guide_rail_1",
    )

    # A split bearing yoke captures the focus shaft without filling the shaft space.
    frame.visual(
        Box((0.040, 0.048, 0.010)),
        origin=Origin(xyz=(-0.130, 0.0, 0.218)),
        material=body_white,
        name="focus_upper_lug",
    )
    frame.visual(
        Box((0.040, 0.048, 0.010)),
        origin=Origin(xyz=(-0.130, 0.0, 0.192)),
        material=body_white,
        name="focus_lower_lug",
    )

    slide_carriage = model.part("slide_carriage")
    slide_carriage.visual(
        Box((0.112, 0.045, 0.006)),
        origin=Origin(),
        material=brushed,
        name="carriage_plate",
    )
    slide_carriage.visual(
        Box((0.032, 0.080, 0.006)),
        origin=Origin(xyz=(-0.038, 0.050, 0.0)),
        material=brushed,
        name="projecting_tongue",
    )
    slide_carriage.visual(
        Box((0.078, 0.026, 0.0015)),
        origin=Origin(xyz=(0.006, -0.002, 0.00375)),
        material=glass,
        name="glass_slide",
    )

    model.articulation(
        "stage_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slide_carriage,
        origin=Origin(xyz=(0.032, 0.0, 0.155)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.12, lower=-0.035, upper=0.035),
    )

    head_carriage = model.part("head_carriage")
    head_carriage.visual(
        Box((0.046, 0.088, 0.060)),
        origin=Origin(),
        material=body_white,
        name="slider_block",
    )
    head_carriage.visual(
        Box((0.056, 0.052, 0.026)),
        origin=Origin(xyz=(0.030, 0.0, -0.015)),
        material=body_white,
        name="head_neck",
    )
    head_carriage.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.052, 0.0, -0.029)),
        material=dark_metal,
        name="nosepiece",
    )
    head_carriage.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=Origin(xyz=(0.062, 0.0, -0.032)),
        material=brushed,
        name="objective_barrel",
    )
    head_carriage.visual(
        Cylinder(radius=0.020, length=0.105),
        origin=Origin(xyz=(0.020, 0.0, 0.060), rpy=(0.0, 0.55, 0.0)),
        material=body_white,
        name="monocular_tube",
    )
    head_carriage.visual(
        Cylinder(radius=0.014, length=0.045),
        origin=Origin(xyz=(0.060, 0.0, 0.113), rpy=(0.0, 0.55, 0.0)),
        material=black,
        name="eyepiece",
    )

    model.articulation(
        "guide_to_head",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=head_carriage,
        origin=Origin(xyz=(-0.020, 0.0, 0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.05, lower=-0.010, upper=0.055),
    )

    focus_knobs = model.part("focus_knobs")
    focus_knobs.visual(
        Cylinder(radius=0.008, length=0.206),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="shared_shaft",
    )
    focus_knobs.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.0, -0.110, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_0",
    )
    focus_knobs.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.0, 0.110, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_1",
    )
    focus_knobs.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, -0.127, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="knob_cap_0",
    )
    focus_knobs.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.127, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="knob_cap_1",
    )

    model.articulation(
        "arm_to_focus",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=focus_knobs,
        origin=Origin(xyz=(-0.145, 0.0, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    slide = object_model.get_part("slide_carriage")
    head = object_model.get_part("head_carriage")
    knobs = object_model.get_part("focus_knobs")
    stage_slide = object_model.get_articulation("stage_to_carriage")
    guide_slide = object_model.get_articulation("guide_to_head")
    focus = object_model.get_articulation("arm_to_focus")

    ctx.expect_gap(
        slide,
        frame,
        axis="z",
        positive_elem="carriage_plate",
        negative_elem="stage_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="slide carriage rides on the stage top",
    )
    ctx.expect_within(
        slide,
        frame,
        axes="x",
        inner_elem="carriage_plate",
        outer_elem="stage_plate",
        margin=0.003,
        name="slide carriage stays within the stage front-back width",
    )
    ctx.expect_gap(
        head,
        frame,
        axis="x",
        positive_elem="slider_block",
        negative_elem="guide_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="head carriage bears on the vertical guide rails",
    )
    ctx.expect_contact(
        knobs,
        frame,
        elem_a="shared_shaft",
        elem_b="focus_upper_lug",
        contact_tol=0.0015,
        name="focus shaft is captured by the upper lug",
    )

    rest_slide = ctx.part_world_position(slide)
    with ctx.pose({stage_slide: stage_slide.motion_limits.upper}):
        extended_slide = ctx.part_world_position(slide)
        ctx.expect_overlap(
            slide,
            frame,
            axes="xy",
            elem_a="carriage_plate",
            elem_b="stage_plate",
            min_overlap=0.030,
            name="translated slide carriage remains on the stage",
        )

    rest_head = ctx.part_world_position(head)
    with ctx.pose({guide_slide: guide_slide.motion_limits.upper}):
        raised_head = ctx.part_world_position(head)
        ctx.expect_overlap(
            head,
            frame,
            axes="z",
            elem_a="slider_block",
            elem_b="guide_rail_0",
            min_overlap=0.035,
            name="raised head carriage remains engaged with the guide",
        )

    ctx.check(
        "slide carriage translates left-right",
        rest_slide is not None
        and extended_slide is not None
        and extended_slide[1] > rest_slide[1] + 0.030,
        details=f"rest={rest_slide}, extended={extended_slide}",
    )
    ctx.check(
        "head carriage slides upward on the vertical guide",
        rest_head is not None
        and raised_head is not None
        and raised_head[2] > rest_head[2] + 0.050,
        details=f"rest={rest_head}, raised={raised_head}",
    )
    ctx.check(
        "focus knobs use a shared horizontal shaft",
        focus.axis == (0.0, 1.0, 0.0)
        and focus.motion_limits is not None
        and focus.motion_limits.lower is not None
        and focus.motion_limits.upper is not None
        and focus.motion_limits.upper - focus.motion_limits.lower >= 2.0 * math.pi - 0.01,
        details=f"axis={focus.axis}, limits={focus.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
