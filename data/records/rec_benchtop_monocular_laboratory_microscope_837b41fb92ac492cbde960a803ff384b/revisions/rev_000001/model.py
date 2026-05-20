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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_X = 0.38
BASE_Y = 0.24
BASE_Z = 0.050
ARM_X = -0.130
GUIDE_X = -0.088
OPTICAL_X = 0.045
STAGE_Z = 0.250
CARRIAGE_Z = 0.430
FOCUS_Z = 0.340


def _fillet_edges(shape, selector: str, radius: float):
    """Best-effort CadQuery fillet used only for visible softenings."""
    try:
        return shape.edges(selector).fillet(radius)
    except Exception:
        return shape


def _rounded_foot():
    foot = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_Z)
    foot = _fillet_edges(foot, "|Z", 0.045)
    foot = _fillet_edges(foot, ">Z", 0.006)
    return foot


def _arm_column_with_bore():
    # Local frame is centered on the arm.  A generous rectangular through-bore
    # leaves real clearance for the rotating focus shaft.
    arm = cq.Workplane("XY").box(0.070, 0.070, 0.540)
    arm = _fillet_edges(arm, "|Z", 0.010)
    bore = cq.Workplane("XY").box(0.034, 0.092, 0.034).translate(
        (0.0, 0.0, FOCUS_Z - 0.315)
    )
    return arm.cut(bore)


def _stage_plate():
    # Rectangular microscope stage with a real transmitted-light aperture.
    return (
        cq.Workplane("XY")
        .box(0.230, 0.145, 0.014)
        .faces(">Z")
        .workplane()
        .circle(0.018)
        .cutThruAll()
    )


def _bearing_ring(length: float = 0.010):
    ring = cq.Workplane("XY").circle(0.024).circle(0.006).extrude(length)
    return ring.translate((0.0, 0.0, -length / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_monocular_microscope")

    enamel = model.material("black_enamel", rgba=(0.025, 0.028, 0.030, 1.0))
    satin_black = model.material("satin_black", rgba=(0.060, 0.064, 0.066, 1.0))
    machined = model.material("machined_metal", rgba=(0.62, 0.63, 0.60, 1.0))
    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.82, 1.0))
    glass = model.material("lens_glass", rgba=(0.30, 0.52, 0.58, 0.55))
    brass = model.material("objective_brass", rgba=(0.75, 0.62, 0.32, 1.0))
    slide_glass = model.material("slide_glass", rgba=(0.76, 0.92, 0.96, 0.35))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_rounded_foot(), "heavy_rounded_foot"),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z / 2.0)),
        material=enamel,
        name="heavy_foot",
    )
    stand.visual(
        mesh_from_cadquery(_arm_column_with_bore(), "vertical_arm_bored"),
        origin=Origin(xyz=(ARM_X, 0.0, 0.315)),
        material=enamel,
        name="vertical_arm",
    )
    stand.visual(
        Box((0.095, 0.090, 0.040)),
        origin=Origin(xyz=(ARM_X + 0.006, 0.0, BASE_Z + 0.015)),
        material=enamel,
        name="arm_heel",
    )

    # Stage support yoke and the rectangular stage are fixed to the stand.
    stand.visual(
        Box((0.175, 0.060, 0.028)),
        origin=Origin(xyz=(-0.040, 0.0, STAGE_Z - 0.042)),
        material=enamel,
        name="stage_support",
    )
    stand.visual(
        Box((0.055, 0.046, 0.021)),
        origin=Origin(xyz=(0.010, 0.0, 0.2325)),
        material=enamel,
        name="stage_pedestal",
    )
    stand.visual(
        mesh_from_cadquery(_stage_plate(), "rectangular_stage_with_aperture"),
        origin=Origin(xyz=(OPTICAL_X, 0.0, STAGE_Z)),
        material=satin_black,
        name="rectangular_stage",
    )
    stand.visual(
        Box((0.075, 0.022, 0.0035)),
        origin=Origin(xyz=(0.030, 0.045, STAGE_Z + 0.00875)),
        material=chrome,
        name="spring_clip_0",
    )
    stand.visual(
        Box((0.075, 0.022, 0.0035)),
        origin=Origin(xyz=(0.030, -0.045, STAGE_Z + 0.00875)),
        material=chrome,
        name="spring_clip_1",
    )
    for y in (-0.045, 0.045):
        stand.visual(
            Cylinder(radius=0.007, length=0.003),
            origin=Origin(xyz=(-0.002, y, STAGE_Z + 0.012)),
            material=machined,
            name=f"clip_screw_{0 if y < 0 else 1}",
        )
    stand.visual(
        Box((0.078, 0.030, 0.002)),
        origin=Origin(xyz=(0.045, 0.0, STAGE_Z + 0.008)),
        material=slide_glass,
        name="glass_slide",
    )

    # Polished guide rail and rack on the front of the vertical arm.
    stand.visual(
        Cylinder(radius=0.008, length=0.340),
        origin=Origin(xyz=(GUIDE_X, 0.0, 0.445)),
        material=chrome,
        name="vertical_guide",
    )
    stand.visual(
        Box((0.006, 0.012, 0.330)),
        origin=Origin(xyz=(GUIDE_X - 0.011, 0.0, 0.445)),
        material=machined,
        name="focus_rack",
    )
    stand.visual(
        Box((0.028, 0.018, 0.006)),
        origin=Origin(xyz=(ARM_X, -0.040, FOCUS_Z - 0.009)),
        material=machined,
        name="focus_bearing_0",
    )
    stand.visual(
        Box((0.028, 0.018, 0.016)),
        origin=Origin(xyz=(ARM_X, -0.040, FOCUS_Z - 0.020)),
        material=machined,
        name="bearing_web_0",
    )
    stand.visual(
        Box((0.028, 0.018, 0.006)),
        origin=Origin(xyz=(ARM_X, 0.040, FOCUS_Z - 0.009)),
        material=machined,
        name="focus_bearing_1",
    )
    stand.visual(
        Box((0.028, 0.018, 0.016)),
        origin=Origin(xyz=(ARM_X, 0.040, FOCUS_Z - 0.020)),
        material=machined,
        name="bearing_web_1",
    )

    carriage = model.part("carriage")
    # A U-shaped saddle wraps the vertical guide without intersecting it.
    carriage.visual(
        Box((0.018, 0.010, 0.090)),
        origin=Origin(xyz=(0.000, 0.024, 0.000)),
        material=enamel,
        name="saddle_cheek_0",
    )
    carriage.visual(
        Box((0.018, 0.010, 0.090)),
        origin=Origin(xyz=(0.000, -0.024, 0.000)),
        material=enamel,
        name="saddle_cheek_1",
    )
    carriage.visual(
        Box((0.014, 0.058, 0.090)),
        origin=Origin(xyz=(0.015, 0.0, 0.000)),
        material=enamel,
        name="saddle_bridge",
    )
    carriage.visual(
        Box((0.114, 0.040, 0.026)),
        origin=Origin(xyz=(0.079, 0.0, 0.036)),
        material=enamel,
        name="tube_support_bridge",
    )
    carriage.visual(
        Box((0.060, 0.058, 0.050)),
        origin=Origin(xyz=(OPTICAL_X - GUIDE_X, 0.0, 0.040)),
        material=enamel,
        name="tube_socket",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.220),
        origin=Origin(xyz=(OPTICAL_X - GUIDE_X, 0.0, 0.128)),
        material=satin_black,
        name="observation_tube",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(OPTICAL_X - GUIDE_X, 0.0, 0.270)),
        material=machined,
        name="eyepiece_sleeve",
    )
    carriage.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(OPTICAL_X - GUIDE_X, 0.0, 0.308)),
        material=glass,
        name="eyepiece_lens",
    )

    model.articulation(
        "stand_to_carriage",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=carriage,
        origin=Origin(xyz=(GUIDE_X, 0.0, CARRIAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.080, lower=-0.045, upper=0.075),
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=machined,
        name="turret_disk",
    )
    nosepiece.visual(
        Cylinder(radius=0.010, length=0.074),
        origin=Origin(xyz=(0.029, 0.0, -0.046)),
        material=brass,
        name="objective_0",
    )
    nosepiece.visual(
        Cylinder(radius=0.0078, length=0.004),
        origin=Origin(xyz=(0.029, 0.0, -0.085)),
        material=glass,
        name="objective_lens_0",
    )
    for index, angle in enumerate((2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        radius = 0.029
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        obj_radius = (0.0085, 0.0075)[index - 1]
        obj_len = (0.064, 0.056)[index - 1]
        nosepiece.visual(
            Cylinder(radius=obj_radius, length=obj_len),
            origin=Origin(xyz=(x, y, -0.009 - obj_len / 2.0)),
            material=satin_black,
            name=f"objective_{index}",
        )
        nosepiece.visual(
            Cylinder(radius=obj_radius * 0.78, length=0.004),
            origin=Origin(xyz=(x, y, -0.011 - obj_len)),
            material=glass,
            name=f"objective_lens_{index}",
        )
    model.articulation(
        "carriage_to_nosepiece",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=nosepiece,
        origin=Origin(xyz=(OPTICAL_X - GUIDE_X, 0.0, 0.009)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.20, velocity=2.0),
    )

    focus_shaft = model.part("focus_shaft")
    focus_shaft.visual(
        Cylinder(radius=0.006, length=0.198),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="shaft_core",
    )
    focus_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.064,
            0.026,
            body_style="faceted",
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0012),
        ),
        "ribbed_focus_knob",
    )
    for index, y in enumerate((-0.094, 0.094)):
        focus_shaft.visual(
            focus_knob_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name=f"focus_knob_{index}",
        )
        focus_shaft.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.0, y * 0.80, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=machined,
            name=f"knob_hub_{index}",
        )
    model.articulation(
        "stand_to_focus_shaft",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=focus_shaft,
        origin=Origin(xyz=(ARM_X, 0.0, FOCUS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.40, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    carriage = object_model.get_part("carriage")
    nosepiece = object_model.get_part("nosepiece")
    focus_shaft = object_model.get_part("focus_shaft")
    slide = object_model.get_articulation("stand_to_carriage")
    turret = object_model.get_articulation("carriage_to_nosepiece")
    focus = object_model.get_articulation("stand_to_focus_shaft")

    ctx.check(
        "microscope_has_requested_articulations",
        slide.articulation_type == ArticulationType.PRISMATIC
        and turret.articulation_type == ArticulationType.CONTINUOUS
        and focus.articulation_type == ArticulationType.CONTINUOUS,
        details=f"slide={slide.articulation_type}, turret={turret.articulation_type}, focus={focus.articulation_type}",
    )
    ctx.check(
        "vertical_carriage_travel",
        slide.axis == (0.0, 0.0, 1.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower < 0.0
        and slide.motion_limits.upper > 0.050,
        details=f"axis={slide.axis}, limits={slide.motion_limits}",
    )
    ctx.check("turret_axis_is_optical", turret.axis == (0.0, 0.0, 1.0), details=f"axis={turret.axis}")
    ctx.check("focus_axis_is_horizontal", focus.axis == (0.0, 1.0, 0.0), details=f"axis={focus.axis}")
    ctx.expect_contact(
        stand,
        carriage,
        elem_a="vertical_guide",
        elem_b="saddle_bridge",
        contact_tol=0.0015,
        name="carriage_saddle_runs_on_vertical_guide",
    )
    ctx.expect_contact(
        stand,
        focus_shaft,
        elem_a="focus_bearing_0",
        elem_b="shaft_core",
        contact_tol=0.0045,
        name="focus_shaft_passes_through_bearings",
    )
    ctx.expect_gap(
        nosepiece,
        stand,
        axis="z",
        positive_elem="objective_0",
        negative_elem="rectangular_stage",
        min_gap=0.020,
        name="objective_clears_stage",
    )
    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: slide.motion_limits.upper}):
        raised_position = ctx.part_world_position(carriage)
        ctx.expect_gap(
            nosepiece,
            stand,
            axis="z",
            positive_elem="objective_0",
            negative_elem="rectangular_stage",
            min_gap=0.080,
            name="raised_objective_clears_stage",
        )
    ctx.check(
        "carriage_slides_upward",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.050,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


object_model = build_object_model()
