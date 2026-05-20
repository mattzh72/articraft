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

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_slider")

    # Housing with T-slot track
    housing_profile = (
        cq.Workplane("YZ")
        .moveTo(-0.015, 0.0)
        .lineTo(0.015, 0.0)
        .lineTo(0.015, 0.01)
        .lineTo(0.007, 0.01)
        .lineTo(0.007, 0.008)
        .lineTo(0.01, 0.008)
        .lineTo(0.01, 0.004)
        .lineTo(-0.01, 0.004)
        .lineTo(-0.01, 0.008)
        .lineTo(-0.007, 0.008)
        .lineTo(-0.007, 0.01)
        .lineTo(-0.015, 0.01)
        .close()
        .extrude(0.07)
        .translate((-0.035, 0, 0))
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(housing_profile, "housing_mesh"),
        name="housing_body",
    )
    # Pin to hold the detent
    housing.visual(
        Cylinder(radius=0.0015, length=0.005),
        origin=Origin(xyz=(0.0, 0.015, 0.0125)),
        name="detent_pin",
    )

    # Slider matching the T-slot
    slider_solid = (
        cq.Workplane("YZ")
        .moveTo(-0.0095, 0.0045)
        .lineTo(0.0095, 0.0045)
        .lineTo(0.0095, 0.0075)
        .lineTo(0.0065, 0.0075)
        .lineTo(0.0065, 0.0105)
        .lineTo(0.012, 0.0105)
        .lineTo(0.012, 0.014)
        .lineTo(-0.012, 0.014)
        .lineTo(-0.012, 0.0105)
        .lineTo(-0.0065, 0.0105)
        .lineTo(-0.0065, 0.0075)
        .lineTo(-0.0095, 0.0075)
        .close()
        .extrude(0.04)
        .translate((-0.02, 0, 0))
    )

    # Cut notches on the +Y side of the thumb pad for the detent to click into
    notches = (
        cq.Workplane("XY")
        .workplane(offset=0.01225)
        .pushPoints([(-0.015, 0.012), (0.015, 0.012)])
        .cylinder(0.005, 0.002)
    )
    slider_solid = slider_solid.cut(notches)

    slider = model.part("slider")
    slider.visual(
        mesh_from_cadquery(slider_solid, "slider_mesh"),
        name="slider_body",
    )

    model.articulation(
        "slider_track",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.015, upper=0.015),
    )

    # Center detent that clicks into the slider notches
    detent = model.part("detent")
    detent.visual(
        Box((0.003, 0.002, 0.003)),
        origin=Origin(xyz=(0.0, -0.001, 0.0)),
        name="detent_arm",
    )
    detent.visual(
        Cylinder(radius=0.0015, length=0.004),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        name="detent_roller",
    )
    
    model.articulation(
        "click_detent",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=detent,
        origin=Origin(xyz=(0.0, 0.015, 0.01225)),
        axis=(0.0, 0.0, 1.0),
        # Limits allow the detent to swing both ways when riding on the flat slider edge
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-1.2, upper=1.2),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    housing = object_model.get_part("housing")
    slider = object_model.get_part("slider")
    detent = object_model.get_part("detent")
    
    ctx.allow_overlap(
        detent,
        housing,
        elem_a="detent_arm",
        elem_b="detent_pin",
        reason="The detent arm is mounted on the housing pin.",
    )
    ctx.allow_overlap(
        slider,
        detent,
        elem_a="slider_body",
        elem_b="detent_roller",
        reason="At the default pose, the spring-loaded detent roller rests against the flat side of the slider, causing intentional overlap until it rotates.",
    )
    ctx.allow_isolated_part(
        slider,
        reason="The slider has intentional sliding clearance in the track.",
    )
    
    ctx.expect_within(slider, housing, axes="y", margin=0.005)
    ctx.expect_overlap(slider, housing, axes="z", min_overlap=0.001)

    # Check left click engagement
    with ctx.pose(slider_track=-0.015, click_detent=0.0):
        ctx.expect_overlap(slider, detent, axes="xy", min_overlap=0.001, name="detent engages left notch")

    # Check right click engagement
    with ctx.pose(slider_track=0.015, click_detent=0.0):
        ctx.expect_overlap(slider, detent, axes="xy", min_overlap=0.001, name="detent engages right notch")

    # Check detent pushed out pose
    with ctx.pose(slider_track=0.0, click_detent=1.05):
        # When riding the flat edge, it should not penetrate deeply
        ctx.expect_gap(detent, slider, axis="y", positive_elem="detent_roller", negative_elem="slider_body", max_penetration=0.001, name="detent rides flat edge")

    return ctx.report()

object_model = build_object_model()
