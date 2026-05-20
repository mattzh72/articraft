from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry")

    # 1. Base Frame
    base = model.part("base")
    # Left and right vertical frames
    base.visual(Box((0.1, 1.2, 0.5)), origin=Origin((-0.45, 0.0, 0.25)), name="left_frame")
    base.visual(Box((0.1, 1.2, 0.5)), origin=Origin((0.45, 0.0, 0.25)), name="right_frame")
    # Table bed connecting them
    base.visual(Box((0.8, 1.2, 0.1)), origin=Origin((0.0, 0.0, 0.05)), name="bed")
    # Rails on top of the frames for the bridge to slide on
    base.visual(Box((0.05, 1.2, 0.02)), origin=Origin((-0.45, 0.0, 0.51)), name="left_rail")
    base.visual(Box((0.05, 1.2, 0.02)), origin=Origin((0.45, 0.0, 0.51)), name="right_rail")

    # 2. Bridge (travels along Y axis)
    bridge = model.part("bridge")
    # Left slider wrapping the left rail
    bridge.visual(Box((0.08, 0.2, 0.01)), origin=Origin((-0.45, 0.0, 0.025)), name="left_slider_top")
    bridge.visual(Box((0.015, 0.2, 0.03)), origin=Origin((-0.4825, 0.0, 0.015)), name="left_slider_outer")
    bridge.visual(Box((0.015, 0.2, 0.03)), origin=Origin((-0.4175, 0.0, 0.015)), name="left_slider_inner")
    # Right slider wrapping the right rail
    bridge.visual(Box((0.08, 0.2, 0.01)), origin=Origin((0.45, 0.0, 0.025)), name="right_slider_top")
    bridge.visual(Box((0.015, 0.2, 0.03)), origin=Origin((0.4825, 0.0, 0.015)), name="right_slider_outer")
    bridge.visual(Box((0.015, 0.2, 0.03)), origin=Origin((0.4175, 0.0, 0.015)), name="right_slider_inner")
    # The main beam spanning across
    bridge.visual(Box((1.0, 0.1, 0.2)), origin=Origin((0.0, 0.0, 0.13)), name="beam")

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin((0.0, 0.0, 0.5)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.4, upper=0.4),
    )

    # 3. Carriage (travels along X axis on the bridge)
    carriage = model.part("carriage")
    # Wraps around the beam
    carriage.visual(Box((0.15, 0.05, 0.24)), origin=Origin((0.0, 0.075, 0.0)), name="front_plate")
    carriage.visual(Box((0.15, 0.05, 0.24)), origin=Origin((0.0, -0.075, 0.0)), name="back_plate")
    carriage.visual(Box((0.15, 0.2, 0.05)), origin=Origin((0.0, 0.0, 0.125)), name="top_plate")
    carriage.visual(Box((0.15, 0.2, 0.05)), origin=Origin((0.0, 0.0, -0.125)), name="bottom_plate")

    model.articulation(
        "bridge_to_carriage",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=carriage,
        origin=Origin((0.0, 0.0, 0.13)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.3, upper=0.3),
    )

    # 4. Tool Slide (travels along Z axis on the carriage)
    tool_slide = model.part("tool_slide")
    tool_slide.visual(Box((0.1, 0.05, 0.4)), origin=Origin((0.0, 0.0, 0.0)), name="slide_body")
    tool_slide.visual(Cylinder(radius=0.02, length=0.1), origin=Origin((0.0, 0.0, -0.25)), name="spindle")

    model.articulation(
        "carriage_to_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=tool_slide,
        origin=Origin((0.0, 0.125, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.15, upper=0.15),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    carriage = object_model.get_part("carriage")
    tool_slide = object_model.get_part("tool_slide")

    # Check that bridge rests on base
    ctx.expect_contact(bridge, base, elem_a="left_slider_top", elem_b="left_rail")
    ctx.expect_contact(bridge, base, elem_a="right_slider_top", elem_b="right_rail")

    # Check that carriage wraps the bridge beam
    ctx.expect_contact(carriage, bridge, elem_a="top_plate", elem_b="beam")
    ctx.expect_contact(carriage, bridge, elem_a="bottom_plate", elem_b="beam")
    ctx.expect_contact(carriage, bridge, elem_a="front_plate", elem_b="beam")
    ctx.expect_contact(carriage, bridge, elem_a="back_plate", elem_b="beam")

    # Check that tool slide is mounted to carriage
    ctx.expect_contact(tool_slide, carriage, elem_a="slide_body", elem_b="front_plate")

    # Check tool bit does not hit table bed at lowest extension
    slide_joint = object_model.get_articulation("carriage_to_slide")
    with ctx.pose({slide_joint: -0.15}):
        ctx.expect_gap(tool_slide, base, axis="z", positive_elem="spindle", negative_elem="bed", min_gap=0.05)

    return ctx.report()

object_model = build_object_model()
