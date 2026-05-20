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


def _rounded_plate(width: float, height: float, thickness: float, radius: float):
    """A rounded rectangular slab centered on the CadQuery origin."""
    return (
        cq.Workplane("XY")
        .box(width, height, thickness)
        .edges("|Z")
        .fillet(radius)
    )


def _front_panel_with_button_slots():
    panel = _rounded_plate(0.076, 0.158, 0.003, 0.006)
    button_xs = (-0.027, -0.009, 0.009, 0.027)
    for x in button_xs:
        panel = (
            panel.faces(">Z")
            .workplane(centerOption="CenterOfBoundBox")
            .moveTo(x, 0.030)
            .rect(0.010, 0.006)
            .cutThruAll()
        )
    return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="handheld_digital_multimeter")

    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.016, 0.017, 1.0))
    dark_rubber = model.material("dark_grey_rubber", rgba=(0.07, 0.075, 0.08, 1.0))
    yellow = model.material("safety_yellow_face", rgba=(0.95, 0.66, 0.08, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.11, 0.12, 0.13, 1.0))
    screen_green = model.material("muted_lcd_glass", rgba=(0.55, 0.70, 0.58, 1.0))
    ink = model.material("black_print", rgba=(0.005, 0.006, 0.005, 1.0))
    white = model.material("white_marking", rgba=(0.92, 0.92, 0.86, 1.0))
    metal = model.material("dull_metal", rgba=(0.48, 0.47, 0.42, 1.0))

    body = model.part("body")

    # Handheld case: roughly 10 cm wide, 20 cm tall and 3.2 cm thick, with a
    # continuous rubber bumper wrapped around a recessed colored front panel.
    bumper_shape = _rounded_plate(0.102, 0.196, 0.032, 0.014)
    body.visual(
        mesh_from_cadquery(bumper_shape, "rubber_bumper", tolerance=0.0008),
        origin=Origin(),
        material=rubber,
        name="rubber_bumper",
    )
    body.visual(
        mesh_from_cadquery(_front_panel_with_button_slots(), "front_panel", tolerance=0.0006),
        origin=Origin(xyz=(0.0, -0.002, 0.0175)),
        material=yellow,
        name="front_panel",
    )

    # Display stack with a black bezel, LCD lens, and simple seven-segment-like
    # dark marks to make the upper face read as an actual digital instrument.
    body.visual(
        Box((0.066, 0.034, 0.003)),
        origin=Origin(xyz=(0.0, 0.056, 0.0205)),
        material=charcoal,
        name="display_bezel",
    )
    body.visual(
        Box((0.056, 0.024, 0.0012)),
        origin=Origin(xyz=(0.0, 0.056, 0.0226)),
        material=screen_green,
        name="lcd_window",
    )
    for i, x in enumerate((-0.020, -0.007, 0.007, 0.020)):
        body.visual(
            Box((0.006, 0.016, 0.0007)),
            origin=Origin(xyz=(x, 0.056, 0.0235), rpy=(0.0, 0.0, math.radians(8 if i % 2 else -8))),
            material=ink,
            name=f"lcd_digit_{i}",
        )

    # Printed selector tick marks around the rotary dial.
    for i in range(10):
        angle = math.radians(-135 + i * 30)
        r = 0.036
        x = r * math.sin(angle)
        y = -0.026 + r * math.cos(angle)
        body.visual(
            Box((0.0022, 0.0085, 0.0007)),
            origin=Origin(xyz=(x, y, 0.01885), rpy=(0.0, 0.0, -angle)),
            material=white,
            name=f"selector_mark_{i}",
        )

    # Four lead-input sockets at the lower front edge.
    for i, x in enumerate((-0.030, -0.010, 0.010, 0.030)):
        body.visual(
            Cylinder(radius=0.0048, length=0.0016),
            origin=Origin(xyz=(x, -0.074, 0.0196)),
            material=charcoal,
            name=f"input_socket_{i}",
        )
        body.visual(
            Cylinder(radius=0.0025, length=0.0020),
            origin=Origin(xyz=(x, -0.074, 0.0205)),
            material=ink,
            name=f"input_hole_{i}",
        )

    # Rear hinge bosses for the fold-out tilt stand.  These are on the back face
    # of the body and share a horizontal hinge line across the meter width.
    for x in (-0.036, 0.036):
        body.visual(
            Cylinder(radius=0.0040, length=0.014),
            origin=Origin(xyz=(x, -0.070, -0.0196), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"stand_hinge_lug_{0 if x < 0 else 1}",
        )

    # Large central rotary selector, modeled as the only continuous rotation.
    selector = model.part("selector")
    selector_knob = KnobGeometry(
        0.055,
        0.011,
        body_style="faceted",
        base_diameter=0.058,
        top_diameter=0.047,
        edge_radius=0.0008,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0011, width=0.0018),
        indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        center=False,
    )
    selector.visual(
        mesh_from_geometry(selector_knob, "selector_knob"),
        origin=Origin(),
        material=dark_rubber,
        name="knob_body",
    )
    selector.visual(
        Box((0.004, 0.021, 0.0012)),
        origin=Origin(xyz=(0.0, 0.012, 0.0113)),
        material=white,
        name="pointer_line",
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, -0.026, 0.0190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    # Four independent function buttons immediately under the display.  The
    # caps sit proud of the front panel, with hidden stems dropping through the
    # panel slots.
    button_positions = (-0.027, -0.009, 0.009, 0.027)
    for i, x in enumerate(button_positions):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.014, 0.009, 0.0040)),
            origin=Origin(xyz=(0.0, 0.0, 0.0020)),
            material=charcoal,
            name="cap",
        )
        button.visual(
            Box((0.006, 0.004, 0.0030)),
            origin=Origin(xyz=(0.0, 0.0, -0.0014)),
            material=charcoal,
            name="stem",
        )
        button.visual(
            Box((0.008, 0.0011, 0.0007)),
            origin=Origin(xyz=(0.0, 0.0018, 0.0042)),
            material=white,
            name="top_mark",
        )
        model.articulation(
            f"body_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, 0.030, 0.0190)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.003),
        )

    # Rear folding tilt stand: a connected U-shaped prop with a central hinge
    # barrel.  In the zero pose it lies against the rear; positive motion swings
    # the foot bar outward away from the meter back.
    stand = model.part("rear_stand")
    stand.visual(
        Cylinder(radius=0.0032, length=0.058),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="hinge_barrel",
    )
    for i, x in enumerate((-0.023, 0.023)):
        stand.visual(
            Box((0.006, 0.097, 0.004)),
            origin=Origin(xyz=(x, 0.0475, -0.0035)),
            material=dark_rubber,
            name=f"leg_{i}",
        )
    stand.visual(
        Box((0.058, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.095, -0.0035)),
        material=dark_rubber,
        name="foot_bar",
    )
    stand.visual(
        Box((0.045, 0.004, 0.0012)),
        origin=Origin(xyz=(0.0, 0.099, -0.0061)),
        material=metal,
        name="wear_strip",
    )
    model.articulation(
        "body_to_rear_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, -0.070, -0.0196)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    selector = object_model.get_part("selector")
    stand = object_model.get_part("rear_stand")
    selector_joint = object_model.get_articulation("body_to_selector")
    stand_joint = object_model.get_articulation("body_to_rear_stand")

    ctx.check(
        "selector uses continuous rotation",
        str(selector_joint.articulation_type).endswith("CONTINUOUS"),
        details=str(selector_joint.articulation_type),
    )
    ctx.expect_contact(
        selector,
        body,
        elem_a="knob_body",
        elem_b="front_panel",
        contact_tol=0.001,
        name="selector sits on front panel",
    )

    for i in range(4):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"body_to_button_{i}")
        ctx.expect_contact(
            button,
            body,
            elem_a="cap",
            elem_b="front_panel",
            contact_tol=0.001,
            name=f"button_{i} cap rests in panel",
        )
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: 0.003}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{i} pushes inward",
            rest is not None and pressed is not None and pressed[2] < rest[2] - 0.0025,
            details=f"rest={rest}, pressed={pressed}",
        )

    folded = ctx.part_element_world_aabb(stand, elem="foot_bar")
    with ctx.pose({stand_joint: 1.20}):
        deployed = ctx.part_element_world_aabb(stand, elem="foot_bar")
    ctx.check(
        "rear stand swings away from back",
        folded is not None and deployed is not None and deployed[0][2] < folded[0][2] - 0.045,
        details=f"folded={folded}, deployed={deployed}",
    )

    return ctx.report()


object_model = build_object_model()
