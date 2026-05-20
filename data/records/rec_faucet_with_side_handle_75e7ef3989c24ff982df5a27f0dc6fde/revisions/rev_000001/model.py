from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_TOP_Z = 0.147
BUTTON_TRAVEL = 0.008
PLUNGER_STEM_RADIUS = 0.010


def _cylinder_between_x(x0: float, length: float, y: float, z: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").add(
        cq.Solid.makeCylinder(
            radius,
            length,
            cq.Vector(x0, y, z),
            cq.Vector(1.0, 0.0, 0.0),
        )
    )


def _cylinder_between_z(z0: float, length: float, x: float, y: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").add(
        cq.Solid.makeCylinder(
            radius,
            length,
            cq.Vector(x, y, z0),
            cq.Vector(0.0, 0.0, 1.0),
        )
    )


def _faucet_body_geometry() -> cq.Workplane:
    deck = cq.Workplane("XY").ellipse(0.095, 0.052).extrude(0.012)

    pedestal = cq.Workplane("XY").circle(0.037).extrude(BODY_TOP_Z - 0.012).translate(
        (0.0, 0.0, 0.012)
    )
    collar = cq.Workplane("XY").circle(0.026).extrude(0.006).translate(
        (0.0, 0.0, BODY_TOP_Z)
    )

    spout = _cylinder_between_x(
        x0=0.018,
        length=0.118,
        y=0.0,
        z=0.119,
        radius=0.016,
    )
    outlet_drop = _cylinder_between_z(
        z0=0.084,
        length=0.023,
        x=0.128,
        y=0.0,
        radius=0.010,
    )

    body = deck.union(pedestal).union(collar).union(spout).union(outlet_drop)

    plunger_bore = _cylinder_between_z(
        z0=BODY_TOP_Z - 0.060,
        length=0.073,
        x=0.0,
        y=0.0,
        radius=0.0130,
    )
    body = body.cut(plunger_bore)

    # Soft fillets make the chrome casting read as a plated washroom faucet
    # rather than a stack of raw primitives.  Fillet the main casting before
    # adding the tiny internal guide rails.
    body = body.edges("|Z").fillet(0.0012)

    # Four internal guide rails create a real sliding bearing surface for the
    # plunger. They are tucked inside the top bore, touch the stem tangentially,
    # and overlap the bore wall just enough to be one cast/inserted chrome body.
    rail_radius = 0.00155
    rail_center = PLUNGER_STEM_RADIUS + rail_radius - 0.00000005
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        rail = _cylinder_between_z(
            z0=BODY_TOP_Z - 0.050,
            length=0.056,
            x=rail_center * math.cos(angle),
            y=rail_center * math.sin(angle),
            radius=rail_radius,
        )
        body = body.union(rail)

    return body

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_closing_push_button_faucet")

    chrome = model.material("polished_chrome", color=(0.82, 0.84, 0.82, 1.0))
    dark_insert = model.material("dark_aerator_screen", color=(0.03, 0.035, 0.04, 1.0))

    faucet = model.part(
        "faucet",
        meta={
            "description": "One-piece chrome deck plate, round pedestal body, short straight spout, guide collar, and internal plunger bore."
        },
    )
    faucet.visual(
        mesh_from_cadquery(_faucet_body_geometry(), "faucet_chrome_body", tolerance=0.0008),
        material=chrome,
        name="chrome_body",
    )
    faucet.visual(
        Cylinder(radius=0.0075, length=0.003),
        origin=Origin(xyz=(0.128, 0.0, 0.0825)),
        material=dark_insert,
        name="aerator_screen",
    )
    button = model.part("button")
    button.visual(
        Cylinder(radius=PLUNGER_STEM_RADIUS, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=chrome,
        name="plunger_stem",
    )
    button.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.065,
                0.026,
                body_style="mushroom",
                base_diameter=0.050,
                top_diameter=0.061,
                crown_radius=0.004,
                edge_radius=0.0015,
                center=False,
            ),
            "mushroom_button_head",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=chrome,
        name="mushroom_head",
    )

    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=faucet,
        child=button,
        # The child frame sits exactly on the round body's top face.  Positive
        # travel is a downward press; q=0 is the spring-returned closed state.
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.12, lower=0.0, upper=BUTTON_TRAVEL),
        motion_properties=MotionProperties(damping=0.9, friction=0.05),
        meta={
            "spring_return_to_closed": True,
            "timed_release_seconds": 6.0,
            "closed_position": 0.0,
            "user_action": "press_down_then_release",
        },
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    faucet = object_model.get_part("faucet")
    button = object_model.get_part("button")
    joint = object_model.get_articulation("body_to_button")
    limits = joint.motion_limits

    ctx.check(
        "button uses downward prismatic press",
        joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(joint.axis) == (0.0, 0.0, -1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper == BUTTON_TRAVEL,
        details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
    )
    ctx.check(
        "joint records spring timed release",
        joint.meta.get("spring_return_to_closed") is True
        and joint.meta.get("closed_position") == 0.0
        and 3.0 <= float(joint.meta.get("timed_release_seconds", 0.0)) <= 12.0,
        details=f"meta={joint.meta}",
    )

    ctx.expect_within(
        button,
        faucet,
        axes="xy",
        inner_elem="plunger_stem",
        outer_elem="chrome_body",
        margin=0.001,
        name="plunger is centered in the body bore footprint",
    )
    ctx.expect_overlap(
        button,
        faucet,
        axes="z",
        elem_a="plunger_stem",
        elem_b="chrome_body",
        min_overlap=0.018,
        name="plunger remains inserted in the faucet body at rest",
    )
    ctx.expect_gap(
        button,
        faucet,
        axis="z",
        positive_elem="mushroom_head",
        negative_elem="chrome_body",
        min_gap=0.006,
        max_gap=0.012,
        name="mushroom head sits above the guide collar at rest",
    )

    rest_pos = ctx.part_world_position(button)
    with ctx.pose({joint: BUTTON_TRAVEL}):
        pressed_pos = ctx.part_world_position(button)
        ctx.expect_overlap(
            button,
            faucet,
            axes="z",
            elem_a="plunger_stem",
            elem_b="chrome_body",
            min_overlap=0.026,
            name="pressed plunger stays retained in the bore",
        )
        ctx.expect_gap(
            button,
            faucet,
            axis="z",
            positive_elem="mushroom_head",
            negative_elem="chrome_body",
            min_gap=0.0,
            max_gap=0.002,
            name="pressed mushroom head bottoms on the guide collar",
        )

    ctx.check(
        "upper travel moves the button downward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.006,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
