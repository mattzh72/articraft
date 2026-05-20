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


OUTER_WIDTH = 0.54
OUTER_HEIGHT = 0.74
FRAME_DEPTH = 0.12
OPENING_WIDTH = 0.39
OPENING_BOTTOM = 0.095
OPENING_HEIGHT = 0.51
HINGE_Z = 0.585

DOOR_WIDTH = 0.320
DOOR_HEIGHT = 0.450
DOOR_THICKNESS = 0.014


def _frame_shell_mesh():
    """One continuous thick plastic frame with a through-opening."""

    outer = cq.Workplane("XY").box(
        FRAME_DEPTH,
        OUTER_WIDTH,
        OUTER_HEIGHT,
        centered=(True, True, False),
    )
    opening = (
        cq.Workplane("XY")
        .box(
            FRAME_DEPTH + 0.03,
            OPENING_WIDTH,
            OPENING_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, OPENING_BOTTOM))
    )
    frame = outer.cut(opening)

    # Rounded molded outside edges and the softer inner return around the flap.
    frame = frame.edges("|Z").fillet(0.012)
    frame = frame.edges(">X").fillet(0.004)
    return mesh_from_cadquery(frame, "insulated_outer_frame")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="large_insulated_pet_flap")

    warm_plastic = Material("warm_insulated_plastic", rgba=(0.82, 0.76, 0.62, 1.0))
    dark_rubber = Material("black_flexible_seal", rgba=(0.015, 0.014, 0.012, 1.0))
    smoky_clear = Material("smoky_clear_polycarbonate", rgba=(0.55, 0.78, 0.92, 0.36))
    amber_edge = Material("translucent_amber_edge", rgba=(0.95, 0.70, 0.32, 0.62))
    steel = Material("brushed_stainless_pin", rgba=(0.72, 0.72, 0.68, 1.0))
    shadow = Material("dark_recess_shadow", rgba=(0.04, 0.04, 0.04, 1.0))

    frame = model.part("frame")
    frame.visual(
        _frame_shell_mesh(),
        material=warm_plastic,
        name="frame_shell",
    )

    # A proud hood at the top of the opening hides the pivot line and gives the
    # large flap the bulky insulated appearance of a weather-sealed pet door.
    frame.visual(
        Box((0.075, OPENING_WIDTH + 0.080, 0.080)),
        origin=Origin(xyz=(0.077, 0.0, HINGE_Z + 0.030)),
        material=warm_plastic,
        name="front_hood",
    )
    frame.visual(
        Box((0.016, OPENING_WIDTH + 0.035, 0.035)),
        origin=Origin(xyz=(0.068, 0.0, OPENING_BOTTOM + OPENING_HEIGHT - 0.018)),
        material=dark_rubber,
        name="top_weather_strip",
    )
    frame.visual(
        Box((0.014, 0.020, OPENING_HEIGHT - 0.025)),
        origin=Origin(xyz=(0.068, -OPENING_WIDTH / 2 + 0.010, OPENING_BOTTOM + OPENING_HEIGHT / 2 - 0.004)),
        material=dark_rubber,
        name="side_seal_0",
    )
    frame.visual(
        Box((0.014, 0.020, OPENING_HEIGHT - 0.025)),
        origin=Origin(xyz=(0.068, OPENING_WIDTH / 2 - 0.010, OPENING_BOTTOM + OPENING_HEIGHT / 2 - 0.004)),
        material=dark_rubber,
        name="side_seal_1",
    )
    frame.visual(
        Box((0.014, OPENING_WIDTH - 0.015, 0.020)),
        origin=Origin(xyz=(0.068, 0.0, OPENING_BOTTOM + 0.012)),
        material=dark_rubber,
        name="bottom_sill_seal",
    )

    # Two short bushings project from the hood/side frame into the clear opening.
    # They are deliberately solid proxy sockets for the flap's captured pins.
    frame.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, -0.185, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow,
        name="pivot_socket_0",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, 0.185, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow,
        name="pivot_socket_1",
    )

    flap = model.part("flap")
    flap.visual(
        Box((DOOR_THICKNESS, DOOR_WIDTH - 0.035, DOOR_HEIGHT - 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.025 - (DOOR_HEIGHT - 0.080) / 2.0)),
        material=smoky_clear,
        name="clear_panel",
    )
    flap.visual(
        Box((0.024, DOOR_WIDTH, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=amber_edge,
        name="top_rail",
    )
    flap.visual(
        Box((0.024, DOOR_WIDTH, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -DOOR_HEIGHT + 0.016)),
        material=amber_edge,
        name="bottom_rail",
    )
    flap.visual(
        Box((0.024, 0.025, DOOR_HEIGHT - 0.040)),
        origin=Origin(xyz=(0.0, -DOOR_WIDTH / 2.0 + 0.0125, -DOOR_HEIGHT / 2.0 - 0.020)),
        material=amber_edge,
        name="side_rail_0",
    )
    flap.visual(
        Box((0.024, 0.025, DOOR_HEIGHT - 0.040)),
        origin=Origin(xyz=(0.0, DOOR_WIDTH / 2.0 - 0.0125, -DOOR_HEIGHT / 2.0 - 0.020)),
        material=amber_edge,
        name="side_rail_1",
    )
    flap.visual(
        Box((0.026, 0.040, 0.046)),
        origin=Origin(xyz=(0.0, -DOOR_WIDTH / 2.0 + 0.018, -0.013)),
        material=amber_edge,
        name="pivot_tab_0",
    )
    flap.visual(
        Box((0.026, 0.040, 0.046)),
        origin=Origin(xyz=(0.0, DOOR_WIDTH / 2.0 - 0.018, -0.013)),
        material=amber_edge,
        name="pivot_tab_1",
    )
    flap.visual(
        Cylinder(radius=0.009, length=0.038),
        origin=Origin(xyz=(0.0, -0.174, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin_0",
    )
    flap.visual(
        Cylinder(radius=0.009, length=0.038),
        origin=Origin(xyz=(0.0, 0.174, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin_1",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-math.pi / 4.0,
            upper=math.pi / 4.0,
            effort=8.0,
            velocity=2.5,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("frame_to_flap")

    ctx.allow_overlap(
        frame,
        flap,
        elem_a="pivot_socket_0",
        elem_b="pivot_pin_0",
        reason="The left pivot pin is intentionally captured inside the solid proxy bushing in the frame hood.",
    )
    ctx.allow_overlap(
        frame,
        flap,
        elem_a="pivot_socket_1",
        elem_b="pivot_pin_1",
        reason="The right pivot pin is intentionally captured inside the solid proxy bushing in the frame hood.",
    )

    ctx.expect_within(
        flap,
        frame,
        axes="yz",
        inner_elem="clear_panel",
        outer_elem="frame_shell",
        margin=0.0,
        name="clear panel sits within the large outer frame envelope",
    )
    ctx.expect_overlap(
        frame,
        flap,
        axes="y",
        elem_a="pivot_socket_0",
        elem_b="pivot_pin_0",
        min_overlap=0.015,
        name="left pivot pin remains seated in its hood socket",
    )
    ctx.expect_overlap(
        frame,
        flap,
        axes="y",
        elem_a="pivot_socket_1",
        elem_b="pivot_pin_1",
        min_overlap=0.015,
        name="right pivot pin remains seated in its hood socket",
    )

    lower = hinge.motion_limits.lower
    upper = hinge.motion_limits.upper
    ctx.check(
        "hinge has about ninety degrees total swing",
        lower is not None and upper is not None and abs((upper - lower) - math.pi / 2.0) < 0.02,
        details=f"lower={lower}, upper={upper}",
    )

    with ctx.pose({hinge: 0.0}):
        panel_aabb = ctx.part_element_world_aabb(flap, elem="clear_panel")
        ctx.check(
            "closed clear panel fits through the frame opening",
            panel_aabb is not None
            and panel_aabb[0][1] > -OPENING_WIDTH / 2.0
            and panel_aabb[1][1] < OPENING_WIDTH / 2.0
            and panel_aabb[0][2] > OPENING_BOTTOM
            and panel_aabb[1][2] < OPENING_BOTTOM + OPENING_HEIGHT,
            details=f"panel_aabb={panel_aabb}",
        )
        rest_pos = ctx.part_world_position(flap)

    with ctx.pose({hinge: math.pi / 4.0}):
        swung_pos = ctx.part_world_position(flap)
        swung_aabb = ctx.part_element_world_aabb(flap, elem="bottom_rail")

    ctx.check(
        "positive swing moves the lower flap edge outward",
        rest_pos is not None
        and swung_pos is not None
        and swung_aabb is not None
        and swung_aabb[1][0] > 0.25,
        details=f"rest={rest_pos}, swung={swung_pos}, bottom_aabb={swung_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
