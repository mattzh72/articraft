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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


HANDLE_LENGTH = 0.165
HANDLE_WIDTH = 0.032
HANDLE_HEIGHT = 0.026
CHANNEL_WIDTH = 0.014
CHANNEL_HEIGHT = 0.012
TOP_SLOT_LENGTH = 0.082
TOP_SLOT_WIDTH = 0.008
BLADE_LOWER = -0.020
BLADE_UPPER = 0.025


def _make_handle_shell() -> cq.Workplane:
    """Metal outer knife body with a real central tunnel and top slider slot."""
    body = cq.Workplane("XY").box(HANDLE_LENGTH, HANDLE_WIDTH, HANDLE_HEIGHT)
    body = body.edges("|X").fillet(0.0045)
    # Chamfer the front and rear end plates so the metal body reads like a
    # stamped utility-knife handle rather than a plain rectangular brick.
    body = body.faces(">X").edges().chamfer(0.0025)
    body = body.faces("<X").edges().chamfer(0.0018)

    channel = (
        cq.Workplane("XY")
        .box(HANDLE_LENGTH + 0.012, CHANNEL_WIDTH, CHANNEL_HEIGHT)
        .translate((0.0, 0.0, -0.001))
    )
    body = body.cut(channel)

    top_slot = (
        cq.Workplane("XY")
        .box(TOP_SLOT_LENGTH, TOP_SLOT_WIDTH, HANDLE_HEIGHT + 0.006)
        .translate((-0.006, 0.0, HANDLE_HEIGHT * 0.5))
    )
    body = body.cut(top_slot)

    # A small side-wall bushing hole for the lock wheel's axle.
    axle_hole = (
        cq.Workplane("XZ")
        .center(0.010, 0.003)
        .circle(0.0034)
        .extrude(HANDLE_WIDTH + 0.018, both=True)
    )
    body = body.cut(axle_hole)

    return body


def _make_blade_sheet() -> cq.Workplane:
    """Thin trapezoidal utility blade held by the sliding carrier."""
    profile = [
        (0.045, -0.0050),
        (0.045, 0.0048),
        (0.079, 0.0048),
        (0.098, -0.0028),
        (0.045, -0.0050),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.0014, both=True)


def _make_slider_cap() -> cq.Workplane:
    """Thumb slider cap, stem, and raised grip bars as one connected part."""
    cap = cq.Workplane("XY").box(0.024, 0.014, 0.006).translate((0.0, 0.0, 0.0175))
    stem = cq.Workplane("XY").box(0.012, 0.006, 0.0165).translate((0.0, 0.0, 0.00625))
    slider = cap.union(stem)

    for x in (-0.008, -0.004, 0.0, 0.004, 0.008):
        rib = cq.Workplane("XY").box(0.0015, 0.012, 0.0018).translate((x, 0.0, 0.0214))
        slider = slider.union(rib)
    return slider


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metal_retractable_utility_knife")

    brushed_metal = model.material("brushed_metal", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.09, 0.095, 0.10, 1.0))
    blade_steel = model.material("sharpened_steel", rgba=(0.82, 0.84, 0.80, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))

    handle_shell = model.part("handle_shell")
    handle_shell.visual(
        mesh_from_cadquery(_make_handle_shell(), "handle_shell"),
        material=brushed_metal,
        name="handle_shell",
    )
    # Flush side rivets on the same rigid shell make the metal body read as a
    # assembled stamped handle. They sit on the wall and are not articulated.
    for index, x in enumerate((-0.055, 0.055)):
        handle_shell.visual(
            Cylinder(radius=0.0043, length=0.0018),
            origin=Origin(xyz=(x, HANDLE_WIDTH * 0.5 + 0.0009, -0.002), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"side_rivet_{index}",
        )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.122, 0.009, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, -0.0045)),
        material=dark_metal,
        name="carrier_rail",
    )
    blade_carrier.visual(
        mesh_from_cadquery(_make_blade_sheet(), "utility_blade"),
        material=blade_steel,
        name="blade",
    )
    blade_carrier.visual(
        mesh_from_cadquery(_make_slider_cap(), "thumb_slider"),
        material=black_plastic,
        name="thumb_slider",
    )

    lock_wheel = model.part("lock_wheel")
    lock_wheel.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.017,
                0.006,
                body_style="cylindrical",
                edge_radius=0.0007,
                grip=KnobGrip(style="ribbed", count=18, depth=0.00065, width=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.00035),
            ),
            "lock_wheel_disk",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="wheel_disk",
    )
    lock_wheel.visual(
        Cylinder(radius=0.0025, length=0.014),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="wheel_axle",
    )

    model.articulation(
        "shell_to_carrier",
        ArticulationType.PRISMATIC,
        parent=handle_shell,
        child=blade_carrier,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=BLADE_LOWER, upper=BLADE_UPPER, effort=30.0, velocity=0.18),
    )

    model.articulation(
        "shell_to_wheel",
        ArticulationType.REVOLUTE,
        parent=handle_shell,
        child=lock_wheel,
        origin=Origin(xyz=(0.010, HANDLE_WIDTH * 0.5 + 0.0030, 0.003)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=1.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle_shell")
    carrier = object_model.get_part("blade_carrier")
    wheel = object_model.get_part("lock_wheel")
    slide_joint = object_model.get_articulation("shell_to_carrier")
    wheel_joint = object_model.get_articulation("shell_to_wheel")

    with ctx.pose({slide_joint: BLADE_LOWER}):
        retracted_pos = ctx.part_world_position(carrier)
        ctx.expect_within(
            carrier,
            handle,
            axes="yz",
            inner_elem="carrier_rail",
            outer_elem="handle_shell",
            margin=0.0,
            name="carrier rail stays inside channel cross section when retracted",
        )
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="carrier_rail",
            elem_b="handle_shell",
            min_overlap=0.09,
            name="carrier remains clipped in handle when retracted",
        )

    with ctx.pose({slide_joint: BLADE_UPPER}):
        extended_pos = ctx.part_world_position(carrier)
        ctx.expect_within(
            carrier,
            handle,
            axes="yz",
            inner_elem="carrier_rail",
            outer_elem="handle_shell",
            margin=0.0,
            name="carrier rail stays inside channel cross section when extended",
        )
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="carrier_rail",
            elem_b="handle_shell",
            min_overlap=0.075,
            name="extended carrier still has retained insertion",
        )

    ctx.check(
        "blade carrier slides forward along handle axis",
        retracted_pos is not None and extended_pos is not None and extended_pos[0] > retracted_pos[0] + 0.035,
        details=f"retracted={retracted_pos}, extended={extended_pos}",
    )

    ctx.expect_contact(
        wheel,
        handle,
        elem_a="wheel_disk",
        elem_b="handle_shell",
        contact_tol=0.0015,
        name="lock wheel is seated against side wall",
    )

    with ctx.pose({wheel_joint: 1.2}):
        ctx.expect_contact(
            wheel,
            handle,
            elem_a="wheel_disk",
            elem_b="handle_shell",
            contact_tol=0.0015,
            name="lock wheel remains mounted while rotated",
        )

    return ctx.report()


object_model = build_object_model()
