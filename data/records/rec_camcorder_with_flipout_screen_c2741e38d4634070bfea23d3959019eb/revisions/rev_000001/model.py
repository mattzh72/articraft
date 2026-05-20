from __future__ import annotations

import cadquery as cq
import math

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


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def _rounded_panel(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    # A thin electronics panel: fillet only the long through-thickness edges so
    # the visible rectangle has rounded X/Z corners while the faces stay flat.
    return cq.Workplane("XY").box(*size).edges("|Y").fillet(radius)


def _tube_along_x(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_camcorder")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = model.material("charcoal_body", rgba=(0.075, 0.078, 0.085, 1.0))
    rubber = model.material("ribbed_rubber", rgba=(0.006, 0.006, 0.007, 1.0))
    dark_glass = model.material("dark_lens_glass", rgba=(0.02, 0.045, 0.075, 0.82))
    screen_glass = model.material("blue_screen_glass", rgba=(0.04, 0.10, 0.16, 0.90))
    hatch_plastic = model.material("hatch_plastic", rgba=(0.038, 0.041, 0.047, 1.0))

    body = model.part("body")
    body_shell = _rounded_box((0.150, 0.050, 0.070), 0.006).union(
        _rounded_box((0.110, 0.024, 0.064), 0.007).translate((-0.008, 0.036, -0.002))
    )
    body_shell = body_shell.union(
        _rounded_box((0.062, 0.026, 0.009), 0.003).translate((-0.018, 0.000, 0.039))
    )
    body.visual(
        mesh_from_cadquery(body_shell, "body_shell", tolerance=0.0008),
        material=charcoal,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_rounded_panel((0.030, 0.006, 0.020), 0.002), "rear_eyepiece"),
        origin=Origin(xyz=(-0.078, 0.000, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="rear_eyepiece",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.083, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lens_mount",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.075),
        origin=Origin(xyz=(0.112, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lens_barrel",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(0.150, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="front_rim",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.002),
        origin=Origin(xyz=(0.157, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_glass,
        name="lens_glass",
    )

    # Fixed receiver knuckles for the flip-out screen's vertical hinge.
    screen_hinge_x = -0.060
    screen_hinge_y = -0.030
    for idx, zc in enumerate((-0.022, 0.022)):
        body.visual(
            Box((0.012, 0.005, 0.020)),
            origin=Origin(xyz=(screen_hinge_x, -0.0265, zc)),
            material=charcoal,
            name=f"screen_leaf_{idx}",
        )
        body.visual(
            Cylinder(radius=0.0040, length=0.020),
            origin=Origin(xyz=(screen_hinge_x, screen_hinge_y, zc)),
            material=matte_black,
            name=f"screen_knuckle_{idx}",
        )
    body.visual(
        Cylinder(radius=0.0015, length=0.058),
        origin=Origin(xyz=(screen_hinge_x, screen_hinge_y, 0.000)),
        material=matte_black,
        name="screen_pin",
    )

    # Small side lugs around the handgrip card hatch hinge line.
    for idx, x in enumerate((-0.023, 0.023)):
        body.visual(
            Box((0.005, 0.006, 0.006)),
            origin=Origin(xyz=(x, 0.0472, -0.030)),
            material=charcoal,
            name=f"card_lug_{idx}",
        )
    body.visual(
        Cylinder(radius=0.0012, length=0.050),
        origin=Origin(xyz=(0.000, 0.0505, -0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="card_pin",
    )

    focus_ring = model.part("focus_ring")
    ring_length = 0.024
    ring_outer = 0.027
    focus_ring.visual(
        mesh_from_cadquery(
            _tube_along_x(ring_length, ring_outer, 0.0226),
            "focus_ring_body",
            tolerance=0.00045,
        ),
        material=rubber,
        name="focus_ring_body",
    )
    for idx in range(18):
        a = 2.0 * math.pi * idx / 18.0
        focus_ring.visual(
            Box((ring_length * 0.96, 0.0026, 0.0040)),
            origin=Origin(
                xyz=(0.0, (ring_outer + 0.0018) * math.cos(a), (ring_outer + 0.0018) * math.sin(a)),
                rpy=(a - math.pi / 2.0, 0.0, 0.0),
            ),
            material=rubber,
            name=f"grip_rib_{idx}",
        )

    screen = model.part("screen")
    screen.visual(
        mesh_from_cadquery(_rounded_panel((0.095, 0.006, 0.055), 0.006), "screen_frame"),
        origin=Origin(xyz=(0.0525, -0.0040, 0.0000)),
        material=matte_black,
        name="screen_frame",
    )
    screen.visual(
        mesh_from_cadquery(_rounded_panel((0.076, 0.001, 0.040), 0.004), "screen_display"),
        origin=Origin(xyz=(0.0560, -0.0074, 0.0000)),
        material=screen_glass,
        name="display",
    )
    screen.visual(
        Cylinder(radius=0.0038, length=0.020),
        origin=Origin(),
        material=matte_black,
        name="screen_knuckle",
    )
    screen.visual(
        Box((0.010, 0.005, 0.020)),
        origin=Origin(xyz=(0.005, -0.0020, 0.0000)),
        material=matte_black,
        name="screen_bridge",
    )

    card_hatch = model.part("card_hatch")
    card_hatch.visual(
        mesh_from_cadquery(_rounded_panel((0.040, 0.003, 0.028), 0.003), "card_hatch_panel"),
        origin=Origin(xyz=(0.0, 0.0020, 0.0155)),
        material=hatch_plastic,
        name="hatch_panel",
    )
    card_hatch.visual(
        Cylinder(radius=0.0020, length=0.040),
        origin=Origin(xyz=(0.0, 0.0020, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="hatch_knuckle",
    )

    model.articulation(
        "focus_ring_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.106, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )
    model.articulation(
        "screen_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen,
        origin=Origin(xyz=(screen_hinge_x, screen_hinge_y, 0.000)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=0.6, velocity=2.0),
    )
    model.articulation(
        "card_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=card_hatch,
        origin=Origin(xyz=(0.000, 0.0485, -0.030)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=0.25, velocity=1.6),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    screen = object_model.get_part("screen")
    focus_ring = object_model.get_part("focus_ring")
    card_hatch = object_model.get_part("card_hatch")
    screen_hinge = object_model.get_articulation("screen_hinge")
    hatch_hinge = object_model.get_articulation("card_hatch_hinge")
    focus_spin = object_model.get_articulation("focus_ring_spin")

    ctx.allow_overlap(
        body,
        focus_ring,
        elem_a="lens_barrel",
        elem_b="focus_ring_body",
        reason="The focus ring is intentionally captured on a simplified lens-barrel bearing surface.",
    )
    ctx.allow_overlap(
        body,
        screen,
        elem_a="screen_pin",
        elem_b="screen_knuckle",
        reason="The screen hinge pin passes through the rotating center knuckle.",
    )
    ctx.allow_overlap(
        body,
        card_hatch,
        elem_a="card_pin",
        elem_b="hatch_knuckle",
        reason="The card-hatch hinge pin is intentionally captured inside the hatch knuckle.",
    )

    ctx.check(
        "focus ring uses continuous rotation",
        str(focus_spin.articulation_type).lower().endswith("continuous"),
        details=f"type={focus_spin.articulation_type}",
    )
    ctx.expect_overlap(
        focus_ring,
        body,
        axes="x",
        min_overlap=0.018,
        elem_a="focus_ring_body",
        elem_b="lens_barrel",
        name="focus ring sits around the lens barrel",
    )
    ctx.expect_overlap(
        screen,
        body,
        axes="z",
        min_overlap=0.016,
        elem_a="screen_knuckle",
        elem_b="screen_pin",
        name="screen hinge knuckle is pinned",
    )
    ctx.expect_overlap(
        card_hatch,
        body,
        axes="x",
        min_overlap=0.036,
        elem_a="hatch_knuckle",
        elem_b="card_pin",
        name="card hatch knuckle is pinned",
    )
    ctx.expect_gap(
        card_hatch,
        body,
        axis="y",
        min_gap=0.0002,
        max_gap=0.004,
        positive_elem="hatch_panel",
        negative_elem="body_shell",
        name="card hatch is flush on handgrip side",
    )

    closed_screen_aabb = ctx.part_world_aabb(screen)
    with ctx.pose({screen_hinge: 1.45}):
        open_screen_aabb = ctx.part_world_aabb(screen)
        ctx.expect_gap(
            body,
            screen,
            axis="y",
            max_penetration=0.0,
            positive_elem="body_shell",
            negative_elem="screen_frame",
            name="opened screen clears the body side",
        )
    ctx.check(
        "screen opens outward from the flank",
        closed_screen_aabb is not None
        and open_screen_aabb is not None
        and open_screen_aabb[0][1] < closed_screen_aabb[0][1] - 0.030,
        details=f"closed={closed_screen_aabb}, open={open_screen_aabb}",
    )

    closed_hatch_aabb = ctx.part_world_aabb(card_hatch)
    with ctx.pose({hatch_hinge: 1.0}):
        open_hatch_aabb = ctx.part_world_aabb(card_hatch)
    ctx.check(
        "card hatch swings outward on its bottom edge",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][1] > closed_hatch_aabb[1][1] + 0.015,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
