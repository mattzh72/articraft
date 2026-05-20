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


HANDLE_LENGTH = 0.166
HANDLE_WIDTH = 0.034
HANDLE_HEIGHT = 0.024
CARRIAGE_TRAVEL = 0.035


def _handle_body_mesh():
    """One continuous hollow handle shell with a front blade mouth and top slot."""
    half_l = HANDLE_LENGTH / 2.0
    # Side silhouette in the XZ plane: slightly dropped rear, tapered utility-knife nose.
    profile = [
        (-half_l, -0.006),
        (-0.076, -0.0115),
        (0.040, -0.0115),
        (half_l, -0.0055),
        (half_l, 0.0055),
        (0.042, 0.0115),
        (-0.070, 0.0115),
        (-half_l, 0.006),
    ]
    body = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(HANDLE_WIDTH)
        .translate((0.0, HANDLE_WIDTH / 2.0, 0.0))
    )
    body = body.edges("|Y").fillet(0.0015)

    # Central through-channel that actually clears the blade carrier and blade.
    channel = cq.Workplane("XY").box(0.154, 0.0215, 0.014).translate((0.006, 0.0, 0.0))
    # Spine slot for the thumb slider stem.
    top_slot = cq.Workplane("XY").box(0.104, 0.009, 0.032).translate((-0.007, 0.0, 0.015))
    return body.cut(channel).cut(top_slot)


def _blade_mesh():
    """Thin snap-off style blade with an angled cutting tip."""
    blade_profile = [
        (0.026, -0.0045),
        (0.084, -0.0045),
        (0.105, 0.0005),
        (0.086, 0.0060),
        (0.026, 0.0060),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(blade_profile)
        .close()
        .extrude(0.0012)
        .translate((0.0, -0.0006, 0.0))
    )


def _thumb_cap_mesh():
    cap = cq.Workplane("XY").box(0.026, 0.020, 0.006).edges("|Z").fillet(0.003)
    return cap.edges("|Y").fillet(0.001)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_lock_utility_knife")

    yellow = model.material("safety_yellow_plastic", rgba=(0.95, 0.66, 0.08, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    dark = model.material("dark_graphite", rgba=(0.09, 0.095, 0.10, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    edge = model.material("sharpened_edge", rgba=(0.88, 0.90, 0.92, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_body_mesh(), "hollow_handle"),
        material=yellow,
        name="handle_shell",
    )
    # Soft side grip pads, slightly proud of the rigid shell.
    for i, y in enumerate((-0.0177, 0.0177)):
        handle.visual(
            Box((0.096, 0.0020, 0.010)),
            origin=Origin(xyz=(-0.018, y, -0.0015)),
            material=black,
            name=f"side_grip_{i}",
        )
    # Dark liner strip and ratchet teeth along the spine slot, making the snap-lock guide readable.
    handle.visual(
        Box((0.100, 0.0020, 0.0012)),
        origin=Origin(xyz=(-0.006, -0.0062, 0.0120)),
        material=dark,
        name="slot_lock_strip",
    )
    for i, x in enumerate((-0.047, -0.033, -0.019, -0.005, 0.009, 0.023, 0.037)):
        handle.visual(
            Box((0.0025, 0.0040, 0.0014)),
            origin=Origin(xyz=(x, -0.0062, 0.0127)),
            material=dark,
            name=f"lock_tooth_{i}",
        )
    # Small metal screw heads on both scales.
    for side_index, y in enumerate((-0.0174, 0.0174)):
        for screw_index, x in enumerate((-0.056, 0.045)):
            handle.visual(
                Cylinder(radius=0.0042, length=0.0024),
                origin=Origin(xyz=(x, y, -0.0008), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"screw_{side_index}_{screw_index}",
            )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.095, 0.0205, 0.006)),
        origin=Origin(xyz=(-0.018, 0.0, -0.001)),
        material=dark,
        name="carrier_rail",
    )
    blade_carrier.visual(
        Box((0.020, 0.012, 0.008)),
        origin=Origin(xyz=(0.034, 0.0, 0.000)),
        material=steel,
        name="blade_clamp",
    )
    blade_carrier.visual(
        mesh_from_cadquery(_blade_mesh(), "snap_blade"),
        material=steel,
        name="blade_plate",
    )
    # Painted score lines on the snap-off blade; short and embedded on the blade face.
    for i, x in enumerate((0.050, 0.064, 0.078)):
        blade_carrier.visual(
            Box((0.0012, 0.0015, 0.010)),
            origin=Origin(xyz=(x, -0.0011, 0.0007), rpy=(0.0, -0.35, 0.0)),
            material=edge,
            name=f"blade_score_{i}",
        )

    thumb_slider = model.part("thumb_slider")
    thumb_slider.visual(
        Box((0.010, 0.0065, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark,
        name="slider_stem",
    )
    thumb_slider.visual(
        mesh_from_cadquery(_thumb_cap_mesh(), "thumb_slider_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.0164)),
        material=black,
        name="thumb_cap",
    )
    for i, x in enumerate((-0.008, -0.004, 0.0, 0.004, 0.008)):
        thumb_slider.visual(
            Box((0.0016, 0.017, 0.0022)),
            origin=Origin(xyz=(x, 0.0, 0.0200)),
            material=dark,
            name=f"thumb_ridge_{i}",
        )

    model.articulation(
        "handle_to_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=blade_carrier,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=CARRIAGE_TRAVEL),
    )
    model.articulation(
        "carrier_to_slider",
        ArticulationType.FIXED,
        parent=blade_carrier,
        child=thumb_slider,
        origin=Origin(xyz=(-0.026, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    blade_carrier = object_model.get_part("blade_carrier")
    thumb_slider = object_model.get_part("thumb_slider")
    slide = object_model.get_articulation("handle_to_carrier")

    limits = slide.motion_limits
    ctx.check(
        "blade carrier has realistic prismatic travel",
        slide.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 0.030 <= limits.upper <= 0.045,
        details=f"axis={slide.axis}, limits={limits}",
    )

    ctx.expect_contact(
        thumb_slider,
        blade_carrier,
        elem_a="slider_stem",
        elem_b="carrier_rail",
        contact_tol=0.0005,
        name="thumb slider stem is seated on the carrier",
    )
    ctx.expect_gap(
        thumb_slider,
        handle,
        axis="z",
        min_gap=0.001,
        positive_elem="thumb_cap",
        negative_elem="handle_shell",
        name="thumb cap rides above the slotted spine",
    )
    ctx.expect_within(
        thumb_slider,
        handle,
        axes="x",
        inner_elem="slider_stem",
        outer_elem="slot_lock_strip",
        margin=0.004,
        name="slider stem sits within the long guide slot at rest",
    )

    rest_carrier = ctx.part_world_position(blade_carrier)
    rest_slider = ctx.part_world_position(thumb_slider)
    with ctx.pose({slide: CARRIAGE_TRAVEL}):
        ctx.expect_within(
            thumb_slider,
            handle,
            axes="x",
            inner_elem="slider_stem",
            outer_elem="slot_lock_strip",
            margin=0.004,
            name="slider stem remains in the guide slot when advanced",
        )
        extended_carrier = ctx.part_world_position(blade_carrier)
        extended_slider = ctx.part_world_position(thumb_slider)

    ctx.check(
        "blade carrier advances along the handle axis",
        rest_carrier is not None
        and extended_carrier is not None
        and extended_carrier[0] > rest_carrier[0] + 0.030,
        details=f"rest={rest_carrier}, extended={extended_carrier}",
    )
    ctx.check(
        "thumb slider translates with the blade carrier",
        rest_slider is not None
        and extended_slider is not None
        and abs((extended_slider[0] - rest_slider[0]) - CARRIAGE_TRAVEL) < 0.001,
        details=f"rest={rest_slider}, extended={extended_slider}",
    )

    return ctx.report()


object_model = build_object_model()
