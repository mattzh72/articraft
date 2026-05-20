from __future__ import annotations

import math

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


HANDLE_LENGTH = 0.172
HANDLE_WIDTH = 0.026
HANDLE_HEIGHT = 0.020
HANDLE_CENTER_X = -0.004
HANDLE_REAR_X = HANDLE_CENTER_X + HANDLE_LENGTH / 2.0
HINGE_Y = -HANDLE_WIDTH / 2.0 - 0.0012


def _utility_handle_shell() -> cq.Workplane:
    """Rounded knife body with a real inner blade channel and side slider slot."""
    shell = (
        cq.Workplane("XY")
        .box(HANDLE_LENGTH, HANDLE_WIDTH, HANDLE_HEIGHT)
        .translate((HANDLE_CENTER_X, 0.0, 0.0))
        .edges("|X")
        .fillet(0.0038)
    )

    channel = (
        cq.Workplane("XY")
        .box(0.158, 0.016, 0.012)
        # Open through the nose, stopped before the rear spare-blade cap.
        .translate((-0.020, 0.0, 0.0))
    )
    side_slot = (
        cq.Workplane("XY")
        .box(0.096, 0.017, 0.010)
        # Cut from the internal channel through the +Y handle side.
        .translate((-0.020, HANDLE_WIDTH / 2.0 + 0.0005, 0.004))
    )
    nose_mouth = (
        cq.Workplane("XY")
        .box(0.018, 0.020, 0.014)
        .translate((-0.090, 0.0, 0.0))
    )

    return shell.cut(channel).cut(side_slot).cut(nose_mouth)


def _utility_blade() -> cq.Workplane:
    """Flat trapezoidal utility blade in the carrier local frame."""
    blade_outline = [
        (-0.114, -0.0048),
        (-0.100, 0.0060),
        (-0.064, 0.0060),
        (-0.054, 0.0020),
        (-0.061, -0.0048),
    ]
    return cq.Workplane("XZ").polyline(blade_outline).close().extrude(0.0012, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_slider_utility_knife")

    yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.06, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    dark = model.material("dark_molded_plastic", rgba=(0.055, 0.060, 0.065, 1.0))
    steel = model.material("brushed_steel", rgba=(0.74, 0.76, 0.76, 1.0))
    shadow = model.material("slot_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_utility_handle_shell(), "handle_shell", tolerance=0.0006),
        material=yellow,
        name="handle_shell",
    )
    handle.visual(
        Box((0.070, 0.0012, 0.010)),
        origin=Origin(xyz=(-0.025, HANDLE_WIDTH / 2.0 + 0.0003, -0.004)),
        material=black,
        name="side_grip",
    )
    handle.visual(
        Box((0.096, 0.0016, 0.0018)),
        origin=Origin(xyz=(-0.020, HANDLE_WIDTH / 2.0 - 0.0004, 0.0095)),
        material=shadow,
        name="slot_upper_lip",
    )
    handle.visual(
        Box((0.096, 0.0009, 0.0012)),
        origin=Origin(xyz=(-0.020, HANDLE_WIDTH / 2.0 + 0.00025, -0.0022)),
        material=shadow,
        name="slot_lower_lip",
    )
    for idx, x in enumerate((-0.055, -0.020, 0.016, 0.052)):
        handle.visual(
            Cylinder(radius=0.0022, length=0.0015),
            origin=Origin(xyz=(x, HANDLE_WIDTH / 2.0 + 0.00045, -0.001), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"screw_{idx}",
        )
    # Rear vertical hinge hardware.  The pin intentionally runs through the cap
    # knuckle so the end cap stays visibly clipped to the knife body.
    handle.visual(
        Cylinder(radius=0.0025, length=0.0050),
        origin=Origin(xyz=(HANDLE_REAR_X, HINGE_Y, 0.0072)),
        material=dark,
        name="hinge_knuckle_0",
    )
    handle.visual(
        Cylinder(radius=0.0025, length=0.0050),
        origin=Origin(xyz=(HANDLE_REAR_X, HINGE_Y, -0.0072)),
        material=dark,
        name="hinge_knuckle_1",
    )
    handle.visual(
        Cylinder(radius=0.00085, length=0.0220),
        origin=Origin(xyz=(HANDLE_REAR_X, HINGE_Y, 0.0)),
        material=steel,
        name="hinge_pin",
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.105, 0.009, 0.005)),
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
        material=dark,
        name="carrier_rail",
    )
    blade_carrier.visual(
        mesh_from_cadquery(_utility_blade(), "utility_blade", tolerance=0.0004),
        material=steel,
        name="blade",
    )
    blade_carrier.visual(
        Box((0.030, 0.001, 0.0014)),
        origin=Origin(xyz=(-0.081, -0.0009, 0.0024)),
        material=shadow,
        name="blade_score",
    )

    thumb_slider = model.part("thumb_slider")
    thumb_slider.visual(
        Box((0.012, 0.013, 0.005)),
        origin=Origin(xyz=(-0.015, 0.0110, 0.0020)),
        material=dark,
        name="stem",
    )
    thumb_slider.visual(
        Box((0.032, 0.007, 0.009)),
        origin=Origin(xyz=(-0.015, 0.0188, 0.0040)),
        material=black,
        name="thumb_pad",
    )
    for idx, x in enumerate((-0.027, -0.021, -0.015, -0.009, -0.003)):
        thumb_slider.visual(
            Box((0.0020, 0.0016, 0.0095)),
            origin=Origin(xyz=(x, 0.0230, 0.0040)),
            material=dark,
            name=f"grip_rib_{idx}",
        )

    end_cap = model.part("end_cap")
    end_cap.visual(
        Box((0.012, 0.028, 0.018)),
        origin=Origin(xyz=(0.0060, 0.0140, 0.0)),
        material=dark,
        name="cap_panel",
    )
    end_cap.visual(
        Box((0.0030, 0.017, 0.010)),
        origin=Origin(xyz=(0.0128, 0.0140, 0.0)),
        material=black,
        name="cap_pull",
    )
    end_cap.visual(
        Box((0.0010, 0.013, 0.004)),
        origin=Origin(xyz=(0.0147, 0.0140, 0.003)),
        material=steel,
        name="spare_blade_mark",
    )
    end_cap.visual(
        Cylinder(radius=0.00235, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark,
        name="cap_barrel",
    )

    slide = model.articulation(
        "carrier_slide",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=blade_carrier,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.35, lower=-0.006, upper=0.032),
    )
    slide.meta["description"] = "Straight internal channel travel along the knife handle axis."

    model.articulation(
        "slider_mount",
        ArticulationType.FIXED,
        parent=blade_carrier,
        child=thumb_slider,
        origin=Origin(),
    )

    cap_hinge = model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=end_cap,
        origin=Origin(xyz=(HANDLE_REAR_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    cap_hinge.meta["description"] = "Rear spare-blade cap swings open while retained on a vertical hinge pin."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    carrier = object_model.get_part("blade_carrier")
    slider = object_model.get_part("thumb_slider")
    end_cap = object_model.get_part("end_cap")
    slide = object_model.get_articulation("carrier_slide")
    cap_hinge = object_model.get_articulation("cap_hinge")

    ctx.allow_overlap(
        handle,
        end_cap,
        elem_a="hinge_pin",
        elem_b="cap_barrel",
        reason="The rear hinge pin is intentionally captured inside the cap knuckle so the spare-blade cap remains attached.",
    )
    ctx.expect_within(
        handle,
        end_cap,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="cap_barrel",
        margin=0.0002,
        name="hinge pin is captured by cap barrel",
    )
    ctx.expect_overlap(
        handle,
        end_cap,
        axes="z",
        elem_a="hinge_pin",
        elem_b="cap_barrel",
        min_overlap=0.005,
        name="cap barrel surrounds hinge pin height",
    )

    ctx.expect_within(
        carrier,
        handle,
        axes="yz",
        inner_elem="carrier_rail",
        outer_elem="handle_shell",
        margin=0.0005,
        name="blade carrier stays inside handle channel cross section",
    )
    ctx.expect_overlap(
        carrier,
        handle,
        axes="x",
        elem_a="carrier_rail",
        elem_b="handle_shell",
        min_overlap=0.070,
        name="carrier rail remains retained in the handle",
    )
    ctx.expect_gap(
        slider,
        handle,
        axis="y",
        positive_elem="thumb_pad",
        negative_elem="handle_shell",
        min_gap=0.0004,
        max_gap=0.004,
        name="thumb pad protrudes outside the side slot",
    )

    rest_carrier = ctx.part_world_position(carrier)
    rest_slider = ctx.part_world_position(slider)
    with ctx.pose({slide: slide.motion_limits.upper}):
        extended_carrier = ctx.part_world_position(carrier)
        extended_slider = ctx.part_world_position(slider)
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="carrier_rail",
            elem_b="handle_shell",
            min_overlap=0.060,
            name="extended carrier remains in channel",
        )
        ctx.expect_gap(
            slider,
            handle,
            axis="y",
            positive_elem="thumb_pad",
            negative_elem="handle_shell",
            min_gap=0.0004,
            max_gap=0.004,
            name="extended thumb pad remains outside slot",
        )

    ctx.check(
        "carrier slides toward blade nose",
        rest_carrier is not None
        and extended_carrier is not None
        and extended_carrier[0] < rest_carrier[0] - 0.025,
        details=f"rest={rest_carrier}, extended={extended_carrier}",
    )
    ctx.check(
        "thumb slider translates with carrier",
        rest_carrier is not None
        and extended_carrier is not None
        and rest_slider is not None
        and extended_slider is not None
        and abs((extended_slider[0] - rest_slider[0]) - (extended_carrier[0] - rest_carrier[0])) < 1.0e-6,
        details=f"carrier {rest_carrier}->{extended_carrier}, slider {rest_slider}->{extended_slider}",
    )

    closed_cap_aabb = ctx.part_element_world_aabb(end_cap, elem="cap_panel")
    with ctx.pose({cap_hinge: 1.20}):
        open_cap_aabb = ctx.part_element_world_aabb(end_cap, elem="cap_panel")
        cap_origin = ctx.part_world_position(end_cap)
        ctx.expect_within(
            handle,
            end_cap,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="cap_barrel",
            margin=0.0002,
            name="opened cap remains clipped to hinge pin",
        )

    ctx.check(
        "end cap swings open behind handle",
        closed_cap_aabb is not None
        and open_cap_aabb is not None
        and open_cap_aabb[1][0] > closed_cap_aabb[1][0] + 0.006,
        details=f"closed={closed_cap_aabb}, open={open_cap_aabb}",
    )
    ctx.check(
        "end cap hinge origin stays attached",
        cap_origin is not None
        and abs(cap_origin[0] - HANDLE_REAR_X) < 1.0e-6
        and abs(cap_origin[1] - HINGE_Y) < 1.0e-6,
        details=f"cap_origin={cap_origin}",
    )

    return ctx.report()


object_model = build_object_model()
