from __future__ import annotations

from math import pi

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


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Centered box with rounded vertical corners, sized in meters."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _hollow_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Open-ended vertical tube from local z=0 to z=height."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_swivel_stool")

    chrome = model.material("brushed_chrome", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_vinyl = model.material("charcoal_vinyl", rgba=(0.035, 0.037, 0.040, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    release_red = model.material("release_red", rgba=(0.85, 0.06, 0.035, 1.0))

    # Fixed pedestal: a broad weighted floor disk and a hollow gas-lift sleeve.
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.27, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=chrome,
        name="floor_disk",
    )
    base.visual(
        mesh_from_cadquery(_hollow_cylinder(0.040, 0.032, 0.340), "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=chrome,
        name="outer_sleeve",
    )
    base.visual(
        mesh_from_cadquery(_hollow_cylinder(0.055, 0.034, 0.025), "sleeve_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=chrome,
        name="sleeve_collar",
    )
    # Hidden nylon guide pads at the sleeve mouth keep the moving post captured.
    base.visual(
        Box((0.010, 0.026, 0.100)),
        origin=Origin(xyz=(0.031, 0.0, 0.330)),
        material=black_plastic,
        name="guide_pad_0",
    )
    base.visual(
        Box((0.010, 0.026, 0.100)),
        origin=Origin(xyz=(-0.031, 0.0, 0.330)),
        material=black_plastic,
        name="guide_pad_1",
    )
    base.visual(
        Box((0.026, 0.010, 0.100)),
        origin=Origin(xyz=(0.0, 0.031, 0.330)),
        material=black_plastic,
        name="guide_pad_2",
    )
    base.visual(
        Box((0.026, 0.010, 0.100)),
        origin=Origin(xyz=(0.0, -0.031, 0.330)),
        material=black_plastic,
        name="guide_pad_3",
    )

    # Sliding height-adjustment column.  Its frame is at the sleeve entry plane.
    column = model.part("column")
    column.visual(
        Cylinder(radius=0.026, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=chrome,
        name="inner_post",
    )
    column.visual(
        Cylinder(radius=0.050, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=chrome,
        name="height_stop_collar",
    )
    column.visual(
        Cylinder(radius=0.064, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=chrome,
        name="top_bearing",
    )

    # Seat, swivel hub, low rear back panel, and the fixed half of the tab pivot.
    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_rounded_box((0.430, 0.430, 0.065), 0.035), "square_seat"),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=dark_vinyl,
        name="square_cushion",
    )
    seat.visual(
        Box((0.300, 0.300, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=black_plastic,
        name="seat_support_plate",
    )
    seat.visual(
        Cylinder(radius=0.072, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=chrome,
        name="swivel_hub",
    )
    seat.visual(
        Cylinder(radius=0.012, length=0.235),
        origin=Origin(xyz=(-0.135, 0.190, 0.205)),
        material=chrome,
        name="back_post_0",
    )
    seat.visual(
        Cylinder(radius=0.012, length=0.235),
        origin=Origin(xyz=(0.135, 0.190, 0.205)),
        material=chrome,
        name="back_post_1",
    )
    seat.visual(
        mesh_from_cadquery(_rounded_box((0.380, 0.045, 0.185), 0.018), "low_back_panel"),
        origin=Origin(xyz=(0.0, 0.215, 0.315)),
        material=dark_vinyl,
        name="low_back_panel",
    )
    # Two cheek lugs form a short fork around the front release-tab pivot.
    seat.visual(
        Box((0.025, 0.036, 0.040)),
        origin=Origin(xyz=(-0.065, -0.155, 0.0025)),
        material=black_plastic,
        name="pivot_lug_0",
    )
    seat.visual(
        Box((0.025, 0.036, 0.040)),
        origin=Origin(xyz=(0.065, -0.155, 0.0025)),
        material=black_plastic,
        name="pivot_lug_1",
    )

    release_tab = model.part("release_tab")
    release_tab.visual(
        Cylinder(radius=0.010, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="pivot_pin",
    )
    release_tab.visual(
        Box((0.034, 0.178, 0.014)),
        origin=Origin(xyz=(0.0, -0.085, -0.007)),
        material=release_red,
        name="front_lever",
    )
    release_tab.visual(
        Box((0.070, 0.038, 0.020)),
        origin=Origin(xyz=(0.0, -0.178, -0.018)),
        material=release_red,
        name="finger_tab",
    )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.25, lower=0.0, upper=0.120),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0),
    )
    model.articulation(
        "tab_pivot",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=release_tab,
        origin=Origin(xyz=(0.0, -0.155, 0.0025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=5.0, lower=-0.25, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    seat = object_model.get_part("seat")
    release_tab = object_model.get_part("release_tab")

    height_slide = object_model.get_articulation("height_slide")
    seat_swivel = object_model.get_articulation("seat_swivel")
    tab_pivot = object_model.get_articulation("tab_pivot")

    ctx.check(
        "height column uses prismatic travel",
        height_slide.articulation_type == ArticulationType.PRISMATIC
        and height_slide.motion_limits is not None
        and abs(height_slide.motion_limits.upper - 0.120) < 1e-6,
        details=f"type={height_slide.articulation_type}, limits={height_slide.motion_limits}",
    )
    ctx.check(
        "seat swivel is continuous about the column",
        seat_swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(seat_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={seat_swivel.articulation_type}, axis={seat_swivel.axis}",
    )
    ctx.check(
        "front tab has a short horizontal pivot",
        tab_pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(tab_pivot.axis) == (1.0, 0.0, 0.0),
        details=f"type={tab_pivot.articulation_type}, axis={tab_pivot.axis}",
    )

    ctx.expect_gap(
        column,
        base,
        axis="z",
        positive_elem="height_stop_collar",
        negative_elem="sleeve_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="height stop collar rests on sleeve collar",
    )
    ctx.expect_gap(
        seat,
        column,
        axis="z",
        positive_elem="swivel_hub",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel hub is seated on the bearing",
    )
    ctx.expect_contact(
        release_tab,
        seat,
        elem_a="pivot_pin",
        elem_b="pivot_lug_0",
        contact_tol=0.002,
        name="release pivot pin reaches first cheek lug",
    )
    ctx.expect_contact(
        release_tab,
        seat,
        elem_a="pivot_pin",
        elem_b="pivot_lug_1",
        contact_tol=0.002,
        name="release pivot pin reaches second cheek lug",
    )

    rest_seat_position = ctx.part_world_position(seat)
    with ctx.pose({height_slide: 0.120}):
        ctx.expect_within(
            column,
            base,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="outer_sleeve",
            margin=0.0,
            name="raised column remains centered in sleeve footprint",
        )
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="outer_sleeve",
            min_overlap=0.10,
            name="raised column remains inserted in the sleeve",
        )
        raised_seat_position = ctx.part_world_position(seat)

    ctx.check(
        "height slide raises counter stool seat",
        rest_seat_position is not None
        and raised_seat_position is not None
        and raised_seat_position[2] > rest_seat_position[2] + 0.11,
        details=f"rest={rest_seat_position}, raised={raised_seat_position}",
    )

    rest_tab_aabb = ctx.part_element_world_aabb(release_tab, elem="finger_tab")
    rest_tab_part_aabb = ctx.part_world_aabb(release_tab)
    rest_seat_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({tab_pivot: 0.45}):
        pulled_tab_aabb = ctx.part_element_world_aabb(release_tab, elem="finger_tab")

    ctx.check(
        "release tab projects from the front of the seat",
        rest_tab_part_aabb is not None
        and rest_seat_aabb is not None
        and rest_tab_part_aabb[0][1] < rest_seat_aabb[0][1] - 0.06,
        details=f"tab_aabb={rest_tab_part_aabb}, seat_aabb={rest_seat_aabb}",
    )
    ctx.check(
        "release tab pull rotates the finger pad downward",
        rest_tab_aabb is not None
        and pulled_tab_aabb is not None
        and pulled_tab_aabb[0][2] < rest_tab_aabb[0][2] - 0.045,
        details=f"rest={rest_tab_aabb}, pulled={pulled_tab_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
