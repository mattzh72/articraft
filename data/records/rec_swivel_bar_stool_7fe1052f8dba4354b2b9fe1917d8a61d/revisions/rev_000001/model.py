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


def _pedestal_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .circle(0.23)
        .extrude(0.016)
        .faces(">Z")
        .workplane()
        .circle(0.18)
        .extrude(0.014)
    )

    lower_shroud = cq.Workplane("XY").circle(0.055).circle(0.030).extrude(0.060).translate((0.0, 0.0, 0.030))
    upper_shroud = cq.Workplane("XY").circle(0.038).circle(0.028).extrude(0.290).translate((0.0, 0.0, 0.090))

    return base_plate.union(lower_shroud).union(upper_shroud)


def _saddle_shell_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .workplane(offset=-0.160)
        .center(0.0, 0.030)
        .ellipse(0.180, 0.022)
        .workplane(offset=0.120)
        .center(0.0, 0.036)
        .ellipse(0.150, 0.028)
        .workplane(offset=0.120)
        .center(0.0, 0.040)
        .ellipse(0.110, 0.030)
        .workplane(offset=0.100)
        .center(0.0, 0.028)
        .ellipse(0.060, 0.020)
        .loft(combine=True)
    )


def _lever_shape() -> cq.Workplane:
    pivot = cq.Workplane("XZ").circle(0.008).extrude(0.015, both=True)
    arm = cq.Workplane("XY").box(0.095, 0.012, 0.008).translate((0.045, 0.0, -0.010)).rotate((0, 0, 0), (0, 1, 0), -14.0)
    paddle = cq.Workplane("XY").box(0.032, 0.022, 0.006).translate((0.088, 0.0, -0.017)).rotate((0, 0, 0), (0, 1, 0), -14.0)
    return pivot.union(arm).union(paddle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_stool")

    powdercoat = model.material("powdercoat", rgba=(0.18, 0.19, 0.20, 1.0))
    chrome = model.material("chrome", rgba=(0.79, 0.80, 0.83, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.12, 0.11, 0.10, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_pedestal_shape(), "stool_pedestal"),
        material=powdercoat,
        name="pedestal",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.024, length=0.400),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=chrome,
        name="shaft",
    )
    mast.visual(
        Cylinder(radius=0.036, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_trim,
        name="bearing",
    )
    mast.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=dark_trim,
        name="top_cap",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_saddle_shell_shape(), "stool_saddle_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=seat_vinyl,
        name="shell",
    )
    seat.visual(
        Box((0.180, 0.190, 0.020)),
        origin=Origin(xyz=(0.0, -0.015, 0.010)),
        material=dark_trim,
        name="pan",
    )
    seat.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_trim,
        name="hub",
    )
    seat.visual(
        Box((0.150, 0.200, 0.040)),
        origin=Origin(xyz=(0.0, -0.015, 0.030)),
        material=dark_trim,
        name="support",
    )
    seat.visual(
        Box((0.016, 0.010, 0.014)),
        origin=Origin(xyz=(0.084, 0.000, 0.005)),
        material=dark_trim,
        name="mount_0",
    )
    seat.visual(
        Box((0.016, 0.010, 0.014)),
        origin=Origin(xyz=(0.084, 0.040, 0.005)),
        material=dark_trim,
        name="mount_1",
    )
    seat.visual(
        Cylinder(radius=0.004, length=0.050),
        origin=Origin(xyz=(0.095, 0.020, 0.006), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_pin",
    )

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_lever_shape(), "stool_lift_lever"),
        material=powdercoat,
        name="body",
    )

    lift_limits = MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.120)
    lever_limits = MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=0.42)

    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "seat_spin",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lever,
        origin=Origin(xyz=(0.095, 0.020, 0.006)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=lever_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    seat = object_model.get_part("seat")
    lever = object_model.get_part("lever")

    mast_lift = object_model.get_articulation("mast_lift")
    seat_spin = object_model.get_articulation("seat_spin")
    lever_pivot = object_model.get_articulation("lever_pivot")

    ctx.allow_overlap(
        seat,
        lever,
        elem_a="pivot_pin",
        elem_b="body",
        reason="The lift lever rotates on a real cross-pin captured below the seat.",
    )

    ctx.expect_contact(
        seat,
        mast,
        elem_a="pan",
        elem_b="top_cap",
        name="seat pan rests on mast cap",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="shaft",
        elem_b="pedestal",
        min_overlap=0.160,
        name="collapsed mast remains engaged in pedestal",
    )
    ctx.expect_within(
        mast,
        base,
        axes="xy",
        elem_a="shaft",
        elem_b="pedestal",
        margin=0.010,
        name="mast stays centered over round base",
    )

    collapsed_seat_pos = ctx.part_world_position(seat)
    collapsed_lever_pos = ctx.part_world_position(lever)
    lever_rest_aabb = ctx.part_element_world_aabb(lever, elem="body")

    with ctx.pose({mast_lift: mast_lift.motion_limits.upper}):
        ctx.expect_contact(
            seat,
            mast,
            elem_a="pan",
            elem_b="top_cap",
            name="seat still bears on mast cap at full lift",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="shaft",
            elem_b="pedestal",
            min_overlap=0.040,
            name="extended mast retains insertion in pedestal",
        )
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            elem_a="shaft",
            elem_b="pedestal",
            margin=0.010,
            name="extended mast stays centered over round base",
        )
        raised_seat_pos = ctx.part_world_position(seat)

    ctx.check(
        "seat rises when gas lift extends",
        collapsed_seat_pos is not None
        and raised_seat_pos is not None
        and raised_seat_pos[2] > collapsed_seat_pos[2] + 0.08,
        details=f"collapsed={collapsed_seat_pos}, raised={raised_seat_pos}",
    )

    with ctx.pose({seat_spin: 1.3}):
        spun_lever_pos = ctx.part_world_position(lever)

    ctx.check(
        "seat assembly swivels around the column",
        collapsed_lever_pos is not None
        and spun_lever_pos is not None
        and abs(spun_lever_pos[0] - collapsed_lever_pos[0]) > 0.03
        and abs(spun_lever_pos[1] - collapsed_lever_pos[1]) > 0.03,
        details=f"rest={collapsed_lever_pos}, spun={spun_lever_pos}",
    )

    with ctx.pose({lever_pivot: lever_pivot.motion_limits.upper}):
        lever_raised_aabb = ctx.part_element_world_aabb(lever, elem="body")

    ctx.check(
        "lift lever tip raises under actuation",
        lever_rest_aabb is not None
        and lever_raised_aabb is not None
        and lever_raised_aabb[1][2] > lever_rest_aabb[1][2] + 0.012,
        details=f"rest={lever_rest_aabb}, raised={lever_raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
