from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _make_trumpet_body() -> object:
    foot = cq.Workplane("XY").circle(0.23).extrude(0.018)
    flare = (
        cq.Workplane("XY")
        .workplane(offset=0.018)
        .circle(0.18)
        .workplane(offset=0.10)
        .circle(0.055)
        .loft(combine=False)
    )
    pedestal = cq.Workplane("XY").workplane(offset=0.118).circle(0.055).extrude(0.312)
    outer = foot.union(flare).union(pedestal)
    bore = cq.Workplane("XY").workplane(offset=0.018).circle(0.041).extrude(0.412)
    return outer.cut(bore)


def _make_sleeve_shell() -> object:
    outer = cq.Workplane("XY").circle(0.055).extrude(0.08)
    collar = cq.Workplane("XY").workplane(offset=0.08).circle(0.067).extrude(0.02)
    bore = cq.Workplane("XY").circle(0.041).extrude(0.10)
    return outer.union(collar).cut(bore)


def _make_seat_shell() -> object:
    return (
        cq.Workplane("XY")
        .circle(0.19)
        .extrude(0.065)
        .faces(">Z")
        .edges()
        .fillet(0.015)
        .faces("<Z")
        .edges()
        .fillet(0.006)
    )


def _make_backrest_shell() -> object:
    stem = (
        cq.Workplane("XY")
        .box(0.028, 0.095, 0.11, centered=(True, True, False))
        .translate((0.010, 0.0, 0.0))
    )
    pad = (
        cq.Workplane("XY")
        .box(0.042, 0.27, 0.145, centered=(True, True, False))
        .translate((0.028, 0.0, 0.075))
        .edges("|Z")
        .fillet(0.012)
    )
    return stem.union(pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    black_vinyl = model.material("black_vinyl", rgba=(0.11, 0.11, 0.12, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_trumpet_body(), "trumpet_body"),
        material=chrome,
        name="trumpet",
    )
    base.visual(
        mesh_from_cadquery(_make_sleeve_shell(), "sleeve_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=satin_steel,
        name="sleeve",
    )
    base.visual(
        Cylinder(radius=0.225, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber,
        name="foot_ring",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.033, length=0.31),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=chrome,
        name="shaft",
    )
    column.visual(
        Cylinder(radius=0.045, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=satin_steel,
        name="head",
    )
    column.visual(
        Cylinder(radius=0.066, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_steel,
        name="flange",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_make_seat_shell(), "seat_shell"),
        material=black_vinyl,
        name="cushion",
    )
    seat.visual(
        Cylinder(radius=0.115, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=matte_black,
        name="pan",
    )
    seat.visual(
        Box((0.03, 0.11, 0.045)),
        origin=Origin(xyz=(-0.175, 0.0, 0.0325)),
        material=matte_black,
        name="backrest_mount",
    )
    seat.visual(
        Box((0.05, 0.024, 0.03)),
        origin=Origin(xyz=(0.015, 0.132, -0.008)),
        material=matte_black,
        name="lever_mount",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.015, length=0.11),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="barrel",
    )
    backrest.visual(
        Box((0.024, 0.09, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=black_vinyl,
        name="neck",
    )
    backrest.visual(
        mesh_from_cadquery(_make_backrest_shell(), "backrest_shell"),
        origin=Origin(xyz=(-0.012, 0.0, 0.02), rpy=(0.0, -0.18, 0.0)),
        material=black_vinyl,
        name="shell",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.008, length=0.032),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="pivot",
    )
    lever.visual(
        Box((0.014, 0.09, 0.010)),
        origin=Origin(xyz=(0.0, 0.045, -0.006)),
        material=matte_black,
        name="arm",
    )
    lever.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.0, 0.096, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="pull",
    )

    lift = model.articulation(
        "lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.16, lower=0.0, upper=0.08),
        motion_properties=MotionProperties(damping=110.0, friction=18.0),
    )
    swivel = model.articulation(
        "swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=4.5),
        motion_properties=MotionProperties(damping=0.35, friction=0.08),
    )
    model.articulation(
        "backrest_tilt",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.205, 0.0, 0.048)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.10, upper=0.30),
        motion_properties=MotionProperties(damping=2.2, friction=0.3),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=lever,
        origin=Origin(xyz=(0.015, 0.152, -0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-0.35, upper=0.28),
        motion_properties=MotionProperties(damping=1.8, friction=0.22),
    )

    lift.meta["qc_samples"] = [0.0, 0.08]
    swivel.meta["qc_samples"] = [0.0, math.pi / 2.0]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    lever = object_model.get_part("lever")

    lift = object_model.get_articulation("lift")
    swivel = object_model.get_articulation("swivel")
    backrest_tilt = object_model.get_articulation("backrest_tilt")
    lever_pivot = object_model.get_articulation("lever_pivot")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    ctx.expect_within(
        column,
        base,
        axes="xy",
        inner_elem="shaft",
        outer_elem="sleeve",
        margin=0.006,
        name="column stays centered in sleeve at rest",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="z",
        elem_a="shaft",
        elem_b="sleeve",
        min_overlap=0.10,
        name="column remains inserted in sleeve at rest",
    )
    ctx.expect_contact(
        column,
        seat,
        elem_a="head",
        elem_b="pan",
        name="seat pan sits on the swivel head",
    )

    seat_rest = ctx.part_world_position(seat)
    with ctx.pose({lift: 0.08}):
        ctx.expect_within(
            column,
            base,
            axes="xy",
            inner_elem="shaft",
            outer_elem="sleeve",
            margin=0.006,
            name="column stays centered in sleeve when raised",
        )
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="shaft",
            elem_b="sleeve",
            min_overlap=0.08,
            name="column retains sleeve engagement when raised",
        )
        seat_raised = ctx.part_world_position(seat)

    ctx.check(
        "seat height increases with lift travel",
        seat_rest is not None and seat_raised is not None and seat_raised[2] > seat_rest[2] + 0.06,
        details=f"rest={seat_rest}, raised={seat_raised}",
    )

    backrest_rest = ctx.part_world_position(backrest)
    with ctx.pose({swivel: math.pi / 2.0}):
        backrest_rotated = ctx.part_world_position(backrest)

    ctx.check(
        "seat swivel carries the backrest around the pedestal axis",
        backrest_rest is not None
        and backrest_rotated is not None
        and abs(backrest_rotated[0]) < 0.03
        and backrest_rotated[1] < -0.14
        and abs(backrest_rotated[2] - backrest_rest[2]) < 1e-6,
        details=f"rest={backrest_rest}, rotated={backrest_rotated}",
    )

    backrest_rest_center = _aabb_center(ctx.part_element_world_aabb(backrest, elem="shell"))
    with ctx.pose({backrest_tilt: 0.24}):
        backrest_tilted_center = _aabb_center(ctx.part_element_world_aabb(backrest, elem="shell"))

    ctx.check(
        "backrest tilts backward from the hinge",
        backrest_rest_center is not None
        and backrest_tilted_center is not None
        and backrest_tilted_center[0] < backrest_rest_center[0] - 0.015
        and backrest_tilted_center[2] < backrest_rest_center[2],
        details=f"rest={backrest_rest_center}, tilted={backrest_tilted_center}",
    )

    lever_rest_center = _aabb_center(ctx.part_element_world_aabb(lever, elem="pull"))
    with ctx.pose({lever_pivot: 0.22}):
        lever_lifted_center = _aabb_center(ctx.part_element_world_aabb(lever, elem="pull"))

    ctx.check(
        "height lever lifts upward on its side pivot",
        lever_rest_center is not None
        and lever_lifted_center is not None
        and lever_lifted_center[2] > lever_rest_center[2] + 0.015
        and lever_lifted_center[1] > 0.18,
        details=f"rest={lever_rest_center}, lifted={lever_lifted_center}",
    )

    return ctx.report()


object_model = build_object_model()
