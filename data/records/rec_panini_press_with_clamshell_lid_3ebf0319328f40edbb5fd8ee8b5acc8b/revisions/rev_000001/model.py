from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

WIDTH = 0.31
DEPTH = 0.26
BASE_HEIGHT = 0.056
HINGE_Y = -0.106
HINGE_Z = 0.078
LID_LENGTH = 0.236
LID_WIDTH = 0.314
LATCH_PIVOT_Y = 0.134
LATCH_PIVOT_Z = 0.063


def make_grill_plate(
    *,
    width: float,
    depth: float,
    plate_thickness: float,
    ridge_count: int,
    ridge_width: float,
    ridge_height: float,
) -> cq.Workplane:
    plate = cq.Workplane("XY").box(width, depth, plate_thickness, centered=(True, True, False))
    ridge_span = depth - 0.026
    if ridge_count > 1:
        step = ridge_span / (ridge_count - 1)
        positions = [(-ridge_span / 2.0) + i * step for i in range(ridge_count)]
    else:
        positions = [0.0]
    for y_pos in positions:
        ridge = (
            cq.Workplane("XY")
            .box(width - 0.012, ridge_width, ridge_height, centered=(True, True, False))
            .translate((0.0, y_pos, plate_thickness))
            .edges("|X")
            .fillet(min(ridge_height * 0.45, ridge_width * 0.35))
        )
        plate = plate.union(ridge)
    return plate


def make_base_body() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(WIDTH, DEPTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
        .faces(">Z")
        .edges()
        .fillet(0.006)
    )

    grill_recess = (
        cq.Workplane("XY")
        .box(WIDTH - 0.032, DEPTH - 0.040, 0.010, centered=(True, True, False))
        .translate((0.0, 0.004, BASE_HEIGHT - 0.010))
    )
    body = body.cut(grill_recess)

    front_cheek = cq.Workplane("XY").box(0.010, 0.024, 0.018, centered=(True, True, False))
    body = body.union(front_cheek.translate((-0.012, 0.124, BASE_HEIGHT - 0.004)))
    body = body.union(front_cheek.translate((0.012, 0.124, BASE_HEIGHT - 0.004)))

    rear_spine = (
        cq.Workplane("XY")
        .box(WIDTH - 0.050, 0.018, 0.004, centered=(True, True, False))
        .translate((0.0, -0.119, BASE_HEIGHT - 0.002))
        .edges("|X")
        .fillet(0.0015)
    )
    body = body.union(rear_spine)
    return body


def make_lid_body() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .box(LID_WIDTH, LID_LENGTH, 0.044, centered=(True, False, False))
        .translate((0.0, 0.0, -0.012))
        .edges("|Z")
        .fillet(0.016)
    )

    dome = (
        cq.Workplane("XY")
        .box(LID_WIDTH - 0.048, LID_LENGTH - 0.060, 0.028, centered=(True, False, False))
        .translate((0.0, 0.030, 0.010))
    )
    lid = lid.union(dome)

    underside_recess = (
        cq.Workplane("XY")
        .box(LID_WIDTH - 0.040, 0.198, 0.006, centered=(True, False, False))
        .translate((0.0, 0.020, -0.012))
    )
    lid = lid.cut(underside_recess)

    latch_relief = (
        cq.Workplane("XY")
        .box(0.022, 0.020, 0.030, centered=(True, False, False))
        .translate((0.0, 0.220, -0.012))
    )
    lid = lid.cut(latch_relief)

    catch_bar = (
        cq.Workplane("XY")
        .box(0.058, 0.010, 0.006, centered=(True, False, False))
        .translate((0.0, 0.224, -0.009))
        .edges("|X")
        .fillet(0.002)
    )
    lid = lid.union(catch_bar)
    return lid


def make_latch_body() -> cq.Workplane:
    pivot = cq.Workplane("YZ").circle(0.0042).extrude(0.007, both=True)

    arm = (
        cq.Workplane("XY")
        .box(0.008, 0.010, 0.030, centered=(True, True, False))
        .translate((0.0, 0.002, -0.004))
    )
    top_finger = (
        cq.Workplane("XY")
        .box(0.008, 0.016, 0.006, centered=(True, False, False))
        .translate((0.0, 0.004, 0.020))
    )

    return pivot.union(arm).union(top_finger)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panini_maker")

    shell = model.material("shell", rgba=(0.16, 0.17, 0.18, 1.0))
    plate = model.material("plate", rgba=(0.57, 0.59, 0.60, 1.0))
    latch_finish = model.material("latch_finish", rgba=(0.22, 0.23, 0.24, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base_shell")
    base.visual(
        mesh_from_cadquery(make_base_body(), "base_body"),
        material=shell,
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(
            make_grill_plate(
                width=WIDTH - 0.040,
                depth=DEPTH - 0.050,
                plate_thickness=0.004,
                ridge_count=7,
                ridge_width=0.011,
                ridge_height=0.006,
            ),
            "lower_grill",
        ),
        origin=Origin(xyz=(0.0, 0.004, BASE_HEIGHT - 0.011)),
        material=plate,
        name="lower_grill",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(make_lid_body(), "lid_body"),
        material=shell,
        name="lid_body",
    )
    lid.visual(
        mesh_from_cadquery(
            make_grill_plate(
                width=LID_WIDTH - 0.042,
                depth=0.196,
                plate_thickness=0.004,
                ridge_count=5,
                ridge_width=0.010,
                ridge_height=0.004,
            ),
            "upper_grill",
        ),
        origin=Origin(xyz=(0.0, 0.118, -0.0125)),
        material=plate,
        name="upper_grill",
    )

    latch = model.part("latch_hook")
    latch.visual(
        mesh_from_cadquery(make_latch_body(), "latch_hook"),
        material=latch_finish,
        name="latch_body",
    )

    dial = model.part("browning_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.022,
                body_style="skirted",
                top_diameter=0.035,
                skirt=KnobSkirt(0.050, 0.005, flare=0.05),
                grip=KnobGrip(style="fluted", count=14, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "browning_dial",
        ),
        material=dial_finish,
        name="dial_body",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=0.0, upper=1.20),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=latch,
        origin=Origin(xyz=(0.0, LATCH_PIVOT_Y, LATCH_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(WIDTH / 2.0, 0.008, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_shell")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch_hook")
    dial = object_model.get_part("browning_dial")
    lid_hinge = object_model.get_articulation("lid_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")
    dial_spin = object_model.get_articulation("dial_spin")

    ctx.allow_overlap(
        base,
        latch,
        elem_a="base_body",
        elem_b="latch_body",
        reason="The latch pivot cylinder is intentionally simplified as rotating inside the lower-shell pivot cheeks.",
    )

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="upper_grill",
        negative_elem="lower_grill",
        min_gap=0.002,
        max_gap=0.020,
        name="upper plate sits just above the lower grill plate",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="lid_body",
        elem_b="base_body",
        min_overlap=0.21,
        name="lid covers the cooking footprint",
    )
    ctx.expect_gap(
        dial,
        base,
        axis="x",
        positive_elem="dial_body",
        negative_elem="base_body",
        min_gap=0.0,
        max_gap=0.002,
        name="dial stays distinct from the right side wall",
    )
    ctx.expect_overlap(
        dial,
        base,
        axes="yz",
        elem_a="dial_body",
        elem_b="base_body",
        min_overlap=0.015,
        name="dial is mounted onto the right wall zone",
    )

    closed_latch = ctx.part_element_world_aabb(latch, elem="latch_body")
    closed_base = ctx.part_element_world_aabb(base, elem="base_body")
    closed_lid = ctx.part_element_world_aabb(lid, elem="lid_body")
    ctx.check(
        "latch bridges the closed shells",
        closed_latch is not None
        and closed_base is not None
        and closed_lid is not None
        and closed_latch[0][2] < closed_base[1][2] + 0.004
        and closed_latch[1][2] > closed_lid[0][2] + 0.010,
        details=f"latch={closed_latch!r}, base={closed_base!r}, lid={closed_lid!r}",
    )

    dial_limits = dial_spin.motion_limits
    ctx.check(
        "dial is continuous",
        dial_spin.joint_type == ArticulationType.CONTINUOUS
        and dial_limits is not None
        and dial_limits.lower is None
        and dial_limits.upper is None,
        details=f"type={dial_spin.joint_type!r}, limits={dial_limits!r}",
    )

    lid_limits = lid_hinge.motion_limits
    latch_limits = latch_pivot.motion_limits
    if lid_limits is not None and lid_limits.upper is not None and latch_limits is not None and latch_limits.upper is not None:
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_body")
        closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_body")
        with ctx.pose({latch_pivot: latch_limits.upper, lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_body")
            released_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_body")
        ctx.check(
            "lid opens upward from the rear hinge",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.090
            and open_lid_aabb[0][1] < closed_lid_aabb[0][1] - 0.020,
            details=f"closed={closed_lid_aabb!r}, open={open_lid_aabb!r}",
        )
        ctx.check(
            "latch releases forward and downward",
            closed_latch_aabb is not None
            and released_latch_aabb is not None
            and released_latch_aabb[1][1] > closed_latch_aabb[1][1] + 0.010
            and released_latch_aabb[0][2] < closed_latch_aabb[0][2] - 0.010,
            details=f"closed={closed_latch_aabb!r}, released={released_latch_aabb!r}",
        )

    return ctx.report()


object_model = build_object_model()
