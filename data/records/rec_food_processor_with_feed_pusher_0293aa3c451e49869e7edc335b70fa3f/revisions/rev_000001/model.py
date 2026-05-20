from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_LENGTH = 0.29
BASE_WIDTH = 0.22
BASE_HEIGHT = 0.082

BOWL_CENTER_X = 0.048
BOWL_BASE_Z = 0.100
BOWL_HEIGHT = 0.134

HINGE_X = -0.088
HINGE_Z = 0.2334
LID_CENTER_X = 0.136
LID_RADIUS = 0.101
CHUTE_OUTER_RADIUS = 0.028
CHUTE_INNER_RADIUS = 0.0215
CHUTE_HEIGHT = 0.086

LATCH_PIVOT_X = 0.040
LATCH_PIVOT_Y = 0.118
LATCH_PIVOT_Z = 0.092


def cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((center[0] - length / 2.0, center[1], center[2]))
    )


def cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((center[0], center[1] - length / 2.0, center[2]))
    )


def make_base_shape() -> cq.Workplane:
    body = cq.Workplane("XY").rect(BASE_LENGTH, BASE_WIDTH).extrude(BASE_HEIGHT)
    body = body.edges("|Z").fillet(0.028)
    body = body.edges(">Z").fillet(0.010)

    rear_riser = (
        cq.Workplane("XY")
        .center(-0.104, 0.0)
        .rect(0.060, 0.148)
        .extrude(0.130)
        .translate((0.0, 0.0, 0.052))
    )
    rear_riser = rear_riser.edges("|Z").fillet(0.018)

    hinge_bridge = (
        cq.Workplane("XY")
        .center(-0.100, 0.0)
        .rect(0.024, 0.082)
        .extrude(0.044)
        .translate((0.0, 0.0, 0.1825))
    )
    hinge_bridge = hinge_bridge.edges("|Z").fillet(0.010)

    bowl_collar = (
        cq.Workplane("XY")
        .center(BOWL_CENTER_X, 0.0)
        .circle(0.038)
        .extrude(0.016)
        .translate((0.0, 0.0, BASE_HEIGHT - 0.001))
    )

    return body.union(rear_riser).union(hinge_bridge).union(bowl_collar)


def make_bowl_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(0.046)
        .workplane(offset=0.024)
        .circle(0.071)
        .workplane(offset=0.100)
        .circle(0.094)
        .loft(combine=True)
    )
    bowl_shell = outer.faces(">Z").shell(-0.004)
    rim = cq.Workplane("XY").circle(0.097).circle(0.089).extrude(0.008).translate((0.0, 0.0, 0.122))
    return bowl_shell.union(rim)


def make_lid_shell_shape() -> cq.Workplane:
    shell_outer = cq.Workplane("XY").center(LID_CENTER_X, 0.0).circle(LID_RADIUS).extrude(0.016)
    shell_outer = shell_outer.edges(">Z").fillet(0.005)
    shell_inner = (
        cq.Workplane("XY")
        .center(LID_CENTER_X, 0.0)
        .circle(0.095)
        .extrude(0.013)
        .translate((0.0, 0.0, -0.002))
    )
    lip = (
        cq.Workplane("XY")
        .center(LID_CENTER_X, 0.0)
        .circle(0.100)
        .circle(0.097)
        .extrude(0.006)
        .translate((0.0, 0.0, -0.004))
    )
    chute_hole = (
        cq.Workplane("XY")
        .center(LID_CENTER_X, 0.0)
        .circle(CHUTE_INNER_RADIUS)
        .extrude(0.060)
        .translate((0.0, 0.0, -0.016))
    )

    lid_shell = shell_outer.cut(shell_inner).union(lip).cut(chute_hole)

    for y in (-0.036, 0.036):
        strap = cq.Workplane("XY").box(0.060, 0.018, 0.012).translate((0.030, y, 0.0))
        ear = cylinder_y(0.007, 0.032, (0.0, y, 0.0))
        lid_shell = lid_shell.union(strap).union(ear)

    return lid_shell


def make_chute_shape() -> cq.Workplane:
    chute_wall = (
        cq.Workplane("XY")
        .center(LID_CENTER_X, 0.0)
        .circle(CHUTE_OUTER_RADIUS)
        .circle(CHUTE_INNER_RADIUS)
        .extrude(CHUTE_HEIGHT)
        .translate((0.0, 0.0, 0.006))
    )
    top_lip = (
        cq.Workplane("XY")
        .center(LID_CENTER_X, 0.0)
        .circle(0.033)
        .circle(CHUTE_OUTER_RADIUS)
        .extrude(0.008)
        .translate((0.0, 0.0, CHUTE_HEIGHT - 0.002))
    )
    return chute_wall.union(top_lip)


def make_blade_carrier_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.017).circle(0.0055).extrude(0.020)

    lower_blade = (
        cq.Workplane("XY")
        .box(0.118, 0.014, 0.002)
        .translate((0.0, 0.0, 0.016))
        .rotate((0.0, 0.0, 0.016), (0.0, 1.0, 0.016), 18.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 18.0)
    )
    upper_blade = (
        cq.Workplane("XY")
        .box(0.100, 0.013, 0.002)
        .translate((0.0, 0.0, 0.026))
        .rotate((0.0, 0.0, 0.026), (0.0, 1.0, 0.026), -22.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), -24.0)
    )

    return hub.union(lower_blade).union(upper_blade)


def add_latch_arm_visuals(part, inward_sign: float, material) -> None:
    part.visual(
        Cylinder(radius=0.0105, length=0.028),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="pivot_barrel",
    )
    part.visual(
        Box((0.016, 0.012, 0.102)),
        origin=Origin(xyz=(0.000, 0.000, 0.051)),
        material=material,
        name="arm_body",
    )
    part.visual(
        Box((0.022, 0.012, 0.020)),
        origin=Origin(xyz=(0.004, 0.000, 0.010)),
        material=material,
        name="arm_base",
    )
    part.visual(
        Box((0.026, 0.022, 0.014)),
        origin=Origin(xyz=(0.012, inward_sign * 0.007, 0.106)),
        material=material,
        name="latch_hook",
    )
    part.visual(
        Box((0.020, 0.024, 0.020)),
        origin=Origin(xyz=(0.000, inward_sign * 0.014, 0.012)),
        material=material,
        name="pivot_lug",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_food_processor")

    base_white = model.material("base_white", rgba=(0.94, 0.94, 0.92, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.12, 0.13, 1.0))
    bowl_clear = model.material("bowl_clear", rgba=(0.82, 0.92, 0.98, 0.35))
    lid_clear = model.material("lid_clear", rgba=(0.88, 0.97, 1.0, 0.28))
    pusher_clear = model.material("pusher_clear", rgba=(0.85, 0.93, 0.98, 0.42))
    blade_metal = model.material("blade_metal", rgba=(0.82, 0.83, 0.86, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(make_base_shape(), "base_body"), material=base_white, name="base_body")
    base.visual(
        Cylinder(radius=0.0081, length=0.030),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=base_white,
        name="hinge_knuckle",
    )
    base.visual(
        Box((0.026, 0.016, 0.028)),
        origin=Origin(xyz=(LATCH_PIVOT_X, LATCH_PIVOT_Y - 0.008, LATCH_PIVOT_Z)),
        material=base_white,
        name="left_pivot_mount",
    )
    base.visual(
        Box((0.026, 0.016, 0.028)),
        origin=Origin(xyz=(LATCH_PIVOT_X, -LATCH_PIVOT_Y + 0.008, LATCH_PIVOT_Z)),
        material=base_white,
        name="right_pivot_mount",
    )

    bowl = model.part("bowl")
    bowl.visual(mesh_from_cadquery(make_bowl_shape(), "bowl_shell"), material=bowl_clear, name="bowl_shell")
    bowl.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=bowl_clear,
        name="spindle_collar",
    )
    bowl.visual(
        Cylinder(radius=0.0035, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=bowl_clear,
        name="spindle_post",
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(make_lid_shell_shape(), "lid_shell"), material=lid_clear, name="lid_shell")
    lid.visual(mesh_from_cadquery(make_chute_shape(), "chute_wall"), material=lid_clear, name="chute_wall")

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=pusher_clear,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.018, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=pusher_clear,
        name="pusher_shaft",
    )

    left_latch_arm = model.part("left_latch_arm")
    add_latch_arm_visuals(left_latch_arm, inward_sign=-1.0, material=charcoal)

    right_latch_arm = model.part("right_latch_arm")
    add_latch_arm_visuals(right_latch_arm, inward_sign=1.0, material=charcoal)

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        mesh_from_cadquery(make_blade_carrier_shape(), "blade_carrier"),
        material=blade_metal,
        name="blade_shell",
    )

    large_dial = model.part("large_dial")
    large_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.023,
                body_style="skirted",
                top_diameter=0.032,
                edge_radius=0.002,
                center=False,
            ),
            "large_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_dark,
        name="dial_shell",
    )

    small_dial = model.part("small_dial")
    small_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.020,
                body_style="skirted",
                top_diameter=0.026,
                edge_radius=0.0015,
                center=False,
            ),
            "small_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_dark,
        name="dial_shell",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(BOWL_CENTER_X, 0.0, BOWL_BASE_Z)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(LID_CENTER_X, 0.0, CHUTE_HEIGHT + 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.10,
            lower=0.0,
            upper=0.055,
        ),
    )

    model.articulation(
        "base_to_left_latch_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_latch_arm,
        origin=Origin(xyz=(LATCH_PIVOT_X, LATCH_PIVOT_Y, LATCH_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    model.articulation(
        "base_to_right_latch_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_latch_arm,
        origin=Origin(xyz=(LATCH_PIVOT_X, -LATCH_PIVOT_Y, LATCH_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
        mimic=Mimic("base_to_left_latch_arm"),
    )

    model.articulation(
        "bowl_to_blade_carrier",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=blade_carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    model.articulation(
        "base_to_large_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=large_dial,
        origin=Origin(xyz=(BASE_LENGTH / 2.0 - 0.003, 0.042, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    model.articulation(
        "base_to_small_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=small_dial,
        origin=Origin(xyz=(BASE_LENGTH / 2.0 - 0.003, -0.038, 0.041)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    left_latch_arm = object_model.get_part("left_latch_arm")
    right_latch_arm = object_model.get_part("right_latch_arm")

    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    left_latch_joint = object_model.get_articulation("base_to_left_latch_arm")
    blade_joint = object_model.get_articulation("bowl_to_blade_carrier")
    large_dial_joint = object_model.get_articulation("base_to_large_dial")
    small_dial_joint = object_model.get_articulation("base_to_small_dial")

    ctx.allow_overlap(
        "base",
        "left_latch_arm",
        reason="The left latch arm pivots on an interleaved side boss at the base wall.",
    )
    ctx.allow_overlap(
        "base",
        "right_latch_arm",
        reason="The right latch arm pivots on an interleaved side boss at the base wall.",
    )
    ctx.allow_overlap(
        "blade_carrier",
        "bowl",
        elem_a="blade_shell",
        elem_b="spindle_post",
        reason="The rotating blade hub is intentionally simplified as sleeving tightly around the fixed center spindle.",
    )

    ctx.expect_overlap(
        lid,
        bowl,
        axes="xy",
        elem_a="lid_shell",
        elem_b="bowl_shell",
        min_overlap=0.16,
        name="closed lid covers the bowl opening",
    )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_shaft",
        outer_elem="chute_wall",
        margin=0.0,
        name="pusher shaft stays centered in the feed chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_shaft",
        elem_b="chute_wall",
        min_overlap=0.040,
        name="resting pusher remains inserted in the chute",
    )

    pusher_upper = pusher_slide.motion_limits.upper if pusher_slide.motion_limits is not None else None
    if pusher_upper is not None:
        with ctx.pose({pusher_slide: pusher_upper}):
            ctx.expect_within(
                pusher,
                lid,
                axes="xy",
                inner_elem="pusher_shaft",
                outer_elem="chute_wall",
                margin=0.0,
                name="raised pusher remains guided by the feed chute",
            )
            ctx.expect_overlap(
                pusher,
                lid,
                axes="z",
                elem_a="pusher_shaft",
                elem_b="chute_wall",
                min_overlap=0.028,
                name="raised pusher still retains insertion in the chute",
            )

        rest_pos = ctx.part_world_position(pusher)
        with ctx.pose({pusher_slide: pusher_upper}):
            raised_pos = ctx.part_world_position(pusher)
        ctx.check(
            "pusher slides upward when extended",
            rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.04,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    def aabb_center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])

    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if lid_upper is not None:
        closed_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: lid_upper}):
            open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward on the rear hinge",
            closed_aabb is not None
            and open_aabb is not None
            and aabb_center(open_aabb, 2) is not None
            and aabb_center(closed_aabb, 2) is not None
            and aabb_center(open_aabb, 2) > aabb_center(closed_aabb, 2) + 0.05,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    latch_upper = left_latch_joint.motion_limits.upper if left_latch_joint.motion_limits is not None else None
    if latch_upper is not None:
        left_closed = ctx.part_world_aabb(left_latch_arm)
        right_closed = ctx.part_world_aabb(right_latch_arm)
        with ctx.pose({left_latch_joint: latch_upper}):
            left_open = ctx.part_world_aabb(left_latch_arm)
            right_open = ctx.part_world_aabb(right_latch_arm)
        ctx.check(
            "side latch arms swing outward together",
            left_closed is not None
            and right_closed is not None
            and left_open is not None
            and right_open is not None
            and aabb_center(left_open, 1) is not None
            and aabb_center(left_closed, 1) is not None
            and aabb_center(right_open, 1) is not None
            and aabb_center(right_closed, 1) is not None
            and aabb_center(left_open, 1) > aabb_center(left_closed, 1) + 0.01
            and aabb_center(right_open, 1) < aabb_center(right_closed, 1) - 0.01,
            details=f"left_closed={left_closed}, left_open={left_open}, right_closed={right_closed}, right_open={right_open}",
        )

    ctx.check(
        "blade carrier uses continuous rotation",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_joint.articulation_type}",
    )
    ctx.check(
        "front control dials use continuous rotation",
        large_dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and small_dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"large={large_dial_joint.articulation_type}, small={small_dial_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
