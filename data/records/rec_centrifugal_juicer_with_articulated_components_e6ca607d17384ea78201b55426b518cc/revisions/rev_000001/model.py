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
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.22
BODY_LENGTH = 0.27
BASE_HEIGHT = 0.06
CHAMBER_CAVITY_Z = 0.198
CHAMBER_RIM_TOP = 0.326
HINGE_X = -0.094
HINGE_Z = 0.327
ARM_PIVOT_Z = 0.226
ARM_PIVOT_Y = 0.122


def _ring_shape(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(height)
        .cut(cq.Workplane("XY").circle(inner_radius).extrude(height))
    )


def _body_shell_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BODY_LENGTH, BODY_WIDTH, BASE_HEIGHT, centered=(True, True, False))

    lower_tower = (
        cq.Workplane("XY")
        .workplane(offset=0.038)
        .ellipse(0.082, 0.068)
        .extrude(0.170)
    )
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=0.195)
        .ellipse(0.108, 0.086)
        .extrude(0.085)
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.028, 0.145, 0.032, centered=(True, True, False))
        .translate((HINGE_X, 0.0, 0.296))
    )
    spout = (
        cq.Workplane("XY")
        .box(0.050, 0.056, 0.018, centered=(True, True, False))
        .translate((0.120, 0.0, 0.214))
    )

    shell = base.union(lower_tower).union(shoulder).union(rear_bridge).union(spout)

    cavity = (
        cq.Workplane("XY")
        .circle(0.084)
        .extrude(CHAMBER_RIM_TOP - CHAMBER_CAVITY_Z + 0.004)
        .translate((0.0, 0.0, CHAMBER_CAVITY_Z))
    )
    shell = shell.cut(cavity)
    drive_spindle = cq.Workplane("XY").circle(0.010).extrude(0.012).translate((0.0, 0.0, 0.193))
    shell = shell.union(drive_spindle)

    return shell.combine()


def _chamber_ring_shape() -> cq.Workplane:
    ring = _ring_shape(0.096, 0.084, 0.022)
    return ring.translate((0.0, 0.0, 0.304))


def _lid_cover_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(0.103)
        .extrude(0.038)
        .translate((0.094, 0.0, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .circle(0.094)
        .extrude(0.034)
        .translate((0.094, 0.0, 0.0))
    )
    feed_opening = (
        cq.Workplane("XY")
        .box(0.062, 0.050, 0.060, centered=(True, True, False))
        .translate((0.122, 0.0, 0.0))
    )
    left_ear = (
        cq.Workplane("XY")
        .box(0.022, 0.012, 0.028, centered=(True, True, False))
        .translate((0.106, 0.090, 0.004))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.022, 0.012, 0.028, centered=(True, True, False))
        .translate((0.106, -0.090, 0.004))
    )
    return outer.cut(inner).cut(feed_opening).union(left_ear).union(right_ear)


def _chute_shell_shape() -> cq.Workplane:
    z_center = 0.1145
    left_wall = cq.Workplane("XY").box(0.086, 0.012, 0.165).translate((0.122, 0.029, z_center))
    right_wall = cq.Workplane("XY").box(0.086, 0.012, 0.165).translate((0.122, -0.029, z_center))
    front_wall = cq.Workplane("XY").box(0.014, 0.046, 0.165).translate((0.158, 0.0, z_center))
    rear_wall = cq.Workplane("XY").box(0.014, 0.046, 0.165).translate((0.086, 0.0, z_center))
    collar = cq.Workplane("XY").box(0.096, 0.080, 0.010).translate((0.122, 0.0, 0.033))
    collar_opening = cq.Workplane("XY").box(0.060, 0.046, 0.012).translate((0.122, 0.0, 0.033))
    chute = left_wall.union(right_wall).union(front_wall).union(rear_wall).union(collar.cut(collar_opening))
    return chute.combine()


def _pusher_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").box(0.056, 0.042, 0.300).translate((0.0, 0.0, -0.140))
    collar = cq.Workplane("XY").box(0.072, 0.058, 0.018).translate((0.0, 0.0, 0.009))
    grip = cq.Workplane("XY").box(0.088, 0.072, 0.054).translate((0.0, 0.0, 0.045))
    return shaft.union(collar).union(grip).combine()


def _basket_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.076).extrude(0.108)
    inner = cq.Workplane("XY").circle(0.070).extrude(0.102).translate((0.0, 0.0, 0.006))
    lip = _ring_shape(0.082, 0.072, 0.004).translate((0.0, 0.0, 0.104))
    hub = cq.Workplane("XY").circle(0.012).extrude(0.020)

    basket = outer.cut(inner).union(lip).union(hub)

    slot = cq.Workplane("XY").box(0.006, 0.018, 0.032).translate((0.073, 0.0, 0.055))
    for angle_deg in range(0, 360, 30):
        basket = basket.cut(slot.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))

    return basket.combine()


def _knob_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.024,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.052, 0.006, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
            center=False,
        ),
        "selector_knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="household_juicer")

    body_metal = model.material("body_metal", rgba=(0.78, 0.79, 0.80, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.17, 1.0))
    clear_lid = model.material("clear_lid", rgba=(0.72, 0.84, 0.92, 0.32))
    basket_steel = model.material("basket_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.08, 0.09, 0.10, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.69, 0.71, 0.73, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=body_metal,
        name="base_shell",
    )
    body.visual(
        Box((0.168, 0.138, 0.138)),
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        material=body_metal,
        name="lower_tower",
    )
    body.visual(
        Box((0.088, 0.102, 0.040)),
        origin=Origin(xyz=(0.040, 0.0, 0.175)),
        material=body_metal,
        name="front_brace",
    )
    body.visual(
        Box((0.084, 0.018, 0.090)),
        origin=Origin(xyz=(0.0, 0.093, 0.205)),
        material=body_metal,
        name="side_riser_0",
    )
    body.visual(
        Box((0.084, 0.018, 0.090)),
        origin=Origin(xyz=(0.0, -0.093, 0.205)),
        material=body_metal,
        name="side_riser_1",
    )
    body.visual(
        Box((0.020, 0.018, 0.060)),
        origin=Origin(xyz=(-0.094, 0.060, 0.190)),
        material=body_metal,
        name="neck_0",
    )
    body.visual(
        Box((0.020, 0.018, 0.060)),
        origin=Origin(xyz=(-0.094, -0.060, 0.190)),
        material=body_metal,
        name="neck_1",
    )
    body.visual(
        Box((0.020, 0.018, 0.092)),
        origin=Origin(xyz=(-0.094, 0.060, 0.250)),
        material=body_metal,
        name="rear_pillar_0",
    )
    body.visual(
        Box((0.020, 0.018, 0.092)),
        origin=Origin(xyz=(-0.094, -0.060, 0.250)),
        material=body_metal,
        name="rear_pillar_1",
    )
    body.visual(
        Box((0.118, 0.030, 0.106)),
        origin=Origin(xyz=(0.0, 0.100, 0.251)),
        material=body_metal,
        name="shoulder_0",
    )
    body.visual(
        Box((0.118, 0.030, 0.106)),
        origin=Origin(xyz=(0.0, -0.100, 0.251)),
        material=body_metal,
        name="shoulder_1",
    )
    body.visual(
        Box((0.028, 0.220, 0.032)),
        origin=Origin(xyz=(HINGE_X, 0.0, 0.312)),
        material=body_metal,
        name="rear_bridge",
    )
    body.visual(
        Box((0.050, 0.056, 0.028)),
        origin=Origin(xyz=(0.106, 0.0, 0.204)),
        material=charcoal,
        name="spout",
    )
    body.visual(
        mesh_from_cadquery(_chamber_ring_shape(), "chamber_ring"),
        material=charcoal,
        name="chamber_ring",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.199)),
        material=charcoal,
        name="drive_spindle",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(HINGE_X - 0.010, 0.054, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(HINGE_X - 0.010, -0.054, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="hinge_barrel_1",
    )
    body.visual(
        Box((0.024, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, ARM_PIVOT_Y - 0.021, ARM_PIVOT_Z)),
        material=charcoal,
        name="arm_mount_0",
    )
    body.visual(
        Box((0.024, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, -(ARM_PIVOT_Y - 0.021), ARM_PIVOT_Z)),
        material=charcoal,
        name="arm_mount_1",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.086, 0.0, 0.128), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="knob_boss",
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_cover_shape(), "lid_cover"), material=clear_lid, name="lid_cover")
    lid.visual(mesh_from_cadquery(_chute_shell_shape(), "chute_shell"), material=clear_lid, name="chute_shell")
    lid.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )

    pusher = model.part("pusher")
    pusher.visual(mesh_from_cadquery(_pusher_shape(), "pusher_body"), material=dark_plastic, name="pusher_shaft")

    basket = model.part("basket")
    basket.visual(mesh_from_cadquery(_basket_shape(), "basket_shell"), material=basket_steel, name="basket_shell")
    basket.visual(
        Box((0.014, 0.012, 0.014)),
        origin=Origin(xyz=(0.074, 0.0, 0.099)),
        material=basket_steel,
        name="basket_tab",
    )

    left_arm = model.part("left_arm")
    left_arm.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_finish,
        name="pivot_hub",
    )
    left_arm.visual(
        Box((0.022, 0.014, 0.178)),
        origin=Origin(xyz=(0.012, 0.0, 0.100)),
        material=arm_finish,
        name="arm_spine",
    )
    left_arm.visual(
        Box((0.030, 0.026, 0.014)),
        origin=Origin(xyz=(0.024, -0.014, 0.188)),
        material=arm_finish,
        name="left_hook",
    )

    right_arm = model.part("right_arm")
    right_arm.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_finish,
        name="pivot_hub",
    )
    right_arm.visual(
        Box((0.022, 0.014, 0.178)),
        origin=Origin(xyz=(0.012, 0.0, 0.100)),
        material=arm_finish,
        name="arm_spine",
    )
    right_arm.visual(
        Box((0.030, 0.026, 0.014)),
        origin=Origin(xyz=(0.024, 0.014, 0.188)),
        material=arm_finish,
        name="right_hook",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="knob_shaft",
    )
    knob.visual(
        _knob_mesh(),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="knob_shell",
    )
    knob.visual(
        Box((0.006, 0.004, 0.012)),
        origin=Origin(xyz=(0.032, 0.0, 0.014)),
        material=body_metal,
        name="selector_tab",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.2, lower=0.0, upper=1.18),
    )
    model.articulation(
        "pusher_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.122, 0.0, 0.197)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.25, lower=0.0, upper=0.11),
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=50.0),
    )
    model.articulation(
        "left_arm_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_arm,
        origin=Origin(xyz=(0.0, ARM_PIVOT_Y, ARM_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=0.92),
    )
    model.articulation(
        "right_arm_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_arm,
        origin=Origin(xyz=(0.0, -ARM_PIVOT_Y, ARM_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=0.92),
    )
    model.articulation(
        "selector_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.090, 0.0, 0.128)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-1.15, upper=1.15),
    )

    return model


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")
    knob = object_model.get_part("knob")

    lid_hinge = object_model.get_articulation("lid_hinge")
    pusher_slide = object_model.get_articulation("pusher_slide")
    basket_spin = object_model.get_articulation("basket_spin")
    left_arm_pivot = object_model.get_articulation("left_arm_pivot")
    right_arm_pivot = object_model.get_articulation("right_arm_pivot")
    selector_turn = object_model.get_articulation("selector_turn")

    ctx.allow_overlap(
        lid,
        pusher,
        elem_a="chute_shell",
        elem_b="pusher_shaft",
        reason="The feed pusher is intentionally represented as a simplified guided member nested inside the chute shell.",
    )
    ctx.allow_overlap(
        body,
        left_arm,
        elem_a="shoulder_0",
        elem_b="pivot_hub",
        reason="The left locking arm pivot hub is intentionally nested into the shoulder mount at the hinge pin.",
    )
    ctx.allow_overlap(
        body,
        right_arm,
        elem_a="shoulder_1",
        elem_b="pivot_hub",
        reason="The right locking arm pivot hub is intentionally nested into the shoulder mount at the hinge pin.",
    )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_cover",
        elem_b="chamber_ring",
        min_overlap=0.16,
        name="lid spans the chamber opening",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_cover",
        negative_elem="chamber_ring",
        max_gap=0.012,
        max_penetration=0.0,
        name="lid rests just above the chamber ring",
    )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_shaft",
        outer_elem="chute_shell",
        margin=0.002,
        name="pusher stays centered in the chute at rest",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_shaft",
        elem_b="chute_shell",
        min_overlap=0.12,
        name="pusher remains inserted in the chute at rest",
    )

    rest_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.11}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_shaft",
            outer_elem="chute_shell",
            margin=0.002,
            name="pusher stays centered in the chute when lifted",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_shaft",
            elem_b="chute_shell",
            min_overlap=0.10,
            name="pusher remains retained when lifted",
        )
        lifted_pusher_pos = ctx.part_world_position(pusher)
    ctx.check(
        "pusher lifts upward",
        rest_pusher_pos is not None
        and lifted_pusher_pos is not None
        and lifted_pusher_pos[2] > rest_pusher_pos[2] + 0.09,
        details=f"rest={rest_pusher_pos}, lifted={lifted_pusher_pos}",
    )

    rest_lid_center = _center_from_aabb(ctx.part_element_world_aabb(lid, elem="lid_cover"))
    with ctx.pose({lid_hinge: 1.0}):
        open_lid_center = _center_from_aabb(ctx.part_element_world_aabb(lid, elem="lid_cover"))
    ctx.check(
        "lid opens upward on rear hinge",
        rest_lid_center is not None
        and open_lid_center is not None
        and open_lid_center[2] > rest_lid_center[2] + 0.07
        and open_lid_center[0] < rest_lid_center[0] - 0.03,
        details=f"rest={rest_lid_center}, open={open_lid_center}",
    )

    rest_left_hook = _center_from_aabb(ctx.part_element_world_aabb(left_arm, elem="left_hook"))
    rest_right_hook = _center_from_aabb(ctx.part_element_world_aabb(right_arm, elem="right_hook"))
    with ctx.pose({left_arm_pivot: 0.82, right_arm_pivot: 0.82}):
        open_left_hook = _center_from_aabb(ctx.part_element_world_aabb(left_arm, elem="left_hook"))
        open_right_hook = _center_from_aabb(ctx.part_element_world_aabb(right_arm, elem="right_hook"))
    ctx.check(
        "locking arms swing outward from the shoulders",
        rest_left_hook is not None
        and rest_right_hook is not None
        and open_left_hook is not None
        and open_right_hook is not None
        and open_left_hook[1] > rest_left_hook[1] + 0.03
        and open_right_hook[1] < rest_right_hook[1] - 0.03
        and open_left_hook[2] < rest_left_hook[2] - 0.03
        and open_right_hook[2] < rest_right_hook[2] - 0.03,
        details=(
            f"left_rest={rest_left_hook}, left_open={open_left_hook}, "
            f"right_rest={rest_right_hook}, right_open={open_right_hook}"
        ),
    )

    rest_selector = _center_from_aabb(ctx.part_element_world_aabb(knob, elem="selector_tab"))
    with ctx.pose({selector_turn: 0.9}):
        turned_selector = _center_from_aabb(ctx.part_element_world_aabb(knob, elem="selector_tab"))
    ctx.check(
        "selector knob turns around its front shaft",
        rest_selector is not None
        and turned_selector is not None
        and abs(turned_selector[1] - rest_selector[1]) > 0.008,
        details=f"rest={rest_selector}, turned={turned_selector}",
    )

    rest_basket_tab = _center_from_aabb(ctx.part_element_world_aabb(basket, elem="basket_tab"))
    with ctx.pose({basket_spin: 1.2}):
        spun_basket_tab = _center_from_aabb(ctx.part_element_world_aabb(basket, elem="basket_tab"))
    ctx.check(
        "basket spins about the vertical axis",
        rest_basket_tab is not None
        and spun_basket_tab is not None
        and abs(spun_basket_tab[1] - rest_basket_tab[1]) > 0.01,
        details=f"rest={rest_basket_tab}, spun={spun_basket_tab}",
    )

    return ctx.report()


object_model = build_object_model()
