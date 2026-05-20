from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _build_base_housing() -> cq.Workplane:
    floor = cq.Workplane("XY").box(0.34, 0.26, 0.02).translate((0.0, 0.0, 0.01))
    left_cheek = (
        cq.Workplane("XY")
        .box(0.23, 0.05, 0.11)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.02, 0.105, 0.075))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.23, 0.05, 0.11)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.02, -0.105, 0.075))
    )
    front_wall = (
        cq.Workplane("XY")
        .box(0.07, 0.16, 0.11)
        .edges("|Z")
        .fillet(0.016)
        .translate((0.135, 0.0, 0.075))
    )
    rear_left = cq.Workplane("XY").box(0.16, 0.04, 0.11).translate((-0.09, 0.11, 0.075))
    rear_right = cq.Workplane("XY").box(0.16, 0.04, 0.11).translate((-0.09, -0.11, 0.075))
    rear_bridge = cq.Workplane("XY").box(0.12, 0.20, 0.04).translate((-0.11, 0.0, 0.14))
    ring_outer = (
        cq.Workplane("XY")
        .center(0.025, 0.0)
        .circle(0.108)
        .extrude(0.036)
        .translate((0.0, 0.0, 0.13))
    )
    ring_inner = (
        cq.Workplane("XY")
        .center(0.025, 0.0)
        .circle(0.074)
        .extrude(0.04)
        .translate((0.0, 0.0, 0.128))
    )
    chamber_ring = ring_outer.cut(ring_inner)
    return (
        floor.union(left_cheek)
        .union(right_cheek)
        .union(front_wall)
        .union(rear_left)
        .union(rear_right)
        .union(rear_bridge)
        .union(chamber_ring)
    )


def _build_lid_shell() -> cq.Workplane:
    outer_cover = cq.Workplane("XY").center(0.105, 0.0).circle(0.108).extrude(0.058)
    hinge_bridge = cq.Workplane("XY").box(0.034, 0.09, 0.024).translate((0.017, 0.0, 0.018))
    front_tongue = cq.Workplane("XY").box(0.055, 0.09, 0.02).translate((0.188, 0.0, 0.018))
    cover = outer_cover.union(hinge_bridge).union(front_tongue)

    inner_cover = (
        cq.Workplane("XY")
        .center(0.105, 0.0)
        .circle(0.101)
        .extrude(0.052)
        .translate((0.0, 0.0, 0.004))
    )
    chute_aperture = (
        cq.Workplane("XY")
        .center(0.115, 0.0)
        .circle(0.037)
        .extrude(0.075)
        .translate((0.0, 0.0, -0.002))
    )
    return cover.cut(inner_cover).cut(chute_aperture)


def _build_chute_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.043).extrude(0.145)
    flange = cq.Workplane("XY").circle(0.05).extrude(0.014)
    inner = cq.Workplane("XY").circle(0.036).extrude(0.16).translate((0.0, 0.0, -0.005))
    return outer.union(flange).cut(inner)


def _build_pulp_bin() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.15, 0.112, 0.096).translate((-0.075, 0.0, 0.048))
    inner = cq.Workplane("XY").box(0.14, 0.102, 0.086).translate((-0.075, 0.0, 0.052))
    handle_cut = cq.Workplane("XY").box(0.018, 0.06, 0.024).translate((-0.149, 0.0, 0.07))
    return outer.cut(inner).cut(handle_cut)


def _build_basket_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.06).extrude(0.072)
    inner = cq.Workplane("XY").circle(0.054).extrude(0.062).translate((0.0, 0.0, 0.01))
    rim = cq.Workplane("XY").circle(0.065).extrude(0.006).translate((0.0, 0.0, 0.072))
    bore = cq.Workplane("XY").circle(0.02).extrude(0.014).translate((0.0, 0.0, -0.001))
    hub = cq.Workplane("XY").circle(0.018).extrude(0.034)
    return outer.cut(inner).cut(bore).union(rim).union(hub)


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_centrifugal_juicer")

    body_plastic = model.material("body_plastic", rgba=(0.16, 0.18, 0.19, 1.0))
    trim_steel = model.material("trim_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    clear_lid = model.material("clear_lid", rgba=(0.80, 0.90, 0.94, 0.35))
    pusher_plastic = model.material("pusher_plastic", rgba=(0.23, 0.23, 0.24, 1.0))
    bin_smoke = model.material("bin_smoke", rgba=(0.28, 0.30, 0.32, 0.78))
    dial_black = model.material("dial_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dial_mark = model.material("dial_mark", rgba=(0.86, 0.22, 0.12, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_build_base_housing(), "juicer_base_housing"), material=body_plastic, name="base_housing")
    base.visual(
        Box((0.06, 0.05, 0.022)),
        origin=Origin(xyz=(0.155, 0.0, 0.124)),
        material=trim_steel,
        name="juice_spout",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.032),
        origin=Origin(xyz=(0.025, 0.0, 0.036)),
        material=trim_steel,
        name="drive_coupling",
    )
    base.visual(
        Box((0.135, 0.024, 0.014)),
        origin=Origin(xyz=(-0.1025, 0.068, 0.027)),
        material=trim_steel,
        name="bin_rail_0",
    )
    base.visual(
        Box((0.135, 0.024, 0.014)),
        origin=Origin(xyz=(-0.1025, -0.068, 0.027)),
        material=trim_steel,
        name="bin_rail_1",
    )
    base.visual(
        Box((0.028, 0.09, 0.03)),
        origin=Origin(xyz=(-0.1, 0.0, 0.179)),
        material=trim_steel,
        name="hinge_block",
    )
    base.visual(
        Box((0.028, 0.05, 0.028)),
        origin=Origin(xyz=(-0.1, 0.0, 0.166)),
        material=trim_steel,
        name="hinge_support",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.034),
        origin=Origin(xyz=(-0.095, 0.0, 0.188), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_steel,
        name="hinge_barrel",
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_build_lid_shell(), "juicer_lid_shell"), material=clear_lid, name="lid_shell")
    lid.visual(
        mesh_from_cadquery(_build_chute_shell(), "juicer_lid_chute"),
        origin=Origin(xyz=(0.115, 0.0, 0.033)),
        material=clear_lid,
        name="lid_chute",
    )
    lid.visual(
        Cylinder(radius=0.0075, length=0.02),
        origin=Origin(xyz=(0.0, -0.028, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_steel,
        name="lid_barrel_0",
    )
    lid.visual(
        Cylinder(radius=0.0075, length=0.02),
        origin=Origin(xyz=(0.0, 0.028, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_steel,
        name="lid_barrel_1",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.031, length=0.165),
        origin=Origin(xyz=(0.0, 0.0, -0.0825)),
        material=pusher_plastic,
        name="pusher_shaft",
    )
    pusher.visual(
        Cylinder(radius=0.041, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=pusher_plastic,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=pusher_plastic,
        name="pusher_knob",
    )

    basket = model.part("basket")
    basket.visual(
        Cylinder(radius=0.058, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=trim_steel,
        name="basket_body",
    )
    basket.visual(Cylinder(radius=0.018, length=0.034), origin=Origin(xyz=(0.0, 0.0, 0.017)), material=trim_steel, name="basket_hub")
    basket.visual(Cylinder(radius=0.064, length=0.006), origin=Origin(xyz=(0.0, 0.0, 0.079)), material=trim_steel, name="basket_rim")
    basket.visual(Cylinder(radius=0.009, length=0.034), origin=Origin(xyz=(0.0, 0.0, -0.017)), material=trim_steel, name="basket_spindle")
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        basket.visual(
            Box((0.05, 0.01, 0.004)),
            origin=Origin(xyz=(0.026, 0.0, 0.075), rpy=(0.0, 0.0, angle)),
            material=trim_steel,
            name="basket_blade" if i == 0 else f"basket_blade_{i}",
        )

    pulp_bin = model.part("pulp_bin")
    pulp_bin.visual(Box((0.15, 0.112, 0.006)), origin=Origin(xyz=(-0.075, 0.0, 0.003)), material=bin_smoke, name="bin_bottom")
    pulp_bin.visual(Box((0.15, 0.005, 0.088)), origin=Origin(xyz=(-0.075, 0.0535, 0.05)), material=bin_smoke, name="bin_side_0")
    pulp_bin.visual(Box((0.15, 0.005, 0.088)), origin=Origin(xyz=(-0.075, -0.0535, 0.05)), material=bin_smoke, name="bin_side_1")
    pulp_bin.visual(Box((0.005, 0.112, 0.088)), origin=Origin(xyz=(-0.0025, 0.0, 0.05)), material=bin_smoke, name="bin_front")
    pulp_bin.visual(Box((0.005, 0.112, 0.04)), origin=Origin(xyz=(-0.1475, 0.0, 0.023)), material=bin_smoke, name="bin_rear_lower")
    pulp_bin.visual(Box((0.005, 0.026, 0.034)), origin=Origin(xyz=(-0.1475, 0.043, 0.071)), material=bin_smoke, name="bin_handle_0")
    pulp_bin.visual(Box((0.005, 0.026, 0.034)), origin=Origin(xyz=(-0.1475, -0.043, 0.071)), material=bin_smoke, name="bin_handle_1")
    pulp_bin.visual(Box((0.005, 0.06, 0.01)), origin=Origin(xyz=(-0.1475, 0.0, 0.087)), material=bin_smoke, name="bin_handle_top")

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_steel,
        name="dial_shaft",
    )
    speed_dial.visual(
        mesh_from_geometry(KnobGeometry(0.046, 0.024, body_style="skirted", top_diameter=0.034), "juicer_speed_dial"),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_black,
        name="dial_body",
    )
    speed_dial.visual(
        Box((0.008, 0.004, 0.014)),
        origin=Origin(xyz=(0.03, 0.0, 0.011)),
        material=dial_mark,
        name="dial_pointer",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.078, 0.0, 0.166)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.2, lower=0.0, upper=1.15),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.115, 0.0, 0.178)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.20, lower=0.0, upper=0.10),
    )
    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.025, 0.0, 0.086)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )
    model.articulation(
        "base_to_pulp_bin",
        ArticulationType.PRISMATIC,
        parent=base,
        child=pulp_bin,
        origin=Origin(xyz=(-0.038, 0.0, 0.021)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=0.105),
    )
    model.articulation(
        "base_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(0.17, 0.05, 0.076)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-1.6, upper=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    pulp_bin = object_model.get_part("pulp_bin")
    speed_dial = object_model.get_part("speed_dial")

    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    basket_spin = object_model.get_articulation("base_to_basket")
    bin_slide = object_model.get_articulation("base_to_pulp_bin")
    dial_turn = object_model.get_articulation("base_to_speed_dial")

    with ctx.pose({lid_hinge: 0.0, pusher_slide: 0.0, bin_slide: 0.0, dial_turn: 0.0, basket_spin: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="base_housing",
            max_gap=0.004,
            max_penetration=0.001,
            name="lid seats on the chamber collar",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="lid_shell",
            elem_b="base_housing",
            min_overlap=0.16,
            name="lid covers the chamber opening",
        )
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_shaft",
            outer_elem="lid_chute",
            margin=0.002,
            name="pusher stays centered in the chute",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_shaft",
            elem_b="lid_chute",
            min_overlap=0.14,
            name="pusher remains inserted when lowered",
        )
        ctx.expect_within(
            pulp_bin,
            base,
            axes="yz",
            outer_elem="base_housing",
            margin=0.01,
            name="pulp bin stays nested in the rear bay",
        )

        ctx.expect_within(
            pulp_bin,
            base,
            axes="y",
            outer_elem="base_housing",
            margin=0.01,
            name="pulp bin stays centered between the rear rails",
        )

        closed_lid_center = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_shell"))
        rest_pusher_pos = ctx.part_world_position(pusher)
        rest_bin_pos = ctx.part_world_position(pulp_bin)
        rest_blade_center = _aabb_center(ctx.part_element_world_aabb(basket, elem="basket_blade"))
        rest_pointer_center = _aabb_center(ctx.part_element_world_aabb(speed_dial, elem="dial_pointer"))

    with ctx.pose({lid_hinge: 1.0}):
        open_lid_center = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_shell"))
    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_center is not None
        and open_lid_center is not None
        and open_lid_center[2] > closed_lid_center[2] + 0.07,
        details=f"closed={closed_lid_center}, open={open_lid_center}",
    )

    with ctx.pose({pusher_slide: 0.10}):
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_shaft",
            elem_b="lid_chute",
            min_overlap=0.06,
            name="pusher retains insertion when raised",
        )
        raised_pusher_pos = ctx.part_world_position(pusher)
    ctx.check(
        "pusher lifts upward through the chute",
        rest_pusher_pos is not None
        and raised_pusher_pos is not None
        and raised_pusher_pos[2] > rest_pusher_pos[2] + 0.08,
        details=f"rest={rest_pusher_pos}, raised={raised_pusher_pos}",
    )

    with ctx.pose({bin_slide: 0.105}):
        ctx.expect_overlap(
            pulp_bin,
            base,
            axes="x",
            elem_b="base_housing",
            min_overlap=0.025,
            name="pulp bin remains engaged at full extension",
        )
        extended_bin_pos = ctx.part_world_position(pulp_bin)
    ctx.check(
        "pulp bin slides rearward",
        rest_bin_pos is not None
        and extended_bin_pos is not None
        and extended_bin_pos[0] < rest_bin_pos[0] - 0.08,
        details=f"rest={rest_bin_pos}, extended={extended_bin_pos}",
    )

    with ctx.pose({basket_spin: math.pi / 2.0}):
        spun_blade_center = _aabb_center(ctx.part_element_world_aabb(basket, elem="basket_blade"))
    ctx.check(
        "basket spins around the vertical drive axis",
        rest_blade_center is not None
        and spun_blade_center is not None
        and abs(spun_blade_center[0] - rest_blade_center[0]) > 0.015
        and abs(spun_blade_center[1] - rest_blade_center[1]) > 0.015,
        details=f"rest={rest_blade_center}, spun={spun_blade_center}",
    )

    with ctx.pose({dial_turn: 1.2}):
        turned_pointer_center = _aabb_center(ctx.part_element_world_aabb(speed_dial, elem="dial_pointer"))
    ctx.check(
        "speed dial rotates on its shaft",
        rest_pointer_center is not None
        and turned_pointer_center is not None
        and abs(turned_pointer_center[1] - rest_pointer_center[1]) > 0.006,
        details=f"rest={rest_pointer_center}, turned={turned_pointer_center}",
    )

    return ctx.report()


object_model = build_object_model()
