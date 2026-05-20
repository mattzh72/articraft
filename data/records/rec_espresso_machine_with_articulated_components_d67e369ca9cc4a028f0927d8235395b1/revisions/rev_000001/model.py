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


def _body_shell_mesh():
    base = cq.Workplane("XY").box(0.300, 0.260, 0.032).translate((0.000, 0.000, 0.016))
    base = base.edges("|Z").fillet(0.016)

    tower = cq.Workplane("XY").box(0.220, 0.180, 0.332).translate((0.000, 0.000, 0.194))
    tower = tower.edges("|Z").fillet(0.020)

    shell = base.union(tower)
    reservoir_cut = cq.Workplane("XY").box(0.104, 0.122, 0.110).translate((-0.045, 0.000, 0.318))
    shell = shell.cut(reservoir_cut)
    tray_cut = cq.Workplane("XY").box(0.132, 0.112, 0.024).translate((0.080, 0.000, 0.044))
    shell = shell.cut(tray_cut)
    front_relief = cq.Workplane("XY").box(0.040, 0.074, 0.170).translate((0.102, 0.000, 0.205))
    shell = shell.cut(front_relief)
    return mesh_from_cadquery(shell, "espresso_body_shell")


def _drip_tray_mesh():
    outer = cq.Workplane("XY").box(0.125, 0.105, 0.018)
    inner = cq.Workplane("XY").box(0.109, 0.089, 0.014).translate((0.000, 0.000, 0.004))
    tray = outer.cut(inner)
    return mesh_from_cadquery(tray, "espresso_drip_tray")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lever_espresso_machine")

    shell_metal = model.material("shell_metal", rgba=(0.79, 0.80, 0.82, 1.0))
    polished_metal = model.material("polished_metal", rgba=(0.86, 0.87, 0.89, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.12, 0.12, 0.13, 1.0))
    tray_black = model.material("tray_black", rgba=(0.16, 0.16, 0.17, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=shell_metal, name="shell")
    body.visual(
        Cylinder(radius=0.028, length=0.068),
        origin=Origin(xyz=(0.144, 0.000, 0.195), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=polished_metal,
        name="group_head",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.022),
        origin=Origin(xyz=(0.139, 0.000, 0.195), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=polished_metal,
        name="group_ring",
    )
    body.visual(
        Box((0.020, 0.090, 0.070)),
        origin=Origin(xyz=(0.100, 0.000, 0.195)),
        material=polished_metal,
        name="group_mount",
    )
    for index, offset_y in enumerate((-0.037, 0.037)):
        body.visual(
            Box((0.020, 0.016, 0.058)),
            origin=Origin(xyz=(0.100, offset_y, 0.279)),
            material=polished_metal,
            name=f"lever_support_{index}",
        )
    body.visual(
        Box((0.024, 0.092, 0.014)),
        origin=Origin(xyz=(0.097, 0.000, 0.306)),
        material=polished_metal,
        name="lever_bridge",
    )
    body.visual(
        Box((0.040, 0.040, 0.055)),
        origin=Origin(xyz=(0.020, 0.110, 0.210)),
        material=polished_metal,
        name="wand_bracket",
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(_drip_tray_mesh(), material=tray_black, name="tray")
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.FIXED,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.080, 0.000, 0.041)),
    )

    brew_lever = model.part("brew_lever")
    brew_lever.visual(
        Cylinder(radius=0.006, length=0.056),
        origin=Origin(rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=dark_polymer,
        name="hub",
    )
    brew_lever.visual(
        Box((0.016, 0.020, 0.120)),
        origin=Origin(xyz=(0.012, 0.000, 0.060)),
        material=dark_polymer,
        name="arm",
    )
    brew_lever.visual(
        Cylinder(radius=0.012, length=0.170),
        origin=Origin(xyz=(0.018, 0.000, 0.165)),
        material=dark_polymer,
        name="grip",
    )
    brew_lever.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(xyz=(0.018, 0.000, 0.265)),
        material=dark_polymer,
        name="knob",
    )
    model.articulation(
        "body_to_brew_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=brew_lever,
        origin=Origin(xyz=(0.116, 0.000, 0.291)),
        axis=(0.000, 1.000, 0.000),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.8,
            lower=0.000,
            upper=1.15,
        ),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.029, length=0.034),
        origin=Origin(xyz=(0.000, 0.000, -0.020)),
        material=polished_metal,
        name="basket",
    )
    portafilter.visual(
        Box((0.012, 0.006, 0.004)),
        origin=Origin(xyz=(0.000, -0.021, -0.001)),
        material=polished_metal,
        name="lug_0",
    )
    portafilter.visual(
        Box((0.012, 0.006, 0.004)),
        origin=Origin(xyz=(0.000, 0.021, -0.001)),
        material=polished_metal,
        name="lug_1",
    )
    portafilter.visual(
        Box((0.020, 0.012, 0.012)),
        origin=Origin(xyz=(0.015, 0.000, -0.020)),
        material=polished_metal,
        name="neck",
    )
    portafilter.visual(
        Cylinder(radius=0.010, length=0.110),
        origin=Origin(xyz=(0.075, 0.000, -0.020), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=dark_polymer,
        name="handle",
    )
    portafilter.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.132, 0.000, -0.020), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=dark_polymer,
        name="handle_cap",
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.151, 0.000, 0.164)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=3.0,
            lower=-0.90,
            upper=0.90,
        ),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, -0.009)),
        material=polished_metal,
        name="pivot",
    )
    steam_wand.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(xyz=(0.016, 0.000, -0.012), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=polished_metal,
        name="elbow",
    )
    steam_wand.visual(
        Cylinder(radius=0.0045, length=0.170),
        origin=Origin(xyz=(0.032, 0.000, -0.097)),
        material=polished_metal,
        name="wand",
    )
    steam_wand.visual(
        Cylinder(radius=0.0035, length=0.018),
        origin=Origin(xyz=(0.032, 0.000, -0.191)),
        material=polished_metal,
        name="tip",
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.020, 0.137, 0.229)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-1.10,
            upper=0.60,
        ),
    )

    reservoir_lid = model.part("reservoir_lid")
    reservoir_lid.visual(
        Cylinder(radius=0.004, length=0.112),
        origin=Origin(xyz=(0.000, 0.000, 0.004), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=polished_metal,
        name="hinge_barrel",
    )
    reservoir_lid.visual(
        Box((0.104, 0.124, 0.006)),
        origin=Origin(xyz=(0.052, 0.000, 0.003)),
        material=shell_metal,
        name="panel",
    )
    reservoir_lid.visual(
        Box((0.010, 0.052, 0.010)),
        origin=Origin(xyz=(0.103, 0.000, 0.005)),
        material=dark_polymer,
        name="pull",
    )
    model.articulation(
        "body_to_reservoir_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=reservoir_lid,
        origin=Origin(xyz=(-0.097, 0.000, 0.360)),
        axis=(0.000, -1.000, 0.000),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.000,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drip_tray = object_model.get_part("drip_tray")
    brew_lever = object_model.get_part("brew_lever")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    reservoir_lid = object_model.get_part("reservoir_lid")

    lever_joint = object_model.get_articulation("body_to_brew_lever")
    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_steam_wand")
    lid_joint = object_model.get_articulation("body_to_reservoir_lid")

    ctx.allow_overlap(
        body,
        drip_tray,
        elem_a="shell",
        elem_b="tray",
        reason="The drip tray is intentionally represented as nested into the recessed base opening of the housing shell.",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            (lower[0] + upper[0]) * 0.5,
            (lower[1] + upper[1]) * 0.5,
            (lower[2] + upper[2]) * 0.5,
        )

    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        positive_elem="group_ring",
        negative_elem="basket",
        max_penetration=0.001,
        max_gap=0.012,
        name="portafilter seats just beneath the group head",
    )
    ctx.expect_overlap(
        drip_tray,
        portafilter,
        axes="xy",
        elem_a="tray",
        elem_b="basket",
        min_overlap=0.020,
        name="drip tray remains under the brewing position",
    )
    ctx.expect_gap(
        reservoir_lid,
        body,
        axis="z",
        positive_elem="panel",
        negative_elem="shell",
        min_gap=0.000,
        max_gap=0.010,
        name="closed reservoir lid sits on the rear deck",
    )
    ctx.expect_overlap(
        reservoir_lid,
        body,
        axes="xy",
        elem_a="panel",
        elem_b="shell",
        min_overlap=0.060,
        name="closed lid covers the reservoir opening footprint",
    )

    lever_rest = aabb_center(ctx.part_element_world_aabb(brew_lever, elem="grip"))
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        lever_lowered = aabb_center(ctx.part_element_world_aabb(brew_lever, elem="grip"))
    ctx.check(
        "brew lever pulls forward and downward",
        lever_rest is not None
        and lever_lowered is not None
        and lever_lowered[0] > lever_rest[0] + 0.12
        and lever_lowered[2] < lever_rest[2] - 0.10,
        details=f"rest={lever_rest}, lowered={lever_lowered}",
    )

    handle_rest = aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    with ctx.pose({portafilter_joint: 0.75}):
        handle_turned = aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    ctx.check(
        "portafilter rotates into and out of lock around the brew axis",
        handle_rest is not None
        and handle_turned is not None
        and handle_turned[1] > handle_rest[1] + 0.05
        and handle_turned[0] < handle_rest[0] - 0.02,
        details=f"rest={handle_rest}, turned={handle_turned}",
    )

    tip_inboard = None
    tip_outboard = None
    with ctx.pose({wand_joint: wand_joint.motion_limits.lower}):
        tip_inboard = aabb_center(ctx.part_element_world_aabb(steam_wand, elem="tip"))
    with ctx.pose({wand_joint: wand_joint.motion_limits.upper}):
        tip_outboard = aabb_center(ctx.part_element_world_aabb(steam_wand, elem="tip"))
    ctx.check(
        "steam wand swings across the side of the machine",
        tip_inboard is not None
        and tip_outboard is not None
        and tip_outboard[1] > tip_inboard[1] + 0.035
        and tip_outboard[0] > tip_inboard[0] + 0.008,
        details=f"inboard={tip_inboard}, outboard={tip_outboard}",
    )

    lid_closed = aabb_center(ctx.part_element_world_aabb(reservoir_lid, elem="panel"))
    with ctx.pose({lid_joint: lid_joint.motion_limits.upper}):
        lid_open = aabb_center(ctx.part_element_world_aabb(reservoir_lid, elem="panel"))
    ctx.check(
        "reservoir lid opens upward from the rear hinge",
        lid_closed is not None
        and lid_open is not None
        and lid_open[2] > lid_closed[2] + 0.030
        and lid_open[0] < lid_closed[0] - 0.020,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    return ctx.report()


object_model = build_object_model()
