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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_DEPTH = 0.29
BODY_WIDTH = 0.155
BODY_HEIGHT = 0.255

BREW_LID_DEPTH = 0.115
BREW_LID_WIDTH = 0.118
BREW_LID_THICKNESS = 0.012
BREW_HINGE_X = -0.005

TANK_LID_DEPTH = 0.085
TANK_LID_WIDTH = 0.096
TANK_LID_THICKNESS = 0.010
TANK_HINGE_X = -0.125

TRAY_ORIGIN_X = 0.035
TRAY_FLOOR_Z = 0.010
TRAY_TRAVEL = 0.045

WAND_PIVOT_X = 0.045
WAND_PIVOT_Y = BODY_WIDTH / 2.0 + 0.023
WAND_PIVOT_Z = 0.180


def _box_xy(size: tuple[float, float, float], *, center_xy: tuple[float, float], z0: float) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy = center_xy
    return cq.Workplane("XY").transformed(offset=(cx, cy, z0)).box(
        sx,
        sy,
        sz,
        centered=(True, True, False),
    )


def _body_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
    shell = shell.edges("|Z").fillet(0.012)

    brew_head = _box_xy((0.072, 0.118, 0.112), center_xy=(0.098, 0.0), z0=0.082)
    brew_head = brew_head.edges("|Z").fillet(0.018)
    brew_head = brew_head.edges(">Z").fillet(0.009)
    tank_shoulder = _box_xy((0.082, 0.122, 0.052), center_xy=(-0.100, 0.0), z0=0.184)
    tank_shoulder = tank_shoulder.edges("|Z").fillet(0.014)
    tank_shoulder = tank_shoulder.edges(">Z").fillet(0.008)
    shell = shell.union(brew_head).union(tank_shoulder)

    shell = shell.cut(_box_xy((0.140, 0.110, 0.040), center_xy=(0.080, 0.0), z0=TRAY_FLOOR_Z))
    shell = shell.cut(_box_xy((0.135, 0.102, 0.112), center_xy=(0.0775, 0.0), z0=0.092))
    shell = shell.cut(_box_xy((0.092, 0.068, 0.014), center_xy=(0.046, 0.0), z0=BODY_HEIGHT - 0.014))
    shell = shell.cut(_box_xy((0.070, 0.058, 0.014), center_xy=(-0.082, 0.0), z0=BODY_HEIGHT - 0.014))

    shell = shell.union(_box_xy((0.028, 0.020, 0.047), center_xy=(0.070, 0.0), z0=0.158))
    shell = shell.union(
        cq.Workplane("XY")
        .transformed(offset=(0.071, -0.007, 0.144))
        .circle(0.003)
        .extrude(0.015)
    )
    shell = shell.union(
        cq.Workplane("XY")
        .transformed(offset=(0.071, 0.007, 0.144))
        .circle(0.003)
        .extrude(0.015)
    )

    bridge = _box_xy((0.024, 0.008, 0.032), center_xy=(WAND_PIVOT_X, BODY_WIDTH / 2.0 + 0.004), z0=WAND_PIVOT_Z - 0.016)
    front_tab = _box_xy((0.006, 0.018, 0.032), center_xy=(WAND_PIVOT_X + 0.009, BODY_WIDTH / 2.0 + 0.009), z0=WAND_PIVOT_Z - 0.016)
    rear_tab = _box_xy((0.006, 0.018, 0.032), center_xy=(WAND_PIVOT_X - 0.009, BODY_WIDTH / 2.0 + 0.009), z0=WAND_PIVOT_Z - 0.016)
    shell = shell.union(bridge).union(front_tab).union(rear_tab)

    return shell


def _lid_shape(depth: float, width: float, thickness: float) -> cq.Workplane:
    panel = cq.Workplane("XY").box(depth, width, thickness, centered=(False, True, False))
    knuckle = (
        cq.Workplane("XZ")
        .center(0.0, thickness * 0.5)
        .circle(thickness * 0.46)
        .extrude(width * 0.88, both=True)
    )
    finger_lip = cq.Workplane("XY").transformed(offset=(depth - 0.010, 0.0, 0.0015)).box(
        0.020,
        width * 0.38,
        0.003,
        centered=(True, True, False),
    )
    return panel.union(knuckle).union(finger_lip)

def _aabb_center(aabb: object) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[idx] + hi[idx]) * 0.5 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="capsule_hybrid_espresso_appliance")

    model.material("body_black", rgba=(0.15, 0.16, 0.17, 1.0))
    model.material("lid_metal", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("tray_steel", rgba=(0.82, 0.82, 0.80, 1.0))
    model.material("wand_metal", rgba=(0.79, 0.80, 0.82, 1.0))
    model.material("wand_tip_dark", rgba=(0.22, 0.23, 0.24, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material="body_black",
        name="body_shell",
    )

    brew_lid = model.part("brew_lid")
    brew_lid.visual(
        mesh_from_cadquery(_lid_shape(BREW_LID_DEPTH, BREW_LID_WIDTH, BREW_LID_THICKNESS), "brew_lid"),
        material="lid_metal",
        name="brew_panel",
    )

    tank_lid = model.part("tank_lid")
    tank_lid.visual(
        mesh_from_cadquery(_lid_shape(TANK_LID_DEPTH, TANK_LID_WIDTH, TANK_LID_THICKNESS), "tank_lid"),
        material="lid_metal",
        name="tank_panel",
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.130, 0.108, 0.010)),
        origin=Origin(xyz=(0.043, 0.0, 0.005)),
        material="tray_steel",
        name="tray_shell",
    )
    drip_tray.visual(
        Box((0.016, 0.094, 0.028)),
        origin=Origin(xyz=(0.116, 0.0, 0.014)),
        material="tray_steel",
        name="tray_lip",
    )
    drip_tray.visual(
        Box((0.082, 0.078, 0.0025)),
        origin=Origin(xyz=(0.040, 0.0, 0.01125)),
        material="body_black",
        name="tray_grille",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="wand_metal",
        name="wand_shaft",
    )
    wand.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.0, 0.008, -0.008)),
        material="wand_metal",
        name="wand_collar",
    )
    wand.visual(
        Cylinder(radius=0.004, length=0.066),
        origin=Origin(xyz=(0.0, 0.010, -0.041)),
        material="wand_metal",
        name="wand_tube",
    )
    wand.visual(
        Cylinder(radius=0.0048, length=0.018),
        origin=Origin(xyz=(0.0, 0.010, -0.078)),
        material="wand_tip_dark",
        name="wand_tip",
    )

    model.articulation(
        "body_to_brew_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=brew_lid,
        origin=Origin(xyz=(BREW_HINGE_X, 0.0, BODY_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=1.28,
        ),
    )
    model.articulation(
        "body_to_tank_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tank_lid,
        origin=Origin(xyz=(TANK_HINGE_X, 0.0, BODY_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "body_to_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(WAND_PIVOT_X, WAND_PIVOT_Y, WAND_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(TRAY_ORIGIN_X, 0.0, TRAY_FLOOR_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.12,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    brew_lid = object_model.get_part("brew_lid")
    tank_lid = object_model.get_part("tank_lid")
    wand = object_model.get_part("wand")
    drip_tray = object_model.get_part("drip_tray")

    brew_joint = object_model.get_articulation("body_to_brew_lid")
    tank_joint = object_model.get_articulation("body_to_tank_lid")
    wand_joint = object_model.get_articulation("body_to_wand")
    tray_joint = object_model.get_articulation("body_to_drip_tray")

    ctx.allow_overlap(
        body,
        drip_tray,
        elem_a="body_shell",
        elem_b="tray_shell",
        reason="The lower body is authored as a continuous outer shell proxy around the internal drip-tray channel, while the tray shell stands in for the sliding insert riding inside that cavity.",
    )

    ctx.expect_gap(
        brew_lid,
        body,
        axis="z",
        positive_elem="brew_panel",
        negative_elem="body_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="brew lid sits flush on the brew deck",
    )
    ctx.expect_overlap(
        brew_lid,
        body,
        axes="xy",
        elem_a="brew_panel",
        elem_b="body_shell",
        min_overlap=0.09,
        name="brew lid covers the capsule opening footprint",
    )
    ctx.expect_gap(
        tank_lid,
        body,
        axis="z",
        positive_elem="tank_panel",
        negative_elem="body_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="water tank lid sits flush on the rear deck",
    )
    ctx.expect_overlap(
        tank_lid,
        body,
        axes="xy",
        elem_a="tank_panel",
        elem_b="body_shell",
        min_overlap=0.06,
        name="water tank lid covers the fill opening footprint",
    )
    ctx.expect_contact(
        wand,
        body,
        elem_a="wand_shaft",
        elem_b="body_shell",
        contact_tol=0.001,
        name="milk wand pivot is supported by the side clevis",
    )
    ctx.expect_contact(
        drip_tray,
        body,
        elem_a="tray_shell",
        elem_b="body_shell",
        contact_tol=0.001,
        name="drip tray rests on the lower guide floor",
    )
    ctx.expect_within(
        drip_tray,
        body,
        axes="yz",
        elem_a="tray_shell",
        elem_b="body_shell",
        margin=0.0,
        name="drip tray stays laterally contained at rest",
    )
    ctx.expect_overlap(
        drip_tray,
        body,
        axes="x",
        elem_a="tray_shell",
        elem_b="body_shell",
        min_overlap=0.085,
        name="drip tray remains inserted at rest",
    )

    brew_closed = ctx.part_element_world_aabb(brew_lid, elem="brew_panel")
    with ctx.pose({brew_joint: brew_joint.motion_limits.upper}):
        brew_open = ctx.part_element_world_aabb(brew_lid, elem="brew_panel")
    ctx.check(
        "brew lid flips upward",
        brew_closed is not None and brew_open is not None and brew_open[1][2] > brew_closed[1][2] + 0.045,
        details=f"closed={brew_closed}, open={brew_open}",
    )

    tank_closed = ctx.part_element_world_aabb(tank_lid, elem="tank_panel")
    with ctx.pose({tank_joint: tank_joint.motion_limits.upper}):
        tank_open = ctx.part_element_world_aabb(tank_lid, elem="tank_panel")
    ctx.check(
        "water tank lid flips upward",
        tank_closed is not None and tank_open is not None and tank_open[1][2] > tank_closed[1][2] + 0.030,
        details=f"closed={tank_closed}, open={tank_open}",
    )

    wand_tip_rest = _aabb_center(ctx.part_element_world_aabb(wand, elem="wand_tip"))
    with ctx.pose({wand_joint: wand_joint.motion_limits.upper}):
        wand_tip_out = _aabb_center(ctx.part_element_world_aabb(wand, elem="wand_tip"))
    ctx.check(
        "wand swings outward from the sidewall",
        wand_tip_rest is not None and wand_tip_out is not None and wand_tip_out[1] > wand_tip_rest[1] + 0.02,
        details=f"rest={wand_tip_rest}, out={wand_tip_out}",
    )

    tray_rest = ctx.part_world_position(drip_tray)
    with ctx.pose({tray_joint: tray_joint.motion_limits.upper}):
        tray_out = ctx.part_world_position(drip_tray)
        ctx.expect_within(
            drip_tray,
            body,
            axes="yz",
            elem_a="tray_shell",
            elem_b="body_shell",
            margin=0.0,
            name="extended drip tray stays laterally guided",
        )
        ctx.expect_overlap(
            drip_tray,
            body,
            axes="x",
            elem_a="tray_shell",
            elem_b="body_shell",
            min_overlap=0.040,
            name="extended drip tray keeps retained insertion",
        )
    ctx.check(
        "drip tray slides forward",
        tray_rest is not None and tray_out is not None and tray_out[0] > tray_rest[0] + 0.03,
        details=f"rest={tray_rest}, extended={tray_out}",
    )

    return ctx.report()


object_model = build_object_model()
