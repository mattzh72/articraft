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


BODY_W = 0.200
BODY_D = 0.140
BODY_H = 0.110
BODY_Z = BODY_H / 2.0
FRONT_Y = -BODY_D / 2.0
SIDE_X = BODY_W / 2.0


def _cylinder_between_y(radius: float, length: float, y0: float, z: float, x: float = 0.0):
    return cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(x, y0, z),
        cq.Vector(0.0, 1.0, 0.0),
    )


def _tube_between_y(
    outer_radius: float,
    inner_radius: float,
    length: float,
    y0: float,
    z: float,
    x: float = 0.0,
):
    outer = _cylinder_between_y(outer_radius, length, y0, z, x)
    inner = _cylinder_between_y(inner_radius, length + 0.002, y0 - 0.001, z, x)
    return outer.cut(inner)


def _build_body_shell():
    drawer_cut = (
        cq.Workplane("XY")
        .box(0.142, 0.110, 0.039)
        .translate((0.0, FRONT_Y + 0.054, 0.032))
    )
    port_cut = _cylinder_between_y(0.0145, 0.095, FRONT_Y - 0.012, 0.080)

    return (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_Z))
        .cut(drawer_cut)
        .cut(port_cut)
    )


def _build_port_bezel():
    return cq.Workplane(obj=_tube_between_y(0.025, 0.0145, 0.007, FRONT_Y - 0.006, 0.080))


def _build_drawer():
    panel_w = 0.132
    panel_t = 0.008
    panel_h = 0.040
    tray_w = 0.112
    tray_len = 0.087
    tray_y0 = -0.002
    wall_t = 0.004
    bottom_t = 0.005
    bottom_z = 0.023
    wall_z = 0.034

    front_panel = (
        cq.Workplane("XY")
        .box(panel_w, panel_t, panel_h)
        .translate((0.0, -panel_t / 2.0, 0.033))
    )
    bottom = (
        cq.Workplane("XY")
        .box(tray_w, tray_len, bottom_t)
        .translate((0.0, tray_y0 + tray_len / 2.0, bottom_z))
    )
    side_a = (
        cq.Workplane("XY")
        .box(wall_t, tray_len, 0.028)
        .translate((tray_w / 2.0 - wall_t / 2.0, tray_y0 + tray_len / 2.0, wall_z))
    )
    side_b = (
        cq.Workplane("XY")
        .box(wall_t, tray_len, 0.028)
        .translate((-tray_w / 2.0 + wall_t / 2.0, tray_y0 + tray_len / 2.0, wall_z))
    )
    back = (
        cq.Workplane("XY")
        .box(tray_w, wall_t, 0.028)
        .translate((0.0, tray_y0 + tray_len - wall_t / 2.0, wall_z))
    )

    pull = (
        cq.Workplane("XY")
        .box(0.054, 0.004, 0.014)
        .translate((0.0, -panel_t - 0.0008, 0.033))
    )

    return front_panel.union(bottom).union(side_a).union(side_b).union(back).union(pull)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_pencil_sharpener")

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    satin_black = model.material("satin_black", rgba=(0.01, 0.012, 0.014, 1.0))
    warm_steel = model.material("warm_steel", rgba=(0.86, 0.80, 0.68, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "body_shell", tolerance=0.0008),
        material=brushed_aluminum,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_build_port_bezel(), "port_bezel", tolerance=0.0006),
        material=dark_graphite,
        name="port_bezel",
    )
    body.visual(
        Box((0.158, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.002, 0.058)),
        material=dark_graphite,
        name="drawer_shadow_line",
    )
    body.visual(
        Box((0.014, 0.092, 0.008)),
        origin=Origin(xyz=(-0.050, -0.018, 0.052)),
        material=dark_graphite,
        name="guide_runner_0",
    )
    body.visual(
        Box((0.014, 0.092, 0.008)),
        origin=Origin(xyz=(0.050, -0.018, 0.052)),
        material=dark_graphite,
        name="guide_runner_1",
    )
    for i, (x, y) in enumerate(((-0.066, -0.046), (0.066, -0.046), (-0.066, 0.046), (0.066, 0.046))):
        body.visual(
            Box((0.030, 0.022, 0.006)),
            origin=Origin(xyz=(x, y, -0.002)),
            material=rubber,
            name=f"foot_{i}",
        )
    body.visual(
        Cylinder(radius=0.022, length=0.011),
        origin=Origin(xyz=(SIDE_X + 0.0035, 0.0, 0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_graphite,
        name="shaft_bushing",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_build_drawer(), "drawer", tolerance=0.0006),
        material=dark_graphite,
        name="drawer_shell",
    )

    crank_hub = model.part("crank_hub")
    crank_hub.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_steel,
        name="hub_disk",
    )
    crank_hub.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_graphite,
        name="hub_cap",
    )
    crank_hub.visual(
        Box((0.016, 0.035, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, -0.010)),
        material=warm_steel,
        name="hinge_lug",
    )
    crank_hub.visual(
        Box((0.010, 0.006, 0.019)),
        origin=Origin(xyz=(0.025, -0.0145, -0.025)),
        material=warm_steel,
        name="hinge_fork_0",
    )
    crank_hub.visual(
        Box((0.010, 0.006, 0.019)),
        origin=Origin(xyz=(0.025, 0.0145, -0.025)),
        material=warm_steel,
        name="hinge_fork_1",
    )

    crank_arm = model.part("crank_arm")
    crank_arm.visual(
        Cylinder(radius=0.0065, length=0.023),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=warm_steel,
        name="hinge_barrel",
    )
    crank_arm.visual(
        Box((0.008, 0.012, 0.047)),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=warm_steel,
        name="folding_arm",
    )
    crank_arm.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="finger_grip",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.002, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.18, lower=0.0, upper=0.052),
    )
    model.articulation(
        "body_to_crank_hub",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank_hub,
        origin=Origin(xyz=(SIDE_X + 0.009, 0.0, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "hub_to_crank_arm",
        ArticulationType.REVOLUTE,
        parent=crank_hub,
        child=crank_arm,
        origin=Origin(xyz=(0.025, 0.0, -0.025)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank_hub = object_model.get_part("crank_hub")
    crank_arm = object_model.get_part("crank_arm")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    hub_spin = object_model.get_articulation("body_to_crank_hub")
    arm_fold = object_model.get_articulation("hub_to_crank_arm")

    ctx.expect_within(
        drawer,
        body,
        axes="xz",
        inner_elem="drawer_shell",
        outer_elem="body_shell",
        name="drawer stays inside front opening in x and z",
    )
    ctx.expect_contact(
        body,
        drawer,
        elem_a="guide_runner_0",
        elem_b="drawer_shell",
        contact_tol=0.002,
        name="drawer bears on one guide runner",
    )
    ctx.expect_contact(
        body,
        drawer,
        elem_a="guide_runner_1",
        elem_b="drawer_shell",
        contact_tol=0.002,
        name="drawer bears on other guide runner",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.052}):
        ctx.expect_within(
            drawer,
            body,
            axes="xz",
            inner_elem="drawer_shell",
            outer_elem="body_shell",
            name="extended drawer remains aligned in x and z",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="drawer_shell",
            elem_b="body_shell",
            min_overlap=0.030,
            name="extended drawer retains guide insertion",
        )
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides straight out of front",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[1] < drawer_rest[1] - 0.045
        and abs(drawer_extended[0] - drawer_rest[0]) < 1e-6,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    ctx.expect_contact(
        crank_hub,
        crank_arm,
        elem_a="hinge_fork_0",
        elem_b="hinge_barrel",
        contact_tol=0.001,
        name="folding hinge barrel meets first fork",
    )
    ctx.expect_contact(
        crank_hub,
        crank_arm,
        elem_a="hinge_fork_1",
        elem_b="hinge_barrel",
        contact_tol=0.001,
        name="folding hinge barrel meets second fork",
    )

    arm_rest_aabb = ctx.part_world_aabb(crank_arm)
    with ctx.pose({arm_fold: 1.35}):
        arm_unfolded_aabb = ctx.part_world_aabb(crank_arm)
    ctx.check(
        "crank arm folds outward on hinge",
        arm_rest_aabb is not None
        and arm_unfolded_aabb is not None
        and arm_unfolded_aabb[1][0] > arm_rest_aabb[1][0] + 0.040,
        details=f"rest_aabb={arm_rest_aabb}, unfolded_aabb={arm_unfolded_aabb}",
    )

    arm_rest_pos = ctx.part_world_position(crank_arm)
    with ctx.pose({hub_spin: pi / 2.0}):
        arm_rotated_pos = ctx.part_world_position(crank_arm)
    ctx.check(
        "crank hub carries arm around side shaft",
        arm_rest_pos is not None
        and arm_rotated_pos is not None
        and arm_rotated_pos[1] > arm_rest_pos[1] + 0.020
        and arm_rotated_pos[2] > arm_rest_pos[2] + 0.020,
        details=f"rest={arm_rest_pos}, rotated={arm_rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
