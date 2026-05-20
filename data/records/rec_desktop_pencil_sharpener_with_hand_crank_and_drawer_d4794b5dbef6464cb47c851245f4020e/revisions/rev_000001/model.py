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


def _cylinder_x(length: float, radius: float, center: tuple[float, float, float]):
    """CadQuery cylinder with its axis along the object X direction."""
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _make_housing_shell():
    """Rounded sharpener housing with real cut-through pencil and drawer openings."""
    shell = (
        cq.Workplane("XY")
        .box(0.170, 0.085, 0.095)
        .translate((0.0, 0.0, 0.0685))
        .edges("|Z")
        .fillet(0.006)
    )

    # Raised front boss gives the insertion hole believable wall thickness.
    front_boss = _cylinder_x(0.012, 0.024, (-0.089, 0.0, 0.092))
    shell = shell.union(front_boss)

    # Front waste-drawer tunnel, sized with clearance around the sliding tray.
    drawer_tunnel = (
        cq.Workplane("XY")
        .box(0.106, 0.066, 0.040)
        .translate((-0.039, 0.0, 0.051))
    )

    # Actual pencil through-opening leading into a larger internal cutter cavity.
    pencil_bore = _cylinder_x(0.150, 0.013, (-0.020, 0.0, 0.092))
    cutter_cavity = _cylinder_x(0.055, 0.018, (0.025, 0.0, 0.092))
    shaving_chute = (
        cq.Workplane("XY")
        .box(0.030, 0.030, 0.018)
        .translate((0.014, 0.0, 0.074))
    )

    return (
        shell.cut(drawer_tunnel)
        .cut(pencil_bore)
        .cut(cutter_cavity)
        .cut(shaving_chute)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="suction_base_pencil_sharpener")

    red_enamel = model.material("red_enamel", color=(0.68, 0.06, 0.04, 1.0))
    dark_metal = model.material("dark_metal", color=(0.08, 0.08, 0.075, 1.0))
    polished_metal = model.material("polished_metal", color=(0.55, 0.56, 0.54, 1.0))
    black_rubber = model.material("black_rubber", color=(0.015, 0.014, 0.012, 1.0))
    drawer_plastic = model.material("smoky_drawer_plastic", color=(0.20, 0.22, 0.23, 1.0))
    grip_material = model.material("warm_wood_grip", color=(0.72, 0.46, 0.22, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_housing_shell(), "housing_shell", tolerance=0.0008),
        material=red_enamel,
        name="housing_shell",
    )
    body.visual(
        Box((0.165, 0.095, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=polished_metal,
        name="base_plate",
    )
    body.visual(
        Cylinder(radius=0.052, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=black_rubber,
        name="suction_cup",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=black_rubber,
        name="suction_neck",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.020, 0.048, 0.080), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="side_bearing",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(-0.072, 0.0, 0.007)),
        material=dark_metal,
        name="pivot_boss",
    )
    body.visual(
        Box((0.016, 0.005, 0.038)),
        origin=Origin(xyz=(-0.092, 0.0365, 0.052)),
        material=dark_metal,
        name="drawer_guide_0",
    )
    body.visual(
        Box((0.016, 0.005, 0.038)),
        origin=Origin(xyz=(-0.092, -0.0365, 0.052)),
        material=dark_metal,
        name="drawer_guide_1",
    )

    waste_drawer = model.part("waste_drawer")
    waste_drawer.visual(
        Box((0.076, 0.054, 0.004)),
        origin=Origin(xyz=(0.038, 0.0, -0.012)),
        material=drawer_plastic,
        name="tray_bottom",
    )
    waste_drawer.visual(
        Box((0.076, 0.004, 0.024)),
        origin=Origin(xyz=(0.038, 0.029, 0.0)),
        material=drawer_plastic,
        name="tray_wall_0",
    )
    waste_drawer.visual(
        Box((0.076, 0.004, 0.024)),
        origin=Origin(xyz=(0.038, -0.029, 0.0)),
        material=drawer_plastic,
        name="tray_wall_1",
    )
    waste_drawer.visual(
        Box((0.004, 0.058, 0.024)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        material=drawer_plastic,
        name="tray_back",
    )
    waste_drawer.visual(
        Box((0.010, 0.068, 0.038)),
        origin=Origin(xyz=(-0.017, 0.0, 0.0)),
        material=drawer_plastic,
        name="drawer_front",
    )
    waste_drawer.visual(
        Box((0.022, 0.050, 0.006)),
        origin=Origin(xyz=(-0.001, 0.0, -0.010)),
        material=drawer_plastic,
        name="front_bridge",
    )
    waste_drawer.visual(
        Box((0.006, 0.038, 0.010)),
        origin=Origin(xyz=(-0.025, 0.0, 0.003)),
        material=dark_metal,
        name="drawer_pull",
    )

    side_crank = model.part("side_crank")
    side_crank.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="shaft",
    )
    side_crank.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    side_crank.visual(
        Box((0.012, 0.006, 0.060)),
        origin=Origin(xyz=(0.0, 0.026, -0.030)),
        material=dark_metal,
        name="crank_arm",
    )
    side_crank.visual(
        Cylinder(radius=0.010, length=0.032),
        origin=Origin(xyz=(0.0, 0.042, -0.060), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grip_material,
        name="grip",
    )

    lock_lever = model.part("lock_lever")
    lock_lever.visual(
        Cylinder(radius=0.011, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=dark_metal,
        name="lever_collar",
    )
    lock_lever.visual(
        Box((0.070, 0.020, 0.006)),
        origin=Origin(xyz=(-0.035, 0.0, -0.0045)),
        material=dark_metal,
        name="lever_paddle",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=waste_drawer,
        origin=Origin(xyz=(-0.085, 0.0, 0.052)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.22, lower=0.0, upper=0.055),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=side_crank,
        origin=Origin(xyz=(0.020, 0.054, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=7.0),
    )
    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_lever,
        origin=Origin(xyz=(-0.072, 0.0, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=-1.2, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("waste_drawer")
    crank = object_model.get_part("side_crank")
    lever = object_model.get_part("lock_lever")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    crank_joint = object_model.get_articulation("body_to_crank")
    lever_joint = object_model.get_articulation("body_to_lever")

    ctx.check(
        "crank uses continuous side-shaft rotation",
        crank_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(crank_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={crank_joint.articulation_type}, axis={crank_joint.axis}",
    )
    ctx.check(
        "front drawer is a retained prismatic slide",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(drawer_joint.axis) == (-1.0, 0.0, 0.0)
        and drawer_joint.motion_limits is not None
        and drawer_joint.motion_limits.upper >= 0.05,
        details=f"type={drawer_joint.articulation_type}, axis={drawer_joint.axis}, limits={drawer_joint.motion_limits}",
    )
    ctx.check(
        "underside lock lever is a limited revolute latch",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lever_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={lever_joint.articulation_type}, axis={lever_joint.axis}",
    )

    ctx.expect_contact(
        crank,
        body,
        elem_a="shaft",
        elem_b="side_bearing",
        contact_tol=0.0015,
        name="crank shaft seats at the side bearing",
    )
    ctx.expect_contact(
        lever,
        body,
        elem_a="lever_collar",
        elem_b="pivot_boss",
        contact_tol=0.0015,
        name="lock lever collar seats under pivot boss",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        inner_elem="tray_bottom",
        outer_elem="housing_shell",
        margin=0.004,
        name="drawer tray fits within the body tunnel",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="tray_bottom",
        elem_b="housing_shell",
        min_overlap=0.040,
        name="closed drawer remains well inserted",
    )

    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: drawer_joint.motion_limits.upper}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="tray_bottom",
            elem_b="housing_shell",
            min_overlap=0.015,
            name="extended drawer retains hidden insertion",
        )
        extended_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer slides out from the front",
        rest_pos is not None and extended_pos is not None and extended_pos[0] < rest_pos[0] - 0.045,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    before = ctx.part_element_world_aabb(lever, elem="lever_paddle")
    with ctx.pose({lever_joint: 0.9}):
        after = ctx.part_element_world_aabb(lever, elem="lever_paddle")
    ctx.check(
        "locking lever sweeps under the base",
        before is not None
        and after is not None
        and abs((after[0][1] + after[1][1]) - (before[0][1] + before[1][1])) > 0.020,
        details=f"before={before}, after={after}",
    )

    return ctx.report()


object_model = build_object_model()
