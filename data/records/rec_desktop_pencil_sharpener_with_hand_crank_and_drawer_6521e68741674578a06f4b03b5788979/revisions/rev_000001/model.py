from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_crank_sharpener")

    enamel = model.material("warm_enamel", rgba=(0.78, 0.73, 0.64, 1.0))
    dark = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    graphite = model.material("graphite_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed = model.material("brushed_metal", rgba=(0.72, 0.70, 0.66, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.90, 0.88, 0.82, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.02, 0.02, 0.02, 1.0))

    body = model.part("body")
    # Rectangular desktop housing: a connected casing assembled from top, bottom,
    # side, rear, and front frame members so the drawer bay is actually open.
    body.visual(Box((0.150, 0.110, 0.015)), origin=Origin(xyz=(0.005, 0.0, 0.0075)), material=enamel, name="bottom_shell")
    body.visual(Box((0.150, 0.110, 0.014)), origin=Origin(xyz=(0.005, 0.0, 0.098)), material=enamel, name="top_shell")
    body.visual(Box((0.150, 0.011, 0.087)), origin=Origin(xyz=(0.005, -0.0495, 0.0555)), material=enamel, name="side_wall_0")
    body.visual(Box((0.150, 0.011, 0.087)), origin=Origin(xyz=(0.005, 0.0495, 0.0555)), material=enamel, name="side_wall_1")
    body.visual(Box((0.012, 0.110, 0.087)), origin=Origin(xyz=(0.074, 0.0, 0.0555)), material=enamel, name="rear_wall")
    body.visual(Box((0.012, 0.110, 0.050)), origin=Origin(xyz=(-0.064, 0.0, 0.067)), material=enamel, name="front_panel")
    body.visual(Box((0.012, 0.010, 0.032)), origin=Origin(xyz=(-0.064, -0.050, 0.030)), material=enamel, name="drawer_jamb_0")
    body.visual(Box((0.012, 0.010, 0.032)), origin=Origin(xyz=(-0.064, 0.050, 0.030)), material=enamel, name="drawer_jamb_1")
    body.visual(Box((0.124, 0.014, 0.006)), origin=Origin(xyz=(-0.007, -0.043, 0.031)), material=graphite, name="drawer_rail_0")
    body.visual(Box((0.124, 0.014, 0.006)), origin=Origin(xyz=(-0.007, 0.043, 0.031)), material=graphite, name="drawer_rail_1")
    # Premium trim rails and rubber feet are mounted into the shell, not floating.
    body.visual(Box((0.105, 0.004, 0.003)), origin=Origin(xyz=(0.003, -0.038, 0.106)), material=brushed, name="top_trim_0")
    body.visual(Box((0.105, 0.004, 0.003)), origin=Origin(xyz=(0.003, 0.038, 0.106)), material=brushed, name="top_trim_1")
    for i, (x, y) in enumerate(((-0.048, -0.036), (-0.048, 0.036), (0.056, -0.036), (0.056, 0.036))):
        body.visual(Box((0.022, 0.018, 0.006)), origin=Origin(xyz=(x, y, -0.003)), material=rubber, name=f"foot_{i}")

    # Fixed side hub/collar that supports the rotating crank.
    body.visual(
        Cylinder(radius=0.025, length=0.013),
        origin=Origin(xyz=(0.012, 0.0605, 0.060), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="side_hub",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.0025),
        origin=Origin(xyz=(-0.07125, 0.0, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="pencil_entry",
    )

    drawer = model.part("drawer")
    drawer.visual(Box((0.008, 0.082, 0.038)), origin=Origin(xyz=(-0.004, 0.0, 0.019)), material=graphite, name="drawer_face")
    drawer.visual(Box((0.078, 0.064, 0.004)), origin=Origin(xyz=(0.039, 0.0, 0.003)), material=graphite, name="tray_floor")
    drawer.visual(Box((0.078, 0.004, 0.025)), origin=Origin(xyz=(0.039, -0.034, 0.0145)), material=graphite, name="tray_wall_0")
    drawer.visual(Box((0.078, 0.004, 0.025)), origin=Origin(xyz=(0.039, 0.034, 0.0145)), material=graphite, name="tray_wall_1")
    drawer.visual(Box((0.004, 0.064, 0.025)), origin=Origin(xyz=(0.076, 0.0, 0.0145)), material=graphite, name="tray_back")
    drawer.visual(Box((0.002, 0.046, 0.010)), origin=Origin(xyz=(-0.009, 0.0, 0.022)), material=dark, name="finger_pull")

    dial = model.part("dial")
    dial_ring = BezelGeometry(
        (0.022, 0.022),
        (0.046, 0.046),
        0.008,
        opening_shape="circle",
        outer_shape="circle",
        center=True,
    )
    dial.visual(
        mesh_from_geometry(dial_ring, "point_adjustment_dial"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="dial_ring",
    )
    # Raised tick marks on the moving dial face.
    for i in range(12):
        theta = i * math.tau / 12.0
        radius = 0.0180
        tick_len = 0.006 if i % 3 == 0 else 0.004
        dial.visual(
            Box((0.002, 0.0014, tick_len)),
            origin=Origin(
                xyz=(-0.0047, radius * math.sin(theta), radius * math.cos(theta)),
                rpy=(theta, 0.0, 0.0),
            ),
            material=dark,
            name=f"dial_tick_{i}",
        )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="crank_boss",
    )
    crank.visual(Box((0.011, 0.006, 0.052)), origin=Origin(xyz=(0.0, 0.010, -0.026)), material=chrome, name="crank_arm")
    crank.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.0, 0.022, -0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="handle_grip",
    )
    crank.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.0, 0.006, -0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_pin",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(-0.070, 0.0, 0.017)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.22, lower=0.0, upper=0.045),
    )
    model.articulation(
        "dial_axis",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(-0.074, 0.0, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0),
    )
    model.articulation(
        "crank_axis",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(0.012, 0.067, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=10.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    dial = object_model.get_part("dial")
    crank = object_model.get_part("crank")
    drawer_slide = object_model.get_articulation("drawer_slide")
    dial_axis = object_model.get_articulation("dial_axis")
    crank_axis = object_model.get_articulation("crank_axis")

    ctx.check("drawer is prismatic", drawer_slide.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("dial rotates continuously", dial_axis.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("crank rotates continuously", crank_axis.articulation_type == ArticulationType.CONTINUOUS)

    ctx.expect_gap(
        body,
        drawer,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem="front_panel",
        negative_elem="drawer_face",
        name="drawer face seats against front",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem="front_panel",
        negative_elem="dial_ring",
        name="dial seats on front face",
    )
    ctx.expect_contact(
        body,
        crank,
        elem_a="side_hub",
        elem_b="crank_boss",
        contact_tol=0.001,
        name="crank boss seats on side hub",
    )
    ctx.expect_within(drawer, body, axes="y", inner_elem="tray_floor", outer_elem="bottom_shell", margin=0.002, name="drawer tray fits body width")
    ctx.expect_overlap(drawer, body, axes="x", elem_a="tray_floor", elem_b="bottom_shell", min_overlap=0.070, name="closed drawer remains inserted")

    rest_drawer_pos = ctx.part_world_position(drawer)
    rest_handle_aabb = ctx.part_element_world_aabb(crank, elem="handle_grip")
    with ctx.pose({drawer_slide: 0.045, crank_axis: math.pi / 2.0, dial_axis: math.pi / 3.0}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        turned_handle_aabb = ctx.part_element_world_aabb(crank, elem="handle_grip")
        ctx.expect_within(drawer, body, axes="y", inner_elem="tray_floor", outer_elem="bottom_shell", margin=0.002, name="extended drawer stays guided")
        ctx.expect_overlap(drawer, body, axes="x", elem_a="tray_floor", elem_b="bottom_shell", min_overlap=0.030, name="extended drawer retains insertion")

    ctx.check(
        "drawer slides out the front",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] < rest_drawer_pos[0] - 0.040,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )
    if rest_handle_aabb is not None and turned_handle_aabb is not None:
        rest_center_x = (rest_handle_aabb[0][0] + rest_handle_aabb[1][0]) * 0.5
        turned_center_x = (turned_handle_aabb[0][0] + turned_handle_aabb[1][0]) * 0.5
        ctx.check(
            "crank handle orbits hub",
            abs(turned_center_x - rest_center_x) > 0.030,
            details=f"rest_x={rest_center_x}, turned_x={turned_center_x}",
        )
    else:
        ctx.fail("crank handle aabb available", "handle_grip element AABB could not be computed")

    return ctx.report()


object_model = build_object_model()
