from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="writing_desk")

    desk = model.part("desk_frame")
    
    # Desk top: 1.2m wide, 0.6m deep, 0.02m thick
    desk.visual(Box((1.2, 0.6, 0.02)), origin=Origin((0.0, 0.0, 0.74)), name="top")
    
    # Four straight legs: 0.04m x 0.04m, 0.73m tall
    desk.visual(Box((0.04, 0.04, 0.73)), origin=Origin((-0.56, -0.26, 0.365)), name="leg_fl")
    desk.visual(Box((0.04, 0.04, 0.73)), origin=Origin((0.56, -0.26, 0.365)), name="leg_fr")
    desk.visual(Box((0.04, 0.04, 0.73)), origin=Origin((-0.56, 0.26, 0.365)), name="leg_bl")
    desk.visual(Box((0.04, 0.04, 0.73)), origin=Origin((0.56, 0.26, 0.365)), name="leg_br")
    
    # Aprons connecting the legs for structural realism
    desk.visual(Box((0.02, 0.48, 0.08)), origin=Origin((-0.56, 0.0, 0.69)), name="apron_left")
    desk.visual(Box((0.02, 0.48, 0.08)), origin=Origin((0.56, 0.0, 0.69)), name="apron_right")
    desk.visual(Box((1.08, 0.02, 0.08)), origin=Origin((0.0, 0.26, 0.69)), name="apron_back")
    
    # Internal runners for the drawer
    # Main side guides
    desk.visual(Box((0.02, 0.53, 0.09)), origin=Origin((-0.31, -0.015, 0.685)), name="runner_left")
    desk.visual(Box((0.02, 0.53, 0.09)), origin=Origin((0.31, -0.015, 0.685)), name="runner_right")
    # Bottom support ledges
    desk.visual(Box((0.02, 0.53, 0.01)), origin=Origin((-0.29, -0.015, 0.645)), name="runner_bottom_left")
    desk.visual(Box((0.02, 0.53, 0.01)), origin=Origin((0.29, -0.015, 0.645)), name="runner_bottom_right")

    drawer = model.part("drawer")
    
    # Hollow drawer box using CadQuery
    drawer_width = 0.59
    drawer_depth = 0.4
    drawer_height = 0.068
    wall_thickness = 0.015

    drawer_box_cq = (
        cq.Workplane("XY")
        .box(drawer_width, drawer_depth, drawer_height)
        .faces("+Z")
        .shell(-wall_thickness)
    )

    drawer.visual(
        mesh_from_cadquery(drawer_box_cq, "drawer_box"),
        origin=Origin((0.0, -0.08, 0.686)),
        name="box"
    )
    
    # Drawer front panel
    drawer.visual(Box((0.64, 0.02, 0.085)), origin=Origin((0.0, -0.29, 0.6825)), name="front_panel")
    
    # Drawer handle
    drawer.visual(Box((0.1, 0.02, 0.02)), origin=Origin((0.0, -0.31, 0.69)), name="handle")

    # Articulation for the drawer
    model.articulation(
        "drawer_pull",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=drawer,
        origin=Origin((0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.35)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desk = object_model.get_part("desk_frame")
    drawer = object_model.get_part("drawer")
    pull = object_model.get_articulation("drawer_pull")
    
    # Check that the drawer box fits between the side guides
    ctx.expect_gap(drawer, desk, axis="x", positive_elem="box", negative_elem="runner_left", min_gap=0.004, max_gap=0.006)
    ctx.expect_gap(desk, drawer, axis="x", positive_elem="runner_right", negative_elem="box", min_gap=0.004, max_gap=0.006)
    
    # Check that the drawer box rests on the bottom ledges
    ctx.expect_gap(drawer, desk, axis="z", positive_elem="box", negative_elem="runner_bottom_left", min_gap=0.001, max_gap=0.003)
    ctx.expect_gap(drawer, desk, axis="z", positive_elem="box", negative_elem="runner_bottom_right", min_gap=0.001, max_gap=0.003)
    
    # Check that the drawer overlaps the bottom ledges in XY footprint (proving it is supported)
    ctx.expect_overlap(drawer, desk, axes="xy", elem_a="box", elem_b="runner_bottom_left", min_overlap=0.01)
    ctx.expect_overlap(drawer, desk, axes="xy", elem_a="box", elem_b="runner_bottom_right", min_overlap=0.01)
    
    # Check that the drawer pulls out correctly and remains supported
    with ctx.pose({pull: 0.25}):
        ctx.expect_overlap(drawer, desk, axes="y", elem_a="box", elem_b="runner_bottom_left", min_overlap=0.1)
        
    return ctx.report()


object_model = build_object_model()
