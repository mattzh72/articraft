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


BASE_DEPTH = 0.078
BASE_WIDTH = 0.118
BASE_HEIGHT = 0.024
HANDLE_PIVOT_X = -0.027
HANDLE_PIVOT_Z = 0.0315
PUNCH_X = 0.024
PUNCH_SPACING = 0.080
TRAY_TRAVEL = 0.024


def _base_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_DEPTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
    )

    tray_cavity = (
        cq.Workplane("XY")
        .box(0.064, 0.094, 0.013, centered=(True, True, False))
        .translate((-0.009, 0.0, 0.0))
    )
    body = body.cut(tray_cavity)

    anvil_holes = (
        cq.Workplane("XY")
        .pushPoints([(PUNCH_X, -PUNCH_SPACING / 2.0), (PUNCH_X, PUNCH_SPACING / 2.0)])
        .circle(0.0037)
        .extrude(BASE_HEIGHT + 0.004)
    )
    body = body.cut(anvil_holes)

    nose_relief = (
        cq.Workplane("XY")
        .transformed(offset=(0.036, 0.0, 0.017), rotate=(0.0, 18.0, 0.0))
        .box(0.028, BASE_WIDTH + 0.030, 0.024, centered=(True, True, False))
    )
    body = body.cut(nose_relief)

    return body


def _handle_shape() -> cq.Workplane:
    lever_plate = (
        cq.Workplane("XY")
        .box(0.070, 0.112, 0.010, centered=(False, True, False))
        .translate((0.004, 0.0, -0.003))
    )
    finger_lip = (
        cq.Workplane("XY")
        .box(0.014, 0.112, 0.007, centered=(False, True, False))
        .translate((0.070, 0.0, -0.006))
    )
    hinge_tube = cq.Workplane("XZ").circle(0.0045).extrude(0.102 / 2.0, both=True)

    shell = lever_plate.union(finger_lip).union(hinge_tube)

    underside_relief = (
        cq.Workplane("XY")
        .box(0.050, 0.090, 0.0045, centered=(False, True, False))
        .translate((0.016, 0.0, -0.003))
    )
    shell = shell.cut(underside_relief)

    front_relief = (
        cq.Workplane("XY")
        .transformed(offset=(0.079, 0.0, 0.003), rotate=(0.0, -22.0, 0.0))
        .box(0.022, 0.130, 0.020, centered=(True, True, False))
    )
    shell = shell.cut(front_relief)

    punch_pins = (
        cq.Workplane("XY")
        .workplane(offset=-0.010)
        .pushPoints([(PUNCH_X - HANDLE_PIVOT_X, -PUNCH_SPACING / 2.0), (PUNCH_X - HANDLE_PIVOT_X, PUNCH_SPACING / 2.0)])
        .circle(0.00315)
        .extrude(0.014)
    )
    shell = shell.union(punch_pins)

    return shell


def _tray_shape() -> cq.Workplane:
    tray = cq.Workplane("XY").box(0.060, 0.090, 0.009, centered=(False, True, False))

    pocket = (
        cq.Workplane("XY")
        .box(0.056, 0.086, 0.010, centered=(False, True, False))
        .translate((0.002, 0.0, 0.0015))
    )
    tray = tray.cut(pocket)

    pull_flange = (
        cq.Workplane("XY")
        .box(0.006, 0.056, 0.006, centered=(False, True, False))
        .translate((-0.004, 0.0, 0.0015))
    )
    grip_rib = (
        cq.Workplane("XY")
        .box(0.003, 0.030, 0.003, centered=(False, True, False))
        .translate((-0.006, 0.0, 0.005))
    )

    return tray.union(pull_flange).union(grip_rib)


def _latch_shape() -> cq.Workplane:
    thickness = 0.008
    hub = cq.Workplane("XZ").circle(0.0055).extrude(thickness / 2.0, both=True)
    arm = (
        cq.Workplane("XZ")
        .center(0.009, 0.0035)
        .rect(0.018, 0.006)
        .extrude(thickness / 2.0, both=True)
    )
    paddle = (
        cq.Workplane("XZ")
        .center(0.016, 0.009)
        .rect(0.010, 0.010)
        .extrude(thickness / 2.0, both=True)
    )
    return hub.union(arm).union(paddle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_hole_punch")

    body_steel = model.material("body_steel", rgba=(0.18, 0.20, 0.23, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.08, 0.08, 0.09, 1.0))
    tray_polymer = model.material("tray_polymer", rgba=(0.13, 0.13, 0.14, 1.0))
    punch_metal = model.material("punch_metal", rgba=(0.74, 0.77, 0.80, 1.0))
    latch_accent = model.material("latch_accent", rgba=(0.77, 0.25, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "hole_punch_base"),
        material=body_steel,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(
            xyz=(-0.023, BASE_WIDTH / 2.0, 0.024),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=body_steel,
        name="latch_boss",
    )
    for x_pos, y_pos in (
        (-0.022, -0.053),
        (-0.022, 0.053),
        (0.026, -0.040),
        (0.026, 0.040),
    ):
        base.visual(
            Cylinder(radius=0.005, length=0.0025),
            origin=Origin(xyz=(x_pos, y_pos, 0.00125)),
            material=rubber,
            name=f"foot_{'rear' if x_pos < 0.0 else 'front'}_{0 if y_pos < 0.0 else 1}",
        )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shape(), "hole_punch_handle"),
        material=dark_polymer,
        name="handle_shell",
    )
    handle.visual(
        Box((0.012, 0.006, 0.004)),
        origin=Origin(xyz=(0.011, 0.059, -0.003)),
        material=dark_polymer,
        name="catch_tab",
    )

    chip_tray = model.part("chip_tray")
    chip_tray.visual(
        mesh_from_cadquery(_tray_shape(), "hole_punch_tray"),
        material=tray_polymer,
        name="tray_body",
    )

    side_latch = model.part("side_latch")
    side_latch.visual(
        mesh_from_cadquery(_latch_shape(), "hole_punch_side_latch"),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=latch_accent,
        name="latch_body",
    )
    side_latch.visual(
        Box((0.006, 0.008, 0.007)),
        origin=Origin(xyz=(0.0105, 0.004, 0.0105)),
        material=latch_accent,
        name="hook_tip",
    )

    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, 0.0, HANDLE_PIVOT_Z)),
        # The closed handle extends forward along local +X from the rear pivot.
        # -Y makes positive motion lift the front edge upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=chip_tray,
        origin=Origin(xyz=(-0.038, 0.0, 0.0015)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=side_latch,
        origin=Origin(xyz=(-0.023, BASE_WIDTH / 2.0 + 0.001, 0.024)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    chip_tray = object_model.get_part("chip_tray")
    side_latch = object_model.get_part("side_latch")

    handle_hinge = object_model.get_articulation("handle_hinge")
    tray_slide = object_model.get_articulation("tray_slide")
    latch_pivot = object_model.get_articulation("latch_pivot")

    tray_upper = tray_slide.motion_limits.upper if tray_slide.motion_limits else 0.0
    latch_lower = latch_pivot.motion_limits.lower if latch_pivot.motion_limits else 0.0
    handle_upper = handle_hinge.motion_limits.upper if handle_hinge.motion_limits else 0.0

    ctx.expect_overlap(
        handle,
        base,
        axes="xy",
        min_overlap=0.060,
        name="closed handle stays centered over the punch body",
    )
    ctx.expect_overlap(
        chip_tray,
        base,
        axes="yz",
        min_overlap=0.008,
        name="chip tray sits within the base guide envelope",
    )
    ctx.expect_contact(
        side_latch,
        handle,
        elem_a="hook_tip",
        elem_b="catch_tab",
        contact_tol=0.0015,
        name="side latch reaches the handle catch in the closed pose",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_shell")
    with ctx.pose({handle_hinge: handle_upper}):
        open_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_shell")

    ctx.check(
        "handle opens upward from the rear hinge",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.025,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    rest_tray_pos = ctx.part_world_position(chip_tray)
    with ctx.pose({tray_slide: tray_upper}):
        extended_tray_pos = ctx.part_world_position(chip_tray)
        ctx.expect_overlap(
            chip_tray,
            base,
            axes="x",
            min_overlap=0.020,
            name="extended chip tray keeps retained insertion in the guide",
        )

    ctx.check(
        "chip tray pulls rearward",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] < rest_tray_pos[0] - 0.018,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    locked_hook_aabb = ctx.part_element_world_aabb(side_latch, elem="hook_tip")
    with ctx.pose({latch_pivot: latch_lower}):
        unlocked_hook_aabb = ctx.part_element_world_aabb(side_latch, elem="hook_tip")

    ctx.check(
        "side latch rotates downward to unlock",
        locked_hook_aabb is not None
        and unlocked_hook_aabb is not None
        and unlocked_hook_aabb[1][2] < locked_hook_aabb[1][2] - 0.004,
        details=f"locked={locked_hook_aabb}, unlocked={unlocked_hook_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
