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


OPENING_WIDTH = 2.40
OPENING_HEIGHT = 3.20
FRAME_POST = 0.38
FRAME_DEPTH = 0.50
TOP_BEAM = 0.42
SILL_BEAM = 0.38
PANEL_WIDTH = 2.16
PANEL_HEIGHT = 3.20
PANEL_SKIN_DEPTH = 0.12
PANEL_Y = -0.09
GUIDE_WIDTH = 0.10
GUIDE_DEPTH = 0.08
PANEL_TRAVEL = 1.15
HOUSING_WIDTH = 0.96
HOUSING_DEPTH = 0.60
HOUSING_HEIGHT = 0.72
PEDESTAL_WIDTH = 0.44
PEDESTAL_DEPTH = 0.32
PEDESTAL_HEIGHT = 0.10
NECK_WIDTH = 0.34
NECK_DEPTH = 0.24
NECK_HEIGHT = 0.12


def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def build_handwheel():
    rim = cq.Workplane("XY").circle(0.34).circle(0.27).extrude(0.05).translate((0.0, 0.0, -0.025))
    hub = cq.Workplane("XY").circle(0.095).extrude(0.05).translate((0.0, 0.0, -0.025))
    spoke_x = cq.Workplane("XY").box(0.54, 0.055, 0.05)
    spoke_y = cq.Workplane("XY").box(0.055, 0.54, 0.05)
    return rim.union(hub).union(spoke_x).union(spoke_y)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sluice_gate")

    frame_mat = model.material("frame_steel", rgba=(0.34, 0.38, 0.40, 1.0))
    panel_mat = model.material("panel_steel", rgba=(0.23, 0.30, 0.35, 1.0))
    housing_mat = model.material("housing_gray", rgba=(0.52, 0.55, 0.58, 1.0))
    trim_mat = model.material("trim_dark", rgba=(0.13, 0.14, 0.15, 1.0))

    total_width = OPENING_WIDTH + 2.0 * FRAME_POST

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_POST, FRAME_DEPTH, OPENING_HEIGHT)),
        origin=Origin(xyz=(-(OPENING_WIDTH + FRAME_POST) / 2.0, 0.0, OPENING_HEIGHT / 2.0)),
        material=frame_mat,
        name="left_post",
    )
    frame.visual(
        Box((FRAME_POST, FRAME_DEPTH, OPENING_HEIGHT)),
        origin=Origin(xyz=((OPENING_WIDTH + FRAME_POST) / 2.0, 0.0, OPENING_HEIGHT / 2.0)),
        material=frame_mat,
        name="right_post",
    )
    frame.visual(
        Box((total_width, FRAME_DEPTH, TOP_BEAM)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_HEIGHT + TOP_BEAM / 2.0)),
        material=frame_mat,
        name="top_beam",
    )
    frame.visual(
        Box((total_width, FRAME_DEPTH, SILL_BEAM)),
        origin=Origin(xyz=(0.0, 0.0, -SILL_BEAM / 2.0)),
        material=frame_mat,
        name="sill_beam",
    )

    left_guide_x = -(OPENING_WIDTH / 2.0 - GUIDE_WIDTH / 2.0)
    right_guide_x = OPENING_WIDTH / 2.0 - GUIDE_WIDTH / 2.0
    guide_z = OPENING_HEIGHT / 2.0
    front_guide_y = 0.06
    rear_guide_y = -0.24

    frame.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, OPENING_HEIGHT)),
        origin=Origin(xyz=(left_guide_x, front_guide_y, guide_z)),
        material=frame_mat,
        name="left_front_guide",
    )
    frame.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, OPENING_HEIGHT)),
        origin=Origin(xyz=(right_guide_x, front_guide_y, guide_z)),
        material=frame_mat,
        name="right_front_guide",
    )
    frame.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, OPENING_HEIGHT)),
        origin=Origin(xyz=(left_guide_x, rear_guide_y, guide_z)),
        material=frame_mat,
        name="left_rear_guide",
    )
    frame.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, OPENING_HEIGHT)),
        origin=Origin(xyz=(right_guide_x, rear_guide_y, guide_z)),
        material=frame_mat,
        name="right_rear_guide",
    )

    panel = model.part("panel")
    panel.visual(
        Box((PANEL_WIDTH, PANEL_SKIN_DEPTH, PANEL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PANEL_HEIGHT / 2.0)),
        material=panel_mat,
        name="panel_skin",
    )
    panel.visual(
        Box((0.14, 0.04, 2.64)),
        origin=Origin(xyz=(-0.58, 0.07, 1.60)),
        material=frame_mat,
        name="stiffener_0",
    )
    panel.visual(
        Box((0.14, 0.04, 2.64)),
        origin=Origin(xyz=(0.58, 0.07, 1.60)),
        material=frame_mat,
        name="stiffener_1",
    )
    panel.visual(
        Box((1.70, 0.04, 0.14)),
        origin=Origin(xyz=(0.0, 0.07, 0.72)),
        material=frame_mat,
        name="rib_0",
    )
    panel.visual(
        Box((1.88, 0.04, 0.14)),
        origin=Origin(xyz=(0.0, 0.07, 1.62)),
        material=frame_mat,
        name="rib_1",
    )
    panel.visual(
        Box((1.58, 0.04, 0.14)),
        origin=Origin(xyz=(0.0, 0.07, 2.52)),
        material=frame_mat,
        name="rib_2",
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, PANEL_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=PANEL_TRAVEL, effort=20000.0, velocity=0.20),
    )

    housing = model.part("housing")
    housing.visual(
        Box((PEDESTAL_WIDTH, PEDESTAL_DEPTH, PEDESTAL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT / 2.0)),
        material=housing_mat,
        name="pedestal",
    )
    housing.visual(
        Box((NECK_WIDTH, NECK_DEPTH, NECK_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.02, PEDESTAL_HEIGHT + NECK_HEIGHT / 2.0)),
        material=housing_mat,
        name="neck",
    )
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.05,
                PEDESTAL_HEIGHT + NECK_HEIGHT + HOUSING_HEIGHT / 2.0,
            )
        ),
        material=housing_mat,
        name="housing_shell",
    )

    model.articulation(
        "frame_to_housing",
        ArticulationType.FIXED,
        parent=frame,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, OPENING_HEIGHT + TOP_BEAM)),
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(build_handwheel(), "handwheel_ring"),
        origin=Origin(xyz=(0.0, 0.19, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="wheel_rim",
    )
    handwheel.visual(
        Cylinder(radius=0.032, length=0.18),
        origin=Origin(xyz=(0.0, 0.09, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="wheel_shaft",
    )
    handwheel.visual(
        Cylinder(radius=0.022, length=0.10),
        origin=Origin(xyz=(0.0, 0.23, 0.27), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="spinner_knob",
    )

    model.articulation(
        "housing_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=handwheel,
        origin=Origin(
            xyz=(
                0.0,
                0.05 + HOUSING_DEPTH / 2.0,
                PEDESTAL_HEIGHT + NECK_HEIGHT + HOUSING_HEIGHT * 0.56,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=5.0),
    )

    door = model.part("door")
    door.visual(
        Box((0.28, 0.04, 0.40)),
        origin=Origin(xyz=(0.14, 0.02, 0.0)),
        material=housing_mat,
        name="door_panel",
    )
    door.visual(
        Box((0.05, 0.03, 0.10)),
        origin=Origin(xyz=(0.23, 0.045, 0.0)),
        material=trim_mat,
        name="door_pull",
    )
    door.visual(
        Cylinder(radius=0.015, length=0.10),
        origin=Origin(xyz=(0.0, 0.02, 0.13)),
        material=trim_mat,
        name="hinge_barrel_0",
    )
    door.visual(
        Cylinder(radius=0.015, length=0.10),
        origin=Origin(xyz=(0.0, 0.02, -0.13)),
        material=trim_mat,
        name="hinge_barrel_1",
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(
            xyz=(
                0.12,
                0.05 + HOUSING_DEPTH / 2.0,
                PEDESTAL_HEIGHT + NECK_HEIGHT + 0.38,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=30.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    housing = object_model.get_part("housing")
    handwheel = object_model.get_part("handwheel")
    door = object_model.get_part("door")
    panel_slide = object_model.get_articulation("frame_to_panel")
    wheel_joint = object_model.get_articulation("housing_to_handwheel")
    door_joint = object_model.get_articulation("housing_to_door")
    slide_limits = panel_slide.motion_limits
    door_limits = door_joint.motion_limits

    ctx.expect_gap(
        panel,
        frame,
        axis="x",
        positive_elem="panel_skin",
        negative_elem="left_front_guide",
        max_gap=0.04,
        max_penetration=0.0,
        name="panel clears left guide",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="x",
        positive_elem="right_front_guide",
        negative_elem="panel_skin",
        max_gap=0.04,
        max_penetration=0.0,
        name="panel clears right guide",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="y",
        positive_elem="left_front_guide",
        negative_elem="panel_skin",
        max_gap=0.05,
        max_penetration=0.0,
        name="panel clears front retainers",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="y",
        positive_elem="panel_skin",
        negative_elem="left_rear_guide",
        max_gap=0.05,
        max_penetration=0.0,
        name="panel clears rear guide liners",
    )

    if slide_limits is not None and slide_limits.upper is not None:
        rest_pos = ctx.part_world_position(panel)
        with ctx.pose({panel_slide: slide_limits.upper}):
            ctx.expect_overlap(
                panel,
                frame,
                axes="z",
                elem_a="panel_skin",
                elem_b="left_front_guide",
                min_overlap=2.0,
                name="panel remains retained by side guides at full lift",
            )
            lifted_pos = ctx.part_world_position(panel)
        ctx.check(
            "panel lifts upward",
            rest_pos is not None and lifted_pos is not None and lifted_pos[2] > rest_pos[2] + 1.0,
            details=f"rest={rest_pos}, lifted={lifted_pos}",
        )

    ctx.expect_contact(
        housing,
        frame,
        elem_a="pedestal",
        elem_b="top_beam",
        name="operator housing is seated on top beam",
    )
    ctx.expect_gap(
        handwheel,
        housing,
        axis="y",
        positive_elem="wheel_rim",
        negative_elem="housing_shell",
        min_gap=0.10,
        name="handwheel projects in front of gearbox housing",
    )
    ctx.expect_gap(
        door,
        housing,
        axis="y",
        positive_elem="door_panel",
        negative_elem="housing_shell",
        max_gap=0.002,
        max_penetration=0.0,
        name="inspection door closes flush on housing face",
    )

    spinner_rest = aabb_center(ctx.part_element_world_aabb(handwheel, elem="spinner_knob"))
    with ctx.pose({wheel_joint: pi / 2.0}):
        spinner_quarter = aabb_center(ctx.part_element_world_aabb(handwheel, elem="spinner_knob"))
    ctx.check(
        "handwheel rotation carries spinner knob around the rim",
        spinner_rest is not None
        and spinner_quarter is not None
        and abs(spinner_quarter[0] - spinner_rest[0]) > 0.20
        and abs(spinner_quarter[2] - spinner_rest[2]) > 0.20,
        details=f"rest={spinner_rest}, quarter_turn={spinner_quarter}",
    )

    if door_limits is not None and door_limits.upper is not None:
        door_rest = aabb_center(ctx.part_element_world_aabb(door, elem="door_panel"))
        with ctx.pose({door_joint: door_limits.upper}):
            door_open = aabb_center(ctx.part_element_world_aabb(door, elem="door_panel"))
        ctx.check(
            "inspection door swings outward from the housing face",
            door_rest is not None and door_open is not None and door_open[1] > door_rest[1] + 0.08,
            details=f"rest={door_rest}, open={door_open}",
        )

    return ctx.report()


object_model = build_object_model()
