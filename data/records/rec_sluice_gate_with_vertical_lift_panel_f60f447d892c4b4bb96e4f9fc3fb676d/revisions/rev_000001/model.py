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


OPENING_WIDTH = 2.40
OPENING_HEIGHT = 2.70
PIER_WIDTH = 0.80
MASONRY_DEPTH = 0.72
SILL_HEIGHT = 0.55
LINTEL_HEIGHT = 0.75
FRAME_HEIGHT = SILL_HEIGHT + OPENING_HEIGHT + LINTEL_HEIGHT
GUIDE_HEIGHT = 3.55
GUIDE_DEPTH = 0.30
GUIDE_BACK_THICKNESS = 0.05
GUIDE_FLANGE_LENGTH = 0.18
GUIDE_FLANGE_THICKNESS = 0.05
PANEL_WIDTH = 2.32
PANEL_HEIGHT = 2.90
PANEL_THICKNESS = 0.08
PANEL_TRAVEL = 1.35
PANEL_CENTER_Y = 0.15
PANEL_CLOSED_CENTER_Z = SILL_HEIGHT + PANEL_HEIGHT / 2.0
GEARBOX_BASE_Z = FRAME_HEIGHT + 0.04
GEARBOX_CENTER_X = 0.68
GEARBOX_CENTER_Y = 0.07
GEARBOX_CENTER_Z = FRAME_HEIGHT + 0.23
HANDWHEEL_CENTER_X = 0.68
HANDWHEEL_CENTER_Y = 0.44
HANDWHEEL_CENTER_Z = FRAME_HEIGHT + 0.23
PAWL_PIVOT_X = 1.12
PAWL_PIVOT_Y = 0.50
PAWL_PIVOT_Z = FRAME_HEIGHT + 0.40


def add_guide(
    guide_part,
    *,
    sign: float,
    steel,
    back_plate_name: str,
    rear_flange_name: str,
    front_flange_name: str,
    shoe_name: str,
) -> None:
    back_plate_x = sign * (OPENING_WIDTH / 2.0 + GUIDE_BACK_THICKNESS / 2.0)
    flange_center_x = sign * (OPENING_WIDTH / 2.0 - GUIDE_FLANGE_LENGTH / 2.0)
    guide_center_z = SILL_HEIGHT + GUIDE_HEIGHT / 2.0 - 0.05

    guide_part.visual(
        Box((GUIDE_BACK_THICKNESS, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(back_plate_x, GUIDE_DEPTH / 2.0, guide_center_z)),
        material=steel,
        name=back_plate_name,
    )
    guide_part.visual(
        Box((GUIDE_FLANGE_LENGTH, GUIDE_FLANGE_THICKNESS, GUIDE_HEIGHT)),
        origin=Origin(xyz=(flange_center_x, GUIDE_FLANGE_THICKNESS / 2.0, guide_center_z)),
        material=steel,
        name=rear_flange_name,
    )
    guide_part.visual(
        Box((GUIDE_FLANGE_LENGTH, GUIDE_FLANGE_THICKNESS, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(flange_center_x, GUIDE_DEPTH - GUIDE_FLANGE_THICKNESS / 2.0, guide_center_z)
        ),
        material=steel,
        name=front_flange_name,
    )
    guide_part.visual(
        Box((0.28, GUIDE_DEPTH, 0.24)),
        origin=Origin(xyz=(sign * (OPENING_WIDTH / 2.0 + 0.12), GUIDE_DEPTH / 2.0, SILL_HEIGHT + 0.12)),
        material=steel,
        name=shoe_name,
    )


def build_handwheel():
    ring = cq.Workplane("XZ").circle(0.36).circle(0.31).extrude(0.03, both=True)
    hub = cq.Workplane("XZ").circle(0.075).extrude(0.06, both=True)
    spoke_h = cq.Workplane("XY").box(0.68, 0.024, 0.042)
    spoke_v = cq.Workplane("XY").box(0.042, 0.024, 0.68)
    return ring.union(hub).union(spoke_h).union(spoke_v)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sluice_gate")

    masonry = model.material("masonry", rgba=(0.58, 0.57, 0.53, 1.0))
    steel = model.material("steel", rgba=(0.33, 0.36, 0.40, 1.0))
    coated_steel = model.material("coated_steel", rgba=(0.18, 0.31, 0.42, 1.0))
    stem_finish = model.material("stem_finish", rgba=(0.56, 0.58, 0.61, 1.0))

    wall = model.part("masonry")
    wall.visual(
        Box((PIER_WIDTH, MASONRY_DEPTH, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(-(OPENING_WIDTH + PIER_WIDTH) / 2.0, -MASONRY_DEPTH / 2.0, FRAME_HEIGHT / 2.0)
        ),
        material=masonry,
        name="left_pier",
    )
    wall.visual(
        Box((PIER_WIDTH, MASONRY_DEPTH, FRAME_HEIGHT)),
        origin=Origin(
            xyz=((OPENING_WIDTH + PIER_WIDTH) / 2.0, -MASONRY_DEPTH / 2.0, FRAME_HEIGHT / 2.0)
        ),
        material=masonry,
        name="right_pier",
    )
    wall.visual(
        Box((OPENING_WIDTH + 2.0 * PIER_WIDTH, MASONRY_DEPTH, SILL_HEIGHT)),
        origin=Origin(xyz=(0.0, -MASONRY_DEPTH / 2.0, SILL_HEIGHT / 2.0)),
        material=masonry,
        name="sill",
    )
    wall.visual(
        Box((OPENING_WIDTH + 2.0 * PIER_WIDTH, 0.40, LINTEL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -0.20,
                SILL_HEIGHT + OPENING_HEIGHT + LINTEL_HEIGHT / 2.0,
            )
        ),
        material=masonry,
        name="lintel",
    )

    left_guide = model.part("left_guide")
    add_guide(
        left_guide,
        sign=-1.0,
        steel=steel,
        back_plate_name="left_back_plate",
        rear_flange_name="left_rear_flange",
        front_flange_name="left_front_flange",
        shoe_name="left_shoe",
    )
    model.articulation("masonry_to_left_guide", ArticulationType.FIXED, parent=wall, child=left_guide)

    right_guide = model.part("right_guide")
    add_guide(
        right_guide,
        sign=1.0,
        steel=steel,
        back_plate_name="right_back_plate",
        rear_flange_name="right_rear_flange",
        front_flange_name="right_front_flange",
        shoe_name="right_shoe",
    )
    model.articulation("masonry_to_right_guide", ArticulationType.FIXED, parent=wall, child=right_guide)

    panel = model.part("panel")
    panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)),
        material=coated_steel,
        name="leaf",
    )
    panel.visual(
        Box((0.12, 0.06, PANEL_HEIGHT - 0.18)),
        origin=Origin(xyz=(0.0, PANEL_THICKNESS / 2.0 + 0.03, 0.0)),
        material=steel,
        name="center_stiffener",
    )
    panel.visual(
        Box((PANEL_WIDTH - 0.24, 0.06, 0.16)),
        origin=Origin(xyz=(0.0, PANEL_THICKNESS / 2.0 + 0.03, PANEL_HEIGHT / 2.0 - 0.22)),
        material=steel,
        name="top_stiffener",
    )
    panel.visual(
        Cylinder(radius=0.045, length=1.70),
        origin=Origin(
            xyz=(0.0, 0.0, PANEL_HEIGHT / 2.0 + 0.85),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=stem_finish,
        name="lifting_stem",
    )
    panel.visual(
        Box((0.24, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, PANEL_HEIGHT / 2.0 + 0.08)),
        material=steel,
        name="stem_head",
    )

    model.articulation(
        "panel_slide",
        ArticulationType.PRISMATIC,
        parent=wall,
        child=panel,
        origin=Origin(xyz=(0.0, PANEL_CENTER_Y, PANEL_CLOSED_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=PANEL_TRAVEL, effort=18000.0, velocity=0.20),
    )

    gearbox = model.part("gearbox")
    gearbox.visual(
        Box((0.44, 0.38, 0.08)),
        origin=Origin(xyz=(-0.34, 0.05, GEARBOX_BASE_Z)),
        material=steel,
        name="left_base_plate",
    )
    gearbox.visual(
        Box((0.78, 0.38, 0.08)),
        origin=Origin(xyz=(0.60, 0.05, GEARBOX_BASE_Z)),
        material=steel,
        name="right_base_plate",
    )
    gearbox.visual(
        Box((0.08, 0.16, 0.42)),
        origin=Origin(xyz=(-0.10, 0.07, FRAME_HEIGHT + 0.21)),
        material=steel,
        name="left_yoke",
    )
    gearbox.visual(
        Box((0.08, 0.16, 0.42)),
        origin=Origin(xyz=(0.10, 0.07, FRAME_HEIGHT + 0.21)),
        material=steel,
        name="right_yoke",
    )
    gearbox.visual(
        Box((0.28, 0.05, 0.24)),
        origin=Origin(xyz=(0.0, -0.02, FRAME_HEIGHT + 0.12)),
        material=steel,
        name="rear_web",
    )
    gearbox.visual(
        Box((0.42, 0.14, 0.12)),
        origin=Origin(xyz=(0.34, 0.06, FRAME_HEIGHT + 0.22)),
        material=steel,
        name="torque_tube",
    )
    gearbox.visual(
        Box((0.52, 0.34, 0.40)),
        origin=Origin(xyz=(GEARBOX_CENTER_X, GEARBOX_CENTER_Y, GEARBOX_CENTER_Z)),
        material=coated_steel,
        name="housing",
    )
    gearbox.visual(
        Box((0.22, 0.12, 0.24)),
        origin=Origin(xyz=(0.48, 0.00, FRAME_HEIGHT + 0.12)),
        material=steel,
        name="housing_pedestal",
    )
    gearbox.visual(
        Cylinder(radius=0.11, length=0.14),
        origin=Origin(
            xyz=(HANDWHEEL_CENTER_X, HANDWHEEL_CENTER_Y - 0.14, HANDWHEEL_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="wheel_boss",
    )
    gearbox.visual(
        Cylinder(radius=0.035, length=0.06),
        origin=Origin(
            xyz=(PAWL_PIVOT_X, PAWL_PIVOT_Y - 0.03, PAWL_PIVOT_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="pawl_boss",
    )
    gearbox.visual(
        Box((0.10, 0.20, 0.14)),
        origin=Origin(xyz=(PAWL_PIVOT_X, 0.34, PAWL_PIVOT_Z)),
        material=steel,
        name="pawl_bracket",
    )
    gearbox.visual(
        Box((0.16, 0.08, 0.10)),
        origin=Origin(xyz=(1.01, 0.28, PAWL_PIVOT_Z)),
        material=steel,
        name="pawl_arm",
    )
    model.articulation("masonry_to_gearbox", ArticulationType.FIXED, parent=wall, child=gearbox)

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(build_handwheel(), "handwheel"),
        material=steel,
        name="wheel",
    )
    handwheel.visual(
        Cylinder(radius=0.08, length=0.02),
        origin=Origin(xyz=(0.0, -0.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_collar",
    )
    handwheel.visual(
        Cylinder(radius=0.022, length=0.12),
        origin=Origin(
            xyz=(0.24, 0.075, -0.19),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=stem_finish,
        name="grip",
    )
    model.articulation(
        "handwheel_spin",
        ArticulationType.CONTINUOUS,
        parent=gearbox,
        child=handwheel,
        origin=Origin(xyz=(HANDWHEEL_CENTER_X, HANDWHEEL_CENTER_Y, HANDWHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=4.0),
    )

    pawl = model.part("pawl")
    pawl.visual(
        Cylinder(radius=0.028, length=0.06),
        origin=Origin(xyz=(0.0, 0.03, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    pawl.visual(
        Box((0.14, 0.05, 0.03)),
        origin=Origin(xyz=(-0.05, 0.04, -0.05), rpy=(0.0, -0.45, 0.0)),
        material=steel,
        name="lever",
    )
    pawl.visual(
        Box((0.035, 0.05, 0.05)),
        origin=Origin(xyz=(-0.11, 0.04, -0.08), rpy=(0.0, -0.45, 0.0)),
        material=steel,
        name="tip",
    )
    model.articulation(
        "pawl_pivot",
        ArticulationType.REVOLUTE,
        parent=gearbox,
        child=pawl,
        origin=Origin(xyz=(PAWL_PIVOT_X, PAWL_PIVOT_Y, PAWL_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.65, upper=0.35, effort=40.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panel = object_model.get_part("panel")
    left_guide = object_model.get_part("left_guide")
    right_guide = object_model.get_part("right_guide")
    gearbox = object_model.get_part("gearbox")
    handwheel = object_model.get_part("handwheel")
    pawl = object_model.get_part("pawl")
    slide = object_model.get_articulation("panel_slide")
    handwheel_spin = object_model.get_articulation("handwheel_spin")
    pawl_pivot = object_model.get_articulation("pawl_pivot")
    limits = slide.motion_limits
    pawl_limits = pawl_pivot.motion_limits

    ctx.expect_gap(
        panel,
        left_guide,
        axis="y",
        positive_elem="leaf",
        negative_elem="left_rear_flange",
        min_gap=0.05,
        max_gap=0.08,
        name="panel clears left rear flange",
    )
    ctx.expect_gap(
        left_guide,
        panel,
        axis="y",
        positive_elem="left_front_flange",
        negative_elem="leaf",
        min_gap=0.05,
        max_gap=0.08,
        name="panel clears left front flange",
    )
    ctx.expect_gap(
        right_guide,
        panel,
        axis="x",
        positive_elem="right_back_plate",
        negative_elem="leaf",
        min_gap=0.01,
        max_gap=0.05,
        name="panel sits inside right guide pocket",
    )
    ctx.expect_gap(
        panel,
        left_guide,
        axis="x",
        positive_elem="leaf",
        negative_elem="left_back_plate",
        min_gap=0.01,
        max_gap=0.05,
        name="panel sits inside left guide pocket",
    )
    ctx.expect_overlap(panel, left_guide, axes="z", min_overlap=2.5, name="panel overlaps left guide at rest")
    ctx.expect_overlap(panel, right_guide, axes="z", min_overlap=2.5, name="panel overlaps right guide at rest")
    ctx.expect_contact(
        handwheel,
        gearbox,
        elem_a="wheel",
        elem_b="wheel_boss",
        contact_tol=0.02,
        name="handwheel mounts on gearbox boss",
    )
    ctx.expect_contact(
        pawl,
        gearbox,
        elem_a="hub",
        elem_b="pawl_boss",
        contact_tol=0.005,
        name="pawl mounts on gearbox bracket",
    )

    if limits is not None and limits.upper is not None:
        rest_pos = ctx.part_world_position(panel)
        with ctx.pose({slide: limits.upper}):
            ctx.expect_overlap(
                panel,
                left_guide,
                axes="z",
                min_overlap=1.4,
                name="panel remains retained in left guide when raised",
            )
            ctx.expect_overlap(
                panel,
                right_guide,
                axes="z",
                min_overlap=1.4,
                name="panel remains retained in right guide when raised",
            )
            raised_pos = ctx.part_world_position(panel)
        ctx.check(
            "panel raises upward",
            rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 1.2,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    grip_rest = ctx.part_element_world_aabb(handwheel, elem="grip")
    with ctx.pose({handwheel_spin: 1.2}):
        grip_turned = ctx.part_element_world_aabb(handwheel, elem="grip")
    ctx.check(
        "handwheel grip changes position when spun",
        grip_rest is not None
        and grip_turned is not None
        and abs(float(grip_turned[1][0]) - float(grip_rest[1][0])) > 0.08
        and abs(float(grip_turned[1][2]) - float(grip_rest[1][2])) > 0.08,
        details=f"rest={grip_rest}, turned={grip_turned}",
    )

    if pawl_limits is not None and pawl_limits.lower is not None and pawl_limits.upper is not None:
        with ctx.pose({pawl_pivot: pawl_limits.lower}):
            tip_low = ctx.part_element_world_aabb(pawl, elem="tip")
        with ctx.pose({pawl_pivot: pawl_limits.upper}):
            tip_high = ctx.part_element_world_aabb(pawl, elem="tip")
        ctx.check(
            "pawl tip lifts when released",
            tip_low is not None
            and tip_high is not None
            and float(tip_high[1][2]) > float(tip_low[1][2]) + 0.05,
            details=f"low={tip_low}, high={tip_high}",
        )

    return ctx.report()


object_model = build_object_model()
