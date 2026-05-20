from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("concrete", rgba=(0.67, 0.68, 0.70, 1.0))
    steel = model.material("bridge_steel", rgba=(0.27, 0.30, 0.34, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.41, 0.45, 0.24, 1.0))
    asphalt = model.material("asphalt", rgba=(0.16, 0.17, 0.18, 1.0))
    machinery = model.material("machinery_dark", rgba=(0.19, 0.20, 0.22, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((5.2, 7.0, 2.8)),
        origin=Origin(xyz=(-2.6, 0.0, -1.48)),
        material=concrete,
        name="abutment_mass",
    )
    shore_frame.visual(
        Box((4.4, 4.3, 0.28)),
        origin=Origin(xyz=(-2.2, 0.0, 0.04)),
        material=asphalt,
        name="approach_roadway",
    )
    shore_frame.visual(
        Box((0.55, 5.4, 0.52)),
        origin=Origin(xyz=(-0.275, 0.0, -0.22)),
        material=steel,
        name="hinge_crosshead",
    )
    shore_frame.visual(
        Box((4.2, 0.78, 1.06)),
        origin=Origin(xyz=(-2.1, 2.54, -0.35)),
        material=concrete,
        name="left_sidewall",
    )
    shore_frame.visual(
        Box((4.0, 0.38, 0.62)),
        origin=Origin(xyz=(-2.0, -2.34, -0.13)),
        material=concrete,
        name="right_sidewall",
    )
    shore_frame.visual(
        Box((0.95, 1.18, 1.52)),
        origin=Origin(xyz=(0.12, 2.75, -0.06)),
        material=steel,
        name="left_bearing_pier",
    )
    shore_frame.visual(
        Box((0.58, 0.92, 1.28)),
        origin=Origin(xyz=(0.02, -2.62, -0.18)),
        material=steel,
        name="right_bearing_pier",
    )
    shore_frame.visual(
        Box((0.42, 0.64, 0.54)),
        origin=Origin(xyz=(0.30, 2.54, 0.05)),
        material=machinery,
        name="left_bearing_housing",
    )
    shore_frame.visual(
        Box((0.34, 0.52, 0.46)),
        origin=Origin(xyz=(0.22, -2.54, 0.03)),
        material=machinery,
        name="right_bearing_housing",
    )
    shore_frame.visual(
        Cylinder(radius=0.26, length=0.22),
        origin=Origin(xyz=(0.30, 2.23, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery,
        name="left_side_bearing",
    )
    shore_frame.visual(
        Cylinder(radius=0.22, length=0.18),
        origin=Origin(xyz=(0.22, -2.21, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery,
        name="right_side_bearing",
    )
    shore_frame.visual(
        Box((2.5, 1.55, 1.72)),
        origin=Origin(xyz=(-1.85, 2.48, 1.04)),
        material=machinery,
        name="left_machinery_house",
    )
    shore_frame.visual(
        Box((0.52, 1.20, 0.44)),
        origin=Origin(xyz=(-0.56, 2.56, 0.40)),
        material=machinery,
        name="left_machinery_plinth",
    )
    shore_frame.visual(
        Box((0.72, 0.56, 0.84)),
        origin=Origin(xyz=(-1.25, -2.42, 0.60)),
        material=machinery,
        name="right_control_cabinet",
    )
    shore_frame.inertial = Inertial.from_geometry(
        Box((5.2, 7.0, 2.8)),
        mass=18500.0,
        origin=Origin(xyz=(-2.6, 0.0, -1.48)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((9.6, 4.0, 0.12)),
        origin=Origin(xyz=(4.8, 0.0, 0.12)),
        material=asphalt,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((9.35, 0.26, 0.72)),
        origin=Origin(xyz=(4.675, 1.85, -0.18)),
        material=painted_steel,
        name="left_main_girder",
    )
    bridge_leaf.visual(
        Box((9.35, 0.26, 0.72)),
        origin=Origin(xyz=(4.675, -1.85, -0.18)),
        material=painted_steel,
        name="right_main_girder",
    )
    bridge_leaf.visual(
        Box((0.52, 4.0, 0.52)),
        origin=Origin(xyz=(0.26, 0.0, -0.14)),
        material=painted_steel,
        name="hinge_floorbeam",
    )
    bridge_leaf.visual(
        Box((0.28, 4.0, 0.46)),
        origin=Origin(xyz=(9.46, 0.0, -0.11)),
        material=painted_steel,
        name="tip_floorbeam",
    )
    for index, rib_x in enumerate((1.4, 3.0, 4.6, 6.2, 7.8), start=1):
        bridge_leaf.visual(
            Box((0.18, 3.46, 0.40)),
            origin=Origin(xyz=(rib_x, 0.0, -0.14)),
            material=painted_steel,
            name=f"deck_rib_{index}",
        )
    bridge_leaf.visual(
        Box((9.0, 0.12, 0.20)),
        origin=Origin(xyz=(4.75, 1.68, 0.28)),
        material=steel,
        name="left_curb_beam",
    )
    bridge_leaf.visual(
        Box((9.0, 0.12, 0.20)),
        origin=Origin(xyz=(4.75, -1.68, 0.28)),
        material=steel,
        name="right_curb_beam",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.18, length=0.14),
        origin=Origin(xyz=(0.18, 2.05, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery,
        name="left_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.18, length=0.14),
        origin=Origin(xyz=(0.18, -2.05, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery,
        name="right_trunnion",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((9.6, 4.0, 0.90)),
        mass=6200.0,
        origin=Origin(xyz=(4.8, 0.0, -0.16)),
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=250000.0,
            velocity=0.45,
            lower=0.0,
            upper=1.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    leaf_hinge = object_model.get_articulation("shore_to_leaf")

    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="x",
        positive_elem="deck_plate",
        negative_elem="approach_roadway",
        min_gap=0.0,
        max_gap=0.02,
        name="leaf seats at the shore hinge line",
    )
    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="y",
        elem_a="deck_plate",
        elem_b="approach_roadway",
        min_overlap=3.8,
        name="leaf deck matches the roadway width",
    )
    ctx.expect_contact(
        bridge_leaf,
        shore_frame,
        elem_a="left_trunnion",
        elem_b="left_side_bearing",
        name="left trunnion seats against the heavy side bearing",
    )
    ctx.expect_contact(
        bridge_leaf,
        shore_frame,
        elem_a="right_trunnion",
        elem_b="right_side_bearing",
        name="right trunnion seats against the outboard bearing",
    )

    closed_tip = ctx.part_element_world_aabb(bridge_leaf, elem="tip_floorbeam")
    with ctx.pose({leaf_hinge: 1.18}):
        opened_tip = ctx.part_element_world_aabb(bridge_leaf, elem="tip_floorbeam")
    tip_rises = (
        closed_tip is not None
        and opened_tip is not None
        and opened_tip[0][2] > closed_tip[1][2] + 7.0
    )
    ctx.check(
        "leaf opens upward under positive rotation",
        tip_rises,
        details=f"closed_tip={closed_tip}, opened_tip={opened_tip}",
    )

    left_house = ctx.part_element_world_aabb(shore_frame, elem="left_machinery_house")
    right_cabinet = ctx.part_element_world_aabb(shore_frame, elem="right_control_cabinet")
    left_heavier = (
        left_house is not None
        and right_cabinet is not None
        and left_house[1][1] > abs(right_cabinet[0][1]) + 0.5
    )
    ctx.check(
        "fixed shore frame is heavier on the left side",
        left_heavier,
        details=f"left_house={left_house}, right_cabinet={right_cabinet}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
