from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_LENGTH = 0.185
BASE_WIDTH = 0.132
BASE_PLATE_THICKNESS = 0.010
BASE_HEIGHT = 0.060
HINGE_X = -0.078
HINGE_Z = 0.068
LEVER_LENGTH = 0.228
DRAWER_CLOSED_X = 0.010
DRAWER_CLOSED_Y = 0.018
DRAWER_CLOSED_Z = 0.010
GUIDE_CLOSED_X = 0.090
GUIDE_Z = 0.034


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_hole_punch")

    model.material("body_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("lever_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("rubber_grip", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("zinc", rgba=(0.70, 0.72, 0.76, 1.0))
    model.material("tray_black", rgba=(0.11, 0.12, 0.14, 1.0))

    base = model.part("base")
    _add_base_visuals(base)

    lever = model.part("lever")
    _add_lever_visuals(lever)

    drawer = model.part("drawer")
    _add_drawer_visuals(drawer)

    guide = model.part("guide")
    _add_guide_visuals(guide)

    model.articulation(
        "lever_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=80.0, velocity=2.2),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(DRAWER_CLOSED_X, DRAWER_CLOSED_Y, DRAWER_CLOSED_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.040, effort=25.0, velocity=0.18),
    )
    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=guide,
        origin=Origin(xyz=(GUIDE_CLOSED_X, 0.0, GUIDE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.040, effort=20.0, velocity=0.16),
    )

    return model


def _add_base_visuals(base) -> None:
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS / 2.0)),
        material="body_steel",
        name="base_plate",
    )
    base.visual(
        Box((0.035, 0.120, BASE_HEIGHT)),
        origin=Origin(xyz=(-0.073, 0.0, BASE_HEIGHT / 2.0)),
        material="body_steel",
        name="rear_block",
    )
    base.visual(
        Box((0.128, 0.108, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, 0.048)),
        material="body_steel",
        name="top_deck",
    )
    base.visual(
        Box((0.118, 0.016, 0.032)),
        origin=Origin(xyz=(0.006, -0.046, 0.026)),
        material="body_steel",
        name="left_wall",
    )
    base.visual(
        Box((0.042, 0.016, 0.032)),
        origin=Origin(xyz=(-0.056, 0.046, 0.026)),
        material="body_steel",
        name="right_rear_wall",
    )
    base.visual(
        Box((0.024, 0.016, 0.024)),
        origin=Origin(xyz=(0.068, 0.046, 0.022)),
        material="body_steel",
        name="right_front_wall",
    )
    base.visual(
        Box((0.056, 0.118, 0.018)),
        origin=Origin(xyz=(0.060, 0.0, 0.019)),
        material="body_steel",
        name="front_nose",
    )
    base.visual(
        Box((0.074, 0.060, 0.010)),
        origin=Origin(xyz=(0.016, 0.0, 0.020)),
        material="body_steel",
        name="die_block",
    )
    for y_pos, name in ((-0.032, "rear_knuckle_0"), (0.032, "rear_knuckle_1")):
        base.visual(
            Cylinder(radius=0.010, length=0.026),
            origin=Origin(xyz=(HINGE_X, y_pos, HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material="body_steel",
            name=name,
        )
    for y_pos, name in ((-0.019, "die_pad_0"), (0.019, "die_pad_1")):
        base.visual(
            Cylinder(radius=0.0085, length=0.002),
            origin=Origin(xyz=(0.010, y_pos, 0.051)),
            material="zinc",
            name=name,
        )
    base.visual(
        Box((0.018, 0.052, 0.008)),
        origin=Origin(xyz=(-0.004, 0.0, 0.058)),
        material="zinc",
        name="lever_stop",
    )
    base.visual(
        Box((0.080, 0.014, 0.006)),
        origin=Origin(xyz=(0.060, 0.0, 0.031)),
        material="zinc",
        name="front_fence",
    )


def _add_lever_visuals(lever) -> None:
    lever.visual(
        Box((LEVER_LENGTH, 0.112, 0.016)),
        origin=Origin(xyz=(LEVER_LENGTH / 2.0 + 0.010, 0.0, 0.008)),
        material="lever_black",
        name="lever_body",
    )
    lever.visual(
        Box((0.030, 0.030, 0.020)),
        origin=Origin(xyz=(0.014, 0.0, 0.000)),
        material="lever_black",
        name="hinge_bridge",
    )
    lever.visual(
        Cylinder(radius=0.0085, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="zinc",
        name="hinge_tube",
    )
    lever.visual(
        Cylinder(radius=0.017, length=0.122),
        origin=Origin(xyz=(0.206, 0.0, 0.012), rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber_grip",
        name="front_grip",
    )
    lever.visual(
        Box((0.070, 0.050, 0.012)),
        origin=Origin(xyz=(0.058, 0.0, -0.002)),
        material="lever_black",
        name="punch_block",
    )
    for y_pos, name in ((-0.019, "punch_0"), (0.019, "punch_1")):
        lever.visual(
            Cylinder(radius=0.0068, length=0.010),
            origin=Origin(xyz=(0.058, y_pos, -0.010)),
            material="zinc",
            name=name,
        )
    lever.visual(
        Box((0.020, 0.052, 0.004)),
        origin=Origin(xyz=(0.074, 0.0, -0.004)),
        material="zinc",
        name="stop_pad",
    )


def _add_drawer_visuals(drawer) -> None:
    drawer.visual(
        Box((0.072, 0.034, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material="tray_black",
        name="drawer_floor",
    )
    drawer.visual(
        Box((0.072, 0.0025, 0.012)),
        origin=Origin(xyz=(0.0, -0.01575, 0.006)),
        material="tray_black",
        name="drawer_wall_inner",
    )
    drawer.visual(
        Box((0.072, 0.0025, 0.012)),
        origin=Origin(xyz=(0.0, 0.01575, 0.006)),
        material="tray_black",
        name="drawer_wall_outer",
    )
    drawer.visual(
        Box((0.0025, 0.029, 0.012)),
        origin=Origin(xyz=(-0.03475, 0.0, 0.006)),
        material="tray_black",
        name="drawer_wall_rear",
    )
    drawer.visual(
        Box((0.0025, 0.029, 0.012)),
        origin=Origin(xyz=(0.03475, 0.0, 0.006)),
        material="tray_black",
        name="drawer_wall_front",
    )
    drawer.visual(
        Box((0.036, 0.010, 0.010)),
        origin=Origin(xyz=(0.006, 0.021, 0.008)),
        material="rubber_grip",
        name="drawer_pull",
    )


def _add_guide_visuals(guide) -> None:
    guide.visual(
        Box((0.030, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="tray_black",
        name="guide_shoe",
    )
    guide.visual(
        Box((0.020, 0.060, 0.022)),
        origin=Origin(xyz=(-0.004, 0.0, 0.017)),
        material="tray_black",
        name="guide_plate",
    )
    guide.visual(
        Box((0.010, 0.022, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, 0.010)),
        material="rubber_grip",
        name="guide_tab",
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lever = object_model.get_part("lever")
    drawer = object_model.get_part("drawer")
    guide = object_model.get_part("guide")
    lever_hinge = object_model.get_articulation("lever_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    guide_slide = object_model.get_articulation("guide_slide")

    with ctx.pose({lever_hinge: 0.0}):
        ctx.expect_contact(
            lever,
            base,
            elem_a="stop_pad",
            elem_b="lever_stop",
            name="lever rests on the base stop in the closed pose",
        )
        ctx.expect_overlap(
            lever,
            base,
            axes="xy",
            min_overlap=0.080,
            name="closed lever stays centered over the punch body",
        )
        ctx.expect_contact(
            drawer,
            base,
            elem_a="drawer_floor",
            elem_b="base_plate",
            name="chip drawer rests on the base plate when closed",
        )
        ctx.expect_contact(
            guide,
            base,
            elem_a="guide_shoe",
            elem_b="front_fence",
            name="depth guide sits on the front fence in the closed pose",
        )
        ctx.expect_within(
            drawer,
            base,
            axes="xz",
            margin=0.003,
            name="closed chip drawer stays within the base cavity envelope",
        )
        ctx.expect_overlap(
            drawer,
            base,
            axes="y",
            min_overlap=0.020,
            name="closed chip drawer remains inserted into the base",
        )
        ctx.expect_within(
            guide,
            base,
            axes="y",
            inner_elem="guide_shoe",
            outer_elem="front_fence",
            margin=0.001,
            name="closed depth guide stays centered on the fence",
        )
        ctx.expect_overlap(
            guide,
            base,
            axes="x",
            elem_a="guide_shoe",
            elem_b="front_fence",
            min_overlap=0.020,
            name="closed depth guide remains captured along the fence",
        )

    closed_grip = ctx.part_element_world_aabb(lever, elem="front_grip")
    closed_drawer_pos = ctx.part_world_position(drawer)
    closed_guide_pos = ctx.part_world_position(guide)
    with ctx.pose({lever_hinge: 1.0}):
        open_grip = ctx.part_element_world_aabb(lever, elem="front_grip")
        ctx.expect_gap(
            lever,
            base,
            axis="z",
            min_gap=0.020,
            positive_elem="front_grip",
            negative_elem="top_deck",
            name="opened lever lifts clear of the base",
        )

    with ctx.pose({drawer_slide: 0.040}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_within(
            drawer,
            base,
            axes="xz",
            margin=0.003,
            name="extended chip drawer stays aligned with the base cavity",
        )
        ctx.expect_overlap(
            drawer,
            base,
            axes="y",
            min_overlap=0.020,
            name="extended chip drawer retains insertion in the base",
        )

    with ctx.pose({guide_slide: 0.040}):
        rear_guide_pos = ctx.part_world_position(guide)
        ctx.expect_within(
            guide,
            base,
            axes="y",
            inner_elem="guide_shoe",
            outer_elem="front_fence",
            margin=0.001,
            name="rearward depth guide stays centered on the fence",
        )
        ctx.expect_overlap(
            guide,
            base,
            axes="x",
            elem_a="guide_shoe",
            elem_b="front_fence",
            min_overlap=0.020,
            name="rearward depth guide remains captured on the fence",
        )

    ctx.check(
        "lever front grip rises when opened",
        closed_grip is not None
        and open_grip is not None
        and open_grip[0][2] > closed_grip[0][2] + 0.055,
        details=f"closed={closed_grip}, open={open_grip}",
    )
    ctx.check(
        "chip drawer slides outward from the side",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > closed_drawer_pos[1] + 0.030,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )
    ctx.check(
        "depth guide shifts rearward along the front fence",
        closed_guide_pos is not None
        and rear_guide_pos is not None
        and rear_guide_pos[0] < closed_guide_pos[0] - 0.030,
        details=f"closed={closed_guide_pos}, rearward={rear_guide_pos}",
    )

    return ctx.report()


object_model = build_object_model()
