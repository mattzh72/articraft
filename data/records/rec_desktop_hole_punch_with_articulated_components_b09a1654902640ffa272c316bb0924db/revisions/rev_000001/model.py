from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merged_box_mesh(
    name: str,
    boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]],
):
    geom = None
    for size, center in boxes:
        piece = BoxGeometry(size)
        piece.translate(*center)
        if geom is None:
            geom = piece
        else:
            geom.merge(piece)
    if geom is None:
        raise ValueError("Expected at least one box when building a merged mesh.")
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_punch")

    steel = model.material("steel", rgba=(0.20, 0.22, 0.24, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.10, 0.10, 0.11, 1.0))
    zinc = model.material("zinc", rgba=(0.67, 0.69, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    guide_plastic = model.material("guide_plastic", rgba=(0.83, 0.85, 0.86, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.34, 0.17, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="base_plate",
    )
    base.visual(
        Box((0.20, 0.080, 0.044)),
        origin=Origin(xyz=(0.0, -0.030, 0.034)),
        material=steel,
        name="rear_housing",
    )
    base.visual(
        Box((0.24, 0.065, 0.020)),
        origin=Origin(xyz=(0.0, 0.040, 0.022)),
        material=steel,
        name="front_bed",
    )
    base.visual(
        Box((0.052, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.015, 0.027)),
        material=black_oxide,
        name="die_block",
    )
    base.visual(
        Box((0.26, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.077, 0.018)),
        material=zinc,
        name="front_rail",
    )
    base.visual(
        Box((0.040, 0.020, 0.016)),
        origin=Origin(xyz=(-0.075, -0.072, 0.064)),
        material=steel,
        name="hinge_support_0",
    )
    base.visual(
        Box((0.040, 0.020, 0.016)),
        origin=Origin(xyz=(0.075, -0.072, 0.064)),
        material=steel,
        name="hinge_support_1",
    )
    base.visual(
        _merged_box_mesh(
            "drawer_sleeve_mesh",
            [
                ((0.060, 0.082, 0.004), (0.132, -0.004, 0.037)),
                ((0.060, 0.004, 0.025), (0.132, 0.035, 0.0235)),
                ((0.060, 0.004, 0.025), (0.132, -0.043, 0.0235)),
                ((0.008, 0.082, 0.025), (0.100, -0.004, 0.0235)),
            ],
        ),
        material=black_oxide,
        name="drawer_sleeve",
    )
    base.visual(
        Box((0.018, 0.030, 0.012)),
        origin=Origin(xyz=(0.108, -0.004, 0.022)),
        material=black_oxide,
        name="sleeve_mount",
    )
    for x_sign in (-1.0, 1.0):
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(
                xyz=(0.082 * x_sign, -0.072, 0.067),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black_oxide,
            name=f"hinge_cap_{0 if x_sign < 0 else 1}",
        )
    for x_pos in (-0.125, 0.125):
        base.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(x_pos, -0.062, 0.003), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"foot_{0 if x_pos < 0 else 1}",
        )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.008, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="hinge_tube",
    )
    lever.visual(
        Box((0.082, 0.220, 0.018)),
        origin=Origin(xyz=(0.0, 0.116, 0.010)),
        material=black_oxide,
        name="lever_arm",
    )
    lever.visual(
        Box((0.090, 0.090, 0.022)),
        origin=Origin(xyz=(0.0, 0.265, 0.014)),
        material=black_oxide,
        name="lever_grip",
    )
    lever.visual(
        Box((0.060, 0.042, 0.016)),
        origin=Origin(xyz=(0.0, 0.092, 0.000)),
        material=black_oxide,
        name="punch_head",
    )
    lever.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(-0.016, 0.092, -0.015)),
        material=zinc,
        name="punch_pin_0",
    )
    lever.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.016, 0.092, -0.015)),
        material=zinc,
        name="punch_pin_1",
    )

    drawer = model.part("drawer")
    drawer.visual(
        _merged_box_mesh(
            "drawer_tray_mesh",
            [
                ((0.056, 0.074, 0.002), (0.000, 0.000, -0.010)),
                ((0.056, 0.002, 0.018), (0.000, 0.036, -0.001)),
                ((0.056, 0.002, 0.018), (0.000, -0.036, -0.001)),
                ((0.002, 0.074, 0.018), (-0.027, 0.000, -0.001)),
                ((0.004, 0.074, 0.020), (0.028, 0.000, 0.000)),
                ((0.006, 0.030, 0.010), (0.033, 0.000, 0.001)),
            ],
        ),
        material=steel,
        name="drawer_tray",
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.030, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=guide_plastic,
        name="guide_shoe",
    )
    guide.visual(
        Box((0.004, 0.036, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=guide_plastic,
        name="guide_fence",
    )
    guide.visual(
        Box((0.016, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.010, 0.034)),
        material=guide_plastic,
        name="guide_tab",
    )

    knob = model.part("index_knob")
    knob.visual(
        Cylinder(radius=0.0035, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="index_shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.018,
                0.010,
                body_style="skirted",
                top_diameter=0.014,
                skirt=KnobSkirt(0.022, 0.003, flare=0.05),
                grip=KnobGrip(style="fluted", count=12, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "paper_guide_index_knob",
        ),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="index_cap",
    )

    model.articulation(
        "base_to_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(0.0, -0.072, 0.067)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )
    model.articulation(
        "base_to_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(0.132, -0.004, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=0.0,
            upper=0.032,
        ),
    )
    model.articulation(
        "base_to_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=guide,
        origin=Origin(xyz=(0.0, 0.077, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.15,
            lower=-0.095,
            upper=0.095,
        ),
    )
    model.articulation(
        "guide_to_index_knob",
        ArticulationType.CONTINUOUS,
        parent=guide,
        child=knob,
        origin=Origin(xyz=(0.0, 0.018, 0.022)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lever = object_model.get_part("lever")
    drawer = object_model.get_part("drawer")
    guide = object_model.get_part("guide")
    knob = object_model.get_part("index_knob")

    lever_hinge = object_model.get_articulation("base_to_lever")
    drawer_slide = object_model.get_articulation("base_to_drawer")
    guide_slide = object_model.get_articulation("base_to_guide")
    knob_spin = object_model.get_articulation("guide_to_index_knob")

    with ctx.pose({lever_hinge: 0.0}):
        ctx.expect_gap(
            lever,
            base,
            axis="z",
            positive_elem="punch_pin_0",
            negative_elem="die_block",
            min_gap=0.002,
            max_gap=0.012,
            name="closed lever seats just above the die block",
        )

    closed_grip_aabb = ctx.part_element_world_aabb(lever, elem="lever_grip")
    with ctx.pose({lever_hinge: math.radians(60.0)}):
        open_grip_aabb = ctx.part_element_world_aabb(lever, elem="lever_grip")
    ctx.check(
        "lever opens upward",
        closed_grip_aabb is not None
        and open_grip_aabb is not None
        and float(open_grip_aabb[1][2]) > float(closed_grip_aabb[1][2]) + 0.10,
        details=f"closed={closed_grip_aabb}, open={open_grip_aabb}",
    )

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_within(
            drawer,
            base,
            axes="yz",
            inner_elem="drawer_tray",
            outer_elem="drawer_sleeve",
            margin=0.002,
            name="drawer tray stays aligned inside the sleeve",
        )
        ctx.expect_overlap(
            drawer,
            base,
            axes="x",
            elem_a="drawer_tray",
            elem_b="drawer_sleeve",
            min_overlap=0.050,
            name="closed drawer remains largely inserted",
        )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.032}):
        ctx.expect_within(
            drawer,
            base,
            axes="yz",
            inner_elem="drawer_tray",
            outer_elem="drawer_sleeve",
            margin=0.002,
            name="extended drawer stays level in the sleeve",
        )
        ctx.expect_overlap(
            drawer,
            base,
            axes="x",
            elem_a="drawer_tray",
            elem_b="drawer_sleeve",
            min_overlap=0.020,
            name="extended drawer retains insertion",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer pulls out to the side",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and float(extended_drawer_pos[0]) > float(rest_drawer_pos[0]) + 0.025,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    with ctx.pose({guide_slide: 0.0}):
        ctx.expect_gap(
            guide,
            base,
            axis="z",
            positive_elem="guide_shoe",
            negative_elem="front_rail",
            min_gap=0.0,
            max_gap=0.002,
            name="guide shoe rides on the front rail",
        )

    left_guide_pos = None
    right_guide_pos = None
    with ctx.pose({guide_slide: -0.090}):
        ctx.expect_overlap(
            guide,
            base,
            axes="x",
            elem_a="guide_shoe",
            elem_b="front_rail",
            min_overlap=0.020,
            name="guide stays captured at the left end",
        )
        left_guide_pos = ctx.part_world_position(guide)
    with ctx.pose({guide_slide: 0.090}):
        ctx.expect_overlap(
            guide,
            base,
            axes="x",
            elem_a="guide_shoe",
            elem_b="front_rail",
            min_overlap=0.020,
            name="guide stays captured at the right end",
        )
        right_guide_pos = ctx.part_world_position(guide)
    ctx.check(
        "paper guide slides across the front edge",
        left_guide_pos is not None
        and right_guide_pos is not None
        and float(right_guide_pos[0]) > float(left_guide_pos[0]) + 0.16,
        details=f"left={left_guide_pos}, right={right_guide_pos}",
    )

    rest_knob_pos = ctx.part_world_position(knob)
    with ctx.pose({knob_spin: 1.7}):
        spun_knob_pos = ctx.part_world_position(knob)
        ctx.expect_contact(
            knob,
            guide,
            elem_a="index_shaft",
            elem_b="guide_fence",
            name="knob shaft stays mounted against the guide fence",
        )
    ctx.check(
        "index knob rotates in place",
        rest_knob_pos is not None
        and spun_knob_pos is not None
        and abs(float(spun_knob_pos[0]) - float(rest_knob_pos[0])) < 1e-6
        and abs(float(spun_knob_pos[1]) - float(rest_knob_pos[1])) < 1e-6
        and abs(float(spun_knob_pos[2]) - float(rest_knob_pos[2])) < 1e-6,
        details=f"rest={rest_knob_pos}, spun={spun_knob_pos}",
    )

    return ctx.report()


object_model = build_object_model()
