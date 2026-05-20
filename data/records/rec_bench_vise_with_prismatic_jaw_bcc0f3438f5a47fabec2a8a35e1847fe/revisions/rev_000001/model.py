from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_base_bench_vise")

    model.material("paint_blue", rgba=(0.20, 0.36, 0.60, 1.0))
    model.material("base_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("jaw_steel", rgba=(0.82, 0.84, 0.86, 1.0))

    swivel_base = model.part("swivel_base")
    swivel_base.visual(
        Box((0.260, 0.180, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="base_dark",
        name="foot",
    )
    swivel_base.visual(
        Box((0.160, 0.120, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material="base_dark",
        name="plinth",
    )
    swivel_base.visual(
        Cylinder(radius=0.062, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material="base_dark",
        name="pedestal",
    )
    swivel_base.visual(
        Cylinder(radius=0.098, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material="steel",
        name="ring",
    )
    for side, x_pos in (("left", -0.105), ("right", 0.105)):
        for index, y_pos in enumerate((-0.011, 0.011)):
            swivel_base.visual(
                Box((0.016, 0.004, 0.016)),
                origin=Origin(xyz=(x_pos, y_pos, 0.062)),
                material="steel",
                name=f"{side}_tab_lug_{index}",
            )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.086, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="paint_blue",
        name="turntable",
    )
    body.visual(
        Box((0.100, 0.060, 0.028)),
        origin=Origin(xyz=(0.0, -0.050, 0.027)),
        material="paint_blue",
        name="mount_web",
    )
    body.visual(
        Box((0.074, 0.160, 0.010)),
        origin=Origin(xyz=(0.0, 0.080, 0.009)),
        material="paint_blue",
        name="slide_bed",
    )
    for side, x_pos in (("left", -0.030), ("right", 0.030)):
        body.visual(
            Box((0.020, 0.160, 0.042)),
            origin=Origin(xyz=(x_pos, 0.080, 0.031)),
            material="paint_blue",
            name=f"{side}_guide",
        )
    body.visual(
        Box((0.082, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.154, 0.057)),
        material="paint_blue",
        name="guide_bridge",
    )
    for side, x_pos in (("left", -0.012), ("right", 0.012)):
        body.visual(
            Box((0.018, 0.044, 0.050)),
            origin=Origin(xyz=(x_pos, -0.006, 0.055)),
            material="paint_blue",
            name=f"{side}_post",
        )
    body.visual(
        Box((0.120, 0.034, 0.050)),
        origin=Origin(xyz=(0.0, 0.000, 0.095)),
        material="paint_blue",
        name="fixed_jaw_head",
    )
    body.visual(
        Box((0.098, 0.004, 0.038)),
        origin=Origin(xyz=(0.0, 0.016, 0.095)),
        material="jaw_steel",
        name="fixed_jaw_face",
    )
    body.visual(
        Box((0.070, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, -0.020, 0.120)),
        material="steel",
        name="anvil",
    )

    jaw = model.part("jaw")
    for side, x_pos in (("left", -0.030), ("right", 0.030)):
        jaw.visual(
            Box((0.012, 0.100, 0.030)),
            origin=Origin(xyz=(x_pos, 0.020, 0.0)),
            material="steel",
            name=f"{side}_runner",
        )
    for side, x_pos in (("left", -0.018), ("right", 0.018)):
        jaw.visual(
            Box((0.016, 0.024, 0.032)),
            origin=Origin(xyz=(x_pos, 0.004, 0.022)),
            material="paint_blue",
            name=f"{side}_stem",
        )
    jaw.visual(
        Box((0.108, 0.034, 0.048)),
        origin=Origin(xyz=(0.0, 0.018, 0.057)),
        material="paint_blue",
        name="jaw_head",
    )
    jaw.visual(
        Box((0.112, 0.040, 0.008)),
        origin=Origin(xyz=(0.0, 0.014, 0.085)),
        material="paint_blue",
        name="jaw_cap",
    )
    jaw.visual(
        Box((0.094, 0.004, 0.038)),
        origin=Origin(xyz=(0.0, -0.003, 0.057)),
        material="jaw_steel",
        name="jaw_face",
    )

    screw = model.part("screw")
    screw.visual(
        Cylinder(radius=0.007, length=0.116),
        origin=Origin(xyz=(0.0, -0.058, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="lead_screw",
    )
    screw.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="thrust_collar",
    )
    screw.visual(
        Cylinder(radius=0.0045, length=0.120),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel",
        name="handle_bar",
    )
    for x_pos, name in ((-0.066, "left_grip"), (0.066, "right_grip")):
        screw.visual(
            Cylinder(radius=0.0065, length=0.018),
            origin=Origin(xyz=(x_pos, 0.012, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="steel",
            name=name,
        )

    left_tab = model.part("left_tab")
    left_tab.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="pivot",
    )
    left_tab.visual(
        Box((0.040, 0.012, 0.010)),
        origin=Origin(xyz=(-0.020, 0.0, -0.006)),
        material="steel",
        name="lever",
    )
    left_tab.visual(
        Box((0.012, 0.020, 0.010)),
        origin=Origin(xyz=(-0.036, 0.0, -0.016)),
        material="steel",
        name="toe",
    )

    right_tab = model.part("right_tab")
    right_tab.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="pivot",
    )
    right_tab.visual(
        Box((0.040, 0.012, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, -0.006)),
        material="steel",
        name="lever",
    )
    right_tab.visual(
        Box((0.012, 0.020, 0.010)),
        origin=Origin(xyz=(0.036, 0.0, -0.016)),
        material="steel",
        name="toe",
    )

    model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=swivel_base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=1.2,
            lower=-math.radians(65.0),
            upper=math.radians(65.0),
        ),
    )
    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.0, 0.024, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=0.0,
            upper=0.085,
        ),
    )
    model.articulation(
        "screw_spin",
        ArticulationType.CONTINUOUS,
        parent=jaw,
        child=screw,
        origin=Origin(xyz=(0.0, 0.038, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=10.0,
        ),
    )
    model.articulation(
        "left_tab_swing",
        ArticulationType.REVOLUTE,
        parent=swivel_base,
        child=left_tab,
        origin=Origin(xyz=(-0.108, 0.0, 0.062)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=math.radians(-20.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "right_tab_swing",
        ArticulationType.REVOLUTE,
        parent=swivel_base,
        child=right_tab,
        origin=Origin(xyz=(0.108, 0.0, 0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=math.radians(-20.0),
            upper=math.radians(55.0),
        ),
        mimic=Mimic("left_tab_swing"),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    screw = object_model.get_part("screw")
    left_tab = object_model.get_part("left_tab")
    right_tab = object_model.get_part("right_tab")
    swivel = object_model.get_articulation("base_swivel")
    slide = object_model.get_articulation("jaw_slide")
    screw_spin = object_model.get_articulation("screw_spin")
    left_tab_swing = object_model.get_articulation("left_tab_swing")
    right_tab_swing = object_model.get_articulation("right_tab_swing")

    ctx.allow_overlap(
        body,
        jaw,
        elem_a="left_guide",
        elem_b="left_runner",
        reason="The sliding jaw is represented with solid guide proxies around the left runner instead of a hollow channel.",
    )
    ctx.allow_overlap(
        body,
        jaw,
        elem_a="right_guide",
        elem_b="right_runner",
        reason="The sliding jaw is represented with solid guide proxies around the right runner instead of a hollow channel.",
    )
    ctx.allow_overlap(
        body,
        jaw,
        elem_a="left_guide",
        elem_b="left_stem",
        reason="The moving jaw stem passes through a simplified solid left guide proxy instead of an open cast channel.",
    )
    ctx.allow_overlap(
        body,
        jaw,
        elem_a="right_guide",
        elem_b="right_stem",
        reason="The moving jaw stem passes through a simplified solid right guide proxy instead of an open cast channel.",
    )

    ctx.expect_gap(
        jaw,
        body,
        axis="y",
        positive_elem="jaw_face",
        negative_elem="fixed_jaw_face",
        min_gap=0.0008,
        max_gap=0.0025,
        name="closed jaw faces leave a light machining gap",
    )
    ctx.expect_within(
        jaw,
        body,
        axes="x",
        inner_elem="left_runner",
        outer_elem="left_guide",
        margin=0.0005,
        name="left runner stays inside the left guide laterally",
    )
    ctx.expect_overlap(
        jaw,
        body,
        axes="y",
        elem_a="left_runner",
        elem_b="left_guide",
        min_overlap=0.093,
        name="closed jaw keeps deep runner engagement",
    )

    rest_pos = ctx.part_world_position(jaw)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_overlap(
            jaw,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="left_guide",
            min_overlap=0.070,
            name="extended jaw still retains guide engagement",
        )
        extended_pos = ctx.part_world_position(jaw)
    ctx.check(
        "jaw opens forward along the guide",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] > rest_pos[1] + 0.05,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    jaw_rest = ctx.part_world_position(jaw)
    with ctx.pose({swivel: math.radians(45.0)}):
        jaw_swiveled = ctx.part_world_position(jaw)
    ctx.check(
        "upper body swivels around the base",
        jaw_rest is not None
        and jaw_swiveled is not None
        and math.hypot(jaw_swiveled[0] - jaw_rest[0], jaw_swiveled[1] - jaw_rest[1]) > 0.012,
        details=f"rest={jaw_rest}, swiveled={jaw_swiveled}",
    )
    handle_rest = ctx.part_element_world_aabb(screw, elem="handle_bar")
    with ctx.pose({screw_spin: math.pi / 2.0}):
        handle_quarter = ctx.part_element_world_aabb(screw, elem="handle_bar")
    ctx.check(
        "t handle rotates around the lead screw axis",
        handle_rest is not None
        and handle_quarter is not None
        and (handle_rest[1][0] - handle_rest[0][0]) > (handle_rest[1][2] - handle_rest[0][2])
        and (handle_quarter[1][2] - handle_quarter[0][2]) > (handle_quarter[1][0] - handle_quarter[0][0]),
        details=f"rest={handle_rest}, quarter={handle_quarter}",
    )
    left_toe_rest = ctx.part_element_world_aabb(left_tab, elem="toe")
    right_toe_rest = ctx.part_element_world_aabb(right_tab, elem="toe")
    with ctx.pose({left_tab_swing: math.radians(35.0)}):
        left_toe_lifted = ctx.part_element_world_aabb(left_tab, elem="toe")
        right_toe_lifted = ctx.part_element_world_aabb(right_tab, elem="toe")
    ctx.check(
        "locking tabs are authored as a matched pair",
        right_tab_swing.mimic is not None
        and right_tab_swing.mimic.joint == "left_tab_swing"
        and left_toe_rest is not None
        and right_toe_rest is not None
        and left_toe_lifted is not None
        and right_toe_lifted is not None
        and left_toe_lifted[1][2] > left_toe_rest[1][2] + 0.01
        and right_toe_lifted[1][2] > right_toe_rest[1][2] + 0.01,
        details=(
            f"mimic={right_tab_swing.mimic}, "
            f"left_rest={left_toe_rest}, left_lifted={left_toe_lifted}, "
            f"right_rest={right_toe_rest}, right_lifted={right_toe_lifted}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
