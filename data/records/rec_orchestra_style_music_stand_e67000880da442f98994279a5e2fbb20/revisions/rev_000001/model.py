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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="symphony_music_stand")

    powder_black = model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    panel_tilt = -math.radians(14.0)

    sleeve_shell = (
        cq.Workplane("XY")
        .circle(0.019)
        .extrude(0.505)
        .union(cq.Workplane("XY").circle(0.030).extrude(0.065).translate((0.0, 0.0, 0.4445)))
        .union(cq.Workplane("XY").circle(0.023).extrude(0.048).translate((0.0, 0.0, 0.5095)))
        .cut(cq.Workplane("XY").circle(0.0140).extrude(0.5575))
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.180, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=powder_black,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.132, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=graphite,
        name="base_cap",
    )
    base.visual(
        Cylinder(radius=0.042, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=powder_black,
        name="base_hub",
    )
    base.visual(
        mesh_from_cadquery(sleeve_shell, "mast_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.0735)),
        material=powder_black,
        name="mast_sleeve",
    )
    base.visual(
        Cylinder(radius=0.168, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_rubber,
        name="base_pad",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.014, length=0.920),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=graphite,
        name="inner_tube",
    )
    mast.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.592)),
        material=powder_black,
        name="head_cap",
    )
    mast.visual(
        Box((0.036, 0.036, 0.110)),
        origin=Origin(xyz=(0.0, 0.014, 0.548)),
        material=powder_black,
        name="head_neck",
    )
    mast.visual(
        Box((0.096, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, 0.022, 0.560)),
        material=powder_black,
        name="yoke_bridge",
    )
    mast.visual(
        Box((0.010, 0.040, 0.060)),
        origin=Origin(xyz=(-0.043, 0.008, 0.560)),
        material=powder_black,
        name="cheek_0",
    )
    mast.visual(
        Box((0.010, 0.040, 0.060)),
        origin=Origin(xyz=(0.043, 0.008, 0.560)),
        material=powder_black,
        name="cheek_1",
    )

    desk = model.part("desk")
    desk.visual(
        Box((0.076, 0.026, 0.026)),
        origin=Origin(xyz=(0.0, -0.028, 0.013)),
        material=powder_black,
        name="receiver_block",
    )
    desk.visual(
        Box((0.018, 0.016, 0.026)),
        origin=Origin(xyz=(-0.028, -0.008, 0.013)),
        material=powder_black,
        name="trunnion_lug_0",
    )
    desk.visual(
        Box((0.018, 0.016, 0.026)),
        origin=Origin(xyz=(0.028, -0.008, 0.013)),
        material=powder_black,
        name="trunnion_lug_1",
    )
    desk.visual(
        Cylinder(radius=0.0075, length=0.018),
        origin=Origin(xyz=(-0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="trunnion_0",
    )
    desk.visual(
        Cylinder(radius=0.0075, length=0.018),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="trunnion_1",
    )
    desk.visual(
        Box((0.070, 0.020, 0.230)),
        origin=Origin(xyz=(0.0, -0.030, 0.115)),
        material=powder_black,
        name="support_spine",
    )
    desk.visual(
        Box((0.540, 0.012, 0.360)),
        origin=Origin(xyz=(0.0, -0.026, 0.225), rpy=(panel_tilt, 0.0, 0.0)),
        material=graphite,
        name="panel",
    )
    desk.visual(
        Box((0.100, 0.022, 0.220)),
        origin=Origin(xyz=(0.0, -0.016, 0.190), rpy=(panel_tilt, 0.0, 0.0)),
        material=powder_black,
        name="center_reinforcement",
    )
    desk.visual(
        Box((0.016, 0.028, 0.360)),
        origin=Origin(xyz=(-0.262, -0.022, 0.225), rpy=(panel_tilt, 0.0, 0.0)),
        material=graphite,
        name="side_flange_0",
    )
    desk.visual(
        Box((0.016, 0.028, 0.360)),
        origin=Origin(xyz=(0.262, -0.022, 0.225), rpy=(panel_tilt, 0.0, 0.0)),
        material=graphite,
        name="side_flange_1",
    )
    desk.visual(
        Box((0.500, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, -0.012, 0.070), rpy=(panel_tilt, 0.0, 0.0)),
        material=graphite,
        name="lower_back_rail",
    )
    desk.visual(
        Box((0.500, 0.055, 0.012)),
        origin=Origin(xyz=(0.0, -0.044, 0.034), rpy=(0.08, 0.0, 0.0)),
        material=graphite,
        name="score_shelf",
    )
    desk.visual(
        Box((0.500, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, -0.068, 0.040)),
        material=graphite,
        name="score_lip",
    )

    collar_knob = model.part("collar_knob")
    collar_knob.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="stem",
    )
    collar_knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="hub",
    )
    collar_knob.visual(
        Cylinder(radius=0.005, length=0.044),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=powder_black,
        name="grip_bar",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.631)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=0.220,
        ),
    )
    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.20,
            lower=-0.250,
            upper=0.400,
        ),
    )
    model.articulation(
        "base_to_collar_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=collar_knob,
        origin=Origin(xyz=(0.030, 0.0, 0.5505)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    collar_knob = object_model.get_part("collar_knob")

    mast_slide = object_model.get_articulation("base_to_mast")
    desk_tilt = object_model.get_articulation("mast_to_desk")
    knob_spin = object_model.get_articulation("base_to_collar_knob")

    mast_limits = mast_slide.motion_limits
    desk_limits = desk_tilt.motion_limits

    ctx.allow_overlap(
        base,
        mast,
        elem_a="mast_sleeve",
        elem_b="inner_tube",
        reason="The inner mast is intentionally represented as sliding inside the hollow lower sleeve.",
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="mast_sleeve",
        margin=0.001,
        name="mast stays centered in the socket",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="mast_sleeve",
        min_overlap=0.20,
        name="collapsed mast remains deeply inserted",
    )
    ctx.expect_origin_gap(
        collar_knob,
        base,
        axis="x",
        min_gap=0.020,
        max_gap=0.040,
        name="clamp knob projects from the collar",
    )

    if mast_limits is not None and mast_limits.upper is not None:
        rest_mast_z = ctx.part_world_position(mast)
        with ctx.pose({mast_slide: mast_limits.upper}):
            ctx.expect_within(
                mast,
                base,
                axes="xy",
                inner_elem="inner_tube",
                outer_elem="mast_sleeve",
                margin=0.001,
                name="extended mast stays centered in the socket",
            )
            ctx.expect_overlap(
                mast,
                base,
                axes="z",
                elem_a="inner_tube",
                elem_b="mast_sleeve",
                min_overlap=0.045,
                name="extended mast retains insertion",
            )
            extended_mast_z = ctx.part_world_position(mast)

        ctx.check(
            "mast extends upward",
            rest_mast_z is not None
            and extended_mast_z is not None
            and extended_mast_z[2] > rest_mast_z[2] + 0.18,
            details=f"rest={rest_mast_z}, extended={extended_mast_z}",
        )

    if desk_limits is not None and desk_limits.lower is not None and desk_limits.upper is not None:
        with ctx.pose({desk_tilt: desk_limits.lower}):
            lower_panel_aabb = ctx.part_element_world_aabb(desk, elem="panel")
        with ctx.pose({desk_tilt: desk_limits.upper}):
            upper_panel_aabb = ctx.part_element_world_aabb(desk, elem="panel")

        ctx.check(
            "desk tilts backward at upper limit",
            lower_panel_aabb is not None
            and upper_panel_aabb is not None
            and upper_panel_aabb[1][1] > lower_panel_aabb[1][1] + 0.05,
            details=f"lower={lower_panel_aabb}, upper={upper_panel_aabb}",
        )

    with ctx.pose({knob_spin: math.pi / 2.0}):
        spun_bar_aabb = ctx.part_element_world_aabb(collar_knob, elem="grip_bar")
    rest_bar_aabb = ctx.part_element_world_aabb(collar_knob, elem="grip_bar")
    ctx.check(
        "knob grip bar visibly rotates about its threaded axis",
        rest_bar_aabb is not None
        and spun_bar_aabb is not None
        and abs((rest_bar_aabb[1][2] - rest_bar_aabb[0][2]) - (spun_bar_aabb[1][2] - spun_bar_aabb[0][2])) > 0.020,
        details=f"rest={rest_bar_aabb}, spun={spun_bar_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
