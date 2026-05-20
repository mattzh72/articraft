from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


CORNER_X = 0.25
CORNER_Y = 0.15
LONG_X = 1.55
LONG_Y = 0.15
RETURN_X = 0.25
RETURN_Y = 1.10
SLEEVE_TOP_Z = 0.70


def _desktop_slab() -> cq.Workplane:
    """One continuous L-shaped desktop slab, authored in top-frame coordinates."""
    pts = [
        (0.00 - CORNER_X, -0.45 - CORNER_Y),
        (1.80 - CORNER_X, -0.45 - CORNER_Y),
        (1.80 - CORNER_X, 0.45 - CORNER_Y),
        (0.45 - CORNER_X, 0.45 - CORNER_Y),
        (0.45 - CORNER_X, 1.35 - CORNER_Y),
        (-0.55 - CORNER_X, 1.35 - CORNER_Y),
        (-0.55 - CORNER_X, 0.00 - CORNER_Y),
        (0.00 - CORNER_X, 0.00 - CORNER_Y),
    ]
    return cq.Workplane("XY").polyline(pts).close().extrude(0.045)


def _add_outer_column(part, x: float, y: float, prefix: str, material: str) -> None:
    """Rectangular hollow sleeve from four steel walls with a clear center."""
    wall = 0.012
    outer = 0.100
    height = 0.660
    zc = 0.370
    off = outer / 2.0 - wall / 2.0
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(x - off, y, zc)),
        material=material,
        name=f"{prefix}_sleeve_x0",
    )
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(x + off, y, zc)),
        material=material,
        name=f"{prefix}_sleeve_x1",
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(x, y - off, zc)),
        material=material,
        name=f"{prefix}_sleeve_y0",
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(x, y + off, zc)),
        material=material,
        name=f"{prefix}_sleeve_y1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_column_l_desk")
    model.material("warm_oak", rgba=(0.72, 0.48, 0.25, 1.0))
    model.material("edge_banding", rgba=(0.42, 0.25, 0.12, 1.0))
    model.material("black_steel", rgba=(0.02, 0.023, 0.025, 1.0))
    model.material("brushed_steel", rgba=(0.45, 0.48, 0.50, 1.0))
    model.material("control_black", rgba=(0.005, 0.006, 0.007, 1.0))
    model.material("button_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("power_button", rgba=(0.07, 0.12, 0.18, 1.0))
    model.material("white_mark", rgba=(0.90, 0.92, 0.88, 1.0))

    base = model.part("base")
    for prefix, x, y in (
        ("corner", CORNER_X, CORNER_Y),
        ("long", LONG_X, LONG_Y),
        ("return", RETURN_X, RETURN_Y),
    ):
        _add_outer_column(base, x, y, prefix, "black_steel")

    # Floor feet and a low L-shaped stabilizer keep all three outer sleeves a
    # single supported base assembly.
    base.visual(
        Box((0.14, 0.58, 0.04)),
        origin=Origin(xyz=(CORNER_X, CORNER_Y, 0.02)),
        material="black_steel",
        name="corner_foot",
    )
    base.visual(
        Box((0.14, 0.58, 0.04)),
        origin=Origin(xyz=(LONG_X, LONG_Y, 0.02)),
        material="black_steel",
        name="long_foot",
    )
    base.visual(
        Box((0.58, 0.14, 0.04)),
        origin=Origin(xyz=(RETURN_X, RETURN_Y, 0.02)),
        material="black_steel",
        name="return_foot",
    )
    base.visual(
        Box((LONG_X - CORNER_X + 0.11, 0.055, 0.05)),
        origin=Origin(xyz=((LONG_X + CORNER_X) / 2.0, CORNER_Y, 0.085)),
        material="black_steel",
        name="long_base_tie",
    )
    base.visual(
        Box((0.055, RETURN_Y - CORNER_Y + 0.11, 0.05)),
        origin=Origin(xyz=(CORNER_X, (RETURN_Y + CORNER_Y) / 2.0, 0.085)),
        material="black_steel",
        name="return_base_tie",
    )

    stage_specs = (
        ("corner_stage", CORNER_X, CORNER_Y),
        ("long_stage", LONG_X, LONG_Y),
        ("return_stage", RETURN_X, RETURN_Y),
    )
    for part_name, _, _ in stage_specs:
        stage = model.part(part_name)
        stage.visual(
            Box((0.064, 0.064, 0.600)),
            origin=Origin(xyz=(0.0, 0.0, -0.260)),
            material="brushed_steel",
            name="stage_tube",
        )
        stage.visual(
            Box((0.086, 0.086, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.033)),
            material="black_steel",
            name="top_plate",
        )
        stage.visual(
            Box((0.006, 0.026, 0.050)),
            origin=Origin(xyz=(0.035, 0.0, -0.420)),
            material="black_steel",
            name="guide_x1",
        )
        stage.visual(
            Box((0.006, 0.026, 0.050)),
            origin=Origin(xyz=(-0.035, 0.0, -0.420)),
            material="black_steel",
            name="guide_x0",
        )
        stage.visual(
            Box((0.026, 0.006, 0.050)),
            origin=Origin(xyz=(0.0, 0.035, -0.360)),
            material="black_steel",
            name="guide_y1",
        )
        stage.visual(
            Box((0.026, 0.006, 0.050)),
            origin=Origin(xyz=(0.0, -0.035, -0.360)),
            material="black_steel",
            name="guide_y0",
        )

    lift_limits = MotionLimits(effort=900.0, velocity=0.08, lower=0.0, upper=0.45)
    model.articulation(
        "lift_corner",
        ArticulationType.PRISMATIC,
        parent=base,
        child="corner_stage",
        origin=Origin(xyz=(CORNER_X, CORNER_Y, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "lift_long",
        ArticulationType.PRISMATIC,
        parent=base,
        child="long_stage",
        origin=Origin(xyz=(LONG_X, LONG_Y, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic("lift_corner", multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "lift_return",
        ArticulationType.PRISMATIC,
        parent=base,
        child="return_stage",
        origin=Origin(xyz=(RETURN_X, RETURN_Y, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic("lift_corner", multiplier=1.0, offset=0.0),
    )

    top_frame = model.part("top_frame")
    top_frame.visual(
        mesh_from_cadquery(_desktop_slab(), "corner_desktop", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material="warm_oak",
        name="desktop_slab",
    )
    # A single rigid L-shaped steel frame is directly under the slab and ties
    # the three lifting stages to one moving corner workstation top.
    top_frame.visual(
        Box((LONG_X - CORNER_X + 0.18, 0.10, 0.052)),
        origin=Origin(xyz=((LONG_X - CORNER_X) / 2.0, 0.0, 0.066)),
        material="black_steel",
        name="long_frame_rail",
    )
    top_frame.visual(
        Box((0.10, RETURN_Y - CORNER_Y + 0.18, 0.052)),
        origin=Origin(xyz=(0.0, (RETURN_Y - CORNER_Y) / 2.0, 0.066)),
        material="black_steel",
        name="return_frame_rail",
    )
    top_frame.visual(
        Box((0.30, 0.30, 0.018)),
        origin=Origin(xyz=(0.055, 0.055, 0.098)),
        material="black_steel",
        name="corner_gusset",
    )
    top_frame.visual(
        Box((1.74, 0.030, 0.040)),
        origin=Origin(xyz=(0.65, -0.600, 0.112)),
        material="edge_banding",
        name="front_edge_band",
    )

    model.articulation(
        "stage_to_frame",
        ArticulationType.FIXED,
        parent="corner_stage",
        child=top_frame,
        origin=Origin(),
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((0.245, 0.085, 0.055)),
        origin=Origin(),
        material="control_black",
        name="pod_shell",
    )
    control_pod.visual(
        Box((0.285, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.026, 0.0215)),
        material="control_black",
        name="mounting_lip",
    )
    model.articulation(
        "frame_to_pod",
        ArticulationType.FIXED,
        parent=top_frame,
        child=control_pod,
        origin=Origin(xyz=(0.70, -0.635, 0.0625)),
    )

    button_limits = MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=0.006)
    preset_xs = (-0.040, 0.0, 0.040)
    for index, x in enumerate(preset_xs):
        button = model.part(f"preset_button_{index}")
        button.visual(
            Box((0.030, 0.009, 0.017)),
            origin=Origin(xyz=(0.0, -0.0045, 0.0)),
            material="button_gray",
            name="button_cap",
        )
        button.visual(
            Box((0.014, 0.001, 0.0025)),
            origin=Origin(xyz=(0.0, -0.0091, 0.0045)),
            material="white_mark",
            name="preset_mark",
        )
        model.articulation(
            f"preset_{index}_press",
            ArticulationType.PRISMATIC,
            parent=control_pod,
            child=button,
            origin=Origin(xyz=(x, -0.0425, 0.006)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=button_limits,
        )

    power = model.part("power_button")
    power.visual(
        Cylinder(radius=0.011, length=0.009),
        origin=Origin(xyz=(0.075, -0.0045, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="power_button",
        name="button_cap",
    )
    power.visual(
        Box((0.010, 0.001, 0.002)),
        origin=Origin(xyz=(0.075, -0.0091, 0.0115)),
        material="white_mark",
        name="power_mark",
    )
    model.articulation(
        "power_press",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=power,
        origin=Origin(xyz=(0.0, -0.0425, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=button_limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top_frame = object_model.get_part("top_frame")
    pod = object_model.get_part("control_pod")
    stages = [
        object_model.get_part("corner_stage"),
        object_model.get_part("long_stage"),
        object_model.get_part("return_stage"),
    ]
    lift = object_model.get_articulation("lift_corner")

    ctx.expect_contact(
        top_frame,
        stages[0],
        elem_a="long_frame_rail",
        elem_b="top_plate",
        name="corner stage bears on rigid frame",
    )
    ctx.expect_contact(
        top_frame,
        stages[1],
        elem_a="long_frame_rail",
        elem_b="top_plate",
        name="long stage bears on rigid frame",
    )
    ctx.expect_contact(
        top_frame,
        stages[2],
        elem_a="return_frame_rail",
        elem_b="top_plate",
        name="return stage bears on rigid frame",
    )
    for stage in stages:
        ctx.expect_overlap(
            stage,
            base,
            axes="z",
            min_overlap=0.35,
            elem_a="stage_tube",
            name=f"{stage.name} remains inserted at rest",
        )

    ctx.expect_gap(
        top_frame,
        pod,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="desktop_slab",
        negative_elem="pod_shell",
        name="control pod is tucked under front edge",
    )
    ctx.expect_overlap(
        pod,
        top_frame,
        axes="x",
        min_overlap=0.20,
        elem_a="pod_shell",
        elem_b="front_edge_band",
        name="controller sits on long front run",
    )

    rest_stage_z = [ctx.part_world_position(stage)[2] for stage in stages]
    rest_top_z = ctx.part_world_position(top_frame)[2]
    with ctx.pose({lift: 0.40}):
        raised_stage_z = [ctx.part_world_position(stage)[2] for stage in stages]
        raised_top_z = ctx.part_world_position(top_frame)[2]
        for stage in stages:
            ctx.expect_overlap(
                stage,
                base,
                axes="z",
                min_overlap=0.08,
                elem_a="stage_tube",
                name=f"{stage.name} retains sleeve insertion raised",
            )

    deltas = [raised - rest for raised, rest in zip(raised_stage_z, rest_stage_z)]
    ctx.check(
        "three lift stages move in sync",
        min(deltas) > 0.395 and max(deltas) - min(deltas) < 0.0001,
        details=f"stage travel deltas={deltas}",
    )
    ctx.check(
        "desktop follows lifting stages",
        raised_top_z > rest_top_z + 0.395,
        details=f"rest_top_z={rest_top_z}, raised_top_z={raised_top_z}",
    )

    for joint_name, part_name in (
        ("preset_0_press", "preset_button_0"),
        ("preset_1_press", "preset_button_1"),
        ("preset_2_press", "preset_button_2"),
        ("power_press", "power_button"),
    ):
        joint = object_model.get_articulation(joint_name)
        button = object_model.get_part(part_name)
        ctx.expect_contact(
            button,
            pod,
            elem_a="button_cap",
            elem_b="pod_shell",
            name=f"{part_name} rests on pod face",
        )
        rest_y = ctx.part_world_position(button)[1]
        with ctx.pose({joint: 0.005}):
            pressed_y = ctx.part_world_position(button)[1]
        ctx.check(
            f"{part_name} depresses inward",
            pressed_y > rest_y + 0.004,
            details=f"rest_y={rest_y}, pressed_y={pressed_y}",
        )

    return ctx.report()


object_model = build_object_model()
