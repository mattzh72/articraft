from __future__ import annotations

import math

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


def _add_tube_visual(
    part,
    *,
    outer: float,
    inner: float,
    z_min: float,
    z_max: float,
    radius: float,
    material,
    name: str,
    mesh_name: str,
) -> None:
    """Build a clearanced square hollow section from four connected wall bars."""
    del radius, mesh_name
    wall = (outer - inner) * 0.5
    length = z_max - z_min
    z_center = (z_min + z_max) * 0.5
    wall_offset = (inner + wall) * 0.5
    wall_specs = (
        (name, (wall, outer, length), (wall_offset, 0.0, z_center)),
        (f"{name}_wall_1", (wall, outer, length), (-wall_offset, 0.0, z_center)),
        (f"{name}_wall_2", (outer, wall, length), (0.0, wall_offset, z_center)),
        (f"{name}_wall_3", (outer, wall, length), (0.0, -wall_offset, z_center)),
    )
    for wall_name, size, xyz in wall_specs:
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=material,
            name=wall_name,
        )


def _add_clamp_knob(part, *, x: float, z: float, material, knob_material, name: str) -> None:
    part.visual(
        Cylinder(radius=0.007, length=0.060),
        origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=f"{name}_screw",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(x + 0.038, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_telescoping_mast")

    powder_black = model.material("powder_black", rgba=(0.08, 0.085, 0.09, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.30, 0.32, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.66, 0.68, 0.68, 1.0))
    light_steel = model.material("light_steel", rgba=(0.78, 0.80, 0.79, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.94, 0.72, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.03, 0.03, 0.035, 1.0))
    bolt_dark = model.material("blackened_bolts", rgba=(0.02, 0.022, 0.024, 1.0))
    service_blue = model.material("service_blue", rgba=(0.10, 0.23, 0.38, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.82, 0.58, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=charcoal,
        name="ground_plate",
    )
    for i, (x, y) in enumerate(
        ((-0.330, -0.210), (-0.330, 0.210), (0.330, -0.210), (0.330, 0.210))
    ):
        base.visual(
            Box((0.140, 0.110, 0.030)),
            origin=Origin(xyz=(x, y, 0.015)),
            material=rubber,
            name=f"foot_pad_{i}",
        )
        base.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(x, y, 0.089)),
            material=bolt_dark,
            name=f"anchor_bolt_{i}",
        )

    base.visual(
        Box((0.320, 0.320, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=powder_black,
        name="pedestal_block",
    )
    base.visual(
        Box((0.030, 0.340, 0.180)),
        origin=Origin(xyz=(0.150, 0.0, 0.165)),
        material=powder_black,
        name="side_gusset_0",
    )
    base.visual(
        Box((0.030, 0.340, 0.180)),
        origin=Origin(xyz=(-0.150, 0.0, 0.165)),
        material=powder_black,
        name="side_gusset_1",
    )
    base.visual(
        Box((0.340, 0.030, 0.180)),
        origin=Origin(xyz=(0.0, 0.150, 0.165)),
        material=powder_black,
        name="side_gusset_2",
    )
    base.visual(
        Box((0.340, 0.030, 0.180)),
        origin=Origin(xyz=(0.0, -0.150, 0.165)),
        material=powder_black,
        name="side_gusset_3",
    )
    _add_tube_visual(
        base,
        outer=0.240,
        inner=0.170,
        z_min=0.180,
        z_max=0.700,
        radius=0.026,
        material=graphite,
        name="outer_sleeve",
        mesh_name="outer_sleeve_mesh",
    )
    _add_tube_visual(
        base,
        outer=0.290,
        inner=0.170,
        z_min=0.645,
        z_max=0.700,
        radius=0.030,
        material=safety_yellow,
        name="base_mouth_collar",
        mesh_name="base_mouth_collar_mesh",
    )
    _add_clamp_knob(
        base,
        x=0.170,
        z=0.675,
        material=bolt_dark,
        knob_material=rubber,
        name="base_clamp_knob",
    )

    stage_1 = model.part("stage_1")
    _add_tube_visual(
        stage_1,
        outer=0.150,
        inner=0.110,
        z_min=-0.420,
        z_max=0.500,
        radius=0.018,
        material=graphite,
        name="section_body",
        mesh_name="stage_1_body_mesh",
    )
    _add_tube_visual(
        stage_1,
        outer=0.170,
        inner=0.150,
        z_min=-0.420,
        z_max=-0.375,
        radius=0.018,
        material=light_steel,
        name="lower_shoe",
        mesh_name="stage_1_lower_shoe_mesh",
    )
    _add_tube_visual(
        stage_1,
        outer=0.176,
        inner=0.110,
        z_min=0.445,
        z_max=0.500,
        radius=0.020,
        material=safety_yellow,
        name="top_collar",
        mesh_name="stage_1_top_collar_mesh",
    )
    _add_clamp_knob(
        stage_1,
        x=0.108,
        z=0.472,
        material=bolt_dark,
        knob_material=rubber,
        name="clamp_knob",
    )

    stage_2 = model.part("stage_2")
    _add_tube_visual(
        stage_2,
        outer=0.095,
        inner=0.067,
        z_min=-0.360,
        z_max=0.450,
        radius=0.013,
        material=aluminum,
        name="section_body",
        mesh_name="stage_2_body_mesh",
    )
    _add_tube_visual(
        stage_2,
        outer=0.110,
        inner=0.095,
        z_min=-0.360,
        z_max=-0.315,
        radius=0.012,
        material=light_steel,
        name="lower_shoe",
        mesh_name="stage_2_lower_shoe_mesh",
    )
    _add_tube_visual(
        stage_2,
        outer=0.120,
        inner=0.067,
        z_min=0.395,
        z_max=0.450,
        radius=0.014,
        material=safety_yellow,
        name="top_collar",
        mesh_name="stage_2_top_collar_mesh",
    )
    _add_clamp_knob(
        stage_2,
        x=0.080,
        z=0.422,
        material=bolt_dark,
        knob_material=rubber,
        name="clamp_knob",
    )

    stage_3 = model.part("stage_3")
    _add_tube_visual(
        stage_3,
        outer=0.058,
        inner=0.036,
        z_min=-0.300,
        z_max=0.360,
        radius=0.008,
        material=light_steel,
        name="section_body",
        mesh_name="stage_3_body_mesh",
    )
    _add_tube_visual(
        stage_3,
        outer=0.067,
        inner=0.058,
        z_min=-0.300,
        z_max=-0.255,
        radius=0.008,
        material=aluminum,
        name="lower_shoe",
        mesh_name="stage_3_lower_shoe_mesh",
    )
    _add_tube_visual(
        stage_3,
        outer=0.078,
        inner=0.036,
        z_min=0.315,
        z_max=0.360,
        radius=0.010,
        material=safety_yellow,
        name="top_collar",
        mesh_name="stage_3_top_collar_mesh",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.076, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=powder_black,
        name="bearing_disk",
    )
    turntable.visual(
        Cylinder(radius=0.049, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=light_steel,
        name="rotor_cap",
    )
    turntable.visual(
        Box((0.160, 0.095, 0.028)),
        origin=Origin(xyz=(0.035, 0.0, 0.066)),
        material=powder_black,
        name="top_plate",
    )
    turntable.visual(
        Box((0.120, 0.070, 0.070)),
        origin=Origin(xyz=(0.075, 0.0, 0.115)),
        material=service_blue,
        name="service_head",
    )
    turntable.visual(
        Cylinder(radius=0.012, length=0.105),
        origin=Origin(xyz=(0.050, 0.0, 0.101), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_dark,
        name="cross_pin",
    )

    model.articulation(
        "base_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.18, lower=0.0, upper=0.340),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.18, lower=0.0, upper=0.300),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(0.0, 0.0, 0.450)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.16, lower=0.0, upper=0.230),
    )
    model.articulation(
        "stage_3_to_turntable",
        ArticulationType.REVOLUTE,
        parent=stage_3,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.75, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")
    turntable = object_model.get_part("turntable")
    j1 = object_model.get_articulation("base_to_stage_1")
    j2 = object_model.get_articulation("stage_1_to_stage_2")
    j3 = object_model.get_articulation("stage_2_to_stage_3")
    yaw = object_model.get_articulation("stage_3_to_turntable")

    ctx.check(
        "mast uses three prismatic sections",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in (j1, j2, j3)),
        details="The nested mast stages should slide vertically.",
    )
    ctx.check(
        "turntable is revolute",
        yaw.articulation_type == ArticulationType.REVOLUTE,
        details="The mast head should yaw on a revolute bearing.",
    )

    ctx.expect_gap(
        turntable,
        stage_3,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="bearing_disk",
        name="turntable bearing sits on top section",
    )
    ctx.expect_overlap(
        turntable,
        base,
        axes="xy",
        elem_a="bearing_disk",
        elem_b="ground_plate",
        min_overlap=0.14,
        name="turntable is visibly smaller than base footprint",
    )

    with ctx.pose({j1: 0.340, j2: 0.300, j3: 0.230}):
        ctx.expect_overlap(
            stage_1,
            base,
            axes="z",
            min_overlap=0.035,
            name="first stage remains captured in base sleeve",
        )
        ctx.expect_within(
            stage_1,
            base,
            axes="xy",
            margin=0.0,
            name="first stage remains centered over the base sleeve",
        )
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="z",
            min_overlap=0.035,
            name="second stage remains captured in first stage",
        )
        ctx.expect_within(
            stage_2,
            stage_1,
            axes="xy",
            margin=0.0,
            name="second stage remains centered in first stage",
        )
        ctx.expect_overlap(
            stage_3,
            stage_2,
            axes="z",
            min_overlap=0.035,
            name="third stage remains captured in second stage",
        )
        ctx.expect_within(
            stage_3,
            stage_2,
            axes="xy",
            margin=0.0,
            name="third stage remains centered in second stage",
        )

    rest_top = ctx.part_world_position(turntable)
    with ctx.pose({j1: 0.340, j2: 0.300, j3: 0.230}):
        extended_top = ctx.part_world_position(turntable)
    ctx.check(
        "mast extends upward",
        rest_top is not None
        and extended_top is not None
        and extended_top[2] > rest_top[2] + 0.80,
        details=f"rest={rest_top}, extended={extended_top}",
    )

    return ctx.report()


object_model = build_object_model()
